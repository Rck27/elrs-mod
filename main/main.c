#include <stdio.h>
#include <string.h>
#include <math.h> // Required for pow() function
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_idf_version.h"

// --- Configuration ---

#define CRSF_UART_PORT UART_NUM_2
#define RX_ELRS_UART (GPIO_NUM_16)
#define TX_FC_UART (GPIO_NUM_4)
#define UART0_BAUDRATE 115200

#define RX_PIN_UART     (GPIO_NUM_16)
#define TX_PIN_UART_RX  (GPIO_NUM_17)
#define CRSF_BAUDRATE   115200

#define TX_UART_NUM     UART_NUM_1
#define TX_PIN_UART     (GPIO_NUM_4)
#define RX_PIN_UART_TX  (GPIO_NUM_5)
#define IBUS_BAUDRATE   115200

#define UART_BUF_SIZE           1024
#define REALTIME_TASK_PRIORITY  (configMAX_PRIORITIES - 1)
#define BLINK_GPIO              GPIO_NUM_2

// --- Protocol Range Definitions ---
// NOTE: I've corrected the CRSF_CHANNEL_MAX to its standard value
#define CRSF_CHANNEL_MIN 191
#define CRSF_CHANNEL_MAX 1792
#define IBUS_CHANNEL_MIN 1000
#define IBUS_CHANNEL_MAX 2000

// CRSF Packet Definitions
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_SYNC_BYTE_RC 0xC8
#define CRSF_FRAME_TYPE_RC_CHANNELS_PACKED 0x16

// i-Bus Frame Definitions
#define IBUS_FRAME_SIZE       32
#define IBUS_HEADER_1         0x20
#define IBUS_HEADER_2         0x40
#define IBUS_MAX_CHANNELS     14

// --- APF Equation Constants ---
#define APF_N_GAIN 50000.0f  // Repulsive force gain factor (tweak this)
#define APF_MAX_DISTANCE 3.0f // The 'boundary' or max sensor range
#define APF_X0_THRESHOLD 2.5f // Distance at which repulsion starts

// --- Global Handles & Type Definitions ---
static const char *TAG_RX = "CRSF_RX";
static const char *TAG_APF = "APF_PROCESS";
static const char *TAG_TX = "IBUS_TX";



#define LIDAR_UART_PORT UART_NUM_1
#define LIDAR_UART_BAUDRATE 158700
#define LIDAR_UART_RX_PORT (GPIO_NUM_19)
#define LIDAR_UART_TX_PORT (GPIO_NUM_5)

// LIDAR Protocol Constants
#define SYNC_BYTE_0          0xAA
#define SYNC_BYTE_1          0x55
#define FULL_ROTATION_TICKS  0xB400 // 46080 in decimal

// --- Global Variable Definitions ---
volatile float g_left_distance_mm = 0.0f;
volatile float g_right_distance_mm = 0.0f;

typedef enum {
    STATE_SYNC0,
    STATE_SYNC1,
    STATE_HEADER,
    STATE_DATA
} lidar_state_t;



typedef uint16_t rc_channels_t[16];

// NEW: We now have two queues for the three-stage pipeline
static QueueHandle_t raw_crsf_to_apf_queue;
static QueueHandle_t modified_ibus_to_tx_queue;

static QueueHandle_t uart_rx_queue_handle;

// --- Helper Functions ---
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a; for (int i = 0; i < 8; ++i) { if (crc & 0x80) crc = (crc << 1) ^ 0xD5; else crc = crc << 1; } return crc;
}

static inline int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- CRSF Receiver Task ---
// Stage 1: Receives CRSF packets and passes the raw channel data to the processing task.
static void crsf_rx_task() {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);
    rc_channels_t local_channels;
    static uint8_t packet[CRSF_MAX_PACKET_LEN];
    static uint8_t packet_index = 0;

    ESP_LOGI(TAG_RX, "CRSF RX task started");
    while(1) {
        if(xQueueReceive(uart_rx_queue_handle, (void * )&event, (TickType_t)portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                size_t buffered_size;
                uart_get_buffered_data_len(CRSF_UART_PORT, &buffered_size);
                const int rxBytes = uart_read_bytes(CRSF_UART_PORT, dtmp, buffered_size, 0);
                for (int i = 0; i < rxBytes; i++) {
                    uint8_t b = dtmp[i];
                    if (packet_index == 0) { if (b == CRSF_SYNC_BYTE_RC) packet[packet_index++] = b; }
                    else {
                        if (packet_index < CRSF_MAX_PACKET_LEN) { packet[packet_index++] = b; }
                        if (packet_index >= 2) {
                            uint8_t packet_len = packet[1];
                            if (packet_len < 2 || packet_len > CRSF_MAX_PACKET_LEN - 2) { packet_index = 0; continue; }
                            if (packet_index == packet_len + 2) {
                                uint8_t crc = 0; for (int j = 2; j < packet_len + 1; j++) { crc = crc8_dvb_s2(crc, packet[j]); }
                                if (crc == packet[packet_len + 1] && packet[2] == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) {
                                    // Decode all 16 channels
                                    local_channels[0]  = ((packet[3]  | packet[4] << 8) & 0x07FF);
                                    local_channels[1]  = ((packet[4]  >> 3 | packet[5] << 5) & 0x07FF);
                                    local_channels[2]  = ((packet[5]  >> 6 | packet[6] << 2 | packet[7] << 10) & 0x07FF);
                                    local_channels[3]  = ((packet[7]  >> 1 | packet[8] << 7) & 0x07FF);
                                    local_channels[4]  = ((packet[8]  >> 4 | packet[9] << 4) & 0x07FF);
                                    local_channels[5]  = ((packet[9]  >> 7 | packet[10] << 1 | packet[11] << 9) & 0x07FF);
                                    local_channels[6]  = ((packet[11] >> 2 | packet[12] << 6) & 0x07FF);
                                    local_channels[7]  = ((packet[12] >> 5 | packet[13] << 3) & 0x07FF);
                                    local_channels[8]  = ((packet[14] | packet[15] << 8) & 0x07FF);
                                    local_channels[9]  = ((packet[15] >> 3 | packet[16] << 5) & 0x07FF);
                                    local_channels[10] = ((packet[16] >> 6 | packet[17] << 2 | packet[18] << 10) & 0x07FF);
                                    local_channels[11] = ((packet[18] >> 1 | packet[19] << 7) & 0x07FF);
                                    local_channels[12] = ((packet[19] >> 4 | packet[20] << 4) & 0x07FF);
                                    local_channels[13] = ((packet[20] >> 7 | packet[21] << 1 | packet[22] << 9) & 0x07FF);
                                    local_channels[14] = ((packet[22] >> 2 | packet[23] << 6) & 0x07FF);
                                    local_channels[15] = ((packet[23] >> 5 | packet[24] << 3) & 0x07FF);

                                    xQueueOverwrite(raw_crsf_to_apf_queue, &local_channels);
                                }
                                packet_index = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    free(dtmp);
}




static void apf_processing_task() {
    rc_channels_t crsf_channels;  // Buffer for incoming raw data
    rc_channels_t ibus_channels;  // Buffer for outgoing processed data
    float sensor_values[2];       // Dummy sensor values

    ESP_LOGI(TAG_APF, "APF Processing task started");

    while (1) {
        // Wait for new raw channel data from the CRSF task
        if (xQueueReceive(raw_crsf_to_apf_queue, &crsf_channels, portMAX_DELAY) == pdTRUE) {
            
            // --- Step 1: Map all channels from CRSF range to i-Bus range ---
            for (int i = 0; i < 16; i++) {
                ibus_channels[i] = map_value(crsf_channels[i], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, IBUS_CHANNEL_MIN, IBUS_CHANNEL_MAX);
            }
            sensor_values[0] = 0;
            sensor_values[1] = 0;
            // sensor_values[0] = vl53l0x_read_distance(); // Dummy Left Sensor: 1.0m away from left wall
            // sensor_values[1] = 3 - s ensor_values[0]; // Dummy Right Sensor: 2.5m away (0.5m from right wall)
            
            // --- Step 3: Apply APF Logic to the Roll channel (CH1) ---
            float force_left = 0.0f;
            float force_right = 0.0f;

            // Calculate repulsive force from the left wall (object at x=0)
            float x_left = sensor_values[0];
            if (x_left < APF_X0_THRESHOLD && x_left > 0) {
                float inv_sq_dist = 1.0f / (x_left * x_left);
                float inv_sq_thresh = 1.0f / (APF_X0_THRESHOLD * APF_X0_THRESHOLD);
                force_left = APF_N_GAIN * (inv_sq_dist - inv_sq_thresh) * inv_sq_dist;
            }

            // Calculate repulsive force from the right wall (object at x=3)
            float dist_from_right = APF_MAX_DISTANCE - sensor_values[1];
            if (dist_from_right < APF_X0_THRESHOLD && dist_from_right > 0) {
                float inv_sq_dist = 1.0f / (dist_from_right * dist_from_right);
                float inv_sq_thresh = 1.0f / (APF_X0_THRESHOLD * APF_X0_THRESHOLD);
                force_right = APF_N_GAIN * (inv_sq_dist - inv_sq_thresh) * inv_sq_dist;
            }

            int16_t roll_offset = (int16_t)(force_left - force_right);
            
            ibus_channels[0] += roll_offset; // Apply the offset

            // --- Step 5: Clamp the final values to ensure they are valid ---
            // This is a critical safety step!
            for (int i = 0; i < 4; i++) { // Only need to clamp the ones we might modify
                if (ibus_channels[i] < IBUS_CHANNEL_MIN) {
                    ibus_channels[i] = IBUS_CHANNEL_MIN;
                } else if (ibus_channels[i] > IBUS_CHANNEL_MAX) {
                    ibus_channels[i] = IBUS_CHANNEL_MAX;
                }
            }
            ESP_LOGI("apf", "left force %.2f right force %.2f", force_left, force_right);
            // --- Step 6: Send the final, modified channel data to the TX task ---
            xQueueOverwrite(modified_ibus_to_tx_queue, &ibus_channels);
        }
    }
}

// --- i-Bus Encoder Function ---
void encode_ibus_packet(uint8_t *ibus_frame, const rc_channels_t channels) {
    ibus_frame[0] = IBUS_HEADER_1; ibus_frame[1] = IBUS_HEADER_2;
    for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
        uint16_t channel_val = (i < 16) ? channels[i] : 1500;
        ibus_frame[2 + (i * 2)] = channel_val & 0xFF;
        ibus_frame[3 + (i * 2)] = (channel_val >> 8) & 0xFF;
    }
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_SIZE - 2; i++) { checksum -= ibus_frame[i]; }
    ibus_frame[30] = checksum & 0xFF;
    ibus_frame[31] = (checksum >> 8) & 0xFF;
}

// --- i-Bus TX Task ---
// Stage 3: Waits for processed channel data, encodes it, and sends it to the FC.
static void ibus_tx_task() {
    uint8_t ibus_packet[IBUS_FRAME_SIZE];
    rc_channels_t ibus_channels;

    ESP_LOGI(TAG_TX, "i-Bus TX task started");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        if (xQueueReceive(modified_ibus_to_tx_queue, &ibus_channels, portMAX_DELAY) == pdTRUE) {
            gpio_set_level(BLINK_GPIO, 1);
            encode_ibus_packet(ibus_packet, ibus_channels);
            uart_write_bytes(CRSF_UART_PORT, (const char*)ibus_packet, IBUS_FRAME_SIZE);
            gpio_set_level(BLINK_GPIO, 0);
        }
    }
}



/**
 * @brief The main FreeRTOS task for parsing LIDAR data.
 *
 * This task runs in an infinite loop, implementing a state machine to read
 * and parse data packets from the LIDAR via UART.
 */
static void lidar_parser_task(void *pvParameters)
{
    // Buffers for reading data
    uint8_t read_byte;
    uint8_t header_buf[8];
    // Max package size is 255, each point is 3 bytes (255*3=765). 1024 is safe.
    uint8_t data_buf[1024];

    lidar_state_t state = STATE_SYNC0;

    ESP_LOGI("LIDAR", "LIDAR parser task started.");

    while (1) {
        switch (state) {
            case STATE_SYNC0:
                ESP_LOGI("L", "sync0");
                // Read 1 byte and wait for the first sync byte
                int bytes_len = uart_read_bytes(LIDAR_UART_PORT, &read_byte, 1, portMAX_DELAY);
                ESP_LOGI("L", "sync0 %X", read_byte);
                if(bytes_len > 0){
                if (read_byte == SYNC_BYTE_0) {
                    state = STATE_SYNC1;
                }
            }
                // break;
            case STATE_SYNC1:
            ESP_LOGI("L", "sync1");
                // Read 1 byte and wait for the second sync byte
                uart_read_bytes(LIDAR_UART_PORT, &read_byte, 1, 0);
                if (read_byte == SYNC_BYTE_1) {
                    state = STATE_HEADER;
                } else {
                    state = STATE_SYNC0; // Reset if sequence is broken
                }
                break;

            case STATE_HEADER:
            ESP_LOGI("L", "header");
                // Read the 8-byte header
                if (uart_read_bytes(LIDAR_UART_PORT, header_buf, 8, 0) == 8) {
                    state = STATE_DATA;
                } else {
                    // If we fail to read 8 bytes, something is wrong, reset.
                    ESP_LOGW("LIDAR", "Failed to read full header, resetting state.");
                    state = STATE_SYNC0;
                }
                break;

            case STATE_DATA: {
                ESP_LOGI("L", "data");
                // Parse header to find data size
                uint8_t package_type = header_buf[0];
                uint8_t package_size = header_buf[1];
                uint16_t start_angle_raw = (header_buf[3] << 8) | header_buf[2];
                uint16_t stop_angle_raw = (header_buf[5] << 8) | header_buf[4];

                // Validate package and read data
                if (package_size > 0 && !(package_type & 0x01)) {
                    int bytes_to_read = package_size * 3;
                    if (uart_read_bytes(LIDAR_UART_PORT, data_buf, bytes_to_read, 0) == bytes_to_read) {
                        
                        // --- Angle and Step Calculation ---
                        int diff = stop_angle_raw - start_angle_raw;
                        if (diff < 0) {
                            diff += FULL_ROTATION_TICKS;
                        }
                        float step = 0.0f;
                        if (diff > 1 && package_size > 1) {
                            step = (float)diff / (package_size - 1);
                        }

                        // --- Averaging Logic ---
                        float left_dist_sum = 0;
                        int left_dist_count = 0;
                        float right_dist_sum = 0;
                        int right_dist_count = 0;

                        for (int i = 0; i < package_size; i++) {
                            uint16_t raw_dist = (data_buf[i * 3 + 2] << 8) | data_buf[i * 3 + 1];
                            float distance_mm = (float)raw_dist / 4.0f;

                            float raw_angle = start_angle_raw + step * i;
                            float angle_deg = (raw_angle / FULL_ROTATION_TICKS) * 360.0f;
                            
                            // Check if angle is in the right-side range (90 +/- 5 degrees)
                            if (angle_deg >= 85.0f && angle_deg <= 95.0f) {
                                right_dist_sum += distance_mm;
                                right_dist_count++;
                            }
                            // Check if angle is in the left-side range (270 +/- 5 degrees)
                            else if (angle_deg >= 265.0f && angle_deg <= 275.0f) {
                                left_dist_sum += distance_mm;
                                left_dist_count++;
                            }
                        }

                        // --- Update Global Variables (if we have new data) ---
                        // Dummy correction angles (you will get these from an IMU)
                        float roll_angle_deg = 0.0f; // DUMMY
                        float yaw_angle_deg = 0.0f;  // DUMMY

                        // Convert to radians for math functions
                        float roll_rad = roll_angle_deg * (M_PI / 180.0);
                        // float yaw_rad = yaw_angle_deg * (M_PI / 180.0); // Yaw correction not shown, but similar

                        if (right_dist_count > 0) {
                            float avg_right = right_dist_sum / right_dist_count;
                            // Apply roll correction
                            g_right_distance_mm = avg_right * cos(roll_rad);
                        }

                        if (left_dist_count > 0) {
                            float avg_left = left_dist_sum / left_dist_count;
                            // Apply roll correction
                            g_left_distance_mm = avg_left * cos(roll_rad);
                        }

                        ESP_LOGI("LIDAR", "left %.2f right %.2f", g_left_distance_mm, g_right_distance_mm);
                    } else {
                        ESP_LOGW("LIDAR", "Failed to read full data payload.");
                    }
                }
                
                // Always reset state to look for the next packet
                state = STATE_SYNC0;
                break;
            } // end case STATE_DATA
        } // end switch
    } // end while
}

void lidar_init_and_start_task(void)
{
    // --- Configure UART ---
    uart_config_t uart_config = {
        .baud_rate = LIDAR_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    (uart_driver_install(LIDAR_UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    (uart_param_config(LIDAR_UART_PORT, &uart_config));
    (uart_set_pin(LIDAR_UART_PORT, LIDAR_UART_RX_PORT, LIDAR_UART_TX_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // --- Create the parsing task ---
    xTaskCreate(
        lidar_parser_task,          // Task function
        "lidar_parser_task",        // Name of the task
        4096,                       // Stack size in words
        NULL,                       // Task input parameter
        5,                          // Priority of the task
        NULL                        // Task handle
    );

    ESP_LOGI("LIDAR", "LIDAR UART initialized and task created.");
}


// --- Main Application ---
void app_main(void) {

    vTaskDelay(pdMS_TO_TICKS(50));

    // Create the two queues for the new 3-stage pipeline
    raw_crsf_to_apf_queue = xQueueCreate(1, sizeof(rc_channels_t));
    modified_ibus_to_tx_queue = xQueueCreate(1, sizeof(rc_channels_t));

    // Configure RX UART (CRSF)
    const uart_config_t rx_uart_config = { .baud_rate = UART0_BAUDRATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(CRSF_UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 20, &uart_rx_queue_handle, 0);
    uart_param_config(CRSF_UART_PORT, &rx_uart_config);
    uart_set_pin(CRSF_UART_PORT, TX_FC_UART, RX_ELRS_UART, -1, -1);

    lidar_init_and_start_task();

    // Port UART IBUS coba dijadiin satu sama RX dari elrs
    // // Configure TX UART (i-Bus)
    // const uart_config_t tx_uart_config = { .baud_rate = IBUS_BAUDRATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    // uart_driver_install(TX_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
    // uart_param_config(TX_UART_NUM, &tx_uart_config);
    // uart_set_pin(TX_UART_NUM, TX_PIN_UART, RX_PIN_UART_TX, -1, -1);
    
    // Create and Pin Tasks
    // xTaskCreatePinnedToCore(crsf_rx_task, "crsf_rx_task", 4096, NULL, REALTIME_TASK_PRIORITY, NULL, 0); // Core 0 for I/O
    // xTaskCreatePinnedToCore(apf_processing_task, "apf_task", 4096, NULL, REALTIME_TASK_PRIORITY, NULL, 1); // Core 1 for processing
    // xTaskCreatePinnedToCore(ibus_tx_task, "ibus_tx_task", 4096, NULL, REALTIME_TASK_PRIORITY, NULL, 1); // Core 1 for I/O
}