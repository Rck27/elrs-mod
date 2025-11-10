#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_idf_version.h"

// --- Configuration ---
// RX UART for incoming CRSF data from ELRS receiver
#define RX_UART_NUM     UART_NUM_2
#define RX_PIN_UART     (GPIO_NUM_16)
#define TX_PIN_UART_RX  (GPIO_NUM_17) // Not used by receiver, but required by driver
#define CRSF_BAUDRATE   420000

// TX UART for outgoing i-Bus data to the Flight Controller
#define TX_UART_NUM     UART_NUM_1
#define TX_PIN_UART     (GPIO_NUM_4)
#define RX_PIN_UART_TX  (GPIO_NUM_5)  // Not used by flight controller, but required by driver
#define IBUS_BAUDRATE   115200        // Standard i-Bus baud rate

#define UART_BUF_SIZE           1024
#define REALTIME_TASK_PRIORITY  (configMAX_PRIORITIES - 1)
#define BLINK_GPIO              GPIO_NUM_2

// --- Protocol Range Definitions ---
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
#define IBUS_COMMAND_CHANNELS 0x40
#define IBUS_HEADER_1         0x20
#define IBUS_HEADER_2         0x40
#define IBUS_MAX_CHANNELS     14

// --- Global Handles & Type Definitions ---
static const char *TAG_PIPELINE = "CRSF_PIPELINE";
static const char *TAG_TX = "IBUS_ENCODER";

typedef uint16_t rc_channels_t[16];

static QueueHandle_t processed_to_tx_queue;
static QueueHandle_t uart_rx_queue_handle;

// --- Helper Functions ---
// CRC function for validating incoming CRSF packets
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// Maps a value from one range to another
static inline int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- CRSF Receiver Task ---
// This task reads UART data, parses CRSF packets, and sends channel data to the next stage.
static void crsf_rx_task() {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);
    rc_channels_t local_channels;
    static uint8_t packet[CRSF_MAX_PACKET_LEN];
    static uint8_t packet_index = 0;

    ESP_LOGI(TAG_PIPELINE, "CRSF RX task started");

    while(1) {
        if(xQueueReceive(uart_rx_queue_handle, (void * )&event, (TickType_t)portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                size_t buffered_size;
                uart_get_buffered_data_len(RX_UART_NUM, &buffered_size);
                const int rxBytes = uart_read_bytes(RX_UART_NUM, dtmp, buffered_size, 0);

                for (int i = 0; i < rxBytes; i++) {
                    uint8_t b = dtmp[i];
                    // Find CRSF sync byte to start a new packet
                    if (packet_index == 0) {
                        if (b == CRSF_SYNC_BYTE_RC) {
                           packet[packet_index++] = b;
                        }
                    } else {
                        // Fill the packet buffer
                        if (packet_index < CRSF_MAX_PACKET_LEN) {
                            packet[packet_index++] = b;
                        }

                        // Once we have the header (2 bytes), we know the expected length
                        if (packet_index >= 2) {
                            uint8_t packet_len = packet[1];

                            // Validate packet length
                            if (packet_len < 2 || packet_len > CRSF_MAX_PACKET_LEN - 2) {
                                packet_index = 0; // Invalid length, reset
                                continue;
                            }
                            
                            // Check if the full packet has been received
                            if (packet_index == packet_len + 2) {
                                uint8_t crc = 0;
                                // Calculate checksum (from byte 2 up to the CRC byte)
                                for (int j = 2; j < packet_len + 1; j++) {
                                    crc = crc8_dvb_s2(crc, packet[j]);
                                }

                                // If checksum is valid and it's a channels packet, decode it
                                if (crc == packet[packet_len + 1] && packet[2] == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) {
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

                                    // Send the decoded channels to the TX task
                                    xQueueOverwrite(processed_to_tx_queue, &local_channels);
                                }
                                packet_index = 0; // Reset for the next packet
                            }
                        }
                    }
                }
            }
        }
    }
    free(dtmp);
}

// --- i-Bus Encoder Function ---
// Constructs a 32-byte i-Bus packet from the channel data.
void encode_ibus_packet(uint8_t *ibus_frame, const rc_channels_t channels) {
    // 1. Set Header
    ibus_frame[0] = IBUS_HEADER_1;
    ibus_frame[1] = IBUS_HEADER_2;

    
    // 2. Pack up to 14 channels into the frame
    for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
        uint16_t channel_val = (i < 16) ? channels[i] : IBUS_CHANNEL_MIN + (IBUS_CHANNEL_MAX - IBUS_CHANNEL_MIN) / 2; // Default to 1500 if channel > 15
        ibus_frame[2 + (i * 2)] = channel_val & 0xFF;        // LSB
        ibus_frame[3 + (i * 2)] = (channel_val >> 8) & 0xFF; // MSB
    }

    // 3. Calculate and pack the checksum
    // Checksum is 0xFFFF minus the sum of the first 30 bytes
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_SIZE - 2; i++) {
        checksum -= ibus_frame[i];
    }
    ibus_frame[30] = checksum & 0xFF;        // LSB
    ibus_frame[31] = (checksum >> 8) & 0xFF; // MSB
}

// --- i-Bus TX Task ---
// This task waits for channel data, maps it, encodes it, and sends it over UART.
static void ibus_tx_task() {
    uint8_t ibus_packet[IBUS_FRAME_SIZE];
    rc_channels_t crsf_channels;
    rc_channels_t ibus_channels; // Holds the correctly mapped data for i-Bus

    ESP_LOGI(TAG_TX, "i-Bus TX task started");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Wait for new channel data from the RX task
        if (xQueueReceive(processed_to_tx_queue, &crsf_channels, portMAX_DELAY) == pdTRUE) {
            gpio_set_level(BLINK_GPIO, 1); // Turn LED ON to indicate packet processing

            // CRITICAL STEP: Map CRSF values (191-1792) to i-Bus values (1000-2000)
            for (int i = 0; i < 16; i++) {
                ibus_channels[i] = map_value(crsf_channels[i], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, IBUS_CHANNEL_MIN, IBUS_CHANNEL_MAX);
            }

            // Create the i-Bus packet
            encode_ibus_packet(ibus_packet, ibus_channels);

            // Write the packet to the flight controller
            uart_write_bytes(TX_UART_NUM, (const char*)ibus_packet, IBUS_FRAME_SIZE);
            
            gpio_set_level(BLINK_GPIO, 0); // Turn LED OFF
        }
    }
}

// --- Main Application ---
void app_main(void) {
    // Create the queue that passes data from the CRSF task to the i-Bus task
    processed_to_tx_queue = xQueueCreate(1, sizeof(rc_channels_t));

    // --- Configure RX UART (CRSF) ---
    const uart_config_t rx_uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(RX_UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 20, &uart_rx_queue_handle, 0);
    uart_param_config(RX_UART_NUM, &rx_uart_config);
    uart_set_pin(RX_UART_NUM, TX_PIN_UART_RX, RX_PIN_UART, -1, -1);

    // --- Configure TX UART (i-Bus) ---
    const uart_config_t tx_uart_config = {
        .baud_rate = IBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, // No parity for i-Bus
        .stop_bits = UART_STOP_BITS_1, // 1 stop bit for i-Bus
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(TX_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(TX_UART_NUM, &tx_uart_config);
    uart_set_pin(TX_UART_NUM, TX_PIN_UART, RX_PIN_UART_TX, -1, -1);
    // NOTE: The uart_set_line_inverse() call for SBUS has been removed.

    // --- Create and Pin Tasks to Core 1 ---
    xTaskCreatePinnedToCore(crsf_rx_task, "crsf_rx_task", 4096, NULL, REALTIME_TASK_PRIORITY, NULL, 1);
    xTaskCreatePinnedToCore(ibus_tx_task, "ibus_tx_task", 4096, NULL, REALTIME_TASK_PRIORITY, NULL, 1);
}