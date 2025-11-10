import struct
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass, field
import serial
import os
import sys

# Constants
MAJOR_VER = 1
MINOR_VER = 0
PATCH_LEV = 0

# Sync bytes
CRSF_RC_SYNC_BYTE = 0xC8
CRSF_TEL_SYNC_BYTE = 0xEE

# Frame IDs
GPS_ID = 0x02
CF_VARIO_ID = 0x07
BATTERY_ID = 0x08
BARO_ALT_ID = 0x09
HEARTBEAT_ID = 0x0B
LINK_ID = 0x14
CHANNELS_ID = 0x16
LINK_RX_ID = 0x1C
LINK_TX_ID = 0x1D
ATTITUDE_ID = 0x1E
FLIGHT_MODE_ID = 0x21
PING_DEVICES_ID = 0x28
DEVICE_INFO_ID = 0x29
REQUEST_SETTINGS_ID = 0x2A
COMMAND_ID = 0x32
RADIO_ID = 0x3A

RADS2DEGS = 57.29578
RSSI_CHANNEL = 16  # Default RSSI channel


@dataclass
class CRSFData:
    """Container for decoded CRSF data"""
    # GPS data
    gps_lat: int = 0
    gps_lon: int = 0
    gpsF_lat: float = 0.0
    gpsF_lon: float = 0.0
    gps_groundspeed: int = 0
    gpsF_groundspeed: float = 0.0
    gps_heading: int = 0
    gpsF_heading: float = 0.0
    gps_altitude: int = 0
    gps_sats: int = 0
    
    # Battery data
    bat_voltage: int = 0
    batF_voltage: float = 0.0
    bat_current: int = 0
    batF_current: float = 0.0
    bat_fuel_drawn: int = 0
    batF_fuel_drawn: float = 0.0
    bat_remaining: int = 0
    
    # Link statistics
    link_up_rssi_ant_1: int = 0
    link_up_rssi_ant_2: int = 0
    link_up_quality: int = 0
    link_up_snr: int = 0
    link_diversity_active_ant: int = 0
    link_rf_mode: int = 0
    link_up_tx_power: int = 0
    link_dn_rssi: int = 0
    link_dn_quality: int = 0
    link_dn_snr: int = 0
    
    # Attitude data
    atti_pitch: int = 0
    atti_roll: int = 0
    atti_yaw: int = 0
    attiF_pitch: float = 0.0
    attiF_roll: float = 0.0
    attiF_yaw: float = 0.0
    
    # Flight mode
    flight_mode: str = ""
    
    # RC channels
    pwm_values: List[int] = field(default_factory=lambda: [1500] * 24)
    rssi_percent: int = 0


class CRSFDecoder:
    def __init__(self, port=None, is_rc_build=False, buffer_size=64, debug=False):
        self.port = port
        self.is_rc_build = is_rc_build
        self.buffer_size = buffer_size
        self.crsf_buf = bytearray(buffer_size)
        self.data = CRSFData()
        self.debug = debug
        
        # MODIFICATION: Added buffer to store the last successfully validated frame
        self.last_good_frame = bytearray()
        
        # Statistics
        self.frames_read = 0
        self.good_frames = 0
        self.crc_errors = 0
        self.frame_errors = 0
        self.unknown_ids = 0
        self.bytes_received = 0
        
        # State for frame reading
        self.idx = 0
        
    def crc8_dvb_s2(self, crc: int, byte: int) -> int:
        """Calculate CRC8 DVB-S2"""
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        return crc
    
    def crc8_dvb_s2_buf(self, data: bytes) -> int:
        """Calculate CRC8 for buffer"""
        crc = 0
        for byte in data:
            crc = self.crc8_dvb_s2(crc, byte)
        return crc
    
    def bytes_to_int32(self, byt: bytes) -> int:
        """Convert 4 bytes to signed int32"""
        return struct.unpack('<i', byt)[0]
    
    def bytes_to_uint16(self, byt: bytes) -> int:
        """Convert 2 bytes to unsigned int16"""
        return struct.unpack('<H', byt)[0]
    
    def bytes_to_int16(self, byt: bytes) -> int:
        """Convert 2 bytes to signed int16"""
        return struct.unpack('<h', byt)[0]
    
    def wrap360(self, ang: int) -> int:
        """Wrap angle to 0-359 range"""
        while ang < 0:
            ang += 360
        while ang > 359:
            ang -= 360
        return ang
    
    def read_crsf_frame(self) -> Tuple[bool, int]:
        """
        Read and validate a CRSF frame from the port
        Returns: (success, frame_length)
        """
        if self.port is None or not hasattr(self.port, 'read'):
            return False, 0
        
        if hasattr(self.port, 'in_waiting'):
            if self.port.in_waiting == 0:
                return False, 0
        
        byte_data = self.port.read(1)
        if not byte_data:
            return False, 0
        
        b = byte_data[0]
        self.bytes_received += 1
        
        if self.debug and self.bytes_received % 100 == 0:
            print(f"Bytes received: {self.bytes_received}, Current byte: 0x{b:02X}, idx: {self.idx}")
        
        sync_byte = CRSF_RC_SYNC_BYTE if self.is_rc_build else CRSF_TEL_SYNC_BYTE
        
        if b == sync_byte:
            if self.idx > 2:
                self.frames_read += 1
                frm_lth = self.idx
                
                if self.debug:
                    print(f"\nFrame detected, length: {frm_lth}")
                    print(f"Buffer: {' '.join(f'{x:02X}' for x in self.crsf_buf[:min(frm_lth+1, len(self.crsf_buf))])}")
                
                if frm_lth > 1:
                    expected_lth = self.crsf_buf[1] + 2
                    
                    if self.debug:
                        print(f"Expected length: {expected_lth}, Actual: {frm_lth}")
                    
                    if frm_lth >= 3:
                        crc_start = 2
                        crc_end = frm_lth - 1
                        crc_data = bytes(self.crsf_buf[crc_start:crc_end])
                        calc_crc = self.crc8_dvb_s2_buf(crc_data)
                        embed_crc = self.crsf_buf[frm_lth - 1]
                        
                        if self.debug:
                            print(f"CRC check: embedded=0x{embed_crc:02X}, calculated=0x{calc_crc:02X}")
                        
                        if embed_crc == calc_crc:
                            self.good_frames += 1
                            # MODIFICATION: Store the validated frame so it can be used in the main loop
                            self.last_good_frame = self.crsf_buf[:frm_lth]
                            result_len = frm_lth
                            
                            self.idx = 0
                            self.crsf_buf = bytearray(self.buffer_size)
                            self.crsf_buf[self.idx] = b
                            self.idx += 1
                            
                            return True, result_len
                        else:
                            self.crc_errors += 1
                            if self.debug:
                                print(f"CRC mismatch!")
            
            self.idx = 0
            self.crsf_buf = bytearray(self.buffer_size)
            self.crsf_buf[self.idx] = b
            self.idx += 1
            
            if self.debug and self.bytes_received % 100 == 0:
                print(f"Sync byte 0x{sync_byte:02X} found at byte {self.bytes_received}")
        else:
            if self.idx > 0 and self.idx < self.buffer_size - 1:
                self.crsf_buf[self.idx] = b
                self.idx += 1
        
        return False, 0
    
    def bytes_to_pwm(self, sb_bytes: bytes, max_ch: int = 16) -> List[int]:
        """
        Convert SBUS bytes to PWM values
        Returns list of PWM values (1000-2000 µs)
        """
        ch_val = [0] * 24
        
        ch_val[0] = (sb_bytes[0] | (sb_bytes[1] << 8)) & 0x07FF
        ch_val[1] = ((sb_bytes[1] >> 3) | (sb_bytes[2] << 5)) & 0x07FF
        ch_val[2] = ((sb_bytes[2] >> 6) | (sb_bytes[3] << 2) | (sb_bytes[4] << 10)) & 0x07FF
        ch_val[3] = ((sb_bytes[4] >> 1) | (sb_bytes[5] << 7)) & 0x07FF
        ch_val[4] = ((sb_bytes[5] >> 4) | (sb_bytes[6] << 4)) & 0x07FF
        ch_val[5] = ((sb_bytes[6] >> 7) | (sb_bytes[7] << 1) | (sb_bytes[8] << 9)) & 0x07FF
        ch_val[6] = ((sb_bytes[8] >> 2) | (sb_bytes[9] << 6)) & 0x07FF
        ch_val[7] = ((sb_bytes[9] >> 5) | (sb_bytes[10] << 3)) & 0x07FF
        
        if max_ch >= 16:
            ch_val[8] = (sb_bytes[11] | (sb_bytes[12] << 8)) & 0x07FF
            ch_val[9] = ((sb_bytes[12] >> 3) | (sb_bytes[13] << 5)) & 0x07FF
            ch_val[10] = ((sb_bytes[13] >> 6) | (sb_bytes[14] << 2) | (sb_bytes[15] << 10)) & 0x07FF
            ch_val[11] = ((sb_bytes[15] >> 1) | (sb_bytes[16] << 7)) & 0x07FF
            ch_val[12] = ((sb_bytes[16] >> 4) | (sb_bytes[17] << 4)) & 0x07FF
            ch_val[13] = ((sb_bytes[17] >> 7) | (sb_bytes[18] << 1) | (sb_bytes[19] << 9)) & 0x07FF
            ch_val[14] = ((sb_bytes[19] >> 2) | (sb_bytes[20] << 6)) & 0x07FF
            ch_val[15] = ((sb_bytes[20] >> 5) | (sb_bytes[21] << 3)) & 0x07FF
        
        for i in range(max_ch, 24):
            ch_val[i] = 1500
        
        rssi_pwm = ch_val[RSSI_CHANNEL - 1]
        rssi_percent = int((rssi_pwm - 1000) * 100 / 1000)
        self.data.rssi_percent = max(0, min(100, rssi_percent))
        
        for i in range(max_ch):
            if ch_val[i] > 0:
                # This scaling is specific to CRSF -> SBUS, which maps 172-1811 to 988-2012us
                # A simpler linear mapping is often used. The original code's scaling is below:
                ch_val[i] = int((ch_val[i] - 192) * 1000 / 1600 + 1000)

        return ch_val
    
    def decode_telemetry(self, buf: bytes, length: int) -> Optional[int]:
        """
        Decode telemetry frame
        Returns frame ID if successful, None otherwise
        """
        if length < 3: return None
        
        crsf_frm_lth = buf[1]
        crsf_id = buf[2]
        
        if crsf_id == 0: return None
        
        try:
            if crsf_id == GPS_ID:
                self.data.gps_lat = self.bytes_to_int32(buf[3:7])
                self.data.gps_lon = self.bytes_to_int32(buf[7:11])
                self.data.gpsF_lat = self.data.gps_lat / 1e7
                self.data.gpsF_lon = self.data.gps_lon / 1e7
                self.data.gps_groundspeed = self.bytes_to_uint16(buf[11:13])
                self.data.gpsF_groundspeed = self.data.gps_groundspeed * 0.1
                self.data.gps_heading = self.bytes_to_uint16(buf[13:15])
                self.data.gpsF_heading = self.data.gps_heading * 0.01
                self.data.gps_altitude = self.bytes_to_uint16(buf[15:17])
                if self.data.gps_altitude > 100: self.data.gps_altitude -= 1000
                self.data.gps_sats = buf[17]
            elif crsf_id == BATTERY_ID:
                self.data.bat_voltage = self.bytes_to_uint16(buf[3:5])
                self.data.batF_voltage = self.data.bat_voltage * 0.1
                self.data.bat_current = self.bytes_to_uint16(buf[5:7])
                self.data.batF_current = self.data.bat_current * 0.1
                self.data.bat_fuel_drawn = int.from_bytes(buf[7:10], 'little', signed=False)
                self.data.batF_fuel_drawn = float(self.data.bat_fuel_drawn)
                self.data.bat_remaining = buf[10]
            elif crsf_id == LINK_ID:
                self.data.link_up_rssi_ant_1 = buf[3]
                self.data.link_up_rssi_ant_2 = buf[4]
                self.data.link_up_quality = buf[5]
                self.data.link_up_snr = struct.unpack('b', bytes([buf[6]]))[0]
                self.data.link_diversity_active_ant = buf[7]
                self.data.link_rf_mode = buf[8]
                self.data.link_up_tx_power = buf[9]
                self.data.link_dn_rssi = buf[10]
                self.data.link_dn_quality = buf[11]
                self.data.link_dn_snr = struct.unpack('b', bytes([buf[12]]))[0]
            elif crsf_id == ATTITUDE_ID:
                self.data.atti_pitch = self.bytes_to_int16(buf[3:5])
                self.data.atti_roll = self.bytes_to_int16(buf[5:7])
                self.data.atti_yaw = self.bytes_to_int16(buf[7:9])
                self.data.attiF_pitch = self.data.atti_pitch * RADS2DEGS * 0.0001
                self.data.attiF_roll = self.data.atti_roll * RADS2DEGS * 0.0001
                yaw_deg = int(self.data.atti_yaw * RADS2DEGS * 0.0001)
                self.data.atti_yaw = self.wrap360(yaw_deg)
                self.data.attiF_yaw = float(self.data.atti_yaw)
            elif crsf_id == FLIGHT_MODE_ID:
                flight_mode_lth = crsf_frm_lth - 3
                self.data.flight_mode = buf[3:3+flight_mode_lth].decode('utf-8', errors='ignore')
            elif crsf_id in [CF_VARIO_ID, BARO_ALT_ID, HEARTBEAT_ID,
                            LINK_RX_ID, LINK_TX_ID, PING_DEVICES_ID, DEVICE_INFO_ID,
                            REQUEST_SETTINGS_ID, COMMAND_ID, RADIO_ID]:
                pass
            else:
                self.unknown_ids += 1
                return None
            return crsf_id
        except (IndexError, struct.error) as e:
            self.frame_errors += 1
            return None
    
    def decode_rc(self, buf: bytes, max_ch: int = 16):
        """Decode RC channels from CRSF frame's payload"""
        if len(buf) < 22: return
        self.data.pwm_values = self.bytes_to_pwm(buf, max_ch)
    
    def print_stats(self):
        """Print frame statistics"""
        print(f"Frames read: {self.frames_read}")
        print(f"Good frames: {self.good_frames}")
        print(f"CRC errors: {self.crc_errors}")
        print(f"Frame errors: {self.frame_errors}")
        print(f"Unknown IDs: {self.unknown_ids}")

# --- NEW VISUALIZATION FUNCTION ---
def visualize_channels(pwm_values: list, rssi: int):
    """Clears the console and prints a text-based visualization of RC channels."""
    os.system('cls' if os.name == 'nt' else 'clear')
    
    print("--- CRSF RC Channel Monitor ---")
    print(f"RSSI: {rssi}% (from Channel {RSSI_CHANNEL})\n")
    
    for i, pwm in enumerate(pwm_values[:16]):  # Visualize the first 16 channels
        min_pwm, max_pwm = 1000, 2000
        bar_width = 50
        
        # Clamp the PWM value to be within the expected visual range
        clamped_pwm = max(min_pwm, min(max_pwm, pwm))
        
        # Map the PWM value to the bar's width
        bar_len = int(((clamped_pwm - min_pwm) / (max_pwm - min_pwm)) * bar_width)
        
        # Create the bar string
        bar = '█' * bar_len + ' ' * (bar_width - bar_len)
        
        # Create the visual representation with a center marker
        center_marker_pos = bar_width // 2
        display_bar = list(bar)
        # Place the marker, but don't overwrite the bar itself
        if display_bar[center_marker_pos] == ' ':
            display_bar[center_marker_pos] = '│'

        print(f"CH {i+1:02d} | {pwm:4d} µs | [{''.join(display_bar)}]")
        
    print("\n--- Press Ctrl+C to stop ---")

# --- MODIFIED EXAMPLE USAGE ---
if __name__ == "__main__":
    try:
        # --- Port Configuration ---
        # Adjust these parameters to match your setup
        PORT = '/dev/ttyUSB0'  # Linux example. Windows: 'COM3'
        BAUDRATE = 420000       # CRSF standard baudrate
        # Set to True for RC frames (0xC8), False for standard telemetry (0xEE)
        # Most radios/receivers send channels with the 0xC8 sync byte.
        IS_RC = True
        
        print(f"Attempting to open {PORT} at {BAUDRATE} baud...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.02)
        decoder = CRSFDecoder(port=ser, is_rc_build=IS_RC, debug=False)
        
        print("CRSF Decoder started. Waiting for data...")
        
        while True:
            success, length = decoder.read_crsf_frame()
            
            if success:
                # Use the saved good frame for decoding
                frame_buffer = decoder.last_good_frame
                frame_id = frame_buffer[2] # Frame ID is the 3rd byte

                if frame_id == CHANNELS_ID:
                    # The payload for channels starts after the ID at index 3
                    # The full frame is [SYNC, LEN, ID, PAYLOAD..., CRC]
                    decoder.decode_rc(bytes(frame_buffer[3:-1]), 16)
                    visualize_channels(decoder.data.pwm_values, decoder.data.rssi_percent)
                    time.sleep(0.01) # Control the screen refresh rate
                
                # You can still decode and print other telemetry frames
                else:
                    decoded_id = decoder.decode_telemetry(bytes(frame_buffer), length)
                    if decoded_id == GPS_ID:
                        print(f"[Telemetry] GPS: Lat={decoder.data.gpsF_lat:.6f}, Lon={decoder.data.gpsF_lon:.6f}, "
                              f"Alt={decoder.data.gps_altitude}m, Sats={decoder.data.gps_sats}")
                    elif decoded_id == BATTERY_ID:
                        print(f"[Telemetry] Battery: {decoder.data.batF_voltage:.1f}V, "
                              f"{decoder.data.batF_current:.1f}A, {decoder.data.bat_remaining}%")
                    elif decoded_id == LINK_ID:
                        print(f"[Telemetry] Link: RSSI={decoder.data.link_dn_rssi}dBm, "
                              f"Quality={decoder.data.link_dn_quality}%")
                    elif decoded_id == FLIGHT_MODE_ID:
                        print(f"[Telemetry] Flight Mode: {decoder.data.flight_mode}")

    except serial.SerialException as e:
        print(f"\n--- Serial Port Error ---")
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Is the correct serial port name used? (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)")
        print("2. Do you have permission to access the port? (On Linux, try 'sudo chmod a+rw /dev/ttyUSB0')")
        print("3. Is the device connected and powered on?")
        print("4. Is another program (like Betaflight Configurator) using the port?")
        print("5. Have you tried other baudrates? (e.g., 115200, 400000)")
    except KeyboardInterrupt:
        print("\n\nStopping decoder...")
        decoder.print_stats()
        print(f"Total bytes received: {decoder.bytes_received}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")