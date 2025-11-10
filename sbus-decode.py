import serial
import struct
import time

class IBusReader:
    """
    A class to read and decode Flysky iBus data from a serial port.
    """
    IBUS_FRAME_LENGTH = 32
    IBUS_HEADER_LSB = 0x20
    IBUS_HEADER_MSB = 0x40
    IBUS_MAX_CHANNELS = 14

    def __init__(self, serial_port):
        """
        Initializes the IBusReader.
        """
        self.ser = serial.Serial(
            port=serial_port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05 # Increased timeout slightly for better reading of full chunks
        )

    def read_ibus_data(self):
        """
        Reads data from the serial port and attempts to find/parse an iBus frame.
        """
        # Read enough data to hold a few packets to ensure a full frame is available
        # The iBus frames are sent every ~7ms, so reading a small chunk is fine.
        raw_data = self.ser.read(self.IBUS_FRAME_LENGTH * 4)
        if not raw_data:
            return None

        # Convert to bytearray for efficient searching and slicing
        data_buffer = bytearray(raw_data)
        
        while len(data_buffer) >= self.IBUS_FRAME_LENGTH:
            # 1. Find the header (0x20 0x40)
            # Find 0x20 first
            start_index_20 = data_buffer.find(self.IBUS_HEADER_LSB)
            
            if start_index_20 == -1:
                # No header start found, discard everything and wait for new data
                data_buffer.clear()
                return None

            # If we found 0x20, check if 0x40 follows it
            if start_index_20 + 1 < len(data_buffer) and data_buffer[start_index_20 + 1] == self.IBUS_HEADER_MSB:
                # Found the full header 0x20 0x40.

                # 2. Check if the full frame is available
                if start_index_20 + self.IBUS_FRAME_LENGTH <= len(data_buffer):
                    # Full frame is available! Extract it.
                    frame = data_buffer[start_index_20 : start_index_20 + self.IBUS_FRAME_LENGTH]
                    
                    # Consume the data up to the end of this frame
                    data_buffer = data_buffer[start_index_20 + self.IBUS_FRAME_LENGTH:]
                    
                    # Parse and return the result
                    result = self.parse_ibus_frame(frame)
                    if result:
                        return result
                    
                    # If parsing failed (checksum), we loop to check for the next possible frame
                    continue 
                else:
                    # Partial frame, wait for more data. Keep the remaining bytes.
                    break
            else:
                # Found 0x20 but not 0x40. Discard 0x20 and continue the search for the next 0x20
                data_buffer = data_buffer[start_index_20 + 1:]
        
        return None

    def parse_ibus_frame(self, frame: bytes):
        """
        Parses a 32-byte iBus frame, validates the checksum, and extracts channel data.
        """
        # 1. Verify Header (redundant, but good final check)
        if frame[0] != self.IBUS_HEADER_LSB or frame[1] != self.IBUS_HEADER_MSB:
             # print("Error: Frame passed to parser does not have correct header.")
             return None
             
        # 2. Verify Checksum
        # Checksum is 0xFFFF minus the sum of the first 30 bytes.
        payload = frame[0:30]
        checksum_calculated = 0xFFFF - sum(payload)
        
        # Checksum from the packet is a 16-bit little-endian value (last 2 bytes)
        checksum_received = struct.unpack('<H', frame[30:32])[0]

        if checksum_calculated != checksum_received:
            # print(f"Checksum mismatch! Calculated: {checksum_calculated}, Received: {checksum_received}. Frame: {' '.join(f'{b:02x}' for b in frame)}")
            return None

        # 3. Extract Channel Data
        # Channels are 14 x 16-bit little-endian values starting from byte 2.
        channel_payload = frame[2:30]
        # '<' means little-endian, '14H' means 14 unsigned short integers (2 bytes each)
        channels = list(struct.unpack('<' + 'H' * self.IBUS_MAX_CHANNELS, channel_payload))
        
        return channels

    def close(self):
        self.ser.close()

if __name__ == "__main__":
    ibus_port = '/dev/ttyUSB0' # <-- CHANGE THIS
    
    try:
        ibus_reader = IBusReader(ibus_port)
        print(f"Successfully opened iBus port: {ibus_port}")
        
        while True:
            channels = ibus_reader.read_ibus_data()
            
            if channels is not None:
                # Format the output to be clean
                # Print only the first 6 channels, as they are most common for control
                channel_str = ", ".join([f"Ch{i+1}: {ch:<4}" for i, ch in enumerate(channels[:6])])
                print(channel_str)
            
            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Error opening serial port {ibus_port}: {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if 'ibus_reader' in locals() and ibus_reader:
            ibus_reader.close()
            print("Serial port closed.")