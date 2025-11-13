import serial
import math
import numpy as np

# This function is now self-contained. It performs one full 360-degree scan
# and returns the result as a NumPy array.
def get_lidar_scan(com):
    """
    Reads data from the LIDAR sensor until one full 360-degree scan is complete.

    Args:
        com (serial.Serial): An active and open pyserial connection object.

    Returns:
        numpy.ndarray: A 360-element array where the index is the angle in
                       degrees (0-359) and the value is the measured distance
                       in millimeters. Returns None if a scan cannot be completed.
    """
    state = 0
    last_angle_deg = 0.0
    scan_data = np.zeros(360)
    
    # We need to wait for the first "zero-crossing" to ensure we have a full scan.
    scan_started = False

    while True:
        # State 0: Look for the first sync byte (0xAA)
        if state == 0:
            # You had com.read(0) here, which reads zero bytes. It should be 1.
            sync = com.read(1)
            if not sync:
                continue
            if sync[0] == 0xAA:
                state = 1

        # State 1: Look for the second sync byte (0x55)
        elif state == 1:
            sync = com.read(1)
            if not sync:
                state = 0
                continue
            if sync[0] == 0x55:
                state = 2
            else:
                state = 0  # Reset if the sequence is wrong

        # State 2: Read the header
        elif state == 2:
            header = com.read(8)
            if len(header) < 8:
                state = 0
                continue

            package_type = header[0]
            package_size = header[1]
            start_angle = (header[3] << 8) | header[2]
            stop_angle = (header[5] << 8) | header[4]
            state = 3

        # State 3: Read and process the data payload
        elif state == 3:
            # Always reset state to look for the next packet's header
            state = 0

            if package_size == 0:
                continue

            data = com.read(package_size * 3)

            # Basic validation
            if len(data) < package_size * 3 or package_type & 0x01:
                continue

            # Calculate the angular difference and step between measurements
            diff = stop_angle - start_angle
            if diff < 0:
                diff = 0xB400 - start_angle + stop_angle

            step = 0
            if diff > 1 and package_size > 1:
                step = diff / (package_size - 1)

            # Process each measurement in the packet
            for i in range(package_size):
                raw_distance = (data[i * 3 + 2] << 8) | data[i * 3 + 1]
                distance_mm = raw_distance / 4.0

                raw_angle = (start_angle + step * i)
                angle_deg = (raw_angle / 0xB400) * 360.0

                # Mark that we have started receiving data for a scan
                if not scan_started:
                    scan_started = True

                # --- THIS IS THE KEY EXIT CONDITION ---
                # If we have started a scan AND the angle has wrapped around,
                # it means the 'scan_data' array is now full of the last
                # complete scan. We can return it.
                if scan_started and angle_deg < last_angle_deg:
                    return scan_data

                # Get the integer part of the angle to use as an array index
                index = int(angle_deg)
                if 0 <= index < 360:
                    # Store the distance at the corresponding angle
                    scan_data[index] = distance_mm
                
                last_angle_deg = angle_deg


# --- Main part of the script to show how to use the function ---
if __name__ == '__main__':
    try:
        print("Connecting to LIDAR...")
        com = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=158700,
            timeout=1  # Using a timeout is good practice
        )
        print("Connected. Getting first scan...")

        # --- USAGE EXAMPLE ---
        lidar_data = get_lidar_scan(com)

        if lidar_data is not None:
            # Now you have the data and can do whatever you want with it
            print("Successfully received a full 360-degree scan.")
            
            # For your drone project, you would extract the specific angles here:
            dist_right = lidar_data[90]
            dist_left = lidar_data[270]
            dist_front = lidar_data[0] # Or 359/0
            dist_back = lidar_data[180]

            print(f"Distance to the Right (90 deg): {dist_right:.2f} mm")
            print(f"Distance to the Left (270 deg): {dist_left:.2f} mm")
            print(f"Distance to the Front (0 deg): {dist_front:.2f} mm")
            print(f"Distance to the Back (180 deg): {dist_back:.2f} mm")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        if 'com' in locals() and com.is_open:
            com.close()