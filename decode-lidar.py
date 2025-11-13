import serial
import math
import time
import numpy as np
import matplotlib.pyplot as plt

# --- Serial Port Configuration ---
# Make sure to use the correct port and baudrate for your device
com = serial.Serial(
    port="/dev/ttyUSB0",  # For Windows, this might be "COM13"
    baudrate=158700,      # Double-check this is correct for your LIDAR
    timeout=1             # Use a timeout to prevent the script from freezing
)

# --- Plotting Setup ---
plt.ion()  # Turn on interactive mode for real-time plotting
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)
ax.set_title('LIDAR Data', fontsize=18)

# Start with a simple scatter plot. We will update its data.
# The 'r' is for red, 'o' is for circle markers.
scan_plot = ax.scatter([], [], s=5, c='r')
ax.set_rlim(0, 1600)  # Set the radial limit (max distance in mm)
ax.set_theta_zero_location('N')  # Set 0 degrees to the top
ax.set_theta_direction(-1)       # Plot angles clockwise

# --- Data Storage ---
# We'll store one full 360-degree scan here.
# Index 0 = 0 degrees, Index 1 = 1 degree, etc.
distances = np.zeros(360)
angles_rad = np.deg2rad(np.arange(360)) # Pre-calculate angles in radians for plotting

# --- State Machine and Parsing Variables ---
state = 0
last_angle_deg = 0

print("Starting LIDAR processing...")

try:
    running = True
    while running:
        # State 0: Look for the first sync byte (0xAA)
        if state == 0:
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
                state = 0 # Reset if the sequence is wrong

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
            # Always reset state to look for the next packet
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
                # Extract distance and convert to millimeters
                raw_distance = (data[i * 3 + 2] << 8) | data[i * 3 + 1]
                distance_mm = raw_distance / 4.0

                # Calculate the angle for this specific measurement
                raw_angle = (start_angle + step * i)
                # Convert from sensor's internal format to degrees (0-359)
                angle_deg = (raw_angle / 0xB400) * 360.0

                # --- This is the key logic for plotting ---
                # Check if a new 360-degree scan has started
                if angle_deg < last_angle_deg:
                    # A new scan has begun. Update the plot with the OLD scan data.
                    valid_points = distances > 0 # Only plot points that have valid data
                    scan_plot.set_offsets(np.vstack((angles_rad[valid_points], distances[valid_points])).T)
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    # Reset the data array for the new scan
                    distances.fill(0)

                # Get the integer part of the angle to use as an array index
                index = int(angle_deg)
                if 0 <= index < 360:
                    # Store the distance at the corresponding angle
                    distances[index] = distance_mm
                
                last_angle_deg = angle_deg

except KeyboardInterrupt:
    print("Stopping.")
finally:
    com.close()
    plt.ioff()
    plt.show() # Show the final plot when the script is stopped