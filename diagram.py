import serial
import time
import math
import matplotlib.pyplot as plt

# -------------------------------
# Configuration Parameters
# -------------------------------
port = 'COM12'              # Your HC-05 outgoing COM port
baudrate = 9600             # Baud rate, must match HC-05 setting
step_size = 1.2          # Movement distance per forward/backward command (in cm)
rotation_angle = math.radians(100)  # Rotate right by 36 degrees per command (in radians)

# -------------------------------
# Initial Car State
# -------------------------------
# Start at origin (0,0), facing east (0 radians)
x = 0.0
y = 0.0
theta = 0.7854            # 45° in radians
counter = 0               # Counter for forward commands

# Lists to store the car's path and coin positions
path_x = [x]
path_y = [y]
coin_positions = []       # List to store coin markers as (x, y)
received_first_command = False  # Flag to check if any command has been received

# -------------------------------
# Open the Serial Port
# -------------------------------
try:
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Wait for the serial connection to stabilize
    print(f"✅ Serial port {port} opened successfully. Listening for commands...")
except serial.SerialException as e:
    print(f"❌ Failed to open serial port {port}: {e}")
    exit()

# -------------------------------
# Setup the Real-time Plot
# -------------------------------
plt.ion()  # Enable interactive mode for live updating
fig, ax = plt.subplots()

# Configure plot appearance (no fixed boundaries; autoscale will adjust based on data)
ax.set_title('Car Path Visualization')
ax.set_xlabel('X (cm)')
ax.set_ylabel('Y (cm)')
ax.grid(True)
ax.set_aspect('equal', adjustable='box')
ax.legend()

# Plot the initial path (blue line) and current car position (blue dot)
line_path, = ax.plot(path_x, path_y, 'b-', label='Path')
current_point, = ax.plot([x], [y], 'bo', label='Car Position')

# Prepare coin scatter plot for coin positions (red dots)
coin_scatter = ax.scatter([], [], c='r', marker='o', label='Coins')

# Draw initial state at (0,0)
current_point.set_data([0], [0])
line_path.set_data([0], [0])
plt.draw()
plt.pause(0.05)

# -------------------------------
# Main Loop: Read Serial Data and Update Plot
# -------------------------------
try:
    while True:
        if ser.in_waiting:
            # Read one line of data and decode it
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print("📥 Received command:", data)
                # Check if the data is a valid digit (command)
                if data.isdigit():
                    received_first_command = True
                    command = int(data)
                    
                    # If command is 0, end the program (e.g., coin collection complete)
                    if command == 0:
                        print("Action: End of coin collection. Exiting...")
                        break

                    if command == 1:  # Forward
                        x += step_size * math.cos(theta)
                        y += step_size * math.sin(theta)
                        counter += 1
                        print("Action: Forward", counter)
                    elif command == 2:  # Backward
                        x -= step_size * math.cos(theta)
                        y -= step_size * math.sin(theta)
                        print("Action: Backward")
                    elif command == 3:  # Rotate Right
                        theta -= rotation_angle  # Rotate right by subtracting the angle
                        print("Action: Rotate Right")
                        time.sleep(1)  # Wait 1 second to simulate turning time
                    elif command == 4:  # Pick Coin
                        # Record coin position; do not change x, y so the car stays in place
                        coin_positions.append((x, y))
                        print("Action: Pick Coin")
                    else:
                        print("Unknown command:", command)
                    
                    # Record the current position (for command 4, this will be the same as before)
                    path_x.append(x)
                    path_y.append(y)
                    
                    # Update the path line and current position marker
                    if received_first_command:
                        line_path.set_data(path_x, path_y)
                        current_point.set_data([x], [y])
                    else:
                        current_point.set_data([0], [0])
                    
                    # Update coin markers if any coins have been picked
                    if coin_positions:
                        coin_scatter.set_offsets(coin_positions)
                    
                    # Redraw plot with updated data
                    ax.relim()
                    ax.autoscale_view()
                    plt.draw()
                    plt.pause(0.05)
        else:
            time.sleep(0.05)
except KeyboardInterrupt:
    print("\n🛑 Exiting program by user.")
finally:
    ser.close()
    plt.ioff()
    plt.show()