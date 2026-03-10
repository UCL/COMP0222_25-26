import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from rplidar import RPLidar
import platform


# 0. Detect OS
os_name = platform.system()

# 2. Assign Port based on OS
if os_name == 'Windows':
    port_name = 'COM7'                # Windows default
elif os_name == 'Darwin':             # macOS
    port_name = '/dev/tty.SLAB_USBtoUART' # Common driver name for RPLIDAR on Mac
else:                                 # Linux / Ubuntu
    port_name = '/dev/ttyUSB0'        # Linux default

print(f"Detected {os_name}. Trying port: {port_name}")

# 1. Setup Lidar
try:
    lidar = RPLidar(port_name, baudrate=256000)
except Exception as e:
    print(f"Error connecting to {port_name}: {e}")
    print("Please check your connection or manually edit the port_name variable.")

# 2. Setup Plot
fig = plt.figure('RPLIDAR Scan')
ax = plt.subplot(111, projection='polar')
ax.set_rmax(4000)
ax.grid(True)

# 3. "Stop" Button
# Position: [left, bottom, width, height] (0-1 coordinates)
ax_stop = plt.axes([0.8, 0.05, 0.1, 0.075]) 
stop_btn = Button(ax_stop, 'Stop')

# 4. Define what happens when the button is clicked
is_running = True

def stop_callback(event):
    global is_running
    print("Stop button clicked!")
    is_running = False

stop_btn.on_clicked(stop_callback)

# 5. Start Scanning
iterator = lidar.iter_scans()

try:
    print("Starting Scan... Click 'Stop' or close window to exit.")
    
    for scan in iterator:
        # Check if the button was clicked
        if not is_running:
            break
            
        # Check if the user closed the window manually using 'X'
        if not plt.fignum_exists(fig.number):
            break

        # --- Data Processing ---
        # Convert (quality, angle, distance) -> (angle_radians, distance)
        offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
        
        # --- Visualization ---
        ax.clear()
        ax.set_rmax(4000) 
        ax.grid(True)
        ax.set_title("RPLIDAR Scan", va='bottom')

        if len(offsets) > 0:
            ax.scatter(offsets[:, 0], offsets[:, 1], s=5, c='blue')

        # This pause updates the plot and checks for button clicks
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Stopping by keyboard...")

except Exception as e:
    print(f"Error: {e}")

finally:
    # 6. Safe Shutdown
    print("Shutting down Lidar...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    plt.close() 
    print("Disconnected.")