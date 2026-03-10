import matplotlib.pyplot as plt
import numpy as np
from rplidar_driver import LidarDriver  # Import the class we built

# 1. Setup the driver in Replay mode
driver = LidarDriver(mode='replay', filename='lab_data_01.json')

# 2. Setup the Plot
plt.ion() # Turn on interactive mode for live updates
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Set a fixed maximum radius (e.g., 2000mm = 2 meters) so the zoom doesn't jump around
MAX_DISTANCE_MM = 2000 
ax.set_rlim(0, MAX_DISTANCE_MM)

print("Starting Replay Visualization...")

try:
    # 3. Loop through the file data
    for scan in driver.iter_scans():
        
        # --- Data Processing ---
        # scan format: [(quality, angle_deg, distance_mm), ...]
        
        # Extract angles and distances
        angles = []
        distances = []
        
        for point in scan:
            quality, angle_deg, dist_mm = point
            
            # Filter out bad data (distance = 0 often means invalid)
            if dist_mm > 0:
                # Matplotlib requires Radians, Lidar gives Degrees
                angles.append(np.radians(angle_deg))
                distances.append(dist_mm)

        # --- Plotting ---
        ax.clear() # Clear previous frame
        
        # Re-apply formatting (clearing removes labels/limits)
        ax.set_rlim(0, MAX_DISTANCE_MM)
        ax.set_title("Lidar Replay (Press Ctrl+C to Stop)", va='bottom')
        
        # Scatter plot: theta (angles), r (distances)
        # s=5 controls dot size, c=distances adds color based on depth
        ax.scatter(angles, distances, s=5, c=distances, cmap='hsv', alpha=0.75)
        
        plt.draw()
        plt.pause(0.01) # Small pause to allow plot to render

except KeyboardInterrupt:
    print("Stopping Replay.")

plt.close()