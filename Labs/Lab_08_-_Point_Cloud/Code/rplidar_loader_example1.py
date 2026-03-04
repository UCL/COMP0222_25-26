from rplidar_driver import LidarDriver

# --- REPLAY & EXPERIMENT ---
# Initialize in 'replay' mode. No USB device needed.
driver = LidarDriver(mode='replay', filename='lab_data_01.json')

print("Starting Replay...")

for i, scan in enumerate(driver.iter_scans()):
    # ----------------------------------------
    # STUDENT CODE GOES HERE
    # Experiment with filtering parameters safely
    # ----------------------------------------
    
    # Example: Filter out noise (distance < 100mm)
    valid_points = [pt for pt in scan if pt[2] > 100]
    
    print(f"Replay Scan #{i}: Raw={len(scan)} points | Filtered={len(valid_points)} points")