from rplidar_driver import LidarDriver

# --- RECORD DATA ---

# Initialize
driver = LidarDriver(mode='live', filename='lab_data_01.json')

# Run the loop. 
# When you press Ctrl+C, it will stop. Run this file from terminal.
for i, scan in enumerate(driver.iter_scans()):
    print(f"Scan {i}: {len(scan)} points")

print("Program finished safely.")