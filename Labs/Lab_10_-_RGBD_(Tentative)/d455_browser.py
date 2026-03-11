import numpy as np
import cv2
import os
import time

# Setup path and file discovery
folder = 'captures'
# Get all file IDs by looking at the color images and stripping 'img_' and '.png'
all_files = sorted([f.replace('img_', '').replace('.png', '') 
                   for f in os.listdir(folder) if f.startswith('img_')])

if not all_files:
    print(f"No files found in {folder}!")
    exit()

print(f"Found {len(all_files)} captures.")
print("Controls:")
print(" [Space] - View next image pair")
print(" [c]     - Continuous playback (1 second interval)")
print(" [q]     - Quit viewer")

idx = 0
continuous_mode = False

while idx < len(all_files):
    file_id = all_files[idx]
    
    # Construct paths
    color_path = os.path.join(folder, f'img_{file_id}.png')
    depth_path = os.path.join(folder, f'depth_{file_id}.npy')

    # Load data
    if os.path.exists(color_path) and os.path.exists(depth_path):
        color_image = cv2.imread(color_path)
        depth_raw = np.load(depth_path)

        # Colorize depth for visualization
        depth_visual = cv2.convertScaleAbs(depth_raw, alpha=0.03)
        depth_colormap = cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)

        # Match sizes and stack
        if color_image.shape[:2] != depth_colormap.shape[:2]:
            depth_colormap = cv2.resize(depth_colormap, (color_image.shape[1], color_image.shape[0]))
        
        display_img = np.hstack((color_image, depth_colormap))
        
        # Add overlay text
        cv2.putText(display_img, f"File: {file_id} ({idx+1}/{len(all_files)})", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('D455 Data Browser', display_img)

    # 2. Handle Logic for Space vs Continuous
    if continuous_mode:
        # Wait 1000ms (1 second). If 'q' is pressed, exit.
        key = cv2.waitKey(1000)
        idx += 1
    else:
        # Manual mode: Wait indefinitely for a key
        key = cv2.waitKey(0)

    # 3. Handle Key Presses
    if key & 0xFF == ord('q') or key == 27: # 'q' or ESC
        break
    elif key & 0xFF == ord('c'):
        print("Entering Continuous Mode...")
        continuous_mode = True
    elif key == 32: # SPACE
        continuous_mode = False
        idx += 1
        print("Next image...")

cv2.destroyAllWindows()
print("Viewer closed.")