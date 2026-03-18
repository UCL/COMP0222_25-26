import numpy as np
import cv2
import os

# Setup path and file discovery
# Update the folder below to view your captured images
folder = 'captures\\Session_20260314_211403'

# Check if path exists
if not os.path.exists(folder):
    print(f"Error: Folder '{folder}' does not exist.")
    exit()

# Get all file IDs by looking at the color images
all_files = sorted([f.replace('img_', '').replace('.png', '') 
                   for f in os.listdir(folder) if f.startswith('img_')])

if not all_files:
    print(f"No files found in {folder}!")
    exit()

print(f"Found {len(all_files)} captures.")
print("Controls:")
print(" [Space] - Toggle Play/Pause (Continuous Playback)")
print(" [d]     - Step forward one frame (Manual)")
print(" [a]     - Step backward one frame (Manual)")
print(" [q/ESC] - Quit viewer")

idx = 0
playing = False

while True:
    file_id = all_files[idx]
    
    # Construct paths
    color_path = os.path.join(folder, f'img_{file_id}.png')
    depth_path = os.path.join(folder, f'depth_{file_id}.npy')

    # Load and process data
    if os.path.exists(color_path) and os.path.exists(depth_path):
        color_image = cv2.imread(color_path)
        depth_raw = np.load(depth_path)

        # Colorize depth
        depth_visual = cv2.convertScaleAbs(depth_raw, alpha=0.03)
        depth_colormap = cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)

        # Match sizes
        if color_image.shape[:2] != depth_colormap.shape[:2]:
            depth_colormap = cv2.resize(depth_colormap, (color_image.shape[1], color_image.shape[0]))
        
        display_img = np.hstack((color_image, depth_colormap))
        
        # UI Overlays
        status = "PLAYING" if playing else "PAUSED"
        color = (0, 255, 0) if playing else (0, 0, 255)
        
        cv2.putText(display_img, f"STATUS: {status}", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(display_img, f"File: {file_id} ({idx+1}/{len(all_files)})", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow('D455 Data Replay', display_img)

    # Timing logic
    if playing:
        # 33ms wait is roughly 30 FPS playback
        key = cv2.waitKey(33) 
        idx = (idx + 1) % len(all_files) # Loop back to start if at end
    else:
        # Wait indefinitely
        key = cv2.waitKey(0)

    # Key Handling
    if key == 27 or key & 0xFF == ord('q'): # ESC or Q
        break
    elif key == 32: # SPACE - Toggle Play/Pause
        playing = not playing
    elif key & 0xFF == ord('d'): # Manual step forward
        playing = False
        idx = (idx + 1) % len(all_files)
    elif key & 0xFF == ord('a'): # Manual step backward
        playing = False
        idx = (idx - 1) % len(all_files)

cv2.destroyAllWindows()
print("Viewer closed.")