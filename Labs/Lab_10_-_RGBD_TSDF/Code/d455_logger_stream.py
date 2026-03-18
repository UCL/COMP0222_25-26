import pyrealsense2 as rs
import numpy as np
import cv2
import os
import csv
from datetime import datetime

# Base directory for all recordings
base_path = 'captures'
if not os.path.exists(base_path):
    os.makedirs(base_path)

# Configure D455
# FPS is 30, change it below if you want a lower FPS. 15 FPS is suitable.
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Setup Align Object
align = rs.align(rs.stream.color)

recording = False
current_session_folder = ""

print("Streaming...")
print("Press 'SPACE' to START/STOP continuous recording.")
print("Press 'ESC' to exit.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth_raw = np.asanyarray(depth_frame.get_data())
        color_raw = np.asanyarray(color_frame.get_data())

        # Logic for continuous capture
        if recording:
            timestamp = datetime.now().strftime("%H%M%S_%f")
            
            # Save files into the session subfolder
            cv2.imwrite(os.path.join(current_session_folder, f'img_{timestamp}.png'), color_raw)
            np.save(os.path.join(current_session_folder, f'depth_{timestamp}.npy'), depth_raw)
            
            # Metadata Log
            meta_path = os.path.join(current_session_folder, f'meta_{timestamp}.csv')
            with open(meta_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Field', 'Value'])
                writer.writerow(['Frame_Number', color_frame.get_frame_number()])
                writer.writerow(['System_Timestamp', timestamp])
                
                # Check hardware metadata
                for opt in [rs.frame_metadata_value.actual_exposure, 
                            rs.frame_metadata_value.gain_level]:
                    if color_frame.supports_frame_metadata(opt):
                        writer.writerow([str(opt).split('.')[-1], color_frame.get_frame_metadata(opt)])

        # Display Visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)
        
        # Add a "REC" indicator to the screen
        display_img = np.hstack((color_raw, depth_colormap))
        if recording:
            cv2.circle(display_img, (30, 30), 15, (0, 0, 255), -1)
            cv2.putText(display_img, "REC", (60, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('D455 Live Feed', display_img)

        key = cv2.waitKey(1)
        
        if key == 32:  # SPACE bar
            recording = not recording # Toggle
            if recording:
                # Create a new subfolder for this specific recording session
                session_name = datetime.now().strftime("Session_%Y%m%d_%H%M%S")
                current_session_folder = os.path.join(base_path, session_name)
                os.makedirs(current_session_folder)
                print(f"Started Recording: {session_name}")
            else:
                print("Recording Stopped.")

        elif key == 27:  # ESC
            print("Exiting...")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()