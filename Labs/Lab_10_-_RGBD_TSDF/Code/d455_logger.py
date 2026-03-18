import pyrealsense2 as rs
import numpy as np
import cv2
import os
import csv
from datetime import datetime

# Setup folders
if not os.path.exists('captures'):
    os.makedirs('captures')

# Configure D455
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

print("Streaming... Press 'SPACE' to capture, 'ESC' to stop.")

# 1. Setup the Align Object
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        frames = pipeline.wait_for_frames()
        
        # Align the depth frame to the color frame
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Display (Colorized Depth + RGB)
        depth_raw = np.asanyarray(depth_frame.get_data())
        color_raw = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('D455 Capture', np.hstack((color_raw, depth_colormap)))

        key = cv2.waitKey(1)
        if key == 32:  # SPACE
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            
            # 1. Save RGB
            cv2.imwrite(f'captures/img_{timestamp}.png', color_raw)
            
            # 2. Save Raw Depth
            np.save(f'captures/depth_{timestamp}.npy', depth_raw)
            
            # 3. Log Metadata to CSV
            meta_path = f'captures/meta_{timestamp}.csv'
            with open(meta_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Field', 'Value'])
                
                # Standard Metadata
                writer.writerow(['Timestamp_System', timestamp])
                writer.writerow(['Frame_Number', color_frame.get_frame_number()])
                writer.writerow(['Backend_Timestamp', color_frame.get_timestamp()])
                
                # Hardware Specific Metadata (Exposure, Gain, etc.)
                for opt in [rs.frame_metadata_value.actual_exposure, 
                            rs.frame_metadata_value.gain_level,
                            rs.frame_metadata_value.frame_timestamp]:
                    if color_frame.supports_frame_metadata(opt):
                        writer.writerow([str(opt).split('.')[-1], color_frame.get_frame_metadata(opt)])

            print(f"Captured: {timestamp}")

        elif key == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()