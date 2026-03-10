import pygame
import math
from rplidar import RPLidar
import platform



# --- Configuration ---
PORT_NAME = ''       # Adjust to your specific port
BAUD_RATE = 256000   # Default for A2M12
MAX_DISTANCE = 4000  # Render range in mm
WINDOW_SIZE = 800
SCALE_RATIO = WINDOW_SIZE / (2 * MAX_DISTANCE)


# Detect OS
os_name = platform.system()

# 2. Assign Port based on OS
if os_name == 'Windows':
    PORT_NAME = 'COM7'                # Windows default
elif os_name == 'Darwin':             # macOS
    PORT_NAME = '/dev/tty.SLAB_USBtoUART' # Common driver name for RPLIDAR on Mac
else:                                 # Linux / Ubuntu
    PORT_NAME = '/dev/ttyUSB0'        # Linux default

print(f"Detected {os_name}. Trying port: {PORT_NAME}")


def main():
    lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Activity 1: Raw Lidar Feed")
    
    # Pre-calculate center
    cx, cy = WINDOW_SIZE // 2, WINDOW_SIZE // 2
    
    print("Streaming data... Press ESC to stop.")
    try:
        for scan in lidar.iter_scans():
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    raise KeyboardInterrupt

            screen.fill((128, 128, 128)) # Clear screen
            
            # Draw Robot Center
            pygame.draw.line(screen, (255, 0, 0), (cx-10, cy), (cx+10, cy))
            pygame.draw.line(screen, (255, 0, 0), (cx, cy-10), (cx, cy+10))

            # Process Scan: (Quality, Angle, Distance)
            for (_, angle, distance) in scan:
                if 0 < distance < MAX_DISTANCE:
                    # Polar -> Cartesian
                    rad = math.radians(angle)


                    # Activity 1: Converting Range/Bearing Measurements
                    x_polar_to_cart = distance * math.cos(rad) # Modify this line
                    y_polar_to_cart = distance * math.sin(rad) # Modify this line                    
                    # End of Activity 1

                    x = cx + (x_polar_to_cart * SCALE_RATIO)
                    y = cy + (y_polar_to_cart * SCALE_RATIO)
                    
                    
                    #screen.set_at((int(x), int(y)), (0, 255, 0)) # Draw green pixel
                    pygame.draw.circle(screen, (255, 0, 0), (x, y), 3)

            pygame.display.flip()
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()