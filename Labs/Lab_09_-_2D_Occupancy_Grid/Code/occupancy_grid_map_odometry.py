import os
import sys
import math
import pygame
import numpy as np
from rplidar import RPLidar, RPLidarException
import platform

# --- Configuration ---
PORT_NAME = ''
BAUD_RATE = 256000
TIMEOUT = 1

# Display & Map Settings
# We use a square map. 800px * 20mm = 16 meters wide/tall.
WINDOW_SIZE = (800, 800)
MAP_DIM = 800
CELL_SIZE_MM = 20
LIDAR_RADIUS_MM = 4000

# SLAM / Probability Constants
CONFIDENCE_FREE = (10, 10, 10)
CONFIDENCE_OCCUPIED = (50, 50, 50)

# Blind Spot (User Location) - 90 degrees behind sensor
CUT_ANGLE_MIN = 135.0
CUT_ANGLE_MAX = 225.0

# Detect OS
os_name = platform.system()

# 2. Assign Port based on OS
if os_name == 'Windows':
    PORT_NAME = 'COM8'                # Windows default
elif os_name == 'Darwin':             # macOS
    PORT_NAME = '/dev/tty.SLAB_USBtoUART' # Common driver name for RPLIDAR on Mac
else:                                 # Linux / Ubuntu
    PORT_NAME = '/dev/ttyUSB0'        # Linux default

print(f"Detected {os_name}. Trying port: {PORT_NAME}")


class PoseEstimator:
    def __init__(self, map_dim, cell_size_mm):
        self.map_w = map_dim
        self.map_h = map_dim
        self.cell_size = cell_size_mm
        
        self.reset()

    def reset(self):
        """Resets the robot to the center of the map."""
        
        # Start in center of the grid (Pixel Coordinates * Scale)
        self.x = (self.map_w * self.cell_size) / 2
        self.y = (self.map_h * self.cell_size) / 2
        self.theta = 0.0        

    def get_pose(self):
        return self.x, self.y, self.theta

    def optimize_pose(self, scan_points, grid_map, iterations=10):
        if len(scan_points) == 0:
            return

        scan_arr = np.array(scan_points)
        angles = scan_arr[:, 0]
        dists = scan_arr[:, 1]
        
        local_x = dists * np.cos(angles)
        local_y = dists * np.sin(angles)

        step_xy = 30
        step_th = np.radians(0.5)

        for _ in range(iterations):
            best_score = -float('inf')
            best_pose = (self.x, self.y, self.theta)
            found_better = False

            search_range_xy = [-step_xy, 0, step_xy]
            search_range_th = [-step_th, 0, step_th]

            for dth in search_range_th:
                test_th = self.theta + dth
                cos_th = np.cos(test_th)
                sin_th = np.sin(test_th)

                for dx in search_range_xy:
                    for dy in search_range_xy:
                        
                        test_x = self.x + dx
                        test_y = self.y + dy

                        # Fast Transform
                        global_x = (local_x * cos_th - local_y * sin_th) + test_x
                        global_y = (local_x * sin_th + local_y * cos_th) + test_y

                        grid_x = (global_x / self.cell_size).astype(int)
                        grid_y = (global_y / self.cell_size).astype(int)

                        # Check bounds
                        valid_mask = (grid_x >= 0) & (grid_x < self.map_w) & \
                                     (grid_y >= 0) & (grid_y < self.map_h)
                        
                        if np.sum(valid_mask) > 5:
                            valid_gx = grid_x[valid_mask]
                            valid_gy = grid_y[valid_mask]
                            score = np.sum(grid_map[valid_gx, valid_gy])
                            
                            if score > best_score:
                                best_score = score
                                best_pose = (test_x, test_y, test_th)
                                found_better = True
            
            if found_better:
                self.x, self.y, self.theta = best_pose
            else:
                break

def run_fixed_map_slam():
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Lidar SLAM: Fixed Map + Trajectory")

    # 1. VISUAL SURFACES
    view_surface = pygame.Surface((MAP_DIM, MAP_DIM))
    view_surface.fill((128, 128, 128))

    # 2. MATH GRID
    occupancy_grid = np.full((MAP_DIM, MAP_DIM), 0.5, dtype=np.float32)

    # 3. TRAJECTORY STORAGE
    trajectory_points = []

    estimator = PoseEstimator(MAP_DIM, CELL_SIZE_MM)
    lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE, timeout=TIMEOUT)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)

    print("Starting... Map is FIXED. Robot will move.")

    try:
        for scan in lidar.iter_scans(max_buf_meas=10000):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE: raise KeyboardInterrupt

                    if event.key == pygame.K_r:
                        print("RESETTING MAP...")
                        # 1. Clear Visuals
                        view_surface.fill((128, 128, 128))
                        # 2. Reset Math Grid to 'Unknown' (0.5)
                        occupancy_grid.fill(0.5)
                        # 3. Clear Trajectory
                        trajectory_points.clear()
                        # 4. Reset Robot Position
                        estimator.reset()



            # --- A. Filter Data ---
            valid_points = []
            for (_, angle, distance) in scan:
                if (angle >= CUT_ANGLE_MIN) and (angle <= CUT_ANGLE_MAX):
                    continue
                if distance > 0:
                    valid_points.append((math.radians(angle), distance))
            
            if not valid_points:
                continue

            # --- B. Pose Estimation ---
            estimator.optimize_pose(valid_points, occupancy_grid, iterations=10)
            curr_x, curr_y, curr_th = estimator.get_pose()

            # --- C. Update Trajectory ---
            # Calculate pixel position of robot
            rob_px = int(curr_x / CELL_SIZE_MM)
            rob_py = int(curr_y / CELL_SIZE_MM)
            
            # Add to path (optimization: only add if moved slightly to save memory, 
            # but for this demo adding every frame is fine)
            trajectory_points.append((rob_px, rob_py))

            # --- D. Update Map ---
            flash_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            flash_surface.fill((0,0,0))
            hits_surface = pygame.Surface((MAP_DIM, MAP_DIM))
            hits_surface.fill((0,0,0))

            math_hits_x = []
            math_hits_y = []
            
            cos_th = math.cos(curr_th)
            sin_th = math.sin(curr_th)

            for (angle_rad, dist) in valid_points:
                lx = dist * math.cos(angle_rad)
                ly = dist * math.sin(angle_rad)
                
                gx_mm = (lx * cos_th - ly * sin_th) + curr_x
                gy_mm = (lx * sin_th + ly * cos_th) + curr_y
                
                px = int(gx_mm / CELL_SIZE_MM)
                py = int(gy_mm / CELL_SIZE_MM)

                if 0 <= px < MAP_DIM and 0 <= py < MAP_DIM:
                    # Draw visual rays
                    pygame.draw.line(flash_surface, CONFIDENCE_FREE, (rob_px, rob_py), (px, py), 2)
                    pygame.draw.circle(hits_surface, CONFIDENCE_OCCUPIED, (px, py), 2)
                    
                    math_hits_x.append(px)
                    math_hits_y.append(py)

            # Blend
            view_surface.blit(flash_surface, (0,0), special_flags=pygame.BLEND_ADD)
            view_surface.blit(hits_surface, (0,0), special_flags=pygame.BLEND_SUB)

            # Update Math
            if math_hits_x:
                rows = np.array(math_hits_x)
                cols = np.array(math_hits_y)
                occupancy_grid[rows, cols] = np.minimum(1.0, occupancy_grid[rows, cols] + 0.1)

            # --- E. Render Fixed Map & Moving Robot ---
            
            screen.fill((40, 40, 40))
            
            # 1. Draw the Map (Fixed at 0,0)
            screen.blit(view_surface, (0, 0))
            
            # 2. Draw Trajectory
            # We need at least 2 points to draw a line
            if len(trajectory_points) > 1:
                # Draw Blue Line (RGB: 0, 100, 255), width 2
                pygame.draw.lines(screen, (0, 191, 255), False, trajectory_points, 2)
            
            # 3. Draw Robot at calculated position
            pygame.draw.circle(screen, (255, 0, 0), (rob_px, rob_py), 8)
            
            # Draw Heading Indicator
            end_x = rob_px + 20 * math.cos(curr_th)
            end_y = rob_py + 20 * math.sin(curr_th)
            pygame.draw.line(screen, (255, 0, 0), (rob_px, rob_py), (end_x, end_y), 3)

            # Draw Blind Spot Wedge (attached to robot)
            blind_p1_x = rob_px + 30 * math.cos(curr_th + math.radians(CUT_ANGLE_MIN))
            blind_p1_y = rob_py + 30 * math.sin(curr_th + math.radians(CUT_ANGLE_MIN))
            blind_p2_x = rob_px + 30 * math.cos(curr_th + math.radians(CUT_ANGLE_MAX))
            blind_p2_y = rob_py + 30 * math.sin(curr_th + math.radians(CUT_ANGLE_MAX))
            pygame.draw.line(screen, (255, 255, 0), (rob_px, rob_py), (blind_p1_x, blind_p1_y), 1)
            pygame.draw.line(screen, (255, 255, 0), (rob_px, rob_py), (blind_p2_x, blind_p2_y), 1)

            # Stats
            fps = int(clock.get_fps())
            info = f"FPS: {fps} | Pose: {curr_x:.0f}, {curr_y:.0f}"
            text = font.render(info, True, (0, 255, 0))
            screen.blit(text, (10, 10))

            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        pygame.quit()

if __name__ == '__main__':
    run_fixed_map_slam()