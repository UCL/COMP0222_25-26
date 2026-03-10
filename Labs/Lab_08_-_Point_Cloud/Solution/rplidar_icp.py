import pygame
import numpy as np
from rplidar import RPLidar
from sklearn.neighbors import NearestNeighbors
import math
import platform

# ==========================================
# PART 0: Setup
# ==========================================
# --- CONFIGURATION ---
PORT_NAME = ''         
BAUD_RATE = 256000
MAX_RANGE_MM = 4000.0      
ICP_MAX_ITER = 10          
CORRESPONDENCE_THRESH = 0.5 
KEYFRAME_DIST_THRESH = 0.2 
KEYFRAME_ANGLE_THRESH = 0.2
LOCAL_MAP_SIZE = 20        

# --- BLIND SPOT FILTER (For 270 degree swath) ---
# We ignore the sector where you are standing.
# Assuming 0 is Front, 180 is Back.
# We cut out 90 degrees in the back (180 +/- 45).
BLIND_SPOT_MIN = 135.0  # Start ignoring here
BLIND_SPOT_MAX = 225.0  # Stop ignoring here

# --- PYGAME VIEW SETTINGS ---
WINDOW_SIZE = 800
METERS_TO_PIXELS = 100.0   
view_offset_x = WINDOW_SIZE // 2
view_offset_y = WINDOW_SIZE // 2

# --- Detect OS ---
os_name = platform.system()

# --- Assign Port based on OS ---
if os_name == 'Windows':
    PORT_NAME = 'COM8'                # Windows default
elif os_name == 'Darwin':             # macOS
    PORT_NAME = '/dev/tty.SLAB_USBtoUART' # Common driver name for RPLIDAR on Mac
else:                                 # Linux / Ubuntu
    PORT_NAME = '/dev/ttyUSB0'        # Linux default

print(f"Detected {os_name}. Trying port: {PORT_NAME}")


# ==========================================
# PART 1: MATH & ICP HELPERS
# ==========================================

def estimate_normals_pca(points, k=5):
    if len(points) < k + 1:
        return np.zeros((len(points), 2))
    
    neigh = NearestNeighbors(n_neighbors=k+1)
    neigh.fit(points)
    _, indices_all = neigh.kneighbors(points)
    
    normals = np.zeros((points.shape[0], 2))
    
    for i in range(points.shape[0]):
        neighbor_points = points[indices_all[i]]
        centered = neighbor_points - np.mean(neighbor_points, axis=0)
        cov = np.dot(centered.T, centered) / k
        eig_vals, eig_vecs = np.linalg.eigh(cov)
        normal = eig_vecs[:, 0]
        if np.dot(normal, points[i]) < 0:
            normal = -normal
        normals[i] = normal
    return normals

def solve_point_to_plane(src, dst, dst_normals):
    A = []
    b = []
    for i in range(len(src)):
        s = src[i]
        d = dst[i]
        n = dst_normals[i]
        # Activity 2: Calculate cross term
        cross_term = s[0] * n[1] - s[1] * n[0]
        # End of Activity 2
        A.append([cross_term, n[0], n[1]])
        b.append(np.dot(d - s, n))

    if not A: return np.identity(3)

    A = np.array(A)
    b = np.array(b)
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    
    c, s = np.cos(x[0]), np.sin(x[0])
    R = np.array([[c, -s], [s, c]])
    T = np.identity(3)
    T[:2, :2] = R
    T[:2, 2] = [x[1], x[2]]
    return T

def icp_scan_to_map(src_points, map_points, map_normals, init_pose_guess):
    m = src_points.shape[1]
    src_h = np.ones((m+1, src_points.shape[0])) 
    src_h[:m,:] = np.copy(src_points.T)
    current_global_pose = np.copy(init_pose_guess)
    
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(map_points)
    
    for i in range(ICP_MAX_ITER):
        src_global_h = np.dot(current_global_pose, src_h)
        src_global = src_global_h[:2, :].T
        
        distances, indices = neigh.kneighbors(src_global, return_distance=True)
        distances = distances.ravel()
        indices = indices.ravel()
        
        mask = distances < CORRESPONDENCE_THRESH
        if np.sum(mask) < 10: break
        
        src_valid = src_global[mask]
        dst_valid = map_points[indices[mask]]
        normals_valid = map_normals[indices[mask]]
        
        T_delta = solve_point_to_plane(src_valid, dst_valid, normals_valid)
        current_global_pose = np.dot(T_delta, current_global_pose)
        
        if np.linalg.norm(T_delta[:2, 2]) < 0.001 and abs(np.arctan2(T_delta[1,0], T_delta[0,0])) < 0.001:
            break
    return current_global_pose

# ==========================================
# PART 2: DATA CONVERSION (WITH FILTER)
# ==========================================

def process_scan(scan_data):
    """ Converts [(qual, angle, dist)...] to Numpy XY (meters) with 270 deg swath """
    raw = np.array(scan_data)
    if len(raw) == 0: return None
    
    distances = raw[:, 2]
    angles = raw[:, 1]
    
    # --- FILTERS ---
    # 1. Distance Filter (Min 10mm, Max 4000mm)
    dist_mask = (distances > 10) & (distances < MAX_RANGE_MM)
    
    # 2. Angle Filter (Exclude Blind Spot)
    # We keep points that are LESS than Min OR GREATER than Max
    # (effectively cutting out the middle chunk)
    angle_mask = (angles < BLIND_SPOT_MIN) | (angles > BLIND_SPOT_MAX)
    
    # Combine masks
    mask = dist_mask & angle_mask
    
    if np.sum(mask) < 10: return None
    
    # Convert to XY
    angles_rad = np.radians(raw[mask, 1])
    dists_m = raw[mask, 2] / 1000.0
    
    x = dists_m * np.cos(angles_rad)
    y = dists_m * np.sin(angles_rad)
    
    return np.column_stack((x, y))

# ==========================================
# PART 3: MAIN LOOP
# ==========================================

def world_to_screen(point, offset_x, offset_y, scale):
    sx = int(offset_x + point[0] * scale)
    sy = int(offset_y - point[1] * scale)
    return (sx, sy)

def main():
    global METERS_TO_PIXELS, view_offset_x, view_offset_y

    lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)
    
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("RPLIDAR SLAM (270 Deg Swath)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)

    current_pose = np.identity(3)
    last_keyframe_pose = np.identity(3)
    
    keyframe_buffer = []      
    global_map_points = []    
    trajectory = [[0,0]]      
    
    first_scan_done = False
    status = "Starting"

    print("Starting SLAM... Blind spot set to [{}, {}] degrees.".format(BLIND_SPOT_MIN, BLIND_SPOT_MAX))

    try:
        iterator = lidar.iter_scans()
        
        for scan in iterator:
            # --- EVENT HANDLING ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT: raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE: raise KeyboardInterrupt

                    # --- RESET FUNCTIONALITY ---
                    if event.key == pygame.K_r:
                        print("Resetting SLAM...")
                        current_pose = np.identity(3)
                        last_keyframe_pose = np.identity(3)
                        keyframe_buffer = []      
                        global_map_points = []    
                        trajectory = [[0,0]]  
                        first_scan_done = False
                        status = "Reset"
                        view_offset_x, view_offset_y = WINDOW_SIZE//2, WINDOW_SIZE//2

                    # Camera Controls                   
                    if event.key == pygame.K_SPACE: 
                        view_offset_x, view_offset_y = WINDOW_SIZE//2, WINDOW_SIZE//2
            
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]: view_offset_y += 5
            if keys[pygame.K_s]: view_offset_y -= 5
            if keys[pygame.K_a]: view_offset_x += 5
            if keys[pygame.K_d]: view_offset_x -= 5
            if keys[pygame.K_q]: METERS_TO_PIXELS *= 1.05
            if keys[pygame.K_e]: METERS_TO_PIXELS *= 0.95

            current_scan_xy = process_scan(scan)
            if current_scan_xy is None: continue

            if not first_scan_done:
                normals = estimate_normals_pca(current_scan_xy)
                keyframe_buffer.append((current_scan_xy, normals))
                global_map_points.append(current_scan_xy)
                first_scan_done = True
                status = "Initializing"
            else:
                active_points = np.vstack([k[0] for k in keyframe_buffer])
                active_normals = np.vstack([k[1] for k in keyframe_buffer])
                
                new_pose = icp_scan_to_map(current_scan_xy, active_points, active_normals, current_pose)
                current_pose = new_pose
                cx, cy = current_pose[0,2], current_pose[1,2]
                trajectory.append([cx, cy])

                delta_T = np.dot(np.linalg.inv(last_keyframe_pose), current_pose)
                dx, dy = delta_T[0,2], delta_T[1,2]
                dtheta = np.arctan2(delta_T[1,0], delta_T[0,0])
                dist_moved = np.sqrt(dx**2 + dy**2)

                if dist_moved > KEYFRAME_DIST_THRESH or abs(dtheta) > KEYFRAME_ANGLE_THRESH:
                    status = "Keyframe Added"
                    curr_h = np.ones((3, current_scan_xy.shape[0]))
                    curr_h[:2,:] = current_scan_xy.T
                    curr_global = np.dot(current_pose, curr_h)[:2,:].T
                    
                    curr_normals = estimate_normals_pca(curr_global)
                    keyframe_buffer.append((curr_global, curr_normals))
                    global_map_points.append(curr_global)
                    
                    last_keyframe_pose = np.copy(current_pose)
                    if len(keyframe_buffer) > LOCAL_MAP_SIZE:
                        keyframe_buffer.pop(0)
                else:
                    status = "Tracking"

            screen.fill((128, 128, 128))
            
            # Draw Map (Grey Circles)            
            if len(global_map_points) > 0:
                all_map_pts = np.vstack(global_map_points)
                # Optimization: Only draw every 5th point to save CPU
                for pt in all_map_pts[::5]: 
                    px, py = world_to_screen(pt, view_offset_x, view_offset_y, METERS_TO_PIXELS)
                    if 0 <= px < WINDOW_SIZE and 0 <= py < WINDOW_SIZE:
                        screen.set_at((px, py), (0, 0, 0))
                        #pygame.draw.circle(screen, (100, 100, 100), (px, py), 1)

            # Draw Current Scan (Red Circles)
            curr_h = np.ones((3, current_scan_xy.shape[0]))
            curr_h[:2,:] = current_scan_xy.T
            viz_scan = np.dot(current_pose, curr_h)[:2,:].T
            
            for pt in viz_scan:
                px, py = world_to_screen(pt, view_offset_x, view_offset_y, METERS_TO_PIXELS)
                if 0 <= px < WINDOW_SIZE and 0 <= py < WINDOW_SIZE:
                    #screen.set_at((px, py), (255, 0, 0))
                    pygame.draw.circle(screen, (255, 0, 0), (px, py), 3)


            # C. Draw Robot Trajectory (Blue)
            if len(trajectory) > 1:
                traj_pts = [world_to_screen(p, view_offset_x, view_offset_y, METERS_TO_PIXELS) for p in trajectory]
                pygame.draw.lines(screen, (0, 0, 255), False, traj_pts, 2)

            rx, ry = world_to_screen([current_pose[0,2], current_pose[1,2]], view_offset_x, view_offset_y, METERS_TO_PIXELS)
            pygame.draw.circle(screen, (0, 255, 0), (rx, ry), 5)
            
            # Draw Blind Spot cone (Visual Aid)
            # This helps you know where NOT to stand
            bs_min_rad = math.radians(BLIND_SPOT_MIN)
            bs_max_rad = math.radians(BLIND_SPOT_MAX)
            # Transform local angles to global pose for drawing would be complex, 
            # so we just draw lines relative to robot position on screen
            # (Approximation for visual feedback)
            
            info_text = f"{status} | Pts: {len(current_scan_xy)} | Press R to Reset"
            screen.blit(font.render(info_text, True, (255, 255, 255)), (10, 10))

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