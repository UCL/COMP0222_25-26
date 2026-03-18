import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# --- 1. Configure RealSense ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
profile = pipeline.start(config)

intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
pinhole_intrinsics = o3d.camera.PinholeCameraIntrinsic(
    intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

# --- 2. SLAM Setup (CPU Based) ---
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=0.03,
    sdf_trunc=0.06,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

align = rs.align(rs.stream.color)
current_pose = np.eye(4)
trajectory_points = [np.zeros(3)]

# --- 3. Visualization Setup ---
vis = o3d.visualization.Visualizer()
vis.create_window("UCL CS - Live SLAM", width=1280, height=720)

mesh_geo = o3d.geometry.TriangleMesh()
vis.add_geometry(mesh_geo)

# ---  4. Initialization for camera frustum
camera_viz = o3d.geometry.LineSet.create_camera_visualization(
    pinhole_intrinsics, np.eye(4), scale=0.2)
camera_viz.paint_uniform_color([1, 0, 0])
vis.add_geometry(camera_viz)

trajectory_geo = o3d.geometry.LineSet()
vis.add_geometry(trajectory_geo)

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
vis.add_geometry(axis)

first_mesh_done = False
frame_count = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
        color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

        curr_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False)

        # ICP Tracking
        curr_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(curr_rgbd, pinhole_intrinsics)
        curr_pcd = curr_pcd.voxel_down_sample(voxel_size=0.04)
        curr_pcd.estimate_normals()

        target_pcd = volume.extract_point_cloud()

        if not target_pcd.is_empty():
            target_pcd = target_pcd.voxel_down_sample(voxel_size=0.04)
            target_pcd.estimate_normals()

            result = o3d.pipelines.registration.registration_icp(
                curr_pcd, target_pcd, 
                max_correspondence_distance=0.07, 
                init=current_pose,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )

            if result.fitness > 0.4:
                current_pose = result.transformation
                volume.integrate(curr_rgbd, pinhole_intrinsics, np.linalg.inv(current_pose))
                
                # Add to trajectory
                new_pos = current_pose[:3, 3]
                if np.linalg.norm(new_pos - trajectory_points[-1]) > 0.02:
                    trajectory_points.append(new_pos)
        else:
            volume.integrate(curr_rgbd, pinhole_intrinsics, np.linalg.inv(current_pose))

        # --- Update Visuals ---
        # Camera Frustum Update
        temp_camera = o3d.geometry.LineSet.create_camera_visualization(
            pinhole_intrinsics, np.eye(4), scale=0.2)
        temp_camera.transform(current_pose)
        camera_viz.points = temp_camera.points
        vis.update_geometry(camera_viz)

        if len(trajectory_points) > 1:
            trajectory_geo.points = o3d.utility.Vector3dVector(np.array(trajectory_points))
            lines = [[i, i+1] for i in range(len(trajectory_points)-1)]
            trajectory_geo.lines = o3d.utility.Vector2iVector(np.array(lines))
            trajectory_geo.paint_uniform_color([0, 1, 0])
            vis.update_geometry(trajectory_geo)

        if frame_count % 10 == 0:
                    new_mesh = volume.extract_triangle_mesh()
                    if not new_mesh.is_empty():
                        new_mesh.compute_vertex_normals()
                        mesh_geo.vertices = new_mesh.vertices
                        mesh_geo.triangles = new_mesh.triangles
                        mesh_geo.vertex_colors = new_mesh.vertex_colors
                        mesh_geo.vertex_normals = new_mesh.vertex_normals
                        vis.update_geometry(mesh_geo)
                        
                        if not first_mesh_done:
                            # 1. Center the view on the object
                            vis.reset_view_point(True)
                            
                            # 2. Get the view controller
                            ctr = vis.get_view_control()
                            
                            # 3. Rotate the view 180 degrees around the Y-axis
                            # This flips the camera so you look at the "front" of the map
                            ctr.rotate(0.0, 1000.0) # Rotate horizontally
                            
                            first_mesh_done = True

        vis.poll_events()
        vis.update_renderer()

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    vis.destroy_window()
    pipeline.stop()