import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

# =========================
# PIPELINE SETUP
# =========================
pipeline = rs.pipeline()
config = rs.config()

config.enable_device_from_file(
    "/home/vinbui/vinh_ws/src/aruco_tag/data/20260414_203433.bag",
    repeat_playback=False
)

config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

# =========================
# GET CALIBRATION
# =========================
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
color_stream = profile.get_stream(rs.stream.color)

depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
extrinsics = depth_stream.get_extrinsics_to(color_stream)

fx_d, fy_d = depth_intrinsics.fx, depth_intrinsics.fy
cx_d, cy_d = depth_intrinsics.ppx, depth_intrinsics.ppy

fx_c, fy_c = color_intrinsics.fx, color_intrinsics.fy
cx_c, cy_c = color_intrinsics.ppx, color_intrinsics.ppy

R = np.array(extrinsics.rotation).reshape(3, 3)
T = np.array(extrinsics.translation).reshape(3, 1)

# Get depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth scale:", depth_scale)

# =========================
# OPEN3D VISUALIZER
# =========================
vis = o3d.visualization.Visualizer()
vis.create_window("Streaming PointCloud")

pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)

# =========================
# STREAM LOOP
# =========================
try:
    while True:
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth = np.asanyarray(depth_frame.get_data())
        color = np.asanyarray(color_frame.get_data())

        h_d, w_d = depth.shape
        h_c, w_c = color.shape[:2]

        # =========================
        # DEPTH → 3D
        # =========================
        u = np.arange(w_d)
        v = np.arange(h_d)
        uu, vv = np.meshgrid(u, v)

        Z = depth * depth_scale
        valid = Z > 0

        X = (uu - cx_d) / fx_d * Z
        Y = (vv - cy_d) / fy_d * Z

        points_depth = np.stack((X, Y, Z), axis=-1)
        points_depth = points_depth[valid]

        # =========================
        # TRANSFORM → COLOR CAM
        # =========================
        points_color = (R @ points_depth.T).T + T.T

        Xc, Yc, Zc = points_color[:, 0], points_color[:, 1], points_color[:, 2]

        valid_z = Zc > 1e-6

        u_c = np.full_like(Xc, -1, dtype=np.int32)
        v_c = np.full_like(Yc, -1, dtype=np.int32)

        u_c[valid_z] = np.round(Xc[valid_z] / Zc[valid_z] * fx_c + cx_c).astype(np.int32)
        v_c[valid_z] = np.round(Yc[valid_z] / Zc[valid_z] * fy_c + cy_c).astype(np.int32)

        valid_uv = (
            valid_z &
            (u_c >= 0) & (u_c < w_c) &
            (v_c >= 0) & (v_c < h_c)
        )

        points_final = points_depth[valid_uv]
        colors_final = color[v_c[valid_uv], u_c[valid_uv]] / 255.0

        # =========================
        # UPDATE VISUALIZATION
        # =========================
        pcd.points = o3d.utility.Vector3dVector(points_final)
        pcd.colors = o3d.utility.Vector3dVector(colors_final)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

except RuntimeError:
    print("End of bag file reached.")

finally:
    pipeline.stop()
    vis.destroy_window()