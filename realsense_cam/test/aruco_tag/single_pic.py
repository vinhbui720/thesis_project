import pyrealsense2 as rs 
import numpy as np
import cv2
import open3d as o3d
pipeline = rs.pipeline()
config = rs.config()

config.enable_device_from_file("/home/vinbui/vinh_ws/src/aruco_tag/data/20260414_203433.bag", repeat_playback =  False)
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

# --- Display the profile infor ---
# for stream in profile.get_streams():
#     print(f"Stream : {stream.stream_type()}")
#     print(f"Format : {stream.format()}")
#     print(f"FPS    : {stream.fps()}")
#     if stream.is_video_stream_profile():
#         vp = stream.as_video_stream_profile()
#         print(f"Size   : {vp.width()} x {vp.height()}")
#     print("---")

# --- Display the first frame ---
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

depth = np.asanyarray(depth_frame.get_data())
color = np.asanyarray(color_frame.get_data())
# cv2.imshow("Depth", depth)
# cv2.imshow("Color", color)
# cv2.waitKey(0)

# creatting a loop doe playing the video
# while True:
#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()

#     depth = np.asanyarray(depth_frame.get_data())
#     color = np.asanyarray(color_frame.get_data())
#     cv2.imshow("Depth", depth)
#     cv2.imshow("Color", color)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# pipeline.stop()


# --- calibration ---

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

depth_scale = depth_frame.get_units()
    
print("Depth Intrinsics:", depth_intrinsics)
print("Color Intrinsics:", color_intrinsics)
print("Extrinsics (R):\n", R)
print("Extrinsics (T):\n", T)
print("Depth Scale:", depth_scale)

h_d, w_d = depth.shape
h_c, w_c = color.shape[:2]

# tạo grid pixel
u = np.arange(w_d)
v = np.arange(h_d)
uu, vv = np.meshgrid(u, v)

Z = depth * depth_scale

# bỏ điểm invalid
valid = Z > 0

X = (uu - cx_d) / fx_d * Z
Y = (vv - cy_d) / fy_d * Z

# stack thành point cloud (depth frame)
points_depth = np.stack((X, Y, Z), axis=-1)
points_depth = points_depth[valid]

points_color = (R @ points_depth.T).T + T.T

Xc = points_color[:, 0]
Yc = points_color[:, 1]
Zc = points_color[:, 2]

# tránh chia 0
valid_z = Zc > 1e-6  # safer than > 0

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

print("Points:", points_final.shape)
print("Colors:", colors_final.shape)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_final)
pcd.colors = o3d.utility.Vector3dVector(colors_final)

o3d.visualization.draw_geometries([pcd])