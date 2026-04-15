import pyrealsense2 as rs
import numpy as np
import cv2

# ===== INIT =====
pipeline = rs.pipeline()
config = rs.config()

config.enable_device_from_file(
    "/home/vinbui/vinh_ws/src/realsense_cam/data/20260408_203426.bag",
    repeat_playback=False
)

config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

# playback
playback = profile.get_device().as_playback()
playback.set_real_time(False)

# align
align = rs.align(rs.stream.color)

# intrinsics
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intrinsics = color_stream.get_intrinsics()

fx, fy = intrinsics.fx, intrinsics.fy
cx, cy = intrinsics.ppx, intrinsics.ppy
width, height = intrinsics.width, intrinsics.height

# pointcloud
pc = rs.pointcloud()

# ROI
u1, v1 = 0, 0
u2, v2 = 1000, 300

# ===== LOOP =====
try:
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # ===== POINT CLOUD FULL =====
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        # ===== CREATE PIXEL GRID =====
        uu, vv = np.meshgrid(np.arange(width), np.arange(height))
        uu = uu.flatten()
        vv = vv.flatten()

        # ===== ROI MASK =====
        roi_mask = (uu >= u1) & (uu <= u2) & (vv >= v1) & (vv <= v2)

        # APPLY ROI FIRST
        vtx_roi = vtx[roi_mask]
        u_roi = uu[roi_mask]
        v_roi = vv[roi_mask]

        # ===== NOW FILTER INVALID =====
        valid = ~np.isnan(vtx_roi[:, 2]) & (vtx_roi[:, 2] > 0)

        vtx_roi = vtx_roi[valid]
        u_roi = u_roi[valid]
        v_roi = v_roi[valid]

        if len(vtx_roi) == 0:
            continue

        # ===== z_min =====
        z_min = np.min(vtx_roi[:, 2])
        z_max = np.max(vtx_roi[:, 2])

        # print(f"z_min: {z_min:.4f}, z_max: {z_max:.4f}")

        # ===== FILTER =====
        z_low = 0.2
        z_high = z_min + 0.04

        mask = (vtx_roi[:, 2] > z_low) & (vtx_roi[:, 2] < z_high)

        obj_points = vtx_roi[mask]
        u_obj = u_roi[mask]
        v_obj = v_roi[mask]

        if len(obj_points) == 0:
            continue

        # print("ratio:", len(obj_points) / len(vtx_roi))

        # ===== BBOX =====
        u_min, u_max = np.min(u_obj), np.max(u_obj)
        v_min, v_max = np.min(v_obj), np.max(v_obj)

        # ===== PADDING =====
        avg_depth = np.mean(obj_points[:, 2])
        pad_pixel = int(fx * 0.01 / avg_depth)

        u_min = max(0, u_min - pad_pixel)
        u_max = min(width, u_max + pad_pixel)
        v_min = max(0, v_min - pad_pixel)
        v_max = min(height, v_max + pad_pixel)

        crop = color_image[v_min:v_max, u_min:u_max]

        # ===== VISUAL =====
        vis = color_image.copy()

        cv2.rectangle(vis, (u1, v1), (u2, v2), (255, 0, 0), 2)
        cv2.rectangle(vis, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)

        idx = np.random.choice(len(u_obj), size=min(5000, len(u_obj)), replace=False)
        vis[v_obj[idx], u_obj[idx]] = [0, 0, 255]

        cv2.imshow("RGB", vis)
        cv2.imshow("CROP", crop)

        if cv2.waitKey(1) == 27:
            break

except RuntimeError:
    print("End of bag")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()