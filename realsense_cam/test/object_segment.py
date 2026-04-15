
# region CONFIG
# ==== CONFIGURABLE PARAMETERS ====
CONFIG = {
    # Background Subtractor
    'history': 1000,           # Số frame dùng để học nền
    'varThreshold': 10,       # Ngưỡng phát hiện chuyển động
    'detectShadows': True,    # Có phát hiện bóng không

    # Morphology kernel size
    'kernel_size': 10,         # Kích thước kernel (hình vuông)

    # ROI
    'u1': 0,
    'v1': 0,
    'u2': 1000,
    'v2': 300,

    # Depth filter
    'z_low': 0.2,             # Ngưỡng dưới chiều sâu
    'z_high_offset': 0.1,     # Khoảng cộng thêm vào z_min

    # Contour area filter (None = bỏ qua)
    'min_contour_area': None, # Ví dụ: 500
    'max_contour_area': None, # Ví dụ: 10000
}
# endregion

import pyrealsense2 as rs
import numpy as np
import cv2

# region INIT PIPELINE
pipeline = rs.pipeline()
config = rs.config()

config.enable_device_from_file(
    "/home/vinbui/vinh_ws/src/realsense_cam/data/20260408_203426.bag",
    repeat_playback=False
)

config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

playback = profile.get_device().as_playback()
playback.set_real_time(False)

align = rs.align(rs.stream.color)

color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intrinsics = color_stream.get_intrinsics()

fx, fy = intrinsics.fx, intrinsics.fy
cx, cy = intrinsics.ppx, intrinsics.ppy
width, height = intrinsics.width, intrinsics.height

pc = rs.pointcloud()
# endregion



# region ROI SETUP
u1, v1 = CONFIG['u1'], CONFIG['v1']
u2, v2 = CONFIG['u2'], CONFIG['v2']
# endregion



# region MOTION DETECTOR
fgbg = cv2.createBackgroundSubtractorMOG2(
    history=CONFIG['history'],
    varThreshold=CONFIG['varThreshold'],
    detectShadows=CONFIG['detectShadows']
)

kernel = np.ones((CONFIG['kernel_size'], CONFIG['kernel_size']), np.uint8)
# endregion


# region MAIN LOOP
try:
    while True:

        # region FRAME ACQUIRE
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        # endregion


        # region POINT CLOUD + ROI
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        uu, vv = np.meshgrid(np.arange(width), np.arange(height))
        uu = uu.flatten()
        vv = vv.flatten()

        roi_mask = (uu >= u1) & (uu <= u2) & (vv >= v1) & (vv <= v2)

        vtx_roi = vtx[roi_mask]
        u_roi = uu[roi_mask]
        v_roi = vv[roi_mask]

        valid = ~np.isnan(vtx_roi[:, 2]) & (vtx_roi[:, 2] > 0)

        vtx_roi = vtx_roi[valid]
        u_roi = u_roi[valid]
        v_roi = v_roi[valid]

        if len(vtx_roi) == 0:
            continue
        # endregion


        # region DEPTH FILTER (OBJECT HEIGHT)
        z_min = np.min(vtx_roi[:, 2])

        z_low = CONFIG['z_low']
        z_high = z_min + CONFIG['z_high_offset']

        mask_depth = (vtx_roi[:, 2] > z_low) & (vtx_roi[:, 2] < z_high)

        u_depth = u_roi[mask_depth]
        v_depth = v_roi[mask_depth]
        # endregion


        # region MOTION SEGMENTATION
        roi_img = color_image[v1:v2, u1:u2]

        fgmask = fgbg.apply(roi_img)

        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(
            fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        motion_mask = np.zeros_like(fgmask)

        # Lọc contour theo diện tích nếu có config
        filtered_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            min_area = CONFIG['min_contour_area']
            max_area = CONFIG['max_contour_area']
            if (min_area is not None and area < min_area):
                continue
            if (max_area is not None and area > max_area):
                continue
            filtered_contours.append(cnt)

        if filtered_contours:
            cnt = max(filtered_contours, key=cv2.contourArea)
            cv2.drawContours(motion_mask, [cnt], -1, 255, -1)
        # endregion


        # region COMBINE DEPTH + MOTION
        # convert motion mask to global coords
        motion_full = np.zeros((height, width), dtype=np.uint8)
        motion_full[v1:v2, u1:u2] = motion_mask

        # keep only points that also move
        motion_filter = motion_full[v_depth, u_depth] > 0

        u_obj = u_depth[motion_filter]
        v_obj = v_depth[motion_filter]

        if len(u_obj) < 50:
            continue
        # endregion


        # region BOUNDING BOX
        u_min, u_max = np.min(u_obj), np.max(u_obj)
        v_min, v_max = np.min(v_obj), np.max(v_obj)

        avg_depth = z_min
        pad_pixel = int(fx * 0.01 / avg_depth)

        u_min = max(0, u_min - pad_pixel)
        u_max = min(width, u_max + pad_pixel)
        v_min = max(0, v_min - pad_pixel)
        v_max = min(height, v_max + pad_pixel)

        crop = color_image[v_min:v_max, u_min:u_max]
        # endregion


        # region VISUALIZATION
        vis = color_image.copy()

        cv2.rectangle(vis, (u1, v1), (u2, v2), (255, 0, 0), 2)
        cv2.rectangle(vis, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)

        # draw points
        idx = np.random.choice(len(u_obj), size=min(3000, len(u_obj)), replace=False)
        vis[v_obj[idx], u_obj[idx]] = [0, 0, 255]

        cv2.imshow("RGB", vis)
        cv2.imshow("CROP", crop)
        cv2.imshow("MOTION", fgmask)
        # endregion


        if cv2.waitKey(1) == 27:
            break

except RuntimeError:
    print("End of bag")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
# endregion