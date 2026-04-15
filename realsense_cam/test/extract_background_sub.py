import pyrealsense2 as rs
import numpy as np
import cv2

# =========================
# region TUNING PARAMETERS
# =========================

ROI_U1, ROI_V1 = 0, 0
ROI_U2, ROI_V2 = 1000, 300

DEPTH_Z_LOW = 0.2
DEPTH_HEIGHT = 0.04

BG_THRESHOLD = 25
BLUR_KERNEL = 5

KERNEL_SIZE = 5
DILATE_ITER = 2

MIN_AREA = 500
PAD_METER = 0.01

BG_PATH = "/home/vinbui/vinh_ws/src/realsense_cam/data/background.png"

# 👇 VIS CONTROL
VIS = {
    "rgb": True,
    "crop": True,
    "mask_bg": False,
    "mask_depth": False,
    "mask_final": True,
}

# =========================
# endregion
# =========================


# =========================
# region HELPER
# =========================
def show(name, img):
    if VIS.get(name, False):
        cv2.imshow(name, img)
# endregion


# =========================
# region INIT PIPELINE
# =========================
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


# =========================
# region LOAD BACKGROUND
# =========================
bg_img = cv2.imread(BG_PATH)
if bg_img is None:
    raise RuntimeError("Cannot load background image")

bg_gray = cv2.cvtColor(bg_img, cv2.COLOR_BGR2GRAY)

if bg_gray.shape != (height, width):
    print("⚠️ Background resized")
    bg_gray = cv2.resize(bg_gray, (width, height))

bg_roi = bg_gray[ROI_V1:ROI_V2, ROI_U1:ROI_U2]

kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
# endregion


# =========================
# region LOOP
# =========================
try:
    while True:

        # region FRAME
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        color = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # endregion


        # region BACKGROUND SUBTRACTION
        roi_gray = gray[ROI_V1:ROI_V2, ROI_U1:ROI_U2]

        roi_blur = cv2.GaussianBlur(roi_gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        bg_blur = cv2.GaussianBlur(bg_roi, (BLUR_KERNEL, BLUR_KERNEL), 0)

        diff = cv2.absdiff(roi_blur, bg_blur)

        _, mask_bg = cv2.threshold(diff, BG_THRESHOLD, 255, cv2.THRESH_BINARY)

        mask_bg = cv2.morphologyEx(mask_bg, cv2.MORPH_OPEN, kernel)
        mask_bg = cv2.dilate(mask_bg, kernel, iterations=DILATE_ITER)
        # endregion


        # region DEPTH PROCESSING
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        uu, vv = np.meshgrid(np.arange(width), np.arange(height))
        uu = uu.flatten()
        vv = vv.flatten()

        roi_mask = (
            (uu >= ROI_U1) & (uu <= ROI_U2) &
            (vv >= ROI_V1) & (vv <= ROI_V2)
        )

        vtx_roi = vtx[roi_mask]
        u_roi = uu[roi_mask]
        v_roi = vv[roi_mask]

        valid = ~np.isnan(vtx_roi[:, 2]) & (vtx_roi[:, 2] > 0)

        vtx_roi = vtx_roi[valid]
        u_roi = u_roi[valid]
        v_roi = v_roi[valid]

        if len(vtx_roi) == 0:
            continue

        z_min = np.min(vtx_roi[:, 2])
        z_high = z_min + DEPTH_HEIGHT

        mask_depth = (
            (vtx_roi[:, 2] > DEPTH_Z_LOW) &
            (vtx_roi[:, 2] < z_high)
        )

        u_d = u_roi[mask_depth]
        v_d = v_roi[mask_depth]
        # endregion


        # region DEPTH TO IMAGE
        mask_depth_img = np.zeros((height, width), dtype=np.uint8)
        mask_depth_img[v_d, u_d] = 255

        mask_depth_roi = mask_depth_img[ROI_V1:ROI_V2, ROI_U1:ROI_U2]
        # endregion


        # region FUSION
        mask_final = cv2.bitwise_and(mask_bg, mask_depth_roi)
        # endregion


        # region OBJECT EXTRACTION
        contours, _ = cv2.findContours(
            mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            continue

        cnt = max(contours, key=cv2.contourArea)

        if cv2.contourArea(cnt) < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)

        u_min = x + ROI_U1
        v_min = y + ROI_V1
        u_max = u_min + w
        v_max = v_min + h
        # endregion


        # region PADDING + CROP
        pad_pixel = int(fx * PAD_METER / z_min)

        u_min = max(0, u_min - pad_pixel)
        u_max = min(width, u_max + pad_pixel)
        v_min = max(0, v_min - pad_pixel)
        v_max = min(height, v_max + pad_pixel)

        crop = color[v_min:v_max, u_min:u_max]
        # endregion


        # region VISUALIZATION
        vis = color.copy()

        cv2.rectangle(vis, (ROI_U1, ROI_V1), (ROI_U2, ROI_V2), (255, 0, 0), 2)
        cv2.rectangle(vis, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)

        show("rgb", vis)
        show("crop", crop)
        show("mask_bg", mask_bg)
        show("mask_depth", mask_depth_roi)
        show("mask_final", mask_final)
        # endregion


        if cv2.waitKey(1) == 27:
            break

except RuntimeError:
    print("End of bag")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
# endregion