import numpy as np
import cv2


class Depth:
    def __init__(self, cfg, intr):
        self.cfg = cfg

        # intrinsics
        self.fx, self.fy, self.cx, self.cy, self.w, self.h = intr

        # Precompute full pixel grid once for fast projections.
        self.uu, self.vv = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self.prev_depth = None

        u1, v1, u2, v2 = self.cfg["roi"]
        self.roi = (
            max(0, min(int(u1), self.w - 1)),
            max(0, min(int(v1), self.h - 1)),
            max(1, min(int(u2), self.w)),
            max(1, min(int(v2), self.h)),
        )

    def process(self, data):

        # =========================
        # 0. GET DEPTH IMAGE (🔥 FIX QUAN TRỌNG)
        # =========================
        depth_frame = data["depth_frame"]

        depth_u16 = np.asanyarray(depth_frame.get_data())

        ksize = int(self.cfg["depth"].get("median_ksize", 0))
        if ksize and ksize > 1:
            if ksize % 2 == 0:
                ksize += 1
            depth_u16 = cv2.medianBlur(depth_u16, ksize)

        # convert mm → meter (RealSense thường mm)
        depth_scale = depth_frame.get_units()  # usually 0.001
        depth_image = depth_u16.astype(np.float32) * depth_scale

        # Keep invalid pixels at zero and smooth only valid depths.
        valid_now = depth_image > 0
        if self.prev_depth is None:
            depth_smooth = depth_image.copy()
        else:
            depth_smooth = self.prev_depth.copy()
            depth_smooth[valid_now] = 0.8 * self.prev_depth[valid_now] + 0.2 * depth_image[valid_now]
            depth_smooth[~valid_now] = 0.0

        self.prev_depth = depth_smooth

        data["depth_image"] = depth_smooth

        # =========================
        # 2. ROI FILTER (FAST)
        # =========================
        u1, v1, u2, v2 = data.get("roi_used", self.roi)

        roi_depth = depth_smooth[v1:v2, u1:u2]
        roi_u = self.uu[v1:v2, u1:u2]
        roi_v = self.vv[v1:v2, u1:u2]

        valid = np.isfinite(roi_depth) & (roi_depth > 0)
        if not np.any(valid):
            return data

        z_valid = roi_depth[valid]

        # =========================
        # 4. DEPTH FILTER (OBJECT)
        # =========================
        z_low = float(self.cfg["depth"]["z_low"])
        near_candidates = z_valid[z_valid > z_low]

        if near_candidates.size == 0:
            data["z_min"] = -1.0
            data["mask_depth"] = np.zeros((v2 - v1, u2 - u1), dtype=np.uint8)
            return data

        # Robust nearest layer estimate to reject single-pixel outliers.
        z_ref = float(np.percentile(near_candidates, 5.0))
        z_high = z_ref + float(self.cfg["depth"]["height"])

        obj_mask = valid & (roi_depth > z_low) & (roi_depth < z_high)

        if not np.any(obj_mask):
            data["z_min"] = z_ref
            data["mask_depth"] = np.zeros((v2 - v1, u2 - u1), dtype=np.uint8)
            return data

        z_obj = roi_depth[obj_mask]
        u_obj = roi_u[obj_mask].astype(np.float32)
        v_obj = roi_v[obj_mask].astype(np.float32)

        x_obj = (u_obj - self.cx) * z_obj / self.fx
        y_obj = (v_obj - self.cy) * z_obj / self.fy
        obj_pts = np.column_stack((x_obj, y_obj, z_obj))

        data["points_obj"] = obj_pts.astype(np.float32, copy=False)
        data["z_min"] = z_ref

        # =========================
        # 5. DEPTH MASK IMAGE
        # =========================
        mask_roi = np.zeros((v2 - v1, u2 - u1), dtype=np.uint8)
        v_local, u_local = np.where(obj_mask)
        mask_roi[v_local, u_local] = 255

        data["mask_depth"] = mask_roi

        # =========================
        # 6. BBOX (RED BOX)
        # =========================
        contours, _ = cv2.findContours(
            mask_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            cnt = max(contours, key=cv2.contourArea)

            if cv2.contourArea(cnt) > self.cfg["detect"]["min_area"]:
                x, y, w, h = cv2.boundingRect(cnt)

                # convert to global coords
                data["bbox_depth"] = (x + u1, y + v1, w, h)

        return data