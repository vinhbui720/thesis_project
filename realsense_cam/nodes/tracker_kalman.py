import numpy as np
import cv2
import time


class TrackerKalman:
    def __init__(self, cfg, intr):
        self.cfg = cfg

        self.fx, self.fy, self.cx, self.cy = intr

        # ===== KALMAN =====
        self.kf = cv2.KalmanFilter(4, 2)

        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-3
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-2

        self.initialized = False
        self.last_time = None
        self.center_3d_ema = None
        self.ema_alpha = 0.75

    # =========================
    # ROBUST DEPTH (median filter)
    # =========================
    def get_depth_robust(self, depth_img, u, v, k=5):
        h, w = depth_img.shape

        u = int(np.clip(u, k, w - k - 1))
        v = int(np.clip(v, k, h - k - 1))

        patch = depth_img[v - k:v + k + 1, u - k:u + k + 1]

        patch = patch[patch > 0]

        if len(patch) == 0:
            return None

        return np.median(patch)

    # =========================
    # PIXEL → 3D
    # =========================
    def pixel_to_3d(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z])

    # =========================
    # MAIN
    # =========================
    def process(self, data):

        if "bbox" not in data or "depth_image" not in data:
            return data

        x, y, w, h = data["bbox"]

        cx = np.float32(x + w / 2)
        cy = np.float32(y + h / 2)

        current_time = time.time()

        # ===== INIT =====
        if not self.initialized:
            self.kf.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
            self.initialized = True
            self.last_time = current_time
            return data

        # ===== dt =====
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 1e-3

        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

        # ===== PREDICT =====
        self.kf.predict()

        # ===== UPDATE =====
        measurement = np.array([[cx], [cy]], np.float32)
        self.kf.correct(measurement)

        state = self.kf.statePost.flatten()

        px, py, vx, vy = state

        # ===== ROBUST DEPTH =====
        depth_img = data["depth_image"]

        z = self.get_depth_robust(depth_img, px, py)

        if z is None:
            return data

        # ===== 3D CENTER =====
        center_3d = self.pixel_to_3d(px, py, z)
        if self.center_3d_ema is None:
            self.center_3d_ema = center_3d
        else:
            self.center_3d_ema = self.ema_alpha * self.center_3d_ema + (1.0 - self.ema_alpha) * center_3d

        data["center_3d"] = self.center_3d_ema

        # ===== VELOCITY (m/s) =====
        vx_m = (vx * z) / self.fx
        vy_m = (vy * z) / self.fy

        data["velocity"] = (vx_m, vy_m)
        data["center"] = (int(px), int(py))

        return data