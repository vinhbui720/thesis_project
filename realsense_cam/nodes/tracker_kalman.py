import numpy as np
import cv2
import time


class TrackerKalman:
    def __init__(self, cfg, intr):
        self.cfg = cfg

        self.fx, self.fy, self.cx, self.cy = intr

        # ===== KALMAN — tunable via config =====
        gate_cfg = cfg.get("tracking_gate", {})

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

        # Non-uniform Q: position states trust the model more,
        #               velocity states have high noise → adapt fast to real speed
        pn_pos = gate_cfg.get("process_noise_pos", 1e-4)
        pn_vel = gate_cfg.get("process_noise_vel", 5e-2)
        self.kf.processNoiseCov = np.diag(
            [pn_pos, pn_pos, pn_vel, pn_vel]).astype(np.float32)
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * gate_cfg.get("measure_noise", 1e-2)

        self.initialized   = False
        self.last_time     = None
        self.center_3d_ema = None
        self.ema_alpha     = gate_cfg.get("ema_alpha",      0.2)
        self.depth_patch_k = gate_cfg.get("depth_patch_k", 5)

        # ===== TRACKING GATE =====
        self.gate_enabled       = gate_cfg.get("enabled",    False)
        self.start_line         = gate_cfg.get("start_line", None)
        self.stop_line          = gate_cfg.get("stop_line",  None)
        self.tracking_active    = not self.gate_enabled
        self._init_vx           = gate_cfg.get("init_velocity_hint",    40.0)
        self._dampen_reverse    = gate_cfg.get("velocity_dampen_reverse", True)

        # Unit vector pointing from start_line midpoint → stop_line midpoint.
        # Used to seed velocity on entry and clamp reverse movement.
        self._dir = np.array([1.0, 0.0], dtype=np.float32)
        if self.start_line and self.stop_line:
            s = np.array([(self.start_line[0][0] + self.start_line[1][0]) / 2,
                           (self.start_line[0][1] + self.start_line[1][1]) / 2], np.float32)
            e = np.array([(self.stop_line[0][0]  + self.stop_line[1][0])  / 2,
                           (self.stop_line[0][1]  + self.stop_line[1][1])  / 2], np.float32)
            diff = e - s
            norm = np.linalg.norm(diff)
            if norm > 0:
                self._dir = diff / norm

    # =========================
    # GATE HELPERS
    # =========================
    @staticmethod
    def _side(line, px, py):
        """
        Cross product sign of point (px,py) relative to line A→B.
          > 0  →  point is to the LEFT  of A→B
          < 0  →  point is to the RIGHT of A→B
        """
        (ax, ay), (bx, by) = line
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax)

    def _in_gate(self, px, py):
        """True when centroid is past start_line (right) and before stop_line (left)."""
        past_start  = self._side(self.start_line, px, py) <= 0
        before_stop = self._side(self.stop_line,  px, py) >= 0
        return past_start and before_stop

    def _reset_kalman(self, cx=None, cy=None):
        """Reset filter state. If (cx, cy) given, seed velocity with direction hint."""
        self.center_3d_ema = None
        self.last_time     = None
        if cx is not None and cy is not None:
            vx0 = self._dir[0] * self._init_vx
            vy0 = self._dir[1] * self._init_vx
            state = np.array([[cx], [cy], [vx0], [vy0]], np.float32)
            self.kf.statePre  = state.copy()
            self.kf.statePost = state.copy()
            # Reset covariance to a modest uncertainty
            self.kf.errorCovPre  = np.eye(4, dtype=np.float32)
            self.kf.errorCovPost = np.eye(4, dtype=np.float32)
            self.initialized = True
        else:
            self.initialized = False
    # =========================
    # ROBUST DEPTH (median filter)
    # =========================
    def get_depth_robust(self, depth_img, u, v):
        k = self.depth_patch_k
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
        # ===== GATE CHECK =====
        if self.gate_enabled:
            in_zone = self._in_gate(cx, cy)
            data["tracking_active"] = in_zone

            if not in_zone:
                # Object left the zone — full reset (hand may carry it back)
                if self.tracking_active:
                    self._reset_kalman()
                self.tracking_active = False
                return data

            if not self.tracking_active:
                # Object just entered the zone — seed velocity in travel direction
                # so the filter never starts from zero or a cached reverse velocity
                self._reset_kalman(cx, cy)
                self.tracking_active = True
        else:
            data["tracking_active"] = True
            
        current_time = time.time()

        # ===== INIT (fallback for gate-disabled mode) =====
        if not self.initialized:
            vx0 = self._dir[0] * self._init_vx
            vy0 = self._dir[1] * self._init_vx
            self.kf.statePre  = np.array([[cx], [cy], [vx0], [vy0]], np.float32)
            self.kf.statePost = np.array([[cx], [cy], [vx0], [vy0]], np.float32)
            self.initialized  = True
            self.last_time    = current_time
            return data

        # _reset_kalman(cx, cy) sets initialized=True but leaves last_time=None
        # (first real frame after a seeded gate entry)
        if self.last_time is None:
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

        # ===== VELOCITY DIRECTION ENFORCEMENT =====
        # If the object is inside the gate and its Kalman velocity points backwards
        # (against start→stop direction), zero out that reverse component.
        # This prevents the hand-carry-back from poisoning the velocity estimate.
        if self._dampen_reverse and self.gate_enabled and self.tracking_active:
            dot = vx * self._dir[0] + vy * self._dir[1]
            if dot < 0:
                # subtract the reverse projection from the velocity state
                self.kf.statePost[2, 0] -= dot * self._dir[0]
                self.kf.statePost[3, 0] -= dot * self._dir[1]
                state = self.kf.statePost.flatten()
                px, py, vx, vy = state

        # ===== ROBUST DEPTH =====
        z = self.get_depth_robust(data["depth_image"], px, py)

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