import numpy as np
import cv2
import time

class Tracker:
    def __init__(self, cfg):
        self.cfg = cfg

        self.prev_center = None
        self.prev_time = None

        self.velocity = (0, 0)

    def process(self, data):

        if "bbox" not in data:
            return data

        x, y, w, h = data["bbox"]

        # ===== CENTROID =====
        cx = int(x + w / 2)
        cy = int(y + h / 2)

        current_center = np.array([cx, cy])

        # ===== TIME =====
        current_time = time.time()

        # ===== VELOCITY =====
        if self.prev_center is not None:
            dt = current_time - self.prev_time

            if dt > 0:
                vel = (current_center - self.prev_center) / dt
                self.velocity = vel

        # ===== SAVE =====
        self.prev_center = current_center
        self.prev_time = current_time

        data["center"] = (cx, cy)
        data["velocity"] = self.velocity

        return data