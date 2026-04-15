import numpy as np

class PoseFusion:
    def __init__(self, cfg=None):
        self.cfg = cfg or {}

    def process(self, data):

        if "pose_icp" not in data:
            return data

        if "center_3d" not in data:
            return data

        T = data["pose_icp"].copy()

        if self.cfg.get("init_pose", {}).get("invert_rotation", True):
            T[:3, :3] = T[:3, :3].T

        T[:3, 3] = data["center_3d"]

        data["pose"] = T

        return data