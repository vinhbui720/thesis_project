import cv2
import numpy as np

class Visualize:
    def __init__(self, cfg, intr):
        self.cfg = cfg
        self.fx, self.fy, self.cx, self.cy = intr
        self.frame_idx = 0

    # =========================
    # PROJECT 3D → 2D
    # =========================
    def project(self, pt):
        x, y, z = pt
        if z <= 0:
            return None

        u = int(self.fx * x / z + self.cx)
        v = int(self.fy * y / z + self.cy)

        return (u, v)

    # =========================
    # DRAW AXIS
    # =========================
    def draw_axis(self, img, pose):

        R = pose[:3, :3]
        t = pose[:3, 3]

        scale = 0.05

        origin = t
        x_axis = t + R[:, 0] * scale
        y_axis = t + R[:, 1] * scale
        z_axis = t + R[:, 2] * scale

        o = self.project(origin)
        px = self.project(x_axis)
        py = self.project(y_axis)
        pz = self.project(z_axis)

        if None in [o, px, py, pz]:
            return img

        cv2.line(img, o, px, (0, 0, 255), 3)
        cv2.line(img, o, py, (0, 255, 0), 3)
        cv2.line(img, o, pz, (255, 0, 0), 3)

        return img

    def process(self, data):
        self.frame_idx += 1
        draw_every = max(1, int(self.cfg.get("vis", {}).get("draw_every_n", 1)))
        if self.frame_idx % draw_every != 0:
            return data

        img = data["color"].copy()

        # ===== ROI =====
        if self.cfg["vis"].get("show_roi", True):
            u1, v1, u2, v2 = data.get("roi_used", self.cfg["roi"])
            cv2.rectangle(img, (u1, v1), (u2, v2), (255, 0, 0), 2)

        if self.cfg["vis"].get("show_debug_text", True):
            nz_depth = int(np.count_nonzero(data["mask_depth"])) if "mask_depth" in data else 0
            nz_final = int(np.count_nonzero(data["mask_final"])) if "mask_final" in data else 0
            icp_state = "READY" if data.get("icp_ready", False) else ("RUN" if data.get("icp_running", False) else "IDLE")
            cv2.putText(img, f"depth_px:{nz_depth} final_px:{nz_final} icp:{icp_state}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # ===== DEPTH BOX =====
        if self.cfg["vis"].get("show_depth_box", True):
            if "bbox_depth" in data:
                x, y, w, h = data["bbox_depth"]
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # ===== FINAL BOX =====
        if self.cfg["vis"].get("show_final_box", True):
            if "bbox" in data:
                x, y, w, h = data["bbox"]
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # ===== VELOCITY =====
        if self.cfg["vis"].get("show_velocity", True):
            if "center" in data and "velocity" in data:

                cx, cy = data["center"]
                vx, vy = data["velocity"]

                scale = 200

                end_x = int(cx + vx * scale)
                end_y = int(cy + vy * scale)

                cv2.arrowedLine(img, (cx, cy), (end_x, end_y),
                                (0, 255, 255), 2, tipLength=0.3)

                speed = np.linalg.norm([vx, vy])

                cv2.putText(img,
                            f"{speed:.3f} m/s",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            2)

        # ===== POSE (AUTO SELECT) =====
        pose = None

        if "pose" in data:
            pose = data["pose"]
        elif "pose_icp" in data:
            pose = data["pose_icp"]

        if pose is not None:
            img = self.draw_axis(img, pose)

        # ===== SHOW ONLY ONE WINDOW =====
        if self.cfg["vis"].get("rgb", True):
            cv2.imshow("RGB", img)

        if self.cfg["vis"].get("mask_final", False):
            if "mask_final" in data:
                cv2.imshow("MASK_FINAL", data["mask_final"])

        if self.cfg["vis"].get("mask_bg", False):
            if "mask_bg" in data:
                cv2.imshow("MASK_BG", data["mask_bg"])

        if self.cfg["vis"].get("mask_depth", False):
            if "mask_depth" in data:
                cv2.imshow("MASK_DEPTH", data["mask_depth"])

        if cv2.waitKey(1) == 27:
            exit()

        return data