import numpy as np
import open3d as o3d
import threading
from concurrent.futures import ThreadPoolExecutor


class PoseInitICPAsync:
    def __init__(self, cfg, model_path, intr):
        self.cfg = cfg
        self.fx, self.fy, self.cx, self.cy = intr

        gpu_cfg = cfg.get("gpu", {})
        want_cuda = gpu_cfg.get("enable", True) and gpu_cfg.get("open3d_cuda", True)
        self.device = o3d.core.Device("CUDA:0") if want_cuda and o3d.core.cuda.is_available() else o3d.core.Device("CPU:0")

        model_legacy = o3d.io.read_point_cloud(model_path)
        model_legacy = model_legacy.voxel_down_sample(cfg["init_pose"]["voxel_size"])
        model_pts = np.asarray(model_legacy.points, dtype=np.float32)
        self.model = o3d.t.geometry.PointCloud(
            o3d.core.Tensor(model_pts, dtype=o3d.core.Dtype.Float32, device=self.device)
        )

        self.pose = np.eye(4)
        self.ready = False
        self.running = False
        self.fitness = 0.0
        self.inlier_rmse = 0.0
        self.icp_future = None

        self.lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=1)

    def _masked_points_from_depth(self, depth_image, mask_roi, color, roi):
        u1, v1, u2, v2 = roi

        if mask_roi.shape[0] != (v2 - v1) or mask_roi.shape[1] != (u2 - u1):
            return None, None

        ys, xs = np.where(mask_roi > 0)
        if len(xs) < self.cfg["init_pose"]["min_points"]:
            return None, None

        uu = xs + u1
        vv = ys + v1

        z = depth_image[vv, uu]
        valid = np.isfinite(z) & (z > 0)

        if np.count_nonzero(valid) < self.cfg["init_pose"]["min_points"]:
            return None, None

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        max_pts = int(self.cfg["init_pose"].get("target_points", 5000))
        if len(z) > max_pts:
            idx = np.random.choice(len(z), max_pts, replace=False)
            uu = uu[idx]
            vv = vv[idx]
            z = z[idx]

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy
        pts = np.column_stack((x, y, z)).astype(np.float32)
        colors = color[vv.astype(np.int32), uu.astype(np.int32)].astype(np.uint8)

        return pts, colors

    def _to_tensor_pcd(self, pts, colors):
        pts_t = o3d.core.Tensor(pts, dtype=o3d.core.Dtype.Float32, device=self.device)
        pcd = o3d.t.geometry.PointCloud(pts_t)

        if colors is not None and len(colors) == len(pts):
            colors_t = o3d.core.Tensor(colors.astype(np.float32) / 255.0, dtype=o3d.core.Dtype.Float32, device=self.device)
            pcd.point["colors"] = colors_t

        return pcd

    # =========================
    # ICP THREAD
    # =========================
    def run_icp(self, pts, colors):
        voxel = self.cfg["init_pose"]["voxel_size"]
        src = self._to_tensor_pcd(pts, colors)
        src = src.voxel_down_sample(voxel)

        src_pts = src.point["positions"].cpu().numpy()
        tgt_pts = self.model.point["positions"].cpu().numpy()

        min_icp_pts = int(self.cfg["init_pose"].get("min_points_icp", 100))
        if len(src_pts) < min_icp_pts:
            with self.lock:
                self.running = False
            if self.cfg.get("debug", {}).get("print_icp", False):
                print(f"ICP skipped: too few downsampled points ({len(src_pts)} < {min_icp_pts})")
            return

        src_center = np.mean(src_pts, axis=0).astype(np.float32)
        tgt_center = np.mean(tgt_pts, axis=0).astype(np.float32)

        T_init = np.eye(4)
        T_init[:3, 3] = tgt_center - src_center

        T_init_t = o3d.core.Tensor(T_init.astype(np.float32), dtype=o3d.core.Dtype.Float32, device=self.device)

        reg = o3d.t.pipelines.registration.icp(
            src,
            self.model,
            self.cfg["init_pose"]["icp_threshold"],
            T_init_t,
            o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=int(self.cfg["init_pose"].get("max_iter", 30))),
        )

        T = reg.transformation.cpu().numpy().astype(np.float64)

        with self.lock:
            self.pose = T
            self.fitness = float(reg.fitness)
            self.inlier_rmse = float(reg.inlier_rmse)
            self.ready = True
            self.running = False

        if self.cfg.get("debug", {}).get("print_icp", False):
            print(f"ICP done: fitness={self.fitness:.4f} rmse={self.inlier_rmse:.5f} device={self.device}")

    # =========================
    # MAIN LOOP
    # =========================
    def process(self, data):

        data["icp_running"] = self.running
        data["icp_ready"] = self.ready

        # already ready
        if self.ready:
            data["pose_icp"] = self.pose
            return data

        # already running
        if self.running:
            return data

        if "mask_final" not in data or "depth_image" not in data:
            return data

        mask_roi = data["mask_final"]

        if np.count_nonzero(mask_roi) < self.cfg["init_pose"]["min_points"]:
            return data

        roi = data.get("roi_used", self.cfg["roi"])
        pts, colors = self._masked_points_from_depth(data["depth_image"], mask_roi, data["color"], roi)
        if pts is None:
            return data

        self.running = True
        self.icp_future = self.pool.submit(self.run_icp, pts, colors)
        data["need_full_pc"] = True

        return data

    def close(self):
        self.pool.shutdown(wait=False)