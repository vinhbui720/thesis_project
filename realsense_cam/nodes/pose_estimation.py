import numpy as np
import open3d as o3d

def pca_axes(pts):
    mean = np.mean(pts, axis=0)
    cov = np.cov((pts - mean).T)
    eigvals, eigvecs = np.linalg.eig(cov)
    return eigvecs

class PoseEstimation:
    def __init__(self, model_path):
        self.model = o3d.io.read_point_cloud(model_path)
        self.model.estimate_normals()

    def to_o3d(self, pts):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd

    def process(self, data):
        if "points_obj" not in data:
            return data

        pts = data["points_obj"]
        if len(pts) < 100:
            return data

        # Convert to Open3D point cloud
        src = self.to_o3d(pts)
        src.estimate_normals()
        tgt = self.model

        # Downsample & remove noise
        src = src.voxel_down_sample(0.005)
        tgt = tgt.voxel_down_sample(0.005)
        src, _ = src.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # PCA initialization
        src_pts = np.asarray(src.points)
        tgt_pts = np.asarray(tgt.points)
        src_center = np.mean(src_pts, axis=0)
        tgt_center = np.mean(tgt_pts, axis=0)
        src_axes = pca_axes(src_pts)
        tgt_axes = pca_axes(tgt_pts)
        R_init = tgt_axes @ src_axes.T

        T_init = np.eye(4)
        T_init[:3, :3] = R_init
        T_init[:3, 3] = tgt_center - src_center

        # ICP registration
        reg = o3d.pipelines.registration.registration_icp(
            src, tgt, 0.02, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        T = reg.transformation
        data["pose"] = T
        return data