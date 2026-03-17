#!/usr/bin/env python3
import os
import sys
import copy
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener

import numpy as np
import open3d as o3d

# Suppress Open3D WARNING logs
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
pkg_share = get_package_share_directory("mesh_processing")
###########################################
# Registration Helpers
###########################################
def preprocess_point_cloud(pcd, voxel_size):
    """Downsample and compute FPFH features for Global Registration"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """Execute RANSAC based on feature matching to find a rough starting point"""
    distance_threshold = voxel_size * 4
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def multi_scale_icp(source, target, initial_transform):
    """Refine the guess to sub-millimeter precision using Point-to-Plane ICP"""
    current_transform = initial_transform
    voxel_radii = [0.005, 0.002] # Coarse then Fine
    max_iterations = [50, 200]
    
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6,
        relative_rmse=1e-6,
        max_iteration=0
    )

    for scale_idx, radius in enumerate(voxel_radii):
        criteria.max_iteration = max_iterations[scale_idx]
        result_icp = o3d.pipelines.registration.registration_icp(
            source, target, radius, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria
        )
        current_transform = result_icp.transformation

    return result_icp

def get_quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix to a quaternion [x, y, z, w]."""
    m = matrix
    q = np.empty((4, ))
    t = np.trace(m)
    if t > 0.0:
        t = np.sqrt(t + 1.0)
        q[3] = 0.5 * t
        t = 0.5 / t
        q[0] = (m[2, 1] - m[1, 2]) * t
        q[1] = (m[0, 2] - m[2, 0]) * t
        q[2] = (m[1, 0] - m[0, 1]) * t
    else:
        i = 0
        if m[1, 1] > m[0, 0]: i = 1
        if m[2, 2] > m[i, i]: i = 2
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k, j] - m[j, k]) * t
        q[j] = (m[j, i] + m[i, j]) * t
        q[k] = (m[k, i] + m[i, k]) * t
    return q

def transform_stamped_to_matrix(t: TransformStamped):
    """Convert a ROS TransformStamped to a 4x4 NumPy Homogeneous Matrix."""
    # Open3D quaternion format is [w, x, y, z]
    q = np.array([
        t.transform.rotation.w, 
        t.transform.rotation.x, 
        t.transform.rotation.y, 
        t.transform.rotation.z
    ])
    R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
    matrix = np.identity(4)
    matrix[0:3, 0:3] = R
    matrix[0, 3] = t.transform.translation.x
    matrix[1, 3] = t.transform.translation.y
    matrix[2, 3] = t.transform.translation.z
    return matrix

###########################################
# Registration Node
###########################################
class RegistrationNode(Node):
    def __init__(self, debug_mode=False):
        super().__init__('pcd_registration_node')

        self.debug_mode = debug_mode

        # TF2 Setup for looking up World -> Camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.reg_voxel_size = 0.005
        self.min_point_threshold = 100 
        self.world_frame = "world"
        
        # State Tracking
        self.is_initialized = False
        self.locked_transformation = np.identity(4) # Camera -> Object
        self.world_to_object_matrix = np.identity(4) # World -> Object
        
        self.fitness_threshold = 0.95    
        self.rmse_threshold = 0.0020     

        # Load Models (Only load full model if debugging)
        self.source_pcd = o3d.geometry.PointCloud()
        self.full_model_pcd = o3d.geometry.PointCloud()
        self.source_down = None
        self.source_fpfh = None
        self.load_registration_models()

        # Publishers 
        self.status_pub = self.create_publisher(String, '/registration_status', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_object_pose', 10)

        # Debug Publishers
        if self.debug_mode:
            self.aligned_pub = self.create_publisher(PointCloud2, '/camera/depth/color/aligned_full_model', 10)
            self.aligned_source_pub = self.create_publisher(PointCloud2, '/camera/depth/color/aligned_source', 10)
            self.get_logger().info("Debug Mode ENABLED. Point clouds will be published.")
        else:
            self.get_logger().info("Debug Mode DISABLED. Only Pose and TF will be published.")

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.pcd_callback, 10
        )
        
        self.get_logger().info("Registration Node Started. Searching for PERFECT static lock...")
        self.publish_status("SEARCHING_FOR_LOCK")

    def load_registration_models(self):
        try:
            source_path = os.path.join(pkg_share, "data", "final_test_cam_view.pcd")
            if os.path.exists(source_path):
                self.source_pcd = o3d.io.read_point_cloud(source_path)
                self.source_pcd.remove_non_finite_points()
                
                self.source_pcd = self.source_pcd.voxel_down_sample(0.001)
                self.source_pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
                )
                
                self.source_down, self.source_fpfh = preprocess_point_cloud(
                    self.source_pcd, self.reg_voxel_size
                )
            else:
                self.get_logger().error("'final_test_cam_view.pcd' not found.")
            full_model_path = os.path.join(pkg_share, "data", "test2_full_aligned.pcd")
            if self.debug_mode and os.path.exists(full_model_path):
                self.full_model_pcd = o3d.io.read_point_cloud(full_model_path)
        except Exception as e:
            self.get_logger().error(f"Error loading registration models: {e}")

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def broadcast_tf_static(self, transform_matrix, parent_frame, child_frame, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(transform_matrix[0, 3])
        t.transform.translation.y = float(transform_matrix[1, 3])
        t.transform.translation.z = float(transform_matrix[2, 3])

        rot_matrix = transform_matrix[0:3, 0:3]
        q = get_quaternion_from_matrix(rot_matrix)
        
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static TF: {parent_frame} -> {child_frame}")

    def publish_pose(self, transform_matrix, frame_id, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id

        pose_msg.pose.position.x = float(transform_matrix[0, 3])
        pose_msg.pose.position.y = float(transform_matrix[1, 3])
        pose_msg.pose.position.z = float(transform_matrix[2, 3])

        rot_matrix = transform_matrix[0:3, 0:3]
        q = get_quaternion_from_matrix(rot_matrix)

        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])

        self.pose_pub.publish(pose_msg)

    def pcd_callback(self, msg):
        if self.source_down is None: return

        points_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [[p[0], p[1], p[2]] for p in points_iter]
        if not points_list: return
            
        points_arr = np.array(points_list, dtype=np.float64)
        if len(points_arr) < self.min_point_threshold: return

        try:
            target_pcd = o3d.geometry.PointCloud()
            target_pcd.points = o3d.utility.Vector3dVector(points_arr)
            
            # --- ALIGNMENT LOGIC ---
            if not self.is_initialized:
                self.publish_status("CALCULATING_GLOBAL_GUESS")
                target_pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.002, max_nn=30)
                )
                
                target_down, target_fpfh = preprocess_point_cloud(target_pcd, self.reg_voxel_size)
                
                result_ransac = execute_global_registration(
                    self.source_down, target_down, self.source_fpfh, target_fpfh, self.reg_voxel_size
                )

                result_icp = multi_scale_icp(self.source_pcd, target_pcd, result_ransac.transformation)
                
                self.get_logger().info(
                    f"[SEARCHING] Points: {len(points_arr)} | Fit: {result_icp.fitness:.4f} | RMSE: {result_icp.inlier_rmse:.6f}"
                )

                if result_icp.fitness >= self.fitness_threshold and result_icp.inlier_rmse <= self.rmse_threshold:
                    
                    # --- MATH: Convert Camera-Object transform to World-Object transform ---
                    try:
                        # Look up World -> Camera transform
                        t_world_cam = self.tf_buffer.lookup_transform(
                            self.world_frame,
                            msg.header.frame_id,
                            rclpy.time.Time()
                        )
                        world_cam_matrix = transform_stamped_to_matrix(t_world_cam)
                        
                        # Multiply: T_world_obj = T_world_cam * T_cam_obj
                        self.world_to_object_matrix = np.dot(world_cam_matrix, result_icp.transformation)
                        
                        self.is_initialized = True
                        self.locked_transformation = result_icp.transformation
                        self.get_logger().info(">>> GLOBAL MINIMUM FOUND! STRICT STATIC LOCK ACQUIRED IN WORLD FRAME. <<<")
                        
                        # Publish the static TF in the WORLD frame
                        self.broadcast_tf_static(
                            transform_matrix=self.world_to_object_matrix, 
                            parent_frame=self.world_frame, 
                            child_frame="aligned_object_frame", 
                            stamp=self.get_clock().now().to_msg()
                        )
                        self.publish_status("LOCKED_SUCCESSFULLY")

                    except Exception as e:
                        self.get_logger().warn(f"Waiting for TF tree to establish World->Camera link: {e}")
                        return # Try again next frame

                else:
                    self.publish_status("FAILED_LOCAL_MINIMA_RETRYING")
                    return 
                    
            # --- CONTINUOUS PUBLISHING AFTER LOCK ---
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.world_frame  

            # Publish the Object Pose in the WORLD frame
            self.publish_pose(self.world_to_object_matrix, self.world_frame, header.stamp)
            self.publish_status("LOCKED_STATICALLY_IN_TF")

            # 2. Publish Debug Point Clouds (ONLY if --debug was passed)
            if self.debug_mode:
                # We also transform the point clouds by the world matrix so they appear correctly in RViz under "world"
                if len(self.source_pcd.points) > 0:
                    aligned_source = copy.deepcopy(self.source_pcd).transform(self.world_to_object_matrix)
                    aligned_source_pts = np.asarray(aligned_source.points)
                    aligned_source_msg = pc2.create_cloud_xyz32(header, aligned_source_pts)
                    self.aligned_source_pub.publish(aligned_source_msg)

                if len(self.full_model_pcd.points) > 0:
                    aligned_full = copy.deepcopy(self.full_model_pcd).transform(self.world_to_object_matrix)
                    aligned_full_pts = np.asarray(aligned_full.points)
                    aligned_full_msg = pc2.create_cloud_xyz32(header, aligned_full_pts)
                    self.aligned_pub.publish(aligned_full_msg)
                
        except Exception as e:
            self.get_logger().error(f"Registration failed on this frame: {e}")
            self.publish_status("ERROR_EXCEPTION")

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', help='Enable point cloud visualization publishers')
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])
    
    rclpy.init(args=ros_args)
    node = RegistrationNode(debug_mode=parsed_args.debug)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()