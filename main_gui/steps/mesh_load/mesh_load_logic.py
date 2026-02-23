import numpy as np
from sensor_msgs_py import point_cloud2
import os


class MeshLoadLogic:
    """
    Logic for mesh loading and point cloud processing.
    Manages ROS2 node lifecycle and configuration.
    """
    
    def __init__(self, controller, view):
        self.controller = controller
        self.view = view

        self.ros = controller.ros_manager
        self.launch = controller.launch_manager

        # Subscribe to point cloud topic
        self.ros.subscribe_cloud(topic="/model_cloud")
        self.ros.cloud_callback = self.cloud_callback
        
        print("[INFO] MeshLoadLogic initialized")

    def launch_model(self, stl_path, sample_points, auto_scale, scale_threshold,
                     center_model, voxel_percentage, voxel_size_override, 
                     normal_radius_multiplier):
        """
        Launch the ROS2 model_manager node with full configuration.
        
        Args:
            stl_path: Path to the STL file
            sample_points: Number of points to sample
            auto_scale: Whether to auto-scale the model
            scale_threshold: Scale threshold value
            center_model: Whether to center the model
            voxel_percentage: Voxel downsampling percentage
            voxel_size_override: Override voxel size (0 = auto)
            normal_radius_multiplier: Radius multiplier for normal estimation
        """
        try:
            # Validate STL file exists
            if not os.path.exists(stl_path):
                raise FileNotFoundError(f"STL file not found: {stl_path}")
            
            print(f"[INFO] Launching model_manager_node")
            print(f"  • STL: {stl_path}")
            print(f"  • Points: {sample_points}")
            print(f"  • Voxel: {voxel_percentage}")

            # Launch the node with all parameters
            self.launch.launch_model_node(
                stl_path,
                sample_points,
                auto_scale,
                scale_threshold,
                center_model,
                voxel_percentage,
                voxel_size_override,
                normal_radius_multiplier
            )
            print("[INFO] Model node launched successfully")

        except Exception as e:
            print(f"[ERROR] Failed to launch model: {e}")
            raise

    def cloud_callback(self, msg):
        """
        Callback for when point cloud is received from ROS2.
        Converts PointCloud2 message to numpy array and updates view.
        """
        try:
            # Convert ROS2 PointCloud2 message to numpy array
            points_list = point_cloud2.read_points(
                msg,
                skip_nans=True,
                field_names=["x", "y", "z"]
            )
            
            points = np.array([
                [p[0], p[1], p[2]]
                for p in points_list
            ])
            
            if len(points) > 0:
                print(f"[INFO] Received point cloud with {len(points)} points")
                self.view.on_cloud_received(points)
            else:
                print("[WARNING] Received empty point cloud")

        except Exception as e:
            print(f"[ERROR] Error processing point cloud: {e}")

    def on_step_exit(self):
        """
        Called when leaving the step.
        Keeps ROS nodes running for next step.
        """
        print("[INFO] Exiting Mesh Load step - keeping ROS nodes running")