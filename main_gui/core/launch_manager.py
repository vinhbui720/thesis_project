import subprocess
import os


class LaunchManager:
    def __init__(self, ros_manager=None):
        self.process = None
        self.ros_manager = ros_manager

    def launch_model_node(self, stl_path, sample_points, auto_scale, scale_threshold,
                          center_model, voxel_percentage, voxel_size_override, 
                          normal_radius_multiplier):
        """
        Launch the model_manager_node directly with ROS2 parameters.
        
        Args:
            Parameters as per the YAML config
        """
        if self.process is None:
            # Build command with all parameters as ROS2 args
            cmd = [
                "ros2", "run",
                "mesh_processing",
                "model_manager_node",
                "--ros-args",
                "-p", f"stl_path:={stl_path}",
                "-p", f"sample_points:={int(sample_points)}",
                "-p", f"auto_scale:={str(auto_scale).lower()}",
                "-p", f"scale_threshold:={float(scale_threshold)}",
                "-p", f"center_model:={str(center_model).lower()}",
                "-p", f"voxel_percentage:={float(voxel_percentage)}",
                "-p", f"voxel_size_override:={float(voxel_size_override)}",
                "-p", f"normal_radius_multiplier:={float(normal_radius_multiplier)}",
            ]
            
            try:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                print(f"[INFO] Launched model_manager_node")
                print(f"  • STL: {stl_path}")
                print(f"  • Points: {sample_points}")
                print(f"  • Voxel: {voxel_percentage}")
            except Exception as e:
                print(f"[ERROR] Failed to launch node: {e}")
                self.process = None
        else:
            print("[WARNING] Model node already running")

    def is_running(self):
        """Check if process is still running"""
        if self.process is None:
            return False
        return self.process.poll() is None

    def terminate_node(self):
        """Terminate the running node"""
        if self.process is not None:
            self.process.terminate()
            self.process = None