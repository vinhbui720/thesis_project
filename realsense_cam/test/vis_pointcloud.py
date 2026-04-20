import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# Add parent directory to path to import config and nodes
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import CONFIG
from nodes.source_realcam import SourceRealCam
from nodes.source_bag import SourceBag

class PointCloudTuningNode(Node):
    def __init__(self, src):
        super().__init__('depth_tuning_node')
        self.src = src
        self.publisher_ = self.create_publisher(PointCloud2, '/camera/point_cloud', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 FPS
        
        # Pre-compute pixel grid
        self.uu, self.vv = np.meshgrid(np.arange(src.width), np.arange(src.height))
        self.u_flat = self.uu.flatten()
        self.v_flat = self.vv.flatten()
        
        # Define PointCloud2 fields (x, y, z, rgb)
        # Note: rgb is packed into a 32-bit float/uint
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.get_logger().info("Depth Tuning Node started. Publishing to /camera/point_cloud")

    def timer_callback(self):
        data = self.src.process({})
        if data is None:
            return

        color_img = data["color"]
        depth_frame = data["depth_frame"]
        
        # Convert depth to meters
        depth_scale = depth_frame.get_units()
        depth_img = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
        
        # Filtering config (from live config.py)
        z_low = CONFIG["depth"]["z_low"]
        height_slice = CONFIG["depth"]["height"]
        
        depth_flat = depth_img.flatten()
        valid_mask = (depth_flat > 0)
        
        if not np.any(valid_mask):
            return

        z = depth_flat[valid_mask]
        u = self.u_flat[valid_mask]
        v = self.v_flat[valid_mask]

        # Log/display the current total point cloud size
        self.get_logger().info(f"[PointCloud] Total points: {z.size}")
        
        # 3D Projection
        x = (u - self.src.cx) * z / self.src.fx
        y = (v - self.src.cy) * z / self.src.fy
        
        # Prepare RGB Colors (BGR -> RGB packed)
        colors_bgr = color_img.reshape(-1, 3)[valid_mask]
        r = colors_bgr[:, 2].astype(np.uint32)
        g = colors_bgr[:, 1].astype(np.uint32)
        b = colors_bgr[:, 0].astype(np.uint32)
        
        # APPLY FILTER HIGHLIGHT (Mirror logic in nodes/depth.py)
        far_enough = z[z > z_low]
        if far_enough.size > 0:
            z_ref = np.percentile(far_enough, 5.0) # Robust min (matched to depth.py 5th percentile)
            z_high = z_ref + height_slice
            
            filter_mask = (z > z_low) & (z < z_high)
            
            # Color filtered points Bright Green (R=0, G=255, B=0)
            r[filter_mask] = 0
            g[filter_mask] = 255
            b[filter_mask] = 0
        
        # Pack RGB into 32-bit integer (R << 16 | G << 8 | B)
        rgb_packed = (r << 16) | (g << 8) | b
        
        # Create PointCloud2 data structure
        cloud_data = np.zeros(len(x), dtype=[
            ('x', np.float32), 
            ('y', np.float32), 
            ('z', np.float32), 
            ('rgb', np.uint32)
        ])
        cloud_data['x'] = x
        cloud_data['y'] = y
        cloud_data['z'] = z
        cloud_data['rgb'] = rgb_packed

        # Build Message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'cam_link' # Fixed frame name as requested
        
        pc2_msg = point_cloud2.create_cloud(header, self.fields, cloud_data)
        self.publisher_.publish(pc2_msg)

def main():
    rclpy.init()
    
    is_live = "--live" in sys.argv or CONFIG.get("source", {}).get("mode") == "live"
    if is_live:
        src = SourceRealCam()
    else:
        bag_path = CONFIG["source"]["bag_path"]
        src = SourceBag(path=bag_path)

    tuning_node = PointCloudTuningNode(src)
    
    try:
        rclpy.spin(tuning_node)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        src.close()
        tuning_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
