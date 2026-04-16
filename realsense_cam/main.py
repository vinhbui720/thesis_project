import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from thesis_project.realsense_cam.core.pipeline import Pipeline
from thesis_project.realsense_cam.config import CONFIG

from thesis_project.realsense_cam.nodes.source_bag import SourceBag
from thesis_project.realsense_cam.nodes.preprocess import Preprocess
from thesis_project.realsense_cam.nodes.background import Background
from thesis_project.realsense_cam.nodes.depth import Depth
from thesis_project.realsense_cam.nodes.fusion import Fusion
from thesis_project.realsense_cam.nodes.detect import Detect
from thesis_project.realsense_cam.nodes.visualize import Visualize
from thesis_project.realsense_cam.nodes.tracker_kalman import TrackerKalman
from thesis_project.realsense_cam.nodes.pose_init_icp import PoseInitICPAsync
from thesis_project.realsense_cam.nodes.pose_fusion import PoseFusion
from thesis_project.realsense_cam.nodes.ros2_publisher import ROS2Publisher

rclpy.init()

src_mode = CONFIG.get("source", {}).get("mode", "bag")
bag_path = CONFIG.get("source", {}).get("bag_path", "data/20260408_203426.bag")

src = SourceBag(path=bag_path, mode=src_mode)

intr = (src.fx, src.fy, src.cx, src.cy, src.width, src.height)

icp_node = PoseInitICPAsync(CONFIG, "data/test1_cam_view.pcd", (src.fx, src.fy, src.cx, src.cy))
ros2_pub  = ROS2Publisher(CONFIG, icp_node)

# Spin the ROS 2 nodes (source_bag + ros2_publisher) in a background thread
# so the service handler can block without stalling the pipeline.
executor = MultiThreadedExecutor()
executor.add_node(src)
executor.add_node(ros2_pub)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

pipeline = Pipeline([
    src,
    Preprocess(CONFIG),
    Background(CONFIG, src.width, src.height),
    Depth(CONFIG, intr),
    Fusion(),
    Detect(CONFIG),
    TrackerKalman(CONFIG, (src.fx, src.fy, src.cx, src.cy)),
    icp_node,
    PoseFusion(CONFIG),
    ros2_pub,
    Visualize(CONFIG, (src.fx, src.fy, src.cx, src.cy)),
])

pipeline.run()

executor.shutdown()
rclpy.shutdown()