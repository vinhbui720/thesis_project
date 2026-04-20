import rclpy
import threading
import argparse
from rclpy.executors import MultiThreadedExecutor

from core.pipeline import Pipeline
from config import CONFIG

# Source nodes will be imported conditionally
from nodes.preprocess import Preprocess
from nodes.background import Background
from nodes.depth import Depth
from nodes.fusion import Fusion
from nodes.detect import Detect
from nodes.visualize import Visualize
from nodes.tracker_kalman import TrackerKalman
from nodes.pose_init_icp import PoseInitICPAsync
from nodes.pose_fusion import PoseFusion
from nodes.ros2_publisher import ROS2Publisher

rclpy.init()

# --- Argument Parsing ---
parser = argparse.ArgumentParser()
parser.add_argument("--realcam", action="store_true", help="Enable real camera (live stream)")
parser.add_argument("--bag", type=str, help="Path to the bag file (overrides config)")
parser.add_argument("--tuning", action="store_true", help="Launch interactive GUI tuning mode to calibrate threshold, blur, etc.")
args = parser.parse_args()

# --- Source Selection ---
initial_bg = None
if args.realcam:
    print("[INFO] Initializing Real Camera source...")
    from nodes.source_realcam import SourceRealCam
    src = SourceRealCam()
    
    # Capture first frame as background after warmup
    print("[INFO] Capturing initial background frame...")
    bg_data = src.process({})
    if bg_data and "color" in bg_data:
        initial_bg = bg_data["color"]
        print("[INFO] background is ready for user when the image is ready")
    else:
        print("[WARNING] Failed to capture initial background frame!")
else:
    src_mode = CONFIG.get("source", {}).get("mode", "bag")
    bag_path = args.bag if args.bag else CONFIG.get("source", {}).get("bag_path", "data/20260408_203426.bag")
    print(f"[INFO] Initializing Bag source: {bag_path}")
    from nodes.source_bag import SourceBag
    src = SourceBag(path=bag_path, mode=src_mode)

intr = (src.fx, src.fy, src.cx, src.cy, src.width, src.height)

icp_node = PoseInitICPAsync(CONFIG, "data/test1_cam_view.pcd", (src.fx, src.fy, src.cx, src.cy))
ros2_pub  = ROS2Publisher(CONFIG, icp_node)

# --- Tuning GUI branching ---
if args.tuning:
    print("[INFO] Launching interactive Tuning GUI...")
    from tools.tuning_gui import TuningGUI
    tuner = TuningGUI(src, CONFIG, initial_bg)
    tuner.start()
    
    # Tuning gui will run and block. When done it will close the src.
    rclpy.shutdown()
    exit(0)

# Spin the ROS 2 node (ros2_publisher) in a background thread
# so the service handler can block without stalling the pipeline.
executor = MultiThreadedExecutor()
executor.add_node(ros2_pub)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

pipeline = Pipeline([
    src,
    Preprocess(CONFIG),
    Background(CONFIG, src.width, src.height, initial_bg=initial_bg),
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