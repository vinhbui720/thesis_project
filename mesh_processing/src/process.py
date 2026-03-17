#!/usr/bin/env python3
import sys
import os
import json
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import argparse

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from collections import Counter
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QDoubleSpinBox, QSpinBox, QGroupBox, 
                             QGridLayout, QPushButton, QCheckBox)
from PyQt5.QtCore import QTimer
pkg_share = get_package_share_directory("mesh_processing")
###########################################
# Hyperparameters Struct
###########################################
@dataclass
class PcdParams:
    voxel_size: float = 0.002
    nb_neighbors: int = 42
    std_ratio: float = 2.4
    plane_dist: float = 0.002
    plane_iter: int = 800
    normal_radius: float = 0.08
    frame_count: int = 10
    
    # ROI Configuration
    roi_min_x: float = -0.15
    roi_min_y: float = 0.27
    roi_min_z: float = -0.19
    roi_max_x: float = 0.15
    roi_max_y: float = 0.17
    roi_max_z: float = 0.35

# Global parameter instance
PARAMS = PcdParams()
CONFIG_FILE = os.path.join(pkg_share, "config", "pcd_config.json")

def load_params():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
                for k, v in data.items():
                    if hasattr(PARAMS, k):
                        setattr(PARAMS, k, v)
            print(f"Loaded optimal config from {CONFIG_FILE}")
        except Exception as e:
            print(f"Failed to load config: {e}")

###########################################
# GUI Configuration Panel
###########################################
class TuningGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Point Cloud Tuning Parameters")
        self.initUI()
        
        # ROS 2 Spin Timer (Integrates ROS 2 with PyQt5 Event Loop)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10) # ~100Hz spinning

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def initUI(self):
        main_layout = QVBoxLayout()
        
        # General Params Group
        gen_group = QGroupBox("Filtering & Segmentation")
        gen_layout = QVBoxLayout()
        self.add_double_spinbox(gen_layout, "Voxel Size:", 0.001, 0.05, 0.001, 4, PARAMS.voxel_size, "voxel_size")
        self.add_int_spinbox(gen_layout, "NB Neighbors:", 5, 100, 1, PARAMS.nb_neighbors, "nb_neighbors")
        self.add_double_spinbox(gen_layout, "Std Ratio:", 0.1, 5.0, 0.1, 2, PARAMS.std_ratio, "std_ratio")
        self.add_double_spinbox(gen_layout, "Plane Dist Thresh:", 0.001, 0.1, 0.001, 4, PARAMS.plane_dist, "plane_dist")
        self.add_int_spinbox(gen_layout, "Plane RANSAC Iters:", 100, 5000, 100, PARAMS.plane_iter, "plane_iter")
        self.add_double_spinbox(gen_layout, "Normal Radius:", 0.01, 0.2, 0.01, 3, PARAMS.normal_radius, "normal_radius")
        gen_group.setLayout(gen_layout)
        
        # ROI Params Group
        roi_group = QGroupBox("Region of Interest (ROI) Bounds")
        roi_layout = QGridLayout()
        self.add_grid_double_spinbox(roi_layout, "Min X:", 0, 0, -2.0, 2.0, 0.01, 3, PARAMS.roi_min_x, "roi_min_x")
        self.add_grid_double_spinbox(roi_layout, "Min Y:", 0, 2, -2.0, 2.0, 0.01, 3, PARAMS.roi_min_y, "roi_min_y")
        self.add_grid_double_spinbox(roi_layout, "Min Z:", 0, 4, -2.0, 2.0, 0.01, 3, PARAMS.roi_min_z, "roi_min_z")
        self.add_grid_double_spinbox(roi_layout, "Max X:", 1, 0, -2.0, 2.0, 0.01, 3, PARAMS.roi_max_x, "roi_max_x")
        self.add_grid_double_spinbox(roi_layout, "Max Y:", 1, 2, -2.0, 2.0, 0.01, 3, PARAMS.roi_max_y, "roi_max_y")
        self.add_grid_double_spinbox(roi_layout, "Max Z:", 1, 4, -2.0, 2.0, 0.01, 3, PARAMS.roi_max_z, "roi_max_z")
        roi_group.setLayout(roi_layout)
        
        # Save Button
        save_btn = QPushButton("Save Current Configuration")
        save_btn.clicked.connect(self.save_params)
        
        main_layout.addWidget(gen_group)
        main_layout.addWidget(roi_group)
        main_layout.addWidget(save_btn)
        self.setLayout(main_layout)

    def save_params(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(PARAMS.__dict__, f, indent=4)
            print(f"Configuration saved to {CONFIG_FILE}")
        except Exception as e:
            print(f"Failed to save config: {e}")

    def add_double_spinbox(self, layout, label_text, min_val, max_val, step, decimals, default_val, param_name):
        row = QHBoxLayout()
        label = QLabel(label_text)
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setDecimals(decimals)
        spinbox.setValue(default_val)
        spinbox.valueChanged.connect(lambda val, p=param_name: setattr(PARAMS, p, val))
        row.addWidget(label)
        row.addWidget(spinbox)
        layout.addLayout(row)
        
    def add_grid_double_spinbox(self, layout, label_text, row, col, min_val, max_val, step, decimals, default_val, param_name):
        label = QLabel(label_text)
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setDecimals(decimals)
        spinbox.setValue(default_val)
        spinbox.valueChanged.connect(lambda val, p=param_name: setattr(PARAMS, p, val))
        layout.addWidget(label, row, col)
        layout.addWidget(spinbox, row, col + 1)

    def add_int_spinbox(self, layout, label_text, min_val, max_val, step, default_val, param_name):
        row = QHBoxLayout()
        label = QLabel(label_text)
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default_val)
        spinbox.valueChanged.connect(lambda val, p=param_name: setattr(PARAMS, p, val))
        row.addWidget(label)
        row.addWidget(spinbox)
        layout.addLayout(row)

###########################################
# Processing Functions
###########################################
def process_pointcloud(raw_pcd, p: PcdParams):
    
    # 1. Check validity
    if len(raw_pcd.points) < 50:
        return raw_pcd

    points = np.asarray(raw_pcd.points)
    valid = ~(np.isnan(points).any(axis=1) | np.isinf(points).any(axis=1))
    raw_pcd.points = o3d.utility.Vector3dVector(points[valid])

    if len(raw_pcd.points) < 50:
        return raw_pcd

    # 2. Voxel downsample
    voxel_pcd = raw_pcd.voxel_down_sample(p.voxel_size)

    if len(voxel_pcd.points) < 50:
        return voxel_pcd

    # 3. ROI crop (Safely handle inverted min/max bounds)
    min_bounds = np.array([min(p.roi_min_x, p.roi_max_x), min(p.roi_min_y, p.roi_max_y), min(p.roi_min_z, p.roi_max_z)])
    max_bounds = np.array([max(p.roi_min_x, p.roi_max_x), max(p.roi_min_y, p.roi_max_y), max(p.roi_min_z, p.roi_max_z)])
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bounds, max_bounds)
    roi_pcd = voxel_pcd.crop(bbox)

    if len(roi_pcd.points) < 30:
        return roi_pcd

    # 4. Outlier removal
    clean_pcd, _ = roi_pcd.remove_statistical_outlier(
        nb_neighbors=p.nb_neighbors,
        std_ratio=p.std_ratio
    )

    if len(clean_pcd.points) < 30:
        return clean_pcd

    # 5. Plane segmentation
    plane_model, inliers = clean_pcd.segment_plane(
        distance_threshold=p.plane_dist,
        ransac_n=3,
        num_iterations=p.plane_iter
    )
    objects = clean_pcd.select_by_index(inliers, invert=True)

    if len(objects.points) < 20:
        return objects

    # 6. Normal estimation
    objects.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=p.normal_radius,
            max_nn=30
        )
    )

    if len(objects.normals) == 0:
        return objects

    objects.orient_normals_towards_camera_location(
        camera_location=np.array([0, 0, 0])
    )

    return objects

def temporal_consistency_filter(frames, voxel=0.002, min_ratio=0.6):
    if len(frames) == 0:
        return o3d.geometry.PointCloud()

    voxel_maps = []
    for pcd in frames:
        down = pcd.voxel_down_sample(voxel)
        pts = np.asarray(down.points)
        if len(pts) == 0:
            continue
        vox = np.floor(pts / voxel).astype(np.int32)
        voxel_maps.append(set(map(tuple, vox)))

    counter = Counter()
    for vset in voxel_maps:
        counter.update(vset)

    threshold = int(len(frames) * min_ratio)
    stable_voxels = {v for v, c in counter.items() if c >= threshold}
    stable_points = []

    # Use the most recent frame as reference
    ref = frames[-1] 
    pts = np.asarray(ref.points)
    if len(pts) > 0:
        vox = np.floor(pts / voxel).astype(np.int32)
        for p, v in zip(pts, vox):
            if tuple(v) in stable_voxels:
                stable_points.append(p)

    result = o3d.geometry.PointCloud()
    if len(stable_points) > 0:
        result.points = o3d.utility.Vector3dVector(np.array(stable_points))

    return result


###########################################
# ROS2 Node
###########################################
class BagToPcdRos2(Node):
    def __init__(self, bag_file=None, use_realcam=False, debug=False):
        super().__init__('bag_to_pcd_streamer')
        self.use_realcam = use_realcam
        # 1. Setup RealSense
        self.debug = debug
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.use_realcam:
            self.get_logger().info("Starting REAL RealSense D435 camera")

            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        else:
            self.get_logger().info(f"Starting BAG playback: {bag_file}")

            config.enable_device_from_file(bag_file, repeat_playback=True)
            
        self.pipeline.start(config)
        
        self.pc = rs.pointcloud()
        self.align = rs.align(rs.stream.color)

        # 2. Publishers
        self.pcd_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        if self.debug:
            self.pcd_raw_pub = self.create_publisher(
                PointCloud2,
                '/camera/depth/color/points_raw',
                10
            )
            self.get_logger().info("Debug mode ON: publishing raw pointcloud")
        else:
            self.get_logger().info("Debug mode OFF: raw pointcloud disabled")
        
        # 3. Frame Buffer for Temporal Consistency
        self.frame_buffer = []
        
        # 5. Timer
        self.timer = self.create_timer(1/15.0, self.publish_pcd)


    def publish_pcd(self):
        success, frames = self.pipeline.try_wait_for_frames(timeout_ms=500)
        if not success: 
            return

        # Align depth to color
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        # Generate points from RealSense
        self.pc.map_to(color_frame)
        rs_points = self.pc.calculate(depth_frame)
        v = np.asanyarray(rs_points.get_vertices()).view(np.float32).reshape(-1, 3)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world_depth_camera_link'

        # --- PUBLISH RAW POINTCLOUD ---
        if self.debug:
            pcd_raw_msg = pc2.create_cloud_xyz32(header, v)
            self.pcd_raw_pub.publish(pcd_raw_msg)

        # Convert to Open3D PointCloud
        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(v)

        # Apply spatial processing using GUI params
        processed_pcd = process_pointcloud(o3d_pcd, PARAMS)
        
        # Add to frame buffer and maintain size
        self.frame_buffer.append(processed_pcd)
        if len(self.frame_buffer) > PARAMS.frame_count:
            self.frame_buffer.pop(0)

        # Only publish when the buffer is full
        if len(self.frame_buffer) == PARAMS.frame_count:
            # Apply temporal consistency
            final_pcd = temporal_consistency_filter(
                self.frame_buffer, 
                voxel=PARAMS.voxel_size
            )
            
            final_points = np.asarray(final_pcd.points)

            # --- PUBLISH PROCESSED POINTCLOUD ---
            if len(final_points) > 0:
                pcd_msg = pc2.create_cloud_xyz32(header, final_points)
                self.pcd_pub.publish(pcd_msg)

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--realcam',
        action='store_true',
        help='Use real RealSense camera'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable raw pointcloud publishing'
    )

    default_bag = os.path.join(
        pkg_share,
        "bags",
        "recording_20260309_090850.bag"
    )

    parser.add_argument(
        '--bag',
        default=default_bag,
        help='Bag file path'
    )

    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=ros_args)

    load_params()

    node = BagToPcdRos2(
        bag_file=parsed_args.bag,
        use_realcam=parsed_args.realcam,
        debug=parsed_args.debug
    )

    DEBUG_GUI = False

    if DEBUG_GUI:

        app = QApplication(sys.argv)

        gui = TuningGUI(node)
        gui.show()

        exit_code = app.exec_()

        try:
            node.pipeline.stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(exit_code)

    else:

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                node.pipeline.stop()
            except Exception:
                pass

            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

            sys.exit(0)


if __name__ == '__main__':
    main()