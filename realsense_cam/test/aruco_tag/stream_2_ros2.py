#!/usr/bin/env python3

import argparse

import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster


class RealSenseBagToRos2(Node):
    def __init__(self, bag_path: str, repeat: bool):
        super().__init__("simple_bag_streamer")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device_from_file(bag_path, repeat_playback=repeat)
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)

        profile = self.pipeline.start(config)
        # Publish at recording rate so RViz is not flooded.
        profile.get_device().as_playback().set_real_time(True)

        self.align_to_color = rs.align(rs.stream.color)
        self.color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.rgb_pub = self.create_publisher(Image, "/cam/rgb/image_raw", reliable_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cam/points", reliable_qos)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.get_logger().info(f"Streaming: {bag_path}")
        self.get_logger().info("Frame for all topics: cam_link")

    def _publish_static_tf(self) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "cam_link"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    @staticmethod
    def _image_msg_bgr8(image: np.ndarray, stamp) -> Image:
        img = np.ascontiguousarray(image)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "cam_link"
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(img.strides[0])
        msg.data = img.tobytes()
        return msg

    def _cloud_msg(self, depth_mm: np.ndarray, color_bgr: np.ndarray, stamp) -> PointCloud2:
        h, w = depth_mm.shape
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = depth_mm.astype(np.float32) * self.depth_scale
        valid = z > 0.0

        x = ((uu - self.color_intr.ppx) / self.color_intr.fx * z)[valid]
        y = ((vv - self.color_intr.ppy) / self.color_intr.fy * z)[valid]
        z = z[valid]

        bgr = color_bgr[valid].astype(np.uint32)
        rgb_uint32 = (bgr[:, 2] << 16) | (bgr[:, 1] << 8) | bgr[:, 0]
        rgb_float32 = rgb_uint32.view(np.float32)

        header = Header()
        header.stamp = stamp
        header.frame_id = "cam_link"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points = zip(x.tolist(), y.tolist(), z.tolist(), rgb_float32.tolist())
        return point_cloud2.create_cloud(header, fields, points)

    def spin(self) -> None:
        try:
            while rclpy.ok():
                frames = self.pipeline.wait_for_frames()
                frames = self.align_to_color.process(frames)

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                depth = np.asanyarray(depth_frame.get_data())
                color = np.asanyarray(color_frame.get_data())
                stamp = self.get_clock().now().to_msg()

                self.rgb_pub.publish(self._image_msg_bgr8(color, stamp))
                self.cloud_pub.publish(self._cloud_msg(depth, color, stamp))
                rclpy.spin_once(self, timeout_sec=0.0)
        except RuntimeError:
            self.get_logger().info("Reached end of .bag file")
        finally:
            self.pipeline.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay bag to RGB + colored point cloud")
    parser.add_argument("bag", help="Absolute path to RealSense .bag file")
    parser.add_argument("--repeat", action="store_true", help="Loop when bag ends")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RealSenseBagToRos2(bag_path=args.bag, repeat=args.repeat)
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
