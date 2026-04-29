#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time


class TrackingTestNode(Node):
    def __init__(self):
        super().__init__('tracking_test_node')

        # motomini_feedback_stream.cpp subscribes to this target pose topic.
        self.pose_topic = '/motomini/target_pose'
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish pose at 50Hz
        self.timer_pose = self.create_timer(0.02, self.publish_pose)

        self.start_time = time.time()
        self.get_logger().info(
            f'Tracking Test Node Started. Publishing oscillating target pose to {self.pose_topic}'
        )

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'  # Assuming 'world' is the fixed frame for the robot's base

        # Calculate time elapsed
        current_time = time.time() - self.start_time

        # Oscillate y between -0.1 and 0.1 using a sine wave
        # Completes one full back and forth cycle every 5 seconds
        period = 5.0
        y_val = 0.1 * math.sin(2.0 * math.pi * current_time / period)

        # Set Position
        msg.pose.position.x = 0.18
        msg.pose.position.y = y_val
        msg.pose.position.z = 0.245

        # Set Orientation (Assuming tool pointing forward/up: W=1)
        # Using Identity quaternion to avoid violent 180-degree wrist flips
        # if the real robot starts from all-zero joints.
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)
        self.publish_tracking_tf(msg)

    def publish_tracking_tf(self, pose_msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = pose_msg.header.stamp
        tf_msg.header.frame_id = pose_msg.header.frame_id
        tf_msg.child_frame_id = 'tracking_frame'

        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
