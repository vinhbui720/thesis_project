#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import time

class TrackingTestNode(Node):
    def __init__(self):
        super().__init__('tracking_test_node')
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/tracking_target_pose', 10)
        self.control_pub = self.create_publisher(Bool, '/tracking_control', 10)
        
        # Timer to publish pose at 50Hz
        self.timer_pose = self.create_timer(0.02, self.publish_pose)
        
        self.start_time = time.time()
        self.get_logger().info('Tracking Test Node Started. Oscillating target pose...')

        # Publish tracking control once at startup
        # We use a short 0.5s timer to ensure the ROS network has connected before publishing
        self.timer_control = self.create_timer(0.5, self.publish_control_once)

    def publish_control_once(self):
        msg = Bool()
        msg.data = True
        self.control_pub.publish(msg)
        self.get_logger().info('Sent /tracking_control = True (Once)')
        # Cancel the timer so it only runs once
        self.timer_control.cancel()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world' # Make sure this matches your base_link
        
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

def main(args=None):
    rclpy.init(args=args)
    node = TrackingTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send false to tracking control when stopping safely before destroy
        if rclpy.ok():
            stop_msg = Bool()
            stop_msg.data = False
            node.control_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
