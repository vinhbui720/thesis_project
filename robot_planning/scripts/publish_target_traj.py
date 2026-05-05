#!/usr/bin/env python3
"""
Publish a target trajectory to /motomini/target_pose without any recording or metrics.
Trajectory: S-curve warmup from current pose to start position, then constant-velocity
linear motion along Y from start_y to end_y.

Usage (ros2 run / direct):
  ros2 run robot_planning publish_target_traj \
    --ros-args -p start_x:=0.25 -p start_y:=-0.15 -p start_z:=0.15 \
               -p end_y:=0.1 -p target_velocity:=0.05 -p warmup_time:=2.0
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class TargetTrajPublisher(Node):
    def __init__(self):
        super().__init__('target_traj_publisher')

        self.declare_parameter('start_x', 0.25)
        self.declare_parameter('start_y', -0.15)
        self.declare_parameter('start_z', 0.15)
        self.declare_parameter('end_y', 0.1)
        self.declare_parameter('target_velocity', 0.1)
        self.declare_parameter('warmup_time', 1.0)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_z = float(self.get_parameter('start_z').value)
        self.end_y = float(self.get_parameter('end_y').value)
        self.target_velocity = float(self.get_parameter('target_velocity').value)
        self.warmup_time = float(self.get_parameter('warmup_time').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.initial_pose = None
        self.start_time = None  # set when first pose is obtained

        self.pose_pub = self.create_publisher(PoseStamped, '/motomini/target_pose', 10)
        self.create_timer(0.02, self._publish_pose)  # 50 Hz

        self.get_logger().info(
            f'Publishing trajectory: start=({self.start_x}, {self.start_y}, {self.start_z}), '
            f'end_y={self.end_y}, vel={self.target_velocity} m/s, warmup={self.warmup_time} s'
        )

    def _publish_pose(self):
        # Acquire initial pose from TF once
        if self.initial_pose is None:
            try:
                t = self.tf_buffer.lookup_transform('world', 'magnetic_link', rclpy.time.Time())
                self.initial_pose = t.transform
                self.start_time = time.time()
                self.get_logger().info('Got initial pose from TF. Starting trajectory.')
            except Exception:
                return

        elapsed = time.time() - self.start_time

        target_x, target_y, target_z = self._compute_position(elapsed)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        msg.pose.orientation = self.initial_pose.rotation
        self.pose_pub.publish(msg)

        # Broadcast as TF for visualisation
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'tracking_frame'
        tf_msg.transform.translation.x = target_x
        tf_msg.transform.translation.y = target_y
        tf_msg.transform.translation.z = target_z
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _compute_position(self, current_time: float):
        """Same trajectory logic as run_tracking_trial: S-curve warmup then linear Y motion."""
        ip = self.initial_pose
        if current_time < self.warmup_time:
            progress = current_time / max(self.warmup_time, 1e-9)
            s = 0.5 * (1.0 - math.cos(math.pi * progress))
            x = ip.translation.x + (self.start_x - ip.translation.x) * s
            y = ip.translation.y + (self.start_y - ip.translation.y) * s
            z = ip.translation.z + (self.start_z - ip.translation.z) * s
        else:
            x = self.start_x
            z = self.start_z
            dist = self.target_velocity * (current_time - self.warmup_time)
            y = min(self.start_y + dist, self.end_y)
        return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrajPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
