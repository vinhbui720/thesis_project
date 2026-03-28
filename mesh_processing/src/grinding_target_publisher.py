#!/usr/bin/env python3
"""
Simple grinding target publisher.

Subscribe to:
  - /trajectory_waypoints (PoseArray): T_world^points from picking_pose.py
  - /pick_point (Point): T_world^ee (reference EE position at picking)

Compute:
  - T_ee^points = T_world^points - T_world^ee (offset from EE to trajectory points)
  
Publish to:
  - /target_poses (PoseArray): Target tool poses for motion planning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
import numpy as np


class GrindingTargetPublisher(Node):

    def __init__(self):
        super().__init__("grinding_target_publisher_node")

        # Subscribers
        self.create_subscription(
            PoseArray,
            "/trajectory_waypoints",
            self.trajectory_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/pick_point",
            self.pick_point_callback,
            10,
        )

        # Publisher
        self.target_pub = self.create_publisher(PoseArray, "/target_poses", 10)

        # Storage
        self.trajectory_waypoints = None
        self.pick_point = None
        self.ee_position = None

        self.get_logger().info(
            "Grinding Target Publisher started\n"
            "  Input:  /trajectory_waypoints, /pick_point\n"
            "  Output: /target_poses"
        )

    def pick_point_callback(self, msg):
        """Store the EE reference position (T_world^ee)."""
        self.ee_position = np.array([msg.x, msg.y, msg.z])
        
        if self.trajectory_waypoints is not None:
            self.publish_targets()

    def trajectory_callback(self, msg):
        """Store trajectory waypoints (T_world^points) and publish targets."""
        self.trajectory_waypoints = msg
        
        if self.ee_position is not None:
            self.publish_targets()

    def publish_targets(self):
        """
        Transform trajectory points to target tool poses.
        
        For each trajectory point:
          T_ee^points = T_world^points - T_world^ee  (offset)
          T_world^tool = T_world^ee + T_ee^points = T_world^points (unchanged positions)
          
        The tool targets are the trajectory waypoint positions themselves.
        """
        target_poses = PoseArray()
        target_poses.header = self.trajectory_waypoints.header
        target_poses.header.stamp = self.get_clock().now().to_msg()

        for pose in self.trajectory_waypoints.poses:
            # Trajectory point in world frame (T_world^points)
            traj_point = np.array(
                [pose.position.x, pose.position.y, pose.position.z]
            )

            # Offset from EE to this trajectory point (T_ee^points)
            offset = traj_point - self.ee_position

            # Tool target position (T_world^tool = T_world^ee + T_ee^points)
            tool_target = self.ee_position + offset  # = traj_point

            # Create target pose with same orientation as trajectory point
            target = Pose()
            target.position.x = tool_target[0]
            target.position.y = tool_target[1]
            target.position.z = tool_target[2]
            target.orientation = pose.orientation

            target_poses.poses.append(target)

        self.target_pub.publish(target_poses)

        self.get_logger().info(
            f"Published {len(target_poses.poses)} target poses "
            f"(EE at [{self.ee_position[0]:.4f}, {self.ee_position[1]:.4f}, {self.ee_position[2]:.4f}])"
        )


def main():
    rclpy.init()
    node = GrindingTargetPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
