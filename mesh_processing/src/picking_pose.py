#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty, Bool

from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_matrix


class PickPointTransformer(Node):

    def __init__(self, debug_mode=False):

        super().__init__("pick_point_transformer_node")

        self.debug_mode = debug_mode

        pkg_share = get_package_share_directory("mesh_processing")
        self.master_csv_path = os.path.join(pkg_share, "data", "final_test_master.csv")

        self.source_frame = "object_link"
        self.target_frame = "world"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pick_pub = self.create_publisher(Point, "/pick_point", 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory_waypoints", 10)
        self.ok_pub = self.create_publisher(Bool, "/trajectory_ok", 10)

        if self.debug_mode:
            self.marker_pub = self.create_publisher(MarkerArray, "/trajectory_markers", 10)

        # Request subscriber
        self.create_subscription(
            Empty,
            "/get_trajectory",
            self.get_trajectory_callback,
            10,
        )

        self.pick_point = None
        self.trajectory_points = []

        self.read_csv_data()

        if self.debug_mode:
            self.timer = self.create_timer(0.2, self.debug_timer)

    # ------------------------------------------------

    def read_csv_data(self):

        if not os.path.exists(self.master_csv_path):
            self.get_logger().error("Master CSV not found")
            return

        traj_csv_path = None

        with open(self.master_csv_path, "r") as f:

            lines = f.readlines()

            for i, line in enumerate(lines):

                line = line.strip()

                if "Trajectory Waypoints Path" in line:

                    traj_filename = line.split(",")[1].strip()

                    traj_csv_path = os.path.join(
                        get_package_share_directory("mesh_processing"),
                        "data",
                        traj_filename,
                    )

                elif "Pick X (m),Pick Y (m),Pick Z (m)" in line:

                    coords = lines[i + 1].strip().split(",")

                    self.pick_point = np.array(
                        [float(coords[0]), float(coords[1]), float(coords[2]), 1.0]
                    )

        if traj_csv_path and os.path.exists(traj_csv_path):

            with open(traj_csv_path, "r") as f:

                for line in f:

                    line = line.strip()

                    if not line or "Waypoint" in line:
                        continue

                    parts = line.split(",")

                    self.trajectory_points.append(
                        np.array(
                            [
                                float(parts[0]),
                                float(parts[1]),
                                float(parts[2]),
                                1.0,
                            ]
                        )
                    )

        self.get_logger().info(
            f"Loaded {len(self.trajectory_points)} trajectory points"
        )

    # ------------------------------------------------

    def get_transform_matrix(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            matrix = quaternion_matrix([q.x, q.y, q.z, q.w])

            matrix[0, 3] = t.x
            matrix[1, 3] = t.y
            matrix[2, 3] = t.z

            return matrix

        except Exception as e:

            self.get_logger().warn(f"TF lookup failed: {e}")

            return None

    # ------------------------------------------------

    def compute_world_points(self):

        matrix = self.get_transform_matrix()

        if matrix is None:
            return None, None, None

        # Pick point
        wp = matrix @ self.pick_point

        world_pick = Point()
        world_pick.x = wp[0]
        world_pick.y = wp[1]
        world_pick.z = wp[2]

        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        world_traj = []

        for pt in self.trajectory_points:

            p = matrix @ pt

            pose = Pose()

            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]

            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

            world_pt = Point()
            world_pt.x = p[0]
            world_pt.y = p[1]
            world_pt.z = p[2]

            world_traj.append(world_pt)

        return world_pick, pose_array, world_traj

    # ------------------------------------------------

    def get_trajectory_callback(self, msg):

        if self.pick_point is None:
            return

        world_pick, pose_array, world_traj = self.compute_world_points()

        if world_pick is None:
            return

        self.pick_pub.publish(world_pick)
        self.traj_pub.publish(pose_array)

        ok = Bool()
        ok.data = True
        self.ok_pub.publish(ok)

        if self.debug_mode:
            self.publish_markers(world_pick, world_traj)

    # ------------------------------------------------

    def debug_timer(self):

        if self.pick_point is None:
            return

        world_pick, pose_array, world_traj = self.compute_world_points()

        if world_pick is None:
            return

        self.publish_markers(world_pick, world_traj)

    # ------------------------------------------------

    def publish_markers(self, pick_point, traj_points):

        marker_array = MarkerArray()

        pick_marker = Marker()

        pick_marker.header.frame_id = self.target_frame
        pick_marker.header.stamp = self.get_clock().now().to_msg()

        pick_marker.ns = "pick_point"
        pick_marker.id = 0
        pick_marker.type = Marker.SPHERE
        pick_marker.action = Marker.ADD

        pick_marker.pose.position = pick_point
        pick_marker.pose.orientation.w = 1.0

        pick_marker.scale.x = 0.01
        pick_marker.scale.y = 0.01
        pick_marker.scale.z = 0.01

        pick_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        marker_array.markers.append(pick_marker)

        traj_marker = Marker()

        traj_marker.header.frame_id = self.target_frame
        traj_marker.header.stamp = self.get_clock().now().to_msg()

        traj_marker.ns = "trajectory"
        traj_marker.id = 1
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD

        traj_marker.scale.x = 0.004

        traj_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        traj_marker.points.append(pick_point)

        for p in traj_points:
            traj_marker.points.append(p)

        marker_array.markers.append(traj_marker)

        self.marker_pub.publish(marker_array)


# ------------------------------------------------


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable RViz markers",
    )

    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=ros_args)

    node = PickPointTransformer(debug_mode=parsed_args.debug)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()