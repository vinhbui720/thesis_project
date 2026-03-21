import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool

import numpy as np
from scipy.spatial.transform import Rotation as R


class EEZConeSender(Node):

    def __init__(self):
        super().__init__('ee_z_cone_sender')

        self.pose_pub = self.create_publisher(PoseArray, '/target_poses', 10)
        self.start_pub = self.create_publisher(Bool, '/start', 10)

        self.timer = self.create_timer(1.0, self.run_once)

    def run_once(self):
        center = np.array([0.18, 0.1, 0.24])

        steps = 5
        total_angle = 2 * np.pi

        alpha = np.deg2rad(20)  # góc mở hình nón

        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'

        world_up = np.array([0, 0, 1])  # trục cone

        for i in range(steps):
            theta = total_angle * i / steps

            # 🔴 Z axis của EE (cone)
            z_axis = np.array([
                np.sin(alpha) * np.cos(theta),
                np.sin(alpha) * np.sin(theta),
                np.cos(alpha)
            ])

            z_axis = z_axis / np.linalg.norm(z_axis)

            # 🔵 giữ X luôn "ổn định"
            x_axis = np.cross(world_up, z_axis)

            # tránh singularity khi song song
            if np.linalg.norm(x_axis) < 1e-6:
                x_axis = np.array([1, 0, 0])

            x_axis = x_axis / np.linalg.norm(x_axis)

            # 🔵 Y = Z × X
            y_axis = np.cross(z_axis, x_axis)

            # Rotation matrix
            R_mat = np.column_stack((x_axis, y_axis, z_axis))

            quat = R.from_matrix(R_mat).as_quat()

            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])

            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)
        self.get_logger().info(f'Published {len(pose_array.poses)} cone poses')

        self.create_timer(0.5, self.send_start_once)
        self.timer.cancel()

    def send_start_once(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)

        self.get_logger().info('Sent start signal')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = EEZConeSender()
    rclpy.spin(node)


if __name__ == '__main__':
    main()