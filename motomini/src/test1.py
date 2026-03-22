import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

import numpy as np
from scipy.spatial.transform import Rotation as R


class EEZConeSender(Node):

    def __init__(self):
        super().__init__('ee_z_cone_sender')

        self.pose_pub = self.create_publisher(PoseArray, '/target_poses', 10)
        self.timer = self.create_timer(1.0, self.run_once)

    def run_once(self):
        # --- CONFIG ---
        center = np.array([0.18, 0.1, 0.24])   # fixed tip position

        R_frame1 = R.from_euler('xyz', [0, 0, 0])

        alpha = np.deg2rad(20)   # tilt angle from Frame 2's X axis (step 3)
        steps = 18                # number of poses sweeping around Z

        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'

        for i in range(steps):
            theta = 2 * np.pi * i / steps

            # Step 2: rotate Frame1 around its Z by theta → Frame 2
            R_step2 = R.from_rotvec(theta * R_frame1.apply([0, 0, 1]))
            R_frame2 = R_step2 * R_frame1

            # Step 3: tilt Frame2 around its X by alpha → Frame 3 (waypoint)
            R_step3 = R.from_rotvec(alpha * R_frame2.apply([1, 0, 0]))
            R_frame3 = R_step3 * R_frame1

            quat = R_frame3.as_quat()  # [x, y, z, w]

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
        self.get_logger().info(
            f'Published {len(pose_array.poses)} precession poses | '
            f'alpha={np.rad2deg(alpha):.1f}deg | steps={steps}')

        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = EEZConeSender()
    rclpy.spin(node)


if __name__ == '__main__':
    main()