#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('traj_plotter')

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/path_command',
            self.listener_callback,
            10
        )

        self.get_logger().info('Continuous plotter started...')

        # 🔥 Enable non-blocking plotting
        plt.ion()

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    def listener_callback(self, msg):
        if not msg.points:
            return

        self.get_logger().info(f'Received {len(msg.points)} points')

        times = []
        positions = {name: [] for name in msg.joint_names}
        velocities = {name: [] for name in msg.joint_names}

        for point in msg.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            times.append(t)

            for i, name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    positions[name].append(point.positions[i])
                if i < len(point.velocities):
                    velocities[name].append(point.velocities[i])

        # 🔥 Clear previous plot instead of blocking
        self.ax1.clear()
        self.ax2.clear()

        for name in msg.joint_names:
            if positions[name]:
                self.ax1.plot(times, positions[name], label=name)
            if velocities[name]:
                self.ax2.plot(times, velocities[name], label=name)

        self.ax1.set_title('Joint Positions')
        self.ax1.set_ylabel('Position (rad)')
        self.ax1.legend()
        self.ax1.grid(True)

        self.ax2.set_title('Joint Velocities')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Velocity (rad/s)')
        self.ax2.legend()
        self.ax2.grid(True)

        self.fig.tight_layout()

        # 🔥 Non-blocking draw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()