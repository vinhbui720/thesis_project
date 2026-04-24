#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('traj_plotter')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/path_command',  # Matches the topic you just changed in setup!
            self.listener_callback,
            10)
        self.get_logger().info('Waiting for trajectory on /path_command...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points.')
        if not msg.points:
            return
            
        times = []
        positions = {name: [] for name in msg.joint_names}
        velocities = {name: [] for name in msg.joint_names}
        
        for point in msg.points:
            # Convert builtin builtin_interfaces/Duration back to float seconds
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            times.append(t)
            
            for i, name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    positions[name].append(point.positions[i])
                if i < len(point.velocities):
                    velocities[name].append(point.velocities[i])
                    
        # Plotting
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        for name in msg.joint_names:
            if positions[name]:
                ax1.plot(times, positions[name], label=name, marker='o', markersize=3)
            if velocities[name]:
                ax2.plot(times, velocities[name], label=name, marker='o', markersize=3)
                
        ax1.set_title('Offline Plan: Joint Positions')
        ax1.set_ylabel('Position (rad)')
        ax1.legend(loc='lower left', bbox_to_anchor=(1, 0.5))
        ax1.grid(True)
        
        ax2.set_title('Offline Plan: Joint Velocities')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.legend(loc='lower left', bbox_to_anchor=(1, 0.5))
        ax2.grid(True)
        
        plt.tight_layout()
        self.get_logger().info('Displaying plot. Close window to receive the next trajectory...')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = TrajectoryPlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
