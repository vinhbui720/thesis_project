#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
import threading

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        # Publish commands to the ROS 2 Joint Trajectory Controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/gantry_controller/joint_trajectory',
            10
        )

    def send_command(self, x_pos, z_pos):
        msg = JointTrajectory()
        # Ensure these names match your URDF and controller config
        msg.joint_names = ['joint_x', 'joint_z']
        
        point = JointTrajectoryPoint()
        # Send the position in METERS
        point.positions = [float(x_pos), float(z_pos)]
        # Tell the controller to complete this movement in 1.0 seconds
        point.time_from_start = Duration(sec=1, nanosec=0) 
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: X={x_pos}m, Z={z_pos}m')


def start_ros_thread(node):
    """Spins ROS 2 node in a background thread so it doesn't freeze Tkinter."""
    rclpy.spin(node)


class RobotGUI:
    def __init__(self, master, ros_node):
        self.master = master
        self.ros_node = ros_node
        self.master.title("2-Axis Robot Command Center")
        
        # X-Axis Controls (Updated to -0.28 to 0.0)
        tk.Label(master, text="X-Axis Target Position (m): [-0.28 to 0.0]", font=("Arial", 10, "bold")).pack(pady=(10, 0))
        self.x_slider = tk.Scale(master, from_=-0.28, to=0.0, resolution=0.01, orient=tk.HORIZONTAL, length=300)
        self.x_slider.pack()
        
        # Z-Axis Controls (Remains -0.06 to 0.0)
        tk.Label(master, text="Z-Axis Target Position (m): [-0.06 to 0.0]", font=("Arial", 10, "bold")).pack(pady=(10, 0))
        self.z_slider = tk.Scale(master, from_=-0.06, to=0.0, resolution=0.005, orient=tk.HORIZONTAL, length=300)
        self.z_slider.pack()
        
        # Send Button
        self.send_btn = tk.Button(master, text="SEND TRAJECTORY COMMAND", command=self.on_send, bg="#28a745", fg="white", font=("Arial", 10, "bold"))
        self.send_btn.pack(pady=20)

    def on_send(self):
        x = self.x_slider.get()
        z = self.z_slider.get()
        self.ros_node.send_command(x, z)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotGuiNode()
    
    # Run the ROS 2 event loop in a daemon thread
    ros_thread = threading.Thread(target=start_ros_thread, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # Initialize and run the Tkinter GUI
    root = tk.Tk()
    root.geometry("400x250")
    app = RobotGUI(root, ros_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()