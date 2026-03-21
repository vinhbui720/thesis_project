#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import tkinter as tk
import threading
import time

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        # Publish commands to the ROS 2 Joint Trajectory Controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/gantry_controller/joint_trajectory',
            10
        )
        
        # Subscribe to the live hardware feedback from the Modbus Bridge
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Thread-safe dictionary to store the latest feedback
        self.latest_fb = {'x_pos': 0.0, 'x_vel': 0.0, 'z_pos': 0.0, 'z_vel': 0.0}
        
        # Variables for tracking connection speed (Hz)
        self.last_msg_time = 0.0
        self.current_hz = 0.0

    def joint_state_callback(self, msg):
        current_time = time.time()
        if self.last_msg_time > 0:
            dt = current_time - self.last_msg_time
            if dt > 0:
                inst_hz = 1.0 / dt
                # Exponential moving average to smooth the reading
                self.current_hz = (self.current_hz * 0.8) + (inst_hz * 0.2)
        self.last_msg_time = current_time

        try:
            x_idx = msg.name.index('joint_x')
            z_idx = msg.name.index('joint_z')
            
            # Save the latest data
            self.latest_fb['x_pos'] = msg.position[x_idx]
            self.latest_fb['x_vel'] = msg.velocity[x_idx]
            self.latest_fb['z_pos'] = msg.position[z_idx]
            self.latest_fb['z_vel'] = msg.velocity[z_idx]
        except ValueError:
            pass # Ignore if the joint names don't match

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
        
        # --- COMMAND SECTION ---
        tk.Label(master, text="TARGET COMMANDS", font=("Arial", 11, "bold"), fg="#333333").pack(pady=(10, 0))
        
        # X-Axis Controls
        tk.Label(master, text="X-Axis Target (m): [-0.28 to 0.0]", font=("Arial", 9)).pack()
        self.x_slider = tk.Scale(master, from_=-0.28, to=0.0, resolution=0.01, orient=tk.HORIZONTAL, length=300)
        self.x_slider.pack()
        
        # Z-Axis Controls
        tk.Label(master, text="Z-Axis Target (m): [-0.06 to 0.0]", font=("Arial", 9)).pack()
        self.z_slider = tk.Scale(master, from_=-0.06, to=0.0, resolution=0.005, orient=tk.HORIZONTAL, length=300)
        self.z_slider.pack()
        
        # Send Button
        self.send_btn = tk.Button(master, text="SEND TRAJECTORY", command=self.on_send, bg="#28a745", fg="white", font=("Arial", 10, "bold"))
        self.send_btn.pack(pady=10)
        
        # --- FEEDBACK SECTION ---
        tk.Frame(master, height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, padx=20, pady=5)
        tk.Label(master, text="LIVE HARDWARE FEEDBACK", font=("Arial", 11, "bold"), fg="#0056b3").pack(pady=(5, 5))
        
        self.speed_var = tk.StringVar(value="Connection Speed: 0.0 Hz")
        self.speed_label = tk.Label(master, textvariable=self.speed_var, font=("Arial", 10, "bold"), fg="red")
        self.speed_label.pack(pady=(0, 5))
        
        self.fb_x_var = tk.StringVar(value="X Pos: 0.0000 m  |  Vel: 0.0000 m/s")
        tk.Label(master, textvariable=self.fb_x_var, font=("Courier", 10)).pack()
        
        self.fb_z_var = tk.StringVar(value="Z Pos: 0.0000 m  |  Vel: 0.0000 m/s")
        tk.Label(master, textvariable=self.fb_z_var, font=("Courier", 10)).pack()

        # Start the UI polling loop
        self.poll_feedback()

    def on_send(self):
        x = self.x_slider.get()
        z = self.z_slider.get()
        self.ros_node.send_command(x, z)

    def poll_feedback(self):
        """Safely updates the Tkinter UI with the latest data from the ROS 2 thread."""
        fb = self.ros_node.latest_fb
        self.fb_x_var.set(f"X Pos: {fb['x_pos']:7.4f} m  |  Vel: {fb['x_vel']:7.4f} m/s")
        self.fb_z_var.set(f"Z Pos: {fb['z_pos']:7.4f} m  |  Vel: {fb['z_vel']:7.4f} m/s")
        
        # Calculate and display connection speed
        time_since_last = time.time() - self.ros_node.last_msg_time
        if time_since_last > 1.0:
            self.ros_node.current_hz = 0.0 # Timeout if no message for 1 second
            
        hz = self.ros_node.current_hz
        self.speed_var.set(f"Connection Speed: {hz:.1f} Hz")
        
        # Visual color indicator based on health (Target is 20 Hz)
        if hz > 15.0:
            self.speed_label.config(fg="green")
        elif hz > 5.0:
            self.speed_label.config(fg="orange")
        else:
            self.speed_label.config(fg="red")
        
        # Schedule the next UI update in 100 milliseconds (10 Hz)
        self.master.after(100, self.poll_feedback)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotGuiNode()
    
    # Run the ROS 2 event loop in a daemon thread
    ros_thread = threading.Thread(target=start_ros_thread, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # Initialize and run the Tkinter GUI
    root = tk.Tk()
    root.geometry("400x380")
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