import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk
import math
import threading

class GuiTesterNode(Node):
    def __init__(self):
        super().__init__('object_tf_gui_tester')
        
        # Publishers matching your C++ node subscriptions
        self.pose_pub = self.create_publisher(PoseStamped, '/target_object_pose', 10)
        self.attach_pub = self.create_publisher(Bool, '/object_attach_signal', 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        # Set Position
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        # Convert Euler (degrees) to Quaternion
        r = math.radians(float(roll))
        p = math.radians(float(pitch))
        y_rad = math.radians(float(yaw))
        
        cy = math.cos(y_rad * 0.5)
        sy = math.sin(y_rad * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy

        self.pose_pub.publish(msg)

    def publish_attach_signal(self, is_attached):
        msg = Bool()
        msg.data = bool(is_attached)
        self.attach_pub.publish(msg)
        state = "Attached (TF Disabled)" if is_attached else "Detached (TF Enabled)"
        self.get_logger().info(f"Published Signal: {state}")


class ObjectTestGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("Continuous Object TF Tester")
        self.root.geometry("400x550")

        # --- Position Sliders (Meters) ---
        tk.Label(root, text="Position (Meters):", font=("Arial", 10, "bold")).pack(pady=(10, 0))
        
        # Ranges set from -2.0m to 2.0m. Adjust if your robot workspace is larger.
        self.scale_x = self.create_slider(root, "X", -2.0, 2.0, 0.01, 0.5)
        self.scale_y = self.create_slider(root, "Y", -2.0, 2.0, 0.01, 0.0)
        self.scale_z = self.create_slider(root, "Z", -2.0, 2.0, 0.01, 0.1)

        ttk.Separator(root, orient='horizontal').pack(fill='x', pady=10)

        # --- Rotation Sliders (Degrees) ---
        tk.Label(root, text="Rotation (Degrees):", font=("Arial", 10, "bold")).pack(pady=(5, 0))
        
        self.scale_roll = self.create_slider(root, "Roll", -180, 180, 1, 0.0)
        self.scale_pitch = self.create_slider(root, "Pitch", -180, 180, 1, 0.0)
        self.scale_yaw = self.create_slider(root, "Yaw", -180, 180, 1, 0.0)

        ttk.Separator(root, orient='horizontal').pack(fill='x', pady=10)

        # --- Attach/Detach Signal Buttons ---
        tk.Label(root, text="Attach Signal Controls:", font=("Arial", 10, "bold")).pack(pady=(5, 0))
        
        # Frame to hold buttons side-by-side
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)
        
        tk.Button(btn_frame, text="Send Attach (True)", bg="lightcoral", width=15,
                  command=lambda: self.node.publish_attach_signal(True)).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Send Detach (False)", bg="lightgreen", width=15,
                  command=lambda: self.node.publish_attach_signal(False)).pack(side=tk.LEFT, padx=10)

    def create_slider(self, parent, label_text, min_val, max_val, res, default_val):
        frame = tk.Frame(parent)
        frame.pack(fill='x', padx=20, pady=2)
        
        tk.Label(frame, text=label_text, width=5, anchor='w').pack(side=tk.LEFT)
        
        # command=self.on_slider_change triggers automatically whenever the slider moves
        scale = tk.Scale(frame, from_=min_val, to=max_val, resolution=res, 
                         orient=tk.HORIZONTAL, length=250, command=self.on_slider_change)
        scale.set(default_val)
        scale.pack(side=tk.RIGHT)
        return scale

    def on_slider_change(self, event=None):
        # Read the current state of all sliders simultaneously
        x = self.scale_x.get()
        y = self.scale_y.get()
        z = self.scale_z.get()
        roll = self.scale_roll.get()
        pitch = self.scale_pitch.get()
        yaw = self.scale_yaw.get()
        
        # Publish the new combined pose
        self.node.publish_pose(x, y, z, roll, pitch, yaw)

def main():
    rclpy.init()
    node = GuiTesterNode()

    # Run ROS 2 node in a separate thread so it doesn't freeze the GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Setup and run Tkinter GUI in the main thread
    root = tk.Tk()
    app = ObjectTestGUI(root, node)
    root.mainloop()

    # Cleanup when GUI is closed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()