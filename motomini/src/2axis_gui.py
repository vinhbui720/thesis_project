#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
import threading


class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')

        # Command publisher → gantry_controller C++ node
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/gantry_controller/joint_trajectory',
            10,
        )

        # Feedback subscriber ← gantry_controller C++ node
        # The first message initialises the sliders; subsequent ones update the
        # live-feedback labels.
        self._initial_pos_set = False
        self._latest_joint_state: JointState | None = None
        self._js_lock = threading.Lock()

        self.js_sub_ = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10,
        )

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        with self._js_lock:
            self._latest_joint_state = msg

    # ── Public API used by the GUI ─────────────────────────────────────────

    def get_latest_joint_state(self) -> JointState | None:
        with self._js_lock:
            return self._latest_joint_state

    def send_command(self, x_pos: float, z_pos: float):
        msg = JointTrajectory()
        msg.joint_names = ['joint_x', 'joint_z']

        point = JointTrajectoryPoint()
        point.positions = [float(x_pos), float(z_pos)]
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command sent → X={x_pos:.4f} m  Z={z_pos:.4f} m')


def _ros_spin(node: Node):
    """Runs rclpy.spin in a daemon thread so Tkinter stays responsive."""
    rclpy.spin(node)


# ═══════════════════════════════════════════════════════════════════════════
#  Tkinter GUI
# ═══════════════════════════════════════════════════════════════════════════

class RobotGUI:
    # Axis limits (metres)
    X_MIN, X_MAX = -0.28, 0.0
    Z_MIN, Z_MAX = -0.06, 0.0

    def __init__(self, master: tk.Tk, ros_node: RobotGuiNode):
        self.master = master
        self.ros_node = ros_node
        self.master.title("2-Axis Gantry Command Centre")
        self.master.resizable(False, False)

        pad = {"padx": 15, "pady": 4}

        # ── Status bar ──────────────────────────────────────────────────────
        self._status_var = tk.StringVar(value="Waiting for controller feedback…")
        status_bar = tk.Label(
            master, textvariable=self._status_var,
            font=("Arial", 9, "italic"), fg="#555555",
        )
        status_bar.pack(**pad)

        # ── X-Axis ──────────────────────────────────────────────────────────
        tk.Label(
            master,
            text=f"X-Axis Target (m)  [{self.X_MIN} → {self.X_MAX}]",
            font=("Arial", 10, "bold"),
        ).pack(**pad)

        self.x_slider = tk.Scale(
            master, from_=self.X_MIN, to=self.X_MAX,
            resolution=0.001, orient=tk.HORIZONTAL, length=360,
            digits=4,
        )
        self.x_slider.pack(**pad)

        self._x_fb_var = tk.StringVar(value="Feedback: –")
        tk.Label(master, textvariable=self._x_fb_var,
                 font=("Arial", 9), fg="#1a6ea8").pack()

        # ── Z-Axis ──────────────────────────────────────────────────────────
        tk.Label(
            master,
            text=f"Z-Axis Target (m)  [{self.Z_MIN} → {self.Z_MAX}]",
            font=("Arial", 10, "bold"),
        ).pack(**pad)

        self.z_slider = tk.Scale(
            master, from_=self.Z_MIN, to=self.Z_MAX,
            resolution=0.001, orient=tk.HORIZONTAL, length=360,
            digits=4,
        )
        self.z_slider.pack(**pad)

        self._z_fb_var = tk.StringVar(value="Feedback: –")
        tk.Label(master, textvariable=self._z_fb_var,
                 font=("Arial", 9), fg="#1a6ea8").pack()

        # ── Send button ─────────────────────────────────────────────────────
        self.send_btn = tk.Button(
            master, text="SEND COMMAND",
            command=self._on_send,
            bg="#28a745", fg="white",
            font=("Arial", 11, "bold"),
            width=22,
        )
        self.send_btn.pack(pady=14)

        # ── Start the feedback polling loop (runs on Tk main thread) ────────
        self._initial_synced = False
        self._poll_feedback()

    # ── Internal helpers ────────────────────────────────────────────────────

    def _poll_feedback(self):
        """Called every 100 ms on the Tk main thread to safely update widgets."""
        js = self.ros_node.get_latest_joint_state()

        if js is not None and len(js.position) >= 2:
            x_fb = js.position[0]
            z_fb = js.position[1]

            # ── One-time initialisation: set sliders to actual current pos ──
            if not self._initial_synced:
                # Clamp to slider range before applying
                x_init = max(self.X_MIN, min(self.X_MAX, x_fb))
                z_init = max(self.Z_MIN, min(self.Z_MAX, z_fb))
                self.x_slider.set(x_init)
                self.z_slider.set(z_init)
                self._initial_synced = True
                self._status_var.set("Synced with controller ✓")

            # ── Live feedback labels ────────────────────────────────────────
            self._x_fb_var.set(f"Feedback: {x_fb:+.4f} m")
            self._z_fb_var.set(f"Feedback: {z_fb:+.4f} m")

        # Reschedule
        self.master.after(100, self._poll_feedback)

    def _on_send(self):
        x = self.x_slider.get()
        z = self.z_slider.get()
        self.ros_node.send_command(x, z)
        self._status_var.set(f"Sent → X={x:+.4f} m  Z={z:+.4f} m")


# ═══════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotGuiNode()

    # Spin ROS 2 in a daemon thread
    threading.Thread(target=_ros_spin, args=(ros_node,), daemon=True).start()

    root = tk.Tk()
    root.geometry("420x420")
    RobotGUI(root, ros_node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
