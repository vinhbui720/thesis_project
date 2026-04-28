#!/usr/bin/env python3

# Pose-based jogging GUI for motomini_feedback_stream.
#
# Maintains an internal target pose seeded from the current EE pose (via TF)
# and nudges it while keys/buttons are held. Streams PoseStamped to
# /user_defined/desired_path_point so the feedback controller chases it.

import math
import threading
import tkinter as tk
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class PoseJoggingNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_jogging_gui")

        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("max_linear_speed", 0.05)   # [m/s] when scale=1
        self.declare_parameter("max_angular_speed", 0.6)   # [rad/s] when scale=1

        self.base_frame: str = self.get_parameter("base_frame").value
        self.ee_frame: str = self.get_parameter("ee_frame").value
        self.publish_rate_hz: float = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.max_linear_speed: float = max(0.0, float(self.get_parameter("max_linear_speed").value))
        self.max_angular_speed: float = max(0.0, float(self.get_parameter("max_angular_speed").value))

        self.dt: float = 1.0 / self.publish_rate_hz
        self.speed_scale: float = 0.5

        self.pressed: Dict[str, float] = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        }

        # Target pose (seeded from TF on first sync). Quaternion is [x,y,z,w].
        self.target_pos: Optional[List[float]] = None
        self.target_quat: Optional[List[float]] = None
        self._target_lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pub = self.create_publisher(PoseStamped, "/user_defined/desired_path_point", 10)
        self.init_pose_pub = self.create_publisher(PoseStamped, "/pose_following/init_pose", 10)

        self.start_cli = self.create_client(Trigger, "/pose_following/start")
        self.stop_cli = self.create_client(Trigger, "/pose_following/stop")
        self.init_start_cli = self.create_client(Trigger, "/pose_following/init_start")

        self.timer = self.create_timer(self.dt, self._on_tick)

        self.get_logger().info(
            f"pose_jogging_gui ready — base={self.base_frame} ee={self.ee_frame} "
            f"rate={self.publish_rate_hz:.0f} Hz")
        self.get_logger().info(
            "XYZ keys: +x:1 +y:2 +z:3  -x:q -y:w -z:e   "
            "RPY keys: +roll:u -roll:o +pitch:i -pitch:k +yaw:j -yaw:l   "
            "Speed: 0/p   Stop: x")

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------
    def sync_target_to_current(self) -> bool:
        """Read current EE pose from TF and copy it into the target."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF {self.base_frame}->{self.ee_frame} unavailable: {ex}")
            return False
        with self._target_lock:
            self.target_pos = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
            self.target_quat = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]
        return True

    # ------------------------------------------------------------------
    # GUI hooks
    # ------------------------------------------------------------------
    def set_axis(self, axis: str, value: float) -> None:
        if axis in self.pressed:
            self.pressed[axis] = value

    def stop_all(self) -> None:
        for k in self.pressed:
            self.pressed[k] = 0.0

    def adjust_scale(self, delta: float) -> None:
        self.speed_scale = min(1.0, max(0.0, self.speed_scale + delta))
        self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")

    # ------------------------------------------------------------------
    # Service calls (non-blocking)
    # ------------------------------------------------------------------
    def call_start(self) -> None:
        if not self.start_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("/pose_following/start unavailable")
            return
        self.start_cli.call_async(Trigger.Request())

    def call_stop(self) -> None:
        if not self.stop_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("/pose_following/stop unavailable")
            return
        self.stop_cli.call_async(Trigger.Request())
        self.stop_all()

    def call_init_to_current(self) -> None:
        """Latch current EE pose as init target and trigger init move."""
        if not self.sync_target_to_current():
            return
        msg = self._make_pose_msg()
        if msg is None:
            return
        self.init_pose_pub.publish(msg)
        if not self.init_start_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("/pose_following/init_start unavailable")
            return
        self.init_start_cli.call_async(Trigger.Request())

    # ------------------------------------------------------------------
    # Internal: build PoseStamped, advance target on tick
    # ------------------------------------------------------------------
    def _make_pose_msg(self) -> Optional[PoseStamped]:
        with self._target_lock:
            if self.target_pos is None or self.target_quat is None:
                return None
            pos = list(self.target_pos)
            quat = list(self.target_quat)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _on_tick(self) -> None:
        # Seed lazily so the GUI is robust to TF coming up later.
        with self._target_lock:
            seeded = self.target_pos is not None
        if not seeded:
            self.sync_target_to_current()
            return

        lin = self.max_linear_speed * self.speed_scale * self.dt
        ang = self.max_angular_speed * self.speed_scale * self.dt

        lin_vec = [self.pressed["x"], self.pressed["y"], self.pressed["z"]]
        lin_norm = math.sqrt(sum(v * v for v in lin_vec))
        if lin_norm > 1.0:
            lin_vec = [v / lin_norm for v in lin_vec]

        ang_vec = [self.pressed["roll"], self.pressed["pitch"], self.pressed["yaw"]]
        ang_norm = math.sqrt(sum(v * v for v in ang_vec))
        if ang_norm > 1.0:
            ang_vec = [v / ang_norm for v in ang_vec]

        with self._target_lock:
            self.target_pos[0] += lin_vec[0] * lin
            self.target_pos[1] += lin_vec[1] * lin
            self.target_pos[2] += lin_vec[2] * lin
            if any(v != 0.0 for v in ang_vec):
                droll = ang_vec[0] * ang
                dpitch = ang_vec[1] * ang
                dyaw = ang_vec[2] * ang
                self.target_quat = self._apply_rpy_delta(self.target_quat, droll, dpitch, dyaw)

        msg = self._make_pose_msg()
        if msg is not None:
            self.target_pub.publish(msg)

    @staticmethod
    def _apply_rpy_delta(quat: List[float], droll: float, dpitch: float, dyaw: float) -> List[float]:
        # Build delta quaternion from RPY (XYZ intrinsic), then post-multiply
        # in body frame so rotations follow the tool, not the world.
        cr, sr = math.cos(droll * 0.5), math.sin(droll * 0.5)
        cp, sp = math.cos(dpitch * 0.5), math.sin(dpitch * 0.5)
        cy, sy = math.cos(dyaw * 0.5), math.sin(dyaw * 0.5)
        dx = sr * cp * cy - cr * sp * sy
        dy = cr * sp * cy + sr * cp * sy
        dz = cr * cp * sy - sr * sp * cy
        dw = cr * cp * cy + sr * sp * sy

        x1, y1, z1, w1 = quat
        rx = w1 * dx + x1 * dw + y1 * dz - z1 * dy
        ry = w1 * dy - x1 * dz + y1 * dw + z1 * dx
        rz = w1 * dz + x1 * dy - y1 * dx + z1 * dw
        rw = w1 * dw - x1 * dx - y1 * dy - z1 * dz
        n = math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw)
        if n < 1e-12:
            return [x1, y1, z1, w1]
        return [rx / n, ry / n, rz / n, rw / n]


class PoseJoggingGui:
    def __init__(self, node: PoseJoggingNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("MotoMini Pose Jogging GUI")
        self.root.geometry("560x540")

        self._build_widgets()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self) -> None:
        title = tk.Label(self.root, text="MotoMini Pose Jogging",
                         font=("Helvetica", 16, "bold"))
        title.pack(pady=8)

        tip = tk.Label(self.root,
                       text=("Streams target pose to /user_defined/desired_path_point.\n"
                             "Hold buttons or hotkeys to nudge the target."),
                       font=("Helvetica", 10), justify="center")
        tip.pack(pady=4)

        keymap = tk.Label(
            self.root,
            text=("XYZ: 1/q (x), 2/w (y), 3/e (z)\n"
                  "RPY: u/o (roll), i/k (pitch), j/l (yaw)\n"
                  "Speed: 0 increase, p decrease, x stop"),
            justify="left", font=("Courier", 10))
        keymap.pack(pady=6)

        # Service buttons
        svc_frame = tk.Frame(self.root)
        svc_frame.pack(pady=6)
        tk.Button(svc_frame, text="Sync Target → EE", width=18,
                  command=self.node.sync_target_to_current).pack(side=tk.LEFT, padx=4)
        tk.Button(svc_frame, text="Init @ Current", width=14,
                  command=self.node.call_init_to_current).pack(side=tk.LEFT, padx=4)
        tk.Button(svc_frame, text="Start", width=10,
                  command=self.node.call_start).pack(side=tk.LEFT, padx=4)
        tk.Button(svc_frame, text="Stop", width=10, bg="#cc6633", fg="white",
                  command=self.node.call_stop).pack(side=tk.LEFT, padx=4)

        # Speed scale
        speed_frame = tk.Frame(self.root)
        speed_frame.pack(pady=8)
        tk.Label(speed_frame, text="Speed scale").pack(side=tk.LEFT, padx=8)
        self.speed_var = tk.DoubleVar(value=self.node.speed_scale)
        tk.Scale(speed_frame, variable=self.speed_var, from_=0.0, to=1.0,
                 resolution=0.01, orient=tk.HORIZONTAL, length=320,
                 command=self._on_speed_scale).pack(side=tk.LEFT)

        grid = tk.Frame(self.root)
        grid.pack(pady=10)

        self._add_axis_button(grid, "+X (1)", 0, 0, "x", 1.0)
        self._add_axis_button(grid, "-X (q)", 0, 1, "x", -1.0)
        self._add_axis_button(grid, "+Y (2)", 1, 0, "y", 1.0)
        self._add_axis_button(grid, "-Y (w)", 1, 1, "y", -1.0)
        self._add_axis_button(grid, "+Z (3)", 2, 0, "z", 1.0)
        self._add_axis_button(grid, "-Z (e)", 2, 1, "z", -1.0)
        self._add_axis_button(grid, "+Roll (u)", 3, 0, "roll", 1.0)
        self._add_axis_button(grid, "-Roll (o)", 3, 1, "roll", -1.0)
        self._add_axis_button(grid, "+Pitch (i)", 4, 0, "pitch", 1.0)
        self._add_axis_button(grid, "-Pitch (k)", 4, 1, "pitch", -1.0)
        self._add_axis_button(grid, "+Yaw (j)", 5, 0, "yaw", 1.0)
        self._add_axis_button(grid, "-Yaw (l)", 5, 1, "yaw", -1.0)

        tk.Button(self.root, text="STOP (x)", width=22, bg="#cc3333", fg="white",
                  command=self.node.call_stop).pack(pady=8)

    def _add_axis_button(self, parent, label: str, row: int, col: int,
                         axis: str, value: float) -> None:
        btn = tk.Button(parent, text=label, width=18)
        btn.grid(row=row, column=col, padx=6, pady=4)
        btn.bind("<ButtonPress-1>", lambda _e, a=axis, v=value: self.node.set_axis(a, v))
        btn.bind("<ButtonRelease-1>", lambda _e, a=axis: self.node.set_axis(a, 0.0))

    def _on_speed_scale(self, _value: str) -> None:
        self.node.speed_scale = float(self.speed_var.get())

    def _bind_keys(self) -> None:
        for key, axis, val in [
            ("1", "x", 1.0), ("q", "x", -1.0),
            ("2", "y", 1.0), ("w", "y", -1.0),
            ("3", "z", 1.0), ("e", "z", -1.0),
            ("u", "roll", 1.0), ("o", "roll", -1.0),
            ("i", "pitch", 1.0), ("k", "pitch", -1.0),
            ("j", "yaw", 1.0), ("l", "yaw", -1.0),
        ]:
            self._bind_hold_key(key, axis, val)

        self.root.bind_all("<KeyPress-0>", lambda _e: self._update_scale(0.05))
        self.root.bind_all("<KeyPress-p>", lambda _e: self._update_scale(-0.05))
        self.root.bind_all("<KeyPress-x>", lambda _e: self.node.call_stop())

    def _bind_hold_key(self, key: str, axis: str, value: float) -> None:
        self.root.bind_all(f"<KeyPress-{key}>",
                           lambda _e, a=axis, v=value: self.node.set_axis(a, v))
        self.root.bind_all(f"<KeyRelease-{key}>",
                           lambda _e, a=axis: self.node.set_axis(a, 0.0))

    def _update_scale(self, delta: float) -> None:
        self.node.adjust_scale(delta)
        self.speed_var.set(self.node.speed_scale)

    def _on_close(self) -> None:
        self.node.stop_all()
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = PoseJoggingNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui = PoseJoggingGui(node)
        gui.run()
    except tk.TclError as exc:
        node.get_logger().error(f"Unable to start Tk GUI: {exc}")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
