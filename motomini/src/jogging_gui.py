#!/usr/bin/env python3

import threading
import tkinter as tk
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class JoggingGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("jogging_gui")

        self.declare_parameter("publish_rate_hz", 40.0)
        self.declare_parameter("max_linear_speed", 0.06)
        self.declare_parameter("max_angular_speed", 0.8)

        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.max_linear_speed = max(0.0, float(self.get_parameter("max_linear_speed").value))
        self.max_angular_speed = max(0.0, float(self.get_parameter("max_angular_speed").value))

        self.speed_scale = 0.5
        self.pressed: Dict[str, float] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        self.cmd_pub = self.create_publisher(Twist, "/smooth_jogging/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_cmd)

        self.get_logger().info("Jogging GUI online.")
        self.get_logger().info("XYZ keys: +x:1 +y:2 +z:3  -x:q -y:w -z:e")
        self.get_logger().info("RPY keys: +roll:u -roll:o +pitch:i -pitch:k +yaw:j -yaw:l")
        self.get_logger().info("Speed keys: increase:0 decrease:p stop:x")

    def set_axis(self, axis: str, value: float) -> None:
        if axis in self.pressed:
            self.pressed[axis] = value
            # Publish immediately so short key/button taps are not missed.
            self._publish_cmd()

    def stop_all(self) -> None:
        for key in self.pressed:
            self.pressed[key] = 0.0
        self._publish_cmd()

    def adjust_scale(self, delta: float) -> None:
        self.speed_scale = min(1.0, max(0.0, self.speed_scale + delta))
        self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")

    def _publish_cmd(self) -> None:
        msg = Twist()
        lin = self.max_linear_speed * self.speed_scale
        ang = self.max_angular_speed * self.speed_scale

        msg.linear.x = self.pressed["x"] * lin
        msg.linear.y = self.pressed["y"] * lin
        msg.linear.z = self.pressed["z"] * lin

        msg.angular.x = self.pressed["roll"] * ang
        msg.angular.y = self.pressed["pitch"] * ang
        msg.angular.z = self.pressed["yaw"] * ang
        self.cmd_pub.publish(msg)


class JoggingGui:
    def __init__(self, node: JoggingGuiNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("MotoMini Jogging GUI")
        self.root.geometry("520x420")

        self._build_widgets()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self) -> None:
        title = tk.Label(self.root, text="MotoMini Velocity Jogging", font=("Helvetica", 16, "bold"))
        title.pack(pady=8)

        tip = tk.Label(
            self.root,
            text="Hold button or hotkey to move. Release to stop that axis.",
            font=("Helvetica", 10),
        )
        tip.pack(pady=4)

        keymap = tk.Label(
            self.root,
            text=(
                "XYZ: 1/q (x), 2/w (y), 3/e (z)\n"
                "RPY: u/o (roll), i/k (pitch), j/l (yaw)\n"
                "Speed: 0 increase, p decrease, x stop"
            ),
            justify="left",
            font=("Courier", 10),
        )
        keymap.pack(pady=6)

        speed_frame = tk.Frame(self.root)
        speed_frame.pack(pady=8)
        tk.Label(speed_frame, text="Speed scale").pack(side=tk.LEFT, padx=8)
        self.speed_var = tk.DoubleVar(value=self.node.speed_scale)
        speed = tk.Scale(
            speed_frame,
            variable=self.speed_var,
            from_=0.0,
            to=1.0,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=300,
            command=self._on_speed_scale,
        )
        speed.pack(side=tk.LEFT)

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

        stop = tk.Button(self.root, text="STOP (x)", width=20, bg="#cc3333", fg="white", command=self.node.stop_all)
        stop.pack(pady=8)

    def _add_axis_button(self, parent, label: str, row: int, col: int, axis: str, value: float) -> None:
        btn = tk.Button(parent, text=label, width=18)
        btn.grid(row=row, column=col, padx=6, pady=4)
        btn.bind("<ButtonPress-1>", lambda _evt, a=axis, v=value: self.node.set_axis(a, v))
        btn.bind("<ButtonRelease-1>", lambda _evt, a=axis: self.node.set_axis(a, 0.0))

    def _on_speed_scale(self, _value: str) -> None:
        self.node.speed_scale = float(self.speed_var.get())

    def _bind_keys(self) -> None:
        self._bind_hold_key("1", "x", 1.0)
        self._bind_hold_key("q", "x", -1.0)
        self._bind_hold_key("2", "y", 1.0)
        self._bind_hold_key("w", "y", -1.0)
        self._bind_hold_key("3", "z", 1.0)
        self._bind_hold_key("e", "z", -1.0)

        self._bind_hold_key("u", "roll", 1.0)
        self._bind_hold_key("o", "roll", -1.0)
        self._bind_hold_key("i", "pitch", 1.0)
        self._bind_hold_key("k", "pitch", -1.0)
        self._bind_hold_key("j", "yaw", 1.0)
        self._bind_hold_key("l", "yaw", -1.0)

        self.root.bind_all("<KeyPress-0>", lambda _evt: self._update_scale(0.05))
        self.root.bind_all("<KeyPress-p>", lambda _evt: self._update_scale(-0.05))
        self.root.bind_all("<KeyPress-x>", lambda _evt: self.node.stop_all())

    def _bind_hold_key(self, key: str, axis: str, value: float) -> None:
        self.root.bind_all(
            f"<KeyPress-{key}>",
            lambda _evt, a=axis, v=value: self.node.set_axis(a, v),
        )
        self.root.bind_all(
            f"<KeyRelease-{key}>",
            lambda _evt, a=axis: self.node.set_axis(a, 0.0),
        )

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
    node = JoggingGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui = JoggingGui(node)
        gui.run()
    except tk.TclError as exc:
        node.get_logger().error(f"Unable to start Tk GUI: {exc}")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()