#!/usr/bin/env python3
"""
Grinding Target Publisher — with GUI.

Subscribe to:
  - /trajectory_waypoints (PoseArray): trajectory points in world frame
  - /pick_point (Pose):               EE reference pose at pick

Stores all computed target poses, then lets the user:
  - choose how many poses to send per publish
  - press Publish repeatedly — each press sends the next N poses
  - reset the cursor back to the start
"""

import sys
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QGroupBox, QProgressBar,
    QGridLayout,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject


# ─────────────────────────────────────────────
# ROS signals bridge (thread-safe Qt signals)
# ─────────────────────────────────────────────

class RosSignals(QObject):
    data_ready = pyqtSignal(int)   # emitted when target list is (re)built; arg = total count


# ─────────────────────────────────────────────
# ROS node — data only, no auto-publishing
# ─────────────────────────────────────────────

class GrindingDataNode(Node):

    def __init__(self, signals: RosSignals):
        super().__init__("grinding_target_publisher_node")

        self.signals = signals

        # Publisher
        self.target_pub = self.create_publisher(PoseArray, "/target_poses", 10)

        # Raw data
        self._trajectory_waypoints: PoseArray | None = None
        self._ee_position: np.ndarray | None = None

        # Computed targets (full list)
        self.all_targets: list[Pose] = []
        self._lock = threading.Lock()

        # Subscriptions
        self.create_subscription(PoseArray, "/trajectory_waypoints", self._traj_cb, 10)
        self.create_subscription(Pose,      "/pick_point",           self._pick_cb, 10)

        self.get_logger().info("Grinding Target Publisher node started — waiting for data…")

    # ── callbacks ──────────────────────────────

    def _pick_cb(self, msg: Pose):
        self._ee_position = np.array([msg.position.x, msg.position.y, msg.position.z])
        self._try_build()

    def _traj_cb(self, msg: PoseArray):
        self._trajectory_waypoints = msg
        self._try_build()

    # ── compute ────────────────────────────────

    def _try_build(self):
        """Rebuild all_targets whenever both sources are available."""
        if self._trajectory_waypoints is None or self._ee_position is None:
            return

        targets: list[Pose] = []
        for pose in self._trajectory_waypoints.poses:
            target = Pose()
            target.position.x = pose.position.x
            target.position.y = pose.position.y
            target.position.z = pose.position.z
            target.orientation = pose.orientation
            targets.append(target)

        with self._lock:
            self.all_targets = targets

        self.get_logger().info(
            f"Target list built: {len(targets)} poses "
            f"(EE ref: [{self._ee_position[0]:.4f}, "
            f"{self._ee_position[1]:.4f}, {self._ee_position[2]:.4f}])"
        )
        self.signals.data_ready.emit(len(targets))

    # ── publish slice ───────────────────────────

    def publish_slice(self, start: int, count: int) -> int:
        """
        Publish `count` poses starting from index `start`.
        Returns the number of poses actually published (may be < count at end).
        """
        with self._lock:
            targets = self.all_targets

        if not targets:
            return 0

        end    = min(start + count, len(targets))
        slice_ = targets[start:end]

        msg             = PoseArray()
        msg.header.frame_id = (
            self._trajectory_waypoints.header.frame_id
            if self._trajectory_waypoints else "world"
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses        = slice_

        self.target_pub.publish(msg)
        self.get_logger().info(f"Published poses [{start}…{end - 1}] ({len(slice_)} pts)")
        return len(slice_)


# ─────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────

class GrindingGUI(QWidget):

    def __init__(self, node: GrindingDataNode, signals: RosSignals):
        super().__init__()
        self.node   = node
        self.cursor = 0
        self.total  = 0

        self._build_ui()
        self._refresh()

        signals.data_ready.connect(self._on_data_ready)

    # ── layout ─────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle("Grinding Target Publisher")
        self.setMinimumWidth(380)

        root = QVBoxLayout(self)
        root.setSpacing(10)

        # Status group
        status_box = QGroupBox("Status")
        sg = QGridLayout(status_box)
        self.lbl_total  = QLabel("—")
        self.lbl_cursor = QLabel("—")
        self.lbl_remain = QLabel("—")
        sg.addWidget(QLabel("Total poses:"),  0, 0); sg.addWidget(self.lbl_total,  0, 1)
        sg.addWidget(QLabel("Next index:"),   1, 0); sg.addWidget(self.lbl_cursor, 1, 1)
        sg.addWidget(QLabel("Remaining:"),    2, 0); sg.addWidget(self.lbl_remain, 2, 1)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        sg.addWidget(self.progress, 3, 0, 1, 2)
        root.addWidget(status_box)

        # Control group
        ctrl_box = QGroupBox("Publish control")
        cg = QVBoxLayout(ctrl_box)

        row = QHBoxLayout()
        row.addWidget(QLabel("Poses per publish:"))
        self.spin_count = QSpinBox()
        self.spin_count.setRange(1, 9999)
        self.spin_count.setValue(5)
        self.spin_count.setMinimumWidth(80)
        row.addWidget(self.spin_count)
        row.addStretch()
        cg.addLayout(row)

        btn_row = QHBoxLayout()
        self.btn_publish = QPushButton("Publish next N")
        self.btn_publish.setEnabled(False)
        self.btn_publish.setMinimumHeight(36)
        self.btn_publish.clicked.connect(self._on_publish)

        self.btn_reset = QPushButton("Reset cursor")
        self.btn_reset.setMinimumHeight(36)
        self.btn_reset.clicked.connect(self._on_reset)

        btn_row.addWidget(self.btn_publish)
        btn_row.addWidget(self.btn_reset)
        cg.addLayout(btn_row)

        self.lbl_last = QLabel("")
        self.lbl_last.setAlignment(Qt.AlignCenter)
        cg.addWidget(self.lbl_last)

        root.addWidget(ctrl_box)

    # ── slots ───────────────────────────────────

    def _on_data_ready(self, total: int):
        self.total  = total
        self.cursor = 0
        self._refresh()

    def _on_publish(self):
        if self.cursor >= self.total:
            self.lbl_last.setText("All poses already sent — reset first.")
            return

        n    = self.spin_count.value()
        sent = self.node.publish_slice(self.cursor, n)
        end  = self.cursor + sent
        self.lbl_last.setText(f"Sent poses [{self.cursor} … {end - 1}]  ({sent} pts)")
        self.cursor = end
        self._refresh()

    def _on_reset(self):
        self.cursor = 0
        self.lbl_last.setText("Cursor reset.")
        self._refresh()

    # ── helpers ─────────────────────────────────

    def _refresh(self):
        remaining = max(0, self.total - self.cursor)
        self.lbl_total.setText(str(self.total))
        self.lbl_cursor.setText(str(self.cursor))
        self.lbl_remain.setText(str(remaining))
        self.progress.setRange(0, max(1, self.total))
        self.progress.setValue(self.cursor)
        self.btn_publish.setEnabled(self.total > 0 and self.cursor < self.total)


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    signals = RosSignals()
    node    = GrindingDataNode(signals)

    # Spin ROS in a background daemon thread
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    gui = GrindingGUI(node, signals)
    gui.show()

    exit_code = app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
