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
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Empty

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QGroupBox, QProgressBar,
    QGridLayout, QLineEdit, QMessageBox, QTextEdit,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject


# ─────────────────────────────────────────────
# ROS signals bridge (thread-safe Qt signals)
# ─────────────────────────────────────────────

class RosSignals(QObject):
    data_ready = pyqtSignal(int)   # emitted when target list is (re)built; arg = total count
    pick_point_updated = pyqtSignal(dict)  # emitted when pick_point is updated; arg = pose dict


# ─────────────────────────────────────────────
# ROS node — data only, no auto-publishing
# ─────────────────────────────────────────────

class GrindingDataNode(Node):

    def __init__(self, signals: RosSignals):
        super().__init__("grinding_target_publisher_node")

        self.signals = signals

        # Publisher
        self.target_pub = self.create_publisher(PoseArray, "/target_poses", 10)
        self.get_traj_pub = self.create_publisher(Empty, "/get_trajectory", 10)

        # Raw data
        self._trajectory_waypoints: PoseArray | None = None
        self._ee_position: np.ndarray | None = None
        self._ee_orientation = None
        
        # Transformation matrices (4x4 homogeneous)
        self.transform_matrix: np.ndarray = np.eye(4)  # Legacy: used if no wk_tip set
        self.wk_tip_matrix: np.ndarray | None = None   # T_world^wk_tip

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
        self._ee_orientation = msg.orientation
        
        # Emit signal with full pose data
        pose_dict = {
            'pos_x': msg.position.x,
            'pos_y': msg.position.y,
            'pos_z': msg.position.z,
            'quat_x': msg.orientation.x,
            'quat_y': msg.orientation.y,
            'quat_z': msg.orientation.z,
            'quat_w': msg.orientation.w,
        }
        self.signals.pick_point_updated.emit(pose_dict)
        
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

    def clear_trajectory_buffer(self):
        """Clear the trajectory waypoints buffer."""
        with self._lock:
            self._trajectory_waypoints = None
            self.all_targets = []
        self.get_logger().info("Trajectory buffer cleared")
        self.signals.data_ready.emit(0)

    def request_trajectory(self):
        """Publish Empty message to /get_trajectory to request new trajectory."""
        msg = Empty()
        self.get_traj_pub.publish(msg)
        self.get_logger().info("Published Empty message to /get_trajectory")

    # ── transformation utilities ──────────────────

    def _pose_to_transform_matrix(self, pose: Pose) -> np.ndarray:
        """Convert a Pose message to a 4x4 homogeneous transformation matrix."""
        # Extract position
        position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        
        # Extract orientation (quaternion: x, y, z, w)
        quat = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat(quat)
        rotation_matrix = rotation.as_matrix()
        
        # Build 4x4 homogeneous transformation matrix
        transform = np.eye(4)
        transform[0:3, 0:3] = rotation_matrix
        transform[0:3, 3] = position
        
        return transform

    def set_wk_tip_pose(self, pose: Pose):
        """Set the T_world^wk_tip transformation from a Pose message."""
        self.wk_tip_matrix = self._pose_to_transform_matrix(pose)
        self.get_logger().info("wk_tip transformation matrix updated")

    def set_wk_tip_from_xyzquat(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float):
        """Set the T_world^wk_tip transformation from position and quaternion."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.set_wk_tip_pose(pose)

    def set_transform_matrix(self, matrix: np.ndarray):
        """Set the 4x4 transformation matrix for filtering."""
        if matrix.shape != (4, 4):
            self.get_logger().warn(f"Transform matrix must be 4x4, got {matrix.shape}")
            return
        self.transform_matrix = matrix
        self.get_logger().info("Transform matrix updated")

    def _apply_transform_filter(self, pose: Pose) -> Pose:
        """Apply transformation matrix to a pose."""
        # Create homogeneous coordinates [x, y, z, 1]
        point = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            1.0
        ])
        
        # Apply transformation: result = transform_matrix.T @ point
        transformed = self.transform_matrix.T @ point
        
        # Extract position and create new pose
        filtered_pose = Pose()
        filtered_pose.position.x = transformed[0]
        filtered_pose.position.y = transformed[1]
        filtered_pose.position.z = transformed[2]
        filtered_pose.orientation = pose.orientation
        
        return filtered_pose

    def _compute_wk_tip_transform(self, trajectory_pose: Pose, ee_pose: Pose) -> Pose:
        """
        Compute: T = T_world^wk_tip * [T_world^T]^-1 * T_world^e
        
        Args:
            trajectory_pose: Pose from trajectory (T_world^T)
            ee_pose: Pose from pick_point (T_world^e)
        
        Returns:
            Transformed pose (result position with ee orientation preserved)
        """
        if self.wk_tip_matrix is None:
            # If wk_tip not set, return trajectory pose as-is
            return trajectory_pose
        
        # Get transformation matrices
        T_wk_tip = self.wk_tip_matrix
        T_trajectory = self._pose_to_transform_matrix(trajectory_pose)
        T_ee = self._pose_to_transform_matrix(ee_pose)
        
        # Compute: T = T_wk_tip * inv(T_trajectory) * T_ee
        T_trajectory_inv = np.linalg.inv(T_trajectory)
        T_result = T_wk_tip @ T_trajectory_inv @ T_ee
        
        # Extract position from result
        result_pose = Pose()
        result_pose.position.x = T_result[0, 3]
        result_pose.position.y = T_result[1, 3]
        result_pose.position.z = T_result[2, 3]
        result_pose.orientation = ee_pose.orientation  # Preserve ee orientation
        
        return result_pose

    # ── publish slice ───────────────────────────

    def publish_slice(self, start: int, count: int) -> int:
        """
        Publish `count` poses starting from index `start`.
        Applies transformation filter before publishing.
        Returns the number of poses actually published (may be < count at end).
        """
        with self._lock:
            targets = self.all_targets

        if not targets:
            return 0

        end    = min(start + count, len(targets))
        slice_ = targets[start:end]
        
        # Apply filter/transformation to each pose
        filtered_slice = []
        for pose in slice_:
            if self.wk_tip_matrix is not None and self._ee_orientation is not None:
                # Use wk_tip transformation
                ee_pose = Pose()
                ee_pose.position.x = self._ee_position[0]
                ee_pose.position.y = self._ee_position[1]
                ee_pose.position.z = self._ee_position[2]
                ee_pose.orientation = self._ee_orientation
                transformed_pose = self._compute_wk_tip_transform(pose, ee_pose)
                filtered_slice.append(transformed_pose)
            else:
                # Use legacy transform matrix
                filtered_slice.append(self._apply_transform_filter(pose))

        msg             = PoseArray()
        msg.header.frame_id = (
            self._trajectory_waypoints.header.frame_id
            if self._trajectory_waypoints else "world"
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses        = filtered_slice

        self.target_pub.publish(msg)
        if self.wk_tip_matrix is not None:
            self.get_logger().info(f"Published poses [{start}…{end - 1}] ({len(filtered_slice)} pts) — wk_tip transformed")
        else:
            self.get_logger().info(f"Published poses [{start}…{end - 1}] ({len(filtered_slice)} pts) — filtered")
        return len(filtered_slice)


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
        signals.pick_point_updated.connect(self._on_pick_point_updated)

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

        # Pick point display group
        pick_box = QGroupBox("Pick Point (T_world^e)")
        pg = QVBoxLayout(pick_box)
        self.txt_pick_point = QTextEdit()
        self.txt_pick_point.setReadOnly(True)
        self.txt_pick_point.setMaximumHeight(80)
        self.txt_pick_point.setText("Waiting for /pick_point message...")
        pg.addWidget(self.txt_pick_point)
        root.addWidget(pick_box)

        # Trajectory management group
        traj_box = QGroupBox("Trajectory Management")
        tg = QVBoxLayout(traj_box)
        
        btn_row = QHBoxLayout()
        self.btn_request_traj = QPushButton("Request Trajectory")
        self.btn_request_traj.setMinimumHeight(32)
        self.btn_request_traj.clicked.connect(self._on_request_trajectory)
        
        self.btn_clear_traj = QPushButton("Clear Buffer")
        self.btn_clear_traj.setMinimumHeight(32)
        self.btn_clear_traj.clicked.connect(self._on_clear_trajectory)
        
        btn_row.addWidget(self.btn_request_traj)
        btn_row.addWidget(self.btn_clear_traj)
        tg.addLayout(btn_row)
        root.addWidget(traj_box)

        # wk_tip configuration group
        wk_tip_box = QGroupBox("wk_tip Configuration (T_world^wk_tip)")
        wg = QGridLayout(wk_tip_box)
        
        wg.addWidget(QLabel("Pos X:"), 0, 0); self.txt_wk_x = QLineEdit("0.0"); wg.addWidget(self.txt_wk_x, 0, 1)
        wg.addWidget(QLabel("Pos Y:"), 1, 0); self.txt_wk_y = QLineEdit("0.0"); wg.addWidget(self.txt_wk_y, 1, 1)
        wg.addWidget(QLabel("Pos Z:"), 2, 0); self.txt_wk_z = QLineEdit("0.0"); wg.addWidget(self.txt_wk_z, 2, 1)
        
        wg.addWidget(QLabel("Quat X:"), 3, 0); self.txt_wk_qx = QLineEdit("0.0"); wg.addWidget(self.txt_wk_qx, 3, 1)
        wg.addWidget(QLabel("Quat Y:"), 4, 0); self.txt_wk_qy = QLineEdit("0.0"); wg.addWidget(self.txt_wk_qy, 4, 1)
        wg.addWidget(QLabel("Quat Z:"), 5, 0); self.txt_wk_qz = QLineEdit("0.0"); wg.addWidget(self.txt_wk_qz, 5, 1)
        wg.addWidget(QLabel("Quat W:"), 6, 0); self.txt_wk_qw = QLineEdit("1.0"); wg.addWidget(self.txt_wk_qw, 6, 1)
        
        self.btn_set_wk_tip = QPushButton("Set wk_tip")
        self.btn_set_wk_tip.setMinimumHeight(32)
        self.btn_set_wk_tip.clicked.connect(self._on_set_wk_tip)
        wg.addWidget(self.btn_set_wk_tip, 7, 0, 1, 2)
        
        self.lbl_wk_tip_status = QLabel("Not set")
        wg.addWidget(self.lbl_wk_tip_status, 8, 0, 1, 2)
        
        root.addWidget(wk_tip_box)

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

    def _on_pick_point_updated(self, pose_dict: dict):
        """Update the pick_point display when new data arrives."""
        text = "Pick Point (T_world^e):\n"
        text += f"Position: ({pose_dict['pos_x']:.4f}, {pose_dict['pos_y']:.4f}, {pose_dict['pos_z']:.4f})\n"
        text += f"Quaternion: ({pose_dict['quat_x']:.4f}, {pose_dict['quat_y']:.4f}, {pose_dict['quat_z']:.4f}, {pose_dict['quat_w']:.4f})"
        self.txt_pick_point.setText(text)

    def _on_request_trajectory(self):
        """Request a new trajectory by publishing Empty message."""
        self.node.request_trajectory()
        self.lbl_last.setText("Trajectory request sent...")

    def _on_clear_trajectory(self):
        """Clear the trajectory waypoints buffer."""
        self.node.clear_trajectory_buffer()
        self.lbl_last.setText("Trajectory buffer cleared.")

    def _on_set_wk_tip(self):
        """Parse wk_tip input fields and set the transformation."""
        try:
            x  = float(self.txt_wk_x.text())
            y  = float(self.txt_wk_y.text())
            z  = float(self.txt_wk_z.text())
            qx = float(self.txt_wk_qx.text())
            qy = float(self.txt_wk_qy.text())
            qz = float(self.txt_wk_qz.text())
            qw = float(self.txt_wk_qw.text())
            
            self.node.set_wk_tip_from_xyzquat(x, y, z, qx, qy, qz, qw)
            self.lbl_wk_tip_status.setText(f"Set: ({x:.3f}, {y:.3f}, {z:.3f})")
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Invalid input: {e}")
            self.lbl_wk_tip_status.setText("Error parsing input")

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
