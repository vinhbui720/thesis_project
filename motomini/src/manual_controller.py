import sys
import rclpy
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

import transforms3d.euler as euler

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout,
    QPushButton, QLabel, QLineEdit, QListWidget
)

from PyQt5.QtCore import QTimer


class PlanningGUI(Node):

    def __init__(self):

        super().__init__("planning_gui")

        # publishers
        self.pub_targets = self.create_publisher(PoseArray, "/target_poses", 10)
        self.pub_start = self.create_publisher(Bool, "/start", 10)
        self.pub_clear = self.create_publisher(Bool, "/clear_targets", 10)

        # subscribers
        self.sub_status = self.create_subscription(
            String,
            "/optimization_status",
            self.status_callback,
            10
        )

        self.sub_tf = self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            50
        )

        # robot state
        self.current_pose = None
        self.status_text = "Waiting..."

    def status_callback(self, msg):
        self.status_text = msg.data

    def tf_callback(self, msg):

        for t in msg.transforms:

            if t.child_frame_id == "tool0":

                p = t.transform.translation
                q = t.transform.rotation

                quat = [q.w, q.x, q.y, q.z]

                rpy = euler.quat2euler(quat)

                self.current_pose = (
                    p.x, p.y, p.z,
                    rpy[0], rpy[1], rpy[2]
                )


class GUI(QWidget):

    def __init__(self, node):

        super().__init__()

        self.node = node
        self.pose_list = []

        layout = QVBoxLayout()

        # ------------------------
        # STATUS
        # ------------------------

        self.status_label = QLabel("Status: Idle")
        layout.addWidget(self.status_label)

        # ------------------------
        # CURRENT ROBOT POSE
        # ------------------------

        self.robot_pose_label = QLabel("Robot Pose: unknown")
        layout.addWidget(self.robot_pose_label)

        # ------------------------
        # INPUT FIELDS
        # ------------------------

        grid = QGridLayout()

        labels = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        self.inputs = {}

        for i, name in enumerate(labels):

            label = QLabel(name)
            field = QLineEdit("0.0")

            self.inputs[name] = field

            grid.addWidget(label, i, 0)
            grid.addWidget(field, i, 1)

        layout.addLayout(grid)

        # ------------------------
        # BUTTONS
        # ------------------------

        btn_add = QPushButton("Add Pose")
        btn_send = QPushButton("Send To Planner")
        btn_start = QPushButton("Start Planning")
        btn_clear = QPushButton("Clear Buffer")

        layout.addWidget(btn_add)
        layout.addWidget(btn_send)
        layout.addWidget(btn_start)
        layout.addWidget(btn_clear)

        btn_add.clicked.connect(self.add_pose)
        btn_send.clicked.connect(self.send_poses)
        btn_start.clicked.connect(self.start_planning)
        btn_clear.clicked.connect(self.clear_buffer)

        # ------------------------
        # POSE LIST
        # ------------------------

        self.pose_widget = QListWidget()
        layout.addWidget(self.pose_widget)

        self.setLayout(layout)

        self.setWindowTitle("MotoMini Planning GUI")
        self.resize(420, 520)

    # ------------------------
    # READ INPUT
    # ------------------------

    def read_pose(self):

        x = float(self.inputs["X"].text())
        y = float(self.inputs["Y"].text())
        z = float(self.inputs["Z"].text())

        r = float(self.inputs["Roll"].text())
        p = float(self.inputs["Pitch"].text())
        yaw = float(self.inputs["Yaw"].text())

        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        q = euler.euler2quat(r, p, yaw)

        pose.orientation.w = q[0]
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]

        return pose, (x, y, z, r, p, yaw)

    # ------------------------
    # ADD POSE
    # ------------------------

    def add_pose(self):

        pose, values = self.read_pose()

        self.pose_list.append(pose)

        x, y, z, r, p, yaw = values

        text = (
            f"X:{x:.3f} Y:{y:.3f} Z:{z:.3f} | "
            f"R:{r:.2f} P:{p:.2f} Y:{yaw:.2f}"
        )

        self.pose_widget.addItem(text)

    # ------------------------
    # SEND POSES
    # ------------------------

    def send_poses(self):

        if len(self.pose_list) == 0:
            return

        msg = PoseArray()

        msg.header.frame_id = "world"
        msg.poses = self.pose_list

        self.node.pub_targets.publish(msg)

    # ------------------------
    # START
    # ------------------------

    def start_planning(self):

        msg = Bool()
        msg.data = True

        self.node.pub_start.publish(msg)

    # ------------------------
    # CLEAR
    # ------------------------

    def clear_buffer(self):

        msg = Bool()
        msg.data = True

        self.node.pub_clear.publish(msg)

        self.pose_widget.clear()
        self.pose_list = []

    # ------------------------
    # GUI UPDATE
    # ------------------------

    def update_gui(self):

        self.status_label.setText("Status: " + self.node.status_text)

        pose = self.node.current_pose

        if pose is not None:

            x, y, z, r, p, yaw = pose

            text = (
                f"Robot Pose | "
                f"X:{x:.3f} Y:{y:.3f} Z:{z:.3f} | "
                f"R:{r:.2f} P:{p:.2f} Y:{yaw:.2f}"
            )

            self.robot_pose_label.setText(text)


def main():

    rclpy.init()

    node = PlanningGUI()

    app = QApplication(sys.argv)

    gui = GUI(node)
    gui.show()

    # ROS thread
    import threading

    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin)
    thread.daemon = True
    thread.start()

    # GUI update timer
    timer = QTimer()
    timer.timeout.connect(gui.update_gui)
    timer.start(100)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()