#!/usr/bin/env python3

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Bool, String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import transforms3d.euler as euler

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


# =====================================================
# ROS SIGNALS
# =====================================================

class RosSignals(QObject):

    status_updated = pyqtSignal(str)
    pose_updated = pyqtSignal(float,float,float,float,float,float)


# =====================================================
# MASTER ROS NODE
# =====================================================

class MasterDebugNode(Node):

    def __init__(self, signals):

        super().__init__("master_debug_gui")

        self.signals = signals

        # ---------- Planning publishers ----------

        self.pub_targets = self.create_publisher(PoseArray,"/target_poses",10)
        self.pub_start = self.create_publisher(Bool,"/start",10)
        self.pub_clear = self.create_publisher(Bool,"/clear_targets",10)

        # ---------- Object test publishers ----------

        self.pub_object_pose = self.create_publisher(PoseStamped,"/target_object_pose",10)
        self.pub_attach = self.create_publisher(Bool,"/object_attach_signal",10)

        # ---------- Subscribers ----------

        self.create_subscription(
            String,
            "/optimization_status",
            self.status_callback,
            10
        )

        # ---------- TF ----------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.timer = self.create_timer(
            0.1,
            self.lookup_robot_pose
        )

    # ------------------------------------------------

    def status_callback(self,msg):

        self.signals.status_updated.emit(msg.data)

    # ------------------------------------------------

    def lookup_robot_pose(self):

        try:

            trans = self.tf_buffer.lookup_transform(
                "world",
                "tool0",
                Time(),
                timeout=Duration(seconds=0.1)
            )

            t = trans.transform.translation
            r = trans.transform.rotation

            quat = [r.w,r.x,r.y,r.z]

            roll,pitch,yaw = euler.quat2euler(quat)

            self.signals.pose_updated.emit(
                t.x,t.y,t.z,roll,pitch,yaw
            )

        except (LookupException,ConnectivityException,ExtrapolationException):
            pass


# =====================================================
# TAB 1 — PLANNING
# =====================================================

class PlanningTab(QWidget):

    def __init__(self,node,signals):

        super().__init__()

        self.node=node
        self.buffer=[]

        signals.status_updated.connect(self.update_status)
        signals.pose_updated.connect(self.update_pose)

        layout=QVBoxLayout()

        self.status_label=QLabel("Status: Idle")
        self.indicator=QLabel("● Idle")

        self.progress=QProgressBar()

        self.robot_pose=QLabel("Robot Pose: waiting TF")

        layout.addWidget(self.status_label)
        layout.addWidget(self.indicator)
        layout.addWidget(self.robot_pose)
        layout.addWidget(self.progress)

        # pose inputs

        grid=QGridLayout()

        self.spin={}

        fields=[
            ("X","x",-5,5,0.05),
            ("Y","y",-5,5,0),
            ("Z","z",-5,5,0.4),
            ("Roll","r",-180,180,180),
            ("Pitch","p",-180,180,0),
            ("Yaw","yaw",-180,180,0)
        ]

        row=0

        for label,key,mn,mx,val in fields:

            grid.addWidget(QLabel(label),row,0)

            sb=QDoubleSpinBox()
            sb.setRange(mn,mx)
            sb.setValue(val)

            self.spin[key]=sb

            grid.addWidget(sb,row,1)

            row+=1

        layout.addLayout(grid)

        # buttons

        btn_layout=QHBoxLayout()

        add=QPushButton("Add")
        send=QPushButton("Send")
        start=QPushButton("Start")
        clear=QPushButton("Clear")

        add.clicked.connect(self.add_pose)
        send.clicked.connect(self.send)
        start.clicked.connect(self.start)
        clear.clicked.connect(self.clear)

        btn_layout.addWidget(add)
        btn_layout.addWidget(send)
        btn_layout.addWidget(start)
        btn_layout.addWidget(clear)

        layout.addLayout(btn_layout)

        self.list=QListWidget()

        layout.addWidget(self.list)

        self.setLayout(layout)

    # ------------------------------------------------

    def update_status(self,text):

        self.status_label.setText(text)

        t=text.lower()

        if "accumulating" in t:
            self.indicator.setText("● Accumulating")
            self.progress.setValue(20)

        elif "planning" in t:
            self.indicator.setText("● Planning")
            self.progress.setValue(40)

        elif "executing" in t:
            self.indicator.setText("● Executing")
            self.progress.setValue(70)

        elif "success" in t:
            self.indicator.setText("● Success")
            self.progress.setValue(100)

        elif "fail" in t:
            self.indicator.setText("● Failed")
            self.progress.setValue(0)

    # ------------------------------------------------

    def update_pose(self,x,y,z,r,p,yaw):

        self.robot_pose.setText(
            f"XYZ {x:.3f} {y:.3f} {z:.3f}"
        )

    # ------------------------------------------------

    def add_pose(self):

        x=self.spin["x"].value()
        y=self.spin["y"].value()
        z=self.spin["z"].value()

        r=math.radians(self.spin["r"].value())
        p=math.radians(self.spin["p"].value())
        yaw=math.radians(self.spin["yaw"].value())

        q=euler.euler2quat(r,p,yaw)

        pose=Pose()

        pose.position.x=x
        pose.position.y=y
        pose.position.z=z

        pose.orientation.w=q[0]
        pose.orientation.x=q[1]
        pose.orientation.y=q[2]
        pose.orientation.z=q[3]

        self.buffer.append(pose)

        self.list.addItem(f"{x},{y},{z}")

    # ------------------------------------------------

    def send(self):

        msg=PoseArray()
        msg.header.frame_id="world"
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.poses=self.buffer

        self.node.pub_targets.publish(msg)

        self.buffer=[]
        self.list.clear()

    # ------------------------------------------------

    def start(self):

        msg=Bool()
        msg.data=True

        self.node.pub_start.publish(msg)

    # ------------------------------------------------

    def clear(self):

        msg=Bool()
        msg.data=True

        self.node.pub_clear.publish(msg)

        self.buffer=[]
        self.list.clear()


# =====================================================
# TAB 2 — OBJECT TESTER
# =====================================================

class ObjectTesterTab(QWidget):

    def __init__(self,node):

        super().__init__()

        self.node=node

        layout=QVBoxLayout()

        self.sliders={}

        params=[
            ("X",-2,2,0.5),
            ("Y",-2,2,0),
            ("Z",-2,2,0.1),
            ("Roll",-180,180,0),
            ("Pitch",-180,180,0),
            ("Yaw",-180,180,0)
        ]

        for name,mn,mx,val in params:

            lab=QLabel(name)

            slider=QSlider(Qt.Horizontal)
            slider.setRange(int(mn*100),int(mx*100))
            slider.setValue(int(val*100))

            slider.valueChanged.connect(self.publish_pose)

            layout.addWidget(lab)
            layout.addWidget(slider)

            self.sliders[name]=slider

        btn_attach=QPushButton("Attach")
        btn_detach=QPushButton("Detach")

        btn_attach.clicked.connect(lambda:self.attach(True))
        btn_detach.clicked.connect(lambda:self.attach(False))

        layout.addWidget(btn_attach)
        layout.addWidget(btn_detach)

        self.setLayout(layout)

    def publish_pose(self):

        x=self.sliders["X"].value()/100
        y=self.sliders["Y"].value()/100
        z=self.sliders["Z"].value()/100

        r=math.radians(self.sliders["Roll"].value()/100)
        p=math.radians(self.sliders["Pitch"].value()/100)
        yaw=math.radians(self.sliders["Yaw"].value()/100)

        q=euler.euler2quat(r,p,yaw)

        msg=PoseStamped()

        msg.header.frame_id="world"
        msg.header.stamp=self.node.get_clock().now().to_msg()

        msg.pose.position.x=x
        msg.pose.position.y=y
        msg.pose.position.z=z

        msg.pose.orientation.w=q[0]
        msg.pose.orientation.x=q[1]
        msg.pose.orientation.y=q[2]
        msg.pose.orientation.z=q[3]

        self.node.pub_object_pose.publish(msg)

    def attach(self,state):

        msg=Bool()
        msg.data=state

        self.node.pub_attach.publish(msg)


# =====================================================
# TAB 3 — TF VIEWER
# =====================================================

class TfViewerTab(QWidget):

    def __init__(self,node):

        super().__init__()

        self.node=node

        layout=QVBoxLayout()

        self.tree=QTextEdit()
        layout.addWidget(self.tree)

        btn=QPushButton("Refresh TF Tree")
        btn.clicked.connect(self.refresh)
        layout.addWidget(btn)

        self.target=QLineEdit()
        self.source=QLineEdit()

        layout.addWidget(QLabel("Target frame"))
        layout.addWidget(self.target)

        layout.addWidget(QLabel("Source frame"))
        layout.addWidget(self.source)

        btn2=QPushButton("Get Transform")
        btn2.clicked.connect(self.get_tf)

        layout.addWidget(btn2)

        self.result=QTextEdit()

        layout.addWidget(self.result)

        self.setLayout(layout)

    def refresh(self):

        data=self.node.tf_buffer.all_frames_as_yaml()

        self.tree.setText(data)

    def get_tf(self):

        target=self.target.text()
        source=self.source.text()

        try:

            trans=self.node.tf_buffer.lookup_transform(
                target,
                source,
                Time()
            )

            t=trans.transform.translation
            r=trans.transform.rotation

            txt=f"{source} -> {target}\n"
            txt+=f"x {t.x} y {t.y} z {t.z}\n"
            txt+=f"qx {r.x} qy {r.y} qz {r.z} qw {r.w}"

            self.result.setText(txt)

        except Exception as e:

            self.result.setText(str(e))


# =====================================================
# MAIN GUI
# =====================================================

class MasterGUI(QMainWindow):

    def __init__(self,node,signals):

        super().__init__()

        self.setWindowTitle("MotoMini Master Debug GUI")

        tabs=QTabWidget()

        tabs.addTab(PlanningTab(node,signals),"Planning Control")
        tabs.addTab(ObjectTesterTab(node),"Object TF Tester")
        tabs.addTab(TfViewerTab(node),"TF Viewer")

        self.setCentralWidget(tabs)


# =====================================================
# MAIN
# =====================================================

def main():

    rclpy.init()

    app=QApplication(sys.argv)

    signals=RosSignals()

    node=MasterDebugNode(signals)

    gui=MasterGUI(node,signals)
    gui.show()

    def spin():

        rclpy.spin(node)

    thread=threading.Thread(target=spin,daemon=True)
    thread.start()

    sys.exit(app.exec_())


if __name__=="__main__":

    main()