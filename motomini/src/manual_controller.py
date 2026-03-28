#!/usr/bin/env python3

import sys
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

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
    gantry_status_updated = pyqtSignal(str)
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
        self.pub_tracking_control = self.create_publisher(Bool,"/tracking_control",10)

        self.pub_gantry_targets = self.create_publisher(PoseArray,"/gantry/target_poses",10)
        self.pub_gantry_start = self.create_publisher(Bool,"/gantry/start",10)
        self.pub_gantry_clear = self.create_publisher(Bool,"/gantry/clear_targets",10)

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

        self.create_subscription(
            String,
            "/gantry/optimization_status",
            self.gantry_status_callback,
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

    def gantry_status_callback(self,msg):

        self.signals.gantry_status_updated.emit(msg.data)

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
        self.motomini_buffer=[]
        self.gantry_buffer=[]

        signals.status_updated.connect(self.update_status)
        signals.gantry_status_updated.connect(self.update_gantry_status)
        signals.pose_updated.connect(self.update_pose)

        layout=QVBoxLayout()

        row_layout=QHBoxLayout()
        row_layout.addWidget(self._build_motomini_box())
        row_layout.addWidget(self._build_gantry_box())
        layout.addLayout(row_layout)

        self.setLayout(layout)

    # ------------------------------------------------

    def _build_pose_inputs(self, defaults):

        grid=QGridLayout()
        spin={}

        fields=[
            ("X","x",-5,5,defaults[0]),
            ("Y","y",-5,5,defaults[1]),
            ("Z","z",-5,5,defaults[2]),
            ("Roll","r",-180,180,defaults[3]),
            ("Pitch","p",-180,180,defaults[4]),
            ("Yaw","yaw",-180,180,defaults[5])
        ]

        for row,(label,key,mn,mx,val) in enumerate(fields):
            grid.addWidget(QLabel(label),row,0)
            sb=QDoubleSpinBox()
            sb.setDecimals(9)          # 9 decimal places — maximum useful float64 precision
            sb.setSingleStep(0.000000001)
            sb.setRange(mn,mx)
            sb.setValue(val)
            spin[key]=sb
            grid.addWidget(sb,row,1)

        return grid,spin

    # ------------------------------------------------

    def _build_motomini_box(self):

        box=QGroupBox("MotoMini")
        box_layout=QVBoxLayout()

        self.status_label=QLabel("Status: Idle")
        self.indicator=QLabel("● Idle")
        self.progress=QProgressBar()
        self.robot_pose=QLabel("Robot Pose: waiting TF")

        box_layout.addWidget(self.status_label)
        box_layout.addWidget(self.indicator)
        box_layout.addWidget(self.robot_pose)
        box_layout.addWidget(self.progress)

        # ---- Tracking mode toggle ----
        tracking_row = QHBoxLayout()
        self.btn_tracking = QPushButton("▶ Enable Tracking")
        self.btn_tracking.setCheckable(True)
        self.btn_tracking.setStyleSheet(
            "QPushButton:checked { background-color: #2ecc71; color: white; font-weight: bold; }"
            "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; }"
        )
        self.btn_tracking.toggled.connect(self.set_tracking_mode)
        self.tracking_mode_label = QLabel("Mode: Planning")
        tracking_row.addWidget(self.btn_tracking)
        tracking_row.addWidget(self.tracking_mode_label)
        box_layout.addLayout(tracking_row)

        input_grid,self.motomini_spin=self._build_pose_inputs([0.18,0.0,0.24,0.0,0.0,0.0])
        box_layout.addLayout(input_grid)

        btn_layout=QHBoxLayout()
        add=QPushButton("Add")
        send=QPushButton("Send")
        start=QPushButton("Start")
        clear=QPushButton("Clear")

        add.clicked.connect(self.add_motomini_pose)
        send.clicked.connect(self.send_motomini)
        start.clicked.connect(self.start_motomini)
        clear.clicked.connect(self.clear_motomini)

        btn_layout.addWidget(add)
        btn_layout.addWidget(send)
        btn_layout.addWidget(start)
        btn_layout.addWidget(clear)
        box_layout.addLayout(btn_layout)

        self.motomini_list=QListWidget()
        box_layout.addWidget(self.motomini_list)

        box.setLayout(box_layout)
        return box

    # ------------------------------------------------

    def _build_gantry_box(self):

        box=QGroupBox("Gantry")
        box_layout=QVBoxLayout()

        self.gantry_status_label=QLabel("Status: Idle")
        self.gantry_indicator=QLabel("● Idle")
        self.gantry_progress=QProgressBar()

        box_layout.addWidget(self.gantry_status_label)
        box_layout.addWidget(self.gantry_indicator)
        box_layout.addWidget(self.gantry_progress)

        input_grid,self.gantry_spin=self._build_pose_inputs([-0.05,-0.30,0.10,0.0,0.0,0.0])
        box_layout.addLayout(input_grid)

        btn_layout=QHBoxLayout()
        add=QPushButton("Add")
        send=QPushButton("Send")
        start=QPushButton("Start")
        clear=QPushButton("Clear")

        add.clicked.connect(self.add_gantry_pose)
        send.clicked.connect(self.send_gantry)
        start.clicked.connect(self.start_gantry)
        clear.clicked.connect(self.clear_gantry)

        btn_layout.addWidget(add)
        btn_layout.addWidget(send)
        btn_layout.addWidget(start)
        btn_layout.addWidget(clear)
        box_layout.addLayout(btn_layout)

        self.gantry_list=QListWidget()
        box_layout.addWidget(self.gantry_list)

        box.setLayout(box_layout)
        return box

    # ------------------------------------------------

    def update_status(self,text):

        self.status_label.setText(text)

        # Sync the toggle button if the node reports a mode change
        t = text.lower()
        if "mode: tracking" in t:
            self.btn_tracking.blockSignals(True)
            self.btn_tracking.setChecked(True)
            self.btn_tracking.setText("⏹ Disable Tracking")
            self.tracking_mode_label.setText("Mode: Tracking")
            self.btn_tracking.blockSignals(False)
        elif "mode: planning" in t:
            self.btn_tracking.blockSignals(True)
            self.btn_tracking.setChecked(False)
            self.btn_tracking.setText("▶ Enable Tracking")
            self.tracking_mode_label.setText("Mode: Planning")
            self.btn_tracking.blockSignals(False)
        else:
            self._set_status_visuals(text, self.indicator, self.progress)

    # ------------------------------------------------

    def update_gantry_status(self,text):

        self.gantry_status_label.setText(text)

        self._set_status_visuals(text,self.gantry_indicator,self.gantry_progress)

    # ------------------------------------------------

    def _set_status_visuals(self,text,indicator,progress):

        t=text.lower()

        if "accumulating" in t:
            indicator.setText("● Accumulating")
            progress.setValue(20)

        elif "planning" in t:
            indicator.setText("● Planning")
            progress.setValue(40)

        elif "executing" in t:
            indicator.setText("● Executing")
            progress.setValue(70)

        elif "success" in t:
            indicator.setText("● Success")
            progress.setValue(100)

        elif "fail" in t:
            indicator.setText("● Failed")
            progress.setValue(0)

    # ------------------------------------------------

    def update_pose(self,x,y,z,r,p,yaw):

        self.robot_pose.setText(
            f"XYZ {x:.3f} {y:.3f} {z:.3f}"
        )

    # ------------------------------------------------

    def _make_pose(self, spin_widgets):

        x=spin_widgets["x"].value()
        y=spin_widgets["y"].value()
        z=spin_widgets["z"].value()

        r=math.radians(spin_widgets["r"].value())
        p=math.radians(spin_widgets["p"].value())
        yaw=math.radians(spin_widgets["yaw"].value())

        q=euler.euler2quat(r,p,yaw)

        pose=Pose()

        pose.position.x=x
        pose.position.y=y
        pose.position.z=z

        pose.orientation.w=q[0]
        pose.orientation.x=q[1]
        pose.orientation.y=q[2]
        pose.orientation.z=q[3]

        return pose,x,y,z

    # ------------------------------------------------

    def add_motomini_pose(self):

        pose,x,y,z=self._make_pose(self.motomini_spin)

        self.motomini_buffer.append(pose)

        self.motomini_list.addItem(f"{x},{y},{z}")

    # ------------------------------------------------

    def add_gantry_pose(self):

        pose,x,y,z=self._make_pose(self.gantry_spin)

        self.gantry_buffer.append(pose)

        self.gantry_list.addItem(f"{x},{y},{z}")

    # ------------------------------------------------

    def send_motomini(self):

        if not self.motomini_buffer:
            return

        msg=PoseArray()
        msg.header.frame_id="world"
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.poses=self.motomini_buffer

        self.node.pub_targets.publish(msg)

        self.motomini_buffer=[]
        self.motomini_list.clear()

    # ------------------------------------------------

    def send_gantry(self):

        if not self.gantry_buffer:
            return

        msg=PoseArray()
        msg.header.frame_id="world"
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.poses=self.gantry_buffer

        self.node.pub_gantry_targets.publish(msg)

        self.gantry_buffer=[]
        self.gantry_list.clear()

    # ------------------------------------------------

    def start_motomini(self):

        msg=Bool()
        msg.data=True

        self.node.pub_start.publish(msg)

    # ------------------------------------------------

    def start_gantry(self):

        msg=Bool()
        msg.data=True

        self.node.pub_gantry_start.publish(msg)

    # ------------------------------------------------

    def clear_motomini(self):

        msg=Bool()
        msg.data=True

        self.node.pub_clear.publish(msg)

        self.motomini_buffer=[]
        self.motomini_list.clear()

    # ------------------------------------------------

    def clear_gantry(self):

        msg=Bool()
        msg.data=True

        self.node.pub_gantry_clear.publish(msg)

        self.gantry_buffer=[]
        self.gantry_list.clear()

    # ------------------------------------------------

    def set_tracking_mode(self, enabled: bool):

        msg = Bool()
        msg.data = enabled
        self.node.pub_tracking_control.publish(msg)

        if enabled:
            self.btn_tracking.setText("⏹ Disable Tracking")
            self.tracking_mode_label.setText("Mode: Tracking")
        else:
            self.btn_tracking.setText("▶ Enable Tracking")
            self.tracking_mode_label.setText("Mode: Planning")


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

    def closeEvent(self,event):

        QApplication.instance().quit()
        event.accept()


# =====================================================
# MAIN
# =====================================================

def main():

    rclpy.init()

    app=QApplication(sys.argv)

    signals=RosSignals()

    node=MasterDebugNode(signals)
    executor=SingleThreadedExecutor()
    executor.add_node(node)

    gui=MasterGUI(node,signals)
    gui.show()

    shutting_down=False

    def shutdown_once():
        nonlocal shutting_down

        if shutting_down:
            return

        shutting_down=True

        spin_timer.stop()

        try:
            executor.remove_node(node)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            executor.shutdown()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def handle_ros_spin():
        if shutting_down:
            return

        try:
            executor.spin_once(timeout_sec=0.0)
        except (ExternalShutdownException, KeyboardInterrupt):
            shutdown_once()
            QApplication.instance().quit()

    spin_timer=QTimer()
    spin_timer.timeout.connect(handle_ros_spin)
    spin_timer.start(10)

    app.aboutToQuit.connect(shutdown_once)

    def _signal_handler(_sig, _frame):
        shutdown_once()
        QMetaObject.invokeMethod(app, "quit", Qt.QueuedConnection)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    sys.exit(app.exec_())


if __name__=="__main__":

    main()