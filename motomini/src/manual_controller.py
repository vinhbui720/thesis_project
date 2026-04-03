#!/usr/bin/env python3

import sys
import math
import signal
import yaml
from pathlib import Path

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

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QValidator


# =====================================================
# CONFIG LOADER
# =====================================================

def load_planning_params():
    """Load planning parameters from planning_params.yaml.
    Returns a dict of parameters, or an empty dict if file not found."""
    config_paths = [
        Path.home() / "vinh_ws" / "install" / "robot_planning" / "share" / "robot_planning" / "config" / "planning_params.yaml",
        Path.home() / "vinh_ws" / "src" / "thesis_project" / "robot_planning" / "config" / "planning_params.yaml",
        Path("/home/vinbui/vinh_ws/src/thesis_project/robot_planning/config/planning_params.yaml"),
    ]
    
    for config_path in config_paths:
        if config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'motomini_planning_node' in data:
                        return data['motomini_planning_node'].get('ros__parameters', {})
            except Exception as e:
                print(f"Error loading {config_path}: {e}")
    
    print("Warning: planning_params.yaml not found, using hardcoded defaults")
    return {}


# =====================================================
# SCIENTIFIC SPINBOX
# =====================================================

class ScientificDoubleSpinBox(QAbstractSpinBox):
    """
    A spinbox that correctly handles scientific-notation values (e.g. 1e-8).

    QDoubleSpinBox is intentionally avoided: it stores values internally as
    int(value × 10^decimals), which silently rounds 1e-8 to 0 when decimals=9
    (int(1e-8 × 1e9) = int(0.01) = 0).  This class owns the float directly.
    """

    valueChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value    = 0.0
        self._min      = -1e308
        self._max      =  1e308
        self._step     = 0.1
        self._decimals = 6
        self.lineEdit().setText(self._fmt(0.0))
        self.lineEdit().editingFinished.connect(self._commit)

    # ---- public API mirroring QDoubleSpinBox ----

    def value(self):
        return self._value

    def setValue(self, v):
        v = float(v)
        v = max(self._min, min(self._max, v))
        if v != self._value:
            self._value = v
            self.valueChanged.emit(v)
        self.lineEdit().setText(self._fmt(v))

    def setRange(self, mn, mx):
        self._min = float(mn)
        self._max = float(mx)

    def minimum(self):
        return self._min

    def maximum(self):
        return self._max

    def setSingleStep(self, step):
        self._step = float(step)

    def setDecimals(self, d):
        self._decimals = int(d)

    def decimals(self):
        return self._decimals

    # ---- QAbstractSpinBox interface ----

    def stepBy(self, steps):
        self.setValue(self._value + steps * self._step)

    def stepEnabled(self):
        flags = QAbstractSpinBox.StepNone
        if self._value > self._min:
            flags |= QAbstractSpinBox.StepDownEnabled
        if self._value < self._max:
            flags |= QAbstractSpinBox.StepUpEnabled
        return flags

    def validate(self, text, pos):
        clean = text.strip()
        try:
            v = float(clean)
            if self._min <= v <= self._max:
                return (QValidator.Acceptable, text, pos)
            return (QValidator.Intermediate, text, pos)
        except ValueError:
            lo = clean.lower()
            if clean in ('', '-', '+', '.', '-.') or lo.endswith(('e', 'e-', 'e+')):
                return (QValidator.Intermediate, text, pos)
            return (QValidator.Invalid, text, pos)

    def fixup(self, text):
        try:
            v = max(self._min, min(self._max, float(text.strip())))
        except ValueError:
            v = self._value
        return self._fmt(v)

    # ---- helpers ----

    def _fmt(self, v):
        """Format: scientific for |v| < 0.001 (and v≠0), fixed otherwise."""
        if v != 0.0 and abs(v) < 0.001:
            return f"{v:.{self._decimals}e}"
        return f"{v:.{self._decimals}f}"

    def _commit(self):
        """Parse the line-edit text and update internal value."""
        try:
            v = float(self.lineEdit().text().strip())
            self.setValue(v)
        except ValueError:
            self.lineEdit().setText(self._fmt(self._value))


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

        # ---------- Parameter service client (for Planner Tuning tab) ----------

        self.param_client = self.create_client(
            SetParameters, '/motomini_planning_node/set_parameters')

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

    # ------------------------------------------------

    def set_planner_params(self, params_dict):
        """Send a dict of {name: value} to the planning node parameter service.
        Returns the async future, or None if the service is unavailable."""

        if not self.param_client.service_is_ready():
            return None

        req = SetParameters.Request()

        for name, value in params_dict.items():
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = value
            elif isinstance(value, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = value
            elif isinstance(value, float):
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = value
            elif isinstance(value, str):
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = value
            else:
                continue

            p = Parameter()
            p.name = name
            p.value = pv
            req.parameters.append(p)

        return self.param_client.call_async(req)


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
            sb=ScientificDoubleSpinBox()
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
        home=QPushButton("🏠 Home")
        home.setStyleSheet("font-weight: bold; background-color: #3498db; color: white;")
        home.setToolTip("Move to home position (x=0.18, y=0.0, z=0.245) and start")

        add.clicked.connect(self.add_motomini_pose)
        send.clicked.connect(self.send_motomini)
        start.clicked.connect(self.start_motomini)
        clear.clicked.connect(self.clear_motomini)
        home.clicked.connect(self.home_motomini)

        btn_layout.addWidget(add)
        btn_layout.addWidget(send)
        btn_layout.addWidget(start)
        btn_layout.addWidget(clear)
        btn_layout.addWidget(home)
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

    def home_motomini(self):
        """Send robot to home position (x=0.18, y=0.0, z=0.245) with no orientation constraint."""
        
        # Create home pose
        pose = Pose()
        pose.position.x = 0.18
        pose.position.y = 0.0
        pose.position.z = 0.245
        # No orientation constraint (identity quaternion)
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        # Add to buffer and send
        self.motomini_buffer = [pose]
        self.motomini_list.clear()
        self.motomini_list.addItem("0.18, 0.0, 0.245 (Home)")
        
        # Send pose and start immediately
        self.send_motomini()
        self.start_motomini()

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
# TAB 4 — PLANNER TUNING
# =====================================================

class PlannerTuningTab(QWidget):
    """GUI tab for online tuning of all planning hyperparameters.
    Changes are sent to the planning node via the SetParameters service
    and take effect on the next Start (no rebuild needed)."""

    def __init__(self, node):

        super().__init__()
        self.node = node
        self._pending_future = None
        
        # Load configuration from planning_params.yaml
        self.params = load_planning_params()

        # Wrap everything in a scroll area (many params)
        outer = QVBoxLayout()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(6)

        # --- Motion Instruction Type ---
        g = QGroupBox("Motion Instruction Type")
        r = QHBoxLayout()
        r.addWidget(QLabel("Type:"))
        self.motion_type = QComboBox()
        self.motion_type.addItems(["FREESPACE", "LINEAR"])
        default_motion_type = self.params.get("move_instruction_type", "FREESPACE")
        self.motion_type.setCurrentText(default_motion_type)
        self.motion_type.setToolTip(
            "FREESPACE: free joint path to reach each waypoint (orientation unconstrained)\n"
            "LINEAR: straight TCP path between waypoints (orientation constrained)")
        r.addWidget(self.motion_type)
        r.addStretch()
        g.setLayout(r)
        layout.addWidget(g)

        # --- Cartesian Constraint Coefficients ---
        g = QGroupBox("Cartesian Constraint Coefficients  [x, y, z,  rx, ry, rz]")
        gl = QGridLayout()
        self.cart_x  = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_x", 100.0), 10.0, "Weight for X translation.")
        self.cart_y  = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_y", 100.0), 10.0, "Weight for Y translation.")
        self.cart_z  = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_z", 100.0), 10.0, "Weight for Z translation.")
        self.cart_rx = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_rx", 0.0), 10.0, "Weight for Roll  (0 = free rotation).")
        self.cart_ry = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_ry", 0.0), 10.0, "Weight for Pitch (0 = free rotation).")
        self.cart_rz = self._dspin(0, 10000, self.params.get("ifopt_cart_coeff_rz", 0.0), 10.0, "Weight for Yaw   (0 = free rotation).")
        gl.addWidget(QLabel("X weight:"),  0, 0); gl.addWidget(self.cart_x,  0, 1)
        gl.addWidget(QLabel("Y weight:"),  1, 0); gl.addWidget(self.cart_y,  1, 1)
        gl.addWidget(QLabel("Z weight:"),  2, 0); gl.addWidget(self.cart_z,  2, 1)
        gl.addWidget(QLabel("Rx (roll):"), 3, 0); gl.addWidget(self.cart_rx, 3, 1)
        gl.addWidget(QLabel("Ry (pitch):"),4, 0); gl.addWidget(self.cart_ry, 4, 1)
        gl.addWidget(QLabel("Rz (yaw):"),  5, 0); gl.addWidget(self.cart_rz, 5, 1)
        gl.addWidget(QLabel("(0 = free rotation, 100+ = constrained)"), 6, 0, 1, 2)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- Joint Cost ---
        g = QGroupBox("Joint Cost  (regularizer — penalises large joint moves)")
        r = QHBoxLayout()
        self.joint_coeff = self._dspin(0, 1000, self.params.get("ifopt_joint_cost_coeff", 5.0), 1.0,
            "Higher = smoother joint trajectory, less aggressive movement.")
        r.addWidget(QLabel("Joint cost coeff:"))
        r.addWidget(self.joint_coeff)
        r.addStretch()
        g.setLayout(r)
        layout.addWidget(g)

        # --- Collision Avoidance ---
        g = QGroupBox("Collision Avoidance  (soft cost)")
        gl = QGridLayout()
        self.coll_margin = self._dspin(0, 0.5, self.params.get("ifopt_coll_cost_margin", 0.02), 0.005, decimals=4,
            tip="Minimum clearance from obstacles (metres).")
        self.coll_coeff  = self._dspin(0, 10000, self.params.get("ifopt_coll_cost_coeff", 500.0), 50.0,
            tip="Penalty weight for violating the margin. Higher = stronger push-away.")
        self.coll_buffer = self._dspin(0, 0.5, self.params.get("ifopt_coll_margin_buffer", 0.02), 0.005, decimals=4,
            tip="Extra buffer on top of margin for LVS swept-volume check.")
        self.coll_eval_type = QComboBox()
        self.coll_eval_type.addItems(["0 – DISCRETE", "1 – CONTINUOUS", "2 – LVS_CONTINUOUS"])
        self.coll_eval_type.setCurrentIndex(self.params.get("ifopt_coll_eval_type", 0))
        self.coll_eval_type.setToolTip(
            "Collision evaluator type used during TrajOpt planning.\n"
            "DISCRETE: check at each point only.\n"
            "CONTINUOUS: swept-volume check between consecutive points.\n"
            "LVS_CONTINUOUS: continuous with longest-valid-segment length.")
        self.coll_lvs = self._dspin(0.0001, 1.0, self.params.get("ifopt_coll_lvs_length", 0.005), 0.001, decimals=5,
            tip="longest_valid_segment_length (m) — used when eval type is LVS_CONTINUOUS.")
        gl.addWidget(QLabel("Margin (m):"),        0, 0); gl.addWidget(self.coll_margin,    0, 1)
        gl.addWidget(QLabel("Coeff:"),             1, 0); gl.addWidget(self.coll_coeff,     1, 1)
        gl.addWidget(QLabel("Buffer (m):"),        2, 0); gl.addWidget(self.coll_buffer,    2, 1)
        gl.addWidget(QLabel("Evaluator type:"),    3, 0); gl.addWidget(self.coll_eval_type, 3, 1)
        gl.addWidget(QLabel("LVS length (m):"),    4, 0); gl.addWidget(self.coll_lvs,       4, 1)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- Trajectory Smoothing ---
        g = QGroupBox("Trajectory Smoothing  (velocity / acceleration / jerk)")
        gl = QGridLayout()
        self.smooth_vel  = self._dspin(0, 100, self.params.get("ifopt_smooth_vel_coeff", 0.1), 0.05, decimals=4,
            tip="Velocity smoothing weight across waypoints.")
        self.smooth_acc  = self._dspin(0, 100, self.params.get("ifopt_smooth_acc_coeff", 1.0), 0.1,
            tip="Acceleration smoothing weight.")
        self.smooth_jerk = self._dspin(0, 100, self.params.get("ifopt_smooth_jerk_coeff", 1.0), 0.1,
            tip="Jerk smoothing weight.")
        gl.addWidget(QLabel("Velocity:"),     0, 0); gl.addWidget(self.smooth_vel,  0, 1)
        gl.addWidget(QLabel("Acceleration:"), 1, 0); gl.addWidget(self.smooth_acc,  1, 1)
        gl.addWidget(QLabel("Jerk:"),         2, 0); gl.addWidget(self.smooth_jerk, 2, 1)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- SQP Solver ---
        g = QGroupBox("SQP Solver")
        gl = QGridLayout()
        self.max_iter   = QSpinBox()
        self.max_iter.setRange(1, 10000)
        self.max_iter.setValue(self.params.get("ifopt_max_iter", 200))
        self.max_iter.setToolTip("Maximum SQP iterations per chunk.")
        self.min_approx = self._dspin(0, 1, self.params.get("ifopt_min_approx_improve", 1.0e-3), 1e-7, decimals=9,
            tip="Stop if cost improvement per iteration < this.")
        self.min_trust  = self._dspin(0, 1, self.params.get("ifopt_min_trust_box_size", 1.0e-3), 1e-6, decimals=9,
            tip="Stop if trust region shrinks below this.")
        self.init_trust = self._dspin(0, 10, self.params.get("ifopt_initial_trust_box_size", 0.5), 0.05,
            tip="Initial SQP step size.")
        gl.addWidget(QLabel("Max iterations:"),      0, 0); gl.addWidget(self.max_iter,   0, 1)
        gl.addWidget(QLabel("Min approx improve:"),  1, 0); gl.addWidget(self.min_approx, 1, 1)
        gl.addWidget(QLabel("Min trust box size:"),  2, 0); gl.addWidget(self.min_trust,  2, 1)
        gl.addWidget(QLabel("Initial trust box:"),   3, 0); gl.addWidget(self.init_trust, 3, 1)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- Chunk Planning ---
        g = QGroupBox("Chunk Planning")
        gl = QGridLayout()
        self.chunk_size     = QSpinBox()
        self.chunk_size.setRange(1, 1000)
        self.chunk_size.setValue(self.params.get("planning_chunk_size", 5))
        self.chunk_size.setToolTip("Number of waypoints per TrajOpt solve. Smaller = faster per chunk, more chunks.")
        self.parallel_chunks = QSpinBox()
        self.parallel_chunks.setRange(1, 32)
        self.parallel_chunks.setValue(self.params.get("planning_parallel_chunks", 2))
        self.parallel_chunks.setToolTip("Maximum concurrent TrajOpt solves (should match CPU core count).")
        gl.addWidget(QLabel("Chunk size (waypoints):"), 0, 0); gl.addWidget(self.chunk_size,      0, 1)
        gl.addWidget(QLabel("Parallel chunks:"),        1, 0); gl.addWidget(self.parallel_chunks, 1, 1)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- OMPL ---
        g = QGroupBox("OMPL  (toggle below or set use_ompl at launch)")
        gl = QGridLayout()
        self.ompl_enable = QCheckBox("Enable OMPL (use_ompl)")
        self.ompl_enable.setChecked(self.params.get("use_ompl", False))
        self.ompl_enable.setToolTip(
            "Toggle OMPL planner on/off at runtime.\n"
            "When ON: FreespacePipeline (OMPL + TrajOpt refine).\n"
            "When OFF: TrajOptIfopt or TrajOpt direct.")
        self.ompl_time    = self._dspin(0.1, 120, self.params.get("ompl_planning_time", 10.0), 1.0,
            tip="Seconds allowed per OMPL planning call.")
        self.ompl_max_sol = QSpinBox()
        self.ompl_max_sol.setRange(1, 100)
        self.ompl_max_sol.setValue(self.params.get("ompl_max_solutions", 5))
        self.ompl_max_sol.setToolTip("OMPL stops after finding this many solutions.")
        self.ompl_simplify = QCheckBox("Simplify path")
        self.ompl_simplify.setChecked(self.params.get("ompl_simplify", True))
        self.ompl_simplify.setToolTip("Post-process OMPL path to smooth / shorten.")
        self.ompl_rrt1 = self._dspin(0.001, 5.0, self.params.get("ompl_rrt_range_1", 0.05), 0.01, decimals=4,
            tip="RRTConnect tree expansion range (tight).")
        self.ompl_rrt2 = self._dspin(0.001, 5.0, self.params.get("ompl_rrt_range_2", 0.10), 0.01, decimals=4,
            tip="RRTConnect tree expansion range (loose).")
        gl.addWidget(self.ompl_enable,             0, 0, 1, 2)
        gl.addWidget(QLabel("Planning time (s):"), 1, 0); gl.addWidget(self.ompl_time,    1, 1)
        gl.addWidget(QLabel("Max solutions:"),     2, 0); gl.addWidget(self.ompl_max_sol, 2, 1)
        gl.addWidget(self.ompl_simplify,           3, 0, 1, 2)
        gl.addWidget(QLabel("RRT range 1:"),       4, 0); gl.addWidget(self.ompl_rrt1,    4, 1)
        gl.addWidget(QLabel("RRT range 2:"),       5, 0); gl.addWidget(self.ompl_rrt2,    5, 1)
        g.setLayout(gl)
        layout.addWidget(g)

        # --- Apply button + status ---
        btn = QPushButton("\u2705  Apply to Planner Node")
        btn.setStyleSheet("font-weight: bold; padding: 8px; font-size: 13px;")
        btn.setToolTip("Sends all parameters to the running planning node.\n"
                       "Changes take effect on the NEXT Start press.")
        btn.clicked.connect(self.apply_clicked)

        self.status_label = QLabel("Edit parameters above, then click Apply.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #555; font-style: italic;")

        layout.addWidget(btn)
        layout.addWidget(self.status_label)
        layout.addStretch()

        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.setLayout(outer)

    # ------------------------------------------------

    def _dspin(self, mn, mx, default, step, tip="", decimals=3):
        sb = ScientificDoubleSpinBox()
        sb.setRange(mn, mx)
        sb.setValue(default)
        sb.setSingleStep(step)
        sb.setDecimals(decimals)
        if tip:
            sb.setToolTip(tip)
        return sb

    # ------------------------------------------------

    def apply_clicked(self):

        params = {
            # Motion type
            "move_instruction_type":      self.motion_type.currentText(),
            # Runtime OMPL toggle
            "use_ompl":                   self.ompl_enable.isChecked(),
            # Per-axis cartesian constraints
            "ifopt_cart_coeff_x":         self.cart_x.value(),
            "ifopt_cart_coeff_y":         self.cart_y.value(),
            "ifopt_cart_coeff_z":         self.cart_z.value(),
            "ifopt_cart_coeff_rx":        self.cart_rx.value(),
            "ifopt_cart_coeff_ry":        self.cart_ry.value(),
            "ifopt_cart_coeff_rz":        self.cart_rz.value(),
            # Joint cost
            "ifopt_joint_cost_coeff":     self.joint_coeff.value(),
            # Collision
            "ifopt_coll_cost_margin":     self.coll_margin.value(),
            "ifopt_coll_cost_coeff":      self.coll_coeff.value(),
            "ifopt_coll_margin_buffer":   self.coll_buffer.value(),
            "ifopt_coll_eval_type":       self.coll_eval_type.currentIndex(),
            "ifopt_coll_lvs_length":      self.coll_lvs.value(),
            # Smoothing
            "ifopt_smooth_vel_coeff":     self.smooth_vel.value(),
            "ifopt_smooth_acc_coeff":     self.smooth_acc.value(),
            "ifopt_smooth_jerk_coeff":    self.smooth_jerk.value(),
            # SQP solver
            "ifopt_max_iter":             self.max_iter.value(),
            "ifopt_min_approx_improve":   self.min_approx.value(),
            "ifopt_min_trust_box_size":   self.min_trust.value(),
            "ifopt_initial_trust_box_size": self.init_trust.value(),
            # Chunk planning
            "planning_chunk_size":        self.chunk_size.value(),
            "planning_parallel_chunks":   self.parallel_chunks.value(),
            # OMPL tuning
            "ompl_planning_time":         self.ompl_time.value(),
            "ompl_max_solutions":         self.ompl_max_sol.value(),
            "ompl_simplify":              self.ompl_simplify.isChecked(),
            "ompl_rrt_range_1":           self.ompl_rrt1.value(),
            "ompl_rrt_range_2":           self.ompl_rrt2.value(),
        }

        self._pending_future = self.node.set_planner_params(params)

        if self._pending_future is None:
            self.status_label.setText(
                "\u274c Service /motomini_planning_node/set_parameters not available. "
                "Is the planning node running?")
            self.status_label.setStyleSheet("color: red;")
        else:
            self.status_label.setText("\u23f3 Sending parameters to planning node...")
            self.status_label.setStyleSheet("color: #e67e00;")
            QTimer.singleShot(2000, self._check_result)

    # ------------------------------------------------

    def _check_result(self):

        if self._pending_future is None:
            return

        if self._pending_future.done():
            result = self._pending_future.result()
            failed = [r for r in result.results if not r.successful]
            if not failed:
                self.status_label.setText(
                    "\u2705 All parameters applied. Takes effect on next \u25b6 Start.")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
            else:
                reasons = "; ".join(r.reason for r in failed)
                self.status_label.setText(f"\u26a0\ufe0f {len(failed)} param(s) failed: {reasons}")
                self.status_label.setStyleSheet("color: orange;")
        else:
            self.status_label.setText(
                "\u26a0\ufe0f No response from planning node (timeout).")
            self.status_label.setStyleSheet("color: orange;")

        self._pending_future = None


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
        tabs.addTab(PlannerTuningTab(node),   "Planner Tuning")
        tabs.addTab(ObjectTesterTab(node),    "Object TF Tester")
        tabs.addTab(TfViewerTab(node),        "TF Viewer")

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