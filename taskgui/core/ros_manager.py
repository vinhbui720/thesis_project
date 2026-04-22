import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from PyQt6.QtCore import QObject, pyqtSignal

class ROSManager(Node, QObject):
    # Signals to UI
    waypoint_received = pyqtSignal(str, object)  # robot_name, pose
    mode_changed = pyqtSignal(bool)
    param_updated = pyqtSignal(str, str, object)  # node, param, value
    status_message = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'task_gui_manager')
        QObject.__init__(self)
        
        self.cb_group = ReentrantCallbackGroup()

        # Subscribers for Robot 1 & 2
        self.create_subscription(PoseStamped, '/robot1/pose', 
                                 lambda msg: self.waypoint_callback('robot1', msg), 10,
                                 callback_group=self.cb_group)
        self.create_subscription(PoseStamped, '/robot2/pose', 
                                 lambda msg: self.waypoint_callback('robot2', msg), 10,
                                 callback_group=self.cb_group)

        # Tool / Mode Triggers
        self.create_subscription(Bool, '/system/operational_mode', 
                                 self.mode_callback, 10,
                                 callback_group=self.cb_group)

        # Parameter Event Listener
        self.create_subscription(ParameterEvent, '/parameter_events', 
                                 self.parameter_callback, 10,
                                 callback_group=self.cb_group)

        self.get_logger().info("ROS Manager Initialized")

    def waypoint_callback(self, robot_name, msg):
        self.waypoint_received.emit(robot_name, msg)

    def mode_callback(self, msg):
        self.mode_changed.emit(msg.data)

    def parameter_callback(self, msg):
        # Filter and emit relevant param changes
        for param in msg.new_parameters:
            self.param_updated.emit(msg.node, param.name, param.value)
        for param in msg.changed_parameters:
            self.param_updated.emit(msg.node, param.name, param.value)

    def call_tool_service(self, tool_id, state):
        # Placeholder for service call logic
        self.status_message.emit(f"Calling Tool {tool_id} -> {state}")
