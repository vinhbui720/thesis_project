from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFileDialog
from ui.components.header import HeaderBar, ControlPanel
from ui.components.task_editor import LiveMonitor, TaskEditor
from core.recorder import TaskRecorder, PlaybackEngine
from core.ros_manager import ROSManager
import rclpy
import threading

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dual-Robot Master GUI")
        self.resize(1100, 750)

        # Initialize ROS 2
        self.ros_manager = ROSManager()
        self.recorder = TaskRecorder()
        self.player = PlaybackEngine(self.ros_manager)

        # Setup Threading for ROS
        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self.ros_manager), daemon=True)
        self.ros_thread.start()

        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header
        self.header = HeaderBar()
        layout.addWidget(self.header)

        # Dashboard layout
        dashboard = QHBoxLayout()
        dashboard.setContentsMargins(10, 10, 10, 10)
        
        # Left Panel (Control + Monitor)
        left_side = QVBoxLayout()
        self.control_panel = ControlPanel()
        self.monitor_panel = LiveMonitor()
        left_side.addWidget(self.control_panel)
        left_side.addWidget(self.monitor_panel)
        dashboard.addLayout(left_side, 1)

        # Right Panel (Timeline)
        self.timeline_panel = TaskEditor()
        dashboard.addWidget(self.timeline_panel, 2)

        layout.addLayout(dashboard)

    def connect_signals(self):
        # ROS -> UI
        self.ros_manager.status_message.connect(lambda msg: self.monitor_panel.status_label.setText(msg))
        self.ros_manager.waypoint_received.connect(self.on_waypoint)
        
        # UI -> Logic
        self.control_panel.btn_record.clicked.connect(self.start_recording)
        self.control_panel.btn_stop.clicked.connect(self.stop_action)
        self.control_panel.btn_play.clicked.connect(self.start_playback)
        self.control_panel.btn_save.clicked.connect(self.save_task)

    def on_waypoint(self, robot, msg):
        self.monitor_panel.update_pose(robot, msg)
        if self.recorder.is_recording:
            # Simple conversion for YAML storage
            data = {
                'x': round(msg.position.x, 4),
                'y': round(msg.position.y, 4),
                'z': round(msg.position.z, 4)
            }
            self.recorder.add_entry('waypoint', robot, data)
            entry = self.recorder.sequence[-1]
            self.timeline_panel.add_row(entry['timestamp'], 'WayPoint', robot, f"Pos: {data}")

    def start_recording(self):
        self.recorder.start()
        self.timeline_panel.clear()
        self.control_panel.btn_record.setEnabled(False)
        self.control_panel.btn_stop.setEnabled(True)
        self.monitor_panel.status_label.setText("RECORDING LIVE...")

    def stop_action(self):
        self.recorder.stop()
        self.player.stop()
        self.control_panel.btn_record.setEnabled(True)
        self.control_panel.btn_stop.setEnabled(False)
        self.monitor_panel.status_label.setText("IDLE / STOPPED")

    def start_playback(self):
        self.monitor_panel.status_label.setText("PLAYING SEQUENCE...")
        # Playback logic should run in separate thread to not block UI
        playback_thread = threading.Thread(target=self.player.execute)
        playback_thread.start()

    def save_task(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Task Sequence", "", "YAML Files (*.yaml)")
        if path:
            self.recorder.save_to_yaml(path)
