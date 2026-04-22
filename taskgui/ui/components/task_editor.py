from PyQt6.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtCore import Qt

class LiveMonitor(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("MonitorPanel")
        layout = QVBoxLayout(self)
        
        layout.addWidget(QLabel("<b>LIVE MONITOR</b>"))
        
        # ROS Status
        self.status_label = QLabel("DISCONNECTED")
        self.status_label.setObjectName("StatusLabel")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status_label)

        # Robot 1 Status
        r1_layout = QHBoxLayout()
        r1_layout.addWidget(QLabel("Robot 1 Pose:"))
        self.r1_pose = QLabel("---")
        self.r1_pose.setObjectName("MonitorValue")
        r1_layout.addWidget(self.r1_pose)
        layout.addLayout(r1_layout)

        # Robot 2 Status
        r2_layout = QHBoxLayout()
        r2_layout.addWidget(QLabel("Robot 2 Pose:"))
        self.r2_pose = QLabel("---")
        self.r2_pose.setObjectName("MonitorValue")
        r2_layout.addWidget(self.r2_pose)
        layout.addLayout(r2_layout)

        layout.addStretch()

    def update_pose(self, robot, pose):
        text = f"X:{pose.position.x:.2f} Y:{pose.position.y:.2f} Z:{pose.position.z:.2f}"
        if robot == 'robot1':
            self.r1_pose.setText(text)
        else:
            self.r2_pose.setText(text)

class TaskEditor(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("TimelinePanel")
        layout = QVBoxLayout(self)
        
        layout.addWidget(QLabel("<b>TASK TIMELINE / SEQUENCE</b>"))

        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Timestamp (s)", "Type", "Source", "Action Data"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.DoubleClicked)
        
        layout.addWidget(self.table)

    def add_row(self, timestamp, entry_type, source, data):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(f"{timestamp:.3f}"))
        self.table.setItem(row, 1, QTableWidgetItem(str(entry_type)))
        self.table.setItem(row, 2, QTableWidgetItem(str(source)))
        self.table.setItem(row, 3, QTableWidgetItem(str(data)))
        self.table.scrollToBottom()

    def clear(self):
        self.table.setRowCount(0)
