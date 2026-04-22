from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame
from PyQt6.QtCore import Qt

class HeaderBar(QFrame):
    def __init__(self, project_name="Dual-Robot Task Manager", mssv="202X-XXXX"):
        super().__init__()
        self.setObjectName("HeaderBar")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)

        # Logo placeholder
        self.logo = QLabel("🤖")
        self.logo.setStyleSheet("font-size: 32px;")
        layout.addWidget(self.logo)

        # Titles
        title_layout = QVBoxLayout()
        self.title_label = QLabel(project_name)
        self.title_label.setObjectName("ProjectTitle")
        self.mssv_label = QLabel(f"Student ID: {mssv}")
        self.mssv_label.setObjectName("MSSVLabel")
        title_layout.addWidget(self.title_label)
        title_layout.addWidget(self.mssv_label)
        layout.addLayout(title_layout)

        layout.addStretch()

class ControlPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("ControlPanel")
        layout = QHBoxLayout(self)
        layout.setSpacing(15)

        self.btn_record = QPushButton("Record")
        self.btn_record.setObjectName("RecordButton")
        
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setObjectName("StopButton")
        self.btn_stop.setEnabled(False)

        self.btn_play = QPushButton("Playback")
        self.btn_play.setObjectName("PlaybackButton")

        self.btn_save = QPushButton("Save Task")

        layout.addWidget(self.btn_record)
        layout.addWidget(self.btn_stop)
        layout.addWidget(self.btn_play)
        layout.addStretch()
        layout.addWidget(self.btn_save)
