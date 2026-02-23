from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QScrollArea,
    QPushButton, QLabel, QFileDialog,
    QLineEdit, QFormLayout, QSpinBox, QDoubleSpinBox,
    QCheckBox, QGroupBox
)
from PyQt6.QtCore import Qt
from steps.base_step import BaseStep
from .viewer_widget import MeshViewerWidget
from .mesh_load_logic import MeshLoadLogic


class MeshLoadView(BaseStep):
    def __init__(self, controller):
        super().__init__(controller)

        self.logic = MeshLoadLogic(controller, self)
        self.mesh_loaded = False
        self.cloud_received = False

        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # ---- LEFT : Viewer (3D Display) ----
        self.viewer = MeshViewerWidget()
        main_layout.addWidget(self.viewer, 3)

        # ---- RIGHT : Control Panel ----
        control_widget = QWidget()
        control_layout = QVBoxLayout()
        control_widget.setLayout(control_layout)

        # Make control panel scrollable
        scroll = QScrollArea()
        scroll.setWidget(control_widget)
        scroll.setWidgetResizable(True)
        main_layout.addWidget(scroll, 1)

        # ============ File Selection ============
        file_group = QGroupBox("Model File")
        file_layout = QVBoxLayout()

        self.stl_path_edit = QLineEdit()
        self.stl_path_edit.setPlaceholderText("Path to STL file...")
        browse_btn = QPushButton("Browse STL")
        browse_btn.clicked.connect(self.choose_stl)

        file_layout.addWidget(QLabel("STL File:"))
        file_layout.addWidget(self.stl_path_edit)
        file_layout.addWidget(browse_btn)
        file_group.setLayout(file_layout)
        control_layout.addWidget(file_group)

        # ============ Model Processing Parameters ============
        model_group = QGroupBox("Model Processing")
        model_layout = QFormLayout()

        self.sample_points = QSpinBox()
        self.sample_points.setRange(1000, 1000000)
        self.sample_points.setValue(10000)
        self.sample_points.setSingleStep(1000)
        model_layout.addRow("Sample Points:", self.sample_points)

        self.auto_scale = QCheckBox("Auto Scale")
        self.auto_scale.setChecked(True)
        model_layout.addRow("Auto Scale:", self.auto_scale)

        self.scale_threshold = QDoubleSpinBox()
        self.scale_threshold.setRange(0.1, 100.0)
        self.scale_threshold.setValue(10.0)
        self.scale_threshold.setSingleStep(0.1)
        model_layout.addRow("Scale Threshold:", self.scale_threshold)

        self.center_model = QCheckBox("Center Model")
        self.center_model.setChecked(True)
        model_layout.addRow("Center Model:", self.center_model)

        model_group.setLayout(model_layout)
        control_layout.addWidget(model_group)

        # ============ Voxel Parameters ============
        voxel_group = QGroupBox("Voxel Downsampling")
        voxel_layout = QFormLayout()

        self.voxel_percentage = QDoubleSpinBox()
        self.voxel_percentage.setRange(0.0001, 1.0)
        self.voxel_percentage.setValue(0.004)
        self.voxel_percentage.setSingleStep(0.0001)
        self.voxel_percentage.setDecimals(4)
        voxel_layout.addRow("Voxel %:", self.voxel_percentage)

        self.voxel_size_override = QDoubleSpinBox()
        self.voxel_size_override.setRange(0.0, 10.0)
        self.voxel_size_override.setValue(0.0)
        self.voxel_size_override.setSingleStep(0.01)
        voxel_layout.addRow("Voxel Size (0=auto):", self.voxel_size_override)

        voxel_group.setLayout(voxel_layout)
        control_layout.addWidget(voxel_group)

        # ============ Normal Estimation ============
        normal_group = QGroupBox("Normal Estimation")
        normal_layout = QFormLayout()

        self.normal_radius = QDoubleSpinBox()
        self.normal_radius.setRange(0.1, 100.0)
        self.normal_radius.setValue(3.0)
        self.normal_radius.setSingleStep(0.1)
        normal_layout.addRow("Radius Multiplier:", self.normal_radius)

        normal_group.setLayout(normal_layout)
        control_layout.addWidget(normal_group)

        # ============ Action Buttons ============
        button_layout = QVBoxLayout()

        self.launch_btn = QPushButton("Launch Model Node")
        self.launch_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.launch_btn.clicked.connect(self.launch_model)
        button_layout.addWidget(self.launch_btn)

        status_layout = QHBoxLayout()
        self.status_label = QLabel("Ready")
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("color: gray;")
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.status_indicator)
        button_layout.addLayout(status_layout)

        self.confirm_btn = QPushButton("Confirm & Next")
        self.confirm_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.confirm_btn.setEnabled(False)
        self.confirm_btn.clicked.connect(self.confirm_step)
        button_layout.addWidget(self.confirm_btn)

        control_layout.addLayout(button_layout)
        control_layout.addStretch()

    def choose_stl(self):
        """Open file dialog to select STL file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select STL File", "", "STL Files (*.stl);;All Files (*)"
        )
        if file_path:
            self.stl_path_edit.setText(file_path)
            try:
                self.viewer.load_mesh(file_path)
                self.mesh_loaded = True
                self.update_status("STL loaded", "green")
            except Exception as e:
                self.update_status(f"Error loading STL: {str(e)}", "red")

    def launch_model(self):
        """Launch the ROS2 model manager node"""
        if not self.stl_path_edit.text():
            self.update_status("Please select STL file first", "orange")
            return

        try:
            self.launch_btn.setEnabled(False)
            self.update_status("Launching model node...", "orange")

            self.logic.launch_model(
                self.stl_path_edit.text(),
                self.sample_points.value(),
                self.auto_scale.isChecked(),
                self.scale_threshold.value(),
                self.center_model.isChecked(),
                self.voxel_percentage.value(),
                self.voxel_size_override.value(),
                self.normal_radius.value()
            )
        except Exception as e:
            self.update_status(f"Launch failed: {str(e)}", "red")
            self.launch_btn.setEnabled(True)

    def on_cloud_received(self, points):
        """Callback when point cloud is received"""
        try:
            self.viewer.show_cloud(points)
            self.cloud_received = True
            self.update_status("Point cloud received", "green")
            self.confirm_btn.setEnabled(True)
        except Exception as e:
            self.update_status(f"Error displaying cloud: {str(e)}", "red")

    def confirm_step(self):
        """Confirm and move to next step"""
        self.update_status("Moving to next step...", "blue")
        self.app_controller.go_to_step(1)

    def update_status(self, message, color="gray"):
        """Update status label with color indicator"""
        self.status_label.setText(message)
        color_map = {
            "green": "#4CAF50",
            "red": "#f44336",
            "orange": "#ff9800",
            "blue": "#2196F3",
            "gray": "#999999"
        }
        self.status_indicator.setStyleSheet(f"color: {color_map.get(color, color)};")
        self.launch_btn.setEnabled(True)