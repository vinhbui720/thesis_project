from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt6.QtCore import Qt
from ui.step_indicator import StepIndicator


class HeaderWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.setObjectName("header")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(30, 15, 30, 15)
        layout.setSpacing(30)

        self.steps = []

        self.step_names = [
            "1  Mesh Load",
            "2  Picking Point",
            "3  Trajectory",
            "4  Calibration",
            "5  Processing",
        ]

        for i, name in enumerate(self.step_names):
            step = StepIndicator(name, i)
            self.steps.append(step)
            layout.addWidget(step)

        layout.addStretch()

        self.logo = QLabel("🤖")
        self.logo.setObjectName("logoLabel")

        self.project_label = QLabel("Thesis Project  |  Bùi Quang Vinh")
        self.project_label.setObjectName("projectLabel")

        layout.addWidget(self.logo)
        layout.addSpacing(20)
        layout.addWidget(self.project_label)

        self.set_active_step(0)

    def set_active_step(self, index: int):
        for i, step in enumerate(self.steps):
            step.set_active(i == index)