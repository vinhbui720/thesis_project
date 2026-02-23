from PyQt6.QtWidgets import QLabel
from PyQt6.QtCore import Qt


class StepIndicator(QLabel):
    def __init__(self, text, index, parent=None):
        super().__init__(text, parent)
        self.index = index
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setObjectName("stepInactive")

    def set_active(self, active: bool):
        if active:
            self.setObjectName("stepActive")
        else:
            self.setObjectName("stepInactive")

        self.style().unpolish(self)
        self.style().polish(self)
