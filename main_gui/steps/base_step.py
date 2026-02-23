from PyQt6.QtWidgets import QWidget

class BaseStep(QWidget):
    def __init__(self, app_controller):
        super().__init__()
        self.app_controller = app_controller

    def on_enter(self):
        """Called when step becomes active"""
        pass

    def on_exit(self):
        """Called when step is left"""
        pass
