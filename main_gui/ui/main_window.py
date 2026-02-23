from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QStackedWidget,
)
from ui.header_widget import HeaderWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Thesis GUI")
        self.resize(1400, 850)

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ---- Header ----
        self.header = HeaderWidget()
        main_layout.addWidget(self.header)

        # ---- Main Content Area ----
        self.stack = QStackedWidget()
        self.stack.setObjectName("mainStack")
        main_layout.addWidget(self.stack)

        self.current_step = 0
        self.app_controller = None

    # 🔥 Controller inject
    def set_controller(self, controller):
        self.app_controller = controller

    # 🔥 Add real step widgets here
    def add_step_widget(self, widget):
        self.stack.addWidget(widget)

    # 🔥 Change step (called by controller)
    def set_step(self, index: int):
        if 0 <= index < self.stack.count():
            self.current_step = index
            self.stack.setCurrentIndex(index)
            self.header.set_active_step(index)