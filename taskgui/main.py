import sys
import os
import rclpy
from PyQt6.QtWidgets import QApplication
from ui.main_window import MainWindow

def main():
    # Initialize ROS 2
    rclpy.init(args=sys.argv)

    # Initialize PyQt6
    app = QApplication(sys.argv)

    # Load Styles
    style_path = os.path.join(os.path.dirname(__file__), "ui", "styles.qss")
    if os.path.exists(style_path):
        with open(style_path, "r") as f:
            app.setStyleSheet(f.read())

    # Create and Show Main Window
    window = MainWindow()
    window.show()

    # Run Event Loop
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
