import sys
import os
import importlib
import pkgutil

from PyQt6.QtWidgets import QApplication

# Ensure project root is in path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.append(PROJECT_ROOT)

from ui.main_window import MainWindow
from core.app_controller import AppController


def load_styles(app):
    style_path = os.path.join(PROJECT_ROOT, "ui", "styles.qss")
    if os.path.exists(style_path):
        with open(style_path, "r") as f:
            app.setStyleSheet(f.read())


def auto_discover_steps(controller, main_window):
    """
    Automatically discover all step views in steps/*/
    Expected format:
        steps/<step_name>/<step_name>_view.py
        class <Something>View(QWidget)
    """
    import steps

    for finder, name, ispkg in pkgutil.iter_modules(steps.__path__):
        if not ispkg:
            continue

        try:
            module = importlib.import_module(f"steps.{name}.{name}_view")

            # Find class that ends with "View"
            for attr_name in dir(module):
                obj = getattr(module, attr_name)

                if isinstance(obj, type) and attr_name.endswith("View"):
                    step_widget = obj(controller)
                    main_window.add_step_widget(step_widget)

        except ModuleNotFoundError:
            print(f"[INFO] Step '{name}' has no view file yet.")
        except Exception as e:
            print(f"[WARNING] Failed loading step '{name}': {e}")


def main():
    app = QApplication(sys.argv)

    load_styles(app)

    main_window = MainWindow()
    controller = AppController(main_window)
    main_window.set_controller(controller)

    auto_discover_steps(controller, main_window)

    main_window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()