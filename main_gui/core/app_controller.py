from core.ros_manager import ROSManager
from core.launch_manager import LaunchManager


class AppController:
    def __init__(self, main_window):
        self.main_window = main_window
        self.current_step = 0
        
        # Initialize managers
        self.ros_manager = ROSManager()
        self.launch_manager = LaunchManager(self.ros_manager)

    def go_to_step(self, index):
        self.current_step = index
        self.main_window.set_step(index)
