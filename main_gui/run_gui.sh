#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash

source /home/vinbui/vinh_ws/install/setup.bash 
# Activate venv
source thesis_gui/bin/activate

# Run app
python main.py
