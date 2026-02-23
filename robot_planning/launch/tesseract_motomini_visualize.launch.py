#!/usr/bin/env python3
"""
Tesseract MotoMini Visualization Launch File

This launch file:
1. Loads the MotoMini robot description
2. Launches the Tesseract visualizer node
3. Optionally launches RViz for joint state visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    robot_planning_share = FindPackageShare("robot_planning")
    
    # Tesseract MotoMini Visualizer Node
    tesseract_visualizer_node = Node(
        package="robot_planning",
        executable="tesseract_motomini_visualizer_node",
        name="tesseract_motomini_visualizer",
        output="screen",
        emulate_tty=True,
    )

    # Launch Description
    ld = LaunchDescription([
        tesseract_visualizer_node,
    ])

    return ld
