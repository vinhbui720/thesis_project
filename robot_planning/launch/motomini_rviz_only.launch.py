#!/usr/bin/env python3
"""
MotoMini RViz-Only Visualization Launch File

This lightweight launch file:
1. Loads the MotoMini robot URDF description
2. Launches the robot state publisher
3. Launches RViz for visualization
4. Runs the motion planning node
5. Provides joint state control via GUI

This is ideal for:
- Quick visualization and testing without Gazebo
- Systems with limited resources
- Pure motion planning visualization

Usage:
    ros2 launch robot_planning motomini_rviz_only.launch.py
    ros2 launch robot_planning motomini_rviz_only.launch.py run_motion_planning:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    robot_planning_pkg_share = FindPackageShare("robot_planning")
    
    # Declare launch arguments
    declare_run_motion_planning_cmd = DeclareLaunchArgument(
        "run_motion_planning",
        default_value="true",
        description="Run the motion planning node"
    )
    
    run_motion_planning = LaunchConfiguration("run_motion_planning")
    
    # Get paths to URDF
    urdf_file = PathJoinSubstitution([robot_planning_pkg_share, "urdf", "motomini_simple.urdf"])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command([
                    FindExecutable(name="xacro"),
                    " ",
                    urdf_file
                ])
            }
        ]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        robot_planning_pkg_share,
        "config",
        "motomini_visualization.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )
    
    # Motion Planning Node
    motion_planning_node = Node(
        package="robot_planning",
        executable="tesseract_motomini_motion_planning",
        name="tesseract_motomini_motion_planning",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(run_motion_planning)
    )
    
    # Launch description
    ld = LaunchDescription([
        declare_run_motion_planning_cmd,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        motion_planning_node,
    ])
    
    return ld
