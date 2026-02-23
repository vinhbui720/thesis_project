#!/usr/bin/env python3
"""
MotoMini with RViz and Gazebo Visualization Launch File

This launch file:
1. Loads the MotoMini robot URDF description
2. Launches the robot state publisher (for TF broadcasting)
3. Launches RViz for visualization and motion planning display
4. Launches Gazebo with the robot model
5. Runs the motion planning node
6. Spawns the robot model into Gazebo

Usage:
    ros2 launch robot_planning motomini_with_visualization.launch.py
    ros2 launch robot_planning motomini_with_visualization.launch.py use_gazebo:=false  # RViz only
    ros2 launch robot_planning motomini_with_visualization.launch.py use_rviz:=false    # Gazebo only
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    robot_planning_pkg_share = FindPackageShare("robot_planning")
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization"
    )
    
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        "use_gazebo",
        default_value="true",
        description="Launch Gazebo for physics simulation"
    )
    
    declare_run_motion_planning_cmd = DeclareLaunchArgument(
        "run_motion_planning",
        default_value="true",
        description="Run the motion planning node"
    )
    
    # Get configuration
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    run_motion_planning = LaunchConfiguration("run_motion_planning")
    
    # Get paths to URDF and SRDF files
    urdf_file = PathJoinSubstitution([robot_planning_pkg_share, "urdf", "motomini_simple.urdf"])
    
    # Robot state publisher - converts joint states to TF transforms
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
                ]),
                "use_sim_time": use_sim_time
            }
        ]
    )
    
    # Joint State Publisher GUI - allows manual control of joints for testing
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    # RViz Node
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
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Gazebo Server (headless physics engine)
    gazebo_server = ExecuteProcess(
        cmd=[
            FindExecutable(name="gzserver"),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so"
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
        additional_env={
            "GAZEBO_PLUGIN_PATH": os.path.join(
                get_package_share_directory("gazebo_ros"),
                "lib"
            )
        }
    )
    
    # Gazebo Client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=[FindExecutable(name="gzclient")],
        output="screen",
        condition=IfCondition(use_gazebo)
    )
    
    # Gazebo Spawn Entity (spawn the robot model into Gazebo)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "motomini",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0"
        ],
        output="screen",
        condition=IfCondition(use_gazebo)
    )
    
    # Tesseract MotoMini Visualizer Node
    tesseract_visualizer_node = Node(
        package="robot_planning",
        executable="tesseract_motomini_visualizer_node",
        name="tesseract_motomini_visualizer",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    # Motion Planning Node
    motion_planning_node = Node(
        package="robot_planning",
        executable="tesseract_motomini_motion_planning",
        name="tesseract_motomini_motion_planning",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(run_motion_planning)
    )
    
    # Create launch description
    ld = LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_use_gazebo_cmd,
        declare_run_motion_planning_cmd,
        
        # Core nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        
        # Visualization nodes
        rviz_node,
        
        # Gazebo nodes
        gazebo_server,
        gazebo_client,
        spawn_entity,
        
        # Planning and Tesseract nodes
        tesseract_visualizer_node,
        motion_planning_node,
    ])
    
    return ld
