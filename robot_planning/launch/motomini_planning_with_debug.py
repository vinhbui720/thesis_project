"""
Complete Planning + Debug Launch File

Launches both the motomini planning system and the enhanced debugging system.
This provides comprehensive visualization and real-time monitoring of planning/optimization.

Usage:
    ros2 launch robot_planning motomini_planning_with_debug.py

Features:
    ✅ Full planning pipeline (trajectory generation)
    ✅ Online collision monitoring
    ✅ Cartesian error tracking
    ✅ Trajectory visualization
    ✅ Kinematic error monitoring
    ✅ Real-time SQP optimization feedback
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable, FindPackageShare, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_planning').find('robot_planning')

    # Declare common arguments
    planning_config = DeclareLaunchArgument(
        'planning_config',
        default_value=os.path.join(pkg_share, 'config', 'motomini_planning_config.yaml'),
        description='Path to planning configuration file'
    )

    robot_description_file = DeclareLaunchArgument(
        'robot_description_file',
        default_value=os.path.join(pkg_share, 'urdf', 'motomini.urdf.xacro'),
        description='Path to robot URDF description'
    )

    # Arguments for debug node
    collision_threshold = DeclareLaunchArgument(
        'collision_threshold',
        default_value='0.1',
        description='Collision distance threshold'
    )

    enable_debug_features = DeclareLaunchArgument(
        'enable_debug_features',
        default_value='true',
        description='Enable all debug visualizations'
    )

    # Include main planning launch
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'motomini_planning.launch.py')
        ),
        launch_arguments={
            'robot_description_file': LaunchConfiguration('robot_description_file'),
            'planning_config': LaunchConfiguration('planning_config'),
        }.items()
    )

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            os.path.join(pkg_share, 'urdf', 'motomini.xacro'),
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            FindExecutable(name='cat'),
            ' ',
            os.path.join(pkg_share, 'config', 'motomini.srdf'),
        ]
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_content, value_type=str)
    }
    collision_wrench_config = os.path.join(pkg_share, 'config', 'collision_wrench.yaml')

    # Enhanced Debug Node
    enhanced_debug = Node(
        package='robot_planning',
        executable='enhanced_debug_node',
        name='enhanced_debug_node',
        output='screen',
        parameters=[
            {
                'collision_threshold': LaunchConfiguration('collision_threshold'),
                'enable_collision_viz': LaunchConfiguration('enable_debug_features'),
                'enable_cartesian_error_viz': LaunchConfiguration('enable_debug_features'),
                'enable_trajectory_viz': LaunchConfiguration('enable_debug_features'),
                'enable_kinematic_error_viz': LaunchConfiguration('enable_debug_features'),
                'enable_collision_gradient': LaunchConfiguration('enable_debug_features'),
                'ee_link': 'ee_link',
                'base_link': 'base_link',
                'manipulator_group': 'manipulator',
                'frame_id': 'world',
            }
        ]
    )

    # Online collision debugger (existing)
    collision_debugger = Node(
        package='robot_planning',
        executable='online_collision_debugger',
        name='online_collision_debugger',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            collision_wrench_config,
            {
                'collision_threshold': LaunchConfiguration('collision_threshold'),
            }
        ]
    )

    return LaunchDescription([
        planning_config,
        robot_description_file,
        collision_threshold,
        enable_debug_features,
        planning_launch,
        enhanced_debug,
        collision_debugger,
    ])
