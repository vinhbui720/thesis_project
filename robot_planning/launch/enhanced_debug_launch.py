"""
Enhanced Debug Node Launch File

Launches the enhanced debugging system for trajectory planning and optimization.
This integrates collision visualization, cartesian error tracking, trajectory animation,
and real-time SQP optimization monitoring.

Usage:
    ros2 launch robot_planning enhanced_debug_launch.py
    
Optional Parameters:
    collision_threshold:=0.1
    enable_collision_viz:=true
    enable_cartesian_error_viz:=true
    enable_trajectory_viz:=true
    enable_kinematic_error_viz:=true
    enable_collision_gradient:=false
    ee_link:=ee_link
    base_link:=base_link
    manipulator_group:=manipulator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
import os


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare(package='robot_planning').find('robot_planning')

    # Declare arguments with defaults
    collision_threshold = DeclareLaunchArgument(
        'collision_threshold',
        default_value='0.1',
        description='Collision distance threshold (meters) for warning visualization'
    )

    enable_collision_viz = DeclareLaunchArgument(
        'enable_collision_viz',
        default_value='true',
        description='Enable collision box/margin visualization'
    )

    enable_cartesian_error_viz = DeclareLaunchArgument(
        'enable_cartesian_error_viz',
        default_value='true',
        description='Enable cartesian error visualization (target vs actual)'
    )

    enable_trajectory_viz = DeclareLaunchArgument(
        'enable_trajectory_viz',
        default_value='true',
        description='Enable trajectory path animation'
    )

    enable_kinematic_error_viz = DeclareLaunchArgument(
        'enable_kinematic_error_viz',
        default_value='true',
        description='Enable kinematic error tracking visualization'
    )

    enable_collision_gradient = DeclareLaunchArgument(
        'enable_collision_gradient',
        default_value='false',
        description='Enable collision gradient arrows (directional avoidance)'
    )

    ee_link = DeclareLaunchArgument(
        'ee_link',
        default_value='ee_link',
        description='End-effector link name for kinematic calculations'
    )

    base_link = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link name for coordinate frame'
    )

    manipulator_group = DeclareLaunchArgument(
        'manipulator_group',
        default_value='manipulator',
        description='Manipulator kinematic group name'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Marker frame ID for visualization'
    )

    # Enhanced Debug Node
    enhanced_debug_node = Node(
        package='robot_planning',
        executable='enhanced_debug_node',
        name='enhanced_debug_node',
        output='screen',
        parameters=[
            {
                'collision_threshold': LaunchConfiguration('collision_threshold'),
                'enable_collision_viz': LaunchConfiguration('enable_collision_viz'),
                'enable_cartesian_error_viz': LaunchConfiguration('enable_cartesian_error_viz'),
                'enable_trajectory_viz': LaunchConfiguration('enable_trajectory_viz'),
                'enable_kinematic_error_viz': LaunchConfiguration('enable_kinematic_error_viz'),
                'enable_collision_gradient': LaunchConfiguration('enable_collision_gradient'),
                'ee_link': LaunchConfiguration('ee_link'),
                'base_link': LaunchConfiguration('base_link'),
                'manipulator_group': LaunchConfiguration('manipulator_group'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/debug_markers', '/debug_markers'),
        ]
    )

    return LaunchDescription([
        collision_threshold,
        enable_collision_viz,
        enable_cartesian_error_viz,
        enable_trajectory_viz,
        enable_kinematic_error_viz,
        enable_collision_gradient,
        ee_link,
        base_link,
        manipulator_group,
        frame_id,
        enhanced_debug_node,
    ])
