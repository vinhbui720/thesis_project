import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Path Helpers
    motomini_share = FindPackageShare("motomini")
    planning_share = FindPackageShare("robot_planning")

    # 2. Launch Configurations
    tool_type = LaunchConfiguration("tool_type")
    real_robot = LaunchConfiguration("real_robot")
    debug = LaunchConfiguration("debug")

    # 3. Dynamic Logic
    # Simplified the PythonExpression by using f-strings for readability
    concrete_ee_link = PythonExpression([
        "{'magnetic': 'magnetic_link', 'gripper': 'gripper_link', "
        "'camera': 'camera_link', 'calib': 'calib_link'}['", tool_type, "']"
    ])

    # 4. Robot Descriptions (Combined into reusable dicts)
    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", 
                 PathJoinSubstitution([motomini_share, "urdf", "motoman_motomini_wrapper_realbot.urdf.xacro"]),
                 " tool_type:=", tool_type]), value_type=str)}

    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        Command(["cat ", PathJoinSubstitution([motomini_share, "urdf", "motoman_motomini.srdf"])]),
        value_type=str)}


    object_urdf_path = PathJoinSubstitution([
            motomini_share, "urdf", "custom_object.urdf.xacro"
        ])
    
    object_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", object_urdf_path]), 
        value_type=str)}
    # 5. Node Definitions
    common_params = [robot_description, robot_description_semantic]

    nodes = [
        # Planning Node
        Node(
            package="robot_planning",
            executable="motomini_planning_node",
            parameters=[*common_params, {
                "manipulator_group": "manipulator",
                "base_link": "world",
                "ee_link": concrete_ee_link,
                "tool_type": tool_type
            }],
            output="screen"
        ),

        # State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
       Node(
            package="robot_state_publisher", 
            executable="robot_state_publisher", 
            name="object_state_publisher",
            parameters=[object_description],
            remappings=[("/robot_description", "/object_description")]
        ),

        # ---> NEW: Object TF Broadcaster <---
        # Update the 'package' name if you built this cpp file in 'robot_planning' instead of 'motomini'
        Node(
            package="motomini", 
            executable="object_tf_broadcaster",
            name="object_tf_broadcaster",
            parameters=[{
                "ee_link": concrete_ee_link,
                "real_robot": real_robot
            }],
            output="screen"
        ),

        # RViz
        Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([motomini_share, "config", "motomini.rviz"])]),

        # Collision Debugger
        Node(package="robot_planning", executable="online_collision_debugger", parameters=common_params, condition=IfCondition(debug)),

        # Manual Controller (FIXED: Uses FindPackageShare instead of hardcoded home path)
        Node(
            package="motomini",
            executable="manual_controller.py",
            output="screen",
            condition=IfCondition(debug)
        ),
        Node(
            package="motomini",
            executable="env_tf_broadcaster",
            name="env_tf_broadcaster",
            output="screen"
        ),
        Node(
            package='motomini',
            executable='command_node',
            name='command_node',
            output='screen',
            condition=UnlessCondition(debug)
        )
    ]

    # 6. Controller Manager & Spawners (Conditional)
    controller_config = PathJoinSubstitution([motomini_share, "config", "motoman_controllers.yaml"])
    
    control_nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config],
            condition=UnlessCondition(real_robot),
            remappings=[("/joint_trajectory_controller/joint_trajectory", "/joint_path_command")]
        ),
        # Spawners
        *[Node(package="controller_manager", executable="spawner", arguments=[s], condition=UnlessCondition(real_robot)) 
          for s in ["joint_state_broadcaster", "joint_trajectory_controller"]]
    ]
    # Mesh processing 
    mesh_nodes = [

        # Point cloud processing
        Node(
            package="mesh_processing",
            executable="process.py",
            output="screen",
            arguments=["--realcam"],
            condition=IfCondition(real_robot)
        ),

        Node(
            package="mesh_processing",
            executable="process.py",
            output="screen",
            condition=UnlessCondition(real_robot)
        ),

        # ICP registration
        Node(
            package="mesh_processing",
            executable="icp.py",
            output="screen",
            arguments=["--debug"],
            condition=IfCondition(debug)
        ),

        Node(
            package="mesh_processing",
            executable="icp.py",
            output="screen",
            condition=UnlessCondition(debug)
        ),

        # Picking pose generator
        Node(
            package="mesh_processing",
            executable="picking_pose.py",
            output="screen",
            arguments=["--debug"],
            condition=IfCondition(debug)
        ),

        Node(
            package="mesh_processing",
            executable="picking_pose.py",
            output="screen",
            condition=UnlessCondition(debug)
        )
    ]
    return LaunchDescription([
        DeclareLaunchArgument("tool_type", default_value="magnetic"),
        DeclareLaunchArgument("real_robot", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),
        *nodes,
        *control_nodes,
        *mesh_nodes
    ])