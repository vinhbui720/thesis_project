import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package name specified in the prompt
    pkg_name = 'motomini'
    
    # Declare the launch argument
    real_hw_arg = DeclareLaunchArgument(
        'real_hw',
        default_value='false',
        description='Set to "true" to launch the real hardware bridge (2axis_controller.py)'
    )
    
    real_hw = LaunchConfiguration('real_hw')

    # Paths to the required files
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", "2axis.urdf.xacro"]),
        ]
    )
    
    # Wrap the command output in a ParameterValue to force it to be evaluated as a string
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", "controllers.yaml"]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "config.rviz"]
    )

    # Core Nodes
    # ONLY run the mock control node if real_hw is FALSE
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
        condition=UnlessCondition(real_hw)
    )

    # Robot State Publisher MUST always run (provides TF tree)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz MUST always run
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Controller Spawners
    # ONLY spawn simulation controllers if real_hw is FALSE
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(real_hw)
    )

    # Delay the gantry controller slightly to ensure the broadcaster is ready
    delay_robot_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gantry_controller", "--controller-manager", "/controller_manager"],
                condition=UnlessCondition(real_hw)
            )
        ]
    )

    # The Real Hardware Bridge Node 
    # ONLY launches if real_hw is TRUE
    hardware_bridge_node = Node(
        package="motomini",
        executable="2axis_controller.py",
        output="screen",
        condition=IfCondition(real_hw)
    )

    return LaunchDescription(
        [
            real_hw_arg,
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller,
            rviz_node,
            hardware_bridge_node,
        ]
    )