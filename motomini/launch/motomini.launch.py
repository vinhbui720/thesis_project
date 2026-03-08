from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # --------------------------------------------------
    # Launch Arguments
    # --------------------------------------------------
    tool_type = LaunchConfiguration("tool_type")
    real_robot = LaunchConfiguration("real_robot")

    declare_tool_type = DeclareLaunchArgument(
        "tool_type",
        default_value="magnetic",
        description="Tool attached to the robot",
    )

    declare_real_robot = DeclareLaunchArgument(
        "real_robot",
        default_value="false",
        description="If true, do not start controllers (use real hardware topics)",
    )

    # --------------------------------------------------
    # Robot Description
    # --------------------------------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("motomini"),
                    "urdf",
                    "motoman_motomini_wrapper_realbot.urdf.xacro",
                ]
            ),
            " ",
            "tool_type:=",
            tool_type,
        ]
    )
    
    srdf_file = PathJoinSubstitution(
        [
            FindPackageShare("motomini"),
            "urdf",
            "motoman_motomini.srdf",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str
        )
    }

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    planning_node = Node(
        package="robot_planning",
        executable="motomini_planning_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
                "tool_type": tool_type
            }
        ],
    )
    # --------------------------------------------------
    # Controller Config Path
    # --------------------------------------------------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("motomini"),
            "config",
            "motoman_controllers.yaml",
        ]
    )

    # --------------------------------------------------
    # RViz Config
    # --------------------------------------------------
    rviz_config_file = PathJoinSubstitution(
        [
            get_package_share_directory("motomini"),
            "config",
            "motomini.rviz",
        ]
    )

    # --------------------------------------------------
    # ROS2 Control Node (disabled for real robot)
    # --------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=UnlessCondition(real_robot),
        remappings=[
            ("/joint_trajectory_controller/joint_trajectory", "/joint_path_command"),
        ],
    )

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # --------------------------------------------------
    # Controller Spawners (disabled for real robot)
    # --------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=UnlessCondition(real_robot),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        condition=UnlessCondition(real_robot),
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_group_controller", "--inactive"],
        condition=UnlessCondition(real_robot),
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_group_controller", "--inactive"],
        condition=UnlessCondition(real_robot),
    )

    # --------------------------------------------------
    # Launch
    # --------------------------------------------------
    return LaunchDescription([
        declare_tool_type,
        declare_real_robot,
        control_node,
        robot_state_pub_node,
        rviz_node,
        planning_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        velocity_controller_spawner,
        effort_controller_spawner,
    ])