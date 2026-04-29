from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # Import added

def generate_launch_description():
    # 1. Define Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # 2. Get Package Paths
    pkg_share = FindPackageShare("robot_planning")

    # 3. Process URDF (Xacro)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "motomini.xacro"]),
        ]
    )
    
    # FIX: Wrap content in ParameterValue to prevent YAML parsing errors
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 4. Process SRDF (Text)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            PathJoinSubstitution([pkg_share, "config", "motomini.srdf"]),
        ]
    )

    # FIX: Wrap content in ParameterValue here as well
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # 5. Define The Planner Node
    planning_params = PathJoinSubstitution([pkg_share, "config", "planning_params.yaml"])
    feedback_params = PathJoinSubstitution([pkg_share, "config", "feedback_controller.yaml"])

    planner_node = Node(
        package="robot_planning",
        executable="motomini_planning_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_params,
            feedback_params,
        ],
    )

    # 6. Define RViz Node
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "motomini_config.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # 7. Return Launch Description
    nodes_to_start = [
        planner_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
