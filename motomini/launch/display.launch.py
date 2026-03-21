from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition


def generate_launch_description():

    # =========================
    # Launch arguments
    # =========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    tool_type = LaunchConfiguration('tool_type')

    pkg_share = FindPackageShare('motomini')

    # =========================
    # Robot Description (IMPORTANT)
    # =========================
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    pkg_share,
                    "urdf",
                    "motoman_motomini_wrapper_realbot.urdf.xacro"
                ]),
                " tool_type:=", tool_type
            ]),
            value_type=str
        )
    }

    # =========================
    # Robot State Publisher
    # =========================
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen"
    )

    # =========================
    # Joint State Publisher GUI (SLIDER)
    # =========================
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # =========================
    # RViz
    # =========================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([
                pkg_share,
                "config",
                "motomini.rviz"   # adjust if needed
            ])
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # =========================
    # Launch
    # =========================
    return LaunchDescription([

        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false"
        ),

        DeclareLaunchArgument(
            name="tool_type",
            default_value="magnetic"
        ),

        rsp_node,
        jsp_gui_node,  
        rviz_node,
    ])