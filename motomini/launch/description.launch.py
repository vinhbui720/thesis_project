import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = "motomini"
    pkg_share = get_package_share_directory(pkg_name)

    # --------------------------------------------------
    # Paths
    # --------------------------------------------------
    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "motoman_motomini_wrapper.urdf.xacro"
    )

    controllers_file = os.path.join(
        pkg_share,
        "config",
        "controllers.yaml"
    )

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    # --------------------------------------------------
    # Generate robot_description from xacro
    # --------------------------------------------------
    robot_description_content = Command([
        "xacro ",
        xacro_file,
        " controllers_file:=",
        controllers_file
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node
    ])
