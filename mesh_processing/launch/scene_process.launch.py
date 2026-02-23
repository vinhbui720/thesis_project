import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'mesh_processing'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    default_config_path = os.path.join(pkg_share, 'config', 'scene_param.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config', 'mesh_processing.rviz')

    # RealSense package share
    realsense_pkg_share = get_package_share_directory('realsense2_camera')

    # ------------------------
    # Launch Arguments
    # ------------------------
    tuning_arg = DeclareLaunchArgument(
        'tuning',
        default_value='false',
        description='Set to true to launch the Python tuning GUI'
    )

    # ------------------------
    # 1. RealSense Launch
    # Equivalent to:
    # ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
    # ------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'true'
        }.items()
    )

    # ------------------------
    # 2. Your C++ Processing Node
    # ------------------------
    preprocessor_node = Node(
        package=pkg_name,
        executable='scene_preprocessor',
        name='scene_preprocessor',
        output='screen',
        parameters=[default_config_path]
    )

    # ------------------------
    # 3. Optional Python GUI
    # ------------------------
    # gui_node = Node(
    #     package=pkg_name,
    #     executable='gui_tuner.py',
    #     name='gui_tuner',
    #     condition=IfCondition(LaunchConfiguration('tuning'))
    # )

    # ------------------------
    # 4. RViz2 with config file
    # Equivalent to:
    # rviz2 -d <config_file>
    # ------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        tuning_arg,
        realsense_launch,
        preprocessor_node,
        # gui_node,
        rviz_node
    ])
