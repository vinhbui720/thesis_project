import os

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "mesh_processing"
    pkg_share = get_package_share_directory(pkg_name)
    realsense_pkg_share = get_package_share_directory("realsense2_camera")

    use_real_camera = LaunchConfiguration("use_real_camera")
    declare_use_real_camera = DeclareLaunchArgument(
        "use_real_camera",
        default_value="false",
        description="Set to 'true' to use the physical RealSense camera. 'false' uses Gazebo simulation."
    )
    use_rviz = LaunchConfiguration("use_rviz")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Set to 'true' to launch RViz visualization. 'false' to skip."
    )
    # ----------------------------
    # Config paths
    # ----------------------------
    model_config = os.path.join(pkg_share, "config", "model.yaml")
    scene_config = os.path.join(pkg_share, "config", "scene_param.yaml")
    # Assuming you create a config for the pose estimator parameters, or you can pass them inline
    pose_config = os.path.join(pkg_share, "config", "pose_param.yaml") 
    trajectory_config = os.path.join(pkg_share, "config", "trajectory_param.yaml")
    rviz_config = os.path.join(pkg_share, "config", "mesh_processing.rviz")

    # ----------------------------
    # RealSense Driver
    # ----------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "camera_name": "camera",      
            "camera_namespace": "",  
        }.items(),
        condition=IfCondition(use_real_camera)
    )

    # ----------------------------
    # STL Model Publisher (Node 1)
    # ----------------------------
    model_node = Node(
        package=pkg_name,
        executable="model_manager_node",  # Ensure this matches your CMakeLists.txt
        name="model_manager",
        parameters=[model_config],
        output="screen"
    )

    # ----------------------------
    # Scene Preprocessing (Node 2)
    # ----------------------------
    preprocessor_node = Node(
        package=pkg_name,
        executable="scene_preprocessor",  # Ensure this matches your CMakeLists.txt
        name="scene_preprocessor",
        parameters=[scene_config],
        output="screen"
    )

    # ----------------------------
    # Pose Estimator (Node 3)
    # ----------------------------
    pose_estimator_node = Node(
        package=pkg_name,
        executable="pose_estimator",     # Ensure this matches your CMakeLists.txt
        name="pose_estimator",
        parameters=[
            pose_config, 
            {"debug_mode": True}         # Forcing debug mode ON directly from launch
        ],
        output="screen"
    )

    # ----------------------------
    # Trajectory Visualizer (Node 4)
    # ----------------------------
    trajectory_visualizer_node = Node(
        package=pkg_name,
        executable="trajectory_visualizer",
        name="trajectory_visualizer",
        parameters=[trajectory_config],
        output="screen"
    )

    # ----------------------------
    # RViz Visualization
    # ----------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_real_camera,
        declare_use_rviz,

        realsense_launch,
        model_node,
        preprocessor_node,
        pose_estimator_node,
        trajectory_visualizer_node,
        rviz_node
    ])