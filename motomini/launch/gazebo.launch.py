import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --------------------------------------------------
    # Package Directories
    # --------------------------------------------------
    pkg_name = "motomini"
    pkg_share = get_package_share_directory(pkg_name)
    mesh_pkg_share = get_package_share_directory("mesh_processing") 

    # --------------------------------------------------
    # Launch Configurations
    # --------------------------------------------------
    use_real_camera = LaunchConfiguration("use_real_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    controller = LaunchConfiguration("controller")

    # --------------------------------------------------
    # Declare Arguments
    # --------------------------------------------------
    declare_use_real_camera = DeclareLaunchArgument(
        "use_real_camera", default_value="false",
        description="Set to 'true' to use physical RealSense. 'false' uses Gazebo simulation."
    )
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    declare_controller = DeclareLaunchArgument(
        "controller", default_value="joint_trajectory_controller"
    )

    # --------------------------------------------------
    # Environment Variables
    # --------------------------------------------------
    # Using SetEnvironmentVariable action is cleaner than os.environ inside the function
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.dirname(pkg_share)
    )

    # --------------------------------------------------
    # Start Gazebo
    # --------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf --verbose"}.items(),
    )

    # --------------------------------------------------
    # Robot Description & State Publisher
    # --------------------------------------------------
    # Important: Check if description.launch.py starts an RSP. 
    # If it does, ensure it receives use_sim_time.
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "description.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --------------------------------------------------
    # RViz (Forced Sim Time via ROS args to prevent resets)
    # --------------------------------------------------
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "display.launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "sim_mode": "true"
        }.items(),
    )

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "motoman_motomini", "-topic", "/robot_description", "-z", "0.0"],
        output="screen",
    )

    # --------------------------------------------------
    # Controller Spawners (CRITICAL: Added use_sim_time to all)
    # --------------------------------------------------
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Sequential execution: Spawn -> Joint Broadcaster -> Main Controller
    delayed_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
    )

    delayed_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[controller_spawner])
    )

    # --------------------------------------------------
    # Bridges (Consolidated into one node for better sync)
    # --------------------------------------------------
    combined_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Base Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Camera topics (only if not using real camera)
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        condition=UnlessCondition(use_real_camera),
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_rect_raw'),
            ('/camera/points', '/camera/depth/color/points'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ],
        output="screen",
    )
    
    # Just the clock bridge if using the real camera
    clock_bridge_only = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=IfCondition(use_real_camera),
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # --------------------------------------------------
    # Real Camera & TF (Only if use_real_camera is true)
    # --------------------------------------------------
    real_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
        ),
        condition=IfCondition(use_real_camera),
        launch_arguments={
            "pointcloud.enable": "true", 
            "align_depth.enable": "true",
            "camera_name": "camera",      
        }.items()
    )
    
    realsense_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        condition=IfCondition(use_real_camera),
        arguments=["0", "0", "0", "0", "0", "0", "world_depth_camera_link", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}] 
    )

    # --------------------------------------------------
    # Logic Nodes (Object Controller, Relay, Pose Filter)
    # --------------------------------------------------
    topic_relay = Node(
        package="topic_tools", executable="relay", name="joint_command_relay",
        arguments=["/path_command", "/joint_trajectory_controller/joint_trajectory"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    object_controller_node = Node(
        package="motomini", executable="object_controller", name="object_controller",
        output="screen", parameters=[{"use_sim_time": use_sim_time}]
    )

    object_pose_node = Node(
        package="motomini", executable="object_pose", name="object_pose_filter",
        output="screen", parameters=[{"use_sim_time": use_sim_time}]
    )
    # --------------------------------------------------
    # Mesh Processing Pipeline 
    # --------------------------------------------------
    mesh_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mesh_pkg_share, "launch", "full_pipeline.launch.py")
        ),
        launch_arguments={
            "use_real_camera": use_real_camera,
            "use_rviz": "false",
        }.items()
    )
    # --------------------------------------------------
    # Final Launch Description
    # --------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_controller,
        declare_use_real_camera,
        set_gz_resource_path,

        gazebo,
        robot_description,
        rviz,

        spawn_robot,
        delayed_joint_state,
        delayed_controller,
        
        combined_bridge,
        clock_bridge_only,
        real_camera_node,
        realsense_tf,

        object_controller_node,
        object_pose_node,
        topic_relay,
        # mesh_processing,
    ])