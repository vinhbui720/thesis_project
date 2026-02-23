import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
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
    # Launch arguments
    # --------------------------------------------------
    use_real_camera = LaunchConfiguration("use_real_camera")
    declare_use_real_camera = DeclareLaunchArgument(
        "use_real_camera",
        default_value="false",
        description="Set to 'true' to use the physical RealSense camera. 'false' uses Gazebo simulation."
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    controller = LaunchConfiguration("controller")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
    )

    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="joint_trajectory_controller"
    )

    # --------------------------------------------------
    # Gazebo resource path (VERY IMPORTANT)
    # --------------------------------------------------
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(pkg_share)

    # --------------------------------------------------
    # Mesh Processing Config Paths
    # --------------------------------------------------
    model_config = os.path.join(mesh_pkg_share, "config", "model.yaml")
    scene_config = os.path.join(mesh_pkg_share, "config", "scene_param.yaml")
    pose_config = os.path.join(mesh_pkg_share, "config", "pose_param.yaml") 

    # --------------------------------------------------
    # Start Gazebo
    # --------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf --verbose"
        }.items(),
    )

    # --------------------------------------------------
    # Robot description
    # --------------------------------------------------
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "description.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --------------------------------------------------
    # RViz (disable GUI publisher in sim mode)
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
        arguments=[
            "-name", "motoman_motomini",
            "-topic", "/robot_description",
            "-z", "0.0",
        ],
        output="screen",
    )

    # --------------------------------------------------
    # Controller spawners (ORDER MATTERS)
    # --------------------------------------------------
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller],
        output="screen",
    )

    delayed_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    )

    delayed_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[controller_spawner],
        )
    )

    # --------------------------------------------------
    # Base Bridge (Always runs for Gazebo Clock)
    # --------------------------------------------------
    base_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/empty/dynamic_pose/info@ros_gz_interfaces/msg/PoseV[gz.msgs.Pose_V",],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # --------------------------------------------------
    # Sim Camera Bridge (Runs ONLY if use_real_camera == false)
    # --------------------------------------------------
    sim_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=UnlessCondition(use_real_camera),
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_rect_raw'),
            ('/camera/points', '/camera/depth/color/points'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ],
        output="screen",
    )
    
    # --------------------------------------------------
    # Real Camera Node (Runs ONLY if use_real_camera == true)
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
            "camera_namespace": "",       
        }.items()
    )
    
    # --------------------------------------------------
    # TF Link: Attach Real Camera to your URDF
    # --------------------------------------------------
    realsense_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        condition=IfCondition(use_real_camera),
        # This locks 'camera_link' (real driver) to 'world_depth_camera_link' (your URDF)
        arguments=["0", "0", "0", "0", "0", "0", "world_depth_camera_link", "camera_link"]
    )

    # ========================================================================
    # Relay Node & Object Controller
    # ========================================================================
    topic_relay = Node(
        package="topic_tools",
        executable="relay",
        name="joint_command_relay",
        output="screen",
        arguments=[
            "/joint_path_command",                         
            "/joint_trajectory_controller/joint_trajectory" 
        ],
        condition=IfCondition("true")
    )

    # --------------------------------------------------
    # Object Controller (CONTROL ONLY)
    # --------------------------------------------------
    object_controller_node = Node(
        package="motomini",
        executable="object_controller",
        name="object_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # --------------------------------------------------
    # Object Pose Filter (Bridge → Filter → RViz)
    # --------------------------------------------------
    object_pose_node = Node(
        package="motomini",
        executable="object_pose",
        name="object_pose_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # ========================================================================
    # Mesh Processing Pipeline (Runs ONLY if use_real_camera == true)
    # ========================================================================
    model_node = Node(
        package="mesh_processing",
        executable="model_manager_node",
        name="model_manager",
        parameters=[model_config],
        condition=IfCondition(use_real_camera),
        output="screen"
    )

    preprocessor_node = Node(
        package="mesh_processing",
        executable="scene_preprocessor",
        name="scene_preprocessor",
        parameters=[scene_config],
        condition=IfCondition(use_real_camera),
        output="screen"
    )

    pose_estimator_node = Node(
        package="mesh_processing",
        executable="pose_estimator",
        name="pose_estimator",
        parameters=[
            pose_config, 
            {"debug_mode": False}
        ],
        condition=IfCondition(use_real_camera),
        output="screen"
    )

    # --------------------------------------------------
    # Launch
    # --------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_controller,
        declare_use_real_camera,

        gazebo,
        robot_description,
        rviz,

        spawn_robot,
        delayed_joint_state,
        delayed_controller,
        
        sim_camera_bridge,
        real_camera_node,
        realsense_tf,
        base_bridge,

        object_controller_node,
        object_pose_node,
        
        topic_relay,
        model_node,
        preprocessor_node,
        pose_estimator_node,
    ])