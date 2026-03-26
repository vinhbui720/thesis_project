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

    # 2. Launch Configurations
    tool_type = LaunchConfiguration("tool_type")
    real_robot = LaunchConfiguration("real_robot")
    debug = LaunchConfiguration("debug")
    gantry_mode = LaunchConfiguration("gantry_mode")
    online_mode = LaunchConfiguration("online_mode")
    use_ompl = LaunchConfiguration("use_ompl")
    tracking_mode = LaunchConfiguration("tracking_mode")
    tracking_rate_hz = LaunchConfiguration("tracking_rate_hz")

    gantry_mode_tesser = IfCondition(PythonExpression(["'", gantry_mode, "' == 'tesser'"]))
    gantry_mode_loop = IfCondition(PythonExpression(["'", gantry_mode, "' == 'loop'"]))
    gantry_mode_gui = IfCondition(PythonExpression(["'", gantry_mode, "' == 'gui'"]))

    # 3. Dynamic Logic
    # Simplified the PythonExpression by using f-strings for readability
    concrete_ee_link = PythonExpression([
        "({'magnetic': 'magnetic_link', 'gripper': 'gripper_link', ",
        "'camera': 'camera_link', 'calib': 'calib_link'}).get('", 
        tool_type, "', 'tool0')"
    ])

    # 4. Robot Descriptions (Combined into reusable dicts)
    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", 
                 PathJoinSubstitution([motomini_share, "urdf", "motoman_motomini_wrapper_realbot.urdf.xacro"]),
                 " tool_type:=", tool_type]), value_type=str)}

    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", 
            PathJoinSubstitution([motomini_share, "urdf", "motoman_motomini.srdf.xacro"]),
            " tool_type:=", tool_type
        ]),value_type=str)}

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
                "online_mode": online_mode,
                "debug": debug,
                "use_ompl": use_ompl,
                "tracking_mode": tracking_mode,
                "tracking_rate_hz": tracking_rate_hz,

            }],
            output="screen"
        ),

        Node(
            package="robot_planning",
            executable="gantry_planning_node",
            parameters=[*common_params, {
                "manipulator_group": "gantry",
                "base_link": "world",
                "ee_link": "gantry_tool_link",
                "debug": debug,
                "use_obstacles": False,
            }],
            condition=gantry_mode_tesser,
            output="screen"
        ),

        Node(
            package="robot_planning",
            executable="gantry_loop_controller_node",
            parameters=[{
                "x_start": 0.0,
                "x_end": -0.28,
                "z_start": 0.0,
                "z_end": -0.06,
                "steps": 28,
                "publish_period_sec": 0.3,
                "motion_time_sec": 0.3,
            }],
            condition=gantry_mode_loop,
            output="screen"
        ),

        Node(
            package="motomini",
            executable="2axis_gui.py",
            condition=gantry_mode_gui,
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

        Node(
            package="rviz2", 
            executable="rviz2", 
            arguments=["-d", PathJoinSubstitution([motomini_share, "config", "motomini.rviz"])],
            parameters=common_params # <-- THIS IS THE CRITICAL ADDITION
        ),
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
    controller_config = PathJoinSubstitution([
        motomini_share, "config", "motoman_controllers.yaml"
    ])

    control_nodes = [

        # -------------------------------
        # ros2_control node (NO remapping here)
        # -------------------------------
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config],
            condition=UnlessCondition(real_robot),
            output="screen"
        ),

        # -------------------------------
        # Delay to ensure controller manager is ready
        # -------------------------------
        ExecuteProcess(
            cmd=["sleep", "2"],
            shell=True,
            condition=UnlessCondition(real_robot)
        ),

        # -------------------------------
        # Spawn controllers
        # -------------------------------
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            condition=UnlessCondition(real_robot),
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motomini_controller"],
            condition=UnlessCondition(real_robot),
            output="screen"
        ),

        # OPTIONAL (only if you really need gantry)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gantry_controller"],
            condition=UnlessCondition(real_robot),
            output="screen"
        ),

        # -------------------------------
        # 🔥 CRITICAL: Relay node (planner → controller)
        # -------------------------------
        Node(
            package="topic_tools",
            executable="relay",
            name="trajectory_relay",
            arguments=[
                "/joint_path_command",
                "/motomini_controller/joint_trajectory"
            ],
            output="screen",
            condition=UnlessCondition(real_robot)
        ),

        Node(
            package="topic_tools",
            executable="relay",
            name="gantry_trajectory_relay",
            arguments=[
                "/gantry/joint_path_command",
                "/gantry_controller/joint_trajectory"
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", real_robot, "' == 'false' and '", gantry_mode, "' == 'tesser'"
            ]))
        ),
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
        DeclareLaunchArgument("debug", default_value="true"),
        DeclareLaunchArgument("gantry_mode", default_value="loop"),
        DeclareLaunchArgument("online_mode", default_value="false"),
        DeclareLaunchArgument("use_ompl", default_value="true"),
        DeclareLaunchArgument("tracking_mode", default_value="false"),
        DeclareLaunchArgument("tracking_rate_hz", default_value="3.0"),
        *nodes,
        *control_nodes,
        *mesh_nodes
    ])