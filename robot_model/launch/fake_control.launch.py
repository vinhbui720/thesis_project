from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_model"), "urdf", "motoman_motomini_controller.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Controller Config Path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_model"),
            "config",
            "motoman_controllers.yaml",
        ]
    )

    # 3. RViz Config Path (NEW)
    # Assumes you saved your config as "default.rviz" in the config folder
    rviz_config_file = PathJoinSubstitution(
        [
            get_package_share_directory("robot_model"),
            "config",
            "fake_controller.rviz",
        ]
    )

    # 4. The Controller Manager Node (Main ROS2 Control Node)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/joint_trajectory_controller/joint_trajectory", "/joint_path_command"),
        ]
    )

    # 5. Robot State Publisher (Required for RViz)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 6. RViz2 Node (Updated with config file)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file], # Load the config file
    )

    # 7. Spawners for Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_group_controller", "--inactive"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_group_controller", "--inactive"],
    )

    # Define the list of nodes to launch
    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        velocity_controller_spawner,
        effort_controller_spawner,
    ]

    return LaunchDescription(nodes)