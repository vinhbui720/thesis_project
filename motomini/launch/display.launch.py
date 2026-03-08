from pathlib import Path
from launch_ros.actions import Node
import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():

    pkg_share = Path(
        launch_ros.substitutions.FindPackageShare(
            package='motomini'
        ).find('motomini')
    )

    default_model_path = pkg_share / 'urdf/motoman_motomini_wrapper.urdf.xacro'
    default_rviz_config_path = pkg_share / 'rviz/robot_description.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    tool_type = LaunchConfiguration('tool_type')
    
    # ----------------------------------
    # Robot State Publisher (always needed)
    # ----------------------------------
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motomini'),
                'launch',
                'description.launch.py',
            ]),
        ]),
        launch_arguments=dict(
            use_sim_time=use_sim_time,
            tool_type=tool_type
        ).items(),
    )
    # ----------------------------------
    # Joint State Publisher (ONLY if not sim mode)
    # ----------------------------------
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=UnlessCondition(sim_mode),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ----------------------------------
    # RViz
    # ----------------------------------
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(default_rviz_config_path)],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return launch.LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            name='sim_mode',
            default_value='false',
            description='Enable simulation mode (disables joint_state_publisher_gui)'
        ),
        DeclareLaunchArgument(
            name='tool_type',
            default_value='magnetic',
            description='Tool attached to robot'
        ),
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
    ])
