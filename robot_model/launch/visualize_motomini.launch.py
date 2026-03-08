import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_model'

    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'robot_state_visualize.rviz'
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':
                    open(os.path.join(
                        get_package_share_directory(pkg_name),
                        'urdf',
                        'motoman_motomini.urdf'
                    )).read()
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
