from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. The Standard GUI (The sliders)
    # Remapped to publish to /gui_commands instead of /joint_states
    gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[
            ('/joint_states', '/gui_commands')
        ]
    )

    # 2. Your New C++ Bridge
    # Reads /gui_commands -> Sends Action to Hardware
    bridge_node = Node(
        package='robot_model',
        executable='hardware_bridge_node',
        name='hardware_bridge_node',
        output='screen'
    )

    # 3. RViz (To see the robot)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Add your config file path here if needed
        # arguments=['-d', '/path/to/config.rviz']
    )

    return LaunchDescription([
        gui_node,
        bridge_node,
        rviz_node
    ])