import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. GET URDF FROM ROBOT_MODEL PACKAGE
    # ------------------------------------
    model_pkg = 'robot_model'
    urdf_path = os.path.join(
        get_package_share_directory(model_pkg),
        'urdf',
        'motoman_motomini.urdf'  # Ensure this matches your file name
    )

    with open(urdf_path, 'r') as file:
        robot_desc = file.read()

    # 2. DEFINE THE NODES
    # -------------------
    
    # A. The Sliders (Controller)
    # We remap 'joint_states' to 'gui_commands' so the sliders don't 
    # confuse RViz. RViz should only show the REAL robot position.
    gui_sliders = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('/joint_states', '/gui_commands')]
    )

    # B. The Bridge (Your C++ Node)
    # Reads /gui_commands -> Sends Action to Real Robot
    bridge_node = Node(
        package='robot_planning',
        executable='hardware_bridge_node',
        output='screen'
    )

    # C. Robot State Publisher (The Visualizer)
    # This listens to /joint_states from the REAL ROBOT driver
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # D. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Optional: Load a saved config if you have one
        # arguments=['-d', os.path.join(get_package_share_directory('robot_planning'), 'config', 'view_robot.rviz')]
    )

    return LaunchDescription([
        gui_sliders,
        bridge_node,
        rsp,
        rviz
    ])