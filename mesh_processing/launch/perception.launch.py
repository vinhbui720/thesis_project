import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    model_node = Node(
        package="mesh_processing",
        executable="model_manager_node",
        name="model_manager",
        parameters=[
            os.path.join(
                get_package_share_directory("mesh_processing"),
                "config",
                "model.yaml"
            )
        ],
    )

    return LaunchDescription([
        model_node
    ])