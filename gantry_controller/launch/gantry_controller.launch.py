from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gantry_controller',
            executable='modbus_hw_bridge',
            name='modbus_hardware_bridge',
            output='screen',
            parameters=[{
                'plc_ip':           '192.168.1.1',
                'plc_port':         502,
                'plc_unit_id':      1,
                'scale_factor':     10000.0,
                'default_velocity': 100,
                'feedback_rate_hz': 20.0,
            }],
        ),
    ])
