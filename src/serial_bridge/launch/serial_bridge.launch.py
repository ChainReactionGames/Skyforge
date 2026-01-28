from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_bridge',
            executable='bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',  # serial port for ESP32
                'baud': 115200           # match your ESP32 baud rate
            }]
        )
    ])
