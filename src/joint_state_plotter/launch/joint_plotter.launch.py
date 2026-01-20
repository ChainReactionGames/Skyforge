from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_state_plotter',  # corrected package name
            executable='joint_state_plotter',  # your node entry point
            name='joint_state_plotter',
            output='screen',
            parameters=[{
                'reference_positions': {
                    'joint1': 1.0,
                    'joint2': 0.5
                }
            }]
        )
    ])
