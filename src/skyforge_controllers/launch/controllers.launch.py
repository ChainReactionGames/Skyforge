from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('skyforge_controllers')
    config = os.path.join(pkg_share, 'config', 'three_link_arm_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )
    
    base_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_trajectory_controller'],
    )


    trajectory_publisher = Node(
        package='skyforge_controllers',
        executable='three_link_trajectory_publisher',
        output='screen'
    )

    # Pick EITHER this one OR the stepping one OR the oscillating one
    base_trajectory_publisher = Node(
        package='skyforge_controllers',
        executable='base_trajectory_publisher',
        output='screen'
    )

    base_stepping_trajectory_publisher = Node(
        package='skyforge_controllers',
        executable='base_stepping_trajectory_publisher',
        output='screen'
    )

    base_oscillating_trajectory_publisher = Node(
        package='skyforge_controllers',
        executable='base_oscillating_trajectory_publisher',
        output='screen'
    )



    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller,
        base_trajectory_controller,
        # trajectory_publisher,
        base_trajectory_publisher
    ])
