from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Get path to URDF
    description_pkg = get_package_share_directory('skyforge_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'skyforge.urdf')

    controllers_pkg = get_package_share_directory('skyforge_controllers')
    controllers_file = os.path.join(controllers_pkg, 'config', 'three_link_arm_controllers.yaml')
    
    gazebo_pkg = get_package_share_directory('skyforge_gazebo')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty_no_gravity.sdf')

    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Gazebo (GZ Sim)
    gz_sim = ExecuteProcess(
        #TODO: Make the worlds folder actually make it to the install so we don't have to point back to the src folder
        cmd=['gz', 'sim', '-v', '4', '-r', '/home/dwidjaja/Desktop/Skyforge/src/skyforge_gazebo/worlds/empty_no_gravity.sdf'], 
        output='screen'
    )

    # Publish the robot state (TFs and joint states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'skyforge',
            '-file', urdf_file,
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file, {'robot_description': robot_desc}],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            controllers_file,
            ],
    )
    base_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'base_trajectory_controller',
            '--param-file',
            controllers_file,
            ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        # joint_trajectory_controller_spawner,
        base_trajectory_controller_spawner
    ])
