from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
import os
import tempfile

def generate_launch_description():
    # -------------------------
    # Paths
    # -------------------------
    # URDF from skyforge_description
    description_pkg = get_package_share_directory('skyforge_description')
    urdf_xacro_file = os.path.join(description_pkg, 'urdf', 'skyforge.urdf.xacro')

    # Empty world from ros_gz_sim
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(ros_gz_sim_share, 'worlds', 'empty.sdf')

    # Process xacro and get URDF contents
    robot_desc = process_file(urdf_xacro_file).toxml()
    
    # Write processed URDF to a temporary file for Gazebo to use
    urdf_temp_fd, urdf_temp_file = tempfile.mkstemp(suffix='.urdf', text=True)
    with os.fdopen(urdf_temp_fd, 'w') as f:
        f.write(robot_desc)

    # -------------------------
    # Launch Gazebo
    # -------------------------
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # -------------------------
    # Joint State Broadcaster
    # -------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # -------------------------
    # Spawn Robot in Gazebo
    # -------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'skyforge',
            '-file', urdf_temp_file,
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # -------------------------
    # Launch Description
    # -------------------------
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        spawn_entity
    ])
