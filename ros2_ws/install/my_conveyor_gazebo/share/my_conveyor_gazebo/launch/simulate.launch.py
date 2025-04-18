from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_my_conveyor_gazebo = FindPackageShare('my_conveyor_gazebo')
    pkg_my_robot = FindPackageShare('my_robot')

    # Đường dẫn tới file Xacro
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_conveyor_belt.xacro'])

    # Đường dẫn tới file thế giới
    world_file = PathJoinSubstitution([pkg_my_conveyor_gazebo, 'worlds', 'empty.world'])

    # Chuyển đổi Xacro thành URDF
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    # Khởi động Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot vào Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_conveyor'],
        output='screen'
    )

    # Load controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_my_conveyor_gazebo, 'config', 'controllers.yaml'])
        ],
        output='screen'
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn arm controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])
