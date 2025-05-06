#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_conveyor_gazebo = FindPackageShare('my_conveyor_gazebo')
    pkg_my_robot           = FindPackageShare('my_robot')
    pkg_conveyorbelt       = FindPackageShare('conveyorbelt_gazebo')

    # WORLD file with real moving conveyor belt
    world_file = PathJoinSubstitution([
        pkg_conveyorbelt, 'worlds', 'conveyorbelt.world'
    ])

    # Robotic arm (xacro to URDF)
    xacro_file = PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_conveyor_belt.xacro'])
    robot_desc = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    # Gazebo Launch (single instance)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn robotic arm
    spawn_arm = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_conveyor',
            '-x', '-0.630291', '-y', '1.954772', '-z', '0.900000',
            '-R', '0.0', '-P', '0.0', '-Y', '-3.14',
        ],
        output='screen'
    )

    # Spawners for the controller manager provided by Gazebo plugin
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',          '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay spawners until Gazebo's controller_manager is up
    spawn_controllers = TimerAction(
        period=3.0,
        actions=[jsb_spawner, arm_spawner]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_arm,
        spawn_controllers,
    ])
