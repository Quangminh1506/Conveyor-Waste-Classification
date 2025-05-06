import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Config paths
    pkg_share = get_package_share_directory('my_conveyor_gazebo')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),

        # Launch ros2_control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_file],
            output='screen'
        ),

        # Spawner for joint_state_broadcaster
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Spawner for arm_controller
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Your conveyor controller node (python)
        Node(
            package='yolobot_control',
            executable='robot_control.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

