from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Đường dẫn tới file URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'my_robot.urdf')

    # Đọc nội dung file URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Đường dẫn tới file cấu hình RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('my_robot'),
        'rviz', 'robot_display.rviz')

    return LaunchDescription([
        # Node robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Node joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',  # In log ra màn hình
            parameters=[{'robot_description': robot_desc}],
        ),

        # Node RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])