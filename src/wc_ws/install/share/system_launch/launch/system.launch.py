#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    get_pkg = get_package_share_directory

    # 1) Launch simulation (Gazebo + real conveyor + robotic arm)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_pkg('my_conveyor_gazebo'), 'launch', 'simulate.launch.py')
        )
    )

    # 2) Camera spawn + AI (YOLOv8) + robot controller
    yolobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_pkg('yolobot_gazebo'), 'launch', 'yolobot_launch.py')
        )
    )

    # 3) Arm control subscriber node that listens to YOLO inference
    arm_ctrl = Node(
        package='conveyor_controller',
        executable='controller',
        name='arm_controller',  # changed to avoid name clash
        output='screen',
        parameters=[{'subscribe_topic': '/yolov8/inference'}]
    )

    return LaunchDescription([
        sim_launch,         # Launch conveyor world + robotic arm
        yolobot_launch,     # Launch camera + AI + yolobot base control
        arm_ctrl            # Launch custom arm control node
    ])
