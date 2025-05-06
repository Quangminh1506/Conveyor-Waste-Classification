#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
from yolov8_msgs.msg import Yolov8Inference

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        self.declare_parameter('subscribe_topic', '/yolov8/inference')
        topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            Yolov8Inference,
            topic,
            self.cb_inference,
            10
        )

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.get_name()}/joint_trajectory',
            10
        )

        self.update_rate = 300.0
        self.move_duration = 1.0
        self.hold_duration = 7.0
        self.num_steps = int(self.move_duration * self.update_rate)
        self.step_duration = 1.0 / self.update_rate

        self.joint_names = [
            'axle_0_to_arm_0',
            'axle_1_to_arm_1',
            'axle_2_to_arm_2',
            'axle_3_to_arm_3'
        ]

        self.current_positions = [0.0] * len(self.joint_names)

    def move_arm_smooth(self, index, target_angle_rad):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_names[index]]

        current_angle = self.current_positions[index]
        angle_step = (target_angle_rad - current_angle) / self.num_steps

        self.get_logger().info(f'Moving arm {index + 1} to {math.degrees(target_angle_rad):.2f} degrees')

        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()
            point.positions = [current_angle + angle_step * step]
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)

        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)
        self.current_positions[index] = target_angle_rad

    def return_arm_smooth(self, index):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_names[index]]

        current_angle = self.current_positions[index]
        target_angle = 0.0
        angle_step = (target_angle - current_angle) / self.num_steps

        self.get_logger().info(f'Returning arm {index + 1} to 0 degrees')

        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()
            point.positions = [current_angle + angle_step * step]
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)

        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)
        self.current_positions[index] = 0.0

    def move_arm(self, index, target_angle_deg):
        target_angle_rad = math.radians(target_angle_deg)
        self.move_arm_smooth(index, target_angle_rad)
        time.sleep(self.hold_duration)
        self.return_arm_smooth(index)

    def cb_inference(self, msg: Yolov8Inference):
        if not msg.yolov8_inference:
            return
        det = msg.yolov8_inference[0]
        try:
            arm_index = int(float(det.class_name))
            if 0 <= arm_index < len(self.joint_names):
                self.move_arm(arm_index, 45)
            else:
                self.get_logger().warn(f"Invalid arm index from class: {det.class_name}")
        except ValueError:
            self.get_logger().warn(f"Invalid class_name format: {det.class_name}")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
