#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        # Publisher topic /arm_controller/joint_trajectory
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        #Config hold, move time of arm
        self.update_rate = 300.0  # 300 Hz
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
        traj_msg.joint_names = self.joint_names

        current_angle = self.current_positions[index]
        angle_step = (target_angle_rad - current_angle) / self.num_steps

        self.get_logger().info(f'Moving arm {index + 1} to {math.degrees(target_angle_rad):.2f} degrees')
        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()

            positions = self.current_positions.copy()
            positions[index] = current_angle + angle_step * step
            point.positions = positions
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)

        self.current_positions[index] = target_angle_rad

    def return_arm_smooth(self, index):
        # Tạo thông điệp JointTrajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        current_angle = self.current_positions[index]
        target_angle = 0.0
        angle_step = (target_angle - current_angle) / self.num_steps

   
        self.get_logger().info(f'Returning arm {index + 1} to 0 degrees')
        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()
            positions = self.current_positions.copy()
            positions[index] = current_angle + angle_step * step
            point.positions = positions
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)

        # Update angle
        self.current_positions[index] = 0.0

    def move_arm(self, index, target_angle_deg):
        target_angle_rad = math.radians(target_angle_deg)
        self.move_arm_smooth(index, target_angle_rad)
        time.sleep(self.hold_duration)
        self.return_arm_smooth(index)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    
    try:
        while rclpy.ok():

            print("Enter arm index (1, 2, 3, or 4), then desired angle (degrees), then press Enter (e.g., '1 30')")
            print("Angle must be between 0 and 45 degrees!")
            print("Enter 'q' to quit")
            user_input = input().strip()
            
            if user_input.lower() == 'q':
                break

            try:
                index_str, angle_str = user_input.split()
                index = int(index_str) - 1  
                angle = float(angle_str)

                if index not in [0, 1, 2, 3]:
                    print("Invalid arm index! Please enter 1, 2, 3, or 4.")
                    continue

                if angle < 0 or angle > 45:
                    print("Invalid angle! Angle must be between 0 and 45 degrees.")
                    continue

                node.move_arm(index, angle)

            except ValueError:
                print("Invalid input! Please enter in format: 'index angle' (e.g., '1 30')")
            except Exception as e:
                print(f"Error: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()