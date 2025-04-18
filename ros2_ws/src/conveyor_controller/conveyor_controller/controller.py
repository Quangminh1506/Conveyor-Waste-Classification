#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        # Publisher cho topic /arm_controller/joint_trajectory
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # Tần suất cập nhật (Hz)
        self.update_rate = 100.0  # 100 Hz
        # Thời gian cho mỗi hành trình (đi và về)
        self.move_duration = 3.0  # 3 giây
        # Số bước trong 3 giây
        self.num_steps = int(self.move_duration * self.update_rate)
        # Thời gian chờ giữa các bước
        self.step_duration = 1.0 / self.update_rate
        # Tên các joint
        self.joint_names = [
            'axle_0_to_arm_0',
            'axle_1_to_arm_1',
            'axle_2_to_arm_2',
            'axle_3_to_arm_3'
        ]

    def move_arm_smooth(self, index, target_angle_rad):
        # Tạo thông điệp JointTrajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        traj_msg.header.stamp = self.get_clock().now().to_msg()

        # Góc hiện tại (giả sử bắt đầu từ 0)
        current_angle = 0.0
        # Tính khoảng cách mỗi bước
        angle_step = (target_angle_rad - current_angle) / self.num_steps

        # Di chuyển đến góc đích
        self.get_logger().info(f'Moving arm {index + 1} to {math.degrees(target_angle_rad):.2f} degrees')
        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()
            # Cập nhật vị trí cho tất cả joint
            positions = [0.0] * len(self.joint_names)
            positions[index] = current_angle + angle_step * step
            point.positions = positions
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)

    def return_arm_smooth(self, index):
        # Tạo thông điệp JointTrajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        traj_msg.header.stamp = self.get_clock().now().to_msg()

        # Góc hiện tại (giả sử là góc đích vừa di chuyển)
        current_angle = math.radians(0.0)  # Đặt lại để đơn giản
        # Góc đích là 0
        target_angle = 0.0
        # Tính khoảng cách mỗi bước
        angle_step = (target_angle - current_angle) / self.num_steps

        # Quay về vị trí ban đầu
        self.get_logger().info(f'Returning arm {index + 1} to 0 degrees')
        for step in range(self.num_steps + 1):
            point = JointTrajectoryPoint()
            positions = [0.0] * len(self.joint_names)
            positions[index] = current_angle + angle_step * step
            point.positions = positions
            point.time_from_start = rclpy.duration.Duration(seconds=(step * self.step_duration)).to_msg()
            traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        time.sleep(self.move_duration)

    def move_arm(self, index, target_angle_deg):
        # Chuyển đổi góc từ độ sang radian
        target_angle_rad = math.radians(target_angle_deg)
        # Di chuyển mượt mà đến góc mong muốn
        self.move_arm_smooth(index, target_angle_rad)
        # Quay về vị trí ban đầu mượt mà
        self.return_arm_smooth(index)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    
    try:
        while rclpy.ok():
            # Nhận input từ terminal
            print("Enter arm index (1, 2, 3, or 4), then desired angle (degrees), then press Enter (e.g., '1 30')")
            print("Angle must be between 0 and 45 degrees!")
            print("Enter 'q' to quit")
            user_input = input().strip()
            
            if user_input.lower() == 'q':
                break

            try:
                # Tách input thành index và góc
                index_str, angle_str = user_input.split()
                index = int(index_str) - 1  # Chuyển index từ 1,2,3,4 thành 0,1,2,3
                angle = float(angle_str)

                # Kiểm tra index hợp lệ
                if index not in [0, 1, 2, 3]:
                    print("Invalid arm index! Please enter 1, 2, 3, or 4.")
                    continue

                # Kiểm tra góc hợp lệ (0 đến 45 độ)
                if angle < 0 or angle > 45:
                    print("Invalid angle! Angle must be between 0 and 45 degrees.")
                    continue

                # Di chuyển cánh tay
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
