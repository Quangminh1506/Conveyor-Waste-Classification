#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

bridge = CvBridge()

class WasteClassifier(Node):
    def __init__(self):
        super().__init__('waste_classifier')

        # 1) Load the YOLOv8‑cls weights we just placed as best.pt
        share = get_package_share_directory('yolobot_recognition')
        weights = os.path.join(share, 'scripts', 'best.pt')
        self.model = YOLO(weights)

        # 2) Map class indices 0–3 to pusher 1–4
        self.class_to_pusher = {0:1, 1:2, 2:3, 3:4}

        # 3) Subscribe camera, publish pusher_index
        self.create_subscription(Image, 'rgb_cam/image_raw',
                                 self.camera_callback, 10)
        self.pusher_pub = self.create_publisher(UInt8, '/pusher_index', 10)

        self.get_logger().info("WasteClassifier ready, waiting for images…")

    def camera_callback(self, msg: Image):
        # a) convert to OpenCV
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')

        # b) run classification @416×416, conf=0.5
        res = self.model.predict(source=img, imgsz=416, conf=0.5)[0]

        # c) did we get a classification output?
        if res.probs is None:
            self.get_logger().warn("No classification outputs; skipping")
            return

        # d) top-1 index & confidence
        cls_id   = int(res.probs.top1)
        cls_conf = float(res.probs.top1conf)

        # e) publish only if idx in 0–3
        if cls_id in self.class_to_pusher:
            pidx = self.class_to_pusher[cls_id]
            self.get_logger().info(f"cls={cls_id} ({cls_conf:.2f}) → pusher {pidx}")
            self.pusher_pub.publish(UInt8(data=pidx))
        else:
            self.get_logger().info(f"cls={cls_id} ({cls_conf:.2f}) → no pusher")

def main(args=None):
    rclpy.init(args=args)
    node = WasteClassifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
