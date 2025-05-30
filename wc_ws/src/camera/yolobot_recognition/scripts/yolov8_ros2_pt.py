#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class WasteDetectorClassifier(Node):
    def __init__(self):
        super().__init__('waste_detector_classifier')
        self.bridge = CvBridge()

        # locate your models
        share = get_package_share_directory('yolobot_recognition')
        det_weights = os.path.join(share, 'scripts', 'Detect_YOLOv8.pt')
        cls_weights = os.path.join(share, 'scripts', 'Classi_YOLOv8.pt')

        # 1) detection model
        self.det_model = YOLO(det_weights)
        # 2) classification model
        self.cls_model = YOLO(cls_weights)

        # 3) map class names → pusher indices
        self.class_to_pusher = {
            'Plastic': 1,
            'Organic_Food_Waste': 2,
            'Paper_Cardboard': 3,
            'Metal_Other': 4,
            # 'Glass': 5   # only if you have a 5th pusher
        }

        # 4) subscribe camera; publish pusher index
        self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10
        )
        self.pusher_pub = self.create_publisher(
            UInt8,
            '/pusher_index',
            10
        )

        # state: only publish once per object pass
        self.object_present = False

        self.get_logger().info("WasteDetectorClassifier ready")

    def camera_callback(self, msg: Image):
        # convert to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # --- DETECTION ---
        det_res = self.det_model.predict(
            source=img,
            imgsz=640,
            conf=0.25
        )[0]
        boxes = det_res.boxes

        # 1) no detections → reset flag, skip
        if boxes is None or len(boxes) == 0:
            if self.object_present:
                self.get_logger().debug(
                    "Object left zone → ready to trigger again"
                )
            self.object_present = False
            return

        # 2) already handled this object → skip
        if self.object_present:
            return

        # 3) first frame with object → pick best box
        confs   = boxes.conf.cpu().numpy().flatten()
        best_idx= int(confs.argmax())
        x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[best_idx].astype(int)

        # crop and run classification
        crop   = img[y1:y2, x1:x2]
        cls_res= self.cls_model.predict(
            source=crop,
            imgsz=224,
            conf=0.0
        )[0]

        if cls_res.probs is None:
            self.get_logger().warn("Classification gave no probability scores")
            return

        # built-in attributes: index and confidence
        idx      = int(cls_res.probs.top1)       # e.g. 4 for Plastic in your model
        cls_conf = float(cls_res.probs.top1conf)

        # get human-readable label
        label    = cls_res.names[idx]            # e.g. 'Plastic'

        # lookup pusher
        pidx     = self.class_to_pusher.get(label, 0)
        if pidx:
            self.get_logger().info(
                f"det→crop cls={label} ({cls_conf:.2f}) → pusher {pidx}"
            )
            self.pusher_pub.publish(UInt8(data=pidx))
            self.object_present = True
        else:
            self.get_logger().info(
                f"det→crop cls={label} ({cls_conf:.2f}) → no pusher"
            )

def main(args=None):
    rclpy.init(args=args)
    node = WasteDetectorClassifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
