#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
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
            'Plastic':            4,
            'Organic_Food_Waste': 2,
            'Paper_Cardboard':    1,
            'Metal_Other':        3,
            # 'Glass': # no pusher
        }
        # 4) subscribe camera; publish pusher index (delayed)
        self.create_subscription(Image, 'rgb_cam/image_raw',
                                 self.camera_callback, 10)
        self.pusher_pub = self.create_publisher(UInt8, '/pusher_index', 10)

        # only publish once per object pass
        self.object_present = False

        # --- timing configuration ---
        # belt_speed in m/s (must match your Gazebo plugin's max_velocity scaling)
        self.declare_parameter('belt_speed', 1.6667)
        self.belt_speed = self.get_parameter('belt_speed').value
        # distances (m) from camera to each pusher
        self.distances = {
            1: 7.4,  # furthest
            2: 5.8,
            3: 4.2,
            4: 2.6,  # closest
        }
        self.extra_delays = {
            1: 3.0,
            2: 2.0,
            3: 1.0,
            4: 0.0,
        }
        self.get_logger().info("WasteDetectorClassifier ready")

    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # --- DETECTION ---
        det_res = self.det_model.predict(source=img, imgsz=640, conf=0.25)[0]
        boxes = det_res.boxes

        # 1) no detections → reset and skip
        if boxes is None or len(boxes) == 0:
            if self.object_present:
                self.get_logger().debug("Object left zone → ready to trigger again")
            self.object_present = False
            return

        # 2) already handled this object → skip
        if self.object_present:
            return

        # 3) first frame with object → pick best box & classify
        confs    = boxes.conf.cpu().numpy().flatten()
        best_idx = int(confs.argmax())
        x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[best_idx].astype(int)

        crop = img[y1:y2, x1:x2]
        cls_res = self.cls_model.predict(source=crop, imgsz=224, conf=0.0)[0]

        if cls_res.probs is None:
            self.get_logger().warn("Classification gave no probability scores")
            return

        idx      = int(cls_res.probs.top1)       # numeric index
        cls_conf = float(cls_res.probs.top1conf) # confidence
        label    = cls_res.names[idx]            # e.g. 'Plastic'
        pidx     = self.class_to_pusher.get(label, 0)

        if pidx:
            # compute delay = distance / speed
            dist  = self.distances[pidx]
            delay = dist / self.belt_speed + self.extra_delays.get(pidx, 0.0)
            self.get_logger().info(
                f"det→crop cls={label} ({cls_conf:.2f}) → scheduling pusher {pidx} in {delay:.2f}s"
            )

            # schedule exactly one publication when the object arrives
            def timer_cb():
                # publish the pusher signal
                self.pusher_pub.publish(UInt8(data=pidx))
                # cancel this timer so it doesn't repeat
                timer.cancel()

            # create the repeating timer...
            timer = self.create_timer(delay, timer_cb)

            # mark handled so we don't schedule again for this object
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
