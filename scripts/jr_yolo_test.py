#!/usr/bin/env python3
# encoding: utf-8
# M4.1 smoke test: grab one RGB frame from the Dabai cam, run YOLOv11 (COCO) on
# the Jetson GPU, print detections (class, conf, pixel center, box) and save an
# annotated image. Own ws, does not touch vendor code.
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
RGB_TOPIC = '/depth_cam/rgb/image_raw'


class YoloTest(Node):
    def __init__(self):
        super().__init__('jr_yolo_test')
        self.bridge = CvBridge()
        self.get_logger().info('loading model %s ...' % MODEL)
        self.model = YOLO(MODEL)
        self.frame = None
        self.sub = self.create_subscription(Image, RGB_TOPIC, self.cb, 1)

    def cb(self, msg):
        if self.frame is None:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')


def main():
    rclpy.init()
    node = YoloTest()

    t0 = time.time()
    while node.frame is None and time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.frame is None:
        print('ERROR: no RGB frame received')
        return

    h, w = node.frame.shape[:2]
    print('frame %dx%d' % (w, h))

    t1 = time.time()
    res = node.model(node.frame, verbose=False)[0]
    dt = (time.time() - t1) * 1000.0
    print('inference %.0f ms on device=%s' % (dt, node.model.device))

    names = res.names
    n = len(res.boxes)
    print('detections: %d' % n)
    for b in res.boxes:
        cls = names[int(b.cls)]
        conf = float(b.conf)
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        print('  %-12s conf=%.2f center=(%.0f,%.0f) box=(%.0f,%.0f,%.0f,%.0f)'
              % (cls, conf, cx, cy, x1, y1, x2, y2))

    import cv2
    out = '/home/ubuntu/jetrover_ws/yolo_out.jpg'
    cv2.imwrite(out, res.plot())
    print('saved %s' % out)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
