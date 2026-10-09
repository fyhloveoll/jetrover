#!/usr/bin/env python3
# encoding: utf-8
# Lightweight monitor stream for viewing the eye-in-hand camera on the laptop.
# Subscribes to the local RGB feed (no network), downscales + throttles, and
# republishes a small raw Image on /jr/view so the laptop can view it in
# rqt_image_view WITHOUT needing the compressed_image_transport plugin.
# Set env JR_YOLO=1 to overlay YOLOv11 detections (heavier; uses the GPU).
# Own ws, does not touch vendor code.
import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT_W = int(os.environ.get('JR_W', '640'))   # output width  (native 640)
OUT_H = int(os.environ.get('JR_H', '360'))   # output height (native 360)
RATE = float(os.environ.get('JR_RATE', '12'))  # max publish Hz
USE_YOLO = os.environ.get('JR_YOLO', '0') == '1'
MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'


class ViewPub(Node):
    def __init__(self):
        super().__init__('jr_view_pub')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/jr/view', 1)
        self.last = 0.0
        self.model = None
        if USE_YOLO:
            from ultralytics import YOLO
            self.get_logger().info('loading YOLO %s ...' % MODEL)
            self.model = YOLO(MODEL)
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, 1)
        self.get_logger().info('publishing /jr/view %dx%d @%.0fHz yolo=%s'
                               % (OUT_W, OUT_H, RATE, USE_YOLO))

    def cb(self, msg):
        now = time.time()
        if now - self.last < 1.0 / RATE:
            return
        self.last = now
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.model is not None:
            res = self.model(img, verbose=False)[0]
            img = res.plot()
        if img.shape[1] != OUT_W or img.shape[0] != OUT_H:
            img = cv2.resize(img, (OUT_W, OUT_H))
        m = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        m.header = msg.header
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ViewPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
