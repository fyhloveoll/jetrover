#!/usr/bin/env python3
# encoding: utf-8
# jr_vision scene node: class-agnostic object segmentation as a live ROS node.
# Subscribes depth+rgb+camera_info, removes the floor plane (scene_segment),
# and publishes an annotated stream + an object registry (each blob has a
# stable-ish id). CAMERA ONLY -- does NOT touch the control board (no hang risk).
#
# Publishes:
#   /jr/scene/annotated             sensor_msgs/Image            (rgb + numbered boxes)
#   /jr/scene/annotated/compressed  sensor_msgs/CompressedImage  (JPEG, for laptop viewing)
#   /jr/scene/objects               std_msgs/String              (JSON: [{id,u,v,bbox,dist}])
#
# View on the laptop:  scripts/view.sh /jr/scene/annotated   (or jr_click_viewer.py)
# Downstream: a click/command picks an id -> the M5 grasp pipeline grasps that blob.
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from jr_vision.scene_segment import segment, annotate


class SceneNode(Node):
    def __init__(self):
        super().__init__('jr_scene')
        self.declare_parameter('rate', 3.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_dist', 0.7)
        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('info_topic', '/depth_cam/depth/camera_info')
        g = self.get_parameter
        self.jpeg_q = int(g('jpeg_quality').value)
        self.max_dist = float(g('max_dist').value)

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        cb = qos_profile_sensor_data
        self.create_subscription(Image, g('rgb_topic').value, self._rgb, cb)
        self.create_subscription(Image, g('depth_topic').value, self._depth, cb)
        self.create_subscription(CameraInfo, g('info_topic').value, self._info, cb)
        self.pub_img = self.create_publisher(Image, '/jr/scene/annotated', 1)
        self.pub_cmp = self.create_publisher(CompressedImage, '/jr/scene/annotated/compressed', 1)
        self.pub_obj = self.create_publisher(String, '/jr/scene/objects', 1)
        self.create_timer(1.0 / float(g('rate').value), self._tick)
        self.get_logger().info('jr_scene up (class-agnostic floor-removal segmentation)')

    def _rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def _depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def _info(self, m):
        self.K = list(m.k)

    def _tick(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return
        rgb, depth, K = self.rgb, self.depth, self.K   # snapshot
        try:
            blobs, _ = segment(depth, K, max_dist=self.max_dist)
        except Exception as e:
            self.get_logger().warn('segment failed: %s' % e)
            return
        img = annotate(rgb, blobs)
        m = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.pub_img.publish(m)
        ok, enc = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
        if ok:
            c = CompressedImage()
            c.format = 'bgr8; jpeg compressed bgr8'
            c.data = enc.tobytes()
            self.pub_cmp.publish(c)
        self.pub_obj.publish(String(data=json.dumps(blobs)))


def main():
    rclpy.init()
    node = SceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
