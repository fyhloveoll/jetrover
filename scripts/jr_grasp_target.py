#!/usr/bin/env python3
# encoding: utf-8
# M4.3: full perception pipeline -> 3D grasp target in the robot base frame.
#   YOLO(rgb) -> bbox center -> registered depth -> camera-frame XYZ
#   -> tf2 transform (eye-in-hand, depends on arm joints) -> base_link XYZ
# Own ws, does not touch vendor code.
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_point

MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
CAM_FRAME = 'depth_cam_color_optical_frame'
BASE_FRAME = 'base_link'
# COCO classes we treat as graspable targets (opaque household items)
TARGET_CLASSES = {'bottle', 'cup', 'wine glass', 'cell phone'}


class GraspTarget(Node):
    def __init__(self):
        super().__init__('jr_grasp_target')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL)
        self.rgb = None
        self.depth = None
        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.rgb_cb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_cb, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.info_cb, 1)

    def rgb_cb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def depth_cb(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def info_cb(self, m):
        self.K = np.array(m.k).reshape(3, 3)


def median_depth_mm(depth, u, v, win=4):
    h, w = depth.shape
    patch = depth[max(0, v - win):min(h, v + win + 1),
                  max(0, u - win):min(w, u + win + 1)].astype(np.float32)
    vals = patch[patch > 0]
    return float(np.median(vals)) if vals.size else 0.0


def main():
    rclpy.init()
    node = GraspTarget()
    t0 = time.time()
    while (node.rgb is None or node.depth is None or node.K is None) and time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    # let tf buffer fill
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.rgb is None or node.depth is None or node.K is None:
        print('missing data'); return

    fx, fy, cx, cy = node.K[0, 0], node.K[1, 1], node.K[0, 2], node.K[1, 2]
    res = node.model(node.rgb, verbose=False)[0]
    names = res.names
    print('=== grasp targets (base_link frame) ===')
    found = False
    for b in res.boxes:
        cls = names[int(b.cls)]
        if cls not in TARGET_CLASSES:
            continue
        conf = float(b.conf)
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
        z = median_depth_mm(node.depth, u, v)
        if z == 0:
            print('  %-10s conf=%.2f px=(%d,%d) depth INVALID (skip)' % (cls, conf, u, v))
            continue
        Z = z / 1000.0
        cam = PointStamped()
        cam.header.frame_id = CAM_FRAME
        cam.point.x = (u - cx) * Z / fx
        cam.point.y = (v - cy) * Z / fy
        cam.point.z = Z
        try:
            tf = node.tf_buffer.lookup_transform(BASE_FRAME, CAM_FRAME, Time())
            base = do_transform_point(cam, tf)
            print('  %-10s conf=%.2f px=(%d,%d) cam=(%.3f,%.3f,%.3f) -> base=(%.3f,%.3f,%.3f)'
                  % (cls, conf, u, v, cam.point.x, cam.point.y, cam.point.z,
                     base.point.x, base.point.y, base.point.z))
            found = True
        except Exception as e:
            print('  TF error: %s' % e)
    if not found:
        print('  (no valid opaque target detected)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
