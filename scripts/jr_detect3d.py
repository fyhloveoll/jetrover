#!/usr/bin/env python3
# encoding: utf-8
# M4.2: YOLO pixel -> registered depth -> 3D point in the camera optical frame.
# Depth is HW-aligned to RGB (same 640x360), so YOLO (u,v) indexes depth directly.
# Back-projection uses the RGB intrinsics. Also probes a few manual pixels to show
# the depth pipeline is clean on opaque surfaces (vs noisy/zero on glass).
# Own ws, does not touch vendor code.
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
# manual opaque probes (approx pixel locations from the annotated frame)
PROBES = {'red_cube': (445, 130), 'blue_cube': (340, 300), 'screwdriver': (360, 215)}


class Detect3D(Node):
    def __init__(self):
        super().__init__('jr_detect3d')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL)
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.rgb_cb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self.depth_cb, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.info_cb, 1)

    def rgb_cb(self, m):
        if self.rgb is None:
            self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def depth_cb(self, m):
        if self.depth is None:
            self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def info_cb(self, m):
        if self.K is None:
            self.K = np.array(m.k).reshape(3, 3)


def median_depth_mm(depth, u, v, win=4):
    h, w = depth.shape
    patch = depth[max(0, v - win):min(h, v + win + 1),
                  max(0, u - win):min(w, u + win + 1)].astype(np.float32)
    vals = patch[patch > 0]
    if vals.size == 0:
        return 0.0
    return float(np.median(vals))


def backproject(u, v, z_mm, fx, fy, cx, cy):
    Z = z_mm / 1000.0
    return ((u - cx) * Z / fx, (v - cy) * Z / fy, Z)


def main():
    rclpy.init()
    node = Detect3D()
    t0 = time.time()
    while (node.rgb is None or node.depth is None or node.K is None) and time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.rgb is None or node.depth is None or node.K is None:
        print('missing data rgb=%s depth=%s K=%s'
              % (node.rgb is not None, node.depth is not None, node.K is not None))
        return

    fx, fy, cx, cy = node.K[0, 0], node.K[1, 1], node.K[0, 2], node.K[1, 2]
    print('intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f' % (fx, fy, cx, cy))

    res = node.model(node.rgb, verbose=False)[0]
    names = res.names
    print('=== YOLO detections: %d ===' % len(res.boxes))
    for b in res.boxes:
        cls = names[int(b.cls)]
        conf = float(b.conf)
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
        z = median_depth_mm(node.depth, u, v)
        X, Y, Z = backproject(u, v, z, fx, fy, cx, cy)
        tag = 'INVALID(glass/edge)' if z == 0 else 'OK'
        print('  %-10s conf=%.2f px=(%d,%d) Z=%.3fm cam_xyz=(%.3f,%.3f,%.3f) %s'
              % (cls, conf, u, v, Z, X, Y, Z, tag))

    print('=== manual opaque probes (depth sanity) ===')
    for name, (u, v) in PROBES.items():
        z = median_depth_mm(node.depth, u, v)
        X, Y, Z = backproject(u, v, z, fx, fy, cx, cy)
        tag = 'INVALID' if z == 0 else 'OK'
        print('  %-12s px=(%d,%d) Z=%.3fm cam_xyz=(%.3f,%.3f,%.3f) %s'
              % (name, u, v, Z, X, Y, Z, tag))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
