#!/usr/bin/env python3
# encoding: utf-8
# Capture one synchronized RGB + depth + camera_info frame and save to disk,
# for offline development of the scene segmenter. Camera only -- does NOT touch
# the control board.
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

rclpy.init()
n = Node('cap')
b = CvBridge()
d = {}
n.create_subscription(Image, '/depth_cam/rgb/image_raw', lambda m: d.setdefault('rgb', b.imgmsg_to_cv2(m, 'bgr8')), 1)
n.create_subscription(Image, '/depth_cam/depth/image_raw', lambda m: d.setdefault('depth', b.imgmsg_to_cv2(m, '16UC1')), 1)
n.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', lambda m: d.setdefault('K', list(m.k)), 1)

t0 = time.time()
while not all(k in d for k in ('rgb', 'depth', 'K')) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)

if all(k in d for k in ('rgb', 'depth', 'K')):
    cv2.imwrite('/home/ubuntu/jetrover_ws/cap_rgb.png', d['rgb'])
    np.save('/home/ubuntu/jetrover_ws/cap_depth.npy', d['depth'])
    np.save('/home/ubuntu/jetrover_ws/cap_K.npy', np.array(d['K']))
    valid = int(np.count_nonzero(d['depth']))
    print('saved rgb=%s depth=%s validpx=%d K=%s'
          % (d['rgb'].shape, d['depth'].shape, valid, [round(x, 1) for x in d['K'][:6]]))
else:
    print('MISSING:', [k for k in ('rgb', 'depth', 'K') if k not in d])
n.destroy_node()
rclpy.shutdown()
