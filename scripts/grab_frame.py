#!/usr/bin/env python3
"""grab_frame.py OUT.jpg -- save one RGB frame from the depth camera (sensor QoS), plus AprilTag count if detector importable"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np, cv2

class G(Node):
    def __init__(self):
        super().__init__('grab_frame'); self.img = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.cb, qos_profile_sensor_data)
    def cb(self, m):
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        self.img = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()

rclpy.init(); g = G(); t = time.monotonic()
while g.img is None and time.monotonic() - t < 8: rclpy.spin_once(g, timeout_sec=0.2)
if g.img is None: print('no frame'); sys.exit(1)
cv2.imwrite(sys.argv[1], g.img, [cv2.IMWRITE_JPEG_QUALITY, 80]); print('saved', g.img.shape)
try:
    sys.path.insert(0, '/home/ubuntu/jetrover_ws'); import handeye_calib
    d = handeye_calib.detect(g.img); print('tags detected:', len(d) if d is not None else d)
except Exception as e:
    print('detect skipped:', e)
g.destroy_node(); rclpy.shutdown()
