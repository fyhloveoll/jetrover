#!/usr/bin/env python3
# read-only 4-direction clearance probe -- same math as jr_grasp_all.clearance()
import math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SCAN_YAW = float(os.environ.get('JR_SCAN_YAW', '0'))

class Probe(Node):
    def __init__(self):
        super().__init__('clearance_probe')
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.cb, 1)
    def cb(self, m):
        self.scan = m
    def clearance(self, direction):
        m = self.scan
        rng = np.asarray(m.ranges, dtype=np.float32)
        ang = m.angle_min + np.arange(rng.size) * m.angle_increment + SCAN_YAW
        d = (ang - direction + np.pi) % (2 * np.pi) - np.pi
        sel = (np.abs(d) <= 0.42) & (rng > m.range_min) & (rng < m.range_max)
        return float(rng[sel].min()) if np.any(sel) else None

rclpy.init()
n = Probe()
t0 = time.time()
while n.scan is None and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.scan is None:
    print('NO SCAN'); raise SystemExit(1)
for _ in range(3):
    rclpy.spin_once(n, timeout_sec=0.3)
    vals = [('front', 0.0), ('left', math.pi/2), ('right', -math.pi/2), ('back', math.pi)]
    print(' | '.join('%s=%.2fm' % (name, c) if (c := n.clearance(a)) is not None
                     else '%s=None' % name for name, a in vals))
    time.sleep(0.5)
