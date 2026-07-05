#!/usr/bin/env python3
# encoding: utf-8
# Lidar orientation probe: prints the bearing (deg, base convention 0=front,
# +90=left) of the CLOSEST obstacle. Put your hand ~30cm in FRONT of the lidar:
# if it prints ~0, SCAN_YAW=0 is right; if it prints ~180, set JR_SCAN_YAW=3.14.
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('scan_probe')
box = {'m': None}
n.create_subscription(LaserScan, '/scan', lambda m: box.update(m=m), 1)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
    m = box['m']
    if m is None:
        continue
    rng = np.asarray(m.ranges, dtype=np.float32)
    ok = (rng > m.range_min) & (rng < m.range_max)
    if not np.any(ok):
        continue
    i = int(np.argmin(np.where(ok, rng, np.inf)))
    a = math.degrees(m.angle_min + i * m.angle_increment)
    print('closest %.2fm at %+.0fdeg (scan frame)' % (float(rng[i]), a))
    time.sleep(0.5)
n.destroy_node()
rclpy.shutdown()
