#!/usr/bin/env python3
# encoding: utf-8
"""map_dump.py -- write the live /map OccupancyGrid to <name>.pgm/.yaml with a plain rclpy
subscriber (map_saver_cli and /slam_toolbox/save_map both hung on 2026-09-03, likely a /map
QoS mismatch). Tries volatile then transient-local durability. Trinary encoding like nav2.
Usage: python3 map_dump.py <name>   -> ~/jetrover_ws/maps/<name>.{pgm,yaml}"""
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

name = sys.argv[1] if len(sys.argv) > 1 else 'map_%d' % int(time.time())
out = os.path.join(os.path.expanduser('~/jetrover_ws/maps'), name)
rclpy.init()
n = Node('map_dump')
got = {}


def cb(m):
    got['m'] = m


for dur in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=dur)
    sub = n.create_subscription(OccupancyGrid, '/map', cb, q)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_subscription(sub)
    if 'm' in got:
        print('received /map with durability', dur.name); break
if 'm' not in got:
    print('no /map message in 16 s'); rclpy.shutdown(); sys.exit(1)

m = got['m']
w, h, res = m.info.width, m.info.height, m.info.resolution
data = np.array(m.data, np.int16).reshape(h, w)
img = np.full((h, w), 205, np.uint8)          # unknown
img[(data >= 0) & (data <= 25)] = 254          # free
img[data >= 65] = 0                            # occupied
img = np.flipud(img)                           # row 0 = top of the image = max y
with open(out + '.pgm', 'wb') as f:
    f.write(b'P5\n# CREATOR: map_dump.py %.3f m/pix\n%d %d\n255\n' % (res, w, h))
    f.write(img.tobytes())
ox, oy = m.info.origin.position.x, m.info.origin.position.y
with open(out + '.yaml', 'w') as f:
    f.write('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n' % (name, res, ox, oy))
free = int((img == 254).sum()); occ = int((img == 0).sum())
print('%s.pgm/.yaml  %dx%d @%.2fm = %.1fx%.1f m, free %.1f m2, occupied %d cells, origin (%.2f, %.2f)' %
      (out, w, h, res, w * res, h * res, free * res * res, occ, ox, oy))
n.destroy_node(); rclpy.shutdown()
