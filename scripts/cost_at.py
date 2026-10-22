#!/usr/bin/env python3
"""cost_at.py x,y,label ...  -- print AMCL pose and costmap values (global/local/map/keepout) at the given map points"""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

def q2yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

class N(Node):
    def __init__(self):
        super().__init__('cost_at')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.g = {}
        for name, topic in (('global', '/global_costmap/costmap'), ('local', '/local_costmap/costmap'),
                            ('map', '/map'), ('keepout', '/keepout_filter_mask')):
            self.create_subscription(OccupancyGrid, topic, (lambda n: (lambda m: self.g.__setitem__(n, m)))(name), qos)
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: setattr(self, 'pose', m), qos)

def sample(m, x, y, r=0.20):
    res = m.info.resolution; w = m.info.width; h = m.info.height
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    mx, my = int((x - ox) / res), int((y - oy) / res)
    if not (0 <= mx < w and 0 <= my < h):
        return 'outside'
    c = m.data[my * w + mx]
    n = int(r / res); mx_ = -2
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if dx * dx + dy * dy > n * n: continue
            u, v = mx + dx, my + dy
            if 0 <= u < w and 0 <= v < h:
                mx_ = max(mx_, m.data[v * w + u])
    return 'centre=%d max(r=%.2f)=%d' % (c, r, mx_)

def main():
    pts = []
    for a in sys.argv[1:]:
        p = a.split(',')
        pts.append((float(p[0]), float(p[1]), p[2] if len(p) > 2 else ''))
    rclpy.init(); n = N()
    t = time.monotonic()
    while time.monotonic() - t < 8 and (len(n.g) < 4 or n.pose is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.pose is not None:
        p = n.pose.pose.pose; c = n.pose.pose.covariance
        print('AMCL pose: (%.2f, %.2f, %.0f deg)  cov xx=%.3f yy=%.3f aa=%.3f' % (p.position.x, p.position.y, q2yaw(p.orientation), c[0], c[7], c[35]))
        pts.insert(0, (p.position.x, p.position.y, 'ROBOT(amcl)'))
    else:
        print('no /amcl_pose received')
    for name in ('map', 'keepout', 'global', 'local'):
        m = n.g.get(name)
        if m is None:
            print('%-8s: NOT RECEIVED' % name); continue
        print('%-8s: %dx%d res %.3f origin (%.2f, %.2f) frame %s' % (name, m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.header.frame_id))
        for x, y, lab in pts:
            print('   %-12s (%.2f, %.2f): %s' % (lab, x, y, sample(m, x, y)))
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
