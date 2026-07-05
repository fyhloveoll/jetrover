#!/usr/bin/env python3
# encoding: utf-8
# M4.5 calibration capture (run ON robot, arm already at FLOOR pose, NO motion here).
# Detects a colored cube and computes its arm-base (x,y,z) two ways:
#   RAW      = box-center pixel + p25 depth -> camera -> arm-base   (pure transform)
#   FOOTPRINT= floor-plane intersection at box bottom (+radius)     (our grasp method)
# Prints both and appends to ~/jetrover_ws/calib_log.csv with a label, so you can
# pair them with tape-measured (forward, lateral) and fit the error offline.
#   JR_COLOR=red python3 calib_measure.py p1
import sys
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from kinematics_msgs.srv import GetRobotPose

COLOR = os.environ.get('JR_COLOR', 'red')
LOG = '/home/ubuntu/jetrover_ws/calib_log.csv'
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
COLORS = {
    'red':    [((0, 100, 60), (10, 255, 255)), ((160, 100, 60), (180, 255, 255))],
    'orange': [((10, 120, 80), (22, 255, 255))],
    'yellow': [((22, 90, 90), (35, 255, 255))],
    'green':  [((36, 60, 45), (85, 255, 255))],
    'cyan':   [((86, 70, 60), (100, 255, 255))],
    'blue':   [((101, 90, 50), (130, 255, 255))],
    'purple': [((131, 60, 50), (159, 255, 255))],
}


def ray(u, v, z, fx, fy, cx, cy):
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])


def quat_to_mat(t, q):
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
    return M


class C(Node):
    def __init__(self):
        super().__init__('calib_measure')
        self.bridge = CvBridge()
        self.rgb = self.depth = self.K = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._r, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._d, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._i, 1)
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.fk.wait_for_service(timeout_sec=5.0)

    def _r(self, m): self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
    def _d(self, m): self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')
    def _i(self, m): self.K = list(m.k)

    def wait(self, t=8.0):
        import time
        t0 = time.time()
        while (self.rgb is None or self.depth is None or self.K is None) and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rgb is not None and self.depth is not None and self.K is not None

    def blob(self):
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        m = None
        for lo, hi in COLORS[COLOR]:
            part = cv2.inRange(hsv, np.array(lo), np.array(hi))
            m = part if m is None else (m | part)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 120:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return (x + w // 2, y + h // 2, (x, y, x + w, y + h))

    def depth_m(self, u, v, win=15):
        d = self.depth; h, w = d.shape
        p = d[max(0, v - win):min(h, v + win + 1), max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = p[(p > 0) & (p < 10000)]
        return float(np.percentile(vals, 25)) / 1000.0 if vals.size >= 10 else 0.0

    def fit_floor(self, box):
        d = self.depth; fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        x1, y1, x2, y2 = box
        ys, xs = np.where((d > 0) & (d < 10000))
        inbox = (xs >= x1) & (xs <= x2) & (ys >= y1) & (ys <= y2)
        xs, ys = xs[~inbox], ys[~inbox]
        if xs.size < 200:
            return None
        idx = np.random.default_rng(0).choice(xs.size, min(4000, xs.size), replace=False)
        xs, ys = xs[idx], ys[idx]
        z = d[ys, xs].astype(np.float32) / 1000.0
        P = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)
        best, bestn = None, 0; rng = np.random.default_rng(0)
        for _ in range(200):
            s = P[rng.choice(P.shape[0], 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0]); ln = np.linalg.norm(n)
            if ln < 1e-6:
                continue
            n = n / ln; dd = -n @ s[0]
            cnt = int((np.abs(P @ n + dd) < 0.01).sum())
            if cnt > bestn:
                bestn, best = cnt, (n, dd)
        if best is None:
            return None
        n, dd = best
        if n[1] > 0:
            n, dd = -n, -dd
        return n, dd

    def footprint(self, box):
        fl = self.fit_floor(box)
        if fl is None:
            return None
        n, dd = fl; fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        x1, y1, x2, y2 = box
        dirb = np.array([((x1 + x2) // 2 - cx) / fx, (y2 - cy) / fy, 1.0])
        den = n @ dirb
        if abs(den) < 1e-6:
            return None
        foot = dirb * (-dd / den)
        view = foot / np.linalg.norm(foot)
        horiz = view - (view @ n) * n; hn = np.linalg.norm(horiz)
        if hn > 1e-6:
            foot = foot + (0.4 * (x2 - x1) * foot[2] / fx) * (horiz / hn)
        return foot

    def endpoint(self):
        fut = self.fk.call_async(GetRobotPose.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result(); p, o = r.pose.position, r.pose.orientation
        return quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

    def to_arm(self, cam, ep):
        c = np.array([cam[0] - 0.01, cam[1], cam[2], 1.0])
        return (ep @ HAND2CAM @ c)[:3]


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'p?'
    rclpy.init()
    node = C()
    if not node.wait():
        print('no camera'); return
    b = node.blob()
    if b is None:
        print('no %s cube seen' % COLOR); return
    u, v, box = b
    dist = node.depth_m(u, v)
    if dist <= 0:
        print('invalid depth'); return
    ep = node.endpoint()
    fx, fy, cx, cy = node.K[0], node.K[4], node.K[2], node.K[5]
    raw = node.to_arm(ray(u, v, dist, fx, fy, cx, cy)[:3], ep)
    fc = node.footprint(box)
    foot = node.to_arm(fc, ep) if fc is not None else np.array([float('nan')] * 3)
    print('label=%s color=%s px=(%d,%d) depth=%.3f' % (label, COLOR, u, v, dist))
    print('  endpoint(arm-base) = %s' % np.round(ep[:3, 3], 4).tolist())
    print('  RAW       (box-center) arm-base x,y,z = %s' % np.round(raw, 4).tolist())
    print('  FOOTPRINT (floor)      arm-base x,y,z = %s' % np.round(foot, 4).tolist())
    new = not os.path.exists(LOG)
    with open(LOG, 'a') as f:
        if new:
            f.write('label,color,u,v,depth,raw_x,raw_y,raw_z,foot_x,foot_y,foot_z\n')
        f.write('%s,%s,%d,%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' %
                (label, COLOR, u, v, dist, raw[0], raw[1], raw[2], foot[0], foot[1], foot[2]))
    print('  appended to %s' % LOG)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
