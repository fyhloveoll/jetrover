#!/usr/bin/env python3
# encoding: utf-8
# Multi-color cube grasp -- validate the vision-grasp loop WITHOUT YOLO.
# Detect a colored cube by HSV segmentation, then reuse the proven pipeline:
# footprint floor-plane localization (+forward nudge) -> arm-base 3D -> IK -> grasp.
#   python3 jr_grasp_color.py survey         # list every colored cube on the floor
#   python3 jr_grasp_color.py dry            # detect JR_COLOR + IK, save debug, NO motion
#   python3 jr_grasp_color.py grab           # execute grasp of JR_COLOR
#   python3 jr_grasp_color.py place          # set whatever is gripped onto the floor
# env: JR_COLOR(red|orange|yellow|green|cyan|blue|purple) JR_FWD JR_GRASP_H JR_DUR JR_PITCH
import sys
import time
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DUR = float(os.environ.get('JR_DUR', '2.0'))
PITCH = float(os.environ.get('JR_PITCH', '80'))
COLOR = os.environ.get('JR_COLOR', 'red')
FWD = float(os.environ.get('JR_FWD', '0.02'))
MIN_AREA = float(os.environ.get('JR_MIN_AREA', '120'))
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600
# HSV ranges (OpenCV H 0-180). red wraps so it has two.
COLORS = {
    'red':    [((0, 100, 60), (10, 255, 255)), ((160, 100, 60), (180, 255, 255))],
    'orange': [((10, 120, 80), (22, 255, 255))],
    'yellow': [((22, 90, 90), (35, 255, 255))],
    'green':  [((36, 60, 45), (85, 255, 255))],
    'cyan':   [((86, 70, 60), (100, 255, 255))],
    'blue':   [((101, 90, 50), (130, 255, 255))],
    'purple': [((131, 60, 50), (159, 255, 255))],
}


def depth_pixel_to_camera(u, v, z, fx, fy, cx, cy):
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])


def quat_to_mat(t, qwxyz):
    w, x, y, z = qwxyz
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def color_mask(hsv, color):
    m = None
    for lo, hi in COLORS[color]:
        part = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = part if m is None else (m | part)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


class Grasp(Node):
    def __init__(self):
        super().__init__('jr_grasp_color')
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._info, 1)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.ik.wait_for_service(timeout_sec=5.0)
        self.fk.wait_for_service(timeout_sec=5.0)
        self.cmd = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.odom = None
        self.create_subscription(Odometry, '/odom_raw', self._odom, 1)

    def _odom(self, m):
        self.odom = m.pose.pose.position

    def _rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def _depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def _info(self, m):
        self.K = list(m.k)

    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def wait_bridge(self, t=8.0):
        t0 = time.time()
        while self.joints.get_subscription_count() < 1 and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        n = self.joints.get_subscription_count()
        self.get_logger().info('servo bridge subscribers = %d' % n)
        return n

    def servos(self, dur, positions):
        msg = ServosPosition()
        msg.duration = float(dur)
        msg.position_unit = 'pulse'
        msg.position = [ServoPosition(id=i, position=float(p)) for i, p in positions]
        self.joints.publish(msg)
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_data(self, t=8.0):
        t0 = time.time()
        while (self.rgb is None or self.depth is None or self.K is None) and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rgb is not None and self.depth is not None and self.K is not None

    def blob(self, color):
        # largest blob of `color`: returns (u, v, area, box) or None
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        mask = color_mask(hsv, color)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return (x + w // 2, y + h // 2, area, (x, y, x + w, y + h))

    def survey(self):
        found = []
        for col in COLORS:
            b = self.blob(col)
            if b:
                u, v, area, box = b
                dist = self.median_depth_m(u, v)
                found.append((col, u, v, area, dist, box))
        return found

    def median_depth_m(self, u, v, win=15):
        d = self.depth
        h, w = d.shape
        patch = d[max(0, v - win):min(h, v + win + 1),
                  max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = patch[(patch > 0) & (patch < 10000)]
        if vals.size < 10:
            return 0.0
        return float(np.percentile(vals, 25)) / 1000.0

    def fit_floor(self, box):
        d = self.depth
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
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
        best, bestn = None, 0
        rng = np.random.default_rng(0)
        for _ in range(200):
            s = P[rng.choice(P.shape[0], 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            ln = np.linalg.norm(n)
            if ln < 1e-6:
                continue
            n = n / ln
            dd = -n @ s[0]
            cnt = int((np.abs(P @ n + dd) < 0.01).sum())
            if cnt > bestn:
                bestn, best = cnt, (n, dd)
        if best is None:
            return None
        n, dd = best
        if n[1] > 0:
            n, dd = -n, -dd
        return n, dd

    def footprint_cam(self, box):
        fl = self.fit_floor(box)
        if fl is None:
            return None
        n, dd = fl
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        x1, y1, x2, y2 = box
        ub, vb = (x1 + x2) // 2, y2
        dirb = np.array([(ub - cx) / fx, (vb - cy) / fy, 1.0])
        denom = n @ dirb
        if abs(denom) < 1e-6:
            return None
        foot = dirb * (-dd / denom)
        view = foot / np.linalg.norm(foot)
        horiz = view - (view @ n) * n
        hn = np.linalg.norm(horiz)
        if hn > 1e-6:
            radius = 0.4 * (x2 - x1) * foot[2] / fx
            foot = foot + radius * (horiz / hn)
        return foot

    def get_endpoint(self):
        r = self._call(self.fk, GetRobotPose.Request())
        p = r.pose.position
        o = r.pose.orientation
        return quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

    def cam_to_armbase(self, cam_xyz, endpoint):
        c = np.array([cam_xyz[0] - 0.01, cam_xyz[1], cam_xyz[2], 1.0])
        return (endpoint @ HAND2CAM @ c)[:3]

    def solve_ik(self, pos, pitch):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in pos]
        req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]
        req.resolution = 1.0
        return self._call(self.ik, req)

    def move(self, dur, positions):
        self.servos(dur * DUR, positions)
        time.sleep(dur * DUR + 0.2)

    def drive_forward(self, dist, speed=0.07):
        # closed-loop straight drive using odom euclidean displacement; dist metres
        if dist <= 0:
            return True
        dist = min(dist, 0.5)  # safety cap per call
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            print('drive: no odom'); return False
        x0, y0 = self.odom.x, self.odom.y
        tw = Twist(); tw.linear.x = float(speed)
        moved = 0.0
        t0 = time.time()
        while moved < dist and time.time() - t0 < 25:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            moved = ((self.odom.x - x0) ** 2 + (self.odom.y - y0) ** 2) ** 0.5
        self.cmd.publish(Twist())  # stop
        for _ in range(6):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        print('drove %.3fm (target %.3f)' % (moved, dist))
        return True

    def approach(self, color, x_reach=0.30, x_target=0.24, max_iter=4):
        # detect target; if beyond arm reach, drive the base forward until in reach
        for i in range(max_iter):
            b = self.blob(color)
            if b is None:
                print('approach: %s not seen' % color); return None
            u, v, area, box = b
            dist = self.median_depth_m(u, v)
            if dist <= 0:
                print('approach: %s depth invalid' % color); return None
            pos, endpoint = self.compute_grasp(box, u, v, dist)
            print('  iter%d %s x=%.3f y=%.3f (reach<=%.2f)' % (i, color, pos[0], pos[1], x_reach))
            if pos[0] <= x_reach:
                return (u, v, area, box, dist, pos, endpoint)
            d = pos[0] - x_target
            print('  -> drive forward %.3fm to bring into reach' % d)
            self.drive_forward(d)
            time.sleep(0.6)  # settle, let camera/odom refresh
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.05)
        # last detection (may still be marginal)
        b = self.blob(color)
        if b is None:
            return None
        u, v, area, box = b
        dist = self.median_depth_m(u, v)
        if dist <= 0:
            return None
        pos, endpoint = self.compute_grasp(box, u, v, dist)
        return (u, v, area, box, dist, pos, endpoint)

    def compute_grasp(self, box, u, v, dist):
        endpoint = self.get_endpoint()
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        old = self.cam_to_armbase(depth_pixel_to_camera(u, v, dist + 0.02, fx, fy, cx, cy)[:3], endpoint)
        foot = self.footprint_cam(box)
        if foot is None:
            pos = list(old)
        else:
            base = self.cam_to_armbase(foot, endpoint)
            z = float(base[2]) + float(os.environ['JR_GRASP_H']) if 'JR_GRASP_H' in os.environ else float(old[2])
            pos = [float(base[0]), float(base[1]), z]
        pos[0] += FWD
        return pos, endpoint

    def pick(self, pos, pitch):
        if self.wait_bridge() < 1:
            self.get_logger().warn('servo bridge not connected; abort')
            return False
        r = self.solve_ik(pos, pitch)
        if not (r and r.pulse):
            self.get_logger().warn('no IK solution for approach; abort')
            return False
        p = r.pulse
        self.get_logger().info('approach pulses %s (DUR=%.1fx)' % (list(p), DUR))
        self.move(0.6, ((10, GRIPPER_OPEN),))
        self.move(1.0, ((1, p[0]),))
        self.move(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        self.move(0.8, ((10, GRIPPER_CLOSE),))
        lift = [pos[0], pos[1], pos[2] + 0.05]
        r2 = self.solve_ik(lift, pitch)
        if r2 and r2.pulse:
            q = r2.pulse
            self.move(1.2, ((1, q[0]), (2, q[1]), (3, q[2]), (4, q[3]), (5, q[4])))
        self.get_logger().info('lifted; holding then returning to observe (still gripping)')
        time.sleep(1.5)
        self.move(1.5, OBSERVE[:5] + ((10, GRIPPER_CLOSE),))
        return True

    def save_debug(self, box, u, v, name='red_detect.png'):
        vis = self.rgb.copy()
        x1, y1, x2, y2 = box
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(vis, (u, v), 4, (255, 0, 0), -1)
        cv2.imwrite('/home/ubuntu/jetrover_ws/' + name, vis)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'dry'
    rclpy.init()
    node = Grasp()

    if mode == 'place':
        node.wait_bridge()
        px = float(os.environ.get('JR_PLACE_X', '0.22'))
        for z in (0.14, 0.07):
            r = node.solve_ik([px, 0.0, z], PITCH)
            if r and r.pulse:
                p = r.pulse
                node.move(1.2, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
            else:
                print('place IK no solution at z=%.2f' % z)
        node.move(0.6, ((10, GRIPPER_OPEN),))
        node.move(1.5, OBSERVE)
        print('placed; returned to observe')
        node.destroy_node(); rclpy.shutdown(); return

    if not node.wait_data():
        print('ERROR: no camera/depth/info'); return

    if mode == 'survey':
        found = node.survey()
        vis = node.rgb.copy()
        if not found:
            print('survey: no colored cubes found')
        for col, u, v, area, dist, box in found:
            print('  %-7s center=(%d,%d) area=%.0f depth=%.3fm box=%s' % (col, u, v, area, dist, box))
            cv2.rectangle(vis, box[:2], box[2:], (0, 255, 0), 2)
            cv2.putText(vis, col, (box[0], box[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite('/home/ubuntu/jetrover_ws/survey.png', vis)
        print('wrote survey.png (%d cubes)' % len(found))
        node.destroy_node(); rclpy.shutdown(); return

    if mode == 'fetch':
        # mobile grasp: drive the base forward until the target is in reach, then grasp
        node.wait_bridge()
        res = node.approach(COLOR)
        if res is None:
            print('fetch: could not localize %s' % COLOR)
            node.destroy_node(); rclpy.shutdown(); return
        u, v, area, box, dist, pos, endpoint = res
        node.save_debug(box, u, v, '%s_detect.png' % COLOR)
        print('FETCH %s GRASP xyz=%s' % (COLOR, np.round(pos, 3).tolist()))
        r = node.solve_ik(pos, PITCH)
        if r and r.pulse:
            print('IK OK pulses=%s rpy=%s' % (list(r.pulse), [round(x, 1) for x in r.rpy]))
            print('=== EXECUTING GRASP (%s) ===' % COLOR)
            node.pick(list(pos), PITCH)
        else:
            print('fetch: no IK solution after approach (x=%.3f)' % pos[0])
        node.destroy_node(); rclpy.shutdown(); return

    b = node.blob(COLOR)
    if b is None:
        print('no %s cube found' % COLOR); node.destroy_node(); rclpy.shutdown(); return
    u, v, area, box = b
    dist = node.median_depth_m(u, v)
    print('%s target: area=%.0f center=(%d,%d) box=%s near_depth=%.3fm' % (COLOR, area, u, v, box, dist))
    node.save_debug(box, u, v, '%s_detect.png' % COLOR)
    if dist <= 0:
        print('center depth invalid'); node.destroy_node(); rclpy.shutdown(); return

    pos, endpoint = node.compute_grasp(box, u, v, dist)
    print('endpoint xyz=%s  GRASP xyz=%s pitch=%.0f FWD=%.3f' %
          (np.round(endpoint[:3, 3], 3).tolist(), np.round(pos, 3).tolist(), PITCH, FWD))
    r = node.solve_ik(pos, PITCH)
    if r and r.pulse:
        print('IK OK: pulses=%s rpy=%s' % (list(r.pulse), [round(x, 1) for x in r.rpy]))
    else:
        print('IK: NO SOLUTION (success=%s)' % (getattr(r, 'success', None)))
    if mode == 'grab':
        if r and r.pulse:
            print('=== EXECUTING GRASP (%s) ===' % COLOR)
            node.pick(list(pos), PITCH)
        else:
            print('no IK solution -> not executing')
    else:
        print('(dry run -- no motion)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
