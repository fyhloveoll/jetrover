#!/usr/bin/env python3
# encoding: utf-8
# M5 grasp STATE MACHINE for continuous success-rate testing (color backend).
# Runs a preset sequence of target colors continuously: for each ->
#   DETECT -> [APPROACH: drive base if out of reach] -> PLAN(IK) -> GRASP ->
#   LIFT -> VERIFY(vision/depth) -> PLACE(on colored paper) -> HOME
# Retries on VERIFY-fail; aborts if board looks locked (N consecutive fails).
# Prints a success-rate + per-state failure breakdown at the end.
#
#   JR_SEQUENCE=red,green,blue JR_PLACE_COLOR=yellow JR_ROUNDS=2 \
#     python3 jr_grasp_fsm.py run
#   python3 jr_grasp_fsm.py survey            # just list cubes
#
# Design: docs/M5_grasp_design.md. VERIFY uses VISION (servo_states echoes goal,
# so gripper-opening read is unreliable); a held object sits ~0.1m from the cam.
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
FWD = float(os.environ.get('JR_FWD', '0.02'))
GRASP_H = os.environ.get('JR_GRASP_H', '0.008')   # metres above floor footprint
MIN_AREA = float(os.environ.get('JR_MIN_AREA', '120'))
SEQUENCE = [c.strip() for c in os.environ.get('JR_SEQUENCE', 'red').split(',') if c.strip()]
PLACE_COLOR = os.environ.get('JR_PLACE_COLOR', '')   # colored paper to place onto
ROUNDS = int(os.environ.get('JR_ROUNDS', '1'))
RETRIES = int(os.environ.get('JR_RETRIES', '1'))
X_REACH = float(os.environ.get('JR_X_REACH', '0.30'))
X_TARGET = float(os.environ.get('JR_X_TARGET', '0.24'))
HELD_DEPTH = float(os.environ.get('JR_HELD_DEPTH', '0.18'))  # blob nearer than this = in gripper
ABORT_FAILS = 3        # consecutive hard fails -> assume board locked, abort

HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
FLOOR = ((1, 500), (2, 700), (3, 15), (4, 175), (5, 500))   # camera looks at floor
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600
COLORS = {
    'red':    [((0, 100, 60), (10, 255, 255)), ((160, 100, 60), (180, 255, 255))],
    'orange': [((10, 120, 80), (22, 255, 255))],
    'yellow': [((22, 90, 90), (35, 255, 255))],
    'green':  [((36, 60, 45), (85, 255, 255))],
    'cyan':   [((86, 70, 60), (100, 255, 255))],
    'blue':   [((101, 90, 50), (130, 255, 255))],
    'purple': [((131, 60, 50), (159, 255, 255))],
    'white':  [((0, 0, 180), (180, 40, 255))],   # place paper: low saturation + high value
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


def color_mask(hsv, color):
    m = None
    for lo, hi in COLORS[color]:
        part = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = part if m is None else (m | part)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


class FSM(Node):
    def __init__(self):
        super().__init__('jr_grasp_fsm')
        self.bridge = CvBridge()
        self.rgb = self.depth = self.K = self.odom = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._info, 1)
        self.create_subscription(Odometry, '/odom_raw', self._odom, 1)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmd = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.ik.wait_for_service(timeout_sec=5.0)
        self.fk.wait_for_service(timeout_sec=5.0)

    def _rgb(self, m): self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
    def _depth(self, m): self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')
    def _info(self, m): self.K = list(m.k)
    def _odom(self, m): self.odom = m.pose.pose.position

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh(self, t=2.0):
        # force a fresh rgb+depth snapshot
        self.rgb = self.depth = None
        t0 = time.time()
        while (self.rgb is None or self.depth is None or self.K is None) and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rgb is not None and self.depth is not None and self.K is not None

    def _call(self, c, req):
        fut = c.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def wait_bridge(self, t=8.0):
        t0 = time.time()
        while self.joints.get_subscription_count() < 1 and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.joints.get_subscription_count()

    def servos(self, dur, positions):
        msg = ServosPosition(); msg.duration = float(dur); msg.position_unit = 'pulse'
        msg.position = [ServoPosition(id=i, position=float(p)) for i, p in positions]
        self.joints.publish(msg)
        self.spin(3)

    def move(self, dur, positions):
        self.servos(dur * DUR, positions)
        time.sleep(dur * DUR + 0.2)

    # ---------- perception ----------
    def blob(self, color):
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        cnts, _ = cv2.findContours(color_mask(hsv, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return (x + w // 2, y + h // 2, area, (x, y, x + w, y + h))

    def blob_angle(self, color):
        # orientation of the largest color blob, normalized to [-45,45) (a square cube
        # repeats every 90 deg, so we align the gripper to the nearest face pair).
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        cnts, _ = cv2.findContours(color_mask(hsv, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0.0
        c = max(cnts, key=cv2.contourArea)
        if len(c) < 5:
            return 0.0
        (_, _), (w, h), ang = cv2.minAreaRect(c)   # ang convention varies by cv2 version
        if w < h:
            ang += 90.0
        ang = ((ang + 45.0) % 90.0) - 45.0
        return float(ang)

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

    def footprint_cam(self, box):
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

    def get_endpoint(self):
        r = self._call(self.fk, GetRobotPose.Request())
        p, o = r.pose.position, r.pose.orientation
        return quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

    def cam_to_arm(self, cam, endpoint):
        c = np.array([cam[0] - 0.01, cam[1], cam[2], 1.0])
        return (endpoint @ HAND2CAM @ c)[:3]

    def grasp_pose(self, color):
        # Geometry-based, auto-height: footprint(floor center) gives XY; the cube's
        # TOP surface (depth at box centre) vs the floor gives the object HEIGHT, so
        # the grasp Z auto-adapts to cubes of different heights.
        # returns (pos, box, u, v, dist, h) or None.
        b = self.blob(color)
        if b is None:
            return None
        u, v, area, box = b
        dist = self.depth_m(u, v)
        if dist <= 0:
            return None
        endpoint = self.get_endpoint()
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        top = self.cam_to_arm(ray(u, v, dist, fx, fy, cx, cy)[:3], endpoint)  # cube top-surface pt
        yoff = float(os.environ.get('JR_Y_OFFSET', '0'))       # calib y-bias correction (computed reads ~+2cm left)
        foot = self.footprint_cam(box)
        if foot is None:
            return ([float(top[0]) + FWD, float(top[1]) - yoff, float(top[2])], box, u, v, dist, 0.0)
        base = self.cam_to_arm(foot, endpoint)
        h = max(0.0, float(top[2]) - float(base[2]))            # object height above floor
        frac = float(os.environ.get('JR_GRASP_FRAC', '0.3'))   # grasp this fraction up the body
        z = float(base[2]) + h * frac
        return ([float(base[0]) + FWD, float(base[1]) - yoff, z], box, u, v, dist, h)

    def solve_ik(self, pos, pitch):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in pos]; req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]; req.resolution = 1.0
        return self._call(self.ik, req)

    # ---------- base motion ----------
    def drive_forward(self, dist, speed=0.07):
        if dist <= 0:
            return
        dist = min(dist, 0.5)
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            print('  drive: no odom'); return
        x0, y0 = self.odom.x, self.odom.y
        tw = Twist(); tw.linear.x = float(speed); moved = 0.0; t0 = time.time()
        while moved < dist and time.time() - t0 < 25:
            self.cmd.publish(tw); rclpy.spin_once(self, timeout_sec=0.05)
            moved = ((self.odom.x - x0) ** 2 + (self.odom.y - y0) ** 2) ** 0.5
        for _ in range(6):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        print('  drove %.3fm' % moved)

    # ---------- one grasp cycle (the state machine) ----------
    def grasp_cycle(self, color):
        # DETECT (+ APPROACH drive if out of reach)
        for it in range(4):
            if not self.fresh():
                return ('FAIL', 'DETECT', 'no camera')
            gp = self.grasp_pose(color)
            if gp is None:
                return ('NO_TARGET', 'DETECT', '%s not seen' % color)
            pos, box, u, v, dist, h = gp
            if pos[0] <= X_REACH:
                break
            print('  [APPROACH] %s x=%.3f > %.2f -> drive' % (color, pos[0], X_REACH))
            self.drive_forward(pos[0] - X_TARGET)
            time.sleep(0.5); self.spin(8)
        # PLAN: print target (even if IK fails), then try top-down first and relax
        # pitch for far targets (a far cube is reachable with a flatter approach).
        print('  [PLAN] %s height=%.3fm target=%s' % (color, h, np.round(pos, 3).tolist()))
        r = None
        used_pitch = PITCH
        for pit in (PITCH, 65.0, 50.0, 35.0):
            rr = self.solve_ik(pos, pit)
            if rr and rr.pulse:
                r = rr
                used_pitch = pit
                break
        if not (r and r.pulse):
            return ('OUT_OF_REACH', 'PLAN', 'no IK x=%.3f (tried pitch 80..35)' % pos[0])
        p = r.pulse
        print('         IK ok pitch=%.0f pulses=%s' % (used_pitch, list(p)))
        # GRASP
        self.move(0.6, ((10, GRIPPER_OPEN),))
        self.move(1.0, ((1, p[0]),))
        self.move(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        time.sleep(0.4)  # settle at grasp pose before closing
        # close the gripper while RE-ASSERTING the arm joints, so the arm holds its
        # pose (does not drift/rise) during the grip
        self.move(1.0, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4]), (10, GRIPPER_CLOSE)))
        time.sleep(0.4)  # let the grip seat before lifting
        # LIFT
        r2 = self.solve_ik([pos[0], pos[1], pos[2] + 0.06], used_pitch)
        if r2 and r2.pulse:
            q = r2.pulse
            self.move(1.2, ((1, q[0]), (2, q[1]), (3, q[2]), (4, q[3]), (5, q[4])))
        self.move(1.2, OBSERVE + ((10, GRIPPER_CLOSE),))
        # VERIFY (vision). A HELD cube sits ~0.1m from the cam -> appears LOW in the
        # frame and large, and its depth often reads invalid(0) because it is closer
        # than the sensor min range. A MISSED cube stays on the floor (~0.3m, mid-frame).
        self.fresh()
        b = self.blob(color)
        H = self.rgb.shape[0]
        if b is None:
            held = True   # no longer on the floor -> grabbed
            print('  [VERIFY] %s gone from view -> held' % color)
        else:
            u2, v2, area2, box2 = b
            vd = self.depth_m(u2, v2)
            # A HELD cube sits in the gripper -> it appears LOW in the frame. Its depth
            # is unreliable (the small cube lets the window read the floor BEHIND it, so
            # depth can read ~0.4 even when held) -> judge by POSITION, not depth.
            held = (v2 > 0.6 * H) or (0 < vd < HELD_DEPTH)
            print('  [VERIFY] %s depth=%.3f v=%d/%d -> held=%s' % (color, vd, v2, H, held))
        return ('SUCCESS' if held else 'GRASP_MISS', 'VERIFY', 'held=%s' % held)

    def place_on(self, color):
        # move to the colored paper and release; falls back to fixed pose if no paper
        self.fresh()
        b = self.blob(color) if color in COLORS else None
        pos = None
        if b is not None:
            u, v, area, box = b
            foot = self.footprint_cam(box)
            if foot is not None:
                base = self.cam_to_arm(foot, self.get_endpoint())
                pos = [float(base[0]) + FWD, float(base[1]), float(base[2]) + 0.05]
        if pos is None:
            pos = [0.22, 0.0, 0.07]
        for z in (pos[2] + 0.07, pos[2]):
            r = self.solve_ik([pos[0], pos[1], z], PITCH)
            if r and r.pulse:
                q = r.pulse
                self.move(1.2, ((1, q[0]), (2, q[1]), (3, q[2]), (4, q[3]), (5, q[4])))
        self.move(0.6, ((10, GRIPPER_OPEN),))

    def home(self):
        self.move(1.2, FLOOR + ((10, GRIPPER_OPEN),))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'run'
    rclpy.init()
    node = FSM()
    if node.wait_bridge() < 1:
        print('servo bridge not connected; abort'); return
    node.fresh()

    if mode == 'survey':
        for col in COLORS:
            b = node.blob(col)
            if b:
                u, v, area, box = b
                print('  %-7s (%d,%d) area=%.0f depth=%.3f' % (col, u, v, area, node.depth_m(u, v)))
        node.destroy_node(); rclpy.shutdown(); return

    # continuous run
    node.home()
    stats = {'attempt': 0, 'success': 0}
    fails = {}
    consec = 0
    log = []
    print('=== FSM run: sequence=%s rounds=%d place=%s ===' % (SEQUENCE, ROUNDS, PLACE_COLOR or 'fixed'))
    for rnd in range(ROUNDS):
        for color in SEQUENCE:
            stats['attempt'] += 1
            outcome = ('FAIL', 'INIT', '')
            for attempt in range(RETRIES + 1):
                print('-- round %d %s (try %d) --' % (rnd, color, attempt))
                outcome = node.grasp_cycle(color)
                if outcome[0] == 'SUCCESS':
                    break
                if outcome[0] == 'NO_TARGET':
                    break
                node.home()  # reset view for retry
            state, where, msg = outcome
            log.append((rnd, color, state, where, msg))
            if state == 'SUCCESS':
                stats['success'] += 1; consec = 0
                if PLACE_COLOR:
                    node.place_on(PLACE_COLOR)
                else:
                    node.place_on('')
            else:
                fails[where] = fails.get(where, 0) + 1
                consec += 1
            node.home()
            if consec >= ABORT_FAILS:
                print('!! %d consecutive fails -> board may be locked. ABORT, power-cycle.' % consec)
                break
        else:
            continue
        break

    print('\n===== RESULT =====')
    print('attempts=%d  success=%d  rate=%.0f%%' %
          (stats['attempt'], stats['success'],
           100.0 * stats['success'] / max(1, stats['attempt'])))
    if fails:
        print('failures by state: %s' % fails)
    for rnd, color, state, where, msg in log:
        print('  r%d %-7s %-12s @%-8s %s' % (rnd, color, state, where, msg))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
