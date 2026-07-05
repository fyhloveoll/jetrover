#!/usr/bin/env python3
# encoding: utf-8
# Object-agnostic GRASP-ALL state machine. Uses jr_detect_objects (depth floor
# segmentation -> any object, ID + label) and clears the floor: each loop detect
# all objects (excluding ones already on the white place-paper), pick the nearest
# reachable one, grasp it by LOCATION with YAW-ALIGNMENT (wrist servo5 turned to the
# object's orientation so the fingers close on OPPOSITE faces, not adjacent) and
# AUTO-HEIGHT, verify, place on the paper, repeat until the floor is clear.
#   python3 jr_grasp_all.py run
#   python3 jr_grasp_all.py survey
# Yaw mapping (servo5 = NEUTRAL + GAIN*angle) NEEDS on-robot calibration: set
# JR_YAW_GAIN sign/magnitude from 1-2 observed grasps (start ~4.17 pulse/deg).
import sys
import time
import os
import math
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
import jr_detect_objects as det     # object-agnostic detection layer (shared)

DUR = float(os.environ.get('JR_DUR', '2.0'))
PITCH = float(os.environ.get('JR_PITCH', '80'))
FWD = float(os.environ.get('JR_FWD', '0.015'))   # forward nudge: 0.02 biased far-edge, 0.01 biased near-edge (batch2/3 A-B test) -> midpoint
Y_OFFSET = float(os.environ.get('JR_Y_OFFSET', '0.022'))
GRASP_FRAC = float(os.environ.get('JR_GRASP_FRAC', '0.35'))   # grip THIS frac up the body;
# lowered from 0.45: the fingertips sit ABOVE the IK point, so 0.45 gripped near the top
# (unstable). ~0.35 lands the actual grip nearer the object's mid-height. Tunable.
YAW_NEUTRAL = float(os.environ.get('JR_YAW_NEUTRAL', '504'))
YAW_GAIN = float(os.environ.get('JR_YAW_GAIN', '4.17'))   # pulse/deg; sign NEEDS calibration
ABORT_FAILS = 3   # reachability is decided purely by the multi-pitch IK solver (no x thresholds)
AUTO_DRIVE = os.environ.get('JR_AUTO_DRIVE', '1') != '0'  # M6: drive the base to fetch out-of-reach / too-close targets
MAX_DRIVES = int(os.environ.get('JR_MAX_DRIVES', '5'))    # total base moves per run (fetch/strafe/reverse)
STRAFE_SIGN = float(os.environ.get('JR_STRAFE_SIGN', '1'))  # +1: Twist.linear.y>0 moves toward arm +y (left); flip if reversed on-robot
MAX_W = float(os.environ.get('JR_MAX_WIDTH', '0.048'))   # gripper max opening (m); wider = ungrippable (a 5.3cm coke can does not fit)
STATS = os.path.expanduser('~/jetrover_ws/grasp_stats.csv')   # per-attempt log, accumulates ACROSS runs/reboots
ANGLE_FRAMES = int(os.environ.get('JR_ANGLE_FRAMES', '6'))   # frames to average the angle over
EDGE_PX = float(os.environ.get('JR_EDGE_PX', '190'))      # |u-cx| beyond this = near frame edge
CENTER_PX = float(os.environ.get('JR_CENTER_PX', '110'))  # |u-cx| within this = centred enough
PAN_GAIN = float(os.environ.get('JR_PAN_GAIN', '4.17'))   # base servo1 pulse/deg (sign NEEDS calib)

HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
FLOOR = ((1, 500), (2, 700), (3, 15), (4, 175), (5, 500))
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600
OPEN_WIDE = int(os.environ.get('JR_OPEN_WIDE', '130'))    # extra-wide open (lower=wider, tested to 100)
WIDE_W = float(os.environ.get('JR_WIDE_W', '0.036'))      # objects wider than this get the wide open


def ray(u, v, z, fx, fy, cx, cy):
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])


def quat_to_mat(t, q):
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
    return M


class GraspAll(Node):
    def __init__(self):
        super().__init__('jr_grasp_all')
        self.bridge = CvBridge()
        self.rgb = self.depth = self.K = self.odom = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._d, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._i, 1)
        self.create_subscription(Odometry, '/odom_raw', self._o, 1)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmd = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.ik.wait_for_service(timeout_sec=5.0)
        self.fk.wait_for_service(timeout_sec=5.0)
        self.place_n = 0   # placements so far -> cycles fallback drop offsets
        # paper contours CACHED from the first (empty-paper) sighting per camera pose:
        # once cubes cover the paper its white blob shrinks below the detection floor,
        # live detection returns None, and on-paper exclusion would silently die --
        # batch2 then re-picked a cube FROM the place zone. Poses repeat exactly, so
        # first-sight pixel contours stay valid for the whole run.
        self.pc_floor = None    # FLOOR-pose paper contour (pickable/on-paper tests)
        self.pc_obs = None      # OBSERVE-pose paper contour (drop-cell selection)
        self.n_bottom = 0       # bottom-edge rejects in the last pickable() scan (M6: reverse to see them)

    def _rgb(self, m): self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
    def _d(self, m): self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')
    def _i(self, m): self.K = list(m.k)
    def _o(self, m): self.odom = m.pose.pose.position

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh(self, t=2.5):
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
        self.joints.publish(msg); self.spin(3)

    def move(self, dur, positions):
        self.servos(dur * DUR, positions); time.sleep(dur * DUR + 0.2)

    def home(self):
        self.move(1.2, FLOOR + ((10, GRIPPER_OPEN),))

    # ---- perception ----
    def detect(self):
        return det.detect(self.rgb, np.asarray(self.depth), self.K)

    @staticmethod
    def circ_mean_angle(angles):
        # circular mean of 90-deg-periodic angles in [-45,45]: map theta->4*theta to use
        # the full circle, average unit vectors, map back -- handles the +-45 wrap.
        if not angles:
            return 0.0
        s = sum(math.sin(math.radians(4 * a)) for a in angles)
        c = sum(math.cos(math.radians(4 * a)) for a in angles)
        return math.degrees(math.atan2(s, c)) / 4.0

    def floor_angle_arm(self, inst, ep):
        # TRUE cube orientation: backproject the blob contour onto the floor plane and
        # measure the angle in the ARM frame. The raw image angle is perspective-
        # distorted by the tilted camera (a 40deg floor angle reads ~29deg in-image),
        # so yaw error GROWS with cube angle -- small cubes forgive it, the 43mm cube
        # (2.5mm entry tolerance) does not.
        cnt = inst.get('cnt')
        fl = self.fit_floor(None)
        if cnt is None or fl is None or len(cnt) < 8:
            return None
        n, dd = fl
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        T = ep @ HAND2CAM
        pts = []
        step = max(1, len(cnt) // 48)
        for p in cnt[::step]:
            u, v = float(p[0][0]), float(p[0][1])
            dirb = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            den = n @ dirb
            if abs(den) < 1e-6:
                continue
            c = dirb * (-dd / den)
            a = T @ np.array([c[0] - 0.01, c[1], c[2], 1.0])
            pts.append([a[0], a[1]])
        if len(pts) < 8:
            return None
        (_, _), (rw, rh), ang = cv2.minAreaRect(np.array(pts, np.float32))
        if rw < rh:
            ang += 90.0
        return ((ang + 45.0) % 90.0) - 45.0

    def stable_yaw(self, u, v, base_pulse, n=ANGLE_FRAMES, rad=40):
        # multi-frame circular-mean of the floor-projected arm-frame angle, then subtract
        # the base yaw the grasp will use: the gripper's closing axis rotates with the
        # base (servo1), so the wrist offset must be RELATIVE to the approach bearing.
        # JR_YAW_MODE=image falls back to the old raw image angle (no projection).
        # default=image: the floor-projected mode's base-yaw (gamma) subtraction used a
        # wrong servo1 pulse->deg model and mangled otherwise-correct angles (user saw
        # the wrist mis-rotate); raw image angles empirically nailed +-40deg cubes (68%).
        mode = os.environ.get('JR_YAW_MODE', 'image')
        ep = self.get_endpoint()
        angs = []
        for _ in range(n):
            self.fresh()
            best, bd = None, float(rad * rad)
            for o in self.detect():
                d = (o['u'] - u) ** 2 + (o['v'] - v) ** 2
                if d < bd:
                    bd, best = d, o
            if best is None:
                continue
            a = self.floor_angle_arm(best, ep) if mode == 'floor' else best['angle']
            if a is not None:
                angs.append(a)
        mean = self.circ_mean_angle(angs)
        if mode == 'floor':
            gamma = (float(base_pulse) - 500.0) / 4.17    # base yaw the arm will take, deg
            off = ((mean - gamma + 45.0) % 90.0) - 45.0
        else:
            off = mean
        return off, len(angs)

    def recenter_if_edge(self, target):
        # objects near the frame edge localize + angle poorly. Rotate the base (servo1)
        # to bring the target toward the image centre, re-detect, return the now-central
        # detection. grasp() reads fresh FK so the transform handles the rotated pose.
        cx, fx = self.K[2], self.K[0]
        if abs(target['u'] - cx) <= EDGE_PX:
            return target
        print('  [RECENTER] %s at u=%d (edge) -> rotating base to centre' % (target['id'], target['u']))
        s1 = 500
        cur = target
        for it in range(4):
            if abs(cur['u'] - cx) <= CENTER_PX:
                break
            ang = math.degrees(math.atan2(cur['u'] - cx, fx))
            s1 = int(max(60, min(940, s1 - PAN_GAIN * ang)))      # PAN_GAIN sign tunable on-robot
            self.move(1.0, ((1, s1),) + FLOOR[1:])
            self.fresh()
            cands = [o for o in self.detect() if o['label'] == target['label']]
            if not cands:
                print('  [RECENTER] lost %s after rotating' % target['label'])
                return None
            cur = min(cands, key=lambda o: abs(o['u'] - cx))
            print('  [RECENTER] iter%d servo1=%d -> u=%d' % (it, s1, cur['u']))
        return cur

    def paper_contour(self):
        # the white place paper's actual CONTOUR (not a loose bbox -- a bbox wrongly
        # swallows floor cubes beside the card). Used for point-in-polygon exclusion.
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        wm = cv2.inRange(hsv, np.array([0, 0, 205]), np.array([180, 30, 255]))
        wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        cnts, _ = cv2.findContours(wm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        return c if cv2.contourArea(c) >= 4000 else None

    @staticmethod
    def on_paper(pc, u, v):
        return pc is not None and cv2.pointPolygonTest(pc, (float(u), float(v)), False) >= 0

    def depth_m(self, u, v, win=15):
        d = self.depth; h, w = d.shape
        p = d[max(0, v - win):min(h, v + win + 1), max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = p[(p > 0) & (p < 10000)]
        return float(np.percentile(vals, 25)) / 1000.0 if vals.size >= 10 else 0.0

    def fit_floor(self, box):
        z = np.asarray(self.depth).astype(np.float32) / 1000.0
        return det.fit_floor(z, self.K[0], self.K[4], self.K[2], self.K[5])

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

    def cam_to_arm(self, cam, ep):
        c = np.array([cam[0] - 0.01, cam[1], cam[2], 1.0])
        return (ep @ HAND2CAM @ c)[:3]

    def solve_ik_multi(self, pos):
        for pit in (PITCH, 65.0, 50.0, 35.0, 22.0, 10.0):   # flatter approaches reach farther
            req = SetRobotPose.Request()
            req.position = [float(v) for v in pos]; req.pitch = float(pit)
            req.pitch_range = [-180.0, 180.0]; req.resolution = 1.0
            r = self._call(self.ik, req)
            if r and r.pulse:
                return list(r.pulse), pit
        return None, None

    def grasp_pos(self, inst):
        # footprint(floor center) for XY + auto-height Z, from the object's box
        u, v, box = inst['u'], inst['v'], inst['box']
        # bottom-truncated object: box bottom = image edge, NOT the real base -> the
        # footprint ray hits the floor far too close (batch1 green1: pos x=0.10, h=0.09
        # for a 3cm cube). Localization is unusable -> reject instead of grasping air.
        if box[3] >= self.rgb.shape[0] - 3:
            print('  reject %s: cut off at bottom edge, footprint unreliable' % inst['id'])
            return None, 0.0
        dist = self.depth_m(u, v)
        if dist <= 0:
            return None, 0.0
        ep = self.get_endpoint()
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        top = self.cam_to_arm(ray(u, v, dist, fx, fy, cx, cy)[:3], ep)
        foot = self.footprint_cam(box)
        if foot is None:
            return [float(top[0]) + FWD, float(top[1]) - Y_OFFSET, float(top[2])], 0.0
        base = self.cam_to_arm(foot, ep)
        h = max(0.0, float(top[2]) - float(base[2]))
        if h > float(os.environ.get('JR_MAX_H', '0.055')):  # 0.055 also catches two stacked 3cm cubes (h~=0.06, garbage angle)
            # no cube here is >7cm tall; an absurd h means the footprint is wrong
            # (partial view / merged blob) -> reject rather than grasp at a bad point
            print('  reject %s: implausible height %.3fm -> bad localization' % (inst['id'], h))
            return None, 0.0
        z = float(base[2]) + h * GRASP_FRAC
        return [float(base[0]) + FWD, float(base[1]) - Y_OFFSET, z], h

    # ---- base motion (M6 fetch: drive to bring far/too-close targets into reach) ----
    def drive(self, dist, axis='x', speed=0.07):
        # odom-closed-loop drive; axis 'x' = forward/backward, 'y' = mecanum strafe
        # (dist>0 toward arm +y = left). Returns the SIGNED distance actually moved.
        if abs(dist) < 0.01:
            return 0.0
        dist = max(-0.45, min(0.45, dist))
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            print('  [DRIVE] no odom, not driving'); return 0.0
        x0, y0 = self.odom.x, self.odom.y
        v = float(speed if dist > 0 else -speed)
        tw = Twist()
        if axis == 'y':
            tw.linear.y = v * STRAFE_SIGN
        else:
            tw.linear.x = v
        moved = 0.0; t0 = time.time()
        while moved < abs(dist) and time.time() - t0 < 20:
            self.cmd.publish(tw); rclpy.spin_once(self, timeout_sec=0.05)
            moved = ((self.odom.x - x0) ** 2 + (self.odom.y - y0) ** 2) ** 0.5
        for _ in range(6):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        print('  [DRIVE %s] moved %+.3fm (target %+.3f)' % (axis, moved if dist > 0 else -moved, dist))
        return moved if dist > 0 else -moved

    def survey_far(self, placed_pts):
        # "look up and scan": the FLOOR pose only sees ~0.15-0.45m; from OBSERVE the
        # camera sees much farther. Return (x, y) of the nearest far floor object
        # worth driving to, or None. Called when the floor view has nothing pickable.
        self.move(1.2, OBSERVE + ((10, GRIPPER_OPEN),))
        self.fresh(); self.fresh()
        pc = self.paper_contour()
        ep = self.get_endpoint()
        best = None
        for d in self.detect():
            if self.on_paper(pc, d['u'], d['v']):
                continue
            if d['box'][3] >= self.rgb.shape[0] - 3:
                continue
            if self.depth_m(d['u'], d['v']) <= 0:
                continue
            foot = self.footprint_cam(d['box'])
            if foot is None:
                continue
            base = self.cam_to_arm(foot, ep)
            x, y = float(base[0]), float(base[1])
            if x < 0.38 or x > 1.2 or abs(y) > 0.45:
                continue    # near ones are the floor view's job; too far/lateral = skip
            if any(abs(x - px) < 0.08 and abs(y - py) < 0.08 for px, py, _pz in placed_pts):
                continue    # something we placed
            print('  [SURVEY] far object %s at (%.2f, %+.2f)' % (d['id'], x, y))
            if best is None or x < best[0]:
                best = (x, y)
        return best

    def settle_after_drive(self):
        # the scene shifted: flush stale frames (the old approach() bug re-detected on a
        # pre-drive frame) and drop pixel-space caches that are no longer valid
        time.sleep(0.6)
        self.fresh(); self.fresh()
        self.pc_floor = None
        self.pc_obs = None

    # ---- one grasp (location + yaw-aligned + auto-height) ----
    def grasp(self, inst):
        pos, h = self.grasp_pos(inst)
        if pos is None:
            return ('FAIL', 'no depth')
        print('  target %s pos=%s h=%.3f angle=%+.0f' % (inst['id'], np.round(pos, 3).tolist(), h, inst['angle']))
        # reachability is decided by the IK SOLVER (multi-pitch), not a hardcoded
        # x threshold -- a far cube is still reachable with a flatter approach.
        p, pit = self.solve_ik_multi(pos)
        if p is None:
            return ('OUT_OF_REACH', 'no IK x=%.3f (tried pitch 80..10)' % pos[0])
        # YAW ALIGN: turn the wrist (servo5) to the object's orientation so the fingers
        # close on OPPOSITE faces. Use a MULTI-FRAME stabilized angle (single-frame
        # minAreaRect jitters). (servo5 mapping needs JR_YAW_GAIN sign/gain calibration.)
        sa, ns = self.stable_yaw(inst['u'], inst['v'], p[0])
        s5 = int(max(0, min(1000, YAW_NEUTRAL + YAW_GAIN * sa)))
        p[4] = s5
        # wide objects (e.g. the 43mm cube in a ~48mm gripper) have only ~2.5mm entry
        # tolerance at the normal opening -- open EXTRA wide for them to double the margin
        g_open = OPEN_WIDE if inst.get('width_m', 0.0) > WIDE_W else GRIPPER_OPEN
        print('  IK pitch=%.0f stable_angle=%+.0f(n=%d raw=%+.0f) servo5=%d open=%d' %
              (pit, sa, ns, inst['angle'], s5, g_open))
        self.move(0.6, ((10, g_open),))
        self.move(1.0, ((1, p[0]),))
        self.move(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        time.sleep(0.4)
        self.move(1.0, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4]), (10, GRIPPER_CLOSE)))
        time.sleep(0.4)
        # lift (keep yaw). NO verify here -- the main loop verifies AFTER placing, by
        # re-checking the pickup spot with an empty gripper (robust).
        lp, _ = self.solve_ik_multi([pos[0], pos[1], pos[2] + 0.06])
        if lp:
            lp[4] = s5
            self.move(1.2, ((1, lp[0]), (2, lp[1]), (3, lp[2]), (4, lp[3]), (5, lp[4])))
        # go to OBSERVE (gripper closed) so place_on_paper sees the paper from the known
        # view -- straight from the lift pose the camera may not have the paper in frame.
        self.move(1.2, OBSERVE + ((10, GRIPPER_CLOSE),))
        return ('LIFTED', 's5=%d' % s5)

    def spot_occupied(self, u, v, label=None, rad=35):
        # is an object still on the floor near (u,v)? (call at FLOOR pose, empty gripper)
        # label-matched + tight radius: a NEIGHBOURING cube inside a loose radius made a
        # real grasp read as MISS (batch5 red1 was physically grasped, logged MISS).
        self.fresh()
        for d in self.detect():
            if label is not None and d['label'] != label:
                continue
            if abs(d['u'] - u) < rad and abs(d['v'] - v) < rad:
                return True
        return False

    def floor_point_cam(self, u, v):
        # ray through pixel (u,v) intersected with the RANSAC floor plane
        fl = self.fit_floor(None)
        if fl is None:
            return None
        n, dd = fl
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        dirb = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        den = n @ dirb
        return dirb * (-dd / den) if abs(den) > 1e-6 else None

    def pick_drop_cell(self, pc):
        # choose an EMPTY spot on the paper: 3x3 grid inside the contour, skip cells
        # near already-placed cubes -- so placements SPREAD instead of piling up (a
        # pile shrinks the visible white area until paper detection fails entirely).
        objs = self.detect()
        x, y, w, h = cv2.boundingRect(pc)
        best = None
        for gy in (0.32, 0.5, 0.68):
            for gx in (0.32, 0.5, 0.68):
                cu, cv_ = int(x + gx * w), int(y + gy * h)
                if cv2.pointPolygonTest(pc, (float(cu), float(cv_)), True) < 14:
                    continue                     # outside / too near the paper edge
                dmin = min((max(abs(o['u'] - cu), abs(o['v'] - cv_)) for o in objs), default=999)
                if dmin < 42:
                    continue                     # occupied by a placed cube
                score = dmin - 60 * (abs(gx - 0.5) + abs(gy - 0.5))   # free-ness, centre bias
                if best is None or score > best[0]:
                    best = (score, cu, cv_)
        return (best[1], best[2]) if best else None

    def place_on_paper(self):
        self.fresh()
        pc_live = self.paper_contour()
        if pc_live is not None:
            self.pc_obs = pc_live      # live-first (tracks a nudged paper), cache as fallback
        pc = pc_live if pc_live is not None else self.pc_obs
        pos = None
        if pc is not None:
            cell = self.pick_drop_cell(pc)
            if cell is None:
                # paper known but no free cell -> still drop ON the paper (centroid +
                # cycling offset); stacking on paper beats dropping on the open floor
                m = cv2.moments(pc)
                if m['m00'] > 0:
                    cell = (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
            if cell is not None:
                fp = self.floor_point_cam(*cell)
                if fp is not None:
                    base = self.cam_to_arm(fp, self.get_endpoint())
                    # sanity-gate the RAW paper point (before the spread offset -- the
                    # offset once pushed a legit point past the gate): a false-white
                    # region gave an absurd [0.86,-0.54] once
                    if float(base[0]) + FWD > 0.35 or abs(float(base[1])) > 0.31:
                        print('  [PLACE] absurd paper point %s -> ignoring paper detection'
                              % np.round(base[:2], 3).tolist())
                    else:
                        off = (0.0, 0.03, -0.03, 0.05, -0.05)[self.place_n % 5]
                        pos = [float(base[0]) + FWD, float(base[1]) + off, float(base[2]) + 0.05]
                        print('  [PLACE] paper cell px=%s off=%+.2f -> %s'
                              % (list(cell), off, np.round(pos, 3).tolist()))
        if pos is None:
            # no idea where the paper is at all: fixed front spot, cycling offset
            off = (0.0, 0.05, -0.05, 0.09, -0.09)[self.place_n % 5]
            pos = [0.22, off, 0.07]
            print('  [PLACE] fallback FLOOR spot y=%+.2f (paper never seen!)' % off)
        lp, _ = self.solve_ik_multi([pos[0], pos[1], pos[2] + 0.07])
        if lp:
            self.move(1.2, ((1, lp[0]), (2, lp[1]), (3, lp[2]), (4, lp[3]), (5, lp[4])))
        lp2, _ = self.solve_ik_multi(pos)
        if lp2:
            self.move(1.0, ((1, lp2[0]), (2, lp2[1]), (3, lp2[2]), (4, lp2[3]), (5, lp2[4])))
        drop = pos
        if not (lp or lp2):
            # never release mid-air over nowhere: retreat to the known-reachable fixed
            # front spot and drop there instead
            off = (0.0, 0.05, -0.05, 0.09, -0.09)[self.place_n % 5]
            print('  [PLACE] WARNING: no IK for %s -> using fixed front spot y=%+.2f'
                  % (np.round(pos, 3).tolist(), off))
            drop = [0.22, off, 0.07]
            lp3, _ = self.solve_ik_multi(drop)
            if lp3:
                self.move(1.2, ((1, lp3[0]), (2, lp3[1]), (3, lp3[2]), (4, lp3[3]), (5, lp3[4])))
        self.move(0.6, ((10, GRIPPER_OPEN),))
        self.place_n += 1
        return drop   # arm-frame drop point (for the placed-cube memory)


def log_stat(inst, result, msg):
    # append one attempt to the stats CSV; accumulates across runs/power-cycles so the
    # 15-30-sample success-rate target survives the board's lockup-reboot cycle.
    try:
        new = not os.path.exists(STATS)
        with open(STATS, 'a') as f:
            if new:
                f.write('ts,id,label,u,v,angle,width_mm,result,msg\n')
            f.write('%.0f,%s,%s,%d,%d,%.0f,%.0f,%s,%s\n' % (
                time.time(), inst['id'], inst['label'], inst['u'], inst['v'],
                inst['angle'], inst.get('width_m', 0.0) * 1000, result,
                str(msg).replace(',', ';')))
    except Exception:
        pass


def stats_summary():
    try:
        rows = [l.split(',') for l in open(STATS).read().strip().split('\n')[1:]]
        n = len(rows)
        s = sum(1 for r in rows if len(r) > 7 and r[7] == 'SUCCESS')
        return 'CUMULATIVE (all runs): %d/%d = %.0f%% success' % (s, n, 100.0 * s / max(1, n))
    except Exception:
        return 'CUMULATIVE: no stats yet'


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'run'
    rclpy.init()
    node = GraspAll()
    if node.wait_bridge() < 1:
        print('servo bridge not connected'); return
    if not node.fresh():
        print('no camera'); return

    if mode == 'survey':
        node.home(); node.fresh()
        pc = node.paper_contour()
        for d in node.detect():
            onp = node.on_paper(pc, d['u'], d['v'])
            print('  %-8s (%d,%d) angle=%+.0f depth=%.3f%s' %
                  (d['id'], d['u'], d['v'], d['angle'], d['depth'], '  [on paper]' if onp else ''))
        node.destroy_node(); rclpy.shutdown(); return

    # GRASP-ALL: clear the floor. blacklist = (u,v) of failed/unreachable targets so
    # we never re-pick them (no infinite loop); verify by re-checking the pickup spot
    # with an EMPTY gripper after placing (robust, unlike in-gripper guessing).
    node.home()
    grabbed, attempts, consec = 0, 0, 0
    blacklist = []
    placed_pts = []   # arm-frame (x,y) of every drop -- drive-proof "we placed that" memory
    log = []

    def pickable():
        node.fresh()
        node.n_bottom = 0
        pc_live = node.paper_contour()
        if pc_live is not None:
            node.pc_floor = pc_live    # live-first (tracks a nudged paper), cache as fallback
        pc = pc_live if pc_live is not None else node.pc_floor
        out = []
        for d in node.detect():
            if d['box'][3] >= node.rgb.shape[0] - 3:
                # too close (cut off at the bottom edge): count it EVEN IF blacklisted --
                # these are invisible to path_clear and MUST be resolved by reversing
                # first, else a fetch drive rolls right over them (it happened).
                node.n_bottom += 1
                continue
            if node.on_paper(pc, d['u'], d['v']):
                continue                                   # on the place paper
            if any(abs(d['u'] - bu) < 45 and abs(d['v'] - bv) < 45 for bu, bv in blacklist):
                continue                                   # known failed/unreachable
            if MAX_W > 0 and d.get('width_m', 0.0) > MAX_W:
                print('  skip %s: width %.0fmm > gripper %.0fmm' %
                      (d['id'], d['width_m'] * 1000, MAX_W * 1000))
                continue                                   # physically ungrippable
            pos, h = node.grasp_pos(d)
            if pos is None:
                continue                                   # no valid depth / bad localization
            if (abs(fetch_off) < 0.02 and abs(fetch_lat) < 0.02
                    and any(abs(pos[0] - px) < 0.06 and abs(pos[1] - py) < 0.06
                            for px, py, _pz in placed_pts)):
                continue                                   # a cube WE placed (arm-frame memory, survives drives/paper-loss)
            out.append((pos, d))                           # reachability decided by IK in grasp()
        out.sort(key=lambda t: t[0][0])                    # nearest first
        return out

    drives = 0          # base moves used (fetch/strafe/reverse), capped
    fetch_off = 0.0     # net forward offset from fetch drives; undone before placing
    fetch_lat = 0.0     # net lateral (strafe) offset; undone before placing

    def path_clear(cands, tpos):
        # nothing else standing in the strip we would drive through; the chassis is
        # ~0.2m wide plus wheels, so anything within |y|<0.14 is in harm's way
        for pp, dd in cands[1:]:
            if pp[0] < tpos[0] - 0.04 and abs(pp[1]) < 0.14:
                return False
        return True

    def strafe_clear(cands, tpos):
        # nothing in the sideways corridor we would strafe through (x out to ~0.42
        # covers the chassis footprint while translating laterally)
        lo, hi = (0.06, tpos[1] + 0.10) if tpos[1] > 0 else (tpos[1] - 0.10, -0.06)
        for pp, dd in cands[1:]:
            if pp[0] < 0.42 and lo < pp[1] < hi:
                return False
        return True

    while rclpy.ok() and attempts < 12:
        cand = pickable()
        # CLOSEST-FIRST: anything cut off at the bottom edge sits right at the wheels --
        # reverse to resolve it BEFORE any grasp or fetch drive (fetch drives crushed one).
        # EXCEPT mid-fetch (fetch_off>0): cubes only look "too close" because WE drove
        # forward past them; reversing would just undo the fetch (it did, and burned the
        # drive budget). Grasp the fetched target first, the return trip restores them.
        if (AUTO_DRIVE and node.n_bottom > 0 and drives < MAX_DRIVES
                and fetch_off < 0.02 and abs(fetch_lat) < 0.02):
            print('[M6] %d target(s) too close (bottom-cut) -> reversing 0.13m first' % node.n_bottom)
            node.drive(-0.13); node.settle_after_drive()
            drives += 1; fetch_off -= 0.13; blacklist.clear()
            continue
        if not cand:
            # nothing in the floor view: LOOK UP AND SCAN from OBSERVE for far objects
            far = None
            if AUTO_DRIVE and drives < MAX_DRIVES:
                far = node.survey_far(placed_pts)
            if far is not None:
                fx_, fy_ = far
                print('[M6] SURVEY target at (%.2f, %+.2f) -> going' % (fx_, fy_))
                if abs(fy_) >= 0.12 and drives + 1 < MAX_DRIVES:
                    node.drive(fy_, axis='y'); drives += 1; fetch_lat += fy_
                d = fx_ - 0.30
                node.drive(d); node.settle_after_drive()
                drives += 1; fetch_off += d; blacklist.clear()
                node.home(); continue
            node.home()
            print('floor clear (nothing pickable left).'); break
        # orig = detection in the HOME view (servo1=500). recenter_if_edge may rotate the
        # base and return a detection in the ROTATED view -- those pixel coords are only
        # valid while rotated. blacklist and the post-home spot check are evaluated in the
        # HOME view, so they must use orig's coords, never the recentered ones.
        orig = cand[0][1]
        pos0 = cand[0][0]
        attempts += 1
        print('-- attempt %d: %s @(%d,%d) angle=%+.0f (%d pickable) --' %
              (attempts, orig['id'], orig['u'], orig['v'], orig['angle'], len(cand)))
        target = node.recenter_if_edge(orig)       # edge -> rotate base to centre + re-detect
        if target is None:
            blacklist.append((orig['u'], orig['v'])); consec += 1
            log_stat(orig, 'RECENTER_LOST', ''); node.home(); continue
        try:
            st, msg = node.grasp(target)
        except Exception as e:
            print('  grasp error: %s' % e); break
        if st != 'LIFTED':
            if st == 'OUT_OF_REACH' and AUTO_DRIVE and drives < MAX_DRIVES:
                if abs(pos0[1]) >= 0.12 and strafe_clear(cand, pos0):
                    # M6 lateral fetch: mecanum-strafe to line the target up first
                    print('[M6] %s lateral (y=%+.2f) -> strafing to align' % (orig['id'], pos0[1]))
                    node.drive(pos0[1], axis='y'); node.settle_after_drive()
                    drives += 1; fetch_lat += pos0[1]; blacklist.clear()
                    log_stat(orig, 'DRIVE_STRAFE', 'y=%.2f' % pos0[1])
                    node.home(); continue
                if abs(pos0[1]) < 0.12 and path_clear(cand, pos0):
                    # M6 fetch: target straight ahead but beyond the arm -> drive to it
                    d = pos0[0] - 0.24
                    print('[M6] %s out of reach (x=%.2f) -> driving %+.2fm to fetch' %
                          (orig['id'], pos0[0], d))
                    node.drive(d); node.settle_after_drive()
                    drives += 1; fetch_off += d; blacklist.clear()
                    log_stat(orig, 'DRIVE_FETCH', 'x=%.2f' % pos0[0])
                    node.home(); continue
            blacklist.append((orig['u'], orig['v'])); consec += 1
            log.append((orig['id'], st, msg)); log_stat(orig, st, msg); node.home(); continue
        if fetch_off != 0.0 or fetch_lat != 0.0:
            # carry the cube back to the start point so the paper is where we left it
            print('[M6] returning (x %+.2f, y %+.2f) before placing' % (-fetch_off, -fetch_lat))
            if abs(fetch_lat) >= 0.01:
                node.drive(-fetch_lat, axis='y')
            if abs(fetch_off) >= 0.01:
                node.drive(-fetch_off)
            node.settle_after_drive()
            fetch_off = 0.0; fetch_lat = 0.0; blacklist.clear()
        ppos = node.place_on_paper()
        node.home()
        still = node.spot_occupied(orig['u'], orig['v'], orig['label'])   # empty-gripper recheck (home view)
        if not still:
            grabbed += 1; consec = 0; log.append((orig['id'], 'SUCCESS', msg))
            placed_pts.append(ppos)
            log_stat(orig, 'SUCCESS', msg)
            print('  -> SUCCESS (pickup spot now empty)')
        else:
            blacklist.append((orig['u'], orig['v'])); consec += 1
            log.append((orig['id'], 'MISS', 'spot still occupied'))
            log_stat(orig, 'MISS', 'spot still occupied')
            print('  -> MISS (still there) -> blacklisted')
        if consec >= ABORT_FAILS:
            print('!! %d consecutive fails -> board may be locked / all hard. ABORT.' % consec); break

    print('\n===== GRASP-ALL DONE =====  grabbed %d / %d attempts (blacklisted %d)' %
          (grabbed, attempts, len(blacklist)))
    for i, (cid, st, m) in enumerate(log):
        print('  %d %-8s %-12s %s' % (i, cid, st, m))
    print(stats_summary())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
