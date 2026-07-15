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
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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
MAX_DRIVES = int(os.environ.get('JR_MAX_DRIVES', '12'))   # total base moves per run (fetch/strafe/reverse)
STRAFE_SIGN = float(os.environ.get('JR_STRAFE_SIGN', '1'))
SCAN_YAW = float(os.environ.get('JR_SCAN_YAW', '0'))        # lidar mounting yaw offset (rad); probe on-robot
CLEAR_MARGIN = float(os.environ.get('JR_CLEAR_MARGIN', '0.28'))  # keep this much lidar clearance beyond the leg  # +1: Twist.linear.y>0 moves toward arm +y (left); flip if reversed on-robot
MAX_W = float(os.environ.get('JR_MAX_WIDTH', '0.048'))   # gripper max opening (m); wider = ungrippable (a 5.3cm coke can does not fit)
HARD_MAX_W = float(os.environ.get('JR_HARD_MAX_W', '0.09'))  # absolute ceiling at ANY distance (furniture filter)
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
        self.scan = None
        self.scan_t = 0.0
        # /scan lives on a DEDICATED mini-node + executor: on the main node the 10Hz
        # scan starves behind the 30Hz rgb / 22Hz depth flood (subscription-order
        # tricks proved non-deterministic) and the in-motion e-stop went blind on a
        # ~4s-stale scan -- drove into a book twice (07-10). The mini executor only
        # ever serves the scan, so pumping it in the drive loop is starvation-proof.
        self.mon = rclpy.create_node('jr_scan_mon')
        # SensorDataQoS (best-effort): the default RELIABLE sub stalls under motion
        # (reliable-protocol bookkeeping in a busy process starves the stream; a
        # concurrent best-effort subscriber unsticking it was the giveaway, 07-10)
        self.mon.create_subscription(LaserScan, '/scan', self._s, qos_profile_sensor_data)
        self.mon_exec = SingleThreadedExecutor()
        self.mon_exec.add_node(self.mon)
        self.create_subscription(Odometry, '/odom_raw', self._o, 1)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._d, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._i, 1)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.cmd = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.ik.wait_for_service(timeout_sec=5.0)
        self.fk.wait_for_service(timeout_sec=5.0)
        self.place_n = 0   # placements so far -> cycles fallback drop offsets
        self.zone_seen = False   # drop zone detected at least once this run
        self.zone_hull = None    # START-frame metric hull of the mapped drop zone (mm ints)
        self.zone_z = 0.0        # zone floor height (arm frame)
        # paper contours CACHED from the first (empty-paper) sighting per camera pose:
        # once cubes cover the paper its white blob shrinks below the detection floor,
        # live detection returns None, and on-paper exclusion would silently die --
        # batch2 then re-picked a cube FROM the place zone. Poses repeat exactly, so
        # first-sight pixel contours stay valid for the whole run.
        self.pc_floor = None    # FLOOR-pose paper contour (pickable/on-paper tests)
        self.pc_obs = None      # OBSERVE-pose paper contour (drop-cell selection)
        self.n_bottom = 0       # bottom-edge rejects in the last pickable() scan (M6: reverse to see them)
        self.survey_chased = []  # arm-frame survey targets already driven to (one chase each)

    def _rgb(self, m): self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
    def _d(self, m): self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')
    def _i(self, m): self.K = list(m.k)
    def _o(self, m): self.odom = m.pose.pose.position
    def _s(self, m): self.scan = m; self.scan_t = time.time()

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

    def board_alive(self):
        # camera-verified actuation. During the classic lockup EVERY telemetry channel
        # lies (servo_states echoes goals, odom integrates commanded velocity while the
        # wheels stand still) -- the only honest witness is the camera: command a small
        # arm tilt and check the scene actually changed. Run this BEFORE a batch so we
        # never execute a whole phantom run again (ga15 was one).
        if not self.fresh():
            return False
        g0 = cv2.cvtColor(cv2.resize(self.rgb, (160, 90)), cv2.COLOR_BGR2GRAY).astype(np.float32)
        self.move(0.8, ((2, 660),))     # small tilt away from FLOOR's (2,700)
        self.fresh()
        g1 = cv2.cvtColor(cv2.resize(self.rgb, (160, 90)), cv2.COLOR_BGR2GRAY).astype(np.float32)
        self.move(0.8, ((2, 700),))     # restore
        diff = float(np.abs(g1 - g0).mean())
        print('[LIVENESS] scene diff %.2f (board dead if < 2.0)' % diff)
        return diff >= 2.0

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
        angs, longs, elongs = [], [], []
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
                longs.append(best.get('angle_long', a))
                elongs.append(best.get('elong', 1.0))
        # ELONGATED objects (pen, toothbrush): the +-45-collapsed cube angle is 90deg-
        # ambiguous -- invisible on squares, but the gripper closed ALONG a pen's body
        # (user, 07-11). Grip PERPENDICULAR to the long axis (mod-180 circular mean).
        if elongs and sorted(elongs)[len(elongs) // 2] > 1.6:
            s = sum(math.sin(math.radians(2 * a)) for a in longs)
            c = sum(math.cos(math.radians(2 * a)) for a in longs)
            long_mean = math.degrees(math.atan2(s, c)) / 2.0
            grip = (long_mean % 180.0) - 90.0      # perpendicular, wrapped to (-90, 90]
            print('  [YAW] elongated (ratio %.1f): long axis %+.0f -> grip %+.0f' %
                  (sorted(elongs)[len(elongs) // 2], long_mean, grip))
            return grip, len(longs)
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
            prev = abs(cur['u'] - cx)
            cur = min(cands, key=lambda o: abs(o['u'] - cx))
            print('  [RECENTER] iter%d servo1=%d -> u=%d' % (it, s1, cur['u']))
            if abs(cur['u'] - cx) > prev - 40:
                # no progress: rotating just brings OTHER same-color objects to the edge
                # (it chased placed cubes in a loop once, wandering the base 1m) -> abort
                print('  [RECENTER] no progress -> abort')
                return None
        return cur

    def _apriltags(self):
        # AprilTag 36h11 corner cards on the drop zone (version-portable cv2.aruco)
        gray = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2GRAY)
        try:
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            try:
                det_ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
                corners, ids, _ = det_.detectMarkers(gray)
            except AttributeError:      # cv2 4.x legacy API (the robot)
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, d, parameters=cv2.aruco.DetectorParameters_create())
        except Exception:
            return []
        return [c[0] for c in corners] if ids is not None and len(ids) else []

    def paper_contour(self):
        c = self._zone_detect()
        if c is not None:
            self.zone_seen = True
        return c

    def _zone_detect(self, strict=False):
        # DROP-ZONE contour. Default mode 'dark': a DARK mat seeded by AprilTag corner
        # cards -- glare on the glossy floor reads as white, and it hijacked the white-
        # paper detector ("absurd paper point", placements dumped on the floor, 07-11).
        # JR_ZONE=white restores the old white-paper detector.
        if os.environ.get('JR_ZONE', 'dark') == 'white':
            hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
            wm = cv2.inRange(hsv, np.array([0, 0, 205]), np.array([180, 30, 255]))
            wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            cnts, _ = cv2.findContours(wm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None
            c = max(cnts, key=cv2.contourArea)
            return c if cv2.contourArea(c) >= 4000 else None
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        dm = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 90]))   # dark mat
        dm = cv2.morphologyEx(dm, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
        cnts, _ = cv2.findContours(dm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) >= 4000]
        if not cnts:
            return None
        tags = self._apriltags()
        if tags:
            # the zone is the dark region carrying/adjacent to a tag (tags kill the
            # "any dark shadow" ambiguity the way they killed the glare one)
            for c in sorted(cnts, key=cv2.contourArea, reverse=True):
                for t in tags:
                    tc = t.mean(axis=0)
                    if cv2.pointPolygonTest(c, (float(tc[0]), float(tc[1])), True) > -25:
                        return c
            return None
        if strict:
            # mosaic mode: a view with NO tag must contribute NOTHING -- the largest-
            # dark fallback fed far furniture/shadow into the pan mosaic and the hull
            # swallowed 1.0m^2 of floor ("drop at own feet was inside the zone", 07-15)
            return None
        # no tag visible (occluded by cubes): largest dark blob as fallback
        return max(cnts, key=cv2.contourArea)

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

    def footprint_cam(self, box, inst=None):
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
            push = 0.4 * (x2 - x1) * foot[2] / fx   # ~0.4 x body width (cube-tuned)
            if inst is not None and inst.get('elong', 1.0) > 1.6 and inst.get('width_m', 0) > 0:
                # thin diagonal body: its BOX width is the diagonal span (a pen's box is
                # ~10cm -> 4cm push, landing the grasp on a fingertip); push by the
                # body's own radius instead
                push = 0.5 * inst['width_m']
            foot = foot + push * (horiz / hn)
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
        foot = self.footprint_cam(box, inst)
        if foot is None:
            return [float(top[0]) + FWD, float(top[1]) - Y_OFFSET, float(top[2])], 0.0
        base = self.cam_to_arm(foot, ep)
        h = max(0.0, float(top[2]) - float(base[2]))
        if h > float(os.environ.get('JR_MAX_H', '0.055')):  # 0.055 also catches two stacked 3cm cubes (h~=0.06, garbage angle)
            # no cube here is >7cm tall; an absurd h means the footprint is wrong
            # (partial view / merged blob) -> reject rather than grasp at a bad point
            print('  reject %s: implausible height %.3fm -> bad localization' % (inst['id'], h))
            return None, 0.0
        trim = float(os.environ.get('JR_Z_TRIM', '0'))
        if h < 0.015:
            # near-floor thin regime: the fully-extended arm sags a few mm, which cube
            # grasps absorb but a 1cm pen does not (calibrated on-robot 07-11: -0.010)
            trim += float(os.environ.get('JR_Z_TRIM_THIN', '-0.010'))
        z = float(base[2]) + h * GRASP_FRAC + trim
        if inst.get('elong', 1.0) > 1.6:
            # diagonal thin body: the box-bottom ray lands on an EMPTY box corner (the
            # body lies along the box DIAGONAL) -- XY straight from the centroid depth
            # point (valid on a matte 1cm body, error ~mm); footprint keeps only floor z
            pos = [float(top[0]) + FWD, float(top[1]) - Y_OFFSET, z]
        else:
            pos = [float(base[0]) + FWD, float(base[1]) - Y_OFFSET, z]
        if pos[0] < 0.08:
            # behind/under the robot = garbage from an extreme rotated view
            print('  reject %s: insane x=%.2f' % (inst['id'], pos[0]))
            return None, 0.0
        return pos, h

    def clearance(self, direction):
        # min lidar range within +-24deg of the intended motion direction (base frame:
        # 0=forward, +pi/2=left, pi=back). None if the lidar is off. The camera cannot
        # see sideways -- a strafe nearly hit a wall -- so base motion consults the scan.
        self.mon_exec.spin_once(timeout_sec=0.005)  # drain any pending scan (0.0 executes nothing in rclpy)
        m = self.scan
        if m is None:
            return None
        rng = np.asarray(m.ranges, dtype=np.float32)
        ang = m.angle_min + np.arange(rng.size) * m.angle_increment + SCAN_YAW
        d = (ang - direction + np.pi) % (2 * np.pi) - np.pi
        sel = (np.abs(d) <= 0.42) & (rng > m.range_min) & (rng < m.range_max)
        return float(rng[sel].min()) if np.any(sel) else None

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
        # lidar safety: check clearance in the motion direction, truncate the leg if a
        # wall/furniture is close (the camera is blind sideways/backwards)
        direction = (0.0 if dist > 0 else math.pi) if axis != 'y' else \
                    (math.pi / 2 if dist > 0 else -math.pi / 2)
        t_req = time.time()
        while self.scan_t < t_req and time.time() - t_req < 0.4:
            self.mon_exec.spin_once(timeout_sec=0.05)   # insist on a fresh pre-drive scan
        c = self.clearance(direction)
        if c is not None and c < abs(dist) + CLEAR_MARGIN:
            allowed = max(0.0, c - CLEAR_MARGIN)
            print('  [DRIVE %s] obstacle at %.2fm -> leg truncated %.2f -> %.2f' %
                  (axis, c, abs(dist), allowed))
            if allowed < 0.02:
                return 0.0
            dist = allowed if dist > 0 else -allowed
        x0, y0 = self.odom.x, self.odom.y
        v = float(speed if dist > 0 else -speed)
        tw = Twist()
        if axis == 'y':
            tw.linear.y = v * STRAFE_SIGN
        else:
            tw.linear.x = v
        moved = 0.0; t0 = time.time(); warned = False
        while moved < abs(dist) and time.time() - t0 < 20:
            # ~10Hz command stream (denser serial traffic raises the lockup risk).
            # DRAIN callbacks for the whole inter-command window: a single spin_once
            # starves the 10Hz scan behind 30Hz rgb + 22Hz depth + 48Hz odom, so the
            # e-stop was checking a frozen pre-drive scan (drove into a book, 07-10).
            self.cmd.publish(tw)
            end = time.time() + 0.1
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.mon_exec.spin_once(timeout_sec=0.005)  # starvation-proof scan pump
            moved = ((self.odom.x - x0) ** 2 + (self.odom.y - y0) ** 2) ** 0.5
            if os.environ.get('JR_DRIVE_DEBUG') == '1':
                print('  [DBG] t=%4.1f moved=%.3f scan_age=%.2f' %
                      (time.time() - t0, moved, time.time() - self.scan_t))
            if time.time() - self.scan_t < 1.0:
                c = self.clearance(direction)
                if c is not None and c < 0.20:
                    print('  [DRIVE %s] EMERGENCY STOP: obstacle %.2fm' % (axis, c))
                    break
            elif self.scan is not None and not warned:
                print('  [DRIVE %s] warn: scan stale %.1fs, e-stop blind' %
                      (axis, time.time() - self.scan_t)); warned = True
        for _ in range(6):
            self.cmd.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.02)
        print('  [DRIVE %s] moved %+.3fm (target %+.3f)' % (axis, moved if dist > 0 else -moved, dist))
        return moved if dist > 0 else -moved

    def survey_far(self, placed_pts, off=(0.0, 0.0)):
        # off = (fetch_off, fetch_lat): current displacement from run start, so chased
        # targets can be remembered in the START frame (arm frame moves with the robot)
        # "look up and scan": the FLOOR pose only sees ~0.15-0.45m; from OBSERVE the
        # camera sees much farther. Return (x, y) of the nearest far floor object
        # worth driving to, or None. Called when the floor view has nothing pickable.
        self.move(1.2, OBSERVE + ((10, GRIPPER_OPEN),))
        self.fresh(); self.fresh()
        pc = self.paper_contour()
        ep = self.get_endpoint()
        best = None
        targets = [t.strip() for t in os.environ.get('JR_TARGET', '').split(',') if t.strip()]
        for d in self.detect():
            if targets and d['label'] not in targets:
                continue      # never drive toward something we'd refuse to pick
            if d.get('width_m', 0.0) > HARD_MAX_W:
                continue      # furniture-sized: never chase it (the nightstand)
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
            if any(abs(x + off[0] - qx) < 0.08 and abs(y + off[1] - qy) < 0.08
                   for qx, qy in self.survey_chased):
                continue    # ONE chase per target: re-chasing a skip (too wide/unreachable)
                            # looped 3x at a merged-cube cluster and wandered 1.1m (07-11)
            print('  [SURVEY] far object %s at (%.2f, %+.2f)' % (d['id'], x, y))
            if best is None or x < best[0]:
                best = (x, y)
        if best is not None:
            self.survey_chased.append((best[0] + off[0], best[1] + off[1]))
        return best

    def settle_after_drive(self):
        # the scene shifted: flush stale frames (the old approach() bug re-detected on a
        # pre-drive frame) and drop pixel-space caches that are no longer valid
        time.sleep(0.6)
        self.fresh(); self.fresh()
        self.pc_floor = None
        self.pc_obs = None
        det._LBL_MEMO.clear()   # sticky names are pixel-keyed; the drive moved every pixel

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
        # thin bodies (a ~1cm pen): pulse 600 is where fingers merely TOUCH a cube-sized
        # object; on 1cm there is no squeeze left and the body slips out on lift (07-11)
        g_close = int(os.environ.get('JR_CLOSE_THIN', '660')) \
            if 0 < inst.get('width_m', 0.0) < 0.015 else GRIPPER_CLOSE
        self.move(0.6, ((10, g_open),))
        self.move(1.0, ((1, p[0]),))
        self.move(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        time.sleep(0.4)
        self.move(1.0, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4]), (10, g_close)))
        time.sleep(0.4)
        # lift (keep yaw). NO verify here -- the main loop verifies AFTER placing, by
        # re-checking the pickup spot with an empty gripper (robust).
        lp, _ = self.solve_ik_multi([pos[0], pos[1], pos[2] + 0.06])
        if lp:
            lp[4] = s5
            self.move(1.2, ((1, lp[0]), (2, lp[1]), (3, lp[2]), (4, lp[3]), (5, lp[4])))
        # go to OBSERVE (gripper closed) so place_on_paper sees the paper from the known
        # view -- straight from the lift pose the camera may not have the paper in frame.
        self.move(1.2, OBSERVE + ((10, g_close),))
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

    def scan_zone(self, off=(0.0, 0.0)):
        # PAN-MOSAIC zone mapping ("扫视获取放置区全貌"): stop-and-stare at 5 base
        # headings, backproject every zone contour sighting onto the floor in ARM
        # coordinates (live FK per view), union into ONE start-frame metric hull.
        # Pixel contours die the moment the robot moves; metric ones survive drives
        # and show the FULL zone, not just the corner one view happens to expose.
        pts, zs, anchors = [], [], []
        # OBSERVE sees 0.38-1.2m; a mat at the robot's feet is BELOW that view, so the
        # sweep also includes FLOOR-pose (0.15-0.45m) headings. Early-exit once the
        # mosaic is rich enough -- no point staring at the far wall (user, 07-15).
        views = [(OBSERVE, 500), (FLOOR, 500), (OBSERVE, 660), (FLOOR, 660),
                 (OBSERVE, 820), (OBSERVE, 340), (FLOOR, 340), (OBSERVE, 180)]
        got_views = 0
        for pose, s1 in views:
            if got_views >= 2 and len(pts) >= 50 and anchors:
                break                                  # enough coverage already
            self.move(0.7, ((1, s1),) + pose[1:])      # gripper untouched (carry-safe)
            time.sleep(0.2)                            # stop-and-stare: no mid-pan frames
            self.fresh(); self.fresh()
            tags = self._apriltags()
            c = self._zone_detect(strict=True)
            print('  [ZONE] view %s s1=%d: tags=%d contour=%s' %
                  ('OBS' if pose is OBSERVE else 'FLR', s1,
                   len(tags), 'Y' if c is not None else 'N'))
            if not tags and c is None:
                continue
            ep = self.get_endpoint()
            # tag centres are ANCHORS: corner-refined and position-accurate; the mat
            # geometry hangs off them, not off the noisy dark blob
            for t in tags:
                tc = t.mean(axis=0)
                fp = self.floor_point_cam(int(tc[0]), int(tc[1]))
                if fp is None:
                    continue
                a = self.cam_to_arm(fp, ep)
                if 0.05 < a[0] < 1.2 and abs(a[1]) < 0.8:
                    anchors.append((float(a[0]) + off[0], float(a[1]) + off[1]))
                    zs.append(float(a[2]))
            if c is None:
                continue
            got_views += 1
            self.zone_seen = True
            for p in c[::6]:
                fp = self.floor_point_cam(int(p[0][0]), int(p[0][1]))
                if fp is None:
                    continue
                a = self.cam_to_arm(fp, ep)
                if 0.05 < a[0] < 1.2 and abs(a[1]) < 0.8:
                    pts.append((float(a[0]) + off[0], float(a[1]) + off[1]))
                    zs.append(float(a[2]))
        self.move(0.8, ((1, 500),) + OBSERVE[1:])
        if anchors:
            # keep only contour points within 30cm of a tag anchor: oblique-view
            # backprojection noise inflated a ~0.1m^2 mat to 0.23-0.34m^2 (07-15);
            # anchors are ground truth, the dark blob only fills in the extent
            pts = [p for p in pts
                   if min((p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2 for a in anchors) < 0.09]
            pts.extend(anchors)
        if len(pts) >= 8:
            hull = cv2.convexHull((np.array(pts, dtype=np.float32) * 1000).astype(np.int32))
            area = cv2.contourArea(hull) / 1e6
            if area > 0.45:
                # sanity cap vs the 1.0m^2 floor-swallow class; oblique backprojection
                # noise inflates a real ~0.1m^2 mat to ~0.35 (a true map was rejected
                # at 0.34 on the first mission delivery, 07-15)
                print('  [ZONE] REJECT map: %.2fm^2 implausible (%d pts)' % (area, len(pts)))
                return False
            self.zone_hull = hull
            self.zone_z = float(np.median(zs))
            print('  [ZONE] mapped: %d pts, area %.2fm^2, floor z=%.3f' %
                  (len(pts), area, self.zone_z))
            return True
        print('  [ZONE] pan scan found no zone (%d pts)' % len(pts))
        return False

    def zone_cell(self, off=(0.0, 0.0), placed=()):
        # first free ~9cm METRIC cell inside the mapped zone hull, nearest-reachable
        # first. Metric cells spread drops across the real zone (no pixel-corner
        # crowding) and the start-frame hull + off compensation survives drives.
        if self.zone_hull is None:
            return None
        h = self.zone_hull.reshape(-1, 2).astype(np.float32) / 1000.0
        cands = []
        for gx in np.arange(h[:, 0].min() + 0.05, h[:, 0].max(), 0.09):
            for gy in np.arange(h[:, 1].min() + 0.05, h[:, 1].max(), 0.09):
                d_in = cv2.pointPolygonTest(self.zone_hull, (float(gx * 1000), float(gy * 1000)), True)
                if d_in < 30:
                    continue                       # >=3cm inside the hull
                if any(abs(gx - pp[0]) < 0.055 and abs(gy - pp[1]) < 0.055 for pp in placed):
                    continue                       # occupied (start frame both sides)
                cx_, cy_ = gx - off[0], gy - off[1]  # current frame
                if not (0.12 < cx_ < 0.36 and abs(cy_) < 0.34):
                    continue                       # outside the arm envelope from HERE
                cands.append((cx_, cy_, d_in))
        if not cands:
            return None
        # DEEPEST-INSIDE first: nearest-first always chose edge cells and drops kept
        # landing on the mat's rim, nearly sliding off (user, 07-16)
        cands.sort(key=lambda c: -c[2])
        return (cands[0][0], cands[0][1])

    def place_on_paper(self, off=(0.0, 0.0), placed=()):
        # metric zone-map path first: full-extent spread cells, displacement-proof
        self.fresh()
        if self.zone_hull is None:
            self.scan_zone(off)
        if self.zone_hull is None:
            # APPROACH-THEN-CONFIRM: tags are unreadable beyond ~0.55m, so a far mat
            # can never be tag-confirmed from here (chicken-and-egg: the approach
            # logic waited for a confirmed map). If the lenient detector sees a dark
            # blob at a plausible mat distance, drive up and re-scan (07-16).
            c = self._zone_detect()
            if c is not None:
                m = cv2.moments(c)
                if m['m00'] > 0:
                    fp = self.floor_point_cam(int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
                    if fp is not None:
                        a = self.cam_to_arm(fp, self.get_endpoint())
                        if 0.42 < a[0] < 1.0 and abs(a[1]) < 0.45:
                            print('  [PLACE] unconfirmed dark blob at (%.2f, %+.2f) -> approaching to confirm'
                                  % (a[0], a[1]))
                            mx = self.drive(min(0.45, float(a[0]) - 0.30))
                            my = self.drive(max(-0.3, min(0.3, float(a[1]))), axis='y')                                 if abs(a[1]) > 0.18 else 0.0
                            if mx or my:
                                off = (off[0] + mx, off[1] + my)
                                self.settle_after_drive()
                                self.scan_zone(off)
        if self.zone_hull is not None:
            cell = self.zone_cell(off, placed)
            if cell is not None:
                pos = [float(cell[0]), float(cell[1]), self.zone_z + 0.05]
                print('  [PLACE] metric zone cell -> %s' % np.round(pos, 3).tolist())
                return self._lower_and_release(pos, True)
        # legacy pixel path (zone map unavailable): live/cached contour + fallbacks
        pc_live = self.paper_contour()
        if pc_live is not None:
            self.pc_obs = pc_live      # live-first (tracks a nudged paper), cache as fallback
        pc = pc_live if pc_live is not None else self.pc_obs
        if pc is None:
            # PAN-SEARCH: the paper may sit outside the straight view (user: "让机器人
            # 自己扫视"). Sweep the base and look; pixel->arm uses live FK, so a panned
            # detection converts exactly (same principle as recenter_if_edge). The
            # panned contour is used LIVE only -- never cached into the straight view.
            for s1 in (660, 820, 340, 180):
                self.move(1.0, ((1, s1),) + OBSERVE[1:])
                self.fresh()
                pc = self.paper_contour()
                if pc is not None:
                    print('  [PLACE] paper found by pan (servo1=%d)' % s1)
                    break
            if pc is None:
                self.move(1.0, OBSERVE)   # sweep failed: face forward again
        pos = None
        on_zone = False
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
                    if float(base[0]) + FWD > 0.38 or abs(float(base[1])) > 0.36:
                        print('  [PLACE] absurd paper point %s -> ignoring paper detection'
                              % np.round(base[:2], 3).tolist())
                    else:
                        off = (0.0, 0.03, -0.03, 0.05, -0.05)[self.place_n % 5]
                        pos = [float(base[0]) + FWD, float(base[1]) + off, float(base[2]) + 0.05]
                        on_zone = True
                        print('  [PLACE] paper cell px=%s off=%+.2f -> %s'
                              % (list(cell), off, np.round(pos, 3).tolist()))
        if pos is None:
            # no idea where the paper is at all: fixed front spot, cycling offset
            off = (0.0, 0.05, -0.05, 0.09, -0.09)[self.place_n % 5]
            pos = [0.22, off, 0.07]
            print('  [PLACE] fallback FLOOR spot y=%+.2f (paper never seen!)' % off)
        return self._lower_and_release(pos, on_zone)

    def _lower_and_release(self, pos, on_zone):
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
            on_zone = False
            lp3, _ = self.solve_ik_multi(drop)
            if lp3:
                self.move(1.2, ((1, lp3[0]), (2, lp3[1]), (3, lp3[2]), (4, lp3[3]), (5, lp3[4])))
        self.move(0.6, ((10, GRIPPER_OPEN),))
        self.place_n += 1
        # on_zone False = fallback drop on open floor: an OBSTACLE but not "stored" --
        # it should be re-collected once the zone is actually seen (07-11)
        return drop, on_zone


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
    if not node.fresh(t=15.0):
        # generous STARTUP wait only: a fresh process can take several seconds to
        # DDS-discover the camera publishers (worse after many short-lived nodes);
        # the camera itself being down still fails, just slower
        print('no camera'); return

    if mode == 'drivetest':
        # guarded-motion test: drivetest <dist> [x|y] -- exercises the REAL drive()
        # (lidar truncation / refusal / in-motion e-stop) with no grasping
        dist = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
        axis = sys.argv[3] if len(sys.argv) > 3 else 'x'
        if not node.board_alive():
            print('!! BOARD DEAD, not driving'); node.destroy_node(); rclpy.shutdown(); return
        moved = node.drive(dist, axis=axis)
        print('[DRIVETEST] requested %+.2f on %s, moved %+.3f' % (dist, axis, moved))
        node.destroy_node(); rclpy.shutdown(); return

    if mode == 'place':
        # mission delivery (jr_mission): the cube is ALREADY in the gripper (a JR_CARRY
        # run grabbed it, nav drove us here). Find the zone HERE by pan-mosaic and
        # place -- the gripper is never opened before the placement motion.
        if not node.board_alive():      # tilts joint2 only; gripper untouched
            print('!! BOARD DEAD, cannot place'); node.destroy_node(); rclpy.shutdown(); return
        node.move(1.5, ((1, 500),) + OBSERVE[1:])   # observe WITHOUT touching servo 10
        node.fresh()
        node.scan_zone()
        off = (0.0, 0.0)
        if node.zone_hull is not None:
            # MICRO-APPROACH: nav parks within tolerance of the standoff point; if the
            # zone centre ended up beyond the arm (x=0.46 > 0.38 on the first mission
            # delivery), drive the delta and re-scan from the closer, cleaner view
            h = node.zone_hull.reshape(-1, 2).astype(np.float32) / 1000.0
            zx, zy = float(h[:, 0].mean()), float(h[:, 1].mean())
            mx = node.drive(max(-0.3, min(0.45, zx - 0.28))) if zx > 0.34 or zx < 0.20 else 0.0
            my = node.drive(max(-0.3, min(0.3, zy)), axis='y') if abs(zy) > 0.18 else 0.0
            if mx or my:
                off = (mx, my)
                node.settle_after_drive()
                node.scan_zone(off)
        drop, on_zone = node.place_on_paper(off)
        print('[PLACE-MODE] released at %s (on_zone=%s)' % (np.round(drop, 3).tolist(), on_zone))
        node.move(1.2, ((1, 500),) + OBSERVE[1:])
        node.destroy_node(); rclpy.shutdown(); return

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
    if not node.board_alive():
        print('!! BOARD DEAD: arm commands are not executing (telemetry/odom LIE during')
        print('!! this lockup). Power-cycle the board (main switch OFF -> ON), then rerun.')
        node.destroy_node(); rclpy.shutdown(); return

    node.home()
    grabbed, attempts, consec = 0, 0, 0
    blacklist = []
    hard_black = []     # start-frame (x,y) parked as unreachable-without-budget; never cleared
    placed_pts = []   # arm-frame (x,y,z) of on-zone drops -- excluded as targets + obstacles
    floor_drops = []  # fallback drops on open floor -- obstacles only, re-collected once zone seen
    log = []

    def pickable():
        node.fresh()
        node.n_bottom = 0
        pc_live = node.paper_contour()
        if pc_live is not None:
            node.pc_floor = pc_live    # live-first (tracks a nudged paper), cache as fallback
        pc = pc_live if pc_live is not None else node.pc_floor
        targets = [t.strip() for t in os.environ.get('JR_TARGET', '').split(',') if t.strip()]
        out = []
        for d in node.detect():
            if targets and d['label'] not in targets:
                continue                                   # semantic filter (e.g. JR_TARGET=banana)
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
            if d.get('width_m', 0.0) > HARD_MAX_W:
                # ABSOLUTE ceiling, trusted at ANY distance: far-view inflation is
                # ~30% max (43mm read 51mm), never 2x -- a 119mm "object" is furniture
                # (the nightstand), don't even drive toward it (user, 07-15)
                print('  skip %s: width %.0fmm = furniture-sized, never graspable' %
                      (d['id'], d['width_m'] * 1000))
                continue
            if MAX_W > 0 and d.get('width_m', 0.0) > MAX_W and d.get('depth', 1.0) < 0.42:
                # width is only trusted NEAR (far blobs read wide); far candidates get
                # fetched closer first and re-judged with an accurate measurement
                print('  skip %s: width %.0fmm > gripper %.0fmm' %
                      (d['id'], d['width_m'] * 1000, MAX_W * 1000))
                continue                                   # physically ungrippable
            pos, h = node.grasp_pos(d)
            if pos is None:
                continue                                   # no valid depth / bad localization
            if any(abs(pos[0] + fetch_off - px) < 0.06 and abs(pos[1] + fetch_lat - py) < 0.06
                   for px, py, _pz in placed_pts):
                # a cube WE placed -- compare in the START frame (displacement-
                # compensated; the old at-start-only gate let a displaced robot
                # re-pick its own placements and re-place them elsewhere, 07-11)
                continue
            if any(abs(pos[0] + fetch_off - hx) < 0.06 and abs(pos[1] + fetch_lat - hy) < 0.06
                   for hx, hy in hard_black):
                continue                                   # parked as unreachable this run
            if (not node.zone_seen) and any(
                    abs(pos[0] + fetch_off - fx) < 0.06 and abs(pos[1] + fetch_lat - fy) < 0.06
                    for fx, fy, _fz in floor_drops):
                continue    # re-placing would just drop it again until the zone is seen
            out.append((pos, d))                           # reachability decided by IK in grasp()
        cor = os.environ.get('JR_CORRIDOR', '')
        if cor:
            # CLEAR-THE-ROAD ordering (mission mode): cubes sitting in the corridor
            # toward the delivery point get picked FIRST -- every carry trip then
            # rolls through ground we already cleared (lidar cannot see 3cm cubes)
            ux, uy = (float(v) for v in cor.split(','))
            def key(t):
                px, py = t[0][0], t[0][1]
                along = px * ux + py * uy
                cross = abs(px * uy - py * ux)
                in_corridor = along > 0.05 and cross < 0.16
                return (0 if in_corridor else 1, px)
            out.sort(key=key)
        else:
            out.sort(key=lambda t: t[0][0])                # nearest first
        return out

    drives = 0          # base moves used (fetch/strafe/reverse), capped
    fetch_off = 0.0     # net forward offset from fetch drives; undone before placing
    fetch_lat = 0.0     # net lateral (strafe) offset; undone before placing
    legs = []           # (axis, actual_moved) stack of outbound legs

    def drive_leg(dist, axis='x'):
        # outbound drive that RECORDS the actual leg driven, so returns can retrace
        # the exact path LIFO (reverse is lidar-blind; net-offset returns swept
        # sideways through space never driven and hit a chair, 07-11)
        moved = node.drive(dist, axis=axis)
        if abs(moved) > 0.005:
            legs.append((axis, moved))
        return moved

    def placed_in_sweep(ax, s):
        # a cube placed AFTER we drove out sits on the "known-driven" path (crushed a
        # green one, 07-11): clamp the leg before the nearest placed cube in its sweep
        lim = s
        for px, py, _pz in placed_pts + floor_drops:
            ox, oy = px - fetch_off, py - fetch_lat     # current frame
            if ax == 'y':
                if -0.05 < ox < 0.30 and (0 < oy < lim + 0.12 if s > 0 else lim - 0.12 < oy < 0):
                    lim = (oy - 0.14) if s > 0 else (oy + 0.14)
            else:
                if abs(oy) < 0.14 and (0.30 < ox < lim + 0.30 if s > 0 else lim + 0.10 < ox < 0.10):
                    lim = (ox - 0.34) if s > 0 else (ox + 0.14)
        if abs(lim) < abs(s):
            print('  [RETRACE] placed cube in sweep -> leg %.2f clamped to %.2f' % (s, lim))
        return lim if abs(lim) >= 0.01 else 0.0

    def retrace_home():
        # unwind outbound legs in reverse order, then correct any residual.
        # coalesce adjacent same-axis legs first: four alternating strafes replayed
        # one by one looked like nervous left-right shuffling (user, 07-11)
        nonlocal fetch_off, fetch_lat
        packed = []
        for ax, mv in legs:
            if packed and packed[-1][0] == ax:
                packed[-1][1] += mv
            else:
                packed.append([ax, mv])
        legs.clear()
        legs.extend((ax, mv) for ax, mv in packed if abs(mv) >= 0.01)
        while legs:
            ax, mv = legs.pop()
            rem = placed_in_sweep(ax, -mv)
            for _ in range(2):
                if abs(rem) < 0.01:
                    break
                got = node.drive(rem, axis=ax)
                rem -= got
                if ax == 'y':
                    fetch_lat += got
                else:
                    fetch_off += got
        rem = placed_in_sweep('x', -fetch_off)   # residual after truncated unwinds
        for _ in range(2):
            if abs(rem) < 0.01:
                break
            got = node.drive(rem); rem -= got; fetch_off += got
        rem = placed_in_sweep('y', -fetch_lat)
        for _ in range(2):
            if abs(rem) < 0.01:
                break
            got = node.drive(rem, axis='y'); rem -= got; fetch_lat += got

    def path_clear(cands, tpos):
        # nothing else standing in the strip we would drive through; the chassis is
        # ~0.2m wide plus wheels, so anything within |y|<0.14 is in harm's way
        for pp, dd in cands[1:]:
            if pp[0] < tpos[0] - 0.04 and abs(pp[1]) < 0.14:
                return False
        for px, py, _pz in placed_pts + floor_drops:
            # placed cubes are excluded as TARGETS but still exist as OBSTACLES --
            # excluding them from candidates blinded this check and a fetch drive
            # crushed one (07-11). Convert start-frame memory to the current frame.
            ox, oy = px - fetch_off, py - fetch_lat
            if 0.05 < ox < tpos[0] - 0.04 and abs(oy) < 0.14:
                return False
        return True

    def strafe_clear(cands, tpos):
        # nothing in the sideways corridor we would strafe through (x out to ~0.42
        # covers the chassis footprint while translating laterally)
        lo, hi = (0.06, tpos[1] + 0.10) if tpos[1] > 0 else (tpos[1] - 0.10, -0.06)
        for pp, dd in cands[1:]:
            if pp[0] < 0.42 and lo < pp[1] < hi:
                return False
        for px, py, _pz in placed_pts + floor_drops:
            ox, oy = px - fetch_off, py - fetch_lat
            if 0.05 < ox < 0.42 and lo < oy < hi:
                return False
        return True

    sem_targets = bool(os.environ.get('JR_TARGET', '').strip())
    flicker = 0
    chase = None    # (start_x, start_y, seen_x) of the last fetch-drive target
    while rclpy.ok() and attempts < int(os.environ.get("JR_MAX_ATTEMPTS", "20")):
        cand = pickable()
        # open-vocab names near the conf threshold FLICKER frame to frame; with a
        # semantic filter one unnamed frame reads as "floor clear" (ended a run with
        # a pen still down, 07-10) -- retry a few frames before believing emptiness
        if not cand and sem_targets and flicker < 3:
            flicker += 1
            print('[FLICKER] no named target this frame -> retry %d/3' % flicker)
            continue
        if cand:
            flicker = 0
        # CLOSEST-FIRST: anything cut off at the bottom edge sits right at the wheels --
        # reverse to resolve it BEFORE any grasp or fetch drive (fetch drives crushed one).
        # EXCEPT mid-fetch (fetch_off>0): cubes only look "too close" because WE drove
        # forward past them; reversing would just undo the fetch (it did, and burned the
        # drive budget). Grasp the fetched target first, the return trip restores them.
        if (AUTO_DRIVE and node.n_bottom > 0 and drives < MAX_DRIVES
                and fetch_off < 0.02 and abs(fetch_lat) < 0.02):
            print('[M6] %d target(s) too close (bottom-cut) -> reversing 0.13m first' % node.n_bottom)
            mv = drive_leg(-0.13); node.settle_after_drive()
            drives += 1; fetch_off += mv; blacklist.clear()
            continue
        if not cand:
            # nothing in the floor view: LOOK UP AND SCAN from OBSERVE for far objects
            far = None
            if AUTO_DRIVE and drives < MAX_DRIVES:
                far = node.survey_far(placed_pts, (fetch_off, fetch_lat))
            if far is not None:
                fx_, fy_ = far
                print('[M6] SURVEY target at (%.2f, %+.2f) -> going' % (fx_, fy_))
                if abs(fy_) >= 0.12 and drives + 1 < MAX_DRIVES:
                    mv = drive_leg(fy_, axis='y'); drives += 1; fetch_lat += mv
                d = fx_ - 0.30
                mv = drive_leg(d); node.settle_after_drive()
                drives += 1; fetch_off += mv; blacklist.clear()
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
            if st == 'OUT_OF_REACH':
                # CHASE-STALL guard: we drove at this target and it is NOT closer --
                # a floor object must close in by ~the drive length; one that stays
                # at the same range is a phantom (elevated object's footprint
                # projection). It dragged the robot 2.6m across the room (07-15).
                sfx, sfy = pos0[0] + fetch_off, pos0[1] + fetch_lat
                if (chase is not None and abs(sfx - chase[0]) < 0.15
                        and abs(sfy - chase[1]) < 0.15 and pos0[0] > chase[2] - 0.10):
                    print('  [PARK] %s: chased but no closer -> phantom, parked' % orig['id'])
                    hard_black.append((sfx, sfy))
                    blacklist.append((orig['u'], orig['v'])); consec += 1
                    log_stat(orig, 'CHASE_STALL', 'x stuck %.2f' % pos0[0])
                    node.home(); continue
            if st == 'OUT_OF_REACH' and AUTO_DRIVE and drives < MAX_DRIVES:
                if (abs(pos0[1]) >= 0.12 and strafe_clear(cand, pos0)
                        and abs(fetch_lat + pos0[1]) <= 0.30):   # net lateral budget: no wandering
                    # M6 lateral fetch: mecanum-strafe to line the target up first
                    print('[M6] %s lateral (y=%+.2f) -> strafing to align' % (orig['id'], pos0[1]))
                    mv = drive_leg(pos0[1], axis='y'); node.settle_after_drive()
                    drives += 1; fetch_lat += mv; blacklist.clear()
                    log_stat(orig, 'DRIVE_STRAFE', 'y=%.2f' % pos0[1])
                    node.home(); continue
                if abs(pos0[1]) < 0.12 and path_clear(cand, pos0):
                    # M6 fetch: target straight ahead but beyond the arm -> drive to it
                    d = pos0[0] - 0.24
                    print('[M6] %s out of reach (x=%.2f) -> driving %+.2fm to fetch' %
                          (orig['id'], pos0[0], d))
                    mv = drive_leg(d); node.settle_after_drive()
                    drives += 1; fetch_off += mv; blacklist.clear()
                    chase = (pos0[0] + fetch_off - mv, pos0[1] + fetch_lat, pos0[0])
                    log_stat(orig, 'DRIVE_FETCH', 'x=%.2f' % pos0[0])
                    node.home(); continue
            if st == 'OUT_OF_REACH' and (not AUTO_DRIVE or drives >= MAX_DRIVES):
                # can never reach it this run (no drive budget left): park it in the
                # PERMANENT list -- the pixel blacklist is cleared after every place/
                # drive, so these came back and burned the whole attempt cap (07-11)
                hard_black.append((pos0[0] + fetch_off, pos0[1] + fetch_lat))
                print('  [PARK] %s unreachable, no drive budget -> parked for this run' % orig['id'])
            blacklist.append((orig['u'], orig['v'])); consec += 1
            log.append((orig['id'], st, msg)); log_stat(orig, st, msg); node.home(); continue
        if os.environ.get('JR_CARRY', '0') == '1':
            # CARRY-MODE VERIFY: the local mode re-checks the pickup spot, carry mode
            # exited blind -- a missed grasp "delivered" an empty gripper on camera
            # (07-16). Look back at the spot (gripper untouched), retry if still there.
            node.move(1.2, FLOOR)
            node.fresh(); node.fresh()
            if node.spot_occupied(orig['u'], orig['v'], orig['label']):
                print('  -> CARRY MISS (cube still on floor) -> retrying')
                node.move(1.2, OBSERVE)
                blacklist.append((orig['u'], orig['v'])); consec += 1
                log.append((orig['id'], 'MISS', 'carry grasp missed'))
                log_stat(orig, 'MISS', 'carry grasp missed')
                continue
            node.move(1.2, OBSERVE)
            # mission mode (jr_mission.py): keep the cube in the gripper and exit -- the
            # mission NAVIGATES to the delivery pose and releases there. Arm is already
            # at OBSERVE with the gripper closed; no local place, no return drive.
            grabbed += 1
            log.append((orig['id'], 'CARRYING', msg)); log_stat(orig, 'CARRYING', msg)
            print('CARRYING: cube in gripper, exiting for delivery (no local place).')
            break
        if fetch_off != 0.0 or fetch_lat != 0.0:
            # carry the cube back to the start point so the paper is where we left it
            print('[M6] returning (x %+.2f, y %+.2f) before placing -> retracing legs' %
                  (-fetch_off, -fetch_lat))
            retrace_home()                      # exact LIFO retrace of the outbound legs
            node.settle_after_drive()
            fetch_off = 0.0; fetch_lat = 0.0; blacklist.clear()
        ppos, on_zone = node.place_on_paper((fetch_off, fetch_lat), placed_pts)
        node.home()
        still = node.spot_occupied(orig['u'], orig['v'], orig['label'])   # empty-gripper recheck (home view)
        if not still:
            grabbed += 1; consec = 0; log.append((orig['id'], 'SUCCESS', msg))
            (placed_pts if on_zone else floor_drops).append(
                (ppos[0] + fetch_off, ppos[1] + fetch_lat, ppos[2]))   # start frame
            log_stat(orig, 'SUCCESS', msg)
            print('  -> SUCCESS (pickup spot now empty)')
        else:
            blacklist.append((orig['u'], orig['v'])); consec += 1
            log.append((orig['id'], 'MISS', 'spot still occupied'))
            log_stat(orig, 'MISS', 'spot still occupied')
            print('  -> MISS (still there) -> blacklisted')
        if consec >= ABORT_FAILS:
            print('!! %d consecutive fails -> board may be locked / all hard. ABORT.' % consec); break

    if fetch_off != 0.0 or fetch_lat != 0.0:
        # never end a run displaced: drive back to the start point
        print('[M6] run ending displaced -> retracing to start (x %+.2f, y %+.2f)' % (-fetch_off, -fetch_lat))
        retrace_home()

    print('\n===== GRASP-ALL DONE =====  grabbed %d / %d attempts (blacklisted %d)' %
          (grabbed, attempts, len(blacklist)))
    for i, (cid, st, m) in enumerate(log):
        print('  %d %-8s %-12s %s' % (i, cid, st, m))
    print(stats_summary())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
