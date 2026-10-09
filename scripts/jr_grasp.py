#!/usr/bin/env python3
# encoding: utf-8
# M5 grasp: YOLO target -> registered depth -> camera 3D -> (hand-eye + FK endpoint)
# -> arm-base 3D -> IK (/kinematics/set_pose_target) -> servo pulses -> grasp sequence.
# Follows the vendor RGBD-grasp convention (hand2cam matrix + get_current_pose), but
# is our own code calling vendor SERVICES; does not modify vendor code.
#
#   python3 jr_grasp.py            # DRY RUN: detect+transform+IK, print, NO arm motion
#   python3 jr_grasp.py grab       # EXECUTE: approach -> close -> lift -> return
#
# Localization (v2): a shiny can loses depth on its body, and its near-surface depth
# maps to the can EDGE (top-down grasp then closes on the edge -> miss). So we localize
# the can's FOOTPRINT center by intersecting the box-bottom ray with the RANSAC floor
# plane (floor depth is valid), and grasp directly above that true center.
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

import os
MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
TARGETS = set(os.environ.get('JR_TARGETS', 'bottle,cup,wine glass').split(','))
DUR = float(os.environ.get('JR_DUR', '2.0'))  # motion-duration scale; >1 = slower (testing)
PITCH = float(os.environ.get('JR_PITCH', '80'))  # gripper pitch deg (80~vertical top-down)
# eye-in-hand calibration: camera-optical -> gripper/end-effector (vendor track_and_grab)
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600


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


class Grasp(Node):
    def __init__(self):
        super().__init__('jr_grasp')
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
        self.model = YOLO(MODEL)

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
        # the servo bridge (controller_manager) must have discovered our publisher
        # before we send, else commands are dropped (esp. after a wifi/DDS re-discovery)
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
        # let the message actually go out before the next sleep/command
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_data(self, t=8.0):
        t0 = time.time()
        while (self.rgb is None or self.depth is None or self.K is None) and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rgb is not None and self.depth is not None and self.K is not None

    def detect_target(self):
        # return the best target that has VALID depth (skip glass/edge invalid ones)
        res = self.model(self.rgb, conf=0.25, verbose=False)[0]
        names = res.names
        cw, ch = self.rgb.shape[1] / 2, self.rgb.shape[0] / 2
        cands = []
        skipped = []
        for b in res.boxes:
            cls = names[int(b.cls)]
            if cls not in TARGETS:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            dist = self.median_depth_m(u, v)
            d = ((u - cw) ** 2 + (v - ch) ** 2) ** 0.5
            if dist <= 0:
                skipped.append((cls, u, v))
                continue
            cands.append((d, u, v, cls, float(b.conf), dist, (x1, y1, x2, y2)))
        for cls, u, v in skipped:
            print('  skip %s @(%d,%d): invalid depth' % (cls, u, v))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        return cands[0]  # (dist_px, u, v, cls, conf, dist_m, box)

    def median_depth_m(self, u, v, win=25):
        # Specular objects (a shiny coke can) lose depth on the body, so a tiny
        # window at the box center reads invalid. Use a wider window and take a
        # LOW percentile: valid returns cluster on the can edges (the closest
        # surface ~0.22m); a low percentile locks onto the object and rejects the
        # further floor that a large window would otherwise mix in.
        d = self.depth
        h, w = d.shape
        patch = d[max(0, v - win):min(h, v + win + 1),
                  max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = patch[(patch > 0) & (patch < 10000)]
        if vals.size < 10:
            return 0.0
        return float(np.percentile(vals, 25)) / 1000.0

    def fit_floor(self, box):
        # RANSAC the dominant plane from valid depth OUTSIDE the can box -> floor.
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
        if n[1] > 0:  # camera optical +y is down; make the plane normal point "up"
            n, dd = -n, -dd
        return n, dd

    def footprint_cam(self, box):
        # intersect the box-bottom-center ray with the floor plane = can base center
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
        foot = dirb * (-dd / denom)          # near base rim (front, camera-facing)
        # the visible base rim is offset from the can's central axis by ~one radius
        # toward the camera; push the point AWAY from the camera along the floor to
        # land on the true base center (radius estimated from the box width).
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
        c = np.array([cam_xyz[0] - 0.01, cam_xyz[1], cam_xyz[2], 1.0])  # rgb/depth offset
        return (endpoint @ HAND2CAM @ c)[:3]

    def target_to_armbase(self, u, v, dist):
        # legacy box-center + near-surface depth (kept for dry-run comparison)
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        cam = depth_pixel_to_camera(u, v, dist, fx, fy, cx, cy)
        endpoint = self.get_endpoint()
        return self.cam_to_armbase(cam[:3], endpoint), endpoint

    def solve_ik(self, pos, pitch):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in pos]
        req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]
        req.resolution = 1.0
        return self._call(self.ik, req)

    def move(self, dur, positions):
        # scaled, settle-coupled motion (slow for testing via JR_DUR)
        self.servos(dur * DUR, positions)
        time.sleep(dur * DUR + 0.2)

    def pick(self, pos, pitch):
        if self.wait_bridge() < 1:
            self.get_logger().warn('servo bridge not connected; commands would be lost; abort')
            return False
        r = self.solve_ik(pos, pitch)
        if not (r and r.pulse):
            self.get_logger().warn('no IK solution for approach; abort')
            return False
        p = r.pulse
        self.get_logger().info('approach pulses %s (DUR=%.1fx)' % (list(p), DUR))
        self.move(0.6, ((10, GRIPPER_OPEN),))                                # OPEN gripper first
        self.move(1.0, ((1, p[0]),))                                          # base yaw first
        self.move(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        self.move(0.8, ((10, GRIPPER_CLOSE),))                               # close gripper
        lift = [pos[0], pos[1], pos[2] + 0.05]                                # lift 5cm
        r2 = self.solve_ik(lift, pitch)
        if r2 and r2.pulse:
            q = r2.pulse
            self.move(1.2, ((1, q[0]), (2, q[1]), (3, q[2]), (4, q[3]), (5, q[4])))
        self.get_logger().info('lifted; holding then returning to observe (still gripping)')
        time.sleep(2.0)
        self.move(1.5, OBSERVE[:5] + ((10, GRIPPER_CLOSE),))
        return True


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'dry'
    rclpy.init()
    node = Grasp()
    if not node.wait_data():
        print('ERROR: no camera/depth/info'); return
    tgt = node.detect_target()
    if tgt is None:
        print('no target with valid depth in view (targets=%s)' % sorted(TARGETS)); return
    _, u, v, cls, conf, dist, box = tgt
    print('target: %s conf=%.2f box=%s center=(%d,%d) near_depth=%.3fm' % (cls, conf, box, u, v, dist))

    # legacy localization (box center + near-surface depth) -- for comparison only
    old_pos, endpoint = node.target_to_armbase(u, v, dist + 0.03)
    print('endpoint(arm-base) xyz = %s' % np.round(endpoint[:3, 3], 3).tolist())
    print('LEGACY pt (box center, arm-base) xyz = %s' % np.round(old_pos, 3).tolist())

    # footprint localization: floor-plane intersection at box bottom -> true center
    foot_cam = node.footprint_cam(box)
    if foot_cam is None:
        print('floor-plane footprint failed; falling back to legacy point')
        pos = list(old_pos)
    else:
        base = node.cam_to_armbase(foot_cam, endpoint)
        print('FOOTPRINT (cam) xyz = %s' % np.round(foot_cam, 3).tolist())
        print('FOOTPRINT (arm-base, floor center) xyz = %s' % np.round(base, 3).tolist())
        # grasp directly above the true center; height = proven-reaching legacy z,
        # or override via JR_GRASP_H (metres above the floor footprint)
        if 'JR_GRASP_H' in os.environ:
            z = float(base[2]) + float(os.environ['JR_GRASP_H'])
        else:
            z = float(old_pos[2])
        pos = [float(base[0]), float(base[1]), z]
    print('GRASP point (arm-base) xyz = %s  pitch=%.0f' % (np.round(pos, 3).tolist(), PITCH))

    r = node.solve_ik(pos, PITCH)
    if r and r.pulse:
        print('IK OK: pulses=%s rpy=%s' % (list(r.pulse), [round(x, 1) for x in r.rpy]))
    else:
        print('IK: NO SOLUTION (success=%s)' % (getattr(r, 'success', None)))
    if mode == 'grab':
        if r and r.pulse:
            print('=== EXECUTING GRASP ===')
            node.pick(list(pos), PITCH)
        else:
            print('no IK solution -> not executing')
    else:
        print('(dry run -- no motion. run "jr_grasp.py grab" to execute)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
