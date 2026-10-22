#!/usr/bin/env python3
# encoding: utf-8
"""station_dock.py -- precise station docking on an AprilTag board (the AMR "QR-code secondary
localisation" idea): Nav2 parks the robot at a station to +-10 cm / +-5 deg; the arm camera then
finds the 36h11 board on the floor and the mecanum base corrects x / y / yaw until the board centre
sits at a fixed offset in the arm frame. Pick / place afterwards is relative to that.

  python3 station_dock.py            # dock once (bringup + camera + kinematics running), print residuals
  python3 station_dock.py look       # measure only, no base motion
  JR_DOCK_PICKUP=1 python3 jr_grasp_all.py run   # placement no longer uses face_station: see task_place.py
Targets (arm frame, x forward, y left; board frame: x right on the page, y UP the page):
  JR_DOCK_X=0.30  JR_DOCK_Y=0.00  JR_DOCK_YAW=-90   (board top edge away from the robot)
Tolerances: 1.5 cm / 2 deg, max 4 iterations, max correction 0.30 m / 30 deg per iteration.
BASE MOVES -- never run while the robot is tethered to the bench adapter.
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import handeye_calib as hec
import jr_grasp_all as ga

TX = float(os.environ.get('JR_DOCK_X', '0.30'))
TY = float(os.environ.get('JR_DOCK_Y', '0.00'))
TYAW = math.radians(float(os.environ.get('JR_DOCK_YAW', '-90')))
TOL_XY, TOL_YAW = 0.015, math.radians(2.0)
MAX_ITERS = 5
TURN_SPEED = float(os.environ.get('JR_TURN_SPEED', '0.35'))     # rad/s
TURN_SIGN = float(os.environ.get('JR_TURN_SIGN', '1'))          # +1: Twist.angular.z>0 turns left (CCW)

# board layout in metres, y flipped to "up the page" so the 2D frame is right-handed seen from above
BOARD_PTS = {mid: np.array([(mid - hec.ID0) % hec.COLS * hec.PITCH, -((mid - hec.ID0) // hec.COLS) * hec.PITCH])
             for mid in range(hec.ID0, hec.ID0 + hec.COLS * hec.ROWS)}
BOARD_CENTRE = np.array([(hec.COLS - 1) / 2.0 * hec.PITCH, -(hec.ROWS - 1) / 2.0 * hec.PITCH])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Docker(object):
    def __init__(self, node):
        self.node = node
        self.yaw = None
        self.turn_sign = TURN_SIGN
        node.create_subscription(Odometry, '/odom_raw', self._odom, 1)

    def _odom(self, m):
        self.yaw = yaw_of(m.pose.pose.orientation)

    # ---- measurement -------------------------------------------------------------
    def measure_once(self):
        """board pose in the arm frame from one frame: (cx, cy, yaw, ntags) or None"""
        n = self.node
        gray = cv2.cvtColor(n.rgb, cv2.COLOR_BGR2GRAY)
        tags = hec.detect(gray)
        if not tags:
            return None
        ep = n.get_endpoint()
        pa, pb = [], []
        for mid, corners in tags.items():
            c = corners.mean(axis=0)
            fp = n.floor_point_cam(int(c[0]), int(c[1]))
            if fp is None:
                continue
            a = n.cam_to_arm(fp, ep)
            pa.append([float(a[0]), float(a[1])])
            pb.append(BOARD_PTS[mid])
        if not pa:
            return None
        pa, pb = np.array(pa), np.array(pb)
        if len(pa) >= 2:
            # 2D rigid fit board -> arm (Kabsch): p_a = R p_b + t
            ca, cb = pa.mean(axis=0), pb.mean(axis=0)
            H = (pb - cb).T @ (pa - ca)
            U, _, Vt = np.linalg.svd(H)
            d = np.sign(np.linalg.det(Vt.T @ U.T))
            R = Vt.T @ np.diag([1.0, d]) @ U.T
            yaw = math.atan2(R[1, 0], R[0, 0])
            centre = R @ (BOARD_CENTRE - cb) + ca
        else:
            # single tag: yaw from its own top edge (tl -> tr = board +x), centre offset along the fit
            mid, corners = next(iter(tags.items()))
            tl = n.cam_to_arm(n.floor_point_cam(int(corners[0][0]), int(corners[0][1])) if n.floor_point_cam(int(corners[0][0]), int(corners[0][1])) is not None else np.zeros(3), ep)
            tr = n.cam_to_arm(n.floor_point_cam(int(corners[1][0]), int(corners[1][1])) if n.floor_point_cam(int(corners[1][0]), int(corners[1][1])) is not None else np.zeros(3), ep)
            yaw = math.atan2(tr[1] - tl[1], tr[0] - tl[0])
            R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            centre = R @ (BOARD_CENTRE - BOARD_PTS[mid]) + pa[0]
        return float(centre[0]), float(centre[1]), float(yaw), len(pa)

    def measure(self, frames=2):
        """median over a few frames; None if the board is not seen"""
        xs, ys, yaws, nt = [], [], [], 0
        for _ in range(frames):
            if not self.node.fresh():
                continue
            m = self.measure_once()
            if m is None:
                continue
            xs.append(m[0]); ys.append(m[1]); yaws.append(m[2]); nt = max(nt, m[3])
        if not xs:
            return None
        return float(np.median(xs)), float(np.median(ys)), float(np.median(yaws)), nt

    def look(self):
        """try the viewing poses until the board is seen"""
        # OBSERVE first: the station board sits 0.5-0.7 m ahead after nav (FLOOR only sees 0.15-0.45 m)
        for pose, s1 in ((ga.OBSERVE, 500), (ga.FLOOR, 500), (ga.OBSERVE, 380), (ga.OBSERVE, 620), (ga.FLOOR, 400), (ga.FLOOR, 600)):
            self.node.move(0.8, ((1, s1),) + pose[1:])
            # 10-22: 0.3 s + 2 frames was measured while the arm was still settling (motion blur):
            # the board 0.40 m ahead with 8 tags visible was declared "not visible" twice (staged takes).
            # Let the arm settle, then retry a few times before giving up on this pose.
            time.sleep(0.6)
            for _ in range(3):
                m = self.measure()
                if m is not None:
                    return m
                time.sleep(0.3)
        return None

    # ---- motion ------------------------------------------------------------------
    def turn(self, dyaw):
        """odom-closed-loop yaw change in rad (+ = left/CCW); returns the achieved change"""
        if abs(dyaw) < math.radians(0.5):
            return 0.0
        n = self.node
        t0 = time.time()
        while self.yaw is None and time.time() - t0 < 3:
            rclpy.spin_once(n, timeout_sec=0.1)
        if self.yaw is None:
            print('  [DOCK] no odom yaw, not turning'); return 0.0
        y0 = self.yaw
        tw = Twist()
        tw.angular.z = float(self.turn_sign * (TURN_SPEED if dyaw > 0 else -TURN_SPEED))
        t0 = time.time()
        checked = False
        while abs(wrap(self.yaw - y0)) < abs(dyaw) and time.time() - t0 < 8:
            n.cmd.publish(tw)                       # ~10 Hz command stream, same as drive()
            end = time.time() + 0.1
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.01)
            # direction sanity after ~1 s: if the odom yaw moves the wrong way, flip the sign
            # once instead of spinning the wrong way for the full timeout
            if not checked and time.time() - t0 > 1.0:
                checked = True
                d = wrap(self.yaw - y0)
                if abs(d) > math.radians(2.0) and (d > 0) != (dyaw > 0):
                    print('  [DOCK]   odom yaw moved the wrong way (%+.1f deg) -> flipping turn sign' % math.degrees(d))
                    self.turn_sign = -self.turn_sign
                    tw.angular.z = -tw.angular.z
                    y0 = self.yaw                   # restart the measurement from here
                    t0 = time.time()
        n.cmd.publish(Twist()); n.cmd.publish(Twist())
        time.sleep(0.3)
        rclpy.spin_once(n, timeout_sec=0.1)
        return wrap(self.yaw - y0)

    def dock(self, move=True):
        n = self.node
        for it in range(MAX_ITERS + 1):
            m = self.look() if it == 0 else self.measure()
            if m is None:
                print('  [DOCK] board not visible -> abort'); return False
            cx, cy, yaw, nt = m
            dx, dy, dyaw = cx - TX, cy - TY, wrap(yaw - TYAW)
            print('  [DOCK] iter %d: board centre (%.3f, %.3f) yaw %+.1f deg from %d tags -> error dx %+.3f dy %+.3f dyaw %+.1f deg'
                  % (it, cx, cy, math.degrees(yaw), nt, dx, dy, math.degrees(dyaw)))
            if abs(dx) <= TOL_XY and abs(dy) <= TOL_XY and abs(dyaw) <= TOL_YAW:
                print('  [DOCK] docked (%.1f cm, %.1f deg residual)' % (math.hypot(dx, dy) * 100, math.degrees(dyaw)))
                return True
            if not move or it == MAX_ITERS:
                print('  [DOCK] %s' % ('measure only' if not move else 'not converged after %d iterations' % MAX_ITERS))
                return False
            if abs(dx) > 0.60 or abs(dy) > 0.60 or abs(dyaw) > math.radians(35):
                print('  [DOCK] correction too large for a docking move -> abort (re-send the nav goal)'); return False
            # per-step caps: the lidar cannot see low furniture (sofa / coffee table zones), so keep
            # every docking move small and let the loop converge over several iterations
            sx = max(-0.20, min(0.20, dx)); sy = max(-0.15, min(0.15, dy))
            syaw = max(-math.radians(20), min(math.radians(20), dyaw))
            # yaw first (it shifts the board in the image), then x / y via the odom-closed-loop drive
            if abs(dyaw) > TOL_YAW:
                got = self.turn(syaw)
                print('  [DOCK]   turned %+.1f deg' % math.degrees(got))
                continue                                # re-measure after a turn before translating
            if abs(dx) > TOL_XY:
                print('  [DOCK]   drive x %+.3f -> moved %+.3f' % (sx, n.drive(sx, 'x', speed=0.06)))
            if abs(dy) > TOL_XY:
                print('  [DOCK]   drive y %+.3f -> moved %+.3f' % (sy, n.drive(sy, 'y', speed=0.06)))
            time.sleep(0.4)
        return False


def dock(node, move=True):
    return Docker(node).dock(move=move)


def face_station(node, standoff=0.50, step_deg=35.0, max_turns=4):   # 4 x 35 = 140 deg max: a full turn wrecked AMCL (mission8)
    """FIND the station board by rotating in place, then FACE it at `standoff` metres (board centre
    ahead of the arm base). One turn to the bearing, one approach drive, one check -- no iterative
    docking. Used at the DELIVERY station before the zone scan (the mat is NOT a keepout -- user
    decision 10-21; the standoff only matters when nav parked us far). Returns True if the board ended up ahead within 10 deg."""
    d = Docker(node)
    m = d.look()
    if m is None:
        # 10-22: the usual reason for "not visible" right at the station is being TOO CLOSE -- the cube in
        # the gripper hides the lower-middle of the image (board at 0.5 m invisible, staged take 1).
        # Back off a little and look again BEFORE spinning the base (spinning wrecked AMCL in mission8).
        back = float(os.environ.get('JR_FACE_BACKOFF', '0.25'))
        print('  [FACE] board not visible -> back off %.2f m and look again' % back)
        node.drive(-back, 'x', speed=0.06)
        time.sleep(0.3)
        m = d.look()
    turned = 0.0
    while m is None and turned < math.radians(step_deg) * max_turns:
        got = d.turn(math.radians(step_deg))
        turned += abs(got)
        print('  [FACE] board not visible -> turned %+.0f deg (total %.0f)' % (math.degrees(got), math.degrees(turned)))
        if abs(got) < math.radians(5):
            print('  [FACE] base did not turn -> abort'); return False
        node.move(0.8, ((1, 500),) + ga.OBSERVE[1:])
        time.sleep(0.3)
        m = d.measure()
    if m is None:
        print('  [FACE] board not found after a full turn -> abort'); return False
    cx, cy, yaw, nt = m
    bearing = math.atan2(cy, cx)
    print('  [FACE] board at (%.2f, %.2f), bearing %+.1f deg, %d tags' % (cx, cy, math.degrees(bearing), nt))
    if abs(bearing) > math.radians(4):
        got = d.turn(max(-math.radians(60), min(math.radians(60), bearing)))
        print('  [FACE]   turned %+.1f deg to face it' % math.degrees(got))
        m = d.measure()
        if m is None:
            print('  [FACE] lost the board after turning -> stop here'); return False
        cx, cy, yaw, nt = m
        bearing = math.atan2(cy, cx)
    dist = math.hypot(cx, cy)
    # only close in when the board is clearly too far for the zone scan (OBSERVE sees to ~1.2 m,
    # tags read to ~0.7 m); otherwise leave the distance to the place routine's own micro-approach
    # 10-22: the cube in the gripper hides the lower-middle of the OBSERVE image (0.38-0.6 m ahead):
    # a board at 0.5 m is INVISIBLE (staged take 1). So never close in below ~1 m here; the place
    # routine scans from here and does its own micro-approach with the mapped hull.
    if dist > float(os.environ.get('JR_FACE_DRIVE_MIN', '1.05')):
        dx = min(0.40, dist - max(standoff, 0.80))
        print('  [FACE]   board at %.2f m -> drive %+.2f to standoff %.2f' % (dist, dx, standoff))
        print('  [FACE]   moved %+.2f' % node.drive(dx, 'x', speed=0.07))
        m = d.measure()
        if m is not None:
            cx, cy, yaw, nt = m
            bearing = math.atan2(cy, cx)
    print('  [FACE] done: board %.2f m ahead, bearing %+.1f deg' % (math.hypot(cx, cy), math.degrees(bearing)))
    return abs(bearing) <= math.radians(10)


def main():
    rclpy.init()
    node = ga.GraspAll()
    if node.wait_bridge() < 1:
        print('servo bridge not connected'); return
    if not node.fresh(t=15.0):
        print('no camera'); return
    ok = dock(node, move=(len(sys.argv) < 2 or sys.argv[1] != 'look'))
    node.move(0.8, ga.OBSERVE)
    node.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
