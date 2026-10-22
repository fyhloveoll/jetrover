#!/usr/bin/env python3
# encoding: utf-8
"""place_skill.py -- put the cube held in the gripper on the DARK MAT.
Mat (dark blob, _zone_detect strict=False) is the primary cue; AprilTags only trim noise / anchor the hull.
Base moves only through GraspAll.drive() (lidar-guarded) and only when the scan is fresh. No base rotation.
The gripper is opened ONLY after a mat cell has IK at hover (+0.07) AND at the drop height.
Non-zero return == cube still in the gripper, arm back at the carry pose. release() sets node.place_released
right after the servo-10 write; task_place.py turns any error AFTER that point into [PLACE/WARN] + exit 0.

Library only: the one-file entry (and what jr_mission.py calls) is task_place.py, which also
documents the exit codes. Every GraspAll call below is quoted with its jr_grasp_all.py line (1738-line file,
10-22). The old place block (:1357-1388 of the 1769-line file) was deleted 10-22; formulas taken from it are
marked "old place block (git history)" and the numbers chosen there have NOT been exercised on the robot.
Env: JR_PLACE_MAX_LEGS (default 3) base legs before giving up; JR_PLACE_BUDGET (default 180 s)."""
import math
import os
import sys
import time

import cv2
import numpy as np
from kinematics_msgs.srv import GetRobotPose          # jr_grasp_all.py:25

sys.path.insert(0, '/home/ubuntu/jetrover_ws')         # put_down.py:4 (flat deploy)
import jr_grasp_all as ga                               # station_dock.py:28 pattern  # noqa: E402

# ---- return codes (task_place.py exits with these) ----
RC_OK, RC_USAGE = 0, 2
RC_STACK, RC_BOARD = 10, 12                 # bridge/camera/FK/scan missing ; BOARD DEAD
RC_NOT_SEEN, RC_MAP_REJECT = 20, 21         # no mat in any view ; <8 pts or >0.45 m^2
RC_SCAN, RC_BLOCKED, RC_BUDGET = 30, 31, 32 # scan stale/no beams ; drive refused / legs or reverse budget ; time budget
RC_NO_CELL, RC_EXEC = 40, 42                # centre in window but no hover+drop IK cell ; runtime error

MAX_LEGS = int(os.environ.get('JR_PLACE_MAX_LEGS', '3'))
BUDGET_S = float(os.environ.get('JR_PLACE_BUDGET', '180'))
X_TARGET = 0.28             # chosen 10-22, never exercised on the robot: (zx - 0.28) came from the old place block
                            # (deleted 10-22, git history); the surviving approach formula is :1178 (x - 0.30, no
                            # 0.34/0.20 window); 0.28 is mid-envelope of zone_cell's 0.12<x<0.36 (:1148)
REV_LEG, REV_TOTAL = 0.15, 0.25   # chosen 10-22, never exercised: reverse is lidar-blind (CLAUDE.md; :1468); the only
                            # exercised reverse leg is 0.13 (:1576-1577); 0.25 = JR_FACE_BACKOFF station_dock.py:219.
                            # 0.15 + 0.15 > 0.25 -> in practice ONE full reverse leg per run (guarded_leg)
SETTLE_S = 0.6              # station_dock.py:121-124 (0.3 s measured while the arm was still settling)
RICH_VIEW_PTS = 20          # jr_grasp_all.py:1071 (one view with >=20 pts is enough)
NEAR_X = 0.60               # rescan with FLOOR-first views when the mat is expected below this:
                            # OBSERVE lower-middle hidden by the carried cube 0.38-0.65 m (handoff:72), FLOOR sees 0.15-0.45 (:1063)
# base servo1 pulses from scan_zone :1065-1066 (500 = straight, 660/340 = +-38 deg at PAN_GAIN 4.17 pulse/deg :63)
VIEWS_FAR = [(ga.OBSERVE, 500), (ga.OBSERVE, 660), (ga.OBSERVE, 340),
             (ga.FLOOR, 500), (ga.FLOOR, 660), (ga.FLOOR, 340)]
VIEWS_NEAR = [(ga.FLOOR, 500), (ga.FLOOR, 660), (ga.FLOOR, 340),
              (ga.OBSERVE, 660), (ga.OBSERVE, 340)]            # OBS 500 omitted: blind band with a cube in the gripper


# ---- tools ----
def log(step, msg):
    print('[PLACE/%s] %s' % (step, msg), flush=True)


class Fail(Exception):
    def __init__(self, rc, why):
        Exception.__init__(self, why)
        self.rc = rc
        self.why = why


def _f(v):
    # clearance()/age formatting: None stays 'None'
    return 'None' if v is None else '%.2f' % v


def carry_pose(node, dur=1.2):
    node.move(dur, ((1, 500),) + ga.OBSERVE[1:])      # same pose as scan_zone's return :1107 (old place block :1369/:1387 deleted 10-22); servo 10 untouched


def arm_from(p):
    return ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4]))   # :1245/:1248


def pump_scan(node, t=0.5):
    # :626-628 verbatim -- scan_t only advances while mon_exec is pumped
    t_req = time.time()
    while node.scan_t < t_req and time.time() - t_req < t:
        node.mon_exec.spin_once(timeout_sec=0.05)
    return node.scan_t >= t_req


# ---- 2.3 preflight ----
def preflight(node, need_scan, liveness=True):
    nb = node.wait_bridge()                                             # :172
    if nb < 1:
        raise Fail(RC_STACK, 'servo bridge not connected -> bash ~/jetrover_ws/place_boot.sh (staged) or mission_boot.sh X Y YAW')
    if not node.fresh(t=15.0):                                          # :160; startup wait as main :1300
        raise Fail(RC_STACK, 'no camera frames in 15 s')
    if node._call(node.fk, GetRobotPose.Request()) is None:             # :167 (5 s timeout -> None)
        raise Fail(RC_STACK, 'FK service /kinematics/get_current_pose not answering -> ros2 launch kinematics kinematics_node.launch.py')
    t_end = time.monotonic() + 2.0
    while node.scan is None and time.monotonic() < t_end:
        node.mon_exec.spin_once(timeout_sec=0.05)
    pump_scan(node, 0.5)
    age = time.time() - node.scan_t if node.scan is not None else float('inf')
    front, left, right, back = [node.clearance(a) for a in (0.0, math.pi / 2, -math.pi / 2, math.pi)]   # :596
    log('PRE', 'bridge=%d cam=ok fk=ok scan=%s age=%.1f front=%s left=%s right=%s back=%s'
        % (nb, 'Y' if node.scan is not None else 'N', age, _f(front), _f(left), _f(right), _f(back)))
    if need_scan and node.scan is None:
        raise Fail(RC_STACK, 'no /scan: base moves would be blind (drive() cannot stop without a scan) -- check the lidar')
    if liveness and not node.board_alive():                             # :189, prints [LIVENESS] scene diff
        raise Fail(RC_BOARD, 'BOARD DEAD (scene diff < 2.0): power-cycle the board')


# ---- 2.4 self_blob: the carried cube / gripper / own front frame seen as a dark blob (ALL views) ----
def self_blob(node, c, pose):
    """True == DISCARD this contour (never used to open the gripper). Three existing rules combined:
    bottom edge cut off (:1404/:551) AND centre inside the gripper ROI (gripper_colour_px :765)
    AND depth invalid for the Dabai (<0.25 m, :730) or absurd.
    10-22 fix round 1: the OBSERVE-only gate is gone. The rules do not depend on the pose (HAND2CAM :79 is rigid,
    so the carried cube sits in the same pixels in every view; the robot's own front frame shows at the FLOOR
    bottom edge, :200-205) and the FLOOR views project to 0.15-0.45 m -- exactly zone_cell's envelope, where an
    unfiltered self blob would have become a "mat" with hover+drop IK. `pose` is kept for the call site only.
    NOT caught: a shadow on the floor in front of the robot (its depth is valid) -- a near, tag-less dark blob
    can still open the gripper; only tag anchoring / strict mode prevents that (handoff 10-22 section)."""
    if c is None:
        return False
    h, w = node.rgb.shape[:2]
    _x, y, _bw, bh = cv2.boundingRect(c)
    if y + bh < h - 3:
        return False
    mom = cv2.moments(c)
    if mom['m00'] <= 0:
        return False
    cu, cv_ = int(mom['m10'] / mom['m00']), int(mom['m01'] / mom['m00'])
    if not (0.3 * w <= cu <= 0.7 * w and cv_ >= 0.6 * h):
        return False
    d = node.depth_m(cu, cv_)                                           # :488 (0.0 when invalid)
    return not (0.25 <= d <= 2.0)


# ---- 2.5 look: stop-and-stare views, everything in the CURRENT arm frame ----
_look_n = 0                 # per-process look counter: the debug jpgs of the look after a leg must not overwrite the first look's


def look(node, views, all_views=False, debug_dir=None):
    global _look_n
    _look_n += 1
    pts, zs, anchors, objs = [], [], [], []
    nviews, seen, floor_seen = 0, 0, False    # seen = views with a usable mat/tag (map_mat: 'seen but unprojectable' != 'not seen')
    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
    for pose, s1 in views:
        tag = 'OBS' if pose is ga.OBSERVE else 'FLR'
        node.move(0.7, ((1, s1),) + pose[1:])                           # :1073, servo 10 untouched
        time.sleep(SETTLE_S)
        if not (node.fresh() and node.fresh()):
            log('LOOK', '%s s1=%d NO FRAME' % (tag, s1))
            continue
        tags = node._apriltags()                                        # :424
        c = node._zone_detect(strict=False)                             # :445
        selfb = self_blob(node, c, pose)
        if debug_dir:
            img = node.rgb.copy()
            if c is not None:
                cv2.drawContours(img, [c], -1, (0, 0, 255) if selfb else (0, 255, 0), 2)
            for t in tags:
                cv2.polylines(img, [np.asarray(t, dtype=np.int32).reshape(-1, 1, 2)], True, (255, 0, 0), 2)
            cv2.imwrite(os.path.join(debug_dir, '%02d_%s_%d.jpg' % (_look_n, tag, s1)), img)
        n0 = len(pts)
        n_none = 0
        fit = '-'
        if tags or (c is not None and not selfb):
            seen += 1
            ep = node.get_endpoint()                                    # :526, once per view (:1083)
            for t in tags:                                              # :1085-1094 tag centres = anchors
                tc = t.mean(axis=0)
                fp = node.floor_point_cam(int(tc[0]), int(tc[1]))      # :1014
                if fp is None:
                    n_none += 1
                    continue
                a = node.cam_to_arm(fp, ep)                             # :531
                if 0.05 < a[0] < 1.2 and abs(a[1]) < 0.8:
                    anchors.append((float(a[0]), float(a[1])))
                    zs.append(float(a[2]))
            if c is not None and not selfb:                             # :1095-1103 contour fill
                for p in c[::6]:
                    fp = node.floor_point_cam(int(p[0][0]), int(p[0][1]))
                    if fp is None:
                        n_none += 1
                        continue
                    a = node.cam_to_arm(fp, ep)
                    if 0.05 < a[0] < 1.2 and abs(a[1]) < 0.8:
                        pts.append((float(a[0]), float(a[1])))
                        zs.append(float(a[2]))
            if pose is ga.FLOOR:
                floor_seen = True
                # objects already on/near the mat -> `placed` exclusions (survey_far pattern :691-698)
                for d in node.detect():                                 # :209
                    if d['box'][3] >= node.rgb.shape[0] - 3:
                        continue                                        # bottom-cut: footprint unreliable (:551)
                    if node.depth_m(d['u'], d['v']) <= 0:
                        continue
                    foot = node.footprint_cam(d['box'], d)              # :498
                    if foot is None:
                        continue
                    a = node.cam_to_arm(foot, ep)
                    objs.append((float(a[0]), float(a[1]), d['label']))
            fit = 'OK' if getattr(node, '_fl_fit', None) is not None else 'NONE'   # NONE == mission8 death (handoff:44)
        log('LOOK', '%s s1=%d tags=%d mat=%s px=%d self=%s pts+=%d (%d unprojectable, floor fit %s) objs=%d'
            % (tag, s1, len(tags), 'Y' if c is not None else 'N',
               int(cv2.contourArea(c)) if c is not None else 0, 'Y' if selfb else 'N',
               len(pts) - n0, n_none, fit, len(objs)))
        nviews += 1
        if not all_views and len(pts) - n0 >= RICH_VIEW_PTS:            # :1069-1071 early exit
            break
    carry_pose(node, 0.8)                                               # :1107
    return dict(pts=pts, zs=zs, anchors=anchors, objs=objs, nviews=nviews, seen=seen, floor_seen=floor_seen)


# ---- 2.6 map_mat: hull in the current arm frame ----
def map_mat(node, s):
    pts, anchors, zs = list(s['pts']), list(s['anchors']), list(s['zs'])
    if not pts and not anchors:
        if s.get('seen', 0) > 0:
            # the mission8 death (handoff:44): mat/tags detected, every point unprojectable -> NOT "not seen"
            raise Fail(RC_MAP_REJECT, 'mat/tags seen in %d of %d views but 0 projectable points -- floor fit None '
                       '(jr_detect_objects.fit_floor needs >=500 valid px)' % (s['seen'], s['nviews']))
        raise Fail(RC_NOT_SEEN, 'dark mat not seen in %d views -- park the robot >= 0.8 m from the mat edge (handoff:75)' % s['nviews'])
    n_raw = len(pts)
    if anchors:                                                         # :1112-1114 tags only trim / anchor
        pts = [p for p in pts if min((p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2 for a in anchors) < 0.09]
        pts.extend(anchors)
    if len(pts) < 8:
        raise Fail(RC_MAP_REJECT, '%s but only %d floor points (raw %d, anchors %d) -- floor fit None? '
                   '(jr_detect_objects.fit_floor needs >=500 valid px)'
                   % ('mat seen' if n_raw else 'tags only', len(pts), n_raw, len(anchors)))
    hull = cv2.convexHull((np.array(pts, dtype=np.float32) * 1000).astype(np.int32))   # :1116 (mm int32)
    area = cv2.contourArea(hull) / 1e6
    if area > 0.45:                                                     # :1117-1121
        raise Fail(RC_MAP_REJECT, 'hull %.2f m2 > 0.45 implausible (%d pts)' % (area, len(pts)))
    h = hull.reshape(-1, 2).astype(np.float32) / 1000.0                 # same reshape as zone_cell :1138; hull mean = mat centre (old place block, deleted 10-22)
    cx, cy = float(h[:, 0].mean()), float(h[:, 1].mean())
    z = float(np.median(zs))                                            # :1125
    log('MAP', 'pts=%d (raw %d, anchors %d) area=%.3f m2 centre=(%.2f,%+.2f) z=%.3f objs=%d%s'
        % (len(pts), n_raw, len(anchors), area, cx, cy, z, len(s['objs']),
           '  (HULL FROM TAGS ONLY)' if n_raw == 0 else ''))
    return dict(hull=hull, z=z, area=area, cx=cx, cy=cy, n=len(pts), ntag=len(anchors))


# ---- 2.7 plan: a mat cell with IK at hover AND drop ----
def plan(node, placed):
    tried = list(placed)                      # (x, y, ...) current arm frame
    for _k in range(5):
        cell = node.zone_cell((0.0, 0.0), tried)                        # :1132
        if cell is None:
            log('PLAN', 'NO_CELL in envelope 0.12<x<0.36 |y|<0.34 (%d excluded)' % len(tried))
            return None, None, None
        pos = [float(cell[0]), float(cell[1]), node.zone_z + 0.05]      # :1187
        hi, _ = node.solve_ik_multi([pos[0], pos[1], pos[2] + 0.07])   # :1243
        lo, pit = node.solve_ik_multi(pos)                              # :1246
        log('PLAN', 'cell=(%.3f,%+.3f) z=%.3f ik_hover=%s ik_drop=%s pitch=%s'
            % (pos[0], pos[1], pos[2], 'ok' if hi else 'NONE', 'ok' if lo else 'NONE', pit))
        if hi and lo:
            return pos, hi, lo
        tried.append((pos[0], pos[1]))        # zone_cell's 0.055 m exclusion skips this cell next time (:1145)
    log('PLAN', 'NO_IK: 5 cells without hover+drop IK')
    return None, None, None


# ---- 2.8 leg_for: the old place block's formula (deleted 10-22, git history; the live relative is :1178-1179 with 0.30) ----
def leg_for(cx, cy):
    dx = max(-0.3, min(0.45, cx - X_TARGET)) if (cx > 0.34 or cx < 0.20) else 0.0
    dy = max(-0.3, min(0.3, cy)) if abs(cy) > 0.18 else 0.0
    return dx, dy


# ---- 2.9 guarded_leg: refuse rather than drive blind ----
def guarded_leg(node, dist, axis, rev_used):
    pump_scan(node, 0.5)
    if node.scan is None:
        raise Fail(RC_SCAN, 'no /scan -> refusing to move')
    age = time.time() - node.scan_t
    if age >= 1.0:
        raise Fail(RC_SCAN, 'scan stale %.1fs -> refusing to move (drive() would go e-stop blind, :664-666)' % age)
    if node.odom is None:                                               # :108/:123/:153; drive() :620-621 would return 0.0 and look like an obstacle
        raise Fail(RC_STACK, 'no /odom_raw -> refusing to move (drive() cannot measure the leg)')
    direction = (0.0 if dist > 0 else math.pi) if axis != 'y' else (math.pi / 2 if dist > 0 else -math.pi / 2)   # :624-625
    if axis == 'x' and dist < 0:
        dist = max(dist, -REV_LEG)
        if rev_used + abs(dist) > REV_TOTAL:
            raise Fail(RC_BLOCKED, 'reverse budget %.2f m exhausted (lidar-blind direction)' % REV_TOTAL)
        log('GO', 'WARNING: lidar-blind REVERSE %.2f m (cap %.2f/leg, %.2f total)' % (dist, REV_LEG, REV_TOTAL))
    else:
        if node.clearance(direction) is None:                           # :596
            raise Fail(RC_SCAN, 'no lidar beams toward %s%+d -> refusing to move' % (axis, 1 if dist > 0 else -1))
    return node.drive(dist, axis=axis)        # :611; truncation / e-stop / zero-velocity / its own print


# ---- 2.10 release: the ONLY servo-10 command in this module ----
def release(node, pos, hi, lo):
    node.move(1.2, arm_from(hi))                                        # :1245 hover
    node.move(1.0, arm_from(lo))                                        # :1248 drop height
    node.move(0.6, ((10, ga.GRIPPER_OPEN),))                            # :1261
    node.place_released = True            # read by task_place.py: from here on an error is AFTER the release (cube on the mat)
    log('RELEASE', 'opened at %s' % np.round(pos, 3).tolist())
    carry_pose(node, 1.2)                                               # old place block :1387 (deleted 10-22); pose = :1107


# ---- 2.11 verify_on_mat: informational only ----
def verify_on_mat(node):
    node.move(1.2, ((1, 500),) + ga.FLOOR[1:])                          # FLOOR :84, servo 10 untouched
    time.sleep(SETTLE_S)
    if not (node.fresh() and node.fresh()):                             # :160 clears rgb first: a missed frame leaves rgb None
        log('VERIFY', 'NO FRAME (informational, rc unchanged)')
    else:
        pc = node._zone_detect(strict=False)
        on = [o['label'] for o in node.detect() if node.on_paper(pc, o['u'], o['v'])]   # :484
        log('VERIFY', 'mat=%s on_mat=%d %s (informational, rc unchanged)'
            % ('Y' if pc is not None else 'N', len(on), on))
    carry_pose(node, 1.2)


# ---- 2.12 run ----
def run(node, dry=False, look_only=False, max_legs=MAX_LEGS, verify=False, debug_dir=None):
    t0 = time.monotonic()                     # handoff:40 NTP jump lesson (stack_wait.py:52-55)
    node.place_released = False               # set True by release() right after the servo-10 write
    carry_pose(node, 1.5)                                               # old place block :1369-1370 (deleted 10-22); pose = :1107
    node.fresh()
    s = look(node, VIEWS_FAR, all_views=look_only, debug_dir=debug_dir)
    m = map_mat(node, s)
    if look_only:
        return RC_OK
    legs, rev_used = 0, 0.0
    placed = [(o[0], o[1]) for o in s['objs']]
    while True:
        if time.monotonic() - t0 > BUDGET_S:
            raise Fail(RC_BUDGET, 'time budget %.0f s exceeded' % BUDGET_S)
        node.zone_hull, node.zone_z = m['hull'], m['z']                 # zone_cell reads these (:1135-1137)
        pos, hi, lo = plan(node, placed)
        if pos is not None:
            break
        dx, dy = leg_for(m['cx'], m['cy'])
        if dx == 0.0 and dy == 0.0:
            raise Fail(RC_NO_CELL, 'hull centre (%.2f,%+.2f) already in the approach window but no cell with '
                       'hover+drop IK (hull %.3f m2, %d objs)' % (m['cx'], m['cy'], m['area'], len(placed)))
        if legs >= max_legs:
            raise Fail(RC_BLOCKED, 'leg budget %d exhausted, mat centre still at (%.2f,%+.2f)' % (max_legs, m['cx'], m['cy']))
        dx_go = max(dx, -REV_LEG) if dx < 0 else dx                     # guarded_leg caps a reverse leg: log what will actually be requested
        log('GO', 'leg %d: mat centre (%.2f,%+.2f) -> drive x %+.2f then y %+.2f (front clearance %s)'
            % (legs + 1, m['cx'], m['cy'], dx_go, dy, _f(node.clearance(0.0))))
        if dry:
            log('GO', 'DRY: not driving')
            return RC_OK
        mx = guarded_leg(node, dx, 'x', rev_used) if dx else 0.0
        rev_used += max(0.0, -mx)
        my = guarded_leg(node, dy, 'y', rev_used) if dy else 0.0
        legs += 1
        if abs(mx) < 0.01 and abs(my) < 0.01:
            raise Fail(RC_BLOCKED, 'drive moved 0 -- see the [DRIVE] line above (obstacle within CLEAR_MARGIN %.2f, '
                       "or drive()'s 20 s timeout without odom progress)" % ga.CLEAR_MARGIN)
        node.settle_after_drive()                                       # :715
        exp_x = m['cx'] - mx
        s = look(node, VIEWS_NEAR if exp_x < NEAR_X else VIEWS_FAR, debug_dir=debug_dir)
        try:
            m = map_mat(node, s)
        except Fail as f:
            raise Fail(f.rc, 'mat lost after leg %d (expected at x=%.2f): %s' % (legs, exp_x, f.why))
        placed = [(o[0], o[1]) for o in s['objs']]
    if not s['floor_seen']:
        # objs (-> placed exclusions) are only collected at FLOOR; a rich OBSERVE view can end look() before one.
        # Informational only: the cell may land on a cube already on the mat (T15). No extra view added (10-22, user decides).
        log('PLAN', 'WARNING: no FLOOR view in the last look -> objects already on the mat were not looked for (%d exclusions)' % len(placed))
    if dry:
        log('PLAN', 'DRY: would lower to %s and open' % np.round(pos, 3).tolist())
        return RC_OK
    release(node, pos, hi, lo)
    if verify:
        try:
            verify_on_mat(node)
        except Exception as e:                                          # informational only: can never change rc
            log('VERIFY', 'skipped: %r (informational, rc unchanged)' % (e,))
            try:
                carry_pose(node, 1.2)
            except Exception:
                pass
    print('PLACE DONE rc=0 released at %s (%d pts, %.3f m2, tags=%d, legs=%d, %.0f s)'
          % (np.round(pos, 3).tolist(), m['n'], m['area'], m['ntag'], legs, time.monotonic() - t0), flush=True)
    return RC_OK
