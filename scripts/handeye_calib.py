#!/usr/bin/env python3
# encoding: utf-8
"""handeye_calib.py -- eye-in-hand calibration with the A4 AprilTag 36h11 board.

Board (docs/tag_board_36h11_A4.png, make_tag_board.py) printed at scale 0.91:
tag side 36.4 mm, centre pitch 54.6 mm, 3 cols x 4 rows, ids 10..21 row-major.
Board frame: origin = centre of tag 10, x = along columns (right on the page),
y = along rows (down the page), z = right-handed (into the board).

Procedure (ONE long-lived node, ~2-3 min):
  for each of N arm poses (IK targets around the board, varied x/y/z/pitch + base yaw):
      move, settle, grab frames, detect tags (cv2.aruco legacy API), solvePnP -> board in camera
      FK -> hand in base
  cv2.calibrateHandEye (several methods) -> camera in hand = HAND2CAM
  consistency metric: board pose in the base frame must be the same from every view

Usage (robot, bringup+kinematics running, board flat on the floor ~25 cm ahead, arrow away):
  python3 handeye_calib.py            # calibrate, print proposals + residuals, save handeye_result.npz
  python3 handeye_calib.py check      # only evaluate the CURRENT HAND2CAM consistency
  JR_TAG_MM=36.4 JR_PITCH_MM=54.6     # override printed sizes
"""
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy

import jr_grasp_all as ga

TAG = float(os.environ.get('JR_TAG_MM', '36.4')) / 1000.0
PITCH = float(os.environ.get('JR_PITCH_MM', '54.6')) / 1000.0
COLS, ROWS, ID0 = 3, 4, 10
MIN_TAGS = 4
# IK targets (x, y, z, pitch_deg) -- the camera must keep the board (25 cm ahead) in view;
# variety in pitch and in base yaw (via y) is what makes the hand-eye problem well-conditioned
POSES = [
    (0.22, 0.00, 0.20, 80), (0.22, 0.00, 0.14, 80), (0.28, 0.00, 0.17, 80),
    (0.22, 0.07, 0.17, 80), (0.22, -0.07, 0.17, 80), (0.26, 0.10, 0.15, 80), (0.26, -0.10, 0.15, 80),
    (0.20, 0.00, 0.18, 60), (0.24, 0.06, 0.16, 60), (0.24, -0.06, 0.16, 60), (0.30, 0.00, 0.14, 60),
    (0.18, 0.00, 0.20, 45), (0.22, 0.08, 0.18, 45), (0.22, -0.08, 0.18, 45),
    (0.16, 0.00, 0.22, 90), (0.24, 0.00, 0.22, 90),
]


def tag_corners_board(mid):
    """4 corners of tag `mid` in the board frame, in the aruco order (tl, tr, br, bl)"""
    k = mid - ID0
    r, c = divmod(k, COLS)
    cx, cy = c * PITCH, r * PITCH
    h = TAG / 2.0
    return np.array([[cx - h, cy - h, 0], [cx + h, cy - h, 0], [cx + h, cy + h, 0], [cx - h, cy + h, 0]], np.float64)


def detect(gray):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    try:
        det_ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        corners, ids, _ = det_.detectMarkers(gray)
    except AttributeError:
        p = cv2.aruco.DetectorParameters_create()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
    if ids is None:
        return {}
    return {int(i): c[0] for i, c in zip(ids.flatten(), corners) if ID0 <= int(i) < ID0 + COLS * ROWS}


def board_pose(tags, K):
    """solvePnP on all detected tag corners -> (R, t) board->camera, reprojection rms (px)"""
    obj, img = [], []
    for mid, c in tags.items():
        obj.append(tag_corners_board(mid)); img.append(c.astype(np.float64))
    obj = np.vstack(obj); img = np.vstack(img)
    Km = np.array([[K[0], 0, K[2]], [0, K[4], K[5]], [0, 0, 1]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, Km, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, Km, None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), rms


def consistency(H, views):
    """board pose in the base frame from every view through hand->cam H; returns
    (position spread mm, orientation spread deg, list of board positions)"""
    Ts = []
    for ep, R, t in views:
        Tc = np.eye(4); Tc[:3, :3] = R; Tc[:3, 3] = t
        Ts.append(ep @ H @ Tc)
    P = np.array([T[:3, 3] for T in Ts])
    Rm = np.mean([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(Rm); Rm = U @ Vt
    angs = []
    for T in Ts:
        dR = Rm.T @ T[:3, :3]
        angs.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2.0)))))
    return float(np.sqrt(np.mean(np.sum((P - P.mean(0)) ** 2, axis=1)))) * 1000.0, float(np.sqrt(np.mean(np.square(angs)))), P


def main():
    check_only = len(sys.argv) > 1 and sys.argv[1] == 'check'
    rclpy.init()
    node = ga.GraspAll()
    if node.wait_bridge() < 1:
        print('servo bridge not up'); return
    node.home()
    views = []
    pulses = []
    for i, (x, y, z, pit) in enumerate(POSES):
        req = ga.SetRobotPose.Request()
        req.position = [float(x), float(y), float(z)]; req.pitch = float(pit)
        req.pitch_range = [-180.0, 180.0]; req.resolution = 1.0
        r = node._call(node.ik, req)
        if not (r and r.pulse):
            print('pose %2d (%.2f,%.2f,%.2f,p%d): no IK, skipped' % (i, x, y, z, pit)); continue
        p = list(r.pulse)
        node.move(1.3, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        time.sleep(0.8)
        got = None
        for _ in range(3):
            if not node.fresh():
                continue
            gray = cv2.cvtColor(node.rgb, cv2.COLOR_BGR2GRAY)
            tags = detect(gray)
            if len(tags) < MIN_TAGS:
                continue
            bp = board_pose(tags, node.K)
            if bp is None:
                continue
            ep = node.get_endpoint()
            got = (ep, bp[0], bp[1], bp[2], len(tags))
            break
        if got is None:
            print('pose %2d (%.2f,%.2f,%.2f,p%d): board not seen (<%d tags)' % (i, x, y, z, pit, MIN_TAGS)); continue
        ep, R, t, rms, ntag = got
        views.append((ep, R, t))
        pulses.append(p[:5])                 # servo pulses (ids 1-5) for kinematic calibration
        print('pose %2d (%.2f,%.2f,%.2f,p%d): %2d tags, pnp rms %.2f px, board at cam dist %.3f m' %
              (i, x, y, z, pit, ntag, rms, float(np.linalg.norm(t))))
    node.home()
    print('\n%d usable views' % len(views))
    if len(views) < 5:
        print('need >= 5 views with the board visible -- move the board / check lighting'); node.destroy_node(); rclpy.shutdown(); return

    H0 = ga.HAND2CAM
    pm, om, _ = consistency(H0, views)
    print('current HAND2CAM (JR_CAM_PITCH=%.1f): board position spread %.1f mm, orientation spread %.2f deg'
          % (ga.CAM_PITCH, pm, om))
    if check_only:
        node.destroy_node(); rclpy.shutdown(); return

    Rg2b = [v[0][:3, :3] for v in views]; tg2b = [v[0][:3, 3] for v in views]
    Rt2c = [v[1] for v in views]; tt2c = [v[2] for v in views]
    best = None
    for name, m in (('TSAI', cv2.CALIB_HAND_EYE_TSAI), ('PARK', cv2.CALIB_HAND_EYE_PARK),
                    ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD), ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS)):
        try:
            Rc2g, tc2g = cv2.calibrateHandEye(Rg2b, tg2b, Rt2c, tt2c, method=m)
        except Exception as e:      # noqa: BLE001
            print('%-10s failed: %s' % (name, e)); continue
        H = np.eye(4); H[:3, :3] = Rc2g; H[:3, 3] = tc2g.reshape(3)
        pm, om, _ = consistency(H, views)
        # express as pitch about y_hand relative to the nominal parallel mount, for comparison
        zc = Rc2g[:, 2]                     # camera forward in the hand frame
        pitch = math.degrees(math.atan2(-zc[2], zc[0]))
        print('%-10s spread %.1f mm / %.2f deg   cam forward in hand %s (pitch %.1f deg)   t = %s mm' %
              (name, pm, om, np.round(zc, 3).tolist(), pitch, np.round(tc2g.reshape(3) * 1000, 1).tolist()))
        if best is None or pm < best[1]:
            best = (name, pm, H)
    if best:
        name, pm, H = best
        np.set_printoptions(precision=4, suppress=True)
        print('\nBEST: %s (spread %.1f mm). Proposed HAND2CAM:' % (name, pm))
        print(H)
        print('current for reference:'); print(H0)
        np.savez(os.path.expanduser('~/jetrover_ws/handeye_result.npz'), H=H, H0=H0,
                 ep=np.array([v[0] for v in views]), R=np.array(Rt2c), t=np.array(tt2c),
                 pulses=np.array(pulses, dtype=np.float64))
        print('saved ~/jetrover_ws/handeye_result.npz  (note: cam_to_arm() still subtracts 0.01 from cam x --'
              ' remove that hack when adopting a calibrated matrix)')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
