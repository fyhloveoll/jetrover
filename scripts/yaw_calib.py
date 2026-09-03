#!/usr/bin/env python3
# encoding: utf-8
"""yaw_calib.py -- phase A of the paper-line yaw calibration: PERCEPTION ONLY, arm parks at
the FLOOR observation pose and never grasps.

Rig: a sheet taped flat on the floor in front of the arm base with lines drawn at known
angles (0, +-15, +-30, +-40; avoid 45 -- a square is ambiguous there). The 0 deg line
must be parallel to the arm's forward axis. Angle convention: seen from above, positive =
counter-clockwise from forward (toward the robot's LEFT). Cube angles are mod 90.

For every placement you type the true angle (and an optional position tag such as
"left", "near"); the script grabs N frames and logs, per frame:
  - angle_image : raw minAreaRect angle in the image  (JR_YAW_MODE=image, current default)
  - angle_floor : contour back-projected onto the fitted floor plane, angle in the ARM
                  frame (JR_YAW_MODE=floor) -- geometrically correct for a tilted camera
  - pixel position, floor-plane normal, depth  (to show position dependence)
and prints per-placement circular means + errors, then an overall summary.

Expected outcome: image error grows with |angle| and changes with position;
floor error is flat (~+-2 deg). A constant offset on BOTH at 0 deg = sheet not parallel
to the arm axis (or hand-eye yaw) -- subtract it, don't chase it.

Usage (robot, bringup with camera + kinematics running):
  cd ~/jetrover_ws && python3 yaw_calib.py [--frames 6] [--csv ~/jetrover_ws/yaw_calib.csv]
"""
import argparse
import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy

import jr_grasp_all as ga


def wrap90(a):
    """wrap a (deg) into [-45, 45)"""
    return ((a + 45.0) % 90.0) - 45.0


def circ_mean(angles):
    return ga.GraspAll.circ_mean_angle(angles)


def circ_std(angles, mean):
    if len(angles) < 2:
        return 0.0
    return math.sqrt(sum(wrap90(a - mean) ** 2 for a in angles) / (len(angles) - 1))


def pick_target(insts, K):
    """the cube on the sheet = detection closest to the image centre"""
    if not insts:
        return None
    cx, cy = K[2], K[5]
    return min(insts, key=lambda o: (o['u'] - cx) ** 2 + (o['v'] - cy) ** 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=6)
    ap.add_argument('--csv', default=os.path.expanduser('~/jetrover_ws/yaw_calib.csv'))
    ap.add_argument('--no-home', action='store_true', help='do not move the arm to FLOOR pose')
    ap.add_argument('--watch', default='', help='remote-driven mode: poll this file for "<deg> [tag]" '
                    'lines (or "peek" = report detections without logging, "q" = quit) instead of stdin')
    ap.add_argument('--dump', default=os.path.expanduser('~/jetrover_ws/yaw_calib_frames'),
                    help='save rgb/depth/K/FK per logged frame here for offline re-analysis ("" = off)')
    args = ap.parse_args()
    if args.dump:
        os.makedirs(args.dump, exist_ok=True)

    def next_command():
        """one command line from stdin, or from the watch file (deleted after reading)"""
        if not args.watch:
            try:
                return input('true angle [tag] > ').strip()
            except EOFError:
                return 'q'
        while True:
            if os.path.exists(args.watch):
                with open(args.watch) as fh:
                    line = fh.read().strip()
                os.remove(args.watch)
                if line:
                    return line
            rclpy.spin_once(node, timeout_sec=0.2)

    rclpy.init()
    node = ga.GraspAll()
    if node.wait_bridge() < 1:
        print('servo bridge not up (is bringup running?)'); return
    if not args.no_home:
        node.home()          # FLOOR observation pose, gripper open -- same view grasps use
        time.sleep(0.5)
    if not node.fresh():
        print('no camera frames (rgb/depth/camera_info) -- is the camera up?'); return

    new = not os.path.exists(args.csv)
    f = open(args.csv, 'a', newline='')
    w = csv.writer(f)
    if new:
        w.writerow(['ts', 'true_deg', 'tag', 'frame', 'u', 'v', 'depth_m', 'angle_image',
                    'angle_floor', 'n_x', 'n_y', 'n_z', 'plane_d', 'elong'])
    sessions = []   # (true, tag, mean_img, mean_floor, n)

    print('\nplace the cube on a line, then type: <true_deg> [tag]   e.g. "30", "-15 left", "0 far"')
    print('"peek" = list detections without logging; "q" = finish.\n', flush=True)
    while True:
        line = next_command()
        if not line or line.lower() == 'q':
            break
        if line.lower().startswith('pose'):
            # "pose <servo1_pulse>": rotate the base (camera viewpoint) keeping the FLOOR
            # observation pose otherwise -- used to check that the arm-frame angle is
            # viewpoint-invariant (500 = straight ahead, ~4.17 pulse/deg)
            try:
                s1 = int(line.split()[1])
            except (IndexError, ValueError):
                print('usage: pose <servo1_pulse>', flush=True); continue
            node.move(1.2, tuple((i, (s1 if i == 1 else p)) for i, p in ga.FLOOR))
            time.sleep(0.8)
            print('[pose] servo1=%d, arm settled\n' % s1, flush=True)
            continue
        if line.lower() == 'peek':
            if node.fresh():
                fl = node.fit_floor(None)
                print('[peek] floor plane: %s' % (('n=%s d=%.3f' % (np.round(fl[0], 3).tolist(), fl[1])) if fl else 'NO FIT'))
                ds = node.detect()
                print('[peek] %d detections' % len(ds))
                for o in ds:
                    print('   %-8s px(%d,%d) area=%.0f depth=%.3f angle=%+.0f elong=%.2f' %
                          (o['id'], o['u'], o['v'], o['area'], o.get('depth', 0.0), o['angle'], o.get('elong', 1.0)))
            else:
                print('[peek] no camera frames')
            print('', flush=True)
            continue
        parts = line.split()
        try:
            true = float(parts[0])
        except ValueError:
            print('  first token must be a number'); continue
        tag = parts[1] if len(parts) > 1 else ''

        ep = node.get_endpoint()
        imgs, floors = [], []
        for i in range(args.frames):
            if not node.fresh():
                print('  frame %d: no data' % i); continue
            fl = node.fit_floor(None)
            if args.dump:
                stem = os.path.join(args.dump, '%d_%+03d_%s_%d' % (int(time.time()), int(true), tag or 'c', i))
                np.save(stem + '_depth.npy', np.asarray(node.depth))
                cv2.imwrite(stem + '_rgb.png', node.rgb)
                np.save(stem + '_K.npy', np.asarray(node.K, dtype=np.float64))
                np.save(stem + '_ep.npy', np.asarray(ep, dtype=np.float64))
            inst = pick_target(node.detect(), node.K)
            if inst is None:
                print('  frame %d: nothing detected' % i); continue
            a_img = float(inst['angle'])
            a_fl = node.floor_angle_arm(inst, ep) if inst.get('cnt') is not None else None
            n = fl[0] if fl else (float('nan'),) * 3
            d = fl[1] if fl else float('nan')
            imgs.append(a_img)
            if a_fl is not None:
                floors.append(float(a_fl))
            w.writerow(['%.2f' % time.time(), true, tag, i, inst['u'], inst['v'],
                        '%.3f' % inst.get('depth', 0.0), '%.1f' % a_img,
                        '%.1f' % a_fl if a_fl is not None else '', '%.3f' % n[0], '%.3f' % n[1],
                        '%.3f' % n[2], '%.3f' % d, '%.2f' % inst.get('elong', 1.0)])
            print('  frame %d: px(%d,%d) depth %.2f  image %+6.1f  floor %s' %
                  (i, inst['u'], inst['v'], inst.get('depth', 0.0), a_img,
                   ('%+6.1f' % a_fl) if a_fl is not None else '  n/a'))
        f.flush()
        if not imgs:
            print('  no usable frames for this placement'); continue
        mi = circ_mean(imgs)
        mf = circ_mean(floors) if floors else float('nan')
        si = circ_std(imgs, mi)
        sf = circ_std(floors, mf) if floors else float('nan')
        ei = wrap90(mi - true)
        ef = wrap90(mf - true) if floors else float('nan')
        sessions.append((true, tag, mi, mf, len(imgs)))
        print('  => true %+5.0f %-6s image %+6.1f (sd %.1f, err %+5.1f)   floor %+6.1f (sd %.1f, err %+5.1f)'
              % (true, tag, mi, si, ei, mf, sf, ef))

    f.close()
    if sessions:
        print('\n==== summary (err = measured - true, wrapped to +-45) ====')
        print('%6s %-8s %8s %8s %8s %8s' % ('true', 'tag', 'image', 'err_img', 'floor', 'err_flr'))
        for true, tag, mi, mf, n in sessions:
            print('%+6.0f %-8s %+8.1f %+8.1f %+8.1f %+8.1f' %
                  (true, tag, mi, wrap90(mi - true), mf, wrap90(mf - true)))
        ei = [abs(wrap90(mi - t)) for t, _, mi, _, _ in sessions]
        ef = [abs(wrap90(mf - t)) for t, _, _, mf, _ in sessions if not math.isnan(mf)]
        print('mean |err|: image %.1f deg   floor %.1f deg   (csv: %s)' %
              (np.mean(ei), np.mean(ef) if ef else float('nan'), args.csv))
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
