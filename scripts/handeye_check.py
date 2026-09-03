#!/usr/bin/env python3
# encoding: utf-8
"""handeye_check.py -- estimate the camera's pitch/roll error from the FLOOR: the fitted
floor normal (camera frame), mapped through FK @ HAND2CAM into the arm frame, must be
vertical. Uses the frames dumped by yaw_calib.py (depth + K + ep per frame).
Prints per-frame and mean tilt, the rotation axis, and a corrected HAND2CAM proposal
(rotation only -- translation is verified separately with the ruler / grasp1 dry).
Usage (robot): python3 handeye_check.py [glob]   (default: all frames)
"""
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser('~/jetrover_ws'))
import jr_detect_objects as det        # noqa: E402
from jr_grasp_all import HAND2CAM      # noqa: E402


def rodrigues(axis, ang):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else '*'
    d = os.path.expanduser('~/jetrover_ws/yaw_calib_frames')
    stems = sorted(p[:-10] for p in glob.glob(os.path.join(d, pattern + '_depth.npy')))
    ups, ncams = [], []
    print('%-26s %8s %8s %8s %7s' % ('frame', 'up_x', 'up_y', 'up_z', 'tilt'))
    for st in stems:
        depth = np.load(st + '_depth.npy'); K = np.load(st + '_K.npy').tolist(); ep = np.load(st + '_ep.npy')
        z = depth.astype(np.float32) / 1000.0
        fl = det.fit_floor(z, K[0], K[4], K[2], K[5])
        if fl is None:
            continue
        n = np.asarray(fl[0], np.float64); n /= np.linalg.norm(n)
        if n[2] > 0:            # make it point toward the camera (camera looks down +z)
            n = -n
        R = (ep @ HAND2CAM)[:3, :3]
        up = R @ n              # should be (0,0,1) in the arm frame
        if up[2] < 0:
            up = -up
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        ups.append(up); ncams.append(n)
        print('%-26s %+8.3f %+8.3f %+8.3f %6.2f' % (os.path.basename(st), up[0], up[1], up[2], tilt))
    if not ups:
        print('no frames'); return
    up = np.mean(ups, axis=0); up /= np.linalg.norm(up)
    tilt = math.degrees(math.acos(up[2]))
    print('\nmean "up" in arm frame: %s   tilt from vertical: %.2f deg' % (np.round(up, 4).tolist(), tilt))
    print('  -> floor appears to rise toward arm %s%s' % (
        '+x (forward)' if up[0] < 0 else '-x (backward)', ' / lateral %+.2f deg' % math.degrees(math.asin(-up[1]))))
    print('  (a pure camera PITCH error shows as up_x != 0; ROLL as up_y != 0; yaw is not observable here)')
    # correction: rotate HAND2CAM's rotation so that the mean measured normal maps to vertical.
    # up = R n  ->  we want R' n = e_z  ->  R' = Q R with Q rotating up onto e_z (arm frame).
    ez = np.array([0.0, 0.0, 1.0])
    axis = np.cross(up, ez)
    if np.linalg.norm(axis) < 1e-9:
        print('no correction needed'); return
    Q = rodrigues(axis, math.acos(up[2]))
    # HAND2CAM is expressed in the HAND frame: R_arm = R_ep @ R_h2c. A correction applied in
    # the arm frame (Q) corresponds to R_h2c' = R_ep^T Q R_ep R_h2c -- pose dependent unless
    # all frames share the pose; the dumped frames are all the FLOOR pose (+ base yaw), so use
    # the mean ep.
    eps = [np.load(st + '_ep.npy') for st in stems]
    Rep = np.mean([e[:3, :3] for e in eps], axis=0)
    U, _, Vt = np.linalg.svd(Rep); Rep = U @ Vt
    Rh = Rep.T @ Q @ Rep @ HAND2CAM[:3, :3]
    H = HAND2CAM.copy(); H[:3, :3] = Rh
    np.set_printoptions(precision=4, suppress=True)
    print('\nproposed HAND2CAM (rotation corrected, translation unchanged):')
    print(H)
    print('\ncheck: mean normal through the proposal -> %s' % np.round((Rep @ Rh) @ np.mean(ncams, axis=0), 4).tolist())


if __name__ == '__main__':
    main()
