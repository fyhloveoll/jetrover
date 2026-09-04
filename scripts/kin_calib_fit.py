#!/usr/bin/env python3
# encoding: utf-8
"""kin_calib_fit.py -- arm kinematic calibration (OFFLINE, numpy + scipy).

Data: docs/handeye_2026-09-04.npz from handeye_calib.py -- for each of N views: servo pulses
(ids 1-5), and the board pose in the camera (R, t from solvePnP on the AprilTag board).

Model (our own FK from the vendor URDF, docs/vendor_ref):
  base_link -joint1(z-)-> link1 -> joint2(y) -> link2 -> joint3(y) -> link3 -> joint4(y) -> link4
  camera: link4 -> camera_connect (xyz -0.0507 0 0.0505, rpy 0 0 -pi/2)
                -> depth_cam_link (xyz 0 0 0.0145, rpy pi -pi/2 -pi/2)
                -> depth_cam_frame (rpy -pi/2 0 -pi/2)  = optical frame (z forward, x right, y down)
  pulses -> joint angles via the vendor maps (kinematics/transform.py): linear, 500 = centre.

Unknowns: dq[5] joint zero offsets (rad; joint5 is unobservable with a link4 camera and is
fixed at 0), dcam[6] correction of the camera mount (rotvec + xyz, applied after the URDF
mount), board[6] pose in base. Residual per view: predicted board pose (from FK) vs the
shared board pose -> 3 position + 3 orientation terms. Reports before/after spread and the
offsets in degrees; the offsets can be applied as a pulse correction after the vendor IK.

Usage: python3 kin_calib_fit.py [docs/handeye_2026-09-04.npz] [--no-cam] [--scale]
  --no-cam : keep the URDF camera mount fixed (fit joint offsets only)
  --scale  : also fit a common link-length scale (arm sag proxy)
"""
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

MAPS = {  # pulse_min, pulse_max, pulse_centre, angle_at_min, angle_at_max, angle_at_centre (deg)
    1: [0, 1000, 500, -120, 120, 0],
    2: [0, 1000, 500, 30, -210, -90],
    3: [0, 1000, 500, 120, -120, 0],
    4: [0, 1000, 500, 30, -210, -90],
    5: [0, 1000, 500, -120, 120, 0],
}


def pulse2angle(p):
    out = []
    for i, v in enumerate(p[:5]):
        m = MAPS[i + 1]
        out.append(math.radians((v - m[2]) / (m[1] - m[0]) * (m[4] - m[3]) + m[5]))
    return np.array(out)


def T(xyz=(0, 0, 0), rpy=(0, 0, 0), R=None):
    M = np.eye(4)
    M[:3, :3] = Rot.from_euler('xyz', rpy).as_matrix() if R is None else R
    M[:3, 3] = xyz
    return M


def rot_axis(axis, th):
    return T(R=Rot.from_rotvec(np.asarray(axis, float) * th).as_matrix())


# URDF chain (metres)
J1_XYZ = (0.0251328065010765, 0, 0.0774026880954513)
J2_XYZ = (0, 0, 0.0338648012164686)
J3_XYZ = (0, 0, 0.129416446394797)
J4_XYZ = (0, 0, 0.129444631186569)
CAM_CONNECT = T((-0.0507060266977644, 0, 0.0505384841187764), (0, 0, -math.pi / 2))
DEPTH_LINK = T((0, 0, 0.014475), (math.pi, -math.pi / 2, -math.pi / 2))
OPTICAL = T((0, 0, 0), (-math.pi / 2, 0, -math.pi / 2))
CAM_NOMINAL = CAM_CONNECT @ DEPTH_LINK @ OPTICAL            # link4 -> optical frame


def fk_link4(q, scale=1.0):
    """base_link -> link4 for joint angles q[0..3] (URDF angle convention: joint angle about axis)"""
    M = T(J1_XYZ) @ rot_axis((0, 0, -1), q[0])
    M = M @ T(np.array(J2_XYZ) * scale) @ rot_axis((0, 1, 0), q[1])
    M = M @ T(np.array(J3_XYZ) * scale) @ rot_axis((0, 1, 0), q[2])
    M = M @ T(np.array(J4_XYZ) * scale) @ rot_axis((0, 1, 0), q[3])
    return M


def board_in_base(q, Rcb, tcb, dq, dcam, scale):
    Tc = np.eye(4); Tc[:3, :3] = Rcb; Tc[:3, 3] = tcb
    cam = CAM_NOMINAL @ T(dcam[3:6], R=Rot.from_rotvec(dcam[0:3]).as_matrix())
    return fk_link4(q + dq, scale) @ cam @ Tc


def spread(Ts):
    P = np.array([M[:3, 3] for M in Ts])
    Rm = Rot.from_matrix([M[:3, :3] for M in Ts]).mean()
    ang = [math.degrees((Rm.inv() * Rot.from_matrix(M[:3, :3])).magnitude()) for M in Ts]
    return float(np.sqrt(np.mean(np.sum((P - P.mean(0)) ** 2, axis=1)))) * 1000, float(np.sqrt(np.mean(np.square(ang))))


def main():
    path = next((a for a in sys.argv[1:] if not a.startswith('--')), 'docs/handeye_2026-09-04.npz')
    fit_cam = '--no-cam' not in sys.argv
    fit_scale = '--scale' in sys.argv
    d = np.load(path)
    P, R, t = d['pulses'], d['R'], d['t']
    Q = np.array([pulse2angle(p) for p in P])
    n = len(Q)
    print('%d views' % n)

    def unpack(x):
        dq = np.zeros(5); dq[:4] = x[0:4]
        dcam = x[4:10] if fit_cam else np.zeros(6)
        k = 10 if fit_cam else 4
        board = x[k:k + 6]
        scale = x[k + 6] if fit_scale else 1.0
        return dq, dcam, board, scale

    def resid(x):
        dq, dcam, board, scale = unpack(x)
        Tb = T(board[3:6], R=Rot.from_rotvec(board[0:3]).as_matrix())
        r = []
        for i in range(n):
            Tp = board_in_base(Q[i], R[i], t[i], dq, dcam, scale)
            r.extend((Tp[:3, 3] - Tb[:3, 3]) * 1000.0)                         # mm
            dR = Rot.from_matrix(Tb[:3, :3].T @ Tp[:3, :3]).as_rotvec()
            r.extend(np.degrees(dR) * 2.0)                                       # deg, weighted x2
        return np.array(r)

    # initial board pose = mean of the nominal predictions
    Ts0 = [board_in_base(Q[i], R[i], t[i], np.zeros(5), np.zeros(6), 1.0) for i in range(n)]
    p0 = np.mean([M[:3, 3] for M in Ts0], axis=0)
    r0 = Rot.from_matrix([M[:3, :3] for M in Ts0]).mean().as_rotvec()
    x0 = np.concatenate([np.zeros(4), np.zeros(6) if fit_cam else [], r0, p0, [1.0] if fit_scale else []])
    print('nominal URDF model: board spread %.1f mm / %.2f deg' % spread(Ts0))

    sol = least_squares(resid, x0, method='lm' if len(x0) <= 6 * n else 'trf', max_nfev=2000)
    dq, dcam, board, scale = unpack(sol.x)
    Ts1 = [board_in_base(Q[i], R[i], t[i], dq, dcam, scale) for i in range(n)]
    print('fitted: board spread %.1f mm / %.2f deg   (cost %.1f -> %.1f)' % (spread(Ts1) + (float(np.sum(resid(x0) ** 2)), float(np.sum(sol.fun ** 2)))))
    print('joint zero offsets dq (deg): ' + '  '.join('j%d %+.2f' % (i + 1, math.degrees(v)) for i, v in enumerate(dq[:4])) + '   (j5 unobservable)')
    if fit_cam:
        print('camera mount correction: rot %s deg, xyz %s mm' % (np.round(np.degrees(dcam[0:3]), 2).tolist(), np.round(dcam[3:6] * 1000, 1).tolist()))
    if fit_scale:
        print('link scale: %.4f' % scale)
    print('per-view position error after fit (mm):')
    Pm = np.mean([M[:3, 3] for M in Ts1], axis=0)
    for i, M in enumerate(Ts1):
        e = (M[:3, 3] - Pm) * 1000
        print('  view %2d pulses %s: %+6.1f %+6.1f %+6.1f |%5.1f|' % (i, np.round(P[i]).astype(int).tolist(), e[0], e[1], e[2], np.linalg.norm(e)))
    # pulse corrections to apply AFTER the vendor IK: command angle = desired - dq
    print('pulse correction after IK (add to the IK pulses):')
    for i in range(4):
        m = MAPS[i + 1]
        dp = -math.degrees(dq[i]) / (m[4] - m[3]) * (m[1] - m[0])
        print('  servo %d: %+.1f pulses' % (i + 1, dp))
    np.savez(os.path.splitext(path)[0] + '_kinfit.npz', dq=dq, dcam=dcam, board=board, scale=scale)


if __name__ == '__main__':
    main()
