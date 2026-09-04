#!/usr/bin/env python3
# encoding: utf-8
"""kin_calib_fit2.py -- arm kinematic calibration in two stages (OFFLINE, numpy + scipy).

Stage 1 (clone the vendor model): with the joint conventions found by kin_model_match.py
  (q1' = -q1, q2' = q2 + 90 deg, q3' = q3, q4' = q4 + 90 deg), fit link lengths L2..L4, the
  tool offset and the base translation so that our FK reproduces the vendor FK end-effector
  positions (npz 'ep') to ~mm. This makes the nominal model == what the vendor IK assumes.
Stage 2 (calibrate against the AprilTag board): on the cloned model, fit joint zero offsets
  dq[1..4] (j5 unobservable), a free camera mount on link4 (6-DoF, init from URDF) and the
  board pose so that the board pose predicted from every view agrees. The offsets are
  relative to the vendor model -> usable as a pulse correction after the vendor IK.

Usage: python3 kin_calib_fit2.py [docs/handeye_2026-09-04.npz]
"""
import math
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, 'scripts')
from kin_calib_fit import J1_XYZ, J2_XYZ, J3_XYZ, J4_XYZ, CAM_NOMINAL, T, pulse2angle, rot_axis, spread  # noqa: E402

SIGN = np.array([-1.0, 1.0, 1.0, 1.0])
OFFS = np.radians([0.0, 90.0, 0.0, 90.0])
SERVO2_Z = 0.0544833339503674
EE_Z = 0.08


def qconv(q):
    return SIGN * q[:4] + OFFS


def fk_chain(qc, L, base):
    """base -> link4 and -> tool, with lengths L = [L2, L3, L4, Ltool] and base translation"""
    M = T(base) @ T(J1_XYZ) @ rot_axis((0, 0, -1), qc[0])
    M = M @ T((0, 0, L[0])) @ rot_axis((0, 1, 0), qc[1])
    M = M @ T((0, 0, L[1])) @ rot_axis((0, 1, 0), qc[2])
    M4 = M @ T((0, 0, L[2])) @ rot_axis((0, 1, 0), qc[3])
    Mt = M4 @ T((0, 0, L[3]))
    return M4, Mt


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'docs/handeye_2026-09-04.npz'
    d = np.load(path)
    P, EPs, R, t = d['pulses'], d['ep'], d['R'], d['t']
    Q = np.array([pulse2angle(p) for p in P])
    n = len(Q)
    tgt = np.array([e[:3, 3] for e in EPs])

    # ---- stage 1: clone the vendor FK ----
    L0 = np.array([J2_XYZ[2], J3_XYZ[2], J4_XYZ[2], SERVO2_Z + EE_Z])

    def res1(x):
        L, base = x[:4], x[4:7]
        return np.concatenate([(fk_chain(qconv(Q[i]), L, base)[1][:3, 3] - tgt[i]) * 1000 for i in range(n)])

    x1 = np.concatenate([L0, [-0.0118, 0.0, 0.0825]])
    s1 = least_squares(res1, x1)
    L, base = s1.x[:4], s1.x[4:7]
    rms1 = math.sqrt(np.mean(np.sum(res1(s1.x).reshape(n, 3) ** 2, axis=1)))
    print('stage 1: URDF lengths %s -> fitted %s mm, base %s mm, ee rms %.2f mm (was %.1f)' %
          (np.round(L0 * 1000, 1).tolist(), np.round(L * 1000, 1).tolist(), np.round(base * 1000, 1).tolist(),
           rms1, math.sqrt(np.mean(np.sum(res1(x1).reshape(n, 3) ** 2, axis=1)))))

    # ---- stage 2: calibrate against the board ----
    cam0 = CAM_NOMINAL                       # link4 -> optical (URDF), refined as a free 6-DoF
    Tcb = []
    for i in range(n):
        M = np.eye(4); M[:3, :3] = R[i]; M[:3, 3] = t[i]; Tcb.append(M)

    def board_pred(i, dq, cam):
        M4, _ = fk_chain(qconv(Q[i] + dq), L, base)
        return M4 @ cam @ Tcb[i]

    def cam_of(x):
        return cam0 @ T(x[3:6], R=Rot.from_rotvec(x[0:3]).as_matrix())

    def res2(x, fit_dq=True):
        dq = np.zeros(5); dq[:4] = x[0:4] if fit_dq else 0.0
        cam = cam_of(x[4:10])
        Tb = T(x[13:16], R=Rot.from_rotvec(x[10:13]).as_matrix())
        r = []
        for i in range(n):
            Tp = board_pred(i, dq, cam)
            r.extend((Tp[:3, 3] - Tb[:3, 3]) * 1000)
            r.extend(np.degrees(Rot.from_matrix(Tb[:3, :3].T @ Tp[:3, :3]).as_rotvec()) * 2.0)
        return np.array(r)

    Ts0 = [board_pred(i, np.zeros(5), cam0) for i in range(n)]
    b0 = np.concatenate([Rot.from_matrix([M[:3, :3] for M in Ts0]).mean().as_rotvec(), np.mean([M[:3, 3] for M in Ts0], axis=0)])
    print('stage 2 nominal (URDF camera mount, no offsets): spread %.1f mm / %.2f deg' % spread(Ts0))
    # 2a: camera mount only (== hand-eye on our model), joint offsets fixed at 0
    x0 = np.concatenate([np.zeros(4), np.zeros(6), b0])
    sA = least_squares(lambda x: res2(x, fit_dq=False), x0)
    camA = cam_of(sA.x[4:10])
    TsA = [board_pred(i, np.zeros(5), camA) for i in range(n)]
    print('2a camera mount only: spread %.1f mm / %.2f deg; mount correction rot %s deg xyz %s mm' %
          (spread(TsA) + (np.round(np.degrees(sA.x[4:7]), 1).tolist(), np.round(sA.x[7:10] * 1000, 1).tolist())))
    # 2b: camera mount + joint offsets
    lo = np.concatenate([np.full(4, -math.radians(10)), np.full(3, -math.radians(30)), np.full(3, -0.1), np.full(6, -np.inf)])
    hi = -lo
    sB = least_squares(res2, np.clip(sA.x, lo + 1e-9, hi - 1e-9), bounds=(lo, hi))
    dq = np.zeros(5); dq[:4] = sB.x[0:4]
    camB = cam_of(sB.x[4:10])
    TsB = [board_pred(i, dq, camB) for i in range(n)]
    print('2b + joint offsets:  spread %.1f mm / %.2f deg' % spread(TsB))
    print('   joint zero offsets (deg, vendor-angle convention): ' +
          '  '.join('j%d %+.2f' % (i + 1, math.degrees(SIGN[i] * dq[i])) for i in range(4)))
    print('   camera mount correction: rot %s deg, xyz %s mm' % (np.round(np.degrees(sB.x[4:7]), 1).tolist(), np.round(sB.x[7:10] * 1000, 1).tolist()))
    Pm = np.mean([M[:3, 3] for M in TsB], axis=0)
    worst = sorted(((np.linalg.norm(M[:3, 3] - Pm) * 1000, i) for i, M in enumerate(TsB)), reverse=True)[:4]
    print('   worst views after fit: %s' % ', '.join('%d (%.1f mm)' % (i, e) for e, i in worst))
    print('pulse correction after the vendor IK (ADD to IK pulses):')
    from kin_calib_fit import MAPS
    for i in range(4):
        m = MAPS[i + 1]
        dv = -math.degrees(SIGN[i] * dq[i])                    # commanded = desired - offset
        print('  servo %d: %+.1f pulses' % (i + 1, dv / (m[4] - m[3]) * (m[1] - m[0])))
    np.savez(path.replace('.npz', '_kinfit2.npz'), L=L, base=base, dq=dq, cam=camB, spread_nominal=spread(Ts0), spread_fit=spread(TsB))


if __name__ == '__main__':
    main()
