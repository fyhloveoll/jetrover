#!/usr/bin/env python3
# encoding: utf-8
"""kin_model_match.py -- align our URDF-based FK with the VENDOR FK by brute-forcing joint
conventions. The npz holds, per view, the servo pulses and the vendor end-effector pose (ep).
For each joint 1-4 try sign in {+1,-1} and offset in {0, +90, -90, 180} deg on the mapped
vendor angle; the base translation is solved in closed form (mean difference). Prints the
conventions that reproduce the vendor end-effector POSITION best (and the rotation match).
Usage: python3 kin_model_match.py [docs/handeye_2026-09-04.npz]"""
import itertools
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, 'scripts')
from kin_calib_fit import J1_XYZ, J2_XYZ, J3_XYZ, J4_XYZ, T, pulse2angle, rot_axis   # noqa: E402

SERVO2 = (0, 0, 0.0544833339503674)   # link4 -> servo_link2 (joint5 origin)
EE = (0, 0, 0.08)                     # link5 -> end_effector_link


def fk_ee(q):
    M = T(J1_XYZ) @ rot_axis((0, 0, -1), q[0])
    M = M @ T(J2_XYZ) @ rot_axis((0, 1, 0), q[1])
    M = M @ T(J3_XYZ) @ rot_axis((0, 1, 0), q[2])
    M = M @ T(J4_XYZ) @ rot_axis((0, 1, 0), q[3])
    M = M @ T(SERVO2) @ rot_axis((0, 0, -1), q[4]) @ T(EE)
    return M


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'docs/handeye_2026-09-04.npz'
    d = np.load(path)
    P, EPs = d['pulses'], d['ep']
    Q = np.array([pulse2angle(p) for p in P])
    n = len(Q)
    tgt = np.array([e[:3, 3] for e in EPs])
    signs = (1, -1); offs = (0, 90, -90, 180)
    results = []
    for combo in itertools.product(itertools.product(signs, offs), repeat=4):
        pos = []
        for i in range(n):
            q = [combo[j][0] * Q[i][j] + math.radians(combo[j][1]) for j in range(4)] + [0.0]
            pos.append(fk_ee(q)[:3, 3])
        pos = np.array(pos)
        off = (tgt - pos).mean(axis=0)
        rms = float(np.sqrt(np.mean(np.sum((pos + off - tgt) ** 2, axis=1))))
        results.append((rms, combo, off))
    results.sort(key=lambda r: r[0])
    print('vendor ee position range: x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f' %
          (tgt[:, 0].min(), tgt[:, 0].max(), tgt[:, 1].min(), tgt[:, 1].max(), tgt[:, 2].min(), tgt[:, 2].max()))
    for rms, combo, off in results[:6]:
        print('rms %6.1f mm  conv %s  base offset xyz %s mm' %
              (rms * 1000, ['%+d*q%+d' % (s, o) for s, o in combo], np.round(off * 1000, 1).tolist()))
    # rotation check for the best combo
    rms, combo, off = results[0]
    angs = []
    for i in range(n):
        q = [combo[j][0] * Q[i][j] + math.radians(combo[j][1]) for j in range(4)] + [0.0]
        Rm = fk_ee(q)[:3, :3]
        angs.append(math.degrees(Rot.from_matrix(EPs[i][:3, :3].T @ Rm).magnitude()))
    print('best combo rotation mismatch vs vendor: mean %.1f deg (a constant value = fixed frame rotation, fine)' % np.mean(angs))
    print('per-view rotation mismatch: %s' % np.round(angs, 1).tolist())


if __name__ == '__main__':
    main()
