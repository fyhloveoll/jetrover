#!/usr/bin/env python3
"""handeye_report.py -- per-view board position error from handeye_result.npz, grouped by pose
pitch, to see how much of the spread is arm-kinematics repeatability vs. camera transform."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser('~/jetrover_ws'))
from handeye_calib import POSES, consistency
d = np.load(os.path.expanduser('~/jetrover_ws/handeye_result.npz'))
views = list(zip(d['ep'], d['R'], d['t']))
for label, H in (('current (pitch 16.7 model)', d['H0']), ('calibrated (HORAUD)', d['H'])):
    pm, om, P = consistency(H, views)
    mean = P.mean(0)
    print('== %s: spread %.1f mm / %.2f deg; board mean at base %s' % (label, pm, om, np.round(mean * 1000, 1).tolist()))
    for (x, y, z, pit), p in zip(POSES, P):
        e = (p - mean) * 1000
        print('   pose (%.2f,%+.2f,%.2f,p%d): err x %+6.1f  y %+6.1f  z %+6.1f  |%5.1f| mm' % (x, y, z, pit, e[0], e[1], e[2], np.linalg.norm(e)))
    for pit in sorted(set(q[3] for q in POSES)):
        idx = [i for i, q in enumerate(POSES) if q[3] == pit]
        sub = P[idx]
        print('   pitch %2d: %d views, spread within group %.1f mm, group mean offset from all %s mm' %
              (pit, len(idx), float(np.sqrt(np.mean(np.sum((sub - sub.mean(0)) ** 2, axis=1)))) * 1000,
               np.round((sub.mean(0) - mean) * 1000, 1).tolist()))
