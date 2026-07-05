#!/usr/bin/env python3
# encoding: utf-8
# M4.5 calibration fit (run on LAPTOP, numpy only). Pairs tape-measured cube
# positions with the computed arm-base coords (from calib_measure.py) and fits the
# error, to decide: constant offset (easy fix, kill JR_FWD) vs scale/skew (deeper).
#
# Fill DATA with one row per cube position:
#   (meas_forward_cm, meas_lateral_cm, raw_x_m, raw_y_m, foot_x_m, foot_y_m)
# meas_forward = distance from your fixed reference along robot's forward axis (cm)
# meas_lateral = left(+)/right(-) of the centerline (cm)
# raw_*/foot_* = printed by calib_measure.py (metres)
import numpy as np

# label, meas_fwd_cm, meas_lat_cm, raw_x, raw_y, foot_x, foot_y   <-- EDIT THIS
DATA = [
    # label, meas_fwd_cm, meas_lat_cm(left+/right-), raw_x, raw_y, foot_x, foot_y
    ('red',   42.5,  12.0, 0.3814,  0.1287, 0.3908,  0.1399),
    ('blue',  30.0, -13.5, 0.2619, -0.0920, 0.2699, -0.1106),
    ('green', 34.0,   0.0, 0.3027,  0.0210, 0.3105,  0.0211),
]


def fit_axis(meas_m, comp_m, name):
    meas = np.array(meas_m); comp = np.array(comp_m)
    # offset-only model: comp = meas + b
    b = float(np.mean(comp - meas))
    res_off = comp - meas - b
    # affine model: comp = a*meas + c
    A = np.vstack([meas, np.ones_like(meas)]).T
    (a, c), *_ = np.linalg.lstsq(A, comp, rcond=None)
    res_aff = comp - (a * meas + c)
    print('  [%s] offset-only: bias b=%+.3fm  resid RMS=%.3fm  (max %.3f)' %
          (name, b, float(np.sqrt(np.mean(res_off ** 2))), float(np.max(np.abs(res_off)))))
    print('       affine:      slope a=%.3f (ideal 1.0)  intercept c=%+.3fm  resid RMS=%.3fm' %
          (a, c, float(np.sqrt(np.mean(res_aff ** 2)))))
    return b, a, c


def report(rows, xi, yi, tag):
    print('\n=== %s vs measured ===' % tag)
    mf = [r[1] / 100.0 for r in rows]
    ml = [r[2] / 100.0 for r in rows]
    cx = [r[xi] for r in rows]
    cy = [r[yi] for r in rows]
    print(' forward (x):')
    bx, ax, _ = fit_axis(mf, cx, 'x')
    print(' lateral (y):')
    by, ay, _ = fit_axis(ml, cy, 'y')
    print(' per-point residual after offset-only (computed - measured - bias), metres:')
    for r in rows:
        ex = (r[xi] - r[1] / 100.0 - bx)
        ey = (r[yi] - r[2] / 100.0 - by)
        print('   %-4s  dx=%+.3f  dy=%+.3f' % (r[0], ex, ey))
    # verdict
    slope_ok = abs(ax - 1) < 0.08 and abs(ay - 1) < 0.08
    print(' verdict: %s' % (
        'constant offset (slope~1) -> apply bias correction x%+.3f y%+.3f, drop JR_FWD'
        % (bx, by) if slope_ok else
        'slope != 1 -> scale/skew error (depth scale / hand-eye / arm-sag), needs deeper fix'))


def main():
    if len(DATA) < 2:
        print('Need >=2 rows in DATA. Fill it from tape measurements + calib_measure output.')
        print('Recommended: 3-5 positions spanning near/far/left/right.')
        return
    print('points: %d' % len(DATA))
    report(DATA, 3, 4, 'RAW (box-center+depth)')
    report(DATA, 5, 6, 'FOOTPRINT (floor-plane)')
    print('\nNote: bias absorbs your reference-point offset; the RESID is the real error.')
    print('Small resid (<~1cm) + slope~1 => transform is good, bias is constant & fixable.')


if __name__ == '__main__':
    main()
