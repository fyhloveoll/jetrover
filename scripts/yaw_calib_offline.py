#!/usr/bin/env python3
# encoding: utf-8
"""yaw_calib_offline.py -- re-analyse frames dumped by yaw_calib.py (no robot motion, no ROS).

For each dumped frame: run the detector, report the target blob (area/box/angles) and
compute a THIRD angle estimate, `top`, from the cube's TOP FACE only:
  pixels inside the blob whose height above the fitted floor plane is within a band just
  below the blob's max height -> back-projected to 3D -> projected onto the floor plane in
  the ARM frame -> minAreaRect.  The silhouette (what `floor` uses) includes the cube's
  visible side faces, which stretch the blob along the viewing direction and bias the
  angle toward 0; the top face does not.
Writes <stem>_vis.png with contour (green), silhouette rect (blue) and top-face pixels (red).

Usage (on the robot, in ~/jetrover_ws):  python3 yaw_calib_offline.py [glob_pattern] [--band 0.008]
"""
import glob
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser('~/jetrover_ws'))
import jr_detect_objects as det          # noqa: E402
from jr_grasp_all import HAND2CAM        # noqa: E402


def wrap90(a):
    return ((a + 45.0) % 90.0) - 45.0


def rect_angle_xy(pts):
    """minAreaRect angle (deg, wrapped to +-45) of Nx2 float points"""
    (_, _), (rw, rh), ang = cv2.minAreaRect(np.asarray(pts, np.float32))
    if rw < rh:
        ang += 90.0
    return wrap90(ang)


def quad_mean_angle(segs):
    """4-fold-symmetric (mod 90) length-weighted circular mean of segment directions.
    segs: list of (x0, y0, x1, y1). Returns deg in [-45, 45) or nan."""
    s = c = 0.0
    for x0, y0, x1, y1 in segs:
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 1e-6:
            continue
        a4 = 4.0 * math.atan2(dy, dx)
        s += L * math.sin(a4); c += L * math.cos(a4)
    if s == 0.0 and c == 0.0:
        return float('nan')
    return wrap90(math.degrees(math.atan2(s, c)) / 4.0)


def robust_quad_mean(segs, thr=10.0, iters=2):
    """quad_mean_angle with iterative rejection of segments farther than thr (deg, mod 90)
    from the current mean -- drops the cube's vertical side edges (near-vertical in the
    image) that otherwise pull the mean toward 0. Returns (angle, n_kept, n_total)."""
    keep = list(segs)
    m = quad_mean_angle(keep)
    for _ in range(iters):
        if math.isnan(m):
            break
        nk = [sg for sg in keep if abs(wrap90(math.degrees(math.atan2(sg[3] - sg[1], sg[2] - sg[0])) - m)) <= thr]
        if len(nk) < 2 or len(nk) == len(keep):
            keep = nk if len(nk) >= 2 else keep
            break
        keep = nk
        m = quad_mean_angle(keep)
    return m, len(keep), len(segs)


def backproject(u, v, n, dd, h, fx, fy, cx, cy, T):
    """pixel (u,v) -> point on the plane parallel to the floor at height h (m) -> arm XY"""
    dirb = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    den = n @ dirb
    if abs(den) < 1e-6:
        return None
    c = dirb * (-(dd + h) / den)
    a = T @ np.array([c[0] - 0.01, c[1], c[2], 1.0])
    return a[0], a[1]


def poly_angle(cnt, n, dd, fx, fy, cx, cy, T, eps=1.5):
    """depth-contour method: smooth the contour with approxPolyDP, back-project the
    polygon vertices onto the floor, 4-fold mean of the polygon edge directions in arm XY"""
    poly = cv2.approxPolyDP(cnt, eps, True).reshape(-1, 2)
    if len(poly) < 4:
        return float('nan')
    pts = [backproject(float(u), float(v), n, dd, 0.0, fx, fy, cx, cy, T) for u, v in poly]
    pts = [p for p in pts if p is not None]
    segs = [(pts[i][0], pts[i][1], pts[(i + 1) % len(pts)][0], pts[(i + 1) % len(pts)][1]) for i in range(len(pts))]
    return quad_mean_angle(segs)


def floor_frame(n):
    """4x4 transform camera -> a frame whose z is the floor normal (pointing to the camera
    side) and whose x is the camera's forward axis projected onto the floor. Angles in its
    XY differ from the arm frame only by a constant offset -- used to test whether the
    hand-eye transform T shrinks angles."""
    z = np.asarray(n, np.float64); z = z / np.linalg.norm(z)
    if z[2] > 0:          # camera looks down (+z): the normal facing the camera has z<0
        z = -z
    f = np.array([0.0, 0.0, 1.0])
    x = f - (f @ z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])          # rows = new axes in camera coords
    T = np.eye(4); T[:3, :3] = R
    return T


def edge_angle(rgb, cnt, n, dd, h_top, fx, fy, cx, cy, T, vis=None, top_mask=None):
    """rgb-edge method: Canny inside the dilated blob, HoughP segments, each segment
    back-projected onto the plane at the cube's top height, 4-fold length-weighted mean.
    Returns dict: raw, robust, top-only (segments whose midpoint lies in top_mask),
    floor-frame (no hand-eye T), n_keep, n_total."""
    mask = np.zeros(rgb.shape[:2], np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    mask = cv2.dilate(mask, np.ones((7, 7), np.uint8))
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 160)
    edges[mask == 0] = 0
    lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=8, minLineLength=8, maxLineGap=3)
    res = {'raw': float('nan'), 'rob': float('nan'), 'top': float('nan'), 'flr': float('nan'), 'nk': 0, 'nt': 0}
    if lines is None:
        return res
    TF = floor_frame(n)
    segs, segs_top, segs_flr = [], [], []
    tm = cv2.dilate(top_mask, np.ones((5, 5), np.uint8)) if top_mask is not None else None
    for x0, y0, x1, y1 in lines[:, 0, :]:
        p0 = backproject(float(x0), float(y0), n, dd, h_top, fx, fy, cx, cy, T)
        p1 = backproject(float(x1), float(y1), n, dd, h_top, fx, fy, cx, cy, T)
        if p0 is None or p1 is None:
            continue
        segs.append((p0[0], p0[1], p1[0], p1[1]))
        q0 = backproject(float(x0), float(y0), n, dd, h_top, fx, fy, cx, cy, TF)
        q1 = backproject(float(x1), float(y1), n, dd, h_top, fx, fy, cx, cy, TF)
        segs_flr.append((q0[0], q0[1], q1[0], q1[1]))
        mx, my = int((x0 + x1) / 2), int((y0 + y1) / 2)
        on_top = tm is not None and 0 <= my < tm.shape[0] and 0 <= mx < tm.shape[1] and tm[my, mx] > 0
        if on_top:
            segs_top.append(segs[-1])
        if vis is not None:
            cv2.line(vis, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 255) if on_top else (255, 0, 255), 1)
    res['raw'] = quad_mean_angle(segs)
    res['rob'], res['nk'], res['nt'] = robust_quad_mean(segs)
    res['top'] = quad_mean_angle(segs_top) if segs_top else float('nan')
    res['flr'] = quad_mean_angle(segs_flr)
    return res


def analyse(stem, band):
    depth = np.load(stem + '_depth.npy')
    rgb = cv2.imread(stem + '_rgb.png')
    K = np.load(stem + '_K.npy').tolist()
    ep = np.load(stem + '_ep.npy')
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    z = depth.astype(np.float32) / 1000.0
    fl = det.fit_floor(z, fx, fy, cx, cy)
    insts = det.detect(rgb, depth, K)
    if not insts:
        print('%s: no detections' % os.path.basename(stem)); return None
    inst = min(insts, key=lambda o: (o['u'] - cx) ** 2 + (o['v'] - cy) ** 2)
    cnt = inst.get('cnt')
    out = {'stem': os.path.basename(stem), 'area': inst['area'], 'box': inst['box'],
           'image': float(inst['angle']), 'floor': float('nan'), 'top': float('nan'),
           'poly': float('nan'), 'edge': float('nan'), 'edgeR': float('nan'), 'edgeT': float('nan'),
           'edgeF': float('nan'), 'n_seg': 0, 'n_keep': 0,
           'h_max': float('nan'), 'n_top': 0}
    vis = rgb.copy()
    if cnt is not None:
        cv2.drawContours(vis, [cnt], -1, (0, 255, 0), 1)
    if fl is None or cnt is None:
        print('%s: no floor fit / no contour' % out['stem'])
        cv2.imwrite(stem + '_vis.png', vis); return out
    n, dd = fl
    n = np.asarray(n, np.float64)
    T = ep @ HAND2CAM

    # --- silhouette (== floor_angle_arm) ---
    pts = []
    for p in cnt[:: max(1, len(cnt) // 48)]:
        u, v = float(p[0][0]), float(p[0][1])
        dirb = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        den = n @ dirb
        if abs(den) < 1e-6:
            continue
        c = dirb * (-dd / den)
        a = T @ np.array([c[0] - 0.01, c[1], c[2], 1.0])
        pts.append([a[0], a[1]])
    if len(pts) >= 8:
        out['floor'] = rect_angle_xy(pts)
        box = cv2.boxPoints(cv2.minAreaRect(np.asarray(cnt[:: max(1, len(cnt) // 48)], np.float32).reshape(-1, 2)))
        cv2.polylines(vis, [np.int32(box)], True, (255, 0, 0), 1)

    # --- top face: height band below the blob's max height ---
    mask = np.zeros(depth.shape, np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    vs, us = np.nonzero(mask)
    zz = z[vs, us]
    ok = (zz > 0.05) & (zz < 2.0)
    us, vs, zz = us[ok], vs[ok], zz[ok]
    if zz.size < 20:
        print('%s: too few depth pixels in blob' % out['stem'])
        cv2.imwrite(stem + '_vis.png', vis); return out
    P = np.stack([(us - cx) / fx * zz, (vs - cy) / fy * zz, zz], axis=1)   # camera frame
    s = P @ n + dd                       # signed distance to floor plane (sign unknown)
    if np.median(s) < 0:
        s = -s
    h_max = float(np.percentile(s, 95))
    sel = (s > h_max - band) & (s < h_max + 0.004)
    out['h_max'] = h_max; out['n_top'] = int(sel.sum())
    top_mask = np.zeros(depth.shape, np.uint8)
    top_mask[vs[sel], us[sel]] = 255
    if sel.sum() >= 20:
        # project the selected 3D points onto the floor plane (drop the normal component),
        # then into the arm frame XY
        Pt = P[sel] - np.outer(s[sel], n) if np.median(P @ n + dd) < 0 else P[sel] - np.outer(s[sel], -n)
        A = (T @ np.c_[Pt[:, 0] - 0.01, Pt[:, 1], Pt[:, 2], np.ones(len(Pt))].T).T
        out['top'] = rect_angle_xy(A[:, :2])
        vis[vs[sel], us[sel]] = (0, 0, 255)
    # backproject() intersects the plane  n.P + (dd + h) = 0.  Flip (n, dd) so that cube-top
    # points give n.P + dd = +h_max; the top plane is then h = -h_max.
    n_sig, dd_sig = (n, dd) if np.median(P @ n + dd) > 0 else (-n, -dd)
    out['poly'] = poly_angle(cnt, n_sig, dd_sig, fx, fy, cx, cy, T)
    e = edge_angle(rgb, cnt, n_sig, dd_sig, -h_max, fx, fy, cx, cy, T, vis, top_mask)
    out['edge'], out['edgeR'], out['edgeT'], out['edgeF'] = e['raw'], e['rob'], e['top'], e['flr']
    out['n_keep'], out['n_seg'] = e['nk'], e['nt']
    cv2.imwrite(stem + '_vis.png', vis)
    return out


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('--') else '*'
    band = 0.008
    if '--band' in sys.argv:
        band = float(sys.argv[sys.argv.index('--band') + 1])
    d = os.path.expanduser('~/jetrover_ws/yaw_calib_frames')
    stems = sorted(p[:-8] for p in glob.glob(os.path.join(d, pattern + '_rgb.png')))
    if not stems:
        print('no frames matching', pattern); return
    print('%-26s %5s %7s %7s %7s %7s %7s %7s %7s %5s %6s' %
          ('frame', 'area', 'image', 'floor', 'top', 'edge', 'edgeR', 'edgeT', 'edgeF', 'keep', 'h_max'))
    rows = []
    for st in stems:
        r = analyse(st, band)
        if r is None:
            continue
        rows.append(r)
        print('%-26s %5.0f %+7.1f %+7.1f %+7.1f %+7.1f %+7.1f %+7.1f %+7.1f %2d/%-2d %6.3f' %
              (r['stem'], r['area'], r['image'], r['floor'], r['top'], r['edge'], r['edgeR'], r['edgeT'],
               r['edgeF'], r['n_keep'], r['n_seg'], r['h_max']))
    # per true-angle summary (true angle parsed from the file name: <ts>_<+dd>_<tag>_<i>)
    by = {}
    for r in rows:
        parts = r['stem'].split('_')
        try:
            t = int(parts[1]); tag = parts[2]
        except (IndexError, ValueError):
            continue
        by.setdefault((t, tag), []).append(r)
    if by:
        print('\n%6s %-8s %9s %9s %9s %9s %9s %9s %9s   (mean err / sd = measured - true; edgeF = raw angle, offset unknown)' %
              ('true', 'tag', 'image', 'floor', 'top', 'edge', 'edgeR', 'edgeT', 'edgeF'))
        for (t, tag), rs in sorted(by.items()):
            def merr(k, raw=False):
                v = [wrap90(r[k] - (0 if raw else t)) for r in rs if not math.isnan(r[k])]
                return (np.mean(v), np.std(v)) if v else (float('nan'), float('nan'))
            cells = ['%+5.1f/%3.1f' % merr(k) for k in ('image', 'floor', 'top', 'edge', 'edgeR', 'edgeT')]
            cells.append('%+5.1f/%3.1f' % merr('edgeF', raw=True))
            print('%+6d %-8s %s   n=%d' % (t, tag, ' '.join('%9s' % c for c in cells), len(rs)))


if __name__ == '__main__':
    main()
