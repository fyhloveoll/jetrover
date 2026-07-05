#!/usr/bin/env python3
# encoding: utf-8
# OFFLINE grasp-geometry diagnostic. Loads a saved RGB+depth+K frame (NO camera,
# NO arm motion, NO serial -> zero hang risk), runs YOLO, and compares grasp-point
# candidates so we can see why the gripper landed on the can EDGE instead of center.
import numpy as np
import cv2
from ultralytics import YOLO

RGB = '/home/ubuntu/jetrover_ws/cap_rgb.png'
DEP = '/home/ubuntu/jetrover_ws/cap_depth.npy'
KF  = '/home/ubuntu/jetrover_ws/cap_K.npy'
MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
TARGETS = {'bottle', 'cup', 'wine glass'}


def ray(u, v, z, fx, fy, cx, cy):
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def patch_pct(d, u, v, win, pct):
    h, w = d.shape
    p = d[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1].astype(np.float32)
    vals = p[(p > 0) & (p < 10000)]
    return (float(np.percentile(vals, pct)) / 1000.0) if vals.size >= 10 else 0.0


def fit_floor(d, fx, fy, cx, cy, box):
    # sample valid depth OUTSIDE the can box, build 3D points, RANSAC a plane
    h, w = d.shape
    x1, y1, x2, y2 = box
    ys, xs = np.where((d > 0) & (d < 10000))
    inbox = (xs >= x1) & (xs <= x2) & (ys >= y1) & (ys <= y2)
    xs, ys = xs[~inbox], ys[~inbox]
    idx = np.random.choice(xs.size, min(4000, xs.size), replace=False)
    xs, ys = xs[idx], ys[idx]
    z = d[ys, xs].astype(np.float32) / 1000.0
    P = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)
    best, bestn = None, 0
    rng = np.random.default_rng(0)
    for _ in range(200):
        s = P[rng.choice(P.shape[0], 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        ln = np.linalg.norm(n)
        if ln < 1e-6:
            continue
        n = n / ln
        dd = -n @ s[0]
        err = np.abs(P @ n + dd)
        cnt = int((err < 0.01).sum())
        if cnt > bestn:
            bestn, best = cnt, (n, dd)
    n, dd = best
    if n[1] > 0:  # make normal point "up" (-y in camera optical is up)
        n, dd = -n, -dd
    return n, dd, bestn, P.shape[0]


def main():
    d = np.load(DEP).astype(np.float32)
    K = np.load(KF)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    img = cv2.imread(RGB)
    res = YOLO(MODEL)(img, conf=0.25, verbose=False)[0]
    box = None
    for b in res.boxes:
        if res.names[int(b.cls)] in TARGETS:
            box = [int(v) for v in b.xyxy[0]]
            print('detect %s conf=%.2f box=%s' % (res.names[int(b.cls)], float(b.conf), box))
            break
    if box is None:
        print('no target'); return
    x1, y1, x2, y2 = box
    uc, vc = (x1 + x2) // 2, (y1 + y2) // 2          # box center (current method)
    ub, vb = (x1 + x2) // 2, y2                       # box bottom-center (footprint)
    bw, bh = x2 - x1, y2 - y1
    print('box wxh=%dx%d  center=(%d,%d)  bottom=(%d,%d)' % (bw, bh, uc, vc, ub, vb))

    z_center = patch_pct(d, uc, vc, 25, 25)
    print('CENTER depth p25(win25) = %.3f m' % z_center)

    n, dd, inl, tot = fit_floor(d, fx, fy, cx, cy, box)
    print('floor plane n=%s d=%.3f  inliers=%d/%d' % (np.round(n, 3).tolist(), dd, inl, tot))

    # footprint: ray through bottom-center pixel, intersect floor plane
    dir_b = ray(ub, vb, 1.0, fx, fy, cx, cy)
    t = -dd / (n @ dir_b)
    foot = dir_b * t
    print('FOOTPRINT (cam) xyz = %s  (dist=%.3f)' % (np.round(foot, 3).tolist(), np.linalg.norm(foot)))

    # current method 3D point (near-surface ray)
    cur = ray(uc, vc, z_center + 0.03, fx, fy, cx, cy)
    print('CURRENT grasp pt (cam) xyz = %s' % np.round(cur, 3).tolist())

    # can geometry: top via box-top pixel projected to footprint x,y, height = |foot - topplane|
    # estimate can height from box pixel height at footprint depth
    can_h = bh * foot[2] / fy
    print('estimated can height ~ %.3f m (box %dpx @ z=%.3f)' % (can_h, bh, foot[2]))

    # offset of current point from footprint axis (how far off-center, in floor plane)
    # project both to floor plane normal-removed
    def planar(p):
        return p - (n @ p + dd) * n
    off = np.linalg.norm(planar(cur) - planar(foot))
    print('CURRENT pt is %.3f m off the footprint axis (can radius ~0.027)' % off)

    vis = img.copy()
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(vis, (uc, vc), 5, (0, 0, 255), -1)    # red = current (box center)
    cv2.circle(vis, (ub, vb), 5, (255, 0, 0), -1)    # blue = footprint
    cv2.imwrite('/home/ubuntu/jetrover_ws/grasp_diag.png', vis)
    print('wrote grasp_diag.png (green=box, red=current center, blue=footprint)')


if __name__ == '__main__':
    main()
