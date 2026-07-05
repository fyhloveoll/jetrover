#!/usr/bin/env python3
# encoding: utf-8
# OBJECT-AGNOSTIC instance detection via depth floor-plane segmentation.
# Detects ANY object resting on the floor -- no hardcoded colors or classes. Flat
# things (the place paper, floor markings) are not "above the floor" so they are
# auto-excluded. Each instance gets an ID + an auto-derived color LABEL (a label,
# not a detection gate, so a new color needs ZERO code change). Output is the unified
# Detection format {id,label,box,centroid,angle,depth,area}; a YOLO backend can emit
# the same format and the grasp layer (grasp-by-id) never changes.
#   python3 jr_detect_objects.py [rgb.png] [depth.npy] [K.npy]
import sys
import numpy as np
import cv2

RGB = sys.argv[1] if len(sys.argv) > 1 else 'scene_for_ids.png'
DEPTH = sys.argv[2] if len(sys.argv) > 2 else 'scene_for_ids_depth.npy'
KF = sys.argv[3] if len(sys.argv) > 3 else 'scene_for_ids_K.npy'
ABOVE = 0.012          # metres above the floor plane to count as an object
MIN_AREA = 250.0       # min blob area (px) to be an instance
# hue(0-180) -> name, only to LABEL a detected object (never gates detection)
HUE_NAMES = [(10, 'red'), (22, 'orange'), (33, 'yellow'), (88, 'green'),
             (135, 'blue'), (160, 'purple'), (180, 'red')]


def fit_floor(z, fx, fy, cx, cy):
    h, w = z.shape
    ys, xs = np.where((z > 0) & (z < 10))
    if xs.size < 500:
        return None
    idx = np.random.default_rng(0).choice(xs.size, min(6000, xs.size), replace=False)
    xs, ys = xs[idx], ys[idx]
    zz = z[ys, xs]
    P = np.stack([(xs - cx) * zz / fx, (ys - cy) * zz / fy, zz], 1)
    best, bestn = None, 0
    rng = np.random.default_rng(1)
    for _ in range(300):
        s = P[rng.choice(P.shape[0], 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        ln = np.linalg.norm(n)
        if ln < 1e-6:
            continue
        n = n / ln
        d = -n @ s[0]
        cnt = int((np.abs(P @ n + d) < 0.01).sum())
        if cnt > bestn:
            bestn, best = cnt, (n, d)
    return best


def color_label(rgb, mask):
    px = rgb[mask > 0]
    if px.size == 0:
        return 'object'
    hsv = cv2.cvtColor(px.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    sat = hsv[:, 1]
    strong = hsv[sat > 60]
    if strong.shape[0] < 0.15 * hsv.shape[0]:
        return 'object'   # low saturation -> unnamed (still detected!)
    hue = float(np.median(strong[:, 0]))
    for hmax, name in HUE_NAMES:
        if hue <= hmax:
            return name
    return 'object'


def detect(rgb, depth, K):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    z = depth.astype(np.float32) / 1000.0
    fl = fit_floor(z, fx, fy, cx, cy)
    if fl is None:
        return []
    n, d = fl
    h, w = z.shape
    us = np.tile(np.arange(w), (h, 1)).astype(np.float32)
    vs = np.tile(np.arange(h).reshape(-1, 1), (1, w)).astype(np.float32)
    dirx = (us - cx) / fx
    diry = (vs - cy) / fy
    denom = n[0] * dirx + n[1] * diry + n[2]            # n . ray_dir
    zfloor = np.where(np.abs(denom) > 1e-6, -d / denom, 0.0)   # floor depth per pixel
    fg = (z > 0.05) & (z < 1.5) & (zfloor > 0) & ((zfloor - z) > ABOVE)
    mask = (fg.astype(np.uint8)) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    try:
        cv2.imwrite('/home/fyh/fg_mask.png', mask)
    except Exception:
        pass
    nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    insts = []
    for i in range(1, nlab):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 90:
            continue   # hard noise floor; the size-vs-distance gate comes after depth is known
        x, y, bw, bh = stats[i, 0], stats[i, 1], stats[i, 2], stats[i, 3]
        cu, cv_ = int(cent[i][0]), int(cent[i][1])
        cm = (lab == i).astype(np.uint8)
        cnts, _ = cv2.findContours(cm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ang = 0.0
        rect = None
        if cnts and len(cnts[0]) >= 5:
            rect = cv2.minAreaRect(cnts[0])
            (_, _), (rw, rh), a = rect
            if rw < rh:
                a += 90.0
            ang = ((a + 45.0) % 90.0) - 45.0
        dep = float(np.median(z[(lab == i) & (z > 0)])) if np.any((lab == i) & (z > 0)) else 0.0
        # distance-scaled area gate: the same physical cube shrinks with 1/z^2 in pixels
        # (a 3cm cube at 0.7m is ~230px -- a fixed 250px gate made far cubes invisible
        # to the survey). Gate on the equivalent area at the 0.35m reference distance.
        if dep > 0 and area < MIN_AREA * (0.35 / max(dep, 0.2)) ** 2:
            continue
        if dep <= 0 and area < MIN_AREA:
            continue
        width_m = 0.0
        if rect is not None and dep > 0:
            (_, _), (rw2, rh2), _ = rect
            # -4px: the CLOSE morphology dilates the blob; negligible near (80px wide)
            # but it inflated a far 43mm cube (34px) to a false 51mm reading
            width_m = max(0.0, min(rw2, rh2) - 4.0) * dep / fx
        insts.append({'label': color_label(rgb, cm), 'u': cu, 'v': cv_,
                      'box': (x, y, x + bw, y + bh), 'area': float(area),
                      'angle': float(ang), 'rect': rect, 'depth': dep, 'width_m': width_m,
                      'cnt': (cnts[0] if cnts else None)})
    # IDs per label, nearest first
    out = []
    for lab_name in sorted(set(d['label'] for d in insts)):
        grp = sorted([d for d in insts if d['label'] == lab_name], key=lambda d: -d['v'])
        for i, dct in enumerate(grp):
            dct['id'] = '%s%d' % (lab_name, i + 1)
            out.append(dct)
    return out


def main():
    rgb = cv2.imread(RGB)
    depth = np.load(DEPTH)
    K = np.load(KF)
    insts = detect(rgb, depth, K)
    vis = rgb.copy()
    print('detected %d objects (floor-segmentation, color-agnostic)' % len(insts))
    for d in insts:
        cv2.rectangle(vis, d['box'][:2], d['box'][2:], (0, 255, 0), 2)
        cv2.putText(vis, d['id'], (d['box'][0], d['box'][1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.drawMarker(vis, (d['u'], d['v']), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
        print('  %-8s (%d,%d) area=%.0f angle=%+.0f depth=%.3f' %
              (d['id'], d['u'], d['v'], d['area'], d['angle'], d['depth']))
    cv2.imwrite('/home/fyh/objects_annotated.png', vis)
    print('wrote objects_annotated.png')


if __name__ == '__main__':
    main()
