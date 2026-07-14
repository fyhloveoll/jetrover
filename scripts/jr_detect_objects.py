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
import os
import sys
import numpy as np
import cv2

RGB = sys.argv[1] if len(sys.argv) > 1 else 'scene_for_ids.png'
DEPTH = sys.argv[2] if len(sys.argv) > 2 else 'scene_for_ids_depth.npy'
KF = sys.argv[3] if len(sys.argv) > 3 else 'scene_for_ids_K.npy'
ABOVE = float(os.environ.get('JR_MIN_H', '0.012'))  # metres above floor to count as object
                       # (flat things -- a lying pen/usb stick ~1cm -- skate under 0.012)
MIN_AREA = 250.0       # min blob area (px) to be an instance

# ---- optional YOLO semantic labels (fusion) ----
# Depth segmentation OWNS the geometry (footprint/height/width/angle); YOLO only NAMES
# the instances it overlaps (banana1, bottle1...). Unnamed ones keep their color label,
# so grasping still works on anything -- YOLO adds semantics, never gates detection.
import os as _os
YOLO_ON = _os.environ.get('JR_YOLO', '0') == '1'
YOLO_CONF = float(_os.environ.get('JR_YOLO_CONF', '0.35'))
_YOLO = None
_LBL_MEMO = {}   # (u//40, v//40) -> [label, conf, ttl] sticky-name cache (see yolo_label)


def _yolo():
    global _YOLO
    if _YOLO is None:
        from ultralytics import YOLO as _Y
        for p in (_os.environ.get('JR_YOLO_MODEL', ''),
                  _os.path.expanduser('~/jetrover_ws/yolo11m.pt'),
                  _os.path.expanduser('~/yolo11m.pt'),
                  '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'):
            if p and _os.path.exists(p):
                _YOLO = _Y(p)
                print('[YOLO] loaded %s' % p)
                break
        if _YOLO is None:
            raise FileNotFoundError('no YOLO model found (set JR_YOLO_MODEL)')
        # open-vocabulary (YOLO-World/YOLOE): the class MENU comes from a runtime
        # string, not training -- "pen, usb flash drive, cube" just works. This is
        # the M9/LLM hook: whatever the language layer asks for becomes a class.
        cls = [c.strip() for c in _os.environ.get('JR_YOLO_CLASSES', '').split(',') if c.strip()]
        if cls:
            if hasattr(_YOLO, 'set_classes'):
                _YOLO.set_classes(cls)
                print('[YOLO] open-vocab classes: %s' % cls)
            else:
                print('[YOLO] model has no set_classes (not open-vocab); ignoring JR_YOLO_CLASSES')
    return _YOLO


def yolo_label(rgb, insts):
    # attach class names to depth-seg instances whose center falls in a YOLO box;
    # tightest (smallest) containing box wins -- big boxes don't steal small objects
    try:
        res = _yolo()(rgb, conf=YOLO_CONF, verbose=False)[0]
    except Exception as e:
        print('[YOLO] disabled: %s' % e)
        return
    boxes = []
    for b in res.boxes:
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
        boxes.append((res.names[int(b.cls)], float(b.conf), x1, y1, x2, y2,
                      (x2 - x1) * (y2 - y1)))
    for d in insts:
        bx1, by1, bx2, by2 = d['box']
        iarea = max(1.0, float(bx2 - bx1) * float(by2 - by1))
        best = None
        for name, conf, x1, y1, x2, y2, area in boxes:
            if not (x1 - 4 <= d['u'] <= x2 + 4 and y1 - 4 <= d['v'] <= y2 + 4):
                continue
            if area > 12.0 * iarea:
                continue    # a big box (e.g. a coke can's) must not swallow small neighbours
            ov_w = min(bx2, x2) - max(bx1, x1)
            ov_h = min(by2, y2) - max(by1, y1)
            if ov_w <= 0 or ov_h <= 0 or (ov_w * ov_h) < 0.5 * iarea:
                continue    # require the yolo box to actually cover the instance
            if best is None or area < best[2]:
                best = (name, conf, area)
        if best is not None:
            d['label'] = best[0].replace(' ', '_')
            d['cls_conf'] = best[1]
            _LBL_MEMO[(d['u'] // 40, d['v'] // 40)] = [d['label'], best[1], 12]
        else:
            # STICKY NAMES: near-threshold open-vocab confidences flicker frame to
            # frame; an instance named at ~this spot recently keeps its name
            for du in (0, -1, 1):
                for dv in (0, -1, 1):
                    m = _LBL_MEMO.get((d['u'] // 40 + du, d['v'] // 40 + dv))
                    if m and m[2] > 0:
                        d['label'] = m[0]; d['cls_conf'] = m[1]; m[2] -= 1
                        break
                else:
                    continue
                break
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
        ang_long = 0.0   # long-axis direction, mod 180 -- NOT collapsed to +-45
        elong = 1.0      # long/short side ratio (1.0 = square-ish)
        rect = None
        if cnts and len(cnts[0]) >= 5:
            rect = cv2.minAreaRect(cnts[0])
            (_, _), (rw, rh), a = rect
            if rw < rh:
                a += 90.0
            # +-45 collapse is fine for squares (90deg symmetric) but loses WHICH axis
            # is long -- a pen grasped on the wrong 90deg branch closes along its body
            ang = ((a + 45.0) % 90.0) - 45.0
            ang_long = ((a + 90.0) % 180.0) - 90.0
            elong = max(rw, rh) / max(1.0, min(rw, rh))
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
                      'angle': float(ang), 'angle_long': float(ang_long),
                      'elong': float(elong), 'rect': rect, 'depth': dep, 'width_m': width_m,
                      'cnt': (cnts[0] if cnts else None)})
    if YOLO_ON and insts:
        yolo_label(rgb, insts)   # semantic names from YOLO; geometry stays depth-seg's
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
