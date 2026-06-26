#!/usr/bin/env python3
# encoding: utf-8
# Class-agnostic scene segmentation for JetRover (M4 interactive grasp): find
# "anything sitting on the floor" by removing the dominant floor plane from the
# depth image (RANSAC) and clustering what sticks up. No model, no classes -- a
# red cube, a screwdriver, a bottle are all just blobs above the floor.
# Each blob gets a stable-ish id; downstream the click/command picks an id and
# the M5 grasp pipeline grasps it. GPU-free.
import numpy as np
import cv2


def backproject(depth, K, step=2):
    # depth uint16 mm -> (Nx3 metres, u[], v[]) at every `step`-th pixel
    h, w = depth.shape
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    us, vs = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
    z = depth[vs, us].astype(np.float32) / 1000.0
    m = z > 0
    u, v, z = us[m], vs[m], z[m]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1), u, v


def fit_plane_ransac(pts, iters=250, thresh=0.012):
    n = len(pts)
    best_in = None
    best = None
    for _ in range(iters):
        idx = np.random.choice(n, 3, replace=False)
        p = pts[idx]
        nrm = np.cross(p[1] - p[0], p[2] - p[0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        d = -nrm.dot(p[0])
        inl = np.abs(pts.dot(nrm) + d) < thresh
        if best_in is None or inl.sum() > best_in.sum():
            best_in, best = inl, (nrm, d)
    return best, best_in


def _bbox_depth_m(depth, bbox):
    x1, y1, x2, y2 = bbox
    patch = depth[y1:y2, x1:x2].astype(np.float32)
    vals = patch[(patch > 0) & (patch < 10000)]
    return float(np.median(vals)) / 1000.0 if vals.size else 0.0


def segment(depth, K, step=2, floor_thresh=0.012, above=0.015,
            min_area=250, max_area=20000, max_area_frac=0.35, max_dist=0.7,
            max_width_frac=0.6, top_margin=4):
    """Return (objects, object_mask_fullres). objects = list of dicts
    {id,u,v,bbox,area,dist} sorted near->far. Floor removed via RANSAC; far
    background/furniture dropped by distance + width filters."""
    h, w = depth.shape
    pts, u, v = backproject(depth, K, step)
    if len(pts) < 100:
        return [], np.zeros((h, w), np.uint8)

    (nrm, d), _ = fit_plane_ransac(pts, thresh=floor_thresh)
    signed = pts.dot(nrm) + d           # 0 on floor
    cam_side = np.sign(d) if d != 0 else 1.0
    obj = (signed * cam_side) > above   # points >`above` m toward the camera = above floor

    # paint object points into a low-res mask, upscale to full res
    mh, mw = (h + step - 1) // step, (w + step - 1) // step
    mask = np.zeros((mh, mw), np.uint8)
    mask[v[obj] // step, u[obj] // step] = 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # merge split blobs
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

    nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    for i in range(1, nlab):
        x, y, bw, bh, area = stats[i]
        if area < min_area or area > max_area or bw * bh > max_area_frac * w * h:
            continue                                    # too small / too big = noise or background
        if bw > max_width_frac * w or y <= top_margin:  # spans width / touches top = far bg/furniture
            continue
        bbox = (x, y, x + bw, y + bh)
        dist = _bbox_depth_m(depth, bbox)
        if dist <= 0 or dist > max_dist:                # far furniture/background -> drop
            continue
        blobs.append({'u': int(cent[i][0]), 'v': int(cent[i][1]),
                      'bbox': bbox, 'area': int(area), 'dist': round(dist, 3)})
    blobs.sort(key=lambda b: b['dist'])                 # nearest first
    for i, b in enumerate(blobs):
        b['id'] = i + 1
    return blobs, mask


def annotate(bgr, blobs):
    out = bgr.copy()
    for b in blobs:
        x1, y1, x2, y2 = b['bbox']
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.circle(out, (b['u'], b['v']), 4, (0, 0, 255), -1)
        cv2.putText(out, '#%d' % b['id'], (x1, max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2, cv2.LINE_AA)
    return out
