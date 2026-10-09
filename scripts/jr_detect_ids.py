#!/usr/bin/env python3
# encoding: utf-8
# Unified INSTANCE detection (color backend). Detects EVERY cube (not just the
# largest per color), assigns stable IDs (red1, red2, green1, ...), excludes the
# white place-paper region (its printed pattern would false-trigger), and outputs
# an annotated image + a list of instances. This is the fusion layer's output
# format; a YOLO backend can later emit the same {id,label,u,v,box,angle,depth}.
#   python3 jr_detect_ids.py [rgb.png] [depth.npy]
import sys
import numpy as np
import cv2

RGB = sys.argv[1] if len(sys.argv) > 1 else 'scene_for_ids.png'
DEPTH = sys.argv[2] if len(sys.argv) > 2 else 'scene_for_ids_depth.npy'
MIN_AREA = 300.0       # reject noise / small pattern specks
PAPER_MIN = 4000.0     # the place paper is a large white blob

COLORS = {
    'red':    [((0, 110, 70), (10, 255, 255)), ((160, 110, 70), (180, 255, 255))],
    'orange': [((10, 120, 90), (20, 255, 255))],
    'yellow': [((22, 110, 110), (33, 255, 255))],
    'green':  [((38, 70, 50), (85, 255, 255))],
    'cyan':   [((87, 90, 80), (99, 255, 255))],
    'blue':   [((100, 110, 60), (128, 255, 255))],
    'purple': [((131, 70, 60), (158, 255, 255))],
}
DRAW = {'red': (0, 0, 255), 'orange': (0, 128, 255), 'yellow': (0, 220, 220),
        'green': (0, 200, 0), 'cyan': (200, 200, 0), 'blue': (255, 0, 0),
        'purple': (200, 0, 200)}


def color_mask(hsv, color):
    m = None
    for lo, hi in COLORS[color]:
        part = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = part if m is None else (m | part)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


def angle_of(c):
    if len(c) < 5:
        return 0.0
    (_, _), (w, h), ang = cv2.minAreaRect(c)
    if w < h:
        ang += 90.0
    return ((ang + 45.0) % 90.0) - 45.0   # nearest-face alignment, [-45,45)


def depth_m(depth, u, v, win=7):
    if depth is None:
        return 0.0
    h, w = depth.shape
    p = depth[max(0, v - win):min(h, v + win + 1), max(0, u - win):min(w, u + win + 1)].astype(np.float32)
    vals = p[(p > 0) & (p < 10000)]
    return float(np.percentile(vals, 25)) / 1000.0 if vals.size >= 8 else 0.0


TARGETS = ('red', 'green', 'blue')   # cube colors present (avoids blue->cyan false hits)


def paper_box(hsv):
    # the white place paper = largest VERY bright, low-saturation region (tight, so the
    # light wood floor is not mistaken for paper)
    wm = cv2.inRange(hsv, np.array([0, 0, 205]), np.array([180, 30, 255]))
    wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cnts, _ = cv2.findContours(wm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < PAPER_MIN:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return (x, y, x + w, y + h)


def inside(u, v, box, pad=6):
    return box and (box[0] - pad <= u <= box[2] + pad) and (box[1] - pad <= v <= box[3] + pad)


def detect_instances(rgb, depth=None):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    pbox = paper_box(hsv)
    insts = []
    for color in TARGETS:
        cnts, _ = cv2.findContours(color_mask(hsv, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w // 2, y + h // 2
            if inside(u, v, pbox):
                continue   # skip the printed pattern on the place paper
            insts.append({'color': color, 'u': u, 'v': v, 'area': a,
                          'box': (x, y, x + w, y + h), 'angle': angle_of(c),
                          'depth': depth_m(depth, u, v)})
    # assign per-color IDs, nearest first (larger v = closer at floor pose)
    out = []
    for color in TARGETS:
        grp = [d for d in insts if d['color'] == color]
        grp.sort(key=lambda d: -d['v'])
        for i, d in enumerate(grp):
            d['id'] = '%s%d' % (color, i + 1)
            out.append(d)
    return out, pbox


def main():
    rgb = cv2.imread(RGB)
    depth = None
    try:
        depth = np.load(DEPTH)
    except Exception:
        pass
    insts, pbox = detect_instances(rgb, depth)
    vis = rgb.copy()
    if pbox:
        cv2.rectangle(vis, pbox[:2], pbox[2:], (255, 255, 255), 2)
        cv2.putText(vis, 'PAPER', (pbox[0], pbox[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    print('detected %d cubes (paper=%s)' % (len(insts), pbox is not None))
    for d in insts:
        col = DRAW.get(d['color'], (255, 255, 255))
        cv2.rectangle(vis, d['box'][:2], d['box'][2:], col, 2)
        cv2.putText(vis, d['id'], (d['box'][0], d['box'][1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        cv2.drawMarker(vis, (d['u'], d['v']), col, cv2.MARKER_CROSS, 10, 2)
        print('  %-8s (%d,%d) area=%.0f angle=%+.0f depth=%.3f' %
              (d['id'], d['u'], d['v'], d['area'], d['angle'], d['depth']))
    cv2.imwrite('/home/fyh/ids_annotated.png', vis)
    print('wrote ids_annotated.png')


if __name__ == '__main__':
    main()
