#!/usr/bin/env python3
"""crop_vis.py -- crop the cube region of yaw_calib *_vis.png frames and upscale 4x.
Usage: python3 crop_vis.py <vis.png> [more...]  -> writes <name>_zoom.png next to each"""
import sys
import cv2
import numpy as np

for p in sys.argv[1:]:
    im = cv2.imread(p)
    # locate the red top-face overlay to centre the crop
    r = (im[:, :, 2] > 200) & (im[:, :, 1] < 60) & (im[:, :, 0] < 60)
    ys, xs = np.nonzero(r)
    if xs.size:
        cx, cy = int(xs.mean()), int(ys.mean())
    else:
        cx, cy = im.shape[1] // 2, im.shape[0] // 2
    x0, y0 = max(0, cx - 60), max(0, cy - 60)
    crop = im[y0:y0 + 120, x0:x0 + 120]
    big = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    out = p[:-4] + '_zoom.png'
    cv2.imwrite(out, big)
    print(out)
