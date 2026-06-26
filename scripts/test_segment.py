#!/usr/bin/env python3
# encoding: utf-8
# Offline test of the class-agnostic scene segmenter on the captured sample
# frame in test_data/. No robot/ROS needed -- just numpy + opencv.
#   python3 scripts/test_segment.py
import os
import sys

import numpy as np
import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'jr_vision', 'jr_vision'))
from scene_segment import segment, annotate  # noqa: E402

DATA = os.path.join(ROOT, 'test_data')
depth = np.load(os.path.join(DATA, 'cap_depth.npy'))
K = list(np.load(os.path.join(DATA, 'cap_K.npy')))
rgb = cv2.imread(os.path.join(DATA, 'cap_rgb.png'))

blobs, mask = segment(depth, K)
print('objects: %d' % len(blobs))
for b in blobs:
    print('  #%d center=(%d,%d) dist=%.2fm area=%d bbox=%s'
          % (b['id'], b['u'], b['v'], b['dist'], b['area'], b['bbox']))
out = os.path.join(DATA, 'seg_out.png')
cv2.imwrite(out, annotate(rgb, blobs))
print('saved %s' % out)
