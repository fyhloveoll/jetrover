#!/usr/bin/env python3
# encoding: utf-8
# Color-segmentation detector backend for jr_vision (M4): find a colored object
# (e.g. a red/green/blue block) by HSV threshold + contour, returning the SAME
# unified Target as the YOLO backend so the M5 grasp pipeline is detector-agnostic.
#
# Solves the "YOLO/COCO can't see color cubes" coverage gap (cubes are not a COCO
# class). Deterministic, GPU-free; mirrors the vendor track_and_grab color tracker.
# Default HSV ranges are starting points -- TUNE on the robot's lighting (or load
# the vendor LAB color calibration). OpenCV HSV is H:0-179, S/V:0-255.
import cv2
import numpy as np

# red Hue wraps around 0/179 -> two segments. Each color = list of (lo, hi) HSV.
DEFAULT_HSV = {
    'red':   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 80, 50), (85, 255, 255))],
    'blue':  [((95, 120, 60), (130, 255, 255))],
}


class ColorDetector:
    def __init__(self, ranges=None, min_area=400, max_area_frac=0.4, open_k=3, close_k=5):
        self.ranges = ranges or DEFAULT_HSV
        self.min_area = min_area
        self.max_area_frac = max_area_frac   # reject blobs filling > this fraction of frame (noise)
        self.open_k = open_k
        self.close_k = close_k

    def colors(self):
        return list(self.ranges.keys())

    def mask(self, bgr, color):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = None
        for lo, hi in self.ranges[color]:
            part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            m = part if m is None else cv2.bitwise_or(m, part)
        if self.open_k:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((self.open_k, self.open_k), np.uint8))
        if self.close_k:
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((self.close_k, self.close_k), np.uint8))
        return m

    def detect(self, bgr, color):
        # -> unified Target dict {u,v,label,score,bbox} or None
        m = self.mask(bgr, color)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.min_area:
            return None
        x, y, w, h = cv2.boundingRect(c)
        if w * h > self.max_area_frac * bgr.shape[0] * bgr.shape[1]:
            return None                         # blob fills the frame -> threshold noise, not an object
        mo = cv2.moments(c)
        if mo['m00'] == 0:
            return None
        u = int(mo['m10'] / mo['m00'])
        v = int(mo['m01'] / mo['m00'])
        return {'u': u, 'v': v, 'label': color, 'score': float(area), 'bbox': (x, y, x + w, y + h)}

    def detect_any(self, bgr, colors=None):
        # best (largest) detection across the given colors
        best = None
        for col in (colors or self.colors()):
            t = self.detect(bgr, col)
            if t and (best is None or t['score'] > best['score']):
                best = t
        return best
