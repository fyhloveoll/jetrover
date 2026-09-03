#!/usr/bin/env python3
"""Render a map .pgm to an upscaled .png with the origin and current robot pose marked.
Usage: python3 map_png.py <map_name_without_ext> [robot_x robot_y]
Reads <name>.pgm/.yaml in the current dir, writes <name>.png (3x nearest-neighbour)."""
import sys, cv2, yaml

name = sys.argv[1]
with open(name + '.yaml') as f:
    meta = yaml.safe_load(f)
res = meta['resolution']
ox, oy = meta['origin'][0], meta['origin'][1]
im = cv2.imread(name + '.pgm', 0)
h, w = im.shape
S = 3
big = cv2.resize(im, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST)
big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)

def to_px(x, y):
    return int((x - ox) / res * S), int((h - (y - oy) / res) * S)

# map origin / robot start (0,0): green
cv2.circle(big, to_px(0.0, 0.0), 6, (0, 200, 0), 2)
if len(sys.argv) >= 4:
    rx, ry = float(sys.argv[2]), float(sys.argv[3])
    cv2.circle(big, to_px(rx, ry), 6, (0, 0, 255), 2)   # robot now: red
cv2.imwrite(name + '.png', big)
print(name + '.png', big.shape[1], 'x', big.shape[0])
