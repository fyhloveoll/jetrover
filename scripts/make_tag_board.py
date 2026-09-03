#!/usr/bin/env python3
# encoding: utf-8
"""make_tag_board.py -- A4 AprilTag 36h11 board for eye-in-hand calibration (cv2.aruco).
Layout: 3 cols x 4 rows, tag side 40 mm, centre pitch 60 mm, IDs 10..21 (0-3 are the drop-zone
corner cards). 300 dpi, portrait. A 100 mm scale bar at the bottom checks the print scale
(print at 100%, no "fit to page").
Usage: python3 make_tag_board.py [out.png]
The same numbers (TAG_MM, PITCH_MM, IDS, layout) are what the calibration solver must use.
"""
import sys
import cv2
import numpy as np

DPI = 300
MM = DPI / 25.4
W, H = int(round(210 * MM)), int(round(297 * MM))
COLS, ROWS = 3, 4
TAG_MM, PITCH_MM = 40.0, 60.0
IDS = list(range(10, 10 + COLS * ROWS))
X0_MM = (210 - (COLS - 1) * PITCH_MM) / 2.0        # centre of first column
Y0_MM = 45.0                                        # centre of first row from the top

out = sys.argv[1] if len(sys.argv) > 1 else 'tag_board_36h11_A4.png'
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
img = np.full((H, W), 255, np.uint8)
tag_px = int(round(TAG_MM * MM))


def marker(mid, px):
    try:
        return cv2.aruco.generateImageMarker(d, mid, px)     # cv2 >= 4.7
    except AttributeError:
        return cv2.aruco.drawMarker(d, mid, px)              # cv2 4.5 (robot)


k = 0
for r in range(ROWS):
    for c in range(COLS):
        cx = int(round((X0_MM + c * PITCH_MM) * MM)); cy = int(round((Y0_MM + r * PITCH_MM) * MM))
        m = marker(IDS[k], tag_px)
        img[cy - tag_px // 2: cy - tag_px // 2 + tag_px, cx - tag_px // 2: cx - tag_px // 2 + tag_px] = m
        cv2.putText(img, 'id %d' % IDS[k], (cx - int(9 * MM), cy + tag_px // 2 + int(6 * MM)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3, cv2.LINE_AA)
        k += 1

# scale bar 100 mm and notes
y = int(round(275 * MM)); x0 = int(round(55 * MM)); x1 = int(round(155 * MM))
cv2.line(img, (x0, y), (x1, y), 0, 6); cv2.line(img, (x0, y - 40), (x0, y + 40), 0, 6); cv2.line(img, (x1, y - 40), (x1, y + 40), 0, 6)
cv2.putText(img, '100 mm  (verify after printing; tag side = 40 mm, pitch = 60 mm)', (int(28 * MM), y - 60),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2, cv2.LINE_AA)
cv2.putText(img, 'AprilTag 36h11  ids 10-21  3x4  JetRover hand-eye board', (int(28 * MM), int(12 * MM)),
            cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3, cv2.LINE_AA)
# orientation mark: arrow "up" so the board pose is unambiguous in photos
cv2.arrowedLine(img, (int(190 * MM), int(28 * MM)), (int(190 * MM), int(8 * MM)), 0, 5, tipLength=0.3)

cv2.imwrite(out, img)
print(out, img.shape[1], 'x', img.shape[0], 'px  (A4 @ %d dpi)' % DPI)
