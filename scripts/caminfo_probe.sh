#!/usr/bin/env bash
# caminfo_probe.sh -- print intrinsics of the depth and rgb streams (are they the same K? distortion?)
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
for t in /depth_cam/depth/camera_info /depth_cam/rgb/camera_info; do
  echo "== $t"
  timeout 10 ros2 topic echo --once "$t" 2>/dev/null | python3 -c '
import sys, re
txt = sys.stdin.read()
def grab(key):
    m = re.search(r"^%s:\s*\n((?:- .*\n)+)" % key, txt, re.M)
    return [float(x) for x in re.findall(r"- (\S+)", m.group(1))] if m else None
for key in ("height", "width", "distortion_model"):
    m = re.search(r"^%s: (.*)$" % key, txt, re.M); print(" ", key, m.group(1) if m else "?")
k = grab("k"); d = grab("d")
if k: print("  fx=%.1f fy=%.1f cx=%.1f cy=%.1f" % (k[0], k[4], k[2], k[5]))
if d is not None: print("  d=", d)
'
done
