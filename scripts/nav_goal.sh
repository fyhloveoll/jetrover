#!/usr/bin/env bash
# nav_goal.sh -- send one NavigateToPose goal in the map frame and report result, time and the
# AMCL pose at the end (P0 regression: does the robot still shuffle/oscillate at the goal?).
# Usage: bash ~/jetrover_ws/nav_goal.sh <x> <y> [yaw_deg] [timeout_s]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
X=${1:?usage: nav_goal.sh x y [yaw_deg] [timeout_s]}; Y=${2:?}; YAW=${3:-0}; TMO=${4:-120}
QZ=$(python3 -c "import math; print(math.sin(math.radians($YAW)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($YAW)/2))")
echo "goal: x=$X y=$Y yaw=$YAW deg  (timeout ${TMO}s)"
T0=$(date +%s.%N)
timeout "$TMO" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}}}" \
  2>&1 | grep -E "Goal accepted|Result|Goal finished|error_code|status" | tail -n 4
T1=$(date +%s.%N)
echo "elapsed: $(python3 -c "print(round($T1-$T0,1))") s"
echo "amcl pose now:"
timeout 6 ros2 topic echo --once /amcl_pose 2>/dev/null | python3 -c '
import sys, re, math
t = sys.stdin.read()
def g(k):
    m = re.search(k + r":\s*([-\d.e]+)", t); return float(m.group(1)) if m else float("nan")
try:
    o = t.split("orientation:")[1]
qz = float(re.search(r"z:s*([-d.e]+)", o).group(1)) if o else float("nan")
qw = float(re.search(r"w:s*([-d.e]+)", o).group(1)) if o else float("nan")
x, y, z, w = g("x"), g("y"), qz, qw
print("  x=%.3f y=%.3f yaw=%.1f deg" % (x, y, math.degrees(2*math.atan2(z, w))))'
