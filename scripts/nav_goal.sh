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
timeout 8 ros2 topic echo --once /amcl_pose 2>/dev/null > /tmp/amcl_pose.txt
python3 - <<'PYEOF'
import math, re
t = open('/tmp/amcl_pose.txt').read()
def num(block, key):
    m = re.search(r'\b' + key + r':\s*([-+\d.eE]+)', block)
    return float(m.group(1)) if m else float('nan')
pos = t.split('position:')[1].split('orientation:')[0] if 'position:' in t else ''
ori = t.split('orientation:')[1] if 'orientation:' in t else ''
x, y = num(pos, 'x'), num(pos, 'y')
qz, qw = num(ori, 'z'), num(ori, 'w')
print('  x=%.3f y=%.3f yaw=%.1f deg' % (x, y, math.degrees(2 * math.atan2(qz, qw))))
PYEOF
