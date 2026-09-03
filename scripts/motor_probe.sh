#!/usr/bin/env bash
# motor_probe.sh -- command-path check: beep, then drive +x at 0.1 m/s for ~1 s, then stop.
# Prints odom_raw x before/after so "did it really move" is answered by the encoders,
# not by the servo/goal echo (which lies when the board is locked).
# Usage: bash ~/jetrover_ws/motor_probe.sh [vx] [seconds]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
VX=${1:-0.10}
SEC=${2:-1}
N=$((SEC*10))

odom_x() { timeout 4 ros2 topic echo --once /odom_raw 2>/dev/null | awk '/position:/{f=1} f&&/x:/{print $2; exit}'; }

echo "odom_raw hz:"; timeout 5 ros2 topic hz /odom_raw 2>/dev/null | grep -m1 average || echo "  NO DATA (board telemetry dead)"
X0=$(odom_x); echo "x before: $X0"
echo "cmd_vel publishers now: $(timeout 4 ros2 topic info /controller/cmd_vel 2>/dev/null | grep -i 'Publisher count')"
echo "driving vx=$VX for ${SEC}s ..."
ros2 topic pub --times "$N" -r 10 /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: $VX}}" >/dev/null 2>&1
ros2 topic pub --times 3 -r 10 /controller/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 1
X1=$(odom_x); echo "x after:  $X1"
awk -v a="$X0" -v b="$X1" 'BEGIN{d=b-a; printf "odom delta x = %.3f m  (expected ~%.2f)\n", d, '"$VX"'*'"$SEC"'}'
