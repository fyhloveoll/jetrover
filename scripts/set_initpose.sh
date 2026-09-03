#!/usr/bin/env bash
# set_initpose.sh -- publish /initialpose for AMCL (map frame). Usage: set_initpose.sh [x y yaw_deg]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
X=${1:-0.0}; Y=${2:-0.0}; YAW=${3:-0.0}
QZ=$(python3 -c "import math; print(math.sin(math.radians($YAW)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($YAW)/2))")
ros2 topic pub --times 3 -r 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}, covariance: [0.05,0,0,0,0,0, 0,0.05,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.05]}}" >/dev/null 2>&1
sleep 3
echo "initial pose sent ($X, $Y, $YAW deg)"
echo -n "map->odom: "; timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 -E "Translation|not exist" 
echo -n "amcl_pose: "; timeout 6 ros2 topic echo --once /amcl_pose 2>/dev/null | grep -A2 "position:" | grep -E "x:|y:" | tr -d '\n'; echo
