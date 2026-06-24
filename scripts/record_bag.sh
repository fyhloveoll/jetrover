#!/usr/bin/env bash
# Record core topics for sim-to-real data. Usage: ./record_bag.sh [tag]
source /home/ubuntu/jetrover_ws/jr_env.sh
TAG="${1:-run}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/ubuntu/jetrover_ws/bags/${STAMP}_${TAG}"
mkdir -p /home/ubuntu/jetrover_ws/bags
echo "Recording -> $OUT  (Ctrl-C to stop)"
ros2 bag record -o "$OUT" \
  /scan /odom /tf /tf_static /controller/cmd_vel /imu /map
