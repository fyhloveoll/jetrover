#!/usr/bin/env bash
# Save current SLAM map. Usage: ./save_map.sh [map_name]
source /home/ubuntu/jetrover_ws/jr_env.sh
NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
DIR="/home/ubuntu/jetrover_ws/maps"
mkdir -p "$DIR"
echo "Saving map -> $DIR/$NAME .pgm/.yaml"
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p save_map_timeout:=10000.0
