#!/usr/bin/env bash
# map_finish.sh -- end-of-mapping bookkeeping. Run when the robot is back on the start mark.
#   1. print the SLAM pose of the robot right now (map -> base_footprint) = loop-closure residual
#      (robot started at map origin; compare with the tape-measured offset from the mark)
#   2. save occupancy grid (pgm/yaml) via map_saver_cli
#   3. serialize the slam_toolbox pose graph (.posegraph/.data) for later localization/continued mapping
# Usage: bash ~/jetrover_ws/map_finish.sh <map_name>
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
NAME="${1:?usage: map_finish.sh <map_name>}"
DIR=~/jetrover_ws/maps
mkdir -p "$DIR"

echo "=== 1. current pose map->base_footprint (x y yaw)"
timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m2 -E "Translation|Rotation.*RPY \(degree\)"
echo "    (robot started at ~(0,0,0); measure the real offset from the tape mark and compare)"

echo "=== 2. occupancy grid -> $DIR/$NAME.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p save_map_timeout:=10000.0 2>&1 | grep -vE "^\[INFO\].*Waiting|^$" | tail -n 3

echo "=== 3. pose graph -> $DIR/$NAME.{posegraph,data}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}" 2>&1 | tail -n 1

echo "=== files"
ls -la "$DIR" | grep "$NAME"
echo "=== map size"
head -n 3 "$DIR/$NAME.yaml"
python3 - "$DIR/$NAME.pgm" <<'EOF'
import sys
p=sys.argv[1]
with open(p,'rb') as f:
    magic=f.readline();
    line=f.readline()
    while line.startswith(b'#'): line=f.readline()
    w,h=map(int,line.split()); f.readline()
    data=f.read()
free=sum(1 for b in data if b>=250); occ=sum(1 for b in data if b<=50)
print(f"{w}x{h} cells @0.05m = {w*0.05:.1f}m x {h*0.05:.1f}m; free {free*0.0025:.1f} m2, occupied {occ} cells")
EOF
