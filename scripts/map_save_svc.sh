#!/usr/bin/env bash
# map_save_svc.sh -- save the current slam_toolbox map via its OWN services (no map_saver_cli,
# which hung on 2026-09-03): pgm/yaml via /slam_toolbox/save_map, then the pose graph via
# /slam_toolbox/serialize_map (serialize LAST: the sync node may stop publishing TF afterwards).
# Usage: bash ~/jetrover_ws/map_save_svc.sh <map_name>
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
NAME="${1:?usage: map_save_svc.sh <map_name>}"
DIR=/home/ubuntu/jetrover_ws/maps
mkdir -p "$DIR"
echo "=== save_map -> $DIR/$NAME.{pgm,yaml}"
timeout 60 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$DIR/$NAME'}}" 2>&1 | tail -n 1
echo "=== serialize_map -> $DIR/$NAME.{posegraph,data}"
timeout 120 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/$NAME'}" 2>&1 | tail -n 1
ls -la "$DIR" | grep "$NAME"
head -n 5 "$DIR/$NAME.yaml" 2>/dev/null
