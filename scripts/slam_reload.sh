#!/usr/bin/env bash
# slam_reload.sh -- restart slam_toolbox (by PID, bringup untouched) loading a saved pose graph.
# Use after the sync node hangs (seen 2026-09-02 right after /slam_toolbox/serialize_map), or to
# continue mapping / relocalize in an existing map. Robot is assumed to stand at map_start_pose.
# Usage: bash ~/jetrover_ws/slam_reload.sh <map_name> [x y yaw]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
NAME="${1:?usage: slam_reload.sh <map_name> [x y yaw]}"
X=${2:-0.0}; Y=${3:-0.0}; TH=${4:-0.0}
SL_PID=$WS/slam.pid
YAML=$WS/install/jr_slam/share/jr_slam/config/slam.yaml

if [ -f "$SL_PID" ] && kill -0 "$(cat "$SL_PID")" 2>/dev/null; then
  OLD=$(cat "$SL_PID")
  kill -TERM -- "-$OLD"; sleep 4          # process group (setsid), so the node under the launch wrapper dies too
  if kill -0 "$OLD" 2>/dev/null; then echo "slam group $OLD ignored TERM, KILL"; kill -KILL -- "-$OLD"; fi
fi
# safety net: comm names are truncated to 15 chars, so match the prefix, not -x
pkill -KILL -f 'slam_toolbox/sync_slam_toolbox_node' 2>/dev/null
sleep 1
echo "old slam stopped; starting with $NAME at ($X,$Y,$TH)"

setsid ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
  --params-file "$YAML" \
  -p map_file_name:="$WS/maps/$NAME" \
  -p map_start_pose:="[$X,$Y,$TH]" \
  >"$WS/slam.log" 2>&1 </dev/null &
echo $! >"$SL_PID"
sleep 12
echo "=== /map rate:"; timeout 6 ros2 topic hz /map 2>/dev/null | grep -m1 average || echo "no /map yet"
echo "=== pose:"; bash "$WS/pose_check.sh"
echo "=== log tail:"; tail -n 5 "$WS/slam.log"
