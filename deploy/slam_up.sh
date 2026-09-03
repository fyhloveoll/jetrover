#!/usr/bin/env bash
# slam_up.sh -- bringup (chassis+lidar+camera) + slam_toolbox for mapping, PID-tracked.
#
#   bash ~/jetrover_ws/slam_up.sh          # start (refuses if a bringup is already running)
#   bash ~/jetrover_ws/slam_up.sh stop     # SIGTERM slam then bringup, by recorded PID
#   bash ~/jetrover_ws/slam_up.sh status   # PIDs + key topic rates
#
# Rule: only ONE bringup instance at a time (serial port is exclusive).

# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
BR_PID=$WS/bringup.pid
SL_PID=$WS/slam.pid

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

case "${1:-start}" in
  stop)
    for p in "$SL_PID" "$BR_PID"; do
      # setsid => recorded PID is the process-group leader; kill the whole group so child nodes die too
      if alive "$p"; then kill -TERM -- "-$(cat "$p")"; echo "stopped $(basename "$p" .pid) group $(cat "$p")"; fi
      rm -f "$p"
    done
    sleep 2
    echo "--- remaining publishers on /controller/cmd_vel:"; timeout 5 ros2 topic info /controller/cmd_vel 2>/dev/null | grep -i publisher
    exit 0 ;;
  status)
    for p in "$BR_PID" "$SL_PID"; do
      if alive "$p"; then echo "$(basename "$p" .pid): RUNNING pid $(cat "$p")"; else echo "$(basename "$p" .pid): not running"; fi
    done
    echo "--- rates (5s each):"
    for t in /scan /odom /map; do echo -n "$t: "; timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 average || echo "no data"; done
    exit 0 ;;
  start) ;;
  *) echo "usage: $0 [start|stop|status]"; exit 2 ;;
esac

# refuse to double-start a bringup
if alive "$BR_PID"; then echo "bringup already running (pid $(cat "$BR_PID")); use stop first"; exit 1; fi
if timeout 5 ros2 topic info /controller/cmd_vel 2>/dev/null | grep -q "Publisher count: [1-9]"; then
  echo "another cmd_vel publisher exists -- some stack is already up, refusing"; exit 1
fi

echo "[slam_up] starting bringup (lidar+camera) ..."
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=true >"$WS/bringup.log" 2>&1 </dev/null &
echo $! >"$BR_PID"

# wait for odom + scan
for i in $(seq 1 30); do
  sleep 1
  if timeout 3 ros2 topic hz /odom 2>/dev/null | grep -q average && timeout 3 ros2 topic hz /scan 2>/dev/null | grep -q average; then
    echo "[slam_up] bringup ready after ~${i}s"; break
  fi
  [ "$i" = 30 ] && { echo "[slam_up] bringup NOT ready after 30s, see $WS/bringup.log"; tail -n 20 "$WS/bringup.log"; exit 1; }
done

echo "[slam_up] starting slam_toolbox ..."
setsid ros2 launch jr_slam slam.launch.py >"$WS/slam.log" 2>&1 </dev/null &
echo $! >"$SL_PID"
sleep 6
echo "--- status:"
bash "$0" status
