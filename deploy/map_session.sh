#!/usr/bin/env bash
# map_session.sh -- mapping on top of an ALREADY RUNNING bringup: slam_toolbox (loop closure on)
# + joystick teleop (speed-limited), PID-tracked process groups.
#   bash ~/jetrover_ws/map_session.sh start [max_lin] [max_ang]   # default 0.2 m/s, 0.6 rad/s
#   bash ~/jetrover_ws/map_session.sh stop
#   bash ~/jetrover_ws/map_session.sh status
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
SL_PID=$WS/slam.pid
JOY_PID=$WS/joy.pid
YAML=$WS/install/jr_slam/share/jr_slam/config/slam.yaml
alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

case "${1:-status}" in
  start)
    if ! timeout 5 ros2 topic info /odom_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
      echo "bringup not running (no /odom_raw publisher) -- start grasp_up.sh or slam_up.sh first"; exit 1
    fi
    if alive "$SL_PID"; then echo "slam already running (group $(cat "$SL_PID"))"; else
      # loop closure ON for the production map (the 09-02 baseline map was built with it off)
      sed -i 's/do_loop_closing: false/do_loop_closing: true/' "$YAML"
      grep -q "do_loop_closing: true" "$YAML" && echo "[map] loop closure: on"
      setsid ros2 launch jr_slam slam.launch.py >"$WS/slam.log" 2>&1 </dev/null &
      echo $! >"$SL_PID"; echo "[map] slam_toolbox started (group $(cat "$SL_PID"))"
    fi
    if alive "$JOY_PID"; then echo "joy_teleop already running"; else
      setsid ros2 run jr_teleop joy_teleop --ros-args -p max_linear:="${2:-0.2}" -p max_angular:="${3:-0.6}" \
        >"$WS/joy_teleop.log" 2>&1 </dev/null &
      echo $! >"$JOY_PID"; echo "[map] joy_teleop started (group $(cat "$JOY_PID")), max ${2:-0.2} m/s ${3:-0.6} rad/s"
    fi
    sleep 6; bash "$0" status ;;
  stopslam)
    if alive "$SL_PID"; then kill -TERM -- "-$(cat "$SL_PID")"; echo "stopped slam group $(cat "$SL_PID")"; fi
    rm -f "$SL_PID" ;;
  stop)
    for p in "$JOY_PID" "$SL_PID"; do
      if alive "$p"; then kill -TERM -- "-$(cat "$p")"; echo "stopped $(basename "$p" .pid) group $(cat "$p")"; fi
      rm -f "$p"
    done ;;
  status)
    for p in "$SL_PID" "$JOY_PID"; do
      if alive "$p"; then echo "$(basename "$p" .pid): RUNNING group $(cat "$p")"; else echo "$(basename "$p" .pid): not running"; fi
    done
    echo "--- rates (5s each):"
    for t in /scan /odom /map /ros_robot_controller/joy; do echo -n "$t: "; timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 average || echo "no data"; done
    echo -n "cmd_vel publishers: "; timeout 5 ros2 topic info /controller/cmd_vel 2>/dev/null | grep -i "publisher count" ;;
  *) echo "usage: $0 start [max_lin] [max_ang]|stop|status"; exit 2 ;;
esac
