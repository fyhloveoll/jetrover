#!/usr/bin/env bash
# grasp_up.sh -- bringup (chassis+lidar+camera) + kinematics service, PID-tracked.
# The stack the grasp/calibration scripts need. No SLAM, no nav.
#   bash ~/jetrover_ws/grasp_up.sh          # start (refuses if a bringup is already running)
#   bash ~/jetrover_ws/grasp_up.sh stop     # SIGTERM kinematics then bringup, by process group
#   bash ~/jetrover_ws/grasp_up.sh status
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
BR_PID=$WS/bringup.pid
KIN_PID=$WS/kinematics.pid

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

case "${1:-start}" in
  stop)
    for p in "$KIN_PID" "$BR_PID"; do
      if alive "$p"; then kill -TERM -- "-$(cat "$p")"; echo "stopped $(basename "$p" .pid) group $(cat "$p")"; fi
      rm -f "$p"
    done
    sleep 2
    echo "--- leftover ROS processes:"; ps -eo pid,pgid,comm | grep -E "ros_robot|ydlidar|depth_cam|ekf|kinematic|servo|odom_pub" || echo "none"
    exit 0 ;;
  status)
    for p in "$BR_PID" "$KIN_PID"; do
      if alive "$p"; then echo "$(basename "$p" .pid): RUNNING group $(cat "$p")"; else echo "$(basename "$p" .pid): not running"; fi
    done
    echo "--- rates (5s each):"
    for t in /odom_raw /depth_cam/rgb/image_raw /depth_cam/depth/image_raw; do
      echo -n "$t: "; timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 average || echo "no data"
    done
    echo -n "IK service: "; timeout 5 ros2 service list 2>/dev/null | grep -q /kinematics/set_pose_target && echo up || echo DOWN
    exit 0 ;;
  start) ;;
  *) echo "usage: $0 [start|stop|status]"; exit 2 ;;
esac

if alive "$BR_PID"; then echo "bringup already running (group $(cat "$BR_PID")); use stop first"; exit 1; fi

echo "[grasp_up] starting bringup (lidar+camera) ..."
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=true >"$WS/bringup.log" 2>&1 </dev/null &
echo $! >"$BR_PID"
for i in $(seq 1 30); do
  sleep 1
  if timeout 3 ros2 topic hz /odom_raw 2>/dev/null | grep -q average && \
     timeout 3 ros2 topic hz /depth_cam/depth/image_raw 2>/dev/null | grep -q average; then
    echo "[grasp_up] bringup ready after ~${i}s"; break
  fi
  [ "$i" = 30 ] && { echo "[grasp_up] bringup NOT ready after 30s"; tail -n 20 "$WS/bringup.log"; exit 1; }
done

echo "[grasp_up] starting kinematics ..."
setsid ros2 launch kinematics kinematics_node.launch.py >"$WS/kinematics.log" 2>&1 </dev/null &
echo $! >"$KIN_PID"
sleep 5
bash "$0" status
