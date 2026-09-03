#!/usr/bin/env bash
# foxglove_up.sh -- install (once) and start foxglove_bridge on the JetRover.
#
# Why: the dev laptop is now Windows; no RViz there. Foxglove desktop on Windows
# connects to ws://<robot-ip>:8765 and shows topics/TF/images/laser/maps.
#
# Usage (on the robot, or via ssh):
#   bash ~/jetrover_ws/foxglove_up.sh            # install if missing, then start in background
#   bash ~/jetrover_ws/foxglove_up.sh stop       # stop the bridge
#   bash ~/jetrover_ws/foxglove_up.sh status     # is it running? which topics?
#
# From Windows/WSL:
#   ssh jetrover 'bash -lc "bash ~/jetrover_ws/foxglove_up.sh"'
# NOTE: no `set -u` -- /opt/ros/humble/setup.bash references unbound variables.

LOG=~/jetrover_ws/foxglove_bridge.log
PIDFILE=~/jetrover_ws/foxglove_bridge.pid
PORT=8765

# jr_env.sh is the clean bash env (ROS_DOMAIN_ID=0, vendor + own workspaces).
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh

case "${1:-start}" in
  stop)
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      kill -TERM -- "-$(cat "$PIDFILE")"   # setsid => PID is the process-group leader; kill the whole group
      echo "foxglove_bridge stopped (pid $(cat "$PIDFILE"))"
      rm -f "$PIDFILE"
    else
      echo "foxglove_bridge not running"
    fi
    exit 0
    ;;
  status)
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "RUNNING pid $(cat "$PIDFILE"), port $PORT, ip $(hostname -I | awk '{print $1}')"
    else
      echo "NOT RUNNING"
    fi
    echo "--- last log lines:"; tail -n 5 "$LOG" 2>/dev/null
    echo "--- topics:"; timeout 5 ros2 topic list 2>/dev/null | head -n 40
    exit 0
    ;;
  start) ;;
  *) echo "usage: $0 [start|stop|status]"; exit 2 ;;
esac

# 1. install once (apt, ~10 MB). Needs sudo (passwordless on the robot).
if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "[foxglove_up] installing ros-humble-foxglove-bridge ..."
  sudo apt-get update -qq
  sudo apt-get install -y -qq ros-humble-foxglove-bridge
  # apt packages land in /opt/ros/humble, already on the path via jr_env.sh
fi

# 2. already running? (single instance)
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "[foxglove_up] already running, pid $(cat "$PIDFILE")"
  exit 0
fi

# 3. start detached so it survives ssh exit; PID recorded for clean stop (no pkill!)
setsid ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$PORT \
  >"$LOG" 2>&1 </dev/null &
echo $! >"$PIDFILE"
sleep 3
if kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "[foxglove_up] running, pid $(cat "$PIDFILE")"
  echo "[foxglove_up] connect Foxglove desktop to: ws://$(hostname -I | awk '{print $1}'):$PORT"
else
  echo "[foxglove_up] FAILED to start, see $LOG"; tail -n 20 "$LOG"; exit 1
fi
