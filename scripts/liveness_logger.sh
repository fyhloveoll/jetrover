#!/usr/bin/env bash
# Lightweight board-liveness logger. Appends one line every 20s recording whether
# the control board's /imu topic is still publishing. Used to catch the rare
# "hangs after sitting idle" failure without holding an SSH session open.
source ~/jetrover_ws/jr_env.sh >/dev/null 2>&1
LOG="$HOME/jetrover_ws/liveness.log"
echo "$(date '+%F %T') START logger" >> "$LOG"
while true; do
  if timeout 4 ros2 topic echo /imu --once >/dev/null 2>&1; then
    st=ALIVE
  else
    st=SILENT
  fi
  echo "$(date '+%T') $st" >> "$LOG"
  sleep 20
done
