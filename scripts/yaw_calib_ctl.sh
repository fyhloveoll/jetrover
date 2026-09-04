#!/usr/bin/env bash
# yaw_calib_ctl.sh -- drive yaw_calib.py remotely (it runs long-lived in --watch mode).
#   bash ~/jetrover_ws/yaw_calib_ctl.sh start            # start the node (arm -> FLOOR pose)
#   bash ~/jetrover_ws/yaw_calib_ctl.sh cmd "30 center"  # one placement: true angle + tag
#   bash ~/jetrover_ws/yaw_calib_ctl.sh cmd peek         # list detections, no logging
#   bash ~/jetrover_ws/yaw_calib_ctl.sh log [N]          # last N log lines
#   bash ~/jetrover_ws/yaw_calib_ctl.sh stop             # send q, wait, kill if needed
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
CMD=$WS/yaw_calib.cmd
LOG=$WS/yaw_calib.log
PID=$WS/yaw_calib.pid

case "${1:-log}" in
  start)
    if [ -f "$PID" ] && kill -0 "$(cat "$PID")" 2>/dev/null; then echo "already running pid $(cat "$PID")"; exit 0; fi
    rm -f "$CMD"
    cd "$WS" || exit 1
    setsid python3 -u "$WS/yaw_calib.py" --watch "$CMD" ${YAW_CALIB_ARGS:-} >"$LOG" 2>&1 </dev/null &
    echo $! >"$PID"
    sleep 10
    echo "started pid $(cat "$PID")"; tail -n 8 "$LOG" ;;
  cmd)
    [ -z "${2:-}" ] && { echo "usage: $0 cmd \"<deg> [tag]|peek\""; exit 2; }
    n0=$(wc -l <"$LOG")
    echo "$2" >"$CMD"
    # wait for the node to consume the command and print a blank line (end of block)
    for _ in $(seq 1 160); do          # grasp takes ~25 s; poll up to 80 s
      sleep 0.5
      [ -f "$CMD" ] && continue
      [ "$(wc -l <"$LOG")" -le "$n0" ] && continue      # nothing printed yet
      if tail -n 1 "$LOG" | grep -qE '^$|=>|\[pose\]|\[verdict\]'; then break; fi
    done
    sleep 1
    tail -n +"$((n0 + 1))" "$LOG" ;;
  log)
    tail -n "${2:-30}" "$LOG" ;;
  stop)
    echo q >"$CMD"; sleep 3
    if [ -f "$PID" ] && kill -0 "$(cat "$PID")" 2>/dev/null; then kill -TERM -- "-$(cat "$PID")"; echo "killed"; else echo "exited cleanly"; fi
    rm -f "$PID" "$CMD"
    tail -n 15 "$LOG" ;;
  *) echo "usage: $0 start|cmd <line>|log [N]|stop"; exit 2 ;;
esac
