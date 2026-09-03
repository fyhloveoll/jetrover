#!/usr/bin/env bash
# nav_stop.sh -- emergency-ish: kill the Nav2 launch group (controller, planner, bt, AMCL, relay),
# then publish zero velocity so the board holds still. Bringup (board driver) stays up.
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
for p in $(pgrep -f 'ros2 launch jr_nav'); do
  g=$(ps -o pgid= -p "$p" | tr -d ' ')
  kill -TERM -- "-$g" 2>/dev/null && echo "nav group $g TERM"
done
sleep 3
for p in $(pgrep -f 'ros2 launch jr_nav'); do g=$(ps -o pgid= -p "$p" | tr -d ' '); kill -KILL -- "-$g" 2>/dev/null; done
timeout 12 ros2 topic pub --times 5 -r 10 /controller/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 && echo "zero velocity sent"
echo "--- leftover nav processes:"; ps -eo pid,pgid,comm | grep -E "component_cont|amcl|cmd_vel_relay|bt_navigator|controller_server|planner_server" | grep -v grep || echo none
uptime
