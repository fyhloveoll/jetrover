#!/usr/bin/env bash
# stop_mission.sh -- stop a running mission / grasp / placement task by PID (never pkill -f: it would kill the
# ssh session that carries the same string). Sends SIGTERM to the process group of every jr_mission.py run,
# jr_grasp_all.py and task_place.py process; bringup / nav stay up. (Copy of the on-robot script, 2026-10-22.)
for p in $(ps -eo pid,args | grep -E "[j]r_mission.py run|[j]r_grasp_all.py|[t]ask_place.py" | awk '{print $1}'); do
  kill -TERM -- -$p 2>/dev/null || kill -TERM $p 2>/dev/null
done
sleep 3
ps -eo pid,args | grep -E "[j]r_mission|[j]r_grasp_all|[t]ask_place" | cut -c1-60 || echo "mission stopped"
