#!/usr/bin/env bash
# clock_probe.sh -- compare wall clock vs header stamps of key topics + time sync status.
# Diagnoses "timestamp earlier than all data in transform cache" (clock jump / stale publisher).
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
echo "wall: $(date +%s.%N)  ($(date))"
for t in /scan /odom /odom_raw /imu; do
  s=$(timeout 10 ros2 topic echo --once --field header.stamp "$t" 2>/dev/null | awk '/sec:/{if($1=="sec:")S=$2; if($1=="nanosec:")N=$2} END{if(S!="")printf "%s.%09d\n",S,N}')
  echo "$t stamp: ${s:-NO DATA}"
done
echo "--- tf odom->base_footprint (latest):"
timeout 10 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 "At time"
echo "--- tf map->odom (latest):"
timeout 10 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 -E "At time|not exist|Waiting"
echo "--- timedatectl:"; timedatectl 2>/dev/null | grep -E "System clock|NTP service|Local time"
echo "--- time sync journal (last 20 min):"
journalctl --since "-20min" 2>/dev/null | grep -iE "time (jump|step|has been changed)|systemd-timesyncd|chronyd|Synchronized|clock" | tail -n 8
