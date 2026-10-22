#!/usr/bin/env bash
source ~/jetrover_ws/jr_env.sh
cd ~/jetrover_ws
rm -f mission.log
export JR_MAX_TRIPS=2 PYTHONUNBUFFERED=1 JR_MAX_WIDTH=0.06   # 0.06: a 40 mm cube seen at 45 deg measures ~53-57 mm (mission7b skipped one at 48)
setsid bash -c 'sleep 10; python3 ~/jetrover_ws/jr_mission.py run' > ~/jetrover_ws/mission.log 2>&1 < /dev/null &
echo "mission launched, group $!"
