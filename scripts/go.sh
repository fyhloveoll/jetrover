#!/usr/bin/env bash
# go.sh -- next boot, ONE command: wait for ssh, push the changed files, run mission_boot.sh (prints READY + time)
#   bash go.sh <init_x> <init_y> <init_yaw_deg>
R=jetrover
REPO=/mnt/c/Users/86133/Desktop/C++/jetrover
S=$REPO/scripts
X=${1:-1.0}; Y=${2:-0.0}; YAW=${3:-0}
T0=$(date +%s)
for i in $(seq 1 60); do ssh -o ConnectTimeout=3 -o BatchMode=yes $R 'echo ssh up' 2>/dev/null && break; sleep 2; done
scp -q $REPO/scripts/stack_wait.py $REPO/scripts/mission_boot.sh $REPO/scripts/jr_grasp_all.py $REPO/scripts/station_dock.py $REPO/scripts/jr_mission.py $S/mission_launch.sh $S/place_skill.py $S/task_place.py $S/place_run.sh $S/place_boot.sh $R:~/jetrover_ws/
scp -q $REPO/docs/mission_2026-10-21.json $R:~/jetrover_ws/mission.json
ssh $R 'cd ~/jetrover_ws && sed -i "s/\r$//" stack_wait.py mission_boot.sh jr_grasp_all.py station_dock.py jr_mission.py mission_launch.sh place_skill.py task_place.py place_run.sh place_boot.sh && python3 -m py_compile stack_wait.py jr_grasp_all.py station_dock.py jr_mission.py place_skill.py task_place.py && echo "files deployed"'
ssh $R "bash -lc \"bash ~/jetrover_ws/mission_boot.sh $X $Y $YAW\""
echo "go.sh total $(( $(date +%s) - T0 )) s"
