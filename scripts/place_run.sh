#!/usr/bin/env bash
# place_run.sh -- staged placement: cube in the closed gripper, robot >= 0.8 m from the mat edge (mat centre >= 0.9 m).
# Runs task_place.py (args pass through: --handin --look-only --dry-run ...); log to place.log (unbuffered); exit code = task_place's.
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
cd ~/jetrover_ws || exit 1
export PYTHONUNBUFFERED=1
python3 task_place.py "$@" 2>&1 | tee place.log | grep -E 'PLACE|DRIVE|LIVENESS|Traceback|Error'
exit "${PIPESTATUS[0]}"
