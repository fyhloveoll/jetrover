#!/usr/bin/env bash
# yaw_view_test.sh -- viewpoint-invariance check: capture the SAME cube from three base
# rotations (servo1 440 / 560 / 500). If the geometry chain is right, the arm-frame angle
# (edge method) must not change with the viewpoint.
# Usage: bash ~/jetrover_ws/yaw_view_test.sh <true_deg>
C=~/jetrover_ws/yaw_calib_ctl.sh
T="${1:?usage: yaw_view_test.sh <true_deg>}"
bash "$C" cmd "pose 440" | tail -n 1
bash "$C" cmd "$T baseL" | tail -n 1
bash "$C" cmd "pose 560" | tail -n 1
bash "$C" cmd "$T baseR" | tail -n 1
bash "$C" cmd "pose 500" | tail -n 1
bash "$C" cmd "$T center2" | tail -n 1
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
cd ~/jetrover_ws && python3 yaw_calib_offline.py "*_${T}_*" 2>&1 | grep -v AMENT | tail -n 6
