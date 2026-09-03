#!/usr/bin/env bash
# nav_lean_up.sh -- navigation-only stack for nav testing: bringup WITHOUT camera (saves ~1 core),
# no foxglove bridge, Nav2 with the apartment map, then the runtime patches (TEB holonomic +
# smoother lateral accel + TEB obstacle margins + progress checker) VERIFIED by read-back.
# Usage: bash ~/jetrover_ws/nav_lean_up.sh [init_x init_y init_yaw_deg]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
WS=~/jetrover_ws
bash $WS/stack_sweep.sh
echo "== bringup (no camera) =="
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=false >$WS/bringup.log 2>&1 </dev/null &
echo $! >$WS/bringup.pid
for i in $(seq 1 30); do sleep 1; timeout 3 ros2 topic hz /scan 2>/dev/null | grep -q average && { echo "  scan up after ${i}s"; break; }; done
echo "== nav =="
setsid ros2 launch jr_nav nav.launch.py map:="${JR_MAP:-/home/ubuntu/jetrover_ws/maps/apt_loop_20260903.yaml}" >$WS/nav.log 2>&1 </dev/null &
echo $! >$WS/nav.pid
for i in $(seq 1 40); do sleep 3; a=$(timeout 5 ros2 lifecycle get /amcl 2>/dev/null | grep -c active); n=$(timeout 5 ros2 action list 2>/dev/null | grep -c navigate_to_pose); [ "$a" -ge 1 ] && [ "$n" -ge 1 ] && { echo "  amcl active + nav action after $((i*3))s"; break; }; done
sleep 5
echo "== patches =="
setv() { for try in 1 2 3; do out=$(timeout 10 ros2 param set "$1" "$2" "$3" 2>&1 | tail -n 1); case "$out" in *successful*) break;; esac; sleep 1; done; rb=$(timeout 8 ros2 param get "$1" "$2" 2>&1 | tail -n 1); printf '  %-46s set:%-22s readback: %s\n' "$2" "${out:-TIMEOUT}" "$rb"; }
setv /controller_server FollowPath.max_vel_y 0.15
setv /controller_server FollowPath.weight_kinematics_nh 1.0
setv /controller_server FollowPath.max_vel_theta 0.6
setv /controller_server FollowPath.min_obstacle_dist 0.10
setv /controller_server FollowPath.inflation_dist 0.30
setv /controller_server FollowPath.weight_obstacle 50.0
setv /controller_server FollowPath.feasibility_check_no_poses 3
setv /controller_server goal_checker.xy_goal_tolerance 0.12
setv /controller_server goal_checker.yaw_goal_tolerance 0.20
setv /controller_server progress_checker.required_movement_radius 0.25
setv /controller_server progress_checker.movement_time_allowance 20.0
setv /velocity_smoother max_velocity "[0.15, 0.15, 0.6]"
setv /velocity_smoother min_velocity "[-0.15, -0.15, -0.6]"
setv /velocity_smoother max_accel "[2.5, 2.5, 3.2]"
setv /velocity_smoother max_decel "[-2.5, -2.5, -3.2]"
echo "== initial pose =="
bash $WS/set_initpose.sh "${1:-0}" "${2:-0}" "${3:-0}"
echo "== sanity =="
echo -n "cmd_vel publishers: "; timeout 5 ros2 topic info /controller/cmd_vel 2>/dev/null | grep -i "publisher count"
uptime
