#!/usr/bin/env bash
# nav_patch_all.sh -- ALL runtime Nav2 patches with read-back verification (vendor yaml untouched).
# Restarts the ros2 CLI daemon first (a stale daemon gives xmlrpc '!rclpy.ok()' on every call).
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 3
setv() { for try in 1 2 3; do out=$(timeout 10 ros2 param set "$1" "$2" "$3" 2>&1 | tail -n 1); case "$out" in *successful*) break;; esac; sleep 1; done; rb=$(timeout 8 ros2 param get "$1" "$2" 2>&1 | tail -n 1); printf '  %-46s %-24s readback: %s\n' "$2" "${out:-TIMEOUT}" "$rb"; }
setv /controller_server FollowPath.max_vel_y 0.15
setv /controller_server FollowPath.weight_kinematics_nh 10.0   # 1.0 -> lateral hunting (vy reversals 56/2min, 09-03); 10 allows but discourages strafing
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
setv /velocity_smoother deadband_velocity "[0.02, 0.05, 0.05]"   # kill sub-5cm/s lateral dither (09-03: vy reversals 56 -> 6)
