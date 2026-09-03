#!/usr/bin/env bash
# nav_patch_smoother.sh -- runtime patch: velocity_smoother lateral ACCEL limits were 0.0 (vendor
# yaml, differential assumption) so vy could never leave 0 even after max_velocity was unlocked
# (found 2026-09-03). Also re-reads the goal tolerances with retries (CLI is flaky under load).
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
ros2 param set /velocity_smoother max_accel "[2.5, 2.5, 3.2]" | tail -n 1
ros2 param set /velocity_smoother max_decel "[-2.5, -2.5, -3.2]" | tail -n 1
for p in "/velocity_smoother max_accel" "/velocity_smoother max_decel" \
         "/controller_server goal_checker.xy_goal_tolerance" "/controller_server goal_checker.yaw_goal_tolerance" \
         "/controller_server FollowPath.yaw_goal_tolerance" "/controller_server FollowPath.xy_goal_tolerance" \
         "/controller_server FollowPath.acc_lim_x" "/controller_server FollowPath.acc_lim_y" "/controller_server FollowPath.min_turning_radius" \
         "/controller_server FollowPath.max_vel_y" "/controller_server FollowPath.weight_kinematics_nh"; do
  set -- $p
  for try in 1 2 3; do
    out=$(timeout 8 ros2 param get "$1" "$2" 2>&1 | tail -n 1)
    case "$out" in *value*|*"not set"*) break;; esac
  done
  printf '%-50s %s\n' "$1 $2" "$out"
done
