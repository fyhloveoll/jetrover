#!/usr/bin/env bash
# nav_patch_teb.sh -- runtime TEB / progress-checker patch for a 30 cm mecanum robot in a small
# apartment (vendor values are for open floors). Found 2026-09-03 from cmd_vel instrumentation:
# "trajectory is not feasible" resets + backup recoveries + lateral hunting came from
# min_obstacle_dist 0.26 (+0.15 footprint = 0.41 m clearance!), weight_obstacle 100,
# inflation_dist 0.6; ABORTs near the goal came from progress_checker 0.5 m / 10 s at 0.15 m/s.
# Vendor yaml untouched; call after mission_up.sh (or fold into its step 3.5).
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
set_p() { printf '%-52s ' "$2=$3"; timeout 10 ros2 param set "$1" "$2" "$3" 2>&1 | tail -n 1; }
set_p /controller_server FollowPath.min_obstacle_dist 0.10
set_p /controller_server FollowPath.inflation_dist 0.30
set_p /controller_server FollowPath.weight_obstacle 50.0
set_p /controller_server FollowPath.feasibility_check_no_poses 3
set_p /controller_server FollowPath.obstacle_poses_affected 10
set_p /controller_server progress_checker.required_movement_radius 0.25
set_p /controller_server progress_checker.movement_time_allowance 20.0
set_p /controller_server goal_checker.yaw_goal_tolerance 0.20
set_p /velocity_smoother max_accel "[2.5, 2.5, 3.2]"
set_p /velocity_smoother max_decel "[-2.5, -2.5, -3.2]"
