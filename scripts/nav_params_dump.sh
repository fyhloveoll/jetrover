#!/usr/bin/env bash
# nav_params_dump.sh -- the controller / goal-checker / smoother params that govern end-of-goal behaviour
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
for p in FollowPath.max_vel_x FollowPath.max_vel_y FollowPath.max_vel_theta FollowPath.max_vel_x_backwards \
         FollowPath.acc_lim_x FollowPath.acc_lim_theta FollowPath.min_turning_radius FollowPath.weight_kinematics_nh \
         FollowPath.xy_goal_tolerance FollowPath.yaw_goal_tolerance FollowPath.free_goal_vel FollowPath.dt_ref \
         FollowPath.max_global_plan_lookahead_dist FollowPath.feasibility_check_no_poses \
         goal_checker.xy_goal_tolerance goal_checker.yaw_goal_tolerance goal_checker.stateful \
         controller_frequency min_x_velocity_threshold min_theta_velocity_threshold \
         progress_checker.required_movement_radius progress_checker.movement_time_allowance \
         FollowPath.angular_dist_threshold FollowPath.rotate_to_heading_angular_vel FollowPath.max_angular_accel FollowPath.simulate_ahead_time; do
  printf '%-48s ' "$p"; timeout 5 ros2 param get /controller_server "$p" 2>&1 | tail -n 1
done
echo "--- velocity_smoother:"
for p in max_velocity min_velocity max_accel max_decel deadband_velocity velocity_timeout smoothing_frequency; do
  printf '%-48s ' "$p"; timeout 5 ros2 param get /velocity_smoother "$p" 2>&1 | tail -n 1
done
