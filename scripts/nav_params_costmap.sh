#!/usr/bin/env bash
# nav_params_costmap.sh -- footprint / inflation / TEB obstacle params (why "trajectory not feasible")
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
get() { for try in 1 2 3; do out=$(timeout 8 ros2 param get "$1" "$2" 2>&1 | tail -n 1); case "$out" in *value*|*"not set"*) break;; esac; done; printf '%-58s %s\n' "$1 $2" "$out"; }
for p in footprint robot_radius footprint_padding width height resolution update_frequency publish_frequency \
         inflation_layer.inflation_radius inflation_layer.cost_scaling_factor obstacle_layer.enabled voxel_layer.enabled \
         obstacle_layer.scan.obstacle_max_range obstacle_layer.scan.raytrace_max_range plugins; do
  get /local_costmap/local_costmap "$p"
done
for p in footprint robot_radius inflation_layer.inflation_radius inflation_layer.cost_scaling_factor plugins; do
  get /global_costmap/global_costmap "$p"
done
for p in FollowPath.min_obstacle_dist FollowPath.inflation_dist FollowPath.include_costmap_obstacles \
         FollowPath.costmap_obstacles_behind_robot_dist FollowPath.obstacle_poses_affected FollowPath.footprint_model.type \
         FollowPath.footprint_model.radius FollowPath.footprint_model.vertices FollowPath.footprint_model.line_start FollowPath.footprint_model.line_end \
         FollowPath.feasibility_check_no_poses FollowPath.weight_obstacle FollowPath.weight_viapoint FollowPath.global_plan_viapoint_sep \
         FollowPath.allow_init_with_backwards_motion FollowPath.max_vel_x_backwards FollowPath.weight_max_vel_x FollowPath.weight_optimaltime \
         FollowPath.enable_homotopy_class_planning FollowPath.dt_ref FollowPath.dt_hysteresis; do
  get /controller_server "$p"
done
