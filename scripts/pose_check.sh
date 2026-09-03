#!/usr/bin/env bash
# pose_check.sh -- print robot pose in map frame (SLAM-corrected) and in odom frame (dead reckoning).
# At the start mark both should read ~0; map vs odom shows how much scan matching corrected drift.
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
for parent in map odom; do
  echo "=== $parent -> base_footprint"
  timeout 6 ros2 run tf2_ros tf2_echo "$parent" base_footprint 2>/dev/null | grep -m2 -E 'Translation|RPY \(degree\)'
done
