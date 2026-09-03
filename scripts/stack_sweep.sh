#!/usr/bin/env bash
# stack_sweep.sh -- kill EVERY ROS process of ours by PROCESS GROUP (setsid launches are group
# leaders; killing only the `ros2 launch` wrapper leaves driver nodes alive -> two
# ros_robot_controller on one serial port -> board lockup, seen 2026-09-03). Keeps foxglove_bridge.
# Usage: bash ~/jetrover_ws/stack_sweep.sh
PAT='ros_robot_controller|ydlidar|ekf_node|odom_publisher|imu_filter|servo_controller|joint_state_pub|robot_state_pub|depth_cam|camera_container|scan_to_scan|kinematics|component_container|amcl|cmd_vel_relay|ros2 launch|ros2 run|jr_grasp_all|jr_mission|yaw_calib|mission_up|slam_toolbox|joy_teleop|keyboard_teleop'
groups=$(ps -eo pgid,args | grep -E "$PAT" | grep -vE "foxglove|stack_sweep|grep" | awk '{print $1}' | sort -u)
for g in $groups; do kill -TERM -- "-$g" 2>/dev/null && echo "TERM group $g"; done
sleep 4
for g in $groups; do kill -KILL -- "-$g" 2>/dev/null; done
sleep 1
left=$(ps -eo pid,pgid,etime,args | grep -E "$PAT" | grep -vE "foxglove|stack_sweep|grep" | cut -c1-110)
if [ -n "$left" ]; then echo "LEFTOVERS:"; echo "$left"; exit 1; else echo "clean (foxglove_bridge kept)"; fi
