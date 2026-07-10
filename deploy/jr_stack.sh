#!/usr/bin/env bash
# JetRover stack launcher (systemd entry). Brings up the full stack for the demo:
# bringup (lidar+camera) + kinematics (+ nav if JR_STACK_NAV=1 in the env file).
# Single instance enforced -- the serial port owner must be unique (lockup lesson).
set -u
source /home/ubuntu/jetrover_ws/jr_env.sh

if pgrep -f "ros_robot_controller" >/dev/null 2>&1; then
    echo "stack already running (serial owner exists); refusing double bringup"
    exit 0
fi

ros2 launch jr_bringup robot.launch.py enable_camera:=true &
BRINGUP=$!
sleep 12
ros2 launch kinematics kinematics_node.launch.py &
KIN=$!
if [ "${JR_STACK_NAV:-0}" = "1" ]; then
    sleep 3
    ros2 launch jr_nav nav.launch.py &
fi
wait $BRINGUP $KIN
