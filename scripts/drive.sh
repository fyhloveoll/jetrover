#!/usr/bin/env bash
# JetRover drive mode: chassis + gamepad teleop (+ hang watchdog in background).
# Run after boot. Gamepad: left stick=move, right stick=turn. Ctrl-C to stop all.
source /home/ubuntu/jetrover_ws/jr_env.sh

# hang watchdog (captures evidence if the control board freezes while driving)
pkill -9 -f imu_watchdog.sh 2>/dev/null
setsid bash -c '/home/ubuntu/jetrover_ws/imu_watchdog.sh > /home/ubuntu/jetrover_ws/imu_watchdog.log 2>&1' < /dev/null &

cleanup() { echo; echo "stopping drive mode..."; pkill -9 -f imu_watchdog.sh 2>/dev/null; }
trap cleanup EXIT

echo "=== JetRover drive mode ==="
echo "Gamepad: left stick = move, right stick = turn. Ctrl-C to stop."
ros2 launch jr_bringup jr_drive.launch.py "$@"
