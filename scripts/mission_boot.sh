#!/usr/bin/env bash
# mission_boot.sh -- ONE-SHOT boot for the carry mission (10-21 rewrite of mission_up.sh + patches + filters):
#   sweep -> launch bringup(camera) + kinematics + nav(zone params) + filter masks (all setsid, in parallel)
#   -> stack_wait.py (one rclpy process: waits, patches, keepout, initial pose, buzzer; no ros2 CLI)
#   -> foxglove bridge.   Prints the total time.
#   bash ~/jetrover_ws/mission_boot.sh <init_x> <init_y> <init_yaw_deg>
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
cd ~/jetrover_ws || exit 1
X=${1:-1.0}; Y=${2:-0.0}; YAW=${3:-0}
T0=$(date +%s)
MAP=/home/ubuntu/jetrover_ws/maps/apt_demo2_20261021.yaml
PARAMS=/home/ubuntu/jetrover_ws/nav2_params_zones.yaml
bash stack_sweep.sh | tail -1
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=true > ~/bringup.log 2>&1 < /dev/null &
echo $! > bringup.pid
sleep 6                                              # serial port + lidar get a head start
setsid ros2 launch kinematics kinematics_node.launch.py > ~/kin.log 2>&1 < /dev/null &
echo $! > kin.pid
setsid ros2 launch jr_nav nav.launch.py map:=$MAP params_file:=$PARAMS > ~/nav.log 2>&1 < /dev/null &
echo $! > nav.pid
setsid ros2 launch /home/ubuntu/jetrover_ws/src/jr_nav/launch/filters.launch.py \
  keepout_mask:=/home/ubuntu/jetrover_ws/maps/zones_keepout.yaml \
  speed_mask:=/home/ubuntu/jetrover_ws/maps/zones_speed.yaml > filters.log 2>&1 < /dev/null &
echo $! > filters.pid
python3 stack_wait.py "$X" "$Y" "$YAW" --timeout 120
RC=$?
bash foxglove_up.sh 2>&1 | tail -1
echo "boot done in $(( $(date +%s) - T0 )) s (rc=$RC)"
exit $RC
