#!/usr/bin/env bash
# place_boot.sh -- light stack for a STAGED placement take: bringup(camera) + kinematics only (no Nav2),
# buzz. Then:  python3 task_place.py --handin   (opens the gripper, waits for the cube, closes, places)  -- or bash place_run.sh --handin
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
cd ~/jetrover_ws || exit 1
T0=$(date +%s)
bash stack_sweep.sh | tail -1
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=true > ~/bringup.log 2>&1 < /dev/null &
echo $! > bringup.pid
sleep 6
setsid ros2 launch kinematics kinematics_node.launch.py > ~/kin.log 2>&1 < /dev/null &
echo $! > kin.pid
python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
rclpy.init(); n = Node('place_wait'); t = time.monotonic()
while time.monotonic() - t < 90:
    names = [x for x, _ in n.get_topic_names_and_types()]
    cam = sum(1 for x in names if 'depth_cam' in x)
    if cam >= 12 and '/servo_controller' in names and '/odom_raw' in names:
        print('[%5.1fs] camera(%d) servo odom OK' % (time.monotonic() - t, cam)); break
    rclpy.spin_once(n, timeout_sec=0.5)
else:
    print('NOT READY (camera/servo/odom missing)')
n.destroy_node(); rclpy.shutdown()
PY
sleep 2
python3 buzz.py >/dev/null 2>&1 || true
echo "place stack ready in $(( $(date +%s) - T0 )) s. Next: python3 task_place.py --handin   (opens, waits for the cube, closes, places)"
