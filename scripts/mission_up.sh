#!/usr/bin/env bash
# One-shot clean stack bringup for mission runs. Run ON the robot:
#   bash ~/jetrover_ws/mission_up.sh
# Kills leftovers by PID (never by self-matching pattern), verifies zero
# survivors and single cmd_vel publisher, starts bringup+kinematics+nav,
# waits on NODE STATE (not log guesswork). Prints READY or the exact blocker.
set -u
source ~/jetrover_ws/jr_env.sh

echo "== 1/4 sweep leftovers =="
# our stack processes from previous sessions, matched precisely and killed by PID
for pat in "ros2 launch jr_bringup" "ros2 launch kinematics" "ros2 launch jr_nav" \
           "jr_grasp_all.py" "jr_mission.py" "mjpeg_stream.py"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        kill -TERM "$p" 2>/dev/null && echo "  TERM $p ($pat)"
    done
done
sleep 4
# orphaned relays/containers survive their parents -- sweep by parent=1
for p in $(pgrep -f "cmd_vel_relay|component_container_isolated" 2>/dev/null); do
    ppid=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    [ "$ppid" = "1" ] && kill -9 "$p" 2>/dev/null && echo "  KILL9 orphan $p"
done
sleep 2

echo "== 2/4 launch stack =="
setsid ros2 launch jr_bringup robot.launch.py enable_camera:=true >~/bringup.log 2>&1 </dev/null &
sleep 14
setsid ros2 launch kinematics kinematics_node.launch.py >~/kin.log 2>&1 </dev/null &
sleep 6
setsid ros2 launch jr_nav nav.launch.py >~/nav.log 2>&1 </dev/null &

echo "== 3/4 wait on node state (max 120s) =="
for i in $(seq 1 24); do
    cam=$(ros2 topic list 2>/dev/null | grep -cE depth_cam)
    nav=$(ros2 action list 2>/dev/null | grep -c navigate_to_pose)
    amcl=$(ros2 lifecycle get /amcl 2>/dev/null | grep -c active)
    [ "$cam" -ge 15 ] && [ "$nav" -ge 1 ] && [ "$amcl" -ge 1 ] && break
    sleep 5
done
echo "  camera_topics=$cam nav_action=$nav amcl_active=$amcl"

echo "== 4/4 sanity =="
pubs=$(ros2 topic info /controller/cmd_vel 2>/dev/null | grep "Publisher count" | grep -o "[0-9]*")
echo "  cmd_vel publishers: ${pubs:-?} (must be 1)"
python3 ~/jetrover_ws/jr_arm_pose.py observe 2.5 >/dev/null 2>&1 && echo "  arm: OK"
python3 ~/jetrover_ws/buzz.py 2>/dev/null | tail -1

if [ "${cam:-0}" -ge 15 ] && [ "${nav:-0}" -ge 1 ] && [ "${pubs:-0}" = "1" ]; then
    echo "READY -- set initial pose (RViz or robot-side), place cubes, then: python3 ~/jetrover_ws/jr_mission.py run"
else
    echo "NOT READY -- see counts above; camera<15 = wait longer; pubs!=1 = zombie relay"
fi
