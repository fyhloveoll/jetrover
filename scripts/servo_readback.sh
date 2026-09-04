#!/usr/bin/env bash
# servo_readback.sh -- REAL servo positions/voltage/temperature via the vendor driver's
# /ros_robot_controller/bus_servo/get_state service (servo_states echoes targets only).
# Compares with the FLOOR observation pose targets. Safe: goes through the running driver,
# never opens the serial port itself.
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
REQ='{cmd: ['
for i in 1 2 3 4 5 10; do REQ="$REQ{id: $i, get_position: 1, get_voltage: 1, get_temperature: 1, get_torque_state: 1},"; done
REQ="${REQ%,}]}"
timeout 20 ros2 service call /ros_robot_controller/bus_servo/get_state ros_robot_controller_msgs/srv/GetBusServoState "$REQ" 2>&1 \
  | python3 -c '
import sys, re
t = sys.stdin.read()
target = {1: 500, 2: 700, 3: 15, 4: 175, 5: 500, 10: 200}
blocks = re.findall(r"BusServoState\((.*?)\)(?=, ros_robot_controller_msgs|\]\))", t, re.S)
if not blocks:
    print("no response / parse failed:"); print(t[-600:]); sys.exit(1)
print("%4s %8s %8s %6s %9s %6s %6s" % ("id", "target", "actual", "diff", "voltage", "temp", "torq"))
for b in blocks:
    def arr(k):
        m = re.search(k + r"=array\(.*?\[(.*?)\]", b); return [int(x) for x in m.group(1).split(",")] if m and m.group(1).strip() else []
    ids = arr("present_id") or arr("target_id"); pos = arr("position"); v = arr("voltage"); tp = arr("temperature"); tq = arr("enable_torque")
    i = ids[0] if ids else -1
    p = pos[0] if pos else -1
    print("%4d %8d %8d %6d %9s %6s %6s" % (i, target.get(i, -1), p, p - target.get(i, p), (v[0] if v else "?"), (tp[0] if tp else "?"), (tq[0] if tq else "?")))
'
