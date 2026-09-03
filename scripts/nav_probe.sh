#!/usr/bin/env bash
# nav_probe.sh -- one nav goal WITH instrumentation: records /controller/cmd_vel and /amcl_pose
# during the goal, then reports settling time (from first entry into 0.3 m of the goal until
# SUCCEEDED), number of velocity sign reversals (vx, vy, wz) in that window, and controller/planner
# rate warnings. Usage: bash ~/jetrover_ws/nav_probe.sh <x> <y> [yaw_deg] [timeout_s]
# shellcheck disable=SC1090
source ~/jetrover_ws/jr_env.sh
X=${1:?}; Y=${2:?}; YAW=${3:-0}; TMO=${4:-120}
WS=~/jetrover_ws
rm -f $WS/probe_cmd.txt $WS/probe_pose.txt
N0=$(grep -c "missed its desired rate" ~/nav.log 2>/dev/null || echo 0)
setsid bash -c "timeout $((TMO+5)) ros2 topic echo /controller/cmd_vel > $WS/probe_cmd.txt 2>&1" </dev/null >/dev/null 2>&1 &
setsid bash -c "timeout $((TMO+5)) ros2 topic echo /amcl_pose > $WS/probe_pose.txt 2>&1" </dev/null >/dev/null 2>&1 &
sleep 2
bash $WS/nav_goal.sh "$X" "$Y" "$YAW" "$TMO"
sleep 1
pkill -TERM -f "ros2 topic echo /controller/cmd_vel" 2>/dev/null; pkill -TERM -f "ros2 topic echo /amcl_pose" 2>/dev/null
N1=$(grep -c "missed its desired rate" ~/nav.log 2>/dev/null || echo 0)
echo "rate warnings during goal: $((N1-N0))   (controller: $(grep -c 'Control loop missed' ~/nav.log), planner: $(grep -c 'Planner loop missed' ~/nav.log) total)"
python3 - "$X" "$Y" <<'PYEOF'
import re, sys, os, math
gx, gy = float(sys.argv[1]), float(sys.argv[2])
ws = os.path.expanduser('~/jetrover_ws')
cmd = open(ws + '/probe_cmd.txt').read().split('---')
vs = []
for blk in cmd:
    m = re.search(r'linear:\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+).*?angular:.*?z:\s*([-\d.e]+)', blk, re.S)
    if m:
        vs.append(tuple(float(v) for v in m.groups()))
def reversals(seq, eps):
    last = 0; n = 0
    for v in seq:
        s = 1 if v > eps else (-1 if v < -eps else 0)
        if s and last and s != last:
            n += 1
        if s:
            last = s
    return n
print('cmd_vel samples: %d' % len(vs))
if vs:
    vx = [v[0] for v in vs]; vy = [v[1] for v in vs]; wz = [v[2] for v in vs]
    print('  reversals: vx %d  vy %d  wz %d   (a reversal = sign flip of a non-zero command)' %
          (reversals(vx, 0.01), reversals(vy, 0.01), reversals(wz, 0.05)))
    print('  |vx|max %.2f  |vy|max %.2f  |wz|max %.2f   nonzero share %.0f%%' %
          (max(map(abs, vx)), max(map(abs, vy)), max(map(abs, wz)),
           100.0 * sum(1 for v in vs if abs(v[0]) > 0.01 or abs(v[1]) > 0.01 or abs(v[2]) > 0.05) / len(vs)))
pose = open(ws + '/probe_pose.txt').read().split('---')
ds = []
for blk in pose:
    m = re.search(r'position:\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)', blk)
    if m:
        ds.append(math.hypot(float(m.group(1)) - gx, float(m.group(2)) - gy))
if ds:
    first_near = next((i for i, d in enumerate(ds) if d < 0.3), None)
    print('amcl samples: %d, final distance to goal %.2f m' % (len(ds), ds[-1]))
    if first_near is not None:
        print('  samples after first entering 0.3 m: %d of %d (amcl publishes only on motion, so this ~ settling activity)'
              % (len(ds) - first_near, len(ds)))
PYEOF
