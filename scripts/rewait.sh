#!/usr/bin/env bash
# rewait.sh -- re-push stack_wait.py (monotonic clock) and run it against the already-launched stack
R=jetrover
REPO=/mnt/c/Users/86133/Desktop/C++/jetrover
scp -q $REPO/scripts/stack_wait.py $R:~/jetrover_ws/
ssh $R 'cd ~/jetrover_ws && sed -i "s/\r$//" stack_wait.py && python3 -m py_compile stack_wait.py && echo "stack_wait deployed"; timedatectl show -p NTPSynchronized -p TimeUSec 2>/dev/null | tr "\n" " "; echo'
ssh $R "bash -lc \"source ~/jetrover_ws/jr_env.sh; cd ~/jetrover_ws; python3 stack_wait.py $1 $2 $3 --timeout ${4:-150}\""
