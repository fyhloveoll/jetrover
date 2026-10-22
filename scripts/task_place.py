#!/usr/bin/env python3
# encoding: utf-8
"""task_place.py -- ONE-FILE placement task: cube already in the closed gripper -> put it on the dark mat.
Runs to completion and exits; the exit code IS the result (jr_mission.py reads it).

  python3 task_place.py [--preflight] [--look-only] [--dry-run] [--handin] [--verify] [--max-legs N] [--no-liveness] [--debug-dir DIR]
    (no args)     full skill (what jr_mission.py calls after the delivery nav)
    --preflight   stack check only (bridge / camera / FK / scan / board liveness), prints their state
    --look-only   preflight + all 6 views (no early exit) + hull; no base motion, gripper untouched
    --dry-run     + cell choice with hover+drop IK; if no cell, prints the leg it would drive; no motion, gripper untouched
    --handin      staged take: arm to OBSERVE, open gripper (200), wait for ENTER, close (GRIPPER_CLOSE=600), then continue (needs a TTY)
    --verify      after the release, look down from FLOOR and report what is on the mat (informational)
    --max-legs N  base legs before giving up (default JR_PLACE_MAX_LEGS=3)
    --no-liveness skip the board liveness check (joint-2 wiggle + camera diff)
    --debug-dir D save one annotated jpg per view (<D>/<look#>_<OBS|FLR>_<servo1>.jpg; the look after a leg gets its own number)

Exit codes: 0 placed on the mat | 2 usage | 10 stack missing (bridge/camera/FK/scan) | 12 board dead |
  20 mat not seen in any view | 21 hull implausible (<8 pts or >0.45 m^2) | 30 scan stale / no beams |
  31 drive refused / leg or reverse budget exhausted | 32 time budget (JR_PLACE_BUDGET, 180 s) |
  40 mat centre in the approach window but no cell with hover+drop IK | 42 runtime error.
  NON-ZERO == the cube is still in the gripper, arm back at the carry pose (release by hand: python3 put_down.py).
  An error AFTER the gripper opened (place_skill.release() sets node.place_released) is logged as [PLACE/WARN]
  and exits 0: the cube is on the mat, only the post-release tidy-up (verify / carry pose) failed. A --handin
  interrupted before the cube was handed in says 'gripper OPEN -- hand-in interrupted' (non-zero, gripper empty).
Does NOT start any launch (CLAUDE.md rules 2/3: one bringup, PIDs are the boot script's business):
  a missing stack item exits 10 and prints the boot command to run.
"""
import argparse
import sys
import traceback

import rclpy
from geometry_msgs.msg import Twist                     # jr_grasp_all.py:27

sys.path.insert(0, '/home/ubuntu/jetrover_ws')         # put_down.py:4 (flat deploy)
import jr_grasp_all as ga                               # noqa: E402
import place_skill as ps                                # noqa: E402


def _settle(node, rc, why, handin_open):
    """One honest line about the gripper, park the arm, return the exit code.
    Header contract: NON-ZERO == cube still in the gripper. So an error that happened AFTER place_skill.release()
    opened the gripper (node.place_released, set right after the servo-10 write) is downgraded to 0 + WARN."""
    if getattr(node, 'place_released', False):
        ps.log('WARN', 'rc=0 cube RELEASED on the mat; error AFTER the release: %s (arm may not be at the carry pose)' % why)
        rc = ps.RC_OK
    elif handin_open:
        ps.log('FAIL', 'rc=%d %s (gripper OPEN -- hand-in interrupted, no cube in it)' % (rc, why))
    else:
        ps.log('FAIL', 'rc=%d %s (gripper untouched, cube kept)' % (rc, why))
    try:
        ps.carry_pose(node, 1.2)
    except Exception:
        pass
    return rc


def main():
    ap = argparse.ArgumentParser(description='put the cube held in the gripper on the dark mat (exit code = result, see file header)')
    ap.add_argument('--preflight', action='store_true', help='stack check only')
    ap.add_argument('--look-only', action='store_true', help='preflight + all views + hull, no motion')
    ap.add_argument('--dry-run', action='store_true', help='+ cell/IK choice or the leg it would drive, no motion')
    ap.add_argument('--handin', action='store_true', help='open gripper, wait for ENTER, close, then continue')
    ap.add_argument('--verify', action='store_true', help='after the release, report what is on the mat')
    ap.add_argument('--max-legs', type=int, default=ps.MAX_LEGS, help='base legs before giving up (default %d)' % ps.MAX_LEGS)
    ap.add_argument('--no-liveness', action='store_true', help='skip the board liveness check')
    ap.add_argument('--debug-dir', default=None, help='save one annotated jpg per view here')
    a = ap.parse_args()                                 # usage error -> argparse exits 2 (RC_USAGE)
    need_scan = not (a.preflight or a.look_only or a.dry_run)   # only the full skill may drive

    rclpy.init()
    node = ga.GraspAll()                                # jr_grasp_all.py:105
    rc = ps.RC_OK
    handin_open = False                                 # True while --handin holds the gripper open (no cube in it yet)
    try:
        ps.preflight(node, need_scan, liveness=not a.no_liveness)
        if a.handin:
            if not sys.stdin.isatty():
                raise ps.Fail(ps.RC_USAGE, '--handin needs a terminal: stdin is not a TTY, input() would raise EOFError '
                              '(mission_launch.sh starts with </dev/null)')
            ps.carry_pose(node, 1.5)
            node.move(0.6, ((10, ga.GRIPPER_OPEN),))                   # :183, :86
            handin_open = True
            ps.log('HANDIN', 'gripper OPEN -- hand the cube in, then press ENTER')   # [PLACE/ prefix: passes place_run.sh's grep
            input()
            node.move(0.8, ((10, ga.GRIPPER_CLOSE),))
            handin_open = False
        if a.preflight:
            ps.log('DONE', 'rc=0 preflight')
        else:
            rc = ps.run(node, dry=a.dry_run, look_only=a.look_only, max_legs=a.max_legs,
                        verify=a.verify, debug_dir=a.debug_dir)
            ps.log('DONE', 'rc=%d %s' % (rc, 'look-only' if a.look_only else ('dry-run' if a.dry_run else 'PLACED')))
    except ps.Fail as f:
        rc = _settle(node, f.rc, f.why, handin_open)
    except Exception as e:
        traceback.print_exc()
        rc = _settle(node, ps.RC_EXEC, 'unexpected %r' % (e,), handin_open)
    finally:
        if need_scan:                                   # only the modes that may drive: never leave a velocity behind an aborted drive()
            for _ in range(6):                          # :667-668
                node.cmd.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
