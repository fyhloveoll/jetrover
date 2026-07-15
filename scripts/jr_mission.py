#!/usr/bin/env python3
# encoding: utf-8
# M6 finale: NAVIGATE -> GRASP -> DELIVER mission orchestrator.
# Requires running: full bringup (lidar+camera) + kinematics + jr_nav nav.launch.py.
#
#   python3 jr_mission.py record
#       Continuously print the AMCL pose. Drive the robot around with teleop and
#       write down the waypoints you like, then put them in mission.json.
#
#   python3 jr_mission.py run
#       Execute ~/jetrover_ws/mission.json:
#         {"mode": "carry",                  # "carry" = grab ONE cube at the zone and
#          "zones": [[x, y, yaw_deg], ...],  #   deliver it to "delivery"; "clear" =
#          "delivery": [x, y, yaw_deg]}      #   grasp-all onto the local paper per zone
#
# Notes: runs ON the robot (timestamps from the robot clock, no cross-machine skew);
# the arm travels in the OBSERVE pose (also the carry pose -- gripper stays closed).
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

MISSION = os.path.expanduser('~/jetrover_ws/mission.json')
GRASP = os.path.expanduser('~/jetrover_ws/jr_grasp_all.py')
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500))
FLOOR = ((1, 500), (2, 700), (3, 15), (4, 175), (5, 500))   # known-safe low pose (delivery lowering)
NAV_TIMEOUT = float(os.environ.get('JR_NAV_TIMEOUT', '150'))


class Mission(Node):
    def __init__(self):
        super().__init__('jr_mission')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._a, 1)

    def _a(self, m):
        self.amcl = m.pose.pose

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)

    def arm(self, positions, gripper=None, dur=2.5):
        t0 = time.time()
        while self.joints.get_subscription_count() < 1 and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        msg = ServosPosition(); msg.duration = float(dur); msg.position_unit = 'pulse'
        pos = list(positions) + ([(10, gripper)] if gripper is not None else [])
        msg.position = [ServoPosition(id=i, position=float(p)) for i, p in pos]
        self.joints.publish(msg)
        self.spin(dur + 0.5)

    def goto(self, x, y, yaw_deg):
        # send a NavigateToPose goal and wait for the result
        if not self.nav.wait_for_server(timeout_sec=8.0):
            print('!! nav2 action server not available (is nav.launch running?)')
            return False
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'      # stamp left 0: robot-local clock, AMCL-friendly
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        yaw = math.radians(float(yaw_deg))
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        print('[NAV] -> (%.2f, %.2f, %.0fdeg)' % (x, y, yaw_deg))
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('!! nav goal rejected')
            return False
        rfut = gh.get_result_async()
        t0 = time.time()
        while not rfut.done() and time.time() - t0 < NAV_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not rfut.done():
            print('!! nav timeout after %.0fs -> canceling' % NAV_TIMEOUT)
            gh.cancel_goal_async()
            self.spin(2)
            return False
        status = rfut.result().status
        ok = (status == 4)   # STATUS_SUCCEEDED
        print('[NAV] result status=%d %s' % (status, 'OK' if ok else 'FAILED'))
        return ok


def run_grasp(carry, corridor=None):
    # one long-lived grasp process per batch (the zero-lockup discipline)
    env = dict(os.environ)
    env['JR_DUR'] = env.get('JR_DUR', '2.0')
    if carry:
        env['JR_CARRY'] = '1'
    if corridor:
        env['JR_CORRIDOR'] = corridor   # arm-frame unit vector toward the delivery
    print('[GRASP] starting %s (carry=%s)' % (GRASP, carry))
    r = subprocess.run(['python3', GRASP, 'run'], env=env,
                       capture_output=True, text=True, timeout=700)
    tail = [l for l in r.stdout.splitlines()
            if not any(w in l for w in ('Warning', 'deprecat'))][-14:]
    print('\n'.join('  | ' + l for l in tail))
    carried = 'CARRYING' in r.stdout
    grabbed = 0
    for l in r.stdout.splitlines():
        if 'grabbed' in l:
            try:
                grabbed = int(l.split('grabbed')[1].split('/')[0])
            except Exception:
                pass
    return grabbed, carried


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'run'
    rclpy.init()
    node = Mission()

    if mode == 'record':
        print('drive around with teleop; AMCL poses print here (Ctrl-C to stop):')
        try:
            while rclpy.ok():
                node.spin(1.0)
                p = node.amcl
                if p is None:
                    print('  (no /amcl_pose yet -- nav stack running?)')
                else:
                    yaw = math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w))
                    print('  pose: [%.2f, %.2f, %.0f]' % (p.position.x, p.position.y, yaw))
        except KeyboardInterrupt:
            pass
        node.destroy_node(); rclpy.shutdown(); return

    if not os.path.exists(MISSION):
        print('no %s -- create it first (see header). Example:' % MISSION)
        print('  {"mode": "carry", "zones": [[1.2, 0.3, 0]], "delivery": [0.0, 0.0, 180]}')
        node.destroy_node(); rclpy.shutdown(); return
    cfg = json.load(open(MISSION))
    carry = cfg.get('mode', 'carry') == 'carry'
    zones = cfg.get('zones', [])
    delivery = cfg.get('delivery')

    total = 0
    for i, z in enumerate(zones):
        print('== zone %d/%d ==' % (i + 1, len(zones)))
        node.arm(OBSERVE, gripper=200)              # travel pose, gripper open
        if not node.goto(*z):
            print('!! skip zone %d (nav failed)' % (i + 1)); continue
        corridor = None
        if carry and delivery:
            # delivery bearing in the robot's arm frame at this zone (clear-the-road
            # grasp ordering: the carry trips drive through cleared ground)
            th = math.radians(z[2])
            vx, vy = delivery[0] - z[0], delivery[1] - z[1]
            ax = math.cos(th) * vx + math.sin(th) * vy
            ay = -math.sin(th) * vx + math.cos(th) * vy
            n = math.hypot(ax, ay)
            if n > 0.2:
                corridor = '%.3f,%.3f' % (ax / n, ay / n)
        grabbed, carried = run_grasp(carry, corridor)
        total += grabbed
        if carry and carried:
            print('[MISSION] cube in gripper -> delivering')
            # arm is already at OBSERVE holding the cube (grasp ends there)
            if delivery and node.goto(*delivery):
                # vision takes over for the last half-metre: pan-scan for the drop
                # zone (dark mat + AprilTag) and place ON it -- nav only parks us
                # nearby (AMCL+goal tolerance is +-20cm, blind release scatters)
                r = subprocess.call(['python3', GRASP, 'place'])
                if r != 0:                          # zone place failed: safe low release
                    node.arm(FLOOR, dur=2.5)
                    node.arm(((10, 200),), dur=1.0)
                node.arm(OBSERVE, gripper=200)
                print('[MISSION] delivered.')
            else:
                print('!! delivery nav failed -- releasing here')
                node.arm(((10, 200),), dur=1.0)
    if not carry and delivery:
        node.arm(OBSERVE, gripper=200)
        node.goto(*delivery)
    print('===== MISSION DONE ===== grabbed total=%d' % total)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
