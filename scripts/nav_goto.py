#!/usr/bin/env python3
# encoding: utf-8
"""nav_goto.py -- send ONE NavigateToPose goal with a proper rclpy action client (the `ros2 action
send_goal` CLI hangs on discovery far too often on this robot) and report acceptance, result,
elapsed time and the AMCL pose at the end.

  python3 nav_goto.py <x> <y> [yaw_deg] [timeout_s]
exit code 0 = SUCCEEDED, 1 = otherwise
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class Goto(Node):
    def __init__(self):
        super().__init__('nav_goto')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._a, 1)
        self.last_fb = 0.0

    def _a(self, m):
        self.amcl = m.pose.pose

    def pose_str(self):
        if self.amcl is None:
            return 'amcl: (no message yet)'
        q = self.amcl.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return 'amcl: x=%.3f y=%.3f yaw=%.1f deg' % (self.amcl.position.x, self.amcl.position.y, yaw)

    def run(self, x, y, yaw_deg, timeout):
        if not self.client.wait_for_server(timeout_sec=15.0):
            print('navigate_to_pose action server NOT available'); return 1
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        t0 = time.time()
        print('goal: x=%.2f y=%.2f yaw=%.0f deg   (%s)' % (x, y, yaw_deg, self.pose_str()))

        def fb(msg):
            if time.time() - self.last_fb > 5.0:
                self.last_fb = time.time()
                f = msg.feedback
                print('  t=%4.1fs  distance_remaining=%.2f m  recoveries=%d' %
                      (time.time() - t0, f.distance_remaining, f.number_of_recoveries))

        fut = self.client.send_goal_async(goal, feedback_callback=fb)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('goal NOT accepted'); return 1
        print('goal accepted after %.1fs' % (time.time() - t0))
        rfut = gh.get_result_async()
        while rclpy.ok() and not rfut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                print('TIMEOUT after %.0fs -> cancelling' % timeout)
                gh.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=1.0)
                break
        status = rfut.result().status if rfut.done() else 0
        print('result: %s   elapsed %.1fs' % (STATUS.get(status, status), time.time() - t0))
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        print('end pose -> %s' % self.pose_str())
        return 0 if status == 4 else 1


def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    tmo = float(sys.argv[4]) if len(sys.argv) > 4 else 120.0
    rclpy.init()
    node = Goto()
    try:
        rc = node.run(x, y, yaw, tmo)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
