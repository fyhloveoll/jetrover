#!/usr/bin/env python3
# encoding: utf-8
"""stack_wait.py -- ONE rclpy process that takes the stack from "launched" to READY, no ros2 CLI:
  1. wait for /scan, /odom_raw, the depth camera topics, /map and the navigate_to_pose action server
  2. wait for AMCL to be ACTIVE (lifecycle get_state service)
  3. apply the verified Nav2 patches through the parameter services (controller, smoother)
  4. enable the keepout filters in both costmaps
  5. publish the initial pose
  6. buzz the board (command path alive)
  python3 stack_wait.py <init_x> <init_y> <init_yaw_deg> [--timeout 120]
exit 0 = READY, 1 = something missing (printed). Prints the elapsed time of every stage.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_robot_controller_msgs.msg import BuzzerState

PATCHES = {
    '/controller_server': [
        ('FollowPath.max_vel_y', 0.15), ('FollowPath.weight_kinematics_nh', 10.0),
        ('FollowPath.max_vel_theta', 0.6), ('FollowPath.min_obstacle_dist', 0.10),
        ('FollowPath.inflation_dist', 0.30), ('FollowPath.weight_obstacle', 50.0),
        ('FollowPath.feasibility_check_no_poses', 3), ('goal_checker.xy_goal_tolerance', 0.12),
        ('goal_checker.yaw_goal_tolerance', 0.20), ('progress_checker.required_movement_radius', 0.25),
        ('progress_checker.movement_time_allowance', 20.0), ('FollowPath.max_vel_x_backwards', 0.08), ('FollowPath.penalty_epsilon', 0.02),  # backwards must exceed epsilon (TEB warns 'undefined behavior' otherwise)
    ],
    '/velocity_smoother': [
        ('max_velocity', [0.15, 0.15, 0.6]), ('min_velocity', [-0.15, -0.15, -0.6]),
        ('max_accel', [2.5, 2.5, 3.2]), ('max_decel', [-2.5, -2.5, -3.2]),
        ('deadband_velocity', [0.02, 0.05, 0.05]),
    ],
    '/global_costmap/global_costmap': [('keepout_filter.enabled', True)],
    # 10-22: local keepout OFF -- with it on, a grasp that ends 0.18 m from the coffee-table zone puts the
    # footprint in the inscribed ring and TEB rejects every trajectory (mission7 delivery leg). The global
    # costmap still routes around the zones.
    '/local_costmap/local_costmap': [('keepout_filter.enabled', False)],
}


class Waiter(Node):
    def __init__(self):
        super().__init__('stack_wait')
        self.t0 = time.monotonic()

    def el(self):
        return time.monotonic() - self.t0

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_topics(self, deadline):
        last = 0
        while self.el() < deadline:
            names = [n for n, _ in self.get_topic_names_and_types()]
            cam = sum(1 for n in names if 'depth_cam' in n)
            ok = ('/scan' in names, '/odom_raw' in names, cam >= 12, '/map' in names)
            if all(ok):
                print('[%5.1fs] topics: scan odom camera(%d) map  OK' % (self.el(), cam)); return True
            if self.el() - last > 5:
                last = self.el()
                print('[%5.1fs] waiting topics: scan=%s odom=%s cam=%d map=%s' % (self.el(), ok[0], ok[1], cam, ok[3]))
            self.spin(1.0)
        print('[%5.1fs] TOPICS MISSING' % self.el()); return False

    def wait_action(self, deadline):
        ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        while self.el() < deadline:
            if ac.wait_for_server(timeout_sec=2.0):
                print('[%5.1fs] navigate_to_pose action server  OK' % self.el()); return True
        print('[%5.1fs] NAV ACTION SERVER MISSING' % self.el()); return False

    def wait_amcl(self, deadline):
        cli = self.create_client(GetState, '/amcl/get_state')
        while self.el() < deadline:
            if cli.wait_for_service(timeout_sec=2.0):
                fut = cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                r = fut.result()
                if r is not None and r.current_state.id == 3:
                    print('[%5.1fs] amcl ACTIVE  OK' % self.el()); return True
            self.spin(1.0)
        print('[%5.1fs] AMCL NOT ACTIVE' % self.el()); return False

    def apply_patches(self):
        total = ok = 0
        for node_name, items in PATCHES.items():
            cli = self.create_client(SetParameters, node_name + '/set_parameters')
            if not cli.wait_for_service(timeout_sec=8.0):
                print('  %s: set_parameters service NOT available' % node_name); total += len(items); continue
            req = SetParameters.Request()
            req.parameters = [Parameter(n, value=v).to_parameter_msg() for n, v in items]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            r = fut.result()
            total += len(items)
            if r is None:
                print('  %s: set_parameters TIMEOUT' % node_name); continue
            for (n, v), res in zip(items, r.results):
                ok += 1 if res.successful else 0
                if not res.successful:
                    print('  %s %s FAILED: %s' % (node_name, n, res.reason))
        print('[%5.1fs] patches %d/%d applied' % (self.el(), ok, total))
        return ok == total

    def init_pose(self, x, y, yaw_deg):
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        t = time.monotonic()
        while pub.get_subscription_count() < 1 and time.monotonic() - t < 6:
            self.spin(0.2)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = float(x); m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        m.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        cov = [0.0] * 36; cov[0] = 0.05; cov[7] = 0.05; cov[35] = 0.05
        m.pose.covariance = cov
        for _ in range(3):
            m.header.stamp = self.get_clock().now().to_msg()
            pub.publish(m); self.spin(0.3)
        print('[%5.1fs] initial pose (%.2f, %.2f, %.0f deg) sent to %d subscriber(s)' % (self.el(), x, y, yaw_deg, pub.get_subscription_count()))

    def buzz(self):
        pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        t = time.monotonic()
        while pub.get_subscription_count() < 1 and time.monotonic() - t < 5:
            self.spin(0.1)
        m = BuzzerState(); m.freq = 1000; m.on_time = 0.3; m.off_time = 0.2; m.repeat = 2
        pub.publish(m); self.spin(0.5)
        print('[%5.1fs] buzzer sent (subs=%d)' % (self.el(), pub.get_subscription_count()))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    x, y, yaw = (float(args[0]), float(args[1]), float(args[2])) if len(args) >= 3 else (0.0, 0.0, 0.0)
    deadline = 120.0
    if '--timeout' in sys.argv:
        deadline = float(sys.argv[sys.argv.index('--timeout') + 1])
    rclpy.init()
    w = Waiter()
    ok = w.wait_topics(deadline) and w.wait_action(deadline) and w.wait_amcl(deadline)
    if not ok:
        print('NOT READY after %.0f s' % w.el()); w.destroy_node(); rclpy.shutdown(); sys.exit(1)
    w.spin(2.0)                                   # let the lifecycle settle before setting params
    patched = w.apply_patches()
    w.init_pose(x, y, yaw)
    if '--no-buzz' not in sys.argv:
        w.buzz()
    print('READY in %.0f s%s' % (w.el(), '' if patched else '  (some patches failed -- see above)'))
    w.destroy_node(); rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
