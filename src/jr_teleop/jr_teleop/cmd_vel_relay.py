#!/usr/bin/env python3
# Bridge Nav2 output (/cmd_vel) to JetRover motor command (/controller/cmd_vel).
# JetRover's /cmd_vel is gated and does not drive motors; /controller/cmd_vel does.
# Does not modify vendor code.
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        # Speed SAFETY NET at the last hop before the motors (guaranteed no matter
        # which Nav2 nodes exist). PROPORTIONAL scaling only -- per-axis clamping
        # changes the ratio between components, i.e. the direction and curvature of
        # the commanded motion, and the planner's corrections get distorted the same
        # way (predicted oscillation driver; quantified: curvature error up to +67%
        # at these limits -- see docs/engineering_decisions.md D6). The proper speed
        # limit belongs in the controller's own params; this net should normally
        # never engage.
        self.max_lin = float(os.environ.get('JR_MAX_LIN', '0.15'))
        self.max_ang = float(os.environ.get('JR_MAX_ANG', '0.6'))
        self.pub = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.get_logger().info('cmd_vel_relay: /cmd_vel -> /controller/cmd_vel '
                               '(proportional cap lin %.2f ang %.2f)' % (self.max_lin, self.max_ang))

    def cb(self, msg):
        s = 1.0
        lin = math.hypot(msg.linear.x, msg.linear.y)
        if lin > self.max_lin > 0:
            s = min(s, self.max_lin / lin)
        if abs(msg.angular.z) > self.max_ang > 0:
            s = min(s, self.max_ang / abs(msg.angular.z))
        m = Twist()
        m.linear.x = msg.linear.x * s
        m.linear.y = msg.linear.y * s
        m.angular.z = msg.angular.z * s
        self.pub.publish(m)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
