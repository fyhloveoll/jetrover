#!/usr/bin/env python3
# Bridge Nav2 output (/cmd_vel) to JetRover motor command (/controller/cmd_vel).
# JetRover's /cmd_vel is gated and does not drive motors; /controller/cmd_vel does.
# Does not modify vendor code.
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        # HARD speed ceiling at the last hop before the motors: guaranteed no matter
        # which Nav2 nodes exist (a velocity_smoother param edit was likely a no-op
        # because the smoother node may not even run -- "还是不慢", 07-15)
        self.max_lin = float(os.environ.get('JR_MAX_LIN', '0.15'))
        self.max_ang = float(os.environ.get('JR_MAX_ANG', '0.6'))
        self.pub = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.get_logger().info('cmd_vel_relay: /cmd_vel -> /controller/cmd_vel '
                               '(clamp lin %.2f ang %.2f)' % (self.max_lin, self.max_ang))

    def cb(self, msg):
        m = Twist()
        m.linear.x = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        m.linear.y = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        m.angular.z = max(-self.max_ang, min(self.max_ang, msg.angular.z))
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
