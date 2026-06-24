#!/usr/bin/env python3
# Bridge Nav2 output (/cmd_vel) to JetRover motor command (/controller/cmd_vel).
# JetRover's /cmd_vel is gated and does not drive motors; /controller/cmd_vel does.
# Does not modify vendor code.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.get_logger().info('cmd_vel_relay: /cmd_vel -> /controller/cmd_vel')

    def cb(self, msg):
        self.pub.publish(msg)


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
