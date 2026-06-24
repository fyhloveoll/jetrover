#!/usr/bin/env python3
# Own gamepad teleop for JetRover. Subscribes the control-board Joy topic
# (/ros_robot_controller/joy) and republishes Twist to /controller/cmd_vel.
# Mecanum mapping mirrors vendor joystick_control (left stick = translate,
# right stick = rotate). Does NOT modify vendor code.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# control-board Joy axis order (from vendor): lx, ly, rx, ry, r2, l2, hat_x, hat_y
LX, LY, RX = 0, 1, 2


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('jr_joy_teleop')
        self.declare_parameter('max_linear', 0.4)    # m/s  (slow, good for mapping)
        self.declare_parameter('max_angular', 1.2)   # rad/s
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('timeout', 0.5)       # s, stop if no joy data
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.deadzone = self.get_parameter('deadzone').value
        self.timeout = self.get_parameter('timeout').value

        self.pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)
        self.sub = self.create_subscription(
            Joy, 'ros_robot_controller/joy', self.joy_cb, 1)
        self.last_joy = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)  # 10Hz deadman
        self.get_logger().info(
            'jr_joy_teleop up: left stick=move, right stick=turn, '
            'max %.2f m/s / %.2f rad/s' % (self.max_linear, self.max_angular))

    def _dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def joy_cb(self, msg):
        self.last_joy = self.get_clock().now()
        ax = msg.axes
        if len(ax) < 3:
            return
        t = Twist()
        t.linear.x = self._dz(ax[LY]) * self.max_linear   # forward/back
        t.linear.y = self._dz(ax[LX]) * self.max_linear   # strafe (mecanum)
        t.angular.z = self._dz(ax[RX]) * self.max_angular  # rotate
        self.pub.publish(t)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
        if dt > self.timeout:
            self.pub.publish(Twist())  # zero = stop


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
