#!/usr/bin/env python3
"""Own keyboard teleop for JetRover. Publishes geometry_msgs/Twist to /controller/cmd_vel
(the low-level motor input; the high-level /cmd_vel is gated by vendor app layer).

Mecanum-aware: supports strafing. Deadman safety: stop pressing -> robot stops within ~0.5s.
"""
import sys
import select
import termios
import tty
import time

import rclpy
from geometry_msgs.msg import Twist

HELP = """
============ JetRover keyboard teleop ============
 publishes -> /controller/cmd_vel   (mecanum)

   w           w/s : forward / backward
 a s d         a/d : strafe left / right
               j/l : rotate left / right
   k (or space): stop now
   z/x : linear  speed  - / +
   ,/. : angular speed  - / +
   q   : quit (sends stop)

 DEADMAN: release keys and the robot stops within ~0.5s.
==================================================
"""

MOVE = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),    # strafe +y (left)
    'd': (0.0, -1.0, 0.0),   # strafe -y (right)
    'j': (0.0, 0.0, 1.0),    # yaw + (left)
    'l': (0.0, 0.0, -1.0),   # yaw - (right)
}


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('jr_keyboard_teleop')
    pub = node.create_publisher(Twist, '/controller/cmd_vel', 10)

    lin = 0.15   # m/s
    ang = 0.5    # rad/s
    deadman = 0.5
    vx = vy = wz = 0.0
    last_cmd = 0.0
    print(HELP)
    print(f"lin={lin:.2f} m/s  ang={ang:.2f} rad/s")
    try:
        while True:
            key = get_key(settings, 0.05)
            now = time.time()
            if key in MOVE:
                fx, fy, fz = MOVE[key]
                vx, vy, wz = fx * lin, fy * lin, fz * ang
                last_cmd = now
            elif key in ('k', ' '):
                vx = vy = wz = 0.0
                last_cmd = 0.0
            elif key == 'z':
                lin = max(0.02, lin - 0.02); print(f"lin={lin:.2f}")
            elif key == 'x':
                lin = min(0.5, lin + 0.02); print(f"lin={lin:.2f}")
            elif key == ',':
                ang = max(0.1, ang - 0.1); print(f"ang={ang:.2f}")
            elif key == '.':
                ang = min(2.0, ang + 0.1); print(f"ang={ang:.2f}")
            elif key == 'q':
                break
            # deadman
            if now - last_cmd > deadman:
                vx = vy = wz = 0.0
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            pub.publish(msg)
    except Exception as exc:  # noqa
        print(exc)
    finally:
        stop = Twist()
        for _ in range(5):
            pub.publish(stop)
            time.sleep(0.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nteleop stopped, robot commanded to halt.")


if __name__ == '__main__':
    main()
