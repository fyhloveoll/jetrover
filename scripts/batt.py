#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
rclpy.init(); n = Node('batt'); v = []
n.create_subscription(UInt16, '/ros_robot_controller/battery', lambda m: v.append(m.data), 10)
t = time.monotonic()
while len(v) < 3 and time.monotonic() - t < 6: rclpy.spin_once(n, timeout_sec=0.2)
print('battery: %s' % (('%.2f V' % (sum(v) / len(v) / 1000.0)) if v else 'no data'))
n.destroy_node(); rclpy.shutdown()
