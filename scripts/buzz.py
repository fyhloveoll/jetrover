#!/usr/bin/env python3
# Beep the control board buzzer -- a direct board-actuation liveness test.
import time
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState

rclpy.init()
n = Node('jr_buzz')
pub = n.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
t0 = time.time()
while pub.get_subscription_count() < 1 and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
m = BuzzerState()
m.freq = 1000
m.on_time = 0.3
m.off_time = 0.2
m.repeat = 2
pub.publish(m)
print('sent buzzer, subs=%d' % pub.get_subscription_count())
for _ in range(25):
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node()
rclpy.shutdown()
