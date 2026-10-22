#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('alive'); c = {'odom': 0, 'cmd': 0, 'scan': 0}; last = {}
n.create_subscription(Odometry, '/odom_raw', lambda m: c.__setitem__('odom', c['odom'] + 1), qos_profile_sensor_data)
n.create_subscription(Twist, '/controller/cmd_vel', lambda m: (c.__setitem__('cmd', c['cmd'] + 1), last.__setitem__('cmd', (round(m.linear.x, 3), round(m.linear.y, 3), round(m.angular.z, 3)))), 10)
n.create_subscription(LaserScan, '/scan', lambda m: c.__setitem__('scan', c['scan'] + 1), qos_profile_sensor_data)
t = time.monotonic()
while time.monotonic() - t < 3: rclpy.spin_once(n, timeout_sec=0.1)
print('3 s: odom_raw=%d scan=%d cmd_vel=%d last_cmd=%s' % (c['odom'], c['scan'], c['cmd'], last.get('cmd')))
n.destroy_node(); rclpy.shutdown()
