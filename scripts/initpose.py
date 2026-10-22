#!/usr/bin/env python3
"""initpose.py x y yaw_deg [cov_xy=0.05] [cov_yaw=0.05] -- publish /initialpose 3x with the given covariances"""
import math, sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
cxy = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
caa = float(sys.argv[5]) if len(sys.argv) > 5 else 0.05
rclpy.init(); n = Node('initpose'); pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
t = time.monotonic()
while pub.get_subscription_count() < 1 and time.monotonic() - t < 6: rclpy.spin_once(n, timeout_sec=0.2)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x = x; m.pose.pose.position.y = y
m.pose.pose.orientation.z = math.sin(math.radians(yaw) / 2); m.pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
cov = [0.0] * 36; cov[0] = cxy; cov[7] = cxy; cov[35] = caa; m.pose.covariance = cov
for _ in range(3):
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m)
    e = time.monotonic() + 0.3
    while time.monotonic() < e: rclpy.spin_once(n, timeout_sec=0.1)
print('initial pose (%.2f, %.2f, %.0f deg) cov xy=%.2f yaw=%.2f sent to %d subscriber(s)' % (x, y, yaw, cxy, caa, pub.get_subscription_count()))
n.destroy_node(); rclpy.shutdown()
