#!/usr/bin/env python3
import time, rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
rclpy.init(); n = Node('motion_check'); v = []; imgs = []
n.create_subscription(Odometry, '/odom_raw', lambda m: v.append((m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z, m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
n.create_subscription(Image, '/depth_cam/rgb/image_raw', lambda m: imgs.append((time.monotonic(), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[::4, ::4].astype(np.int16))), qos_profile_sensor_data)
t = time.monotonic()
while time.monotonic() - t < 3: rclpy.spin_once(n, timeout_sec=0.1)
a = np.array(v) if v else np.zeros((1, 5))
print('odom twist mean |vx|=%.3f |vy|=%.3f |wz|=%.3f  pose moved %.3f m over 3 s (%d msgs)' % (np.abs(a[:, 0]).mean(), np.abs(a[:, 1]).mean(), np.abs(a[:, 2]).mean(), np.hypot(a[-1, 3] - a[0, 3], a[-1, 4] - a[0, 4]), len(v)))
if len(imgs) >= 2:
    d = np.abs(imgs[-1][1] - imgs[0][1]).mean(); print('camera frames %d, mean abs diff first->last = %.2f (static scene < ~2)' % (len(imgs), d))
else: print('camera frames: %d' % len(imgs))
n.destroy_node(); rclpy.shutdown()
