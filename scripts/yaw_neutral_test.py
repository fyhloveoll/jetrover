#!/usr/bin/env python3
# encoding: utf-8
# Wrist-neutral calibration aid: move the arm to a straight-ahead grasp pose with the
# wrist at a given pulse (default 504), gripper open, and HOLD, so the user can look
# from ABOVE and judge the fingers' closing-axis skew vs. straight-across.
#   python3 yaw_neutral_test.py [servo5_pulse]
import sys
import time
import rclpy
from rclpy.node import Node
from kinematics_msgs.srv import SetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

S5 = int(sys.argv[1]) if len(sys.argv) > 1 else 504

rclpy.init()
n = Node('yaw_neutral_test')
pub = n.create_publisher(ServosPosition, 'servo_controller', 1)
ik = n.create_client(SetRobotPose, '/kinematics/set_pose_target')
ik.wait_for_service(timeout_sec=5.0)
t0 = time.time()
while pub.get_subscription_count() < 1 and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

req = SetRobotPose.Request()
req.position = [0.24, 0.0, 0.06]
req.pitch = 80.0
req.pitch_range = [-180.0, 180.0]
req.resolution = 1.0
fut = ik.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
r = fut.result()
if not (r and r.pulse):
    print('IK failed'); sys.exit(1)
p = list(r.pulse)
p[4] = S5
m = ServosPosition(); m.duration = 2.5; m.position_unit = 'pulse'
m.position = [ServoPosition(id=i + 1, position=float(v)) for i, v in enumerate(p[:5])]
m.position.append(ServoPosition(id=10, position=200.0))
pub.publish(m)
print('arm -> straight-ahead grasp pose, servo5=%d, gripper open. LOOK FROM ABOVE:' % S5)
print('fingers should close exactly LEFT-RIGHT (perpendicular to forward). holding...')
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
