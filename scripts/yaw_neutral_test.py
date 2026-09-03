#!/usr/bin/env python3
# encoding: utf-8
# Wrist-yaw calibration aid (phase B of the paper-line rig, 2026-09).
# Moves the arm to a straight-ahead grasp pose over the calibration sheet, sets the wrist
# (servo5) to NEUTRAL + GAIN*angle, opens the gripper and HOLDS, so you can judge from
# ABOVE whether the fingers' closing axis is perpendicular to the line drawn at `angle`.
#
#   python3 yaw_neutral_test.py                       # wrist at neutral (default 504): axis must be
#                                                     #   perpendicular to the 0 deg line
#   python3 yaw_neutral_test.py --angle 30            # wrist should align with the +30 deg line
#   python3 yaw_neutral_test.py --angle -30 --neutral 510 --gain -4.17
#   python3 yaw_neutral_test.py --y 0.08              # target 8cm to the LEFT: base turns, wrist
#                                                     #   offset must stay relative to that bearing
#
# Angle convention (same as the detector): seen from above, positive = counter-clockwise
# from the arm's forward axis (toward the robot's LEFT). Cube angles are mod 90.
# Neutral/gain found here go into JR_YAW_NEUTRAL / JR_YAW_GAIN for jr_grasp_all.py.
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from kinematics_msgs.srv import SetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

ap = argparse.ArgumentParser()
ap.add_argument('--angle', type=float, default=0.0, help='line angle to align with (deg, CCW+)')
ap.add_argument('--neutral', type=float, default=504.0, help='servo5 pulse for 0 deg')
ap.add_argument('--gain', type=float, default=4.17, help='servo5 pulse per deg (sign matters)')
ap.add_argument('--x', type=float, default=0.24, help='target x in arm frame (m, forward)')
ap.add_argument('--y', type=float, default=0.0, help='target y in arm frame (m, left+)')
ap.add_argument('--z', type=float, default=0.06, help='target z in arm frame (m)')
ap.add_argument('--hold', type=float, default=6.0, help='seconds to hold the pose')
a = ap.parse_args()

# the gripper's closing axis rotates with the base, so the wrist offset is RELATIVE to the
# approach bearing gamma = atan2(y, x) (pure geometry, no servo1 pulse model needed)
gamma = math.degrees(math.atan2(a.y, a.x))
s5 = int(max(0, min(1000, a.neutral + a.gain * (a.angle - gamma))))

rclpy.init()
n = Node('yaw_neutral_test')
pub = n.create_publisher(ServosPosition, 'servo_controller', 1)
ik = n.create_client(SetRobotPose, '/kinematics/set_pose_target')
ik.wait_for_service(timeout_sec=5.0)
t0 = time.time()
while pub.get_subscription_count() < 1 and time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

req = SetRobotPose.Request()
req.position = [a.x, a.y, a.z]
req.pitch = 80.0
req.pitch_range = [-180.0, 180.0]
req.resolution = 1.0
fut = ik.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
r = fut.result()
if not (r and r.pulse):
    print('IK failed for', req.position); raise SystemExit(1)
p = list(r.pulse)
p[4] = s5
m = ServosPosition(); m.duration = 2.5; m.position_unit = 'pulse'
m.position = [ServoPosition(id=i + 1, position=float(v)) for i, v in enumerate(p[:5])]
m.position.append(ServoPosition(id=10, position=200.0))
pub.publish(m)
print('target (%.2f, %.2f, %.2f)  bearing gamma=%+.1f deg  line angle=%+.1f deg' % (a.x, a.y, a.z, gamma, a.angle))
print('servo5 = %.0f + %.2f * (%+.1f - %+.1f) = %d   (IK servo1=%d)' % (a.neutral, a.gain, a.angle, gamma, s5, p[0]))
print('LOOK FROM ABOVE: finger closing axis should be PERPENDICULAR to the %+.0f deg line. holding %.0fs...' % (a.angle, a.hold))
t0 = time.time()
while time.time() - t0 < a.hold:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
