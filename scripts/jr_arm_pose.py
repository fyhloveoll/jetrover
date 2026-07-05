#!/usr/bin/env python3
# encoding: utf-8
# Move JetRover arm to a named pose via the vendor high-level servo bridge
# (publishes servo_controller_msgs/ServosPosition to /servo_controller).
# Does NOT touch vendor code. Default = observe/home pose (looking forward/down).
import sys
import time
import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition

# id -> pulse (0..1000 = 0..240deg). ids 1-5 = arm joints, 10 = gripper (200 open, 540 closed)
POSES = {
    # safe observe (track_and_grab values; joint3=100/joint4=120 are moderate,
    # NOT the extreme joint3=15 of automatic_pick that can stall from a curled start)
    'observe': [(1, 500), (2, 720), (3, 100), (4, 120), (5, 500)],
    'safe': [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500)],
    # automatic_pick observe: forearm extended (joint3=15) -> camera looks at the
    # FLOOR ahead (good for grasp detection). Reach via 'observe' first, NOT from curled.
    'floor': [(1, 500), (2, 700), (3, 15), (4, 175), (5, 500)],
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'observe'
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
    pose = list(POSES[name])
    # gripper (servo 10) is DECOUPLED: only commanded if explicitly given as a 3rd
    # arg (open|close|<pulse>), else left untouched so moving while holding an object
    # does NOT drop it.
    if len(sys.argv) > 3:
        g = sys.argv[3]
        gv = {'open': 200, 'close': 600}.get(g)
        pose = pose + [(10, gv if gv is not None else int(float(g)))]

    rclpy.init()
    node = Node('jr_arm_pose')
    pub = node.create_publisher(ServosPosition, 'servo_controller', 1)

    # wait for the controller_manager bridge to be subscribed before publishing
    t0 = time.time()
    while pub.get_subscription_count() < 1 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = ServosPosition()
    msg.duration = duration
    msg.position_unit = 'pulse'
    msg.position = [ServoPosition(id=i, position=float(p)) for i, p in pose]
    pub.publish(msg)
    node.get_logger().info(
        'sent pose "%s" %s over %.1fs, subs=%d'
        % (name, pose, duration, pub.get_subscription_count()))

    # keep node alive so the message is delivered and the motion completes
    t0 = time.time()
    while time.time() - t0 < duration + 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
