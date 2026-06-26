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
    'observe': [(1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)],
    'safe': [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (10, 200)],
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'observe'
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
    pose = POSES[name]

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
