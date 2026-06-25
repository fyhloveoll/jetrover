#!/usr/bin/env python3
# encoding: utf-8
# Map the arm's reachable workspace by probing /kinematics/set_pose_target over a
# grid of (x forward, z up) points in the arm-base frame, trying several pitches.
# Prints a reachability map so we know where a graspable object must sit.
import rclpy
from rclpy.node import Node
from kinematics_msgs.srv import SetRobotPose

XS = [0.10, 0.14, 0.18, 0.22, 0.26, 0.30, 0.34]
ZS = [0.20, 0.15, 0.10, 0.05, 0.00, -0.05, -0.10]
PITCHES = [0.0, 30.0, 60.0, 80.0, 90.0]


class Probe(Node):
    def __init__(self):
        super().__init__('ik_probe')
        self.cli = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.cli.wait_for_service(timeout_sec=5.0)

    def reach(self, x, z):
        for p in PITCHES:
            req = SetRobotPose.Request()
            req.position = [float(x), 0.0, float(z)]
            req.pitch = float(p)
            req.pitch_range = [-180.0, 180.0]
            req.resolution = 1.0
            fut = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=4.0)
            r = fut.result()
            if r and r.pulse:
                return int(p)
        return None


def main():
    rclpy.init()
    n = Probe()
    print('rows = z(up), cols = x(forward); value = min pitch that solves, "." = unreachable')
    header = 'z\\x  ' + ' '.join('%5.2f' % x for x in XS)
    print(header)
    for z in ZS:
        cells = []
        for x in XS:
            p = n.reach(x, z)
            cells.append('%5s' % ('.' if p is None else str(p)))
        print('%5.2f ' % z + ' '.join(cells))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
