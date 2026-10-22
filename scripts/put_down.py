#!/usr/bin/env python3
"""put_down.py [x=0.25] [y=0.0] -- pure motion: lower the held cube to (x, y, 0.07) in the ARM frame ahead of the robot and open the gripper. No vision."""
import sys, numpy as np, rclpy
sys.path.insert(0, '/home/ubuntu/jetrover_ws')
import jr_grasp_all as ga
x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
rclpy.init(); node = ga.GraspAll()
if node.wait_bridge() < 1: print('servo bridge not connected'); sys.exit(1)
node.place_n = 0
drop, _ = node._lower_and_release([x, y, 0.07], True)
print('released at %s' % np.round(drop, 3).tolist())
node.move(1.2, ((1, 500),) + ga.OBSERVE[1:])
node.destroy_node(); rclpy.shutdown()
