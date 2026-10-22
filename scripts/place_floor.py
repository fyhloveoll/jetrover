#!/usr/bin/env python3
"""place_floor.py -- put the cube held in the gripper down on the floor straight ahead (the place routine's safe low release), no vision"""
import sys, time, numpy as np, rclpy
sys.path.insert(0, '/home/ubuntu/jetrover_ws')
import jr_grasp_all as ga
rclpy.init(); node = ga.GraspAll()
if node.wait_bridge() < 1: print('servo bridge not connected'); sys.exit(1)
node.fresh(t=5.0)
node.zone_hull = None
drop, on_zone = node.place_on_paper((0.0, 0.0))
print('released at %s' % np.round(drop, 3).tolist())
node.move(1.2, ((1, 500),) + ga.OBSERVE[1:])
node.destroy_node(); rclpy.shutdown()
