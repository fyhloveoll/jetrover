#!/usr/bin/env python3
# encoding: utf-8
# M5 grasp: YOLO target -> registered depth -> camera 3D -> (hand-eye + FK endpoint)
# -> arm-base 3D -> IK (/kinematics/set_pose_target) -> servo pulses -> grasp sequence.
# Follows the vendor RGBD-grasp convention (hand2cam matrix + get_current_pose), but
# is our own code calling vendor SERVICES; does not modify vendor code.
#
#   python3 jr_grasp.py            # DRY RUN: detect+transform+IK, print, NO arm motion
#   python3 jr_grasp.py grab       # EXECUTE: approach -> close -> lift -> return
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

import os
MODEL = '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'
TARGETS = set(os.environ.get('JR_TARGETS', 'bottle,cup,wine glass').split(','))
# eye-in-hand calibration: camera-optical -> gripper/end-effector (vendor track_and_grab)
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600


def depth_pixel_to_camera(u, v, z, fx, fy, cx, cy):
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])


def quat_to_mat(t, qwxyz):
    w, x, y, z = qwxyz
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


class Grasp(Node):
    def __init__(self):
        super().__init__('jr_grasp')
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._info, 1)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.ik.wait_for_service(timeout_sec=5.0)
        self.fk.wait_for_service(timeout_sec=5.0)
        self.model = YOLO(MODEL)

    def _rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def _depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def _info(self, m):
        self.K = list(m.k)

    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def wait_bridge(self, t=8.0):
        # the servo bridge (controller_manager) must have discovered our publisher
        # before we send, else commands are dropped (esp. after a wifi/DDS re-discovery)
        t0 = time.time()
        while self.joints.get_subscription_count() < 1 and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        n = self.joints.get_subscription_count()
        self.get_logger().info('servo bridge subscribers = %d' % n)
        return n

    def servos(self, dur, positions):
        msg = ServosPosition()
        msg.duration = float(dur)
        msg.position_unit = 'pulse'
        msg.position = [ServoPosition(id=i, position=float(p)) for i, p in positions]
        self.joints.publish(msg)
        # let the message actually go out before the next sleep/command
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_data(self, t=8.0):
        t0 = time.time()
        while (self.rgb is None or self.depth is None or self.K is None) and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.rgb is not None and self.depth is not None and self.K is not None

    def detect_target(self):
        # return the best target that has VALID depth (skip glass/edge invalid ones)
        res = self.model(self.rgb, conf=0.25, verbose=False)[0]
        names = res.names
        cw, ch = self.rgb.shape[1] / 2, self.rgb.shape[0] / 2
        cands = []
        skipped = []
        for b in res.boxes:
            cls = names[int(b.cls)]
            if cls not in TARGETS:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            dist = self.median_depth_m(u, v)
            d = ((u - cw) ** 2 + (v - ch) ** 2) ** 0.5
            if dist <= 0:
                skipped.append((cls, u, v))
                continue
            cands.append((d, u, v, cls, float(b.conf), dist))
        for cls, u, v in skipped:
            print('  skip %s @(%d,%d): invalid depth' % (cls, u, v))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        return cands[0]  # (dist_px, u, v, cls, conf, dist_m)

    def median_depth_m(self, u, v, win=5):
        d = self.depth
        h, w = d.shape
        patch = d[max(0, v - win):min(h, v + win + 1),
                  max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = patch[(patch > 0) & (patch < 10000)]
        return float(np.median(vals)) / 1000.0 if vals.size else 0.0

    def get_endpoint(self):
        r = self._call(self.fk, GetRobotPose.Request())
        p = r.pose.position
        o = r.pose.orientation
        return quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

    def target_to_armbase(self, u, v, dist):
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        cam = depth_pixel_to_camera(u, v, dist, fx, fy, cx, cy)
        cam[0] -= 0.01  # rgb/depth tf offset (vendor)
        endpoint = self.get_endpoint()
        world = endpoint @ HAND2CAM @ cam
        return world[:3], endpoint

    def solve_ik(self, pos, pitch):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in pos]
        req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]
        req.resolution = 1.0
        return self._call(self.ik, req)

    def pick(self, pos):
        if self.wait_bridge() < 1:
            self.get_logger().warn('servo bridge not connected; commands would be lost; abort')
            return False
        pitch = 80.0 if pos[2] < 0.2 else 30.0
        r = self.solve_ik(pos, pitch)
        if not (r and r.pulse):
            self.get_logger().warn('no IK solution for approach; abort')
            return False
        p = r.pulse
        self.get_logger().info('approach pulses %s' % list(p))
        self.servos(1.0, ((1, p[0]),))                 # base yaw first
        time.sleep(1.0)
        self.servos(1.5, ((1, p[0]), (2, p[1]), (3, p[2]), (4, p[3]), (5, p[4])))
        time.sleep(1.6)
        self.servos(0.6, ((10, GRIPPER_CLOSE),))       # close gripper
        time.sleep(1.0)
        lift = [pos[0], pos[1], pos[2] + 0.05]          # lift 5cm
        r2 = self.solve_ik(lift, pitch)
        if r2 and r2.pulse:
            q = r2.pulse
            self.servos(1.0, ((1, q[0]), (2, q[1]), (3, q[2]), (4, q[3]), (5, q[4])))
            time.sleep(1.2)
        self.get_logger().info('lifted; holding 2s then returning to observe (still gripping)')
        time.sleep(2.0)
        self.servos(1.5, OBSERVE[:5] + ((10, GRIPPER_CLOSE),))
        time.sleep(1.6)
        return True


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'dry'
    rclpy.init()
    node = Grasp()
    if not node.wait_data():
        print('ERROR: no camera/depth/info'); return
    tgt = node.detect_target()
    if tgt is None:
        print('no target with valid depth in view (targets=%s)' % sorted(TARGETS)); return
    _, u, v, cls, conf, dist = tgt
    dist += 0.03  # radius + error compensation (vendor)
    pos, endpoint = node.target_to_armbase(u, v, dist)
    print('target: %s conf=%.2f px=(%d,%d) dist=%.3fm' % (cls, conf, u, v, dist))
    print('endpoint(arm-base) xyz = %s' % np.round(endpoint[:3, 3], 3).tolist())
    print('GRASP point (arm-base) xyz = %s' % np.round(pos, 3).tolist())
    pitch = 80.0 if pos[2] < 0.2 else 30.0
    r = node.solve_ik(pos, pitch)
    if r and r.pulse:
        print('IK OK (pitch=%.0f): pulses=%s rpy=%s' % (pitch, list(r.pulse), [round(x, 1) for x in r.rpy]))
    else:
        print('IK: NO SOLUTION (success=%s)' % (getattr(r, 'success', None)))
    if mode == 'grab':
        if r and r.pulse:
            print('=== EXECUTING GRASP ===')
            node.pick(list(pos))
        else:
            print('no IK solution -> not executing')
    else:
        print('(dry run -- no motion. run "jr_grasp.py grab" to execute)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
