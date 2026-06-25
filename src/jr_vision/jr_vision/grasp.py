#!/usr/bin/env python3
# encoding: utf-8
# jr_vision grasp node (M5): pick a detected object using the eye-in-hand pipeline.
#   YOLO target -> registered depth -> camera 3D -> (HAND2CAM hand-eye + FK endpoint)
#   -> arm-base 3D -> IK (/kinematics/set_pose_target) -> servo pulses -> grasp.
# Follows the vendor RGBD-grasp convention (track_and_grab) but is our own code
# calling vendor SERVICES; does not modify vendor code.
#
# Hardened from the throwaway jr_grasp.py with the lessons from 2026-06-25:
#  - board-liveness guard: refuse to send if /imu has gone silent (board hung)
#  - servo throttle: enforce a minimum gap between bus-servo commands
#  - wait for the servo bridge to subscribe before sending (else commands drop)
#  - clean shutdown on SIGINT/SIGTERM (never leave the serial mid-transaction)
#
# Trigger a grasp:  ros2 service call /jr/grasp/trigger std_srvs/srv/Trigger
# Or one-shot:      ros2 launch jr_vision grasp.launch.py auto_grab:=true dry_run:=true
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition

# eye-in-hand calibration: camera-optical -> gripper/end-effector (vendor track_and_grab)
HAND2CAM = np.array([[0.0, 0.0, 1.0, -0.101],
                     [-1.0, 0.0, 0.0, 0.011],
                     [0.0, -1.0, 0.0, 0.045],
                     [0.0, 0.0, 0.0, 1.0]])
OBSERVE = ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500))
GRIPPER_OPEN, GRIPPER_CLOSE = 200, 600
RGB_DEPTH_X_OFFSET = -0.01   # vendor: rgb/depth camera TFs differ ~1cm in x


def quat_to_mat(t, qwxyz):
    w, x, y, z = qwxyz
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def arm_servos(pulse):
    # IK pulse array [j1..j5] -> ((1,p0),(2,p1),...,(5,p4)) for set_servo_position
    return tuple((i + 1, int(pulse[i])) for i in range(5))


class GraspNode(Node):
    def __init__(self):
        super().__init__('jr_grasp')
        self.declare_parameter('model_path', '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt')
        self.declare_parameter('target_classes', ['bottle', 'cup', 'wine glass'])
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('auto_grab', False)
        self.declare_parameter('servo_min_interval', 0.4)   # min gap between bus-servo cmds (s)
        self.declare_parameter('imu_timeout', 1.5)          # /imu older than this => board hung
        self.declare_parameter('depth_compensation', 0.03)  # radius + error push along view ray (m)
        g = self.get_parameter
        self.targets = set(g('target_classes').value)
        self.conf = float(g('conf').value)
        self.dry_run = bool(g('dry_run').value)
        self.servo_gap = float(g('servo_min_interval').value)
        self.imu_timeout = float(g('imu_timeout').value)
        self.depth_comp = float(g('depth_compensation').value)

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.last_imu = 0.0
        self._last_servo = 0.0
        self._lock = threading.Lock()       # serialize grasp attempts (atomic, no race)

        cb = ReentrantCallbackGroup()
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._rgb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._depth, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._info, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data, callback_group=cb)
        self.joints = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.status = self.create_publisher(String, '/jr/grasp/status', 1)
        self.ik = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=cb)
        self.fk = self.create_client(GetRobotPose, '/kinematics/get_current_pose', callback_group=cb)
        self.srv = self.create_service(Trigger, '/jr/grasp/trigger', self._on_trigger, callback_group=cb)

        for cli, name in ((self.ik, 'set_pose_target'), (self.fk, 'get_current_pose')):
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('kinematics service /kinematics/%s not up yet '
                                       '(launch kinematics_node first)' % name)

        from ultralytics import YOLO
        self.get_logger().info('loading YOLO %s ...' % g('model_path').value)
        self.model = YOLO(g('model_path').value)
        self.get_logger().info('jr_grasp ready: targets=%s dry_run=%s (call /jr/grasp/trigger)'
                               % (sorted(self.targets), self.dry_run))
        if bool(g('auto_grab').value):
            threading.Thread(target=self._auto, daemon=True).start()

    # ---- subscriptions -------------------------------------------------
    def _rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')

    def _depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def _info(self, m):
        self.K = list(m.k)

    def _imu(self, m):
        self.last_imu = time.time()

    def _say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    # ---- helpers -------------------------------------------------------
    def board_alive(self):
        # True only once /imu has been seen AND is recent. last_imu==0 => never seen
        # (board down / topic missing), which is NOT the same as "was alive, now hung".
        return self.last_imu > 0.0 and (time.time() - self.last_imu) < self.imu_timeout

    def wait_ready(self, t=8.0):
        # wait for camera (rgb+depth+info) and the first /imu before attempting a grasp
        t0 = time.time()
        while time.time() - t0 < t:
            if (self.rgb is not None and self.depth is not None
                    and self.K is not None and self.last_imu > 0.0):
                return True
            time.sleep(0.05)
        return False

    def _await(self, fut, t=5.0):
        # a separate MultiThreadedExecutor thread services the future; we just poll done()
        t0 = time.time()
        while not fut.done() and time.time() - t0 < t:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def servos(self, dur, positions):
        gap = self.servo_gap - (time.time() - self._last_servo)   # throttle: never burst the MCU
        if gap > 0:
            time.sleep(gap)
        msg = ServosPosition()
        msg.duration = float(dur)
        msg.position_unit = 'pulse'
        msg.position = [ServoPosition(id=i, position=float(p)) for i, p in positions]
        self.joints.publish(msg)
        self._last_servo = time.time()

    def wait_bridge(self, t=8.0):
        t0 = time.time()
        while self.joints.get_subscription_count() < 1 and time.time() - t0 < t:
            time.sleep(0.05)
        return self.joints.get_subscription_count()

    def median_depth_m(self, depth, u, v, win=5):
        h, w = depth.shape
        patch = depth[max(0, v - win):min(h, v + win + 1),
                      max(0, u - win):min(w, u + win + 1)].astype(np.float32)
        vals = patch[(patch > 0) & (patch < 10000)]
        return float(np.median(vals)) / 1000.0 if vals.size else 0.0

    def detect_target(self, rgb, depth):
        res = self.model(rgb, conf=self.conf, verbose=False)[0]
        names = res.names
        cw, ch = rgb.shape[1] / 2, rgb.shape[0] / 2
        cands = []
        for b in res.boxes:
            cls = names[int(b.cls)]
            if cls not in self.targets:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            dist = self.median_depth_m(depth, u, v)
            if dist <= 0:                       # glass / edge -> no depth, skip
                continue
            cands.append((math.hypot(u - cw, v - ch), u, v, cls, float(b.conf), dist))
        cands.sort(key=lambda c: c[0])
        return cands[0] if cands else None

    def target_to_armbase(self, u, v, dist, K):
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        # back-project along the view ray at the (compensated) depth, then hand-eye -> arm base
        cam = np.array([(u - cx) * dist / fx + RGB_DEPTH_X_OFFSET,
                        (v - cy) * dist / fy, dist, 1.0])
        r = self._await(self.fk.call_async(GetRobotPose.Request()))
        if r is None:
            return None
        p, o = r.pose.position, r.pose.orientation
        endpoint = quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])
        return (endpoint @ HAND2CAM @ cam)[:3]

    def solve_ik(self, pos, pitch):
        req = SetRobotPose.Request()
        req.position = [float(v) for v in pos]
        req.pitch = float(pitch)
        req.pitch_range = [-180.0, 180.0]
        req.resolution = 1.0
        r = self._await(self.ik.call_async(req))
        if r is None or len(r.pulse) < 5:       # service down / degenerate solution
            return None
        return r

    # ---- the grasp -----------------------------------------------------
    def grasp_once(self):
        if not self._lock.acquire(blocking=False):
            return False, 'busy'
        try:
            if not self.wait_ready():
                if self.last_imu == 0.0:
                    return False, 'no /imu yet (board down or bringup not up)'
                return False, 'no camera/depth/info yet'
            if not self.board_alive():
                return False, 'control board appears HUNG (/imu silent) -- reboot the robot'
            rgb, depth, K = self.rgb, self.depth, self.K   # snapshot together
            if depth.shape[:2] != rgb.shape[:2]:
                self.get_logger().warn('depth %s != rgb %s; assuming registered/aligned'
                                       % (depth.shape[:2], rgb.shape[:2]))
            if self.wait_bridge() < 1:
                return False, 'servo bridge not connected'
            tgt = self.detect_target(rgb, depth)
            if tgt is None:
                return False, 'no target with valid depth in view'
            _, u, v, cls, conf, dist = tgt
            dist += self.depth_comp
            pos = self.target_to_armbase(u, v, dist, K)
            if pos is None:
                return False, 'FK (get_current_pose) failed -- kinematics service down?'
            pitch = 80.0 if pos[2] < 0.2 else 30.0
            r = self.solve_ik(pos, pitch)
            if r is None:
                return False, ('no IK solution for %s @arm-base %s (out of reach / service down?)'
                               % (cls, np.round(pos, 3).tolist()))
            self._say('target %s conf=%.2f arm-base=%s pitch=%.0f pulses=%s'
                      % (cls, conf, np.round(pos, 3).tolist(), pitch, list(r.pulse)))
            if self.dry_run:
                return True, 'dry-run OK (no motion)'
            return self._execute(list(pos), pitch, r.pulse)
        finally:
            self._lock.release()

    def _execute(self, pos, pitch, pulse):
        self.servos(0.5, ((10, GRIPPER_OPEN),))                    # ensure gripper open first
        time.sleep(0.6)
        self.servos(1.0, ((1, int(pulse[0])),))                    # base yaw first
        time.sleep(1.0)
        if not self.board_alive():                                 # don't close on a dead board
            return self._home(), 'aborted before grasp (board hung)'
        self.servos(1.5, arm_servos(pulse))                        # full approach
        time.sleep(1.6)
        self.servos(0.6, ((10, GRIPPER_CLOSE),))                   # close gripper
        time.sleep(1.0)
        r2 = self.solve_ik([pos[0], pos[1], pos[2] + 0.05], pitch)
        if r2 is not None:
            self.servos(1.0, arm_servos(r2.pulse))                 # lift 5cm
            time.sleep(1.2)
        ok = self._home(close=True)                                # always try to re-home
        return ok, ('grasped + returned' if ok else 'grasped but board hung before re-home')

    def _home(self, close=False):
        # always *attempt* the return; harmless if the board is dead, recovers if alive
        grip = GRIPPER_CLOSE if close else GRIPPER_OPEN
        self.servos(1.5, OBSERVE + ((10, grip),))
        time.sleep(1.6)
        return self.board_alive()

    def _on_trigger(self, request, response):
        ok, msg = self.grasp_once()
        response.success = ok
        response.message = msg
        self._say('trigger -> %s: %s' % (ok, msg))
        return response

    def _auto(self):
        time.sleep(3.0)
        ok, msg = self.grasp_once()
        self._say('auto_grab -> %s: %s' % (ok, msg))


def main():
    rclpy.init()
    node = GraspNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
