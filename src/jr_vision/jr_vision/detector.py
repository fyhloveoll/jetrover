#!/usr/bin/env python3
# encoding: utf-8
# jr_vision detector: the durable M4 perception node for JetRover.
#   /depth_cam RGB  --YOLOv11(GPU)-->  2D boxes
#   box center  --HW-aligned depth-->  camera-frame 3D
#   --tf2 (eye-in-hand, moves with the arm)-->  base-frame 3D grasp target
#
# Publishes:
#   /jr/camera/annotated             sensor_msgs/Image            (RGB + boxes, for RViz / raw viewers)
#   /jr/camera/annotated/compressed  sensor_msgs/CompressedImage  (JPEG, for remote viewing over wifi)
#   /jr/grasp/target                 geometry_msgs/PointStamped   (nearest valid target, base frame)
#   /jr/grasp/markers                visualization_msgs/MarkerArray (all targets, for RViz)
#
# Pure consumer of vendor topics; does NOT touch vendor code. Throttled to `rate`
# to bound GPU/battery. Set annotate:=false / enable_3d:=false to lighten it.
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import cv2


class Detector(Node):
    def __init__(self):
        super().__init__('jr_detector')
        self.declare_parameter('model_path', '/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt')
        self.declare_parameter('target_classes', ['bottle', 'cup', 'wine glass'])
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('rate', 8.0)
        self.declare_parameter('annotate', True)
        self.declare_parameter('enable_3d', True)
        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('info_topic', '/depth_cam/rgb/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('depth_window', 4)
        self.declare_parameter('jpeg_quality', 80)

        g = self.get_parameter
        self.targets = set(g('target_classes').value)
        self.conf = float(g('conf').value)
        self.annotate = bool(g('annotate').value)
        self.enable_3d = bool(g('enable_3d').value)
        self.base_frame = g('base_frame').value
        self.win = int(g('depth_window').value)
        self.jpeg_q = int(g('jpeg_quality').value)
        rate = float(g('rate').value)

        self.bridge = CvBridge()
        self.rgb = None
        self.rgb_frame = None
        self.depth = None
        self.K = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, g('rgb_topic').value, self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic').value, self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('info_topic').value, self._info, qos_profile_sensor_data)

        self.pub_img = self.create_publisher(Image, '/jr/camera/annotated', 1)
        self.pub_cmp = self.create_publisher(CompressedImage, '/jr/camera/annotated/compressed', 1)
        self.pub_pt = self.create_publisher(PointStamped, '/jr/grasp/target', 1)
        self.pub_mk = self.create_publisher(MarkerArray, '/jr/grasp/markers', 1)

        self.get_logger().info('loading YOLO %s ...' % g('model_path').value)
        from ultralytics import YOLO
        self.model = YOLO(g('model_path').value)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('jr_detector up: targets=%s rate=%.1f annotate=%s 3d=%s'
                               % (sorted(self.targets), rate, self.annotate, self.enable_3d))

    def _rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        self.rgb_frame = m.header.frame_id

    def _depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, '16UC1')

    def _info(self, m):
        self.K = np.array(m.k).reshape(3, 3)

    def _median_mm(self, u, v):
        d = self.depth
        h, w = d.shape
        patch = d[max(0, v - self.win):min(h, v + self.win + 1),
                  max(0, u - self.win):min(w, u + self.win + 1)].astype(np.float32)
        vals = patch[patch > 0]
        return float(np.median(vals)) if vals.size else 0.0

    def _tick(self):
        if self.rgb is None:
            return
        img = self.rgb.copy()
        res = self.model(img, conf=self.conf, verbose=False)[0]
        names = res.names

        have_3d = self.enable_3d and self.depth is not None and self.K is not None and self.rgb_frame
        tf = None
        if have_3d:
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.rgb_frame, Time())
            except Exception:
                tf = None

        markers = MarkerArray()
        best = None  # (planar_dist, PointStamped)
        mid = 0
        for b in res.boxes:
            cls = names[int(b.cls)]
            conf = float(b.conf)
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            is_target = cls in self.targets
            color = (0, 220, 0) if is_target else (170, 170, 170)
            label = '%s %.2f' % (cls, conf)
            if is_target and tf is not None:
                z = self._median_mm(u, v)
                if z > 0:
                    Z = z / 1000.0
                    cam = PointStamped()
                    cam.header.frame_id = self.rgb_frame
                    cam.point.x = (u - cx) * Z / fx
                    cam.point.y = (v - cy) * Z / fy
                    cam.point.z = Z
                    try:
                        bp = do_transform_point(cam, tf)
                        label += ' [%.2f %.2f %.2f]' % (bp.point.x, bp.point.y, bp.point.z)
                        dist = (bp.point.x ** 2 + bp.point.y ** 2) ** 0.5
                        if best is None or dist < best[0]:
                            best = (dist, bp)
                        markers.markers.append(self._marker(bp, mid))
                        mid += 1
                    except Exception:
                        pass
            if self.annotate:
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(img, label, (x1, max(12, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        if self.annotate:
            im = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            im.header.frame_id = self.rgb_frame or ''
            self.pub_img.publish(im)
            ok, enc = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
            if ok:
                c = CompressedImage()
                c.header.frame_id = self.rgb_frame or ''
                c.format = 'bgr8; jpeg compressed bgr8'
                c.data = enc.tobytes()
                self.pub_cmp.publish(c)

        if best is not None:
            self.pub_pt.publish(best[1])
        if markers.markers:
            self.pub_mk.publish(markers)

    def _marker(self, pt, mid):
        mk = Marker()
        mk.header.frame_id = self.base_frame
        mk.ns = 'grasp'
        mk.id = mid
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = pt.point.x
        mk.pose.position.y = pt.point.y
        mk.pose.position.z = pt.point.z
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.05
        mk.color.r = 1.0
        mk.color.g = 0.2
        mk.color.b = 0.2
        mk.color.a = 0.9
        mk.lifetime.sec = 1
        return mk


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
