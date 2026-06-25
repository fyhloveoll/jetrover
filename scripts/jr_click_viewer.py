#!/usr/bin/env python3
# encoding: utf-8
# Laptop-side click-to-grasp viewer. Shows the robot's annotated scene stream
# (compressed JPEG) and, on left-click, publishes the clicked pixel to /jr/click
# so the robot grasps whatever object is under the cursor.
#
# Decodes CompressedImage with cv2.imdecode directly -> no cv_bridge needed.
# Low bandwidth (compressed), intuitive (click the camera image).
#
#   source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0
#   python3 jr_click_viewer.py                       # default /jr/scene/annotated/compressed
#   python3 jr_click_viewer.py /jr/camera/annotated/compressed
import sys
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped

DEFAULT_TOPIC = '/jr/scene/annotated/compressed'
WIN = 'jr click-to-grasp  (left-click an object, q to quit)'


class ClickViewer(Node):
    def __init__(self, topic):
        super().__init__('jr_click_viewer')
        self.frame = None
        self.create_subscription(CompressedImage, topic, self._img, 10)
        self.click_pub = self.create_publisher(PointStamped, '/jr/click', 1)
        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, self._on_mouse)
        self.get_logger().info('viewing %s  (publishing clicks to /jr/click)' % topic)

    def _img(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frame = img

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            p = PointStamped()
            p.header.frame_id = 'image'
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x = float(x)
            p.point.y = float(y)
            p.point.z = 0.0
            self.click_pub.publish(p)
            self.get_logger().info('clicked pixel (%d,%d) -> /jr/click' % (x, y))


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    rclpy.init()
    node = ClickViewer(topic)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is not None:
                cv2.imshow(WIN, node.frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
