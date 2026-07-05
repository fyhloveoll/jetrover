#!/usr/bin/env python3
# encoding: utf-8
# Live MJPEG stream of the depth camera with the object-agnostic detection overlaid
# (boxes + IDs + angle, paper contour white, on-paper = red), so we can debug the
# scene TOGETHER in a browser:  http://192.168.0.48:8080/
# No extra packages (cv2 + http.server + rclpy). Run on the robot.
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from http.server import BaseHTTPRequestHandler, HTTPServer
import jr_detect_objects as det

L = {'rgb': None, 'depth': None, 'K': None, 'vis': None}


class Cam(Node):
    def __init__(self):
        super().__init__('mjpeg_stream')
        self.b = CvBridge()
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self._r, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw', self._d, 1)
        self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self._i, 1)

    def _r(self, m): L['rgb'] = self.b.imgmsg_to_cv2(m, 'bgr8')
    def _d(self, m): L['depth'] = self.b.imgmsg_to_cv2(m, '16UC1')
    def _i(self, m): L['K'] = list(m.k)


def detect_loop():
    while True:
        rgb, depth, K = L['rgb'], L['depth'], L['K']
        if rgb is not None and depth is not None and K is not None:
            vis = rgb.copy()
            try:
                hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
                wm = cv2.inRange(hsv, np.array([0, 0, 205]), np.array([180, 30, 255]))
                wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
                cnts, _ = cv2.findContours(wm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                pc = max(cnts, key=cv2.contourArea) if cnts else None
                if pc is not None and cv2.contourArea(pc) >= 4000:
                    cv2.drawContours(vis, [pc], -1, (255, 255, 255), 2)
                else:
                    pc = None
                for o in det.detect(rgb, np.asarray(depth), K):
                    onp = pc is not None and cv2.pointPolygonTest(pc, (float(o['u']), float(o['v'])), False) >= 0
                    col = (0, 0, 255) if onp else (0, 255, 0)
                    # ORIENTED box (yellow) shows the detected angle; axis-aligned box faint
                    if o.get('rect') is not None:
                        bp = cv2.boxPoints(o['rect']).astype(int)
                        cv2.drawContours(vis, [bp], 0, (0, 255, 255), 2)
                    else:
                        cv2.rectangle(vis, o['box'][:2], o['box'][2:], col, 2)
                    cv2.putText(vis, '%s %+.0f' % (o['id'], o['angle']), (o['box'][0], o['box'][1] - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                    cv2.drawMarker(vis, (o['u'], o['v']), col, cv2.MARKER_CROSS, 9, 2)
            except Exception as e:
                cv2.putText(vis, 'det err: %s' % str(e)[:36], (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            L['vis'] = vis
        time.sleep(1.0)   # detection overlay at ~1Hz to keep Jetson load low


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                img = L['vis'] if L['vis'] is not None else L['rgb']
                if img is not None:
                    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(jpg.tobytes())
                        self.wfile.write(b'\r\n')
                time.sleep(0.1)
        except Exception:
            pass

    def log_message(self, *a):
        pass


def main():
    rclpy.init()
    node = Cam()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    threading.Thread(target=detect_loop, daemon=True).start()
    print('MJPEG stream on http://0.0.0.0:8080/  (open http://192.168.0.48:8080/)')
    HTTPServer(('0.0.0.0', 8080), H).serve_forever()


if __name__ == '__main__':
    main()
