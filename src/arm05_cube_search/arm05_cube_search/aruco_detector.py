import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from arm05_search_interfaces.msg import ArucoDetection


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.ids = set(self.declare_parameter('expected_ids', [0, 1, 2, 3, 4]).value)
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.camera = None
        self.last_stamp = -125_000_000
        h = 0.035
        self.square = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])

        self.pub = self.create_publisher(ArucoDetection, '/aruco_detections', 20)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.calibrate, qos_profile_sensor_data
        )
        self.create_subscription(Image, '/camera/image_raw', self.detect, qos_profile_sensor_data)

    def calibrate(self, msg):
        if self.camera is None and msg.k[0] > 0:
            self.camera = np.array(msg.k).reshape(3, 3), np.array(msg.d) if msg.d else None

    def detect(self, msg):
        if self.camera is None:
            return

        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        markers = []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(preview, corners, ids)
            markers = sorted(
                zip(ids.ravel(), corners),
                key=lambda item: cv2.contourArea(item[1]),
                reverse=True,
            )

        matrix, distortion = self.camera
        seen = set()

        for marker_id, corner in markers:
            marker_id = int(marker_id)
            if marker_id not in self.ids or marker_id in seen or cv2.contourArea(corner) < 196:
                continue

            ok, rotation, position = cv2.solvePnP(
                self.square, corner.reshape(4, 2), matrix, distortion,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok or position[2, 0] <= 0 or np.linalg.norm(position) > 4:
                continue

            cv2.drawFrameAxes(preview, matrix, distortion, rotation, position, 0.05)
            x, y = corner.reshape(4, 2)[0].astype(int)
            cv2.putText(
                preview, f'Z = {position[2, 0]:.2f} m',
                (int(x), max(25, int(y) - 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
            )

            detection = ArucoDetection(id=marker_id)
            detection.header.stamp = msg.header.stamp
            detection.header.frame_id = 'camera_rgb_optical_frame'
            p = detection.pose.position
            p.x, p.y, p.z = map(float, position.ravel())

            angle = float(np.linalg.norm(rotation))
            scale = math.sin(angle / 2) / angle if angle else 0.5
            q = detection.pose.orientation
            q.x, q.y, q.z = map(float, rotation.ravel() * scale)
            q.w = math.cos(angle / 2)

            self.pub.publish(detection)
            seen.add(marker_id)

        cv2.imshow('ArUco - podglad', cv2.resize(preview, (960, 540)))
        cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(ArucoDetector())