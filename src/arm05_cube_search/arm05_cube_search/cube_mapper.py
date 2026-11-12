from collections import defaultdict, deque
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener

from arm05_search_interfaces.msg import ArucoDetection, Cube, CubeArray
from arm05_search_interfaces.srv import FinalizeCubes


class CubeMapper(Node):
    def __init__(self):
        super().__init__('cube_mapper')
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.samples = defaultdict(list)
        self.pending = deque()
        self.active = False

        self.pub = self.create_publisher(CubeArray, '/detected_cubes', 10)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.observe, 20)
        self.create_service(Trigger, '/reset_cubes', self.reset)
        self.create_service(FinalizeCubes, '/finalize_cubes', self.finalize)
        self.create_timer(0.25, self.update)

    def reset(self, _request, response):
        self.samples.clear()
        self.pending.clear()
        self.active = True
        self.pub.publish(self.estimates())
        response.success = True
        return response

    def observe(self, msg):
        if self.active:
            self.pending.append((msg, time.monotonic() + 1.0))

    def collect(self):
        for _ in range(len(self.pending)):
            msg, deadline = self.pending.popleft()
            try:
                transform = self.tf.lookup_transform(
                    'map', msg.header.frame_id, Time.from_msg(msg.header.stamp)
                )
            except TransformException:
                if time.monotonic() < deadline:
                    self.pending.append((msg, deadline))
                continue

            # Przesuń pomiar o pół boku kostki od znacznika do środka.
            p, q = msg.pose.position, msg.pose.orientation
            p.x -= 0.09 * (q.x * q.z + q.w * q.y)
            p.y -= 0.09 * (q.y * q.z - q.w * q.x)
            p.z -= 0.045 * (1 - 2 * (q.x**2 + q.y**2))

            center = do_transform_pose(msg.pose, transform).position
            if abs(center.z - 0.045) < 0.12:
                self.samples[msg.id].append((center.x, center.y, center.z))

    def estimates(self):
        result = CubeArray()
        result.header.frame_id = 'map'
        result.header.stamp = self.get_clock().now().to_msg()

        for marker_id, samples in sorted(self.samples.items()):
            if len(samples) < 4:
                continue

            points = np.array(samples)
            distances = np.linalg.norm(points - np.median(points, axis=0), axis=1)
            points = points[distances <= max(0.08, 2.5 * np.median(distances))]
            if len(points) < 4:
                continue

            cube = Cube(id=marker_id, samples=len(points))
            cube.position.x, cube.position.y, cube.position.z = map(float, points.mean(axis=0))
            result.cubes.append(cube)

        return result

    def update(self):
        if self.active:
            self.collect()
            self.pub.publish(self.estimates())

    def finalize(self, _request, response):
        self.collect()
        self.pending.clear()
        self.active = False
        response.result = self.estimates()
        self.pub.publish(response.result)
        return response


def main():
    rclpy.init()
    rclpy.spin(CubeMapper())