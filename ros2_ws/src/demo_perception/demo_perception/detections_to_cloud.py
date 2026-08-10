"""
Detections -> PointCloud2 adapter, so perception reaches the Nav2 costmap.

Runs on: same container as detection_stub (Aquila in target mode, host in learn).

WHY THIS NODE EXISTS
CLAUDE.md requires that detections feed a Nav2 costmap layer, not just the HMI
screen. Nav2's stock ObstacleLayer consumes LaserScan or PointCloud2 — it cannot
read Detection2DArray. The two ways to close that gap are a C++ costmap plugin,
or converting to a message the stock layer already accepts. This project's
conventions say C++ only where hardware-measured performance justifies it, and
nothing has been measured on the AM69 yet, so this is the Python adapter.

Swapping it for a native C++ layer later changes nothing upstream or downstream:
demo_perception still publishes Detection2DArray, and Nav2 still marks the same
cells.

PROJECTION MODEL AND ITS LIMITS
A 2D bounding box carries no depth. This node assumes every detection sits on
the ground plane at a fixed distance, then uses the pinhole model to convert the
box's horizontal position into a bearing. The result is a coarse obstacle
placed at roughly the right angle, at an assumed range.

That is honest for a stub and adequate for proving the costmap wiring. It is NOT
a substitute for depth. When real inference arrives, replace the assumed range
with a depth image, a stereo pair, or the detection's own 3D pose.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection2DArray


class DetectionsToCloud(Node):
    """Project 2D detections onto the ground plane as a PointCloud2."""

    DETECTIONS_TOPIC = '/demo/perception/detections'
    CLOUD_TOPIC = '/demo/perception/detection_cloud'

    DEFAULT_ASSUMED_RANGE_M = 2.0
    DEFAULT_HFOV_RAD = 1.089          # must match camera_sensor hfov in the xacro
    DEFAULT_IMAGE_WIDTH_PX = 640.0    # must match camera_width in the xacro
    DEFAULT_OBSTACLE_HEIGHT_M = 0.3
    DEFAULT_POINTS_PER_DETECTION = 5
    DEFAULT_MIN_SCORE = 0.5

    def __init__(self) -> None:
        super().__init__('detections_to_cloud')

        self.declare_parameter('assumed_range_m', self.DEFAULT_ASSUMED_RANGE_M)
        self.declare_parameter('hfov_rad', self.DEFAULT_HFOV_RAD)
        self.declare_parameter('image_width_px', self.DEFAULT_IMAGE_WIDTH_PX)
        self.declare_parameter('obstacle_height_m', self.DEFAULT_OBSTACLE_HEIGHT_M)
        self.declare_parameter('points_per_detection', self.DEFAULT_POINTS_PER_DETECTION)
        self.declare_parameter('min_score', self.DEFAULT_MIN_SCORE)

        self._publisher = self.create_publisher(PointCloud2, self.CLOUD_TOPIC, 10)
        self._subscription = self.create_subscription(
            Detection2DArray, self.DETECTIONS_TOPIC, self._on_detections, 10)

        self.get_logger().info(
            f'detections_to_cloud up: {self.DETECTIONS_TOPIC} -> {self.CLOUD_TOPIC}')

    def _on_detections(self, detections: Detection2DArray) -> None:
        min_score = float(self.get_parameter('min_score').value)

        points: list[tuple[float, float, float]] = []
        for detection in detections.detections:
            if self._best_score(detection) < min_score:
                continue
            points.extend(self._project(detection))

        # Publish even when empty: the ObstacleLayer needs a steady stream to
        # clear stale marks. Going silent freezes old obstacles in the costmap.
        self._publisher.publish(self._build_cloud(detections.header, points))

    @staticmethod
    def _best_score(detection) -> float:
        if not detection.results:
            return 0.0
        return max(result.hypothesis.score for result in detection.results)

    def _project(self, detection) -> list[tuple[float, float, float]]:
        """Map one bounding box to a small vertical stack of points."""
        assumed_range = float(self.get_parameter('assumed_range_m').value)
        hfov = float(self.get_parameter('hfov_rad').value)
        image_width = float(self.get_parameter('image_width_px').value)
        height_m = float(self.get_parameter('obstacle_height_m').value)
        count = int(self.get_parameter('points_per_detection').value)

        if image_width <= 0.0 or count <= 0:
            return []

        # Pinhole bearing: offset from image centre, normalised to [-0.5, 0.5],
        # scaled by the horizontal field of view.
        centre_x = detection.bbox.center.position.x
        normalised = (centre_x - image_width / 2.0) / image_width
        bearing = -normalised * hfov  # +x right in image, +y left in ROS

        # Points are expressed in the detection's own frame (camera_link). The
        # costmap transforms them into the global frame using TF, which is why
        # the header must be preserved end to end.
        x = assumed_range * math.cos(bearing)
        y = assumed_range * math.sin(bearing)

        if count == 1:
            return [(x, y, 0.0)]
        step = height_m / float(count - 1)
        return [(x, y, index * step) for index in range(count)]

    def _build_cloud(self, header, points: list[tuple[float, float, float]]) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        return cloud


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetectionsToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
