"""
Detect the simulation maze-exit marker from the robot RGB camera.

The detector is deliberately small and deterministic.  It recognises the
project-owned magenta panel, estimates its range from its known width and the
camera intrinsics, and publishes a pose in the camera frame.  It does not know
where the panel is in the maze.
"""

from __future__ import annotations

from collections import deque
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


IMAGE_TOPIC = '/demo/camera/image_raw'
CAMERA_INFO_TOPIC = '/demo/camera/camera_info'
DETECTIONS_TOPIC = '/demo/perception/maze_exit/detections'
POSE_TOPIC = '/demo/perception/maze_exit/pose'


def magenta_bbox(image: Image, stride: int = 4) -> tuple[int, int, int, int, int] | None:
    """Return ``min_x, min_y, max_x, max_y, samples`` for the magenta panel."""
    if image.width <= 0 or image.height <= 0 or image.step <= 0:
        return None
    if image.encoding not in ('rgb8', 'bgr8'):
        return None

    data = memoryview(image.data)
    rgb = image.encoding == 'rgb8'
    min_x, min_y = image.width, image.height
    max_x = max_y = -1
    count = 0
    stride = max(1, int(stride))
    for y in range(0, image.height, stride):
        row = y * image.step
        for x in range(0, image.width, stride):
            offset = row + x * 3
            if offset + 2 >= len(data):
                continue
            if rgb:
                red, green, blue = data[offset], data[offset + 1], data[offset + 2]
            else:
                blue, green, red = data[offset], data[offset + 1], data[offset + 2]
            # Wide enough for Gazebo lighting/JPEG artefacts, but deliberately
            # excludes the blue sky and the neutral maze/ground materials.
            if red >= 170 and blue >= 150 and green <= 105 \
                    and red - green >= 70 and blue - green >= 50:
                count += 1
                min_x, max_x = min(min_x, x), max(max_x, x)
                min_y, max_y = min(min_y, y), max(max_y, y)

    if count == 0:
        return None
    return min_x, min_y, max_x, max_y, count


def marker_pose(
    bbox: tuple[int, int, int, int, int],
    info: CameraInfo,
    marker_width_m: float,
) -> tuple[float, float] | None:
    """Return forward/left marker coordinates in the project's camera frame."""
    min_x, _, max_x, _, _ = bbox
    width_px = float(max_x - min_x + 1)
    fx = float(info.k[0])
    cx = float(info.k[2])
    if width_px <= 0.0 or fx <= 0.0 or marker_width_m <= 0.0:
        return None
    centre_x = (min_x + max_x) / 2.0
    range_m = fx * marker_width_m / width_px
    bearing = math.atan2(cx - centre_x, fx)  # image-left is ROS +y
    return range_m * math.cos(bearing), range_m * math.sin(bearing)


class MazeExitDetector(Node):
    """Publish a confirmed visual observation of the maze exit marker."""

    def __init__(self) -> None:
        super().__init__('maze_exit_detector')
        self.declare_parameter('marker_width_m', 0.8)
        self.declare_parameter('sample_stride', 4)
        self.declare_parameter('min_samples', 40)
        self.declare_parameter('min_bbox_width_px', 12)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('confirmation_window', 5)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._info: CameraInfo | None = None
        self._history: deque[bool] = deque(maxlen=5)
        self._detections_pub = self.create_publisher(
            Detection2DArray, DETECTIONS_TOPIC, 10)
        self._pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_info, qos)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, qos)

    def _on_info(self, message: CameraInfo) -> None:
        self._info = message

    def _on_image(self, image: Image) -> None:
        window = max(1, int(self.get_parameter('confirmation_window').value))
        if self._history.maxlen != window:
            self._history = deque(self._history, maxlen=window)

        bbox = magenta_bbox(image, int(self.get_parameter('sample_stride').value))
        valid = bbox is not None
        if bbox is not None:
            valid = (
                bbox[4] >= int(self.get_parameter('min_samples').value)
                and bbox[2] - bbox[0] + 1
                >= int(self.get_parameter('min_bbox_width_px').value)
            )
        self._history.append(valid)

        detections = Detection2DArray()
        detections.header = image.header
        confirmed = valid and sum(self._history) >= int(
            self.get_parameter('confirm_frames').value)
        if not confirmed or bbox is None:
            self._detections_pub.publish(detections)
            return

        detection = self._detection(image, bbox)
        detections.detections.append(detection)
        self._detections_pub.publish(detections)

        if self._info is None:
            return
        xy = marker_pose(
            bbox, self._info,
            float(self.get_parameter('marker_width_m').value))
        if xy is None:
            return
        forward, left = xy
        if not 0.3 <= math.hypot(forward, left) <= 8.0:
            return
        pose = PoseStamped()
        pose.header = image.header
        pose.pose.position.x = forward
        pose.pose.position.y = left
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

    @staticmethod
    def _detection(
        image: Image, bbox: tuple[int, int, int, int, int],
    ) -> Detection2D:
        min_x, min_y, max_x, max_y, _ = bbox
        detection = Detection2D()
        detection.header = image.header
        detection.id = 'maze_exit'
        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = (min_x + max_x) / 2.0
        detection.bbox.center.position.y = (min_y + max_y) / 2.0
        detection.bbox.size_x = float(max_x - min_x + 1)
        detection.bbox.size_y = float(max_y - min_y + 1)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'maze_exit'
        hypothesis.hypothesis.score = 1.0
        detection.results.append(hypothesis)
        return detection


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MazeExitDetector()
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
