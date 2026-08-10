"""
Unit tests for the detections -> PointCloud2 adapter.

This node is the seam between perception and the Nav2 costmap. The tests below
pin the properties the costmap depends on: correct bearing sign, a well-formed
cloud, score filtering, and — importantly — that an empty detection set still
publishes, because silence freezes stale obstacles in the costmap.
"""

import struct

from demo_perception.detections_to_cloud import DetectionsToCloud
import pytest
import rclpy
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


def _make_detection(centre_x: float, score: float = 0.9) -> Detection2D:
    detection = Detection2D()
    detection.bbox = BoundingBox2D()
    detection.bbox.center.position.x = centre_x
    detection.bbox.center.position.y = 240.0
    detection.bbox.size_x = 120.0
    detection.bbox.size_y = 160.0

    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'box'
    hypothesis.hypothesis.score = score
    detection.results.append(hypothesis)
    return detection


def _unpack(cloud) -> list[tuple[float, float, float]]:
    count = cloud.width
    return [
        struct.unpack_from('<fff', bytes(cloud.data), offset=index * 12)
        for index in range(count)
    ]


@pytest.fixture
def node():
    """Provide a DetectionsToCloud with rclpy initialised and torn down."""
    rclpy.init()
    adapter = DetectionsToCloud()
    yield adapter
    adapter.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_centred_detection_projects_straight_ahead(node) -> None:
    """A box at the image centre sits dead ahead: y ~ 0, x = assumed range."""
    points = node._project(_make_detection(centre_x=320.0))

    assert points, 'centred detection produced no points'
    for x, y, _z in points:
        assert abs(y) < 1e-6, f'expected zero lateral offset, got {y}'
        assert abs(x - node.DEFAULT_ASSUMED_RANGE_M) < 1e-6


def test_bearing_sign_follows_ros_convention(node) -> None:
    """
    A detection on the right of the image must land at negative y.

    In image coordinates +x runs right; in ROS +y runs left. Getting this
    backwards steers the robot into the obstacle it is trying to avoid, and
    nothing else in the pipeline would catch it.
    """
    right_points = node._project(_make_detection(centre_x=600.0))
    left_points = node._project(_make_detection(centre_x=40.0))

    assert right_points[0][1] < 0.0, 'detection on the right should have y < 0'
    assert left_points[0][1] > 0.0, 'detection on the left should have y > 0'


def test_low_score_detections_are_filtered(node) -> None:
    """Below min_score the detection must not reach the costmap at all."""
    detections = Detection2DArray()
    detections.header = Header(frame_id='camera_link')
    detections.detections.append(_make_detection(centre_x=320.0, score=0.1))

    published: list = []
    node._publisher.publish = published.append

    node._on_detections(detections)

    assert len(published) == 1, 'adapter must publish even when filtering'
    assert published[0].width == 0, 'low-score detection leaked into the cloud'


def test_empty_detections_still_publish(node) -> None:
    """
    Silence is not the same as "no obstacles".

    The ObstacleLayer needs a continuous stream to age out old marks. If this
    node goes quiet when nothing is detected, the last obstacle stays in the
    costmap forever and the robot refuses to plan through it.
    """
    detections = Detection2DArray()
    detections.header = Header(frame_id='camera_link')

    published: list = []
    node._publisher.publish = published.append

    node._on_detections(detections)

    assert len(published) == 1
    assert published[0].width == 0


def test_cloud_is_structurally_valid(node) -> None:
    """point_step, row_step and the payload length must agree."""
    detections = Detection2DArray()
    detections.header = Header(frame_id='camera_link')
    detections.detections.append(_make_detection(centre_x=320.0))

    published: list = []
    node._publisher.publish = published.append
    node._on_detections(detections)
    cloud = published[0]

    assert cloud.height == 1
    assert cloud.point_step == 12
    assert cloud.row_step == cloud.point_step * cloud.width
    assert len(cloud.data) == cloud.row_step
    assert len(_unpack(cloud)) == cloud.width


def test_cloud_preserves_detection_frame(node) -> None:
    """Without the right frame_id the costmap cannot transform the points."""
    detections = Detection2DArray()
    detections.header = Header(frame_id='camera_link')
    detections.detections.append(_make_detection(centre_x=320.0))

    published: list = []
    node._publisher.publish = published.append
    node._on_detections(detections)

    assert published[0].header.frame_id == 'camera_link'


def test_points_form_a_vertical_stack(node) -> None:
    """
    One point per detection would slip between costmap cells.

    The stack gives the obstacle vertical extent so it marks reliably.
    """
    points = node._project(_make_detection(centre_x=320.0))

    assert len(points) == node.DEFAULT_POINTS_PER_DETECTION
    heights = sorted(z for _x, _y, z in points)
    assert heights[0] == 0.0
    assert abs(heights[-1] - node.DEFAULT_OBSTACLE_HEIGHT_M) < 1e-6
