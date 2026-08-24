"""
Unit tests for the detection stub.

The properties under test are contract properties, not implementation details:
determinism, staying inside the frame, and preserving the incoming header. Real
TIDL inference will break determinism by design, but the header and framing
guarantees must survive the swap.
"""

from demo_perception.detection_stub import DetectionStub
import pytest
import rclpy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image


def _make_image(width: int = 640, height: int = 480, frame_id: str = 'camera_link') -> Image:
    image = Image()
    image.header.frame_id = frame_id
    image.header.stamp.sec = 42
    image.header.stamp.nanosec = 7
    image.width = width
    image.height = height
    image.encoding = 'rgb8'
    return image


@pytest.fixture
def node():
    """Provide a DetectionStub with rclpy initialised and torn down cleanly."""
    rclpy.init()
    stub = DetectionStub()
    yield stub
    stub.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_same_frame_index_yields_same_box(node) -> None:
    """Determinism: the box is a pure function of the frame index."""
    image = _make_image()

    first = node._build_bbox(image, frame_index=17)
    second = node._build_bbox(image, frame_index=17)

    assert first.center.position.x == second.center.position.x
    assert first.center.position.y == second.center.position.y
    assert first.size_x == second.size_x
    assert first.size_y == second.size_y


def test_camera_subscription_is_reliable(node) -> None:
    """Large fragmented HIL images require retransmission of missing pieces."""
    assert node._subscription.qos_profile.reliability == \
        QoSReliabilityPolicy.RELIABLE


def test_box_moves_between_frames(node) -> None:
    """A static box would not exercise the costmap; the stub must sweep."""
    image = _make_image()

    start = node._build_bbox(image, frame_index=0)
    quarter_period = node._build_bbox(image, frame_index=22)

    assert start.center.position.x != quarter_period.center.position.x


def test_box_stays_inside_the_frame(node) -> None:
    """A box hanging off the edge would project to a bogus bearing."""
    image = _make_image(width=640, height=480)
    half_width = node.DEFAULT_BOX_WIDTH_PX / 2.0

    # One full period, sampled densely enough to catch the extremes.
    for frame_index in range(0, 180):
        bbox = node._build_bbox(image, frame_index)
        left = bbox.center.position.x - half_width
        right = bbox.center.position.x + half_width
        assert left >= -1e-6, f'box crosses the left edge at frame {frame_index}'
        assert right <= 640.0 + 1e-6, \
            f'box crosses the right edge at frame {frame_index}'


def test_zero_sized_image_does_not_produce_negative_coordinates(node) -> None:
    """A malformed frame from a rosbag must not poison the costmap."""
    image = _make_image(width=0, height=0)

    bbox = node._build_bbox(image, frame_index=0)

    assert bbox.center.position.x > 0.0
    assert bbox.center.position.y > 0.0


def test_detection_preserves_image_header(node) -> None:
    """
    Header propagation is what lets the costmap transform the detection.

    Dropping it is the classic silent failure: detections publish fine and the
    costmap ignores every one of them.
    """
    image = _make_image(frame_id='camera_link')

    detection = node._build_detection(image, frame_index=3)

    assert detection.header.frame_id == 'camera_link'
    assert detection.header.stamp.sec == 42
    assert detection.header.stamp.nanosec == 7


def test_detection_carries_a_scored_hypothesis(node) -> None:
    """Consumers filter on score; a detection without one is unusable."""
    detection = node._build_detection(_make_image(), frame_index=0)

    assert len(detection.results) == 1
    assert detection.results[0].hypothesis.class_id == DetectionStub.DEFAULT_CLASS_ID
    assert 0.0 <= detection.results[0].hypothesis.score <= 1.0


def test_rejects_out_of_range_score(node) -> None:
    """Validate at the boundary — a score above 1.0 is a programming error."""
    with pytest.raises(ValueError, match=r'score must be in'):
        node._validate_score(1.5)


def test_rejects_non_positive_period(node) -> None:
    """A zero period would divide by zero inside the sweep."""
    with pytest.raises(ValueError, match=r'period_frames must be > 0'):
        node._validate_period(0.0)
