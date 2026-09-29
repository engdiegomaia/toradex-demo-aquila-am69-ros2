"""
Temporal confirmation of the exit detector, and what it refuses to publish.

`test_maze_exit_detector.py` covers the pure functions -- segmentation and
geometry. What is here is the part that decides WHEN the explorer is
interrupted: a single magenta frame must not cancel a frontier goal, and a
pose without intrinsics must not be invented.

The detector never knows where the exit is. It knows it saw a magenta panel,
how many times in a row, and at what distance -- and that is all it says.
"""

from demo_perception.maze_exit_detector import MazeExitDetector
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image


PANEL = (255, 0, 255)


def image_with_panel(width: int = 64, height: int = 48,
                     box: tuple[int, int, int, int] = (16, 12, 48, 36),
                     encoding: str = 'rgb8') -> Image:
    """Build a frame with a saturated magenta rectangle, like the panel in Gazebo."""
    message = Image()
    message.header.frame_id = 'front_camera'
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = width * 3
    data = bytearray(message.step * height)
    min_x, min_y, max_x, max_y = box
    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            offset = y * message.step + x * 3
            data[offset:offset + 3] = bytes(PANEL)
    message.data = bytes(data)
    return message


def blank_image() -> Image:
    """Return the same frame with no panel at all."""
    image = image_with_panel()
    image.data = bytes(len(image.data))
    return image


def camera_info(fx: float = 40.0) -> CameraInfo:
    """
    Intrinsics for this test's 64 px camera.

    `fx` is a parameter because the estimated distance is `fx * 0.8 / width_px`:
    which width falls outside the usable band depends on the lens, and testing
    the extremes requires choosing the lens in which that extreme exists.
    """
    info = CameraInfo()
    info.k = [fx, 0.0, 32.0, 0.0, fx, 24.0, 0.0, 0.0, 1.0]
    return info


@pytest.fixture
def node():
    """
    Detector with stride 1 and the outputs captured instead of published.

    `detector_backend` pinned to 'magenta' because this file specifically
    tests the confirmation gate and the magenta panel geometry
    (`test_maze_exit_detector.py` covers the fiducial backend); the node's
    default changed to 'fiducial' when the tag was added.
    """
    rclpy.init()
    detector = MazeExitDetector()
    detector.set_parameters([
        Parameter('sample_stride', Parameter.Type.INTEGER, 1),
        Parameter('detector_backend', Parameter.Type.STRING, 'magenta'),
    ])
    detector.detections = []
    detector.poses = []
    detector._detections_pub.publish = detector.detections.append
    detector._pose_pub.publish = detector.poses.append
    yield detector
    detector.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def confirmed(detector) -> list:
    """Return the detection messages that actually carry a box."""
    return [message for message in detector.detections if message.detections]


def test_one_frame_is_not_enough_to_interrupt_the_explorer(node) -> None:
    """
    A single frame is only noise, and cancelling the frontier goal over noise is expensive.

    The explorer cancels the in-flight goal as soon as the pose is fresh. If
    a reflection were enough, it would oscillate between `navigating` and
    `homing_exit` and the robot would stop at every false positive.
    """
    node._on_image(image_with_panel())
    assert confirmed(node) == []


def test_three_frames_in_the_window_confirm(node) -> None:
    """3 out of 5 is the declared criterion; the third frame is what publishes."""
    for _ in range(2):
        node._on_image(image_with_panel())
    assert confirmed(node) == []
    node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1


def test_a_gap_inside_the_window_still_confirms(node) -> None:
    """The criterion is 3 IN 5, not 3 in a row: the panel flickers with gait."""
    node._on_image(image_with_panel())
    node._on_image(blank_image())
    node._on_image(image_with_panel())
    assert confirmed(node) == []
    node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1


def test_losing_the_panel_drops_below_the_threshold_again(node) -> None:
    """Falling out of the window is how the detector says it lost the marker."""
    for _ in range(3):
        node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1
    for _ in range(5):
        node._on_image(blank_image())
    assert len(confirmed(node)) == 1


def test_the_current_frame_must_itself_be_valid(node) -> None:
    """
    Three old confirmations do not authorize publishing over an empty frame.

    The published box must come from the frame that just arrived; inheriting
    the previous one would give the explorer a direction nobody is seeing
    anymore.
    """
    for _ in range(3):
        node._on_image(image_with_panel())
    before = len(confirmed(node))
    node._on_image(blank_image())
    assert len(confirmed(node)) == before


def test_detections_are_published_every_frame_even_when_empty(node) -> None:
    """Silence and "I see nothing" must be distinguishable on the consumer side."""
    for _ in range(4):
        node._on_image(blank_image())
    assert len(node.detections) == 4
    assert all(not message.detections for message in node.detections)


def test_a_panel_too_small_is_refused_even_when_repeated(node) -> None:
    """
    Below the minimum box the estimated distance has no precision at all.

    The distance comes from the WIDTH in pixels; at few pixels, one pixel of
    error at the edge turns into metres of error on the target.
    """
    tiny = image_with_panel(box=(30, 22, 36, 28))
    for _ in range(5):
        node._on_image(tiny)
    assert confirmed(node) == []


def test_no_pose_without_camera_info(node) -> None:
    """
    Without intrinsics there is no distance, and inventing one is worse than not publishing.

    This is NOT hypothetical on the HIL: the image reaches the module by its
    own path (compressed, rewired by a launch remap) and `camera_info` arrives
    by another. If only one of the two gets through, the detection appears
    and the pose never comes out -- and this is the test that names that
    failure mode.
    """
    for _ in range(4):
        node._on_image(image_with_panel())
    assert confirmed(node) != []
    assert node.poses == []


def test_pose_is_published_in_the_camera_frame_once_intrinsics_arrive(node) -> None:
    """The pose comes out in the frame's frame; the explorer transforms it to `map`."""
    node._on_info(camera_info())
    for _ in range(3):
        node._on_image(image_with_panel())
    assert len(node.poses) == 1
    pose = node.poses[0]
    assert pose.header.frame_id == 'front_camera'
    assert pose.pose.position.x > 0.0


def test_a_panel_filling_the_frame_is_too_near_to_be_the_exit(node) -> None:
    """
    Below 0.3 m what is seen is a wall pressed against the lens, not the exit.

    fx = 20 on a 64 px image is the lens where "full frame" falls below the
    band: 20 x 0.8 / 64 = 0.25 m.
    """
    node._on_info(camera_info(fx=20.0))
    full_frame = image_with_panel(box=(0, 0, 64, 48))
    for _ in range(4):
        node._on_image(full_frame)
    assert confirmed(node) != []
    assert node.poses == []


def test_a_panel_at_the_horizon_is_too_far_to_be_trusted(node) -> None:
    """
    Above 8 m the width in pixels no longer supports the estimate.

    fx = 200 with a 12 px box -- the smallest the detector accepts -- gives
    13.3 m: the detection is published, the pose is not. The distinction
    matters: the explorer must not abandon the frontier over a marker it still
    cannot measure.
    """
    node._on_info(camera_info(fx=200.0))
    distant = image_with_panel(box=(26, 18, 38, 30))
    for _ in range(4):
        node._on_image(distant)
    assert confirmed(node) != []
    assert node.poses == []


def test_the_marker_detections_never_reach_the_costmap_topic(node) -> None:
    """
    The panel is a visual cue, not an obstacle.

    `detections_to_cloud` subscribes to `/demo/perception/detections`.
    Publishing the marker there would turn it into an obstacle in the
    costmap, right in front of the opening the robot needs to cross.
    """
    topic = node._detections_pub.topic_name
    assert topic.endswith('/demo/perception/maze_exit/detections')
    assert not topic.endswith('/demo/perception/detections')
