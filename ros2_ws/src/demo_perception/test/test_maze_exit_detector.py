import json
import math

import cv2
from demo_perception.maze_exit_detector import (
    _corners_fully_inside,
    fiducial_pose,
    find_fiducial,
    magenta_bbox,
    marker_pose,
    MazeExitDetector,
)
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image


def image_with_marker(encoding='rgb8') -> Image:
    message = Image()
    message.width = 40
    message.height = 30
    message.encoding = encoding
    message.step = message.width * 3
    data = bytearray(message.step * message.height)
    colour = (255, 0, 255)
    for y in range(8, 22):
        for x in range(12, 28):
            offset = y * message.step + x * 3
            data[offset:offset + 3] = bytes(colour)
    message.data = bytes(data)
    return message


def test_magenta_bbox_supports_rgb_and_bgr():
    assert magenta_bbox(image_with_marker('rgb8'), 1) == (12, 8, 27, 21, 224)
    assert magenta_bbox(image_with_marker('bgr8'), 1) == (12, 8, 27, 21, 224)


def test_marker_pose_uses_camera_intrinsics_and_known_width():
    info = CameraInfo()
    info.k = [100.0, 0.0, 20.0, 0.0, 100.0, 15.0, 0.0, 0.0, 1.0]
    forward, left = marker_pose((12, 8, 27, 21, 224), info, 0.8)
    assert math.hypot(forward, left) == pytest.approx(5.0)
    assert left > 0.0  # bbox centre is half a pixel left of principal point


def test_non_magenta_image_has_no_detection():
    image = image_with_marker()
    image.data = bytes(len(image.data))
    assert magenta_bbox(image, 1) is None


def image_with_two_patches(encoding='rgb8') -> Image:
    """Build the real panel plus a second magenta region far away from it."""
    message = image_with_marker(encoding)
    data = bytearray(message.data)
    for y in range(2, 6):                      # small speck, opposite corner
        for x in range(2, 6):
            offset = y * message.step + x * 3
            data[offset:offset + 3] = bytes((255, 0, 255))
    message.data = bytes(data)
    return message


def test_a_second_magenta_region_does_not_widen_the_panel_bbox():
    """
    The global bbox over EVERY magenta pixel is the defect measured in R6.

    `range_m = fx * marker_width_m / width_px`, so inflating `width_px` shrinks
    the distance. In R5+R6, 12 independent samples gave
    estimated/true = 0.478 (0.402 to 0.539) against the SDF marker at
    (-4.90, -2.60) -- roughly 2x too close, and homing walked to a point that
    is not the exit. Calibration was ruled out as the cause: `horizontal_fov`
    2.094 rad over 640 px gives fx 184.75, and `camera_info` publishes 184.836.
    """
    one = magenta_bbox(image_with_marker(), 1)
    two = magenta_bbox(image_with_two_patches(), 1)

    assert two == one, 'the extra speck must not enter the panel bbox'


def test_blobs_are_reported_so_the_extra_region_is_observable():
    """Without counting the regions, the scale error stays invisible in the log."""
    from demo_perception.maze_exit_detector import magenta_blobs

    assert len(magenta_blobs(image_with_marker(), 1)) == 1
    assert len(magenta_blobs(image_with_two_patches(), 1)) == 2


def test_a_panel_sampled_with_stride_stays_one_blob():
    """
    With `stride` 4 the sampling is sparse; 4-connectivity would split the panel.

    A panel split into pieces would give the LARGEST piece, not the panel --
    which is the same scale error in reverse. The neighbourhood must be 8.
    """
    from demo_perception.maze_exit_detector import magenta_blobs

    assert len(magenta_blobs(image_with_marker(), 4)) == 1


def test_diagnostics_report_every_region_not_just_the_chosen_one():
    """
    The region count is the discriminator that was missing in R6.

    The defect was a global bbox over EVERY magenta pixel: a second region
    entered the box, inflated `width_px` and, since
    `range_m = fx * marker_width_m / width_px`, cut the distance in half.
    None of this was published, so the investigation had to infer the cause.
    With `regions` and the widths, a merge becomes visible in the record itself.
    """
    from demo_perception.maze_exit_detector import blob_diagnostics

    panel = (10, 10, 40, 40, 100)
    speck = (2, 2, 6, 6, 9)
    record = blob_diagnostics([panel, speck], panel, stamp_ns=7, range_m=4.5)

    assert record['regions'] == 2
    assert record['chosen']['width_px'] == 31
    assert record['chosen']['aspect_ratio'] == 1.0   # panel is square
    assert record['region_widths_px'] == [5, 31]
    assert record['stamp_ns'] == 7


def test_diagnostics_survive_a_frame_with_no_marker():
    """A rejected frame is also evidence; it must not turn into an exception."""
    from demo_perception.maze_exit_detector import blob_diagnostics

    record = blob_diagnostics([], None, stamp_ns=1, range_m=None)

    assert record == {'stamp_ns': 1, 'regions': 0, 'chosen': None}


def test_a_partial_view_is_visible_as_a_non_square_aspect_ratio():
    """
    The panel is 0.80 x 0.80 m; seen through an opening, it looks narrower.

    That is the origin of the range overestimate above 4 m measured in R7
    (mean absolute error of 3.08 m above 6 m). Without the aspect ratio in
    the record there is no way to distinguish "far" from "partially occluded".
    """
    from demo_perception.maze_exit_detector import blob_diagnostics

    sliver = (10, 10, 18, 40, 40)
    record = blob_diagnostics([sliver], sliver, stamp_ns=3, range_m=9.0)

    assert record['chosen']['aspect_ratio'] < 0.5


# --- fiducial backend -------------------------------------------------------
#
# ML3.5 F5, 29/08/2026: the magenta path's range estimate is unbiased but
# noisy with distance (R7). These guard the fiducial replacement, which fails
# CLOSED (no pose) instead of returning a confident wrong range. No test here
# depends on where this repository happens to be cloned -- everything is
# generated in-memory with `cv2.aruco`, the same library the detector itself
# imports, so a passing run here is real evidence `cv2.aruco` resolves in
# whatever environment runs this file (there is no separate mocked code path).

FIDUCIAL_DICTIONARY = 'DICT_APRILTAG_36h11'
IMAGE_WIDTH, IMAGE_HEIGHT = 640, 480


def _camera_info(fx=500.0, fy=500.0, cx=320.0, cy=240.0) -> CameraInfo:
    info = CameraInfo()
    info.width, info.height = IMAGE_WIDTH, IMAGE_HEIGHT
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    return info


def _fiducial_image(
    marker_id: int = 0, tag_px: int = 300, origin=(170, 90),
    stamp=(42, 123), frame_id='front_camera',
) -> Image:
    """
    Build a deterministic synthetic frame with one printed AprilTag.

    `origin` may be negative (or run past the right/bottom edge) to produce a
    genuinely clipped tag: the marker is drawn on an oversized canvas first,
    then the visible `IMAGE_WIDTH x IMAGE_HEIGHT` window is cropped out of it,
    so a negative origin does not wrap around like a raw negative numpy slice
    would.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, marker_id, tag_px)
    pad = tag_px
    world = np.full(
        (IMAGE_HEIGHT + 2 * pad, IMAGE_WIDTH + 2 * pad), 255, dtype=np.uint8)
    ox, oy = origin
    world[oy + pad:oy + pad + tag_px, ox + pad:ox + pad + tag_px] = marker
    canvas = world[pad:pad + IMAGE_HEIGHT, pad:pad + IMAGE_WIDTH]
    bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    image = Image()
    image.width, image.height = IMAGE_WIDTH, IMAGE_HEIGHT
    image.encoding = 'bgr8'
    image.step = IMAGE_WIDTH * 3
    image.data = bgr.tobytes()
    image.header.frame_id = frame_id
    image.header.stamp.sec = stamp[0]
    image.header.stamp.nanosec = stamp[1]
    return image


def test_find_fiducial_reads_the_correct_id_fully_inside_the_frame():
    image = _fiducial_image(marker_id=0)

    detection = find_fiducial(image, FIDUCIAL_DICTIONARY, marker_id=0)

    assert detection['ids_found'] == [0]
    assert detection['reject_reason'] is None
    assert detection['corners'].shape == (4, 2)


def test_find_fiducial_rejects_the_wrong_id():
    image = _fiducial_image(marker_id=0)

    detection = find_fiducial(image, FIDUCIAL_DICTIONARY, marker_id=7)

    assert detection['ids_found'] == [0]
    assert detection['corners'] is None
    assert detection['reject_reason'] == 'incorrect id'


def test_find_fiducial_rejects_a_tag_clipped_by_the_frame_edge():
    """
    Clip a tag against the frame edge; the detector must never return it.

    A tag cut by the image boundary loses its quiet border and the detector
    itself never returns it -- there is no partially-decoded quad to reject
    downstream. That is still "no pose for a partial view", just enforced one
    layer earlier than `_corners_fully_inside`.
    """
    image = _fiducial_image(marker_id=0, tag_px=300, origin=(-150, 90))

    detection = find_fiducial(image, FIDUCIAL_DICTIONARY, marker_id=0)

    assert detection['ids_found'] == []
    assert detection['corners'] is None
    assert detection['reject_reason'] == 'no tag found'


def test_corners_fully_inside_rejects_a_corner_at_the_frame_edge():
    """
    Direct guard for `_corners_fully_inside`.

    Real clipped tags are normally never returned by `cv2.aruco` at all (see
    the test above).
    """
    touching_edge = np.array([[0.0, 50.0], [40.0, 50.0], [40.0, 90.0], [0.0, 90.0]])
    well_inside = touching_edge + [10.0, 0.0]

    assert not _corners_fully_inside(touching_edge, IMAGE_WIDTH, IMAGE_HEIGHT)
    assert _corners_fully_inside(well_inside, IMAGE_WIDTH, IMAGE_HEIGHT)


def test_fiducial_pose_reports_bounded_range_error_and_low_reprojection_error():
    image = _fiducial_image(marker_id=0, tag_px=300, origin=(170, 90))
    detection = find_fiducial(image, FIDUCIAL_DICTIONARY, marker_id=0)
    info = _camera_info(fx=500.0)
    marker_size_m = 0.64

    pose = fiducial_pose(detection['corners'], info, marker_size_m)

    # Pinhole approximation: range ~= fx * marker_size_m / tag_px.
    expected_range_m = 500.0 * marker_size_m / 300.0
    assert pose is not None
    assert pose['range_m'] == pytest.approx(expected_range_m, rel=0.05)
    assert pose['reprojection_error_px'] < 1.0


def test_fiducial_pose_is_none_without_camera_intrinsics():
    image = _fiducial_image(marker_id=0)
    detection = find_fiducial(image, FIDUCIAL_DICTIONARY, marker_id=0)

    assert fiducial_pose(detection['corners'], CameraInfo(), 0.64) is None


@pytest.fixture
def node():
    """Build a MazeExitDetector with every publisher captured instead of published."""
    rclpy.init()
    detector = MazeExitDetector()
    detector.published_poses = []
    detector.published_diagnostics = []
    detector.published_detections = []
    detector._pose_pub.publish = detector.published_poses.append
    detector._diagnostics_pub.publish = lambda message: (
        detector.published_diagnostics.append(json.loads(message.data)))
    detector._detections_pub.publish = detector.published_detections.append
    yield detector
    detector.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _set_params(node, **values) -> None:
    node.set_parameters([
        Parameter(name, value=value) for name, value in values.items()])


def test_fiducial_backend_publishes_a_finite_pose_for_the_correct_tag(node):
    node._on_info(_camera_info(fx=500.0))
    image = _fiducial_image(marker_id=0, tag_px=300, origin=(170, 90))

    node._on_image(image)

    assert len(node.published_poses) == 1
    pose = node.published_poses[0]
    forward, left = pose.pose.position.x, pose.pose.position.y
    assert math.isfinite(forward) and math.isfinite(left)
    assert math.hypot(forward, left) == pytest.approx(
        500.0 * 0.64 / 300.0, rel=0.05)


def test_fiducial_backend_preserves_the_original_image_timestamp_and_frame(node):
    node._on_info(_camera_info())
    image = _fiducial_image(marker_id=0, stamp=(99, 4242), frame_id='front_camera')

    node._on_image(image)

    pose = node.published_poses[0]
    assert pose.header.stamp.sec == 99
    assert pose.header.stamp.nanosec == 4242
    assert pose.header.frame_id == 'front_camera'


def test_fiducial_backend_wrong_id_does_not_publish_a_pose(node):
    node._on_info(_camera_info())
    _set_params(node, fiducial_id=7)
    image = _fiducial_image(marker_id=0)

    node._on_image(image)

    assert node.published_poses == []
    assert node.published_diagnostics[-1]['reject_reason'] == 'incorrect id'


def test_fiducial_backend_clipped_tag_does_not_publish_a_pose(node):
    node._on_info(_camera_info())
    image = _fiducial_image(marker_id=0, tag_px=300, origin=(-150, 90))

    node._on_image(image)

    assert node.published_poses == []
    assert node.published_diagnostics[-1]['reject_reason'] is not None


def test_fiducial_backend_without_camera_info_does_not_publish_a_pose(node):
    image = _fiducial_image(marker_id=0)

    node._on_image(image)

    assert node.published_poses == []
    assert node.published_diagnostics[-1]['reject_reason'] == 'no camera_info'


def test_fiducial_backend_publishes_diagnostics_even_when_rejected(node):
    node._on_info(_camera_info())
    _set_params(node, fiducial_id=7)
    image = _fiducial_image(marker_id=0)

    node._on_image(image)

    assert len(node.published_diagnostics) == 1
    record = node.published_diagnostics[0]
    assert record['backend'] == 'fiducial'
    assert record['ids_found'] == [0]
    assert record['reject_reason'] == 'incorrect id'
    assert record['pose'] is None


def test_magenta_backend_still_works_as_a_configurable_fallback(node):
    """
    Switching `detector_backend` back to magenta reproduces prior behaviour.

    The fiducial path is additive, not a replacement of the magenta code.
    """
    node._on_info(_camera_info(fx=100.0, fy=100.0, cx=20.0, cy=15.0))
    _set_params(
        node, detector_backend='magenta', sample_stride=1, min_samples=1,
        min_bbox_width_px=1, confirm_frames=1, confirmation_window=1)
    image = image_with_marker()

    node._on_image(image)

    assert len(node.published_poses) == 1
    pose = node.published_poses[0]
    assert math.isfinite(pose.pose.position.x)
    assert math.isfinite(pose.pose.position.y)
    assert len(node.published_detections) == 1
    assert len(node.published_detections[0].detections) == 1
    assert 'backend' not in node.published_diagnostics[-1]
