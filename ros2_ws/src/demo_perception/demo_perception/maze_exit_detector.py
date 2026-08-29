"""
Detect the simulation maze-exit marker from the robot RGB camera.

The detector is deliberately small and deterministic.  It recognises the
project-owned magenta panel, estimates its range from its known width and the
camera intrinsics, and publishes a pose in the camera frame.  It does not know
where the panel is in the maze.
"""

from __future__ import annotations

from collections import deque
import json
import math

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
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
# Diagnostics, not a contract topic: the R6/R7 investigation had to guess at
# blob geometry because none of it was published. Same JSON-on-String shape
# as /demo/exploration/status so one recorder pattern reads both.
DIAGNOSTICS_TOPIC = '/demo/perception/maze_exit/diagnostics'

# ML3.5 F5, 29/08/2026: the magenta panel's range estimate is unbiased but
# noisy with distance (R7: 0.41 m mean error at 3-4 m, 3.08 m above 6 m,
# signature of partial occlusion through the maze opening). A fiducial fails
# CLOSED instead of returning a confident wrong range, so it replaces magenta
# as the pose source homing acts on -- see `detector_backend` below. The
# panel, its pose and `MazeEscapeValidator` are unchanged; only what is
# painted on the panel and how range is computed from it changed.
FIDUCIAL_DICTIONARIES = {'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11}
# Physical size of the printed tag, inside the 0.80 x 0.80 m magenta panel
# painted on the exit marker model -- see `docs/ml35/estado-fases.md` for the
# scenario asset note. The scenario file and pose are deliberately not named
# here: this module must not know which world it is running in.
FIDUCIAL_SIZE_M_DEFAULT = 0.64
FIDUCIAL_MIN_RANGE_M = 0.3
FIDUCIAL_MAX_RANGE_M = 8.0


def _magenta_samples(image: Image, stride: int) -> tuple[set, int]:
    """Return the sampled magenta lattice points and the stride actually used."""
    data = memoryview(image.data)
    rgb = image.encoding == 'rgb8'
    stride = max(1, int(stride))
    points = set()
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
                points.add((x, y))
    return points, stride


def magenta_blobs(
    image: Image, stride: int = 4,
) -> list[tuple[int, int, int, int, int]]:
    """
    Return one ``min_x, min_y, max_x, max_y, samples`` per magenta region.

    Regions are 8-connected on the SAMPLED lattice, not on the full image: with
    `stride` 4 the samples are 4 px apart, so 4-connectivity would split a
    single panel along its diagonal edges and the largest "blob" would be a
    fragment rather than the panel.
    """
    if image.width <= 0 or image.height <= 0 or image.step <= 0:
        return []
    if image.encoding not in ('rgb8', 'bgr8'):
        return []

    points, step = _magenta_samples(image, stride)
    blobs = []
    unvisited = set(points)
    while unvisited:
        seed = unvisited.pop()
        queue = [seed]
        min_x = max_x = seed[0]
        min_y = max_y = seed[1]
        count = 0
        while queue:
            x, y = queue.pop()
            count += 1
            min_x, max_x = min(min_x, x), max(max_x, x)
            min_y, max_y = min(min_y, y), max(max_y, y)
            for dx in (-step, 0, step):
                for dy in (-step, 0, step):
                    neighbour = (x + dx, y + dy)
                    if neighbour in unvisited:
                        unvisited.discard(neighbour)
                        queue.append(neighbour)
        blobs.append((min_x, min_y, max_x, max_y, count))
    return blobs


def magenta_bbox(image: Image, stride: int = 4) -> tuple[int, int, int, int, int] | None:
    """
    Return ``min_x, min_y, max_x, max_y, samples`` for the magenta panel.

    The LARGEST connected region, not the span of every magenta pixel in the
    frame. The global min/max this replaces is the R6 defect: any second
    magenta region -- a reflection, emissive bleed, the panel glimpsed twice
    through an opening -- merged into one box and inflated `width_px`. Since
    `marker_pose` reads range as ``fx * marker_width_m / width_px``, inflating
    the width collapses the range, and homing walked to a point that was not
    the exit. Measured over R5+R6: 12 samples, estimated/true 0.478 on average.
    """
    blobs = magenta_blobs(image, stride)
    if not blobs:
        return None
    return max(blobs, key=lambda blob: blob[4])


def blob_diagnostics(
    blobs: list, chosen: tuple | None, stamp_ns: int, range_m: float | None,
) -> dict:
    """
    Summarise one frame's magenta regions for the record.

    `regions` is the number the R6 defect turned on: a second magenta region
    merging into a global bounding box inflated `width_px` and halved the
    published range. Aspect ratio is here because the panel is square (0.80 x
    0.80 m), so a chosen blob far from 1.0 is a partial view -- the residual
    R7 error at range.
    """
    if chosen is None:
        return {'stamp_ns': stamp_ns, 'regions': len(blobs), 'chosen': None}
    min_x, min_y, max_x, max_y, samples = chosen
    width = max_x - min_x + 1
    height = max_y - min_y + 1
    return {
        'stamp_ns': stamp_ns,
        'regions': len(blobs),
        'chosen': {
            'min_x': min_x, 'min_y': min_y, 'max_x': max_x, 'max_y': max_y,
            'width_px': width, 'height_px': height, 'samples': samples,
            'aspect_ratio': round(width / height, 3) if height else None,
            'range_m': None if range_m is None else round(range_m, 3),
        },
        # Widths of every region, so a merge is visible even when the chosen
        # blob looks healthy on its own.
        'region_widths_px': sorted(
            (b[2] - b[0] + 1) for b in blobs)[-4:],
    }


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


def _image_to_bgr_array(image: Image) -> np.ndarray | None:
    """Decode a ROS ``Image`` (rgb8/bgr8, no compression) into a BGR array."""
    if image.width <= 0 or image.height <= 0 or image.step <= 0:
        return None
    if image.encoding not in ('rgb8', 'bgr8'):
        return None
    expected = image.step * image.height
    flat = np.frombuffer(bytes(image.data), dtype=np.uint8)
    if flat.size < expected:
        return None
    array = flat[:expected].reshape((image.height, image.step))
    array = array[:, :image.width * 3].reshape((image.height, image.width, 3))
    if image.encoding == 'rgb8':
        array = array[:, :, ::-1]  # cv2 wants BGR
    return np.ascontiguousarray(array)


def _corners_fully_inside(
    corners: np.ndarray, width: int, height: int, margin_px: float = 1.0,
) -> bool:
    """Reject a tag with a corner at or past the frame edge as a partial view."""
    xs, ys = corners[:, 0], corners[:, 1]
    return bool(
        xs.min() >= margin_px and ys.min() >= margin_px
        and xs.max() <= width - 1 - margin_px
        and ys.max() <= height - 1 - margin_px
    )


def find_fiducial(image: Image, dictionary_name: str, marker_id: int) -> dict:
    """
    Detect fiducial markers in ``image`` and select ``marker_id``.

    Always returns a dict shaped for `fiducial_diagnostics`, even on
    rejection. ``corners`` is populated only when `marker_id` was found fully
    inside the frame; the caller decides whether to also gate on pose
    quality (range, reprojection error), which needs `CameraInfo`.
    """
    array = _image_to_bgr_array(image)
    if array is None:
        return {'ids_found': [], 'corners': None, 'reject_reason': 'imagem invalida'}

    dictionary = cv2.aruco.getPredefinedDictionary(FIDUCIAL_DICTIONARIES[dictionary_name])
    parameters = cv2.aruco.DetectorParameters_create()
    all_corners, ids, _ = cv2.aruco.detectMarkers(array, dictionary, parameters=parameters)
    ids_found = [] if ids is None else [int(i) for i in ids.flatten()]

    if marker_id not in ids_found:
        reason = 'nenhuma tag encontrada' if not ids_found else 'id incorreto'
        return {'ids_found': ids_found, 'corners': None, 'reject_reason': reason}

    corners = all_corners[ids_found.index(marker_id)][0]  # shape (4, 2)
    height, width = array.shape[:2]
    if not _corners_fully_inside(corners, width, height):
        return {
            'ids_found': ids_found, 'corners': corners,
            'reject_reason': 'tag parcialmente fora da imagem',
        }
    return {'ids_found': ids_found, 'corners': corners, 'reject_reason': None}


# Object-space corners in the tag's own frame (Z=0, X right, Y up), in the
# SAME order OpenCV's own ``estimatePoseSingleMarkers`` assumes: top-left,
# top-right, bottom-right, bottom-left. Kept explicit here (rather than
# calling that function) so the reprojection-error check below reuses the
# exact points the pose was solved from.
def _fiducial_object_points(marker_size_m: float) -> np.ndarray:
    half = marker_size_m / 2.0
    return np.array([
        [-half, half, 0.0], [half, half, 0.0],
        [half, -half, 0.0], [-half, -half, 0.0],
    ], dtype=np.float64)


def fiducial_pose(
    corners: np.ndarray, info: CameraInfo, marker_size_m: float,
) -> dict | None:
    """
    Estimate range/bearing and reprojection error for one tag detection.

    Returns ``None`` if `CameraInfo` or the solved pose is unusable -- never
    a confident number computed from garbage intrinsics. Coordinates follow
    the project's camera-frame convention (see `marker_pose` above):
    forward = tvec.z, left = -tvec.x.
    """
    camera_matrix = np.array(info.k, dtype=np.float64).reshape(3, 3)
    if camera_matrix[0, 0] <= 0.0 or marker_size_m <= 0.0:
        return None
    dist_coeffs = (
        np.array(info.d, dtype=np.float64) if len(info.d) else np.zeros(5))

    corners_cv = np.asarray([corners], dtype=np.float32)  # shape (1, 4, 2)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners_cv, marker_size_m, camera_matrix, dist_coeffs)
    rvec, tvec = rvecs[0][0], tvecs[0][0]
    if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
        return None

    forward, left = float(tvec[2]), float(-tvec[0])
    projected, _ = cv2.projectPoints(
        _fiducial_object_points(marker_size_m), rvec, tvec,
        camera_matrix, dist_coeffs)
    reprojection_error_px = float(np.max(np.linalg.norm(
        projected.reshape(-1, 2) - np.asarray(corners, dtype=np.float64), axis=1)))
    return {
        'forward': forward, 'left': left,
        'range_m': math.hypot(forward, left),
        'reprojection_error_px': reprojection_error_px,
    }


def fiducial_diagnostics(
    detection: dict, stamp_ns: int, pose: dict | None, chosen_id: int | None,
) -> dict:
    """Summarise one frame's fiducial detection, published on every frame."""
    corners = detection['corners']
    return {
        'stamp_ns': stamp_ns,
        'backend': 'fiducial',
        'ids_found': detection['ids_found'],
        'corners': None if corners is None else [
            [round(float(x), 2), round(float(y), 2)] for x, y in corners],
        'chosen_id': chosen_id if corners is not None else None,
        'pose': None if pose is None else {
            'range_m': round(pose['range_m'], 3),
            'reprojection_error_px': round(pose['reprojection_error_px'], 2),
        },
        'reject_reason': detection['reject_reason'],
    }


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

        # ML3.5 F5, 29/08/2026: fiducial is the default -- see the module
        # docstring note above `FIDUCIAL_DICTIONARIES`. `detector_backend`
        # set to 'magenta' restores the exact prior behaviour unchanged, as
        # a configurable fallback/diagnostic path.
        self.declare_parameter('detector_backend', 'fiducial')
        self.declare_parameter('fiducial_dictionary', 'DICT_APRILTAG_36h11')
        self.declare_parameter('fiducial_id', 0)
        self.declare_parameter('fiducial_size_m', FIDUCIAL_SIZE_M_DEFAULT)
        # PLACEHOLDER, not yet measured in HIL -- tighten once R9-class
        # evidence exists for this specific camera/lens.
        self.declare_parameter('max_reprojection_error_px', 8.0)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._info: CameraInfo | None = None
        self._history: deque[bool] = deque(maxlen=5)
        self._detections_pub = self.create_publisher(
            Detection2DArray, DETECTIONS_TOPIC, 10)
        self._pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self._diagnostics_pub = self.create_publisher(
            String, DIAGNOSTICS_TOPIC, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_info, qos)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, qos)

    def _on_info(self, message: CameraInfo) -> None:
        self._info = message

    def _on_image(self, image: Image) -> None:
        # Exactly one backend decides, per frame, what (if anything) gets
        # published on POSE_TOPIC -- never two concurrent publishers to the
        # same pose topic (requirement carried over from the fiducial spec).
        backend = str(self.get_parameter('detector_backend').value)
        if backend == 'fiducial':
            self._on_image_fiducial(image)
        else:
            self._on_image_magenta(image)

    def _on_image_fiducial(self, image: Image) -> None:
        stamp_ns = image.header.stamp.sec * 1_000_000_000 \
            + image.header.stamp.nanosec
        marker_id = int(self.get_parameter('fiducial_id').value)
        dictionary_name = str(self.get_parameter('fiducial_dictionary').value)
        marker_size_m = float(self.get_parameter('fiducial_size_m').value)
        max_reprojection_error_px = float(
            self.get_parameter('max_reprojection_error_px').value)

        detection = find_fiducial(image, dictionary_name, marker_id)
        reject_reason = detection['reject_reason']
        pose_data = None

        if detection['corners'] is not None and reject_reason is None:
            if self._info is None:
                reject_reason = 'sem camera_info'
            else:
                pose_data = fiducial_pose(
                    detection['corners'], self._info, marker_size_m)
                if pose_data is None:
                    reject_reason = 'pose nao finita'
                elif pose_data['reprojection_error_px'] > max_reprojection_error_px:
                    reject_reason = 'erro de reprojecao acima do limite'
                    pose_data = None
                elif not FIDUCIAL_MIN_RANGE_M <= pose_data['range_m'] <= FIDUCIAL_MAX_RANGE_M:
                    reject_reason = 'distancia fora dos limites'
                    pose_data = None

        detection['reject_reason'] = reject_reason
        self._diagnostics_pub.publish(String(data=json.dumps(
            fiducial_diagnostics(detection, stamp_ns, pose_data, marker_id))))

        detections = Detection2DArray()
        detections.header = image.header
        if pose_data is not None:
            detections.detections.append(
                self._fiducial_detection(image, detection['corners']))
        self._detections_pub.publish(detections)

        if pose_data is None:
            return
        pose = PoseStamped()
        pose.header = image.header
        pose.pose.position.x = pose_data['forward']
        pose.pose.position.y = pose_data['left']
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

    @staticmethod
    def _fiducial_detection(image: Image, corners: np.ndarray) -> Detection2D:
        xs, ys = corners[:, 0], corners[:, 1]
        detection = Detection2D()
        detection.header = image.header
        detection.id = 'maze_exit'
        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = float(xs.mean())
        detection.bbox.center.position.y = float(ys.mean())
        detection.bbox.size_x = float(xs.max() - xs.min())
        detection.bbox.size_y = float(ys.max() - ys.min())
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'maze_exit'
        hypothesis.hypothesis.score = 1.0
        detection.results.append(hypothesis)
        return detection

    def _on_image_magenta(self, image: Image) -> None:
        window = max(1, int(self.get_parameter('confirmation_window').value))
        if self._history.maxlen != window:
            self._history = deque(self._history, maxlen=window)

        blobs = magenta_blobs(
            image, int(self.get_parameter('sample_stride').value))
        bbox = max(blobs, key=lambda blob: blob[4]) if blobs else None
        valid = bbox is not None
        if bbox is not None:
            valid = (
                bbox[4] >= int(self.get_parameter('min_samples').value)
                and bbox[2] - bbox[0] + 1
                >= int(self.get_parameter('min_bbox_width_px').value)
            )
        stamp_ns = image.header.stamp.sec * 1_000_000_000 \
            + image.header.stamp.nanosec
        range_m = None
        if bbox is not None and self._info is not None:
            estimate = marker_pose(
                bbox, self._info,
                float(self.get_parameter('marker_width_m').value))
            if estimate is not None:
                range_m = math.hypot(*estimate)
        self._diagnostics_pub.publish(String(data=json.dumps(
            blob_diagnostics(blobs, bbox, stamp_ns, range_m))))

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
