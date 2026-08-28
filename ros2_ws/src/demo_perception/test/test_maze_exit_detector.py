import math

from demo_perception.maze_exit_detector import magenta_bbox, marker_pose
import pytest
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
