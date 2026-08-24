"""
Synthetic detection stub — the perception half of ML3.

Runs on: Aquila AM69 (arm64) in target mode, x86 host in learn mode. Same code
either way; it is CPU-only and has no GPU or OpenGL dependency.

Subscribes to /demo/camera/image_raw and republishes a deterministic
Detection2DArray on /demo/perception/detections. It never inspects pixel
content and never learns where the image came from — Gazebo, a USB camera and a
rosbag are indistinguishable from here. That is the whole point of the topic
contract (CLAUDE.md rule 6): replacing this node with real TIDL inference must
be a container swap, not an interface change.

Determinism matters. The detection walks a fixed sinusoidal path across the
frame as a function of the received-frame counter, so a given frame index always
yields the same box. That makes downstream behaviour reproducible while the real
model does not exist yet.
"""

import math

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class DetectionStub(Node):
    """Publish deterministic synthetic detections derived from frame count."""

    IMAGE_TOPIC = '/demo/camera/image_raw'
    DETECTIONS_TOPIC = '/demo/perception/detections'

    DEFAULT_CLASS_ID = 'box'
    DEFAULT_SCORE = 0.87
    DEFAULT_BOX_WIDTH_PX = 120.0
    DEFAULT_BOX_HEIGHT_PX = 160.0
    DEFAULT_PERIOD_FRAMES = 90.0

    def __init__(self) -> None:
        super().__init__('detection_stub')

        self.declare_parameter('class_id', self.DEFAULT_CLASS_ID)
        self.declare_parameter('score', self.DEFAULT_SCORE)
        self.declare_parameter('box_width_px', self.DEFAULT_BOX_WIDTH_PX)
        self.declare_parameter('box_height_px', self.DEFAULT_BOX_HEIGHT_PX)
        self.declare_parameter('period_frames', self.DEFAULT_PERIOD_FRAMES)

        self._validate_score(float(self.get_parameter('score').value))
        self._validate_period(float(self.get_parameter('period_frames').value))

        # The ros_gz_bridge camera publisher is RELIABLE. This matters for the
        # 921600-byte rgb8 samples used by the demo: over the HIL Ethernet link,
        # a BEST_EFFORT reader was discovered normally but lost every fragmented
        # sample, while a RELIABLE probe received the full 10 Hz stream. Keep the
        # request aligned with the measured producer so DDS can retransmit a
        # missing fragment instead of silently dropping the whole frame.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._publisher = self.create_publisher(
            Detection2DArray, self.DETECTIONS_TOPIC, 10)
        self._subscription = self.create_subscription(
            Image, self.IMAGE_TOPIC, self._on_image, sensor_qos)

        self._frame_count = 0

        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info(
            f'detection_stub up: {self.IMAGE_TOPIC} -> {self.DETECTIONS_TOPIC} '
            f'(synthetic, deterministic)')

    def _on_image(self, image: Image) -> None:
        """Emit one synthetic detection per received frame."""
        detections = Detection2DArray()
        # Reusing the image header keeps frame_id and the sim timestamp intact,
        # which is what lets the costmap transform the detection correctly.
        detections.header = image.header
        detections.detections.append(
            self._build_detection(image, self._frame_count))

        self._publisher.publish(detections)
        self._frame_count += 1

    def _build_detection(self, image: Image, frame_index: int) -> Detection2D:
        detection = Detection2D()
        detection.header = image.header
        detection.id = self.get_parameter('class_id').value

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.get_parameter('class_id').value
        hypothesis.hypothesis.score = float(self.get_parameter('score').value)
        detection.results.append(hypothesis)

        detection.bbox = self._build_bbox(image, frame_index)
        return detection

    def _build_bbox(self, image: Image, frame_index: int) -> BoundingBox2D:
        """Sweep the box horizontally as a pure function of the frame index."""
        width_px = float(self.get_parameter('box_width_px').value)
        height_px = float(self.get_parameter('box_height_px').value)
        period = float(self.get_parameter('period_frames').value)

        # Guard against a zero-size image: a rosbag with an empty frame would
        # otherwise place the box at negative coordinates.
        image_width = float(image.width) if image.width else 640.0
        image_height = float(image.height) if image.height else 480.0

        phase = (2.0 * math.pi * frame_index) / period
        # Keep the whole box inside the frame: the centre travels only over the
        # span that leaves half a box-width of margin on each side.
        travel = max(0.0, (image_width - width_px) / 2.0)
        centre_x = (image_width / 2.0) + travel * math.sin(phase)
        centre_y = image_height / 2.0

        bbox = BoundingBox2D()
        bbox.center.position.x = centre_x
        bbox.center.position.y = centre_y
        bbox.center.theta = 0.0
        bbox.size_x = width_px
        bbox.size_y = height_px
        return bbox

    @staticmethod
    def _validate_score(score: float) -> None:
        if not 0.0 <= score <= 1.0:
            raise ValueError(f'score must be in [0.0, 1.0], got {score}')

    @staticmethod
    def _validate_period(period: float) -> None:
        if period <= 0.0:
            raise ValueError(f'period_frames must be > 0, got {period}')

    def _on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            try:
                if param.name == 'score':
                    self._validate_score(float(param.value))
                elif param.name == 'period_frames':
                    self._validate_period(float(param.value))
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetectionStub()
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
