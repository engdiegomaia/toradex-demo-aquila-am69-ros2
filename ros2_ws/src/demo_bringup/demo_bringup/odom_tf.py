"""
Close the top of the TF tree: publish `odom` -> `base` from `/demo/odom`.

Runs on the x86 workstation in learn mode and on the Aquila AM69 in hil mode.
No graphics dependency, no architecture dependency.

    ros2 run demo_bringup odom_tf --ros-args -p base_frame:=base

## Why this node exists

Measured on 20/08/2026 with `tools/diagnostics/scenario_check.py`: the Go2's
TF tree has 20 edges, 8 static, rooted at `base` — `base` -> `trunk` ->
`lidar`, `imu_link`, `front_camera`, and the four legs down to the feet. The
robot's own tree is complete. Exactly **two edges are missing at the top**:

- `odom` -> `base`, which is odometry;
- `map` -> `odom`, which is localization.

And `/demo/odom` already declares `header.frame_id: "odom"` in a message whose
frame nobody publishes. Without these two edges Nav2 cannot place the scan in
a costmap, and it fails in a way that never mentions TF.

## What this node is NOT

**It is not state estimation.** It republishes as TF the odometry that already
exists, and in simulation that odometry is **Gazebo ground truth** — see the
header of `demo_simulation/config/bridge_quadruped.yaml`. That is good enough
for exercising perception and planning, and honestly useless as localization
validation: the robot knows exactly where it is because the simulator told it.

Leg-based state estimation is F5, and once it exists **this node goes away**.

## The trap it can cause

Two publishers on the same TF edge do not raise an error: the consumer
receives both and uses whichever arrived last, alternating between two
beliefs. That is why `bridge_quadruped.yaml` deliberately does NOT bridge
`/go2/ground_truth_tf`, and why this node must not run alongside:

- the `/go2/ground_truth_tf` bridge;
- the F5 estimator, once it exists;
- a second instance of itself.

The node warns in the log if it detects another publisher on `/tf` for the
same edge, but the warning is best-effort — the definitive check is
`ros2 run tf2_tools view_frames`.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomTf(Node):
    """Republish `/demo/odom` as the `odom` -> `base` TF edge."""

    def __init__(self) -> None:
        """Declare parameters, arm the edges, and publish the static one if asked."""
        super().__init__('odom_tf')

        # `base`, not `base_link`: it is the name the Go2's URDF uses as its
        # root, and go2_description is vendored with byte-for-byte guarantee,
        # so the frame name is given, not chosen. What has to give is Nav2's
        # parameter file, which belongs to this project.
        self._base = self.declare_parameter('base_frame', 'base').value
        self._odom = self.declare_parameter('odom_frame', 'odom').value
        self._map = self.declare_parameter('map_frame', 'map').value

        # The `map` -> `odom` identity gives Nav2 a `map` frame without SLAM
        # or AMCL. It is not localization: it declares that the planner's
        # world is the odometry's world. It serves reactive obstacle
        # avoidance, which is what the local costmap does. It does NOT serve
        # navigating over a saved map -- there the edge belongs to SLAM or
        # AMCL, and this parameter has to go to false or there will be two
        # publishers on the same edge.
        self._publish_map = self.declare_parameter(
            'publish_map_identity', True).value

        self._broadcaster = TransformBroadcaster(self)
        self._received = 0

        if self._publish_map:
            static = StaticTransformBroadcaster(self)
            static.sendTransform(self._identity(self._map, self._odom))
            self.get_logger().info(
                'published static identity %s -> %s. If you bring up SLAM or '
                'AMCL, set publish_map_identity:=false or there will be two '
                'publishers on that edge.' % (self._map, self._odom))

        self.create_subscription(
            Odometry, '/demo/odom', self._on_odom,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.get_logger().info(
            'publishing %s -> %s from /demo/odom. In simulation this '
            'odometry is Gazebo ground truth, not an estimate: use it to '
            'exercise perception and planning, never as localization '
            'validation.' % (self._odom, self._base))

    def _identity(self, parent: str, child: str) -> TransformStamped:
        """Build an identity transform between two frames."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def _on_odom(self, message: Odometry) -> None:
        """Build the TF edge corresponding to an odometry message."""
        transform = TransformStamped()
        # The stamp comes from the message, not the local clock. Copying the
        # local clock here produces TF that leads or lags the scan, and the
        # costmap drops the reading with "message filter dropping message",
        # which never says the cause is the stamp.
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = message.header.frame_id or self._odom
        transform.child_frame_id = self._base
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._broadcaster.sendTransform(transform)

        self._received += 1
        if self._received == 1:
            self.get_logger().info(
                'first odometry received in frame "%s"; tree closed up to '
                '"%s"' % (transform.header.frame_id, self._base))


def main(args=None) -> None:
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = OdomTf()
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
