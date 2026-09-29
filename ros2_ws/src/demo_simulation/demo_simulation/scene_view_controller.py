"""
Orbital camera for the scene views — what the cockpit buttons move.

Runs on: x86 workstation ONLY, in the `sim` container. It talks to Gazebo's
`set_pose` service, which only exists inside the simulator process (rule 1
of CLAUDE.md).

WHY THIS NODE EXISTS, INSTEAD OF THE COCKPIT CALLING set_pose DIRECTLY

Because `set_pose` only accepts an ABSOLUTE pose, and "rotate 15 degrees to
the left" is a relative operation. Someone needs to know where the camera is
right now. If that someone were the browser, the framing would be declared
in two places — in `scene_cameras.launch.py`'s arguments and again in
JavaScript — and they would diverge the moment a new scenario appeared.
Worse: with `SIM_ARGS` reframing the cameras for the maze, the first click
of a button would yank the camera from the maze framing to a warehouse
default written into the browser.

So the orbital state lives here, next to the cameras, seeded by the SAME
parameters that positioned them at spawn. The cockpit only publishes a step,
and does not need to know anything about geometry.

ORBITAL MODEL

A camera is (target T on the ground, distance d, azimuth a, pitch p):

    P = T - d * (cos p * cos a, cos p * sin a, -sin p)

which inverts, given P and (p, a) with the target on the z = 0 plane:

    d = Pz / sin p        T = (Px, Py) + d * cos p * (cos a, sin a)

The TOP view is the degenerate case and needs no code of its own: with
p = pi/2 you get d = Pz and T = P, i.e. the target is the point under the
camera, "zoom" becomes height, and "rotate" becomes the image spinning on
its own axis. An `if` for the top view would be a second model to keep in
sync with the first.

FOLLOWING THE ROBOT

Both views follow the robot by default: the orbit's target becomes the
robot's pose plus whatever pan the operator applied. Distance, azimuth and
pitch are not touched, so the iso view keeps the measured framing and slides
along with the robot, and the top view (pitch = pi/2, target = point under
the camera) stays over it.

Without this, on maze11 the robot leaves the iso view's frame within a few
metres, and the operator loses precisely the screen's independent witness.

Following does NOT disable panning: while following, the move buttons shift
the OFFSET relative to the robot, not a point in the world. That is why
holding "move right" keeps doing what it says, with the robot staying where
the operator left it in the frame.

The switch is /demo/cockpit/scene/follow (std_srvs/SetBool), because the
static wide view — the one showing the whole maze, with the framing MEASURED
in scene_cameras.launch.py — remains a resource and must not disappear
without a button.

And the state comes out of here, on /demo/cockpit/scene/following
(std_msgs/Bool, latched), not from the cockpit's last click. Same rule as
the simulation label: the cockpit can be reloaded, opened in two tabs, or
opened after someone turned following off from the command line, and in all
three cases a button painted by its own click would be lying. Latched
(TRANSIENT_LOCAL) so a new tab receives the value without waiting for the
next change.

WHERE THE ROBOT'S POSE COMES FROM, AND THE TRAP THAT LIVES IN THAT

From /demo/odom. What this node needs is the pose in the WORLD frame, the
only one Gazebo's set_pose understands, and /demo/odom only coincides with
it by luck of configuration:

  quadruped    /go2/odom comes from gz-sim-odometry-publisher-system, which
               publishes the model's exact pose in the world (ground
               truth). Always coincides, including with maze11's
               `yaw:=1.5708`.

  diff-drive   /odom comes from the DiffDrive plugin, which INTEGRATES the
               encoders starting from zero. The odom origin is the SPAWN
               pose, not the world's. Only coincides while the spawn's
               x/y/yaw are 0 — which is the default.

That is why the follow_offset_{x,y,yaw} parameters exist: they are the
odom -> world transform, and simulation.launch.py ties them to the SAME
x/y/yaw that spawned the robot. quadruped.launch.py leaves them at zero, on
purpose.

Without that seed, an `x:=5` on the diff-drive would make the camera follow
a ghost 5 m beside the robot — wrong by a constant offset, which is exactly
the kind of error that reads as "the camera is a bit off" rather than "the
reference frame is wrong".

CONTRACT

    /demo/cockpit/scene/cmd_view    geometry_msgs/TwistStamped   (in)
    /demo/cockpit/scene/reset_view  std_srvs/Trigger             (in)
    /demo/cockpit/scene/follow      std_srvs/SetBool             (in)
    /demo/odom                      nav_msgs/Odometry            (in)
    /demo/cockpit/scene/following   std_msgs/Bool                (out, latched)
    /demo/sim/set_entity_pose       ros_gz_interfaces/SetEntityPose (out)

The TwistStamped's `header.frame_id` selects the camera: `scene_iso` or
`scene_top`. Twist has the six degrees of freedom the orbit needs and is a
standard message — CLAUDE.md asks not to redefine Twist equivalents:

    angular.z   azimuth, rad          rotate around the target
    angular.y   pitch, rad            raise/lower the viewpoint
    linear.x    move closer, m        negative = zoom in
    linear.y    lateral shift, m      pan on the image's left/right axis
    linear.z    forward shift, m      pan on the image's in/out axis

Steps come from the client and not from here, on purpose: the step size is
an interface decision, and it is the UI that knows whether the operator held
the button down.
"""

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
)
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

# Limits. The camera must not pass the zenith (the orbit loses azimuth) or
# sink into the ground (the view turns into a wall of texture), and without a
# distance ceiling an operator with a finger stuck on the button sends the
# camera into space, from where no button brings it back — only the reset.
MIN_PITCH_RAD = 0.12
MAX_PITCH_RAD = math.pi / 2
MIN_DISTANCE_M = 1.0
MAX_DISTANCE_M = 80.0

# A target too far away is also unrecoverable in practice.
MAX_TARGET_RADIUS_M = 60.0

# How far the target may drift from the ROBOT while the view is following it.
# Smaller than MAX_TARGET_RADIUS_M because the question is different: there
# it is "don't send the camera into space", here it is "don't lose the robot
# from view with the pan button itself".
MAX_FOLLOW_OFFSET_M = 15.0

# How often the followed pose is rewritten in Gazebo. The scene cameras
# render at 10 Hz (models/cockpit_scene_*.sdf), so pushing faster spends a
# service call with no new render to show for it.
FOLLOW_PERIOD_S = 0.1

# Movement below this is not worth a set_pose call. With the robot stopped,
# odometry keeps arriving at 50 Hz and jitters in the last millimetre;
# without this deadband the node would call set_pose 10 times a second
# forever.
FOLLOW_DEADBAND_M = 0.02

# Two different things have similar names, and confusing them costs an
# afternoon:
#
#   scene_iso           the SENSOR, and the topic prefix
#                       (/demo/cockpit/scene_iso/image_raw)
#   cockpit_scene_iso   the Gazebo MODEL, which is what `set_pose` moves
#
# `set_pose` on a sensor name returns success=false and nothing moves. Since
# both spellings genuinely exist in the system, both are accepted here and
# resolved to the model name — instead of forcing the caller to know which
# of the two Gazebo wanted.
MODELS = {
    'scene_iso': 'cockpit_scene_iso',
    'scene_top': 'cockpit_scene_top',
}
# Both spellings resolve to the orbit's KEY, never to the model name: the
# model name is what leaves here toward Gazebo, not what indexes the state.
# Swapping the two sides would make every legitimate message fall into the
# 'camera does not exist' branch — with a message that lists exactly the name
# that was sent.
ALIASES = {**{key: key for key in MODELS},
           **{model: key for key, model in MODELS.items()}}
CAMERAS = tuple(MODELS)


class Orbit:
    """Orbital state of ONE camera, and the conversion to and from pose."""

    def __init__(self, x, y, z, pitch, yaw):
        self.home = (x, y, z, pitch, yaw)
        # Last known robot pose, or None. It lives ON the orbit and not just
        # on the node because `apply` needs it: without this, a pan applied
        # while following leaves `target` stale until the next tick, and
        # `position()` lies during that interval — which is exactly the
        # moment the command's `_push` reads the pose to send to Gazebo.
        self.anchor = None
        self.reset()

    def reset(self):
        x, y, z, pitch, yaw = self.home
        self.pitch = min(max(pitch, MIN_PITCH_RAD), MAX_PITCH_RAD)
        self.yaw = yaw
        # sin(pitch) is never zero because of MIN_PITCH_RAD; a horizontal
        # camera does not cross the ground plane and has no defined target.
        self.distance = min(max(z / math.sin(self.pitch), MIN_DISTANCE_M),
                            MAX_DISTANCE_M)
        reach = self.distance * math.cos(self.pitch)
        self.target = (x + reach * math.cos(self.yaw),
                       y + reach * math.sin(self.yaw))
        # Pan accumulated WHILE FOLLOWING, relative to the robot. Zeroing it
        # here is what makes "recentre" mean the same thing in both modes:
        # stopped, it returns to the measured framing; following, it puts
        # the robot back in the centre.
        #
        # `anchor` is NOT forgotten: recentring is about framing, not about
        # forgetting where the robot is. The node decides whether the target
        # goes back to the robot right afterwards, and it only does that
        # while following.
        self.follow_offset = (0.0, 0.0)

    def follow(self, anchor):
        """
        Store the robot's pose and retarget. Called on every tick.

        Only the TARGET moves: distance, azimuth and pitch are the framing
        someone measured, and following the robot is no reason to touch
        them.
        """
        self.anchor = anchor
        self._retarget()

    def _retarget(self):
        """Target = robot + pan. With no known robot, the target stays put."""
        if self.anchor is None:
            return
        self.target = (self.anchor[0] + self.follow_offset[0],
                       self.anchor[1] + self.follow_offset[1])

    def position(self):
        reach = self.distance * math.cos(self.pitch)
        return (self.target[0] - reach * math.cos(self.yaw),
                self.target[1] - reach * math.sin(self.yaw),
                self.distance * math.sin(self.pitch))

    def apply(self, twist, following=False):
        """Apply a relative step, already clamped to the limits."""
        self.yaw = _wrap(self.yaw + twist.angular.z)
        self.pitch = min(max(self.pitch + twist.angular.y, MIN_PITCH_RAD),
                         MAX_PITCH_RAD)
        self.distance = min(max(self.distance + twist.linear.x, MIN_DISTANCE_M),
                            MAX_DISTANCE_M)

        # Pan in the IMAGE frame, not the world's: the operator is looking at
        # the screen and "to the right" has to mean to the right on screen,
        # whatever the azimuth. `forward` is the sightline's projection onto
        # the ground.
        forward = (math.cos(self.yaw), math.sin(self.yaw))
        right = (forward[1], -forward[0])
        dx = right[0] * twist.linear.y + forward[0] * twist.linear.z
        dy = right[1] * twist.linear.y + forward[1] * twist.linear.z

        # While following, panning moves the OFFSET and not a point in the
        # world. Writing to the target here would be writing to a field the
        # next `follow()` overwrites 100 ms later — the move button would
        # look like it has no effect, which is the worst way for this to
        # break.
        if following:
            self.follow_offset = _clamp_radius(
                (self.follow_offset[0] + dx, self.follow_offset[1] + dy),
                MAX_FOLLOW_OFFSET_M,
            )
            # Retarget NOW, not on the next tick: the caller is about to read
            # position() next to write the pose to Gazebo.
            self._retarget()
            return

        self.target = _clamp_radius((self.target[0] + dx, self.target[1] + dy),
                                    MAX_TARGET_RADIUS_M)


def _clamp_radius(point, limit):
    """Shrink the vector to the maximum radius, preserving direction."""
    radius = math.hypot(point[0], point[1])
    if radius <= limit:
        return point
    scale = limit / radius
    return (point[0] * scale, point[1] * scale)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _quaternion(pitch, yaw):
    """RPY -> quaternion with roll = 0. A scene camera never tilts."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


class SceneViewController(Node):
    def __init__(self):
        super().__init__('scene_view_controller')

        # The defaults DUPLICATE those of scene_cameras.launch.py, and that is
        # why the launch file passes them through explicitly: anything
        # started via launch never falls back to these values. They exist
        # only so a standalone `ros2 run` does not blow up.
        self.declare_parameter('iso_x', -3.0)
        self.declare_parameter('iso_y', 3.0)
        self.declare_parameter('iso_z', 2.4)
        self.declare_parameter('iso_pitch', 0.5150)
        self.declare_parameter('iso_yaw', -0.7854)
        self.declare_parameter('top_x', 0.0)
        self.declare_parameter('top_y', 0.0)
        self.declare_parameter('top_z', 6.0)
        self.declare_parameter('top_pitch', math.pi / 2)
        self.declare_parameter('top_yaw', math.pi / 2)

        # Follow the robot. On by default: losing sight of the robot is the
        # blue panel's common failure mode, and the static wide view remains
        # one click away (/demo/cockpit/scene/follow).
        self.declare_parameter('follow', True)
        self.declare_parameter('follow_topic', '/demo/odom')
        # odom -> world. Zero when odometry is ground truth (quadruped); the
        # spawn pose when it is integrated from the encoders (diff-drive).
        # See the "WHERE THE ROBOT'S POSE COMES FROM" section in the module
        # docstring — this is the parameter behind the silent error described
        # there.
        self.declare_parameter('follow_offset_x', 0.0)
        self.declare_parameter('follow_offset_y', 0.0)
        self.declare_parameter('follow_offset_yaw', 0.0)

        def orbit(prefix):
            def value(name):
                return self.get_parameter(f'{prefix}_{name}').value

            return Orbit(value('x'), value('y'), value('z'),
                         value('pitch'), value('yaw'))

        self._orbits = {'scene_iso': orbit('iso'), 'scene_top': orbit('top')}

        # ReentrantCallbackGroup: the call to set_pose happens INSIDE the
        # topic's callback. With the default mutually exclusive group the
        # executor does not run the service response until the topic
        # callback returns, and the node hangs on the first button press —
        # with no error at all.
        group = ReentrantCallbackGroup()

        self._set_pose = self.create_client(
            SetEntityPose, '/demo/sim/set_entity_pose', callback_group=group)

        self.create_subscription(
            TwistStamped, '/demo/cockpit/scene/cmd_view', self._on_command, 10,
            callback_group=group)

        self.create_service(
            Trigger, '/demo/cockpit/scene/reset_view', self._on_reset,
            callback_group=group)

        # --- follow the robot -------------------------------------------------
        self._following = bool(self.get_parameter('follow').value)
        self._seed = (self.get_parameter('follow_offset_x').value,
                      self.get_parameter('follow_offset_y').value,
                      self.get_parameter('follow_offset_yaw').value)
        # No sample yet: until the first one, following changes nothing and
        # both views stay at the framing the launch file measured. An
        # initial `(0, 0)` would yank the cameras from the maze to the
        # world's origin before the robot has even published odometry.
        self._anchor = None

        follow_topic = self.get_parameter('follow_topic').value
        # SENSOR_DATA, the same profile demo_bringup/odom_tf.py already uses
        # to read this topic — and that is the only /demo/odom path
        # validated on the Aquila. Best-effort against the bridge's reliable
        # publisher is compatible; choosing another profile here would be
        # debuting a new QoS combination on the camera, and incompatible QoS
        # gives no error, only silence.
        self.create_subscription(
            Odometry, follow_topic, self._on_odom, qos_profile_sensor_data,
            callback_group=group)

        self.create_service(
            SetBool, '/demo/cockpit/scene/follow', self._on_follow,
            callback_group=group)

        # Published state, and latched. See the "FOLLOWING THE ROBOT" section
        # in the module docstring: the cockpit button is painted by this,
        # not by its own click.
        self._following_pub = self.create_publisher(
            Bool, '/demo/cockpit/scene/following',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._announce_following()

        # A timer, and not the odometry callback itself: odometry arrives at
        # 50 Hz and the cameras render at 10 Hz. Pushing set_pose at 50 Hz
        # would spend five service calls per rendered frame.
        self.create_timer(FOLLOW_PERIOD_S, self._on_follow_tick,
                          callback_group=group)

        framing = ', '.join(
            f'{name} at ({x:.2f}, {y:.2f}, {z:.2f})'
            for name, (x, y, z) in
            ((n, o.position()) for n, o in self._orbits.items()))
        # rclpy has no printf-style logging: RcutilsLogger accepts ONE
        # string. Passing positional args raises TypeError while
        # constructing the node, which dies before it exists for any
        # diagnostics.
        self.get_logger().info(
            f'view control ready: {framing}; '
            f'following={self._following} via {follow_topic} '
            f'with odom->world seed {self._seed}')

    # --- input -----------------------------------------------------------

    def _on_command(self, message):
        requested = message.header.frame_id or 'scene_iso'
        name = ALIASES.get(requested)
        orbit = self._orbits.get(name) if name else None
        if orbit is None:
            # Name the options: a wrong frame_id is a typo in the client, and
            # "unknown camera" without the list sends the person to read code
            # to find out the right name.
            self.get_logger().warning(
                f'camera "{requested}" does not exist; '
                f'use one of {", ".join(CAMERAS)}')
            return
        orbit.apply(message.twist, following=self._is_following())
        self._apply_follow(orbit)
        self._push(name, orbit)

    def _on_reset(self, request, response):
        del request
        for name, orbit in self._orbits.items():
            orbit.reset()
            self._apply_follow(orbit)
            self._push(name, orbit)
        response.success = True
        response.message = 'scene views back to the initial framing'
        return response

    def _on_odom(self, message):
        """
        Store the robot pose already in the world frame.

        The composition is `world = seed ∘ odom`, with the seed coming from the
        parameters: rotate the point by the spawn yaw, then translate. Applying
        only the translation would be right as long as the spawn does not
        rotate -- and maze11 is precisely the one that rotates (`yaw:=1.5708`).
        """
        position = message.pose.pose.position
        sx, sy, syaw = self._seed
        if syaw:
            cos_yaw, sin_yaw = math.cos(syaw), math.sin(syaw)
            x = cos_yaw * position.x - sin_yaw * position.y
            y = sin_yaw * position.x + cos_yaw * position.y
        else:
            x, y = position.x, position.y
        self._anchor = (x + sx, y + sy)

    def _on_follow(self, request, response):
        self._following = bool(request.data)
        self._announce_following()
        if not self._following:
            # Switching off leaves the cameras EXACTLY where they are, aimed at
            # the last followed point. Returning to the initial framing is what
            # "recenter" does, and doing both on this button would take from the
            # operator the only way to freeze a view that is good.
            response.success = True
            response.message = 'scene views frozen where they are'
            return response

        # When switching back on, the pan accumulated in free mode is
        # meaningless as an offset relative to the robot: it was measured
        # against the world. Zeroing it is what makes "follow" mean "robot at
        # the centre" again.
        for name, orbit in self._orbits.items():
            orbit.follow_offset = (0.0, 0.0)
            self._apply_follow(orbit)
            self._push(name, orbit)
        response.success = True
        response.message = 'scene views following the robot'
        return response

    def _on_follow_tick(self):
        if not self._is_following():
            return
        for name, orbit in self._orbits.items():
            before = orbit.position()
            self._apply_follow(orbit)
            after = orbit.position()
            # Deadband on the CAMERA POSITION, not on the robot's: in the iso
            # view a robot step moves the camera by the same amount, but in the
            # top view with pitch = pi/2 there are reasons for the two numbers
            # to diverge, and what decides whether a call is worth it is what
            # the camera does.
            if math.dist(before, after) < FOLLOW_DEADBAND_M:
                continue
            self._push(name, orbit)

    # --- state ------------------------------------------------------------

    def _announce_following(self):
        self._following_pub.publish(Bool(data=self._following))

    def _is_following(self):
        """Following for real needs a target: without odometry there is nothing to follow."""
        return self._following and self._anchor is not None

    def _apply_follow(self, orbit):
        if self._is_following():
            orbit.follow(self._anchor)

    # --- output -------------------------------------------------------------

    def _push(self, name, orbit):
        if not self._set_pose.service_is_ready():
            # The service bridge comes up along with Gazebo and may take a
            # while. Saying so is better than queueing calls nobody will answer.
            self.get_logger().warning(
                '/demo/sim/set_entity_pose does not exist yet; '
                'is the ros_gz simulation control bridge up?')
            return

        x, y, z = orbit.position()
        qx, qy, qz, qw = _quaternion(orbit.pitch, orbit.yaw)

        request = SetEntityPose.Request()
        request.entity.name = MODELS[name]
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        future = self._set_pose.call_async(request)
        future.add_done_callback(lambda done: self._log_result(name, done))

    def _log_result(self, name, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - we want any failure in the log
            self.get_logger().error(f'set_pose for {name} failed: {error}')
            return
        if not result.success:
            # Gazebo returns success=false when the model does not exist, which
            # is exactly what happens when the world came up without the
            # cameras.
            self.get_logger().warning(
                f'Gazebo refused to move "{name}"; was the model spawned?')


def main(args=None):
    rclpy.init(args=args)
    node = SceneViewController()
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
