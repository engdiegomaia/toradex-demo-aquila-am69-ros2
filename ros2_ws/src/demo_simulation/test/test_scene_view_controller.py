"""
Scene camera orbit: the math, with no node and no Gazebo.

What goes wrong here does not show up as an error anywhere -- it shows up as
a camera pointing at the wrong place, or one that does not move. That is why
the target of these tests is the `Orbit` class, the only part with real
logic and the only one that can be exercised without a simulator.
"""

import math

from demo_simulation.scene_view_controller import (
    MAX_FOLLOW_OFFSET_M,
    MIN_PITCH_RAD,
    Orbit,
)
from geometry_msgs.msg import Twist


def _twist(**fields):
    """Build a Twist with one axis moved, the other five at zero."""
    message = Twist()
    for name, value in fields.items():
        axis, component = name.split('_')
        setattr(getattr(message, axis), component, value)
    return message


def _iso():
    """Build the launch default iso view."""
    return Orbit(-3.0, 3.0, 2.4, 0.5150, -0.7854)


def _top():
    """Build the launch default top view: the degenerate case, pitch = pi/2."""
    return Orbit(0.0, 0.0, 6.0, math.pi / 2, math.pi / 2)


# --- the measured framing must survive a round trip -------------------------

def test_orbit_round_trips_the_launch_pose():
    """
    Position() must return the pose the camera was born with.

    The orbit is seeded by (x, y, z, pitch, yaw) and stores (target, distance,
    pitch, yaw). If the inversion is wrong, the camera JUMPS on the first
    command -- it is born in the right place, spawned by `create`, and moves
    to another one on the first set_pose.
    """
    for orbit in (_iso(), _top()):
        x, y, z = orbit.position()
        expected = orbit.home[:3]
        assert math.isclose(x, expected[0], abs_tol=1e-9)
        assert math.isclose(y, expected[1], abs_tol=1e-9)
        assert math.isclose(z, expected[2], abs_tol=1e-9)


def test_top_view_target_is_the_point_below_the_camera():
    """With pitch = pi/2 the target is the point under the camera: cos(pitch) = 0."""
    top = _top()
    assert math.isclose(top.target[0], 0.0, abs_tol=1e-9)
    assert math.isclose(top.target[1], 0.0, abs_tol=1e-9)


# --- following the robot ----------------------------------------------------

def test_following_moves_the_camera_by_the_robot_displacement():
    """
    Following moves the TARGET, and only the target.

    Distance, azimuth and pitch are the framing someone measured against the
    scenario's real footprint (see the header of scene_cameras.launch.py).
    Following the robot is no reason to touch them, and the test that
    guarantees this is this one: the camera slides, it does not reframe.
    """
    orbit = _iso()
    before = (orbit.distance, orbit.pitch, orbit.yaw)
    start = orbit.position()
    # The home target is NOT exactly the origin: the launch yaw is -0.7854, a
    # rounded pi/4, leaving ~1 mm over. Comparing against (0, 0) would make
    # this test fail for an error that is not what it guards.
    target_before = orbit.target

    orbit.follow((2.0, -1.0))
    moved = orbit.position()

    assert (orbit.distance, orbit.pitch, orbit.yaw) == before
    # The camera moves the SAME vector the target moved.
    assert math.isclose(moved[0] - start[0], 2.0 - target_before[0], abs_tol=1e-9)
    assert math.isclose(moved[1] - start[1], -1.0 - target_before[1], abs_tol=1e-9)
    assert math.isclose(moved[2], start[2], abs_tol=1e-9)


def test_following_the_top_view_puts_the_camera_over_the_robot():
    """
    In the top view, following means staying ABOVE the robot.

    This is the degenerate case of the orbital model and the most visible one
    on screen: if the target and the position diverge here, the robot appears
    at the edge of the frame instead of the centre. The height does not
    change -- the top view's zoom IS the height.
    """
    top = _top()
    height = top.position()[2]

    top.follow((-4.855, 4.855))
    x, y, z = top.position()

    assert math.isclose(x, -4.855, abs_tol=1e-9)
    assert math.isclose(y, 4.855, abs_tol=1e-9)
    assert math.isclose(z, height, abs_tol=1e-9)


def test_pan_while_following_survives_the_next_follow_tick():
    """
    The pan must live in the OFFSET while following.

    Written to the target, it would be overwritten by the next tick's
    `follow()` 100 ms later: the operator presses "move right", the image
    flicks back and the button seems not to work. That is why `apply` gained
    the `following` argument.
    """
    orbit = _iso()
    orbit.follow((0.0, 0.0))

    orbit.apply(_twist(linear_y=0.8), following=True)
    panned = orbit.position()

    # The next tick, with the robot stopped in the same place.
    orbit.follow((0.0, 0.0))
    assert orbit.position() == panned, 'the pan was swallowed by the follow tick'
    assert orbit.follow_offset != (0.0, 0.0)


def test_pan_when_not_following_moves_the_world_target():
    """Stopped, the pan still moves a point in the world, as before."""
    orbit = _iso()
    target = orbit.target

    orbit.apply(_twist(linear_y=0.8), following=False)

    assert orbit.target != target
    assert orbit.follow_offset == (0.0, 0.0), (
        'panning in free mode must not dirty the follow offset'
    )


def test_follow_offset_is_bounded():
    """
    A finger stuck on the pan button must not lose sight of the robot.

    With no cap, holding "move" while following pushes the target away
    indefinitely and the robot leaves the frame -- exactly what following
    exists to prevent. There is no button that brings it back, only
    recentring.
    """
    orbit = _iso()
    for _ in range(200):
        orbit.apply(_twist(linear_y=0.8), following=True)

    assert math.hypot(*orbit.follow_offset) <= MAX_FOLLOW_OFFSET_M + 1e-9


def test_reset_clears_the_follow_offset():
    """
    Recentring means the same thing in both modes.

    Stopped, it goes back to the measured framing. Following, it goes back
    to having the robot in the centre -- which only happens if the offset
    is zeroed here.
    """
    orbit = _iso()
    orbit.apply(_twist(linear_y=3.0), following=True)
    home = _iso().position()

    orbit.reset()

    assert orbit.follow_offset == (0.0, 0.0)
    assert orbit.position() == home


# --- the limits that existed before still hold -------------------------------

def test_pitch_never_reaches_the_ground():
    """
    Below MIN_PITCH_RAD the orbit loses the target.

    sin(pitch) -> 0 and the distance explodes; the clamp is what prevents
    division by near-zero in `reset`.
    """
    orbit = _iso()
    for _ in range(50):
        orbit.apply(_twist(angular_y=-0.18))

    assert orbit.pitch >= MIN_PITCH_RAD
