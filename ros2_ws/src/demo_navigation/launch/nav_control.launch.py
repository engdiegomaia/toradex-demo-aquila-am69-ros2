"""
Navigation control facade -- the cockpit's "reset goal".

Runs on: Aquila AM69 (arm64) in hil mode, x86 workstation in learn mode. Always
in the SAME container as Nav2, because what it does is manipulate Nav2's
lifecycle.

SHARED fragment, included by:

    demo_navigation/navigation.launch.py   static-map path + AMCL
                                           (nav.launch.py and learn.launch.py)
    demo_bringup/nav_quadruped.launch.py   reactive path of the Go2

It is not a container entrypoint and does not appear in compose -- the same
role demo_simulation/launch/scene_cameras.launch.py plays on the simulator side.

WHY A FRAGMENT INSTEAD OF TWO COPIES OF THE Node

The two navigation paths are deliberately separate (nav_quadruped.launch.py
explains why), but this facade is identical in both: it talks to
`lifecycle_manager_navigation`, which both bring up. Duplicating the `Node`
block would mean two things to keep in sync, and the way that breaks is the
worst possible -- the cockpit button works on one ROBOT_TYPE and not the other,
with no error anywhere.

    ros2 launch ... nav_control:=false

turns the facade off, to bring up navigation with no restart service exposed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enabled_arg = DeclareLaunchArgument(
        'nav_control',
        default_value='true',
        description='Expose /demo/nav/{reset,cancel} to the cockpit.',
    )

    # The use_sim_time argument WAS REMOVED on 25/08/2026, on purpose.
    #
    # Keeping a declared argument nobody consumes is a silent failure: whoever
    # passed use_sim_time:=true would see the value accepted and ignored, with
    # not a line of log. Removed, the same call fails LOUD -- "is not a valid
    # launch argument" -- and the caller finds out at once, not in a CPU
    # measurement.
    #
    # Why nobody consumes it: see the block in `relay` below.

    relay = Node(
        package='demo_navigation',
        executable='nav_control_relay',
        name='nav_control_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav_control')),
        # use_sim_time is NOT set, and the reversal dates from 25/08/2026.
        #
        # The previous comment said "YES, so the node lives in the same time as
        # the rest of the stack" -- and admitted in the next sentence that the
        # internal timeouts do NOT use that clock (the docstring of `_wait`
        # explains: a Nav2 deactivated in the middle of a RESET coexists with a
        # stopped /clock, and measuring a timeout there would mean waiting
        # forever). That is still true, and so is the rest: this node has NOT ONE
        # call to get_clock(), no timer, no stamp. Living "in the same time"
        # bought nothing, because it never asks what time it is.
        #
        # What it did buy was cost: 35% of a core on the AM69 just receiving
        # /clock at ~870 Hz. See the IDLE CPU FLOOR block in
        # demo_bringup/launch/nav_quadruped.launch.py for the full measurement.
        #
        # It applies to BOTH robots, because this launch is shared. There is no
        # per-mode path here, and there must not be.
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([enabled_arg, relay])
