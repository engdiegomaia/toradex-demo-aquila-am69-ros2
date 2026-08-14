"""
Plant container entrypoint — Gazebo, the robot, and the ROS<->gz bridge.

Runs on: x86 workstation ONLY, in the `sim` container. Gazebo is an OGRE 2 /
desktop-OpenGL application and must never be placed on the Aquila AM69
(CLAUDE.md rule 1). In hil mode this container still runs on the host; only nav
and perception move to the module.

    ros2 launch demo_bringup sim.launch.py

This is the ML3.5 F1 decomposition of learn.launch.py: one launch file per
container role, each the entrypoint of exactly one compose service. It wraps
demo_simulation/simulation.launch.py and adds nothing to it. The 12 s spawn and
15 s bridge delays that the demo depends on live inside that file and are
unchanged — see the long notes there for why they are load-bearing.

`robot_type` is accepted here in F1 but only `diffdrive` is implemented. The
compose file in docs/ml35/guia-ml35-docker.md §6 passes
`robot_type:=${ROBOT_TYPE:-quadruped}`, so the argument has to exist before F6
fills it in; F1 defaults it to `diffdrive` and rejects anything else loudly
rather than silently launching the wrong plant. The selectable fallback is the
F6 gate, not this one.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

SUPPORTED_ROBOT_TYPES = ('diffdrive',)


def _check_robot_type(context, *args, **kwargs):
    """
    Fail loudly on an unimplemented robot_type instead of ignoring it.

    A launch argument that is declared and then never read is the silent kind
    of failure this project keeps paying for: `robot_type:=quadruped` would
    start the diff-drive plant and look like it worked.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    if robot_type not in SUPPORTED_ROBOT_TYPES:
        raise RuntimeError(
            f'robot_type:={robot_type!r} is not implemented in ML3.5 F1. '
            'Supported: ' + ', '.join(SUPPORTED_ROBOT_TYPES) + '. '
            'The quadruped plant arrives in F3 and becomes selectable in F6.'
        )
    return []


def generate_launch_description() -> LaunchDescription:
    # Same source and same reasoning as simulation.launch.py's default: the
    # world ships with ros-jazzy-nav2-minimal-tb4-sim and is not vendored.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_minimal_tb4_sim'), 'worlds', 'warehouse.sdf',
        ]),
        description='SDF world to load.',
    )

    # Default true: the sim container gets /dev/dri and the X socket, so the
    # GUI is the normal case. Set false for headless CI runs.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless runs.',
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='diffdrive',
        description=(
            'Which plant to simulate. F1 implements diffdrive only; '
            'quadruped arrives in F3 and becomes selectable in F6.'
        ),
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch', 'simulation.launch.py',
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        robot_type_arg,
        OpaqueFunction(function=_check_robot_type),
        simulation,
    ])
