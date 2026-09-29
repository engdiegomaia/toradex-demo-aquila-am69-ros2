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

`robot_type` selects the plant: `diffdrive` (ML1-ML3, Gazebo's native
gz-sim-diff-drive-system) or `quadruped` (ML3.5 F3, Unitree Go2 on
ros2_control). Each maps to exactly one launch file in demo_simulation; an
unknown value fails loudly rather than silently starting the wrong plant.

The ML3.5 default is `quadruped`; `diffdrive` remains the tested fallback. The
same selector is consumed by `nav_select.launch.py`, so plant and navigation
cannot silently select different robots.
"""

from demo_bringup.robot_selection import (
    launch_file,
    official_world,
    ROBOT_LAUNCH_FILES,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Each entry maps to exactly one launch file in demo_simulation. Adding a plant
# means adding a launch file, not adding a conditional to an existing one
# (CLAUDE.md: no single launch file full of conditionals).
SUPPORTED_ROBOTS = tuple(sorted(ROBOT_LAUNCH_FILES))


def _check_robot_type(context, *args, **kwargs):
    """
    Fail loudly on an unimplemented robot_type instead of ignoring it.

    A launch argument that is declared and then never read is the silent kind
    of failure this project keeps paying for: `robot_type:=quadruped` would
    start the diff-drive plant and look like it worked.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    try:
        launch_file(robot_type, 'plant')
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    return []


def _launch_plant(context, *args, **kwargs):
    """
    Include the launch file for the selected plant.

    Resolved here rather than with a substitution so that an unknown
    robot_type fails in _check_robot_type with a message that names the
    problem, instead of failing later as a missing-file error that does not.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)

    # The world is resolved HERE and passed explicitly, not left to the plant's
    # default. The reason is launch scope: a `world` declared in this file enters
    # the context, and a DeclareLaunchArgument in the included description does
    # NOT override an inherited value -- the parent's value wins. If this file
    # declared an empty `world` and trusted the plant's default, the
    # scene_cameras.launch.py fragment would inherit the empty value, fall back
    # to the generic framing, and the cockpit cameras would point at the origin
    # in a world whose usable area is at (-4.855, 4.855). No error, no log: blue
    # panel staring at the floor.
    world = LaunchConfiguration('world').perform(context)
    if not world:
        package, *parts = official_world(robot_type)
        world = PathJoinSubstitution(
            [FindPackageShare(package)] + parts).perform(context)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'),
            'launch',
            launch_file(robot_type, 'plant'),
        ])),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'scene_cameras': LaunchConfiguration('scene_cameras'),
        }.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    # EMPTY = the official scenario of the selected robot_type
    # (robot_selection.py): maze for the quadruped, warehouse for the diff-drive.
    #
    # The warehouse used to be hard-coded here for both. The quadruped became the
    # default robot in F6 and its Nav2 tuning was measured in the maze, so the
    # hard-coded default left the shortest path running the official robot in the
    # wrong world -- with the 0.85 inflation designed for a 1.20 m corridor
    # applied in an open warehouse, and the scene cameras framing somewhere else.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='SDF world to load. Empty = official scenario of the robot '
                    '(maze for the quadruped, warehouse for the diff-drive).',
    )

    # Default true: the sim container gets /dev/dri and the X socket, so the
    # GUI is the normal case. Set false for headless CI runs.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless runs.',
    )

    scene_cameras_arg = DeclareLaunchArgument(
        'scene_cameras',
        default_value='true',
        description='Spawn cockpit scene cameras when the selected plant '
                    'supports them.',
    )

    # F5 validated the quadruped plant with Nav2 on the host and the Aquila.
    # F6 promotes it while preserving diffdrive as an explicit fallback.
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='quadruped',
        description=(
            'Which plant to simulate: ' + ' | '.join(SUPPORTED_ROBOTS) + '.'
        ),
    )

    # THE SAME FRAME, COMPRESSED, FOR WHOEVER IS ON THE OTHER END OF THE WIRE.
    #
    # Measured on 25/08/2026 on the wired HIL: the RAW image is 640x480 rgb8 at
    # 10 Hz, 0.92 MB per message, 9.3 MB/s -- about 75 Mbit/s crossing the
    # network to the Aquila. The 1 Gbit/s link covers the bandwidth; what does
    # NOT add up is the CPU cost of reassembling that stream on the module. With
    # Nav2 (600-727% of 800%) and perception (~187%) up, the AM69 saturates,
    # sensor freshness collapses, the collision_monitor rejects the LiDAR cloud
    # at 1.0-1.2 s of lag, and the mean speed drops 2.8x (0.0429 -> 0.0155 m/s).
    # Evidence: docs/results/ml35-f5-ethernet0-repeticao.md.
    #
    # THIS NODE DOES NOT REPLACE THE RAW TOPIC. The CLAUDE.md contract remains
    # /demo/camera/image_raw as sensor_msgs/Image, and it is still published
    # here, which is where RViz and the cockpit's web_video_server consume it --
    # for free, because it is the same machine. What this node adds is the
    # transported /demo/camera/image_raw/compressed, and whoever pays for the
    # wire subscribes to that.
    #
    # Runs in BOTH modes, on purpose. In learn it costs a bit of host CPU with
    # no benefit, and that is the right trade: a stack that only compresses in
    # hil would be different code per mode, which is exactly what CLAUDE.md
    # forbids. The measured path is the executed path.
    #
    # `republish` comes from upstream image_transport: nothing here is
    # hand-written. The plugin comes from ros-${ROS_DISTRO}-compressed-image-
    # transport, installed in docker/sim/Dockerfile. Without the plugin this
    # node comes up and publishes NOTHING, with no error anyone reads -- check
    # with `ros2 run image_transport list_transports`, which must declare
    # image_transport/compressed in addition to /raw.
    # in_transport/out_transport ARE PARAMETERS, NOT POSITIONAL ARGUMENTS.
    #
    # THIS COST A BUILD CYCLE ON 25/08/2026 and fails in the worst way. Written
    # as arguments=['raw', 'compressed'], Jazzy consumes the first as
    # in_transport and leaves out_transport EMPTY. The node comes up, reports no
    # error, and the log literally says:
    #
    #     The 'out_transport' parameter is set to:
    #
    # with the value blank after the colon -- which nobody reads as a failure.
    #
    # THE DAMAGE ON THE HOST SIDE is worse than "does not publish": with an empty
    # out_transport the republish becomes raw->raw over the SAME input topic, and
    # the node appears as both publisher AND subscriber of /demo/camera/image_raw
    # at once. Measured: the contract camera went from 10 Hz to 118 Hz through
    # feedback, with `Publisher count: 2`, and the module's detection_stub began
    # receiving that inflated stream over the wire. Check with:
    #
    #     ros2 topic info -v /demo/camera/image_raw   # Publisher count must be 1
    camera_compressor = Node(
        package='image_transport',
        executable='republish',
        name='camera_compressor',
        parameters=[{'in_transport': 'raw', 'out_transport': 'compressed'}],
        # THE REMAP MUST CARRY THE TRANSPORT SUFFIX.
        #
        # SECOND TRAP OF THE SAME NODE, 25/08/2026. image_transport creates the
        # topic already with the suffix -- `out/compressed`, not `out` -- so a
        # remap rule for `out` does NOT match and is simply ignored. The `in`
        # side is deceptive here, because `raw` has no suffix and the plain remap
        # matches.
        #
        # Symptom: the node comes up, logs the two correct transports,
        # subscribes to the camera (Subscription count 1 on the contract topic)
        # and publishes on `/out/compressed` -- a root-namespace topic nobody
        # looks for. `ros2 topic list | grep camera` shows nothing wrong; only
        # `ros2 node info /camera_compressor` gives it away, in the Publishers
        # list.
        remappings=[
            ('in', '/demo/camera/image_raw'),
            ('out/compressed', '/demo/camera/image_raw/compressed'),
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        scene_cameras_arg,
        robot_type_arg,
        OpaqueFunction(function=_check_robot_type),
        OpaqueFunction(function=_launch_plant),
        camera_compressor,
    ])
