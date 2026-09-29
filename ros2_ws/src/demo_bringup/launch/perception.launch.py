"""
Perception container entrypoint — detection stub + costmap adapter.

Runs on: x86 host in learn mode, Aquila AM69 (arm64) in hil mode. Identical
image and identical launch either way; this node pair is CPU-only.

    ros2 launch demo_bringup perception.launch.py use_sim_time:=true

ML3.5 F1 decomposition of learn.launch.py. Wraps
demo_perception/perception.launch.py and adds nothing to it — demo_perception
itself is not touched by any ML3.5 phase (rule 6).

Like nav.launch.py, this gates on /clock advancing instead of carrying over
learn.launch.py's t=20s timer. The reason is the same: across a container
boundary the timer counts from the wrong start. The consequence here is milder
than Nav2's — the stub subscribes with sensor QoS and simply receives nothing
until the bridge is up — but starting with use_sim_time:=true against a clock
that does not yet exist gives every published detection a zero timestamp, and
the costmap silently discards those.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    assumed_range_arg = DeclareLaunchArgument(
        'assumed_range_m',
        default_value='2.0',
        description=(
            'Ground-plane distance assumed for every detection. A 2D box has '
            'no depth; see detections_to_cloud.py.'
        ),
    )

    clock_timeout_arg = DeclareLaunchArgument(
        'clock_timeout_s',
        default_value='120.0',
        description='How long to wait for /clock before giving up.',
    )

    detector_backend_arg = DeclareLaunchArgument(
        'detector_backend',
        default_value='fiducial',
        description=(
            'Maze-exit pose source: fiducial (AprilTag, default) or magenta '
            '(the panel-colour fallback). Never both at once.'
        ),
    )

    wait_for_clock = Node(
        package='demo_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        parameters=[{
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Not use_sim_time — this node runs before sim time exists.
            'use_sim_time': False,
        }],
    )

    # THE IMAGE ARRIVES COMPRESSED AND IS OPENED HERE, ON THE CONSUMER'S SIDE.
    #
    # `sim.launch.py` publishes /demo/camera/image_raw/compressed in addition to
    # the RAW. This node subscribes to the compressed one and returns
    # sensor_msgs/Image on /demo/perception/image_in, which is local to this
    # machine. The SetRemap below rewires the detection_stub to it.
    #
    # WHY THE NAME CHANGES, AND WHY THAT IS NOT OPTIONAL: publishing the
    # decompressed image back on /demo/camera/image_raw would put TWO publishers
    # on the same topic of the same domain -- the bridge's on the host and this
    # one. That raises no error: the subscriber obeys the last message that
    # arrived, and the result is an image alternating between two sources with
    # nothing in the log. The project has already paid for this class of failure
    # on /demo/cmd_vel (see the docstring of nav_trial.py).
    #
    # THE CONTRACT DOES NOT CHANGE. demo_perception keeps consuming
    # sensor_msgs/Image and still does not know where the frame came from
    # (CLAUDE.md rule 6) -- it is not even touched: the rewiring is by launch
    # remap, not by editing the node. When TIDL replaces the stub, it receives
    # the same type in the same place.
    #
    # COST THIS ADDS, STATED EXPLICITLY: decoding JPEG spends CPU on the module,
    # which is precisely the scarce resource. The bet is that decoding 640x480
    # costs less than reassembling 75 Mbit/s of fragmented RAW, and it is
    # MEASURED, not assumed. If decoding eats the savings, the next step is for
    # the stub to subscribe to CompressedImage directly and take the dimensions
    # from /demo/camera/camera_info -- it only uses header, width and height,
    # never the pixels (detection_stub.py:91,100,119).
    camera_decompressor = Node(
        package='image_transport',
        executable='republish',
        name='camera_decompressor',
        # Parameters, not positional -- see the trap documented in
        # sim.launch.py, which cost a build cycle. Here an empty out_transport
        # does not create a loop (the output topic has a different name), but
        # does something harder to find: the decompressor publishes nothing, the
        # detection_stub is left with no image at all, and perception dies
        # silently.
        parameters=[{
            'in_transport': 'compressed',
            'out_transport': 'raw',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # `in` carries the transport suffix and `out` does not -- see the trap
        # documented in sim.launch.py. Here the roles are reversed relative to
        # the compressor, because here the compressed image is the INPUT.
        remappings=[
            ('in/compressed', '/demo/camera/image_raw/compressed'),
            ('out', '/demo/perception/image_in'),
        ],
        output='screen',
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_perception'), 'launch', 'perception.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'assumed_range_m': LaunchConfiguration('assumed_range_m'),
            'detector_backend': LaunchConfiguration('detector_backend'),
        }.items(),
    )

    # SetRemap applies to the group's scope, and ONLY the include goes into it:
    # camera_decompressor is deliberately left out, otherwise its own `in`
    # would be rewritten and it would end up subscribing to itself.
    perception_group = GroupAction([
        SetRemap(src='/demo/camera/image_raw', dst='/demo/perception/image_in'),
        perception,
    ])

    return LaunchDescription([
        use_sim_time_arg,
        assumed_range_arg,
        clock_timeout_arg,
        detector_backend_arg,
        wait_for_clock,
        camera_decompressor,
        perception_group,
    ])
