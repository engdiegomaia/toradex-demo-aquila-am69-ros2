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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_perception'), 'launch', 'perception.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'assumed_range_m': LaunchConfiguration('assumed_range_m'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        assumed_range_arg,
        clock_timeout_arg,
        wait_for_clock,
        perception,
    ])
