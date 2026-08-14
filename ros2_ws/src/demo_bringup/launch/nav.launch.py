"""
Navigation container entrypoint — Nav2 over a static map.

Runs on: x86 host in learn mode, Aquila AM69 (arm64) in hil mode. Identical
image and identical launch either way; the only thing that changes is which
compose file starts it. This is the half of the demo that migrates to the
module.

    ros2 launch demo_bringup nav.launch.py use_sim_time:=true

ML3.5 F1 decomposition of learn.launch.py. Wraps
demo_navigation/navigation.launch.py and adds nothing to it.

NO TIMER HERE — and that is the substantive change from learn.launch.py.

learn.launch.py starts Nav2 at t=25s because everything shared one process
tree: the timer was the only ordering mechanism available. Across a container
boundary that timer measures the wrong thing. It counts from when *this*
container started, which has no fixed relationship to when the `sim` container
finished loading the warehouse world. `docker compose up` starts both at once,
so t=25s here is a bet on sim being ready, not a guarantee — and on a cold image
pull or a slower host that bet loses.

What replaces it is `wait_for_clock`, a real precondition rather than a proxy
for one. Nav2's failure mode when started early is the one this guards: its
lifecycle nodes come up before /clock is publishing and stall waiting for
transforms that have no timestamps yet, with no error message that names the
cause. Waiting on the actual clock topic makes the ordering explicit and
removes the dependency on wall-clock guesswork.

RViz is deliberately absent — OGRE 2, x86 only (CLAUDE.md rule 1). It is the
viz container's job.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'maps', 'warehouse.yaml',
        ]),
        description='Static map for AMCL and the global costmap.',
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'config', 'nav2_params.yaml',
        ]),
        description='Nav2 parameter file.',
    )

    # True whenever Gazebo drives the demo — which includes hil mode, where the
    # module follows the host's /clock over DDS. A module running on wall time
    # while the host runs on sim time produces TF extrapolation errors and Nav2
    # refusing goals, with nothing in the logs that points at the clock.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    # Bounded on purpose. An unbounded wait turns "sim never started" into a
    # container that sits silent forever, which reads as a hang rather than a
    # failure. 120 s is generous for warehouse world load plus an arm64 image
    # coming up on the module; when it expires the node exits and compose
    # surfaces it.
    clock_timeout_arg = DeclareLaunchArgument(
        'clock_timeout_s',
        default_value='120.0',
        description=(
            'How long to wait for /clock before giving up. Replaces the '
            'wall-clock timer that learn.launch.py used inside one process.'
        ),
    )

    wait_for_clock = Node(
        package='demo_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        parameters=[{
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Deliberately NOT use_sim_time: this node's whole job is to run
            # before sim time exists. A node waiting for /clock while itself
            # waiting on /clock never completes.
            'use_sim_time': False,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'launch', 'navigation.launch.py',
        ])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        clock_timeout_arg,
        wait_for_clock,
        navigation,
    ])
