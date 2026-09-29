"""
Nav2 for the quadruped with a live map persisted by slam_toolbox.

Runs on the x86 workstation (amd64) in learn mode and on the Aquila AM69
(arm64) in HIL. Nav2 has no graphical dependency; Gazebo and RViz stay
exclusively on the host. The composed arm64 run on the module was measured on
21/08/2026; see docs/results/ml35-hil-aquila.md.

    ros2 launch demo_bringup nav_quadruped.launch.py

## Why this file exists instead of an argument to nav.launch.py

`nav.launch.py` navigates over a STATIC MAP: it includes `bringup_launch.py`,
which loads map_server and AMCL, and passes `nav2_params.yaml`, which is the
TurtleBot 4 file. This file navigates with no map at all. The two diverge in
four points that are not a single parameter:

- the included stack is `navigation_launch.py`, with no localization or
  map_server;
- `map -> odom` is published by `odom_tf`, as an identity, not by AMCL;
- the global costmap is ROLLING and has no `static_layer`;
- the parameter file is `nav2_params_go2.yaml`.

A single launch file with conditionals to cover both cases is exactly what
CLAUDE.md forbids ("No single launch file full of conditionals"), and here the
prohibition has substance: half the errors on this path come from bringing up
the wrong combination of the four points above, and an `if` hides which
combination is active.

## WHAT MUST NOT RUN TOGETHER

`demo_routine`. Both publish `/demo/cmd_vel` -- Nav2 through the
`collision_monitor`, the routine directly. Two publishers on the same topic do
not raise an error: `twist_to_inputs` receives both messages and obeys the last
one to arrive, alternating between the avoidance and the choreography at 20 Hz.
The robot moves in spasms and nothing in the log says why. Pick one.

To move the robot under Nav2 use `patrol_commander`, which sends GOALS, not
velocities.

## The empty-map trap

Without `static_layer` and with `track_unknown_space: true`, everything outside
lidar range is unknown, and `NavfnPlanner` plans through the unknown because
`allow_unknown: true`. This is intentional -- it is what allows requesting a
goal 5 m away with no map. The consequence is that Nav2 ACCEPTS a goal outside
the 20 m rolling window and then fails when it gets near the edge. Keep goals
within ~8 m of the origin; `patrol_commander` already does this.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'config',
            'nav2_params_go2.yaml',
        ]),
        description=(
            'Parameter file. The default is the Go2 one; nav2_params.yaml is the '
            'TurtleBot 4 file and does NOT work here -- see the list of deltas in '
            'its header.'
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the run.',
    )

    clock_timeout_arg = DeclareLaunchArgument(
        'clock_timeout_s',
        default_value='120.0',
        description='How long to wait for /clock before giving up.',
    )

    # True on the reactive path, which is what this launch does. False if you
    # bring up AMCL or SLAM: both publish `map -> odom`, and two publishers on
    # the same TF edge do not raise an error -- the consumer alternates between
    # the two beliefs. See the header of odom_tf.py.
    map_identity_arg = DeclareLaunchArgument(
        'publish_map_identity',
        default_value='false',
        description=(
            'Publish map -> odom as an identity. Set false if you bring up AMCL '
            'or SLAM, or there will be two publishers on that edge.'
        ),
    )

    wait_for_clock = Node(
        package='demo_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        parameters=[{
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Deliberately NOT use_sim_time: this node's job is to run before
            # simulation time exists.
            'use_sim_time': False,
        }],
    )

    # Closes `odom -> base` and, optionally, `map -> odom`. Without it Nav2
    # does not place the scan in any costmap and fails without mentioning TF.
    odom_tf = Node(
        package='demo_bringup',
        executable='odom_tf',
        name='odom_tf',
        output='screen',
        parameters=[{
            'base_frame': 'base',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'publish_map_identity': LaunchConfiguration('publish_map_identity'),
            # Does NOT follow LaunchConfiguration('use_sim_time'), and that is
            # deliberate. See the IDLE CPU FLOOR block below cmd_vel_adapter.
            #
            # This node's hot path (`_on_odom`) COPIES the stamp of the odometry
            # message -- the header of odom_tf.py explains why, and it still
            # holds. The only call to get_clock() is in `_identity()`, which
            # stamps the map -> odom edge published on /tf_static ONCE. tf2's
            # static buffer returns the static transform for any queried
            # instant: its stamp enters no lookup.
            #
            # So nothing this node publishes changes value because of this
            # line. What changes is that it stops receiving ~870 /clock
            # messages per second only to use none of them.
            'use_sim_time': False,
        }],
    )

    # Unit boundary. Nav2 publishes SI on /demo/cmd_vel_si and this node
    # converts it to the stick scale of /demo/cmd_vel. Without it the robot
    # moves at 40% of the request and nothing complains -- see the header of
    # cmd_vel_si_to_stick.py.
    cmd_vel_adapter = Node(
        package='demo_bringup',
        executable='cmd_vel_si_to_stick',
        name='cmd_vel_si_to_stick',
        output='screen',
        # ================= IDLE CPU FLOOR OF THE MODULE =================
        #
        # This node converts Twist to Twist. It has no header, no timer, and
        # calls get_clock() nowhere -- verifiable by grep, and a test locks it.
        # With use_sim_time: true it subscribed to /clock anyway, because the
        # subscription is created by rclpy, not by the node's code.
        #
        # MEASURED ON THE AQUILA AM69 ON 25/08/2026, stack up and NO ACTIVE
        # GOAL, sampling /proc/<tid>/stat per thread inside the `nav`
        # container:
        #
        #   total idle floor           367% of 800%
        #   component_container (Nav2) 215%
        #   odom_tf                     40%   <- trivial republisher
        #   recvUC (Cyclone receive)    38%
        #   cmd_vel_si_to_stick         36%   <- this node
        #   nav_control_rel             35%   <- relay, also clockless
        #
        # Three Python republishers burning 111% of 800% -- 14% of the machine
        # -- with the robot STANDING STILL. Their useful work fits in ~1%; the
        # rest is delivery of /clock at ~870 Hz, which Gazebo publishes at that
        # rate because the gait physics step is 1 ms.
        #
        # This is NOT the /clock throttling that was tried and FAILED on 21/08
        # (see demo_simulation/clock_throttle.py): there the rate dropped for
        # EVERYONE, including MPPI, and navigation died. Here the rate does not
        # change for anyone. What changes is who subscribes -- three nodes that
        # had nothing to do with the message.
        parameters=[{'use_sim_time': False}],
    )

    # Operational channel for the cockpit. It stays on the target next to Nav2:
    # in HIL the displayed resources and temperature belong to the Aquila, and
    # the axis logs come from the real Nav2(SI) -> stick boundary, not from the
    # raw /rosout mixed with the host.
    target_monitor = Node(
        package='demo_bringup',
        executable='target_monitor',
        name='target_monitor',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # The Go2 lidar has 16 rings. The single-ring LaserScan published by the
    # bridge does not see the maze obstacles; SLAM needs the full flattened
    # cloud. Transforming to `base` also makes the height cuts relative to the
    # robot, discarding the floor without depending on the pose in the world.
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/demo/scan_cloud'),
            ('scan', '/demo/scan_slam'),
        ],
        parameters=[{
            'target_frame': 'base',
            'transform_tolerance': 0.10,
            'min_height': 0.12,
            'max_height': 1.00,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.008726646259972,
            'scan_time': 0.10,
            'range_min': 0.10,
            'range_max': 9.00,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Do not let this include inherit `params_file` from the parent launch. In
    # this file that argument is the Nav2 YAML (`nav2_params_go2.yaml`), while
    # slam_toolbox requires its own namespace and frames in `slam_params.yaml`.
    # LaunchConfiguration has scope shared across includes; without the
    # explicit pass-through below SLAM silently receives the Nav2 file and
    # falls back to the default `base_footprint`.
    slam_params = PathJoinSubstitution([
        FindPackageShare('demo_navigation'), 'config', 'slam_params.yaml',
    ])
    slam = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('demo_navigation'), 'launch', 'slam.launch.py',
            ])),
            launch_arguments={
                'params_file': slam_params,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': '/demo/scan_slam',
            }.items(),
        )],
    )

    # The behavior tree path must be ABSOLUTE, and cannot be hard-coded in the
    # YAML: it is FindPackageShare that knows the install prefix.
    #
    # The rewrite is done HERE, not in `navigation_launch.py`, because that
    # file is a vendored copy of Nav2 and its provenance depends on it staying
    # identical to upstream (see launch/nav2_vendored/README.md). We rewrite
    # beforehand and pass the result as params_file; navigation_launch does its
    # own `autostart` rewrite on top of this file, which composes fine.
    smoothed_bt = PathJoinSubstitution([
        FindPackageShare('demo_navigation'),
        'behavior_trees', 'nav_to_pose_smoothed.xml',
    ])
    exploration_bt = PathJoinSubstitution([
        FindPackageShare('demo_navigation'),
        'behavior_trees', 'nav_to_pose_exploration.xml',
    ])
    maze_explorer = Node(
        package='demo_navigation',
        executable='maze_explorer',
        name='maze_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'exploration_bt_xml': exploration_bt,
        }],
    )
    # RewrittenYaml IS ALREADY a substitution that resolves to the path of the
    # rewritten file, so it goes straight into launch_arguments. Wrapping it in
    # ParameterFile does not work here: launch_arguments accepts a string or a
    # substitution, and ParameterFile is neither.
    params_with_bt = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': smoothed_bt},
        convert_types=True,
    )

    # `navigation_launch.py`, not `bringup_launch.py`: the latter drags in
    # map_server and AMCL, which is the static-map path.
    #
    # The container that `navigation_launch.py` expects but does not create.
    # The name `nav2_container` matches the default of its `container_name`
    # argument; if either one changes, the nodes are loaded nowhere, with no
    # error.
    #
    # `component_container_isolated`, not `component_container`: each node gets
    # its own single-threaded executor inside the process. A shared executor
    # would let a long MPPI callback delay the `lifecycle_manager` heartbeat,
    # and the symptom would be the manager declaring the nodes dead in the
    # middle of navigation.
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_with_bt,
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'autostart': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    # COMPOSITION ON, and the container is created RIGHT ABOVE
    # (`nav2_container`).
    #
    # `navigation_launch.py` with composition uses LoadComposableNodes to load
    # the servers inside `/nav2_container`, but does NOT create that container
    # -- upstream it is created by `bringup_launch.py`, which is precisely the
    # file we are not including. Turning `use_composition` on without creating
    # the container is a SILENT failure: the nodes are loaded into a container
    # nobody created, NOTHING comes up and NOTHING prints an error. Measured on
    # 20/08/2026 -- the launch log ends with `wait_for_clock` and `odom_tf` and
    # nothing else, and `ros2 action list` never shows navigate_to_pose. The
    # visible symptom is "Nav2 did not activate", which does not point at this
    # parameter. If you touch the `nav2_container` block, this is how it
    # breaks.
    #
    # WHY COMPOSE, measured on the Aquila AM69 on 21/08/2026, hil mode:
    #
    #   13 separate processes      demo-nav container at 470% CPU (of 800%)
    #                              odom_tf and cmd_vel_si_to_stick, trivial
    #                              Python republishers, at ~87% of a core each
    #
    # The cost is not in the algorithm, it is in the multiplication:
    # `use_sim_time` makes EVERY node subscribe to `/clock`, which Gazebo
    # publishes at ~880 Hz because the gait physics step is 1 ms. Thirteen
    # subscribers x 880 Hz = ~11 thousand cross-process deliveries per second,
    # on a Cortex-A72.
    #
    # Composed, the servers share ONE process: one `/clock` subscription
    # instead of thirteen, and intra-process communication instead of DDS for
    # the internal topics. It attacks the same cost without touching the clock
    # the algorithms see.
    #
    # THE ALTERNATIVE THAT WAS TRIED AND REJECTED: throttling `/clock` to 100 Hz
    # (demo_simulation/clock_throttle.py). It lowered the container CPU from 470%
    # to 324%, and navigation STOPPED -- 0.0039 m/s versus 0.0251 m/s, with peak
    # `cmd_vx` falling from 0.138 to 0.003. A CPU saving that buys nothing is
    # not an optimization. See docs/results/ml35-hil-aquila.md for the A/B.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'),
            'launch', 'nav2_vendored', 'navigation_launch.py',
        ])),
        launch_arguments={
            'params_file': params_with_bt,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items(),
    )

    # std_srvs facade for the cockpit's "reset goal": cancels the goal, clears
    # the costmaps and cycles the `lifecycle_manager_navigation` brought up by
    # the include above via PAUSE/RESUME. RESET + STARTUP would be the obvious
    # choice and kills the container -- the header of nav_control_relay.py has
    # the measurement. It is a fragment shared with the static-map path, hence
    # an include -- the header of nav_control.launch.py says why duplicating
    # the Node would be worse. Deliberately after `navigation` in the list: the
    # relay tolerates the manager not existing yet (`wait_for_service`), but the
    # reading order should say who depends on whom.
    nav_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'),
            'launch', 'nav_control.launch.py',
        ])),
        # No launch_arguments: the relay no longer declares use_sim_time,
        # because it does not call the clock. Passing it here would now be a
        # launch error, and that is the intent -- see nav_control.launch.py.
    )

    # Last link before Nav2, and the one that was missing: the odom -> base
    # edge only exists once the FIRST /demo/odom crosses the container
    # boundary. Without this gate Nav2 bets on DDS discovery speed -- and on
    # 26/08 the bet lost, the local_costmap did not activate within 60 s and
    # the manager ABORTED the bringup for good. See the header of
    # wait_for_tf.py.
    wait_for_tf = Node(
        package='demo_bringup',
        executable='wait_for_tf',
        name='wait_for_tf',
        output='screen',
        parameters=[{
            'parent_frame': 'odom',
            'child_frame': 'base',
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Queries with Time() (latest common instant), which does not use
            # the node's clock -- so it does not need to subscribe to /clock.
            'use_sim_time': False,
        }],
    )

    # Fail LOUD. Launching Nav2 anyway reproduces exactly the defect this gate
    # exists to prevent, and the symptom would again be "goal rejected" with
    # nobody mentioning TF or the clock.
    def _gate(following, what):
        def _on_exit(event, context):
            if event.returncode == 0:
                return following
            return [
                LogInfo(msg=f'[nav_quadruped] {what} failed (code '
                            f'{event.returncode}). Nav2 will NOT be started.'),
                Shutdown(reason=f'{what} not satisfied'),
            ]
        return _on_exit

    # Startup chain, each link conditioned on the previous one FINISHING and
    # not on elapsed time -- same discipline as quadruped.launch.py, which this
    # file did not follow. Before 26/08/2026 the five nodes below were emitted
    # TOGETHER, and `wait_for_clock` conditioned nothing despite what its
    # docstring promises. It was a race, and it was lost on the AM69.
    #
    #   wait_for_clock  ->  wait_for_tf  ->  Nav2
    #
    # `odom_tf` and `cmd_vel_adapter` come up immediately, on purpose: it is
    # odom_tf that PRODUCES the edge wait_for_tf waits for.
    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        clock_timeout_arg,
        map_identity_arg,
        odom_tf,
        cmd_vel_adapter,
        target_monitor,
        wait_for_clock,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=wait_for_clock,
            on_exit=_gate([wait_for_tf], 'wait_for_clock'),
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=wait_for_tf,
            on_exit=_gate([
                cloud_to_scan, slam, nav2_container, navigation, nav_control,
                maze_explorer,
            ],
                          'wait_for_tf'),
        )),
    ])
