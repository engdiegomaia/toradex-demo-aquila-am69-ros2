"""
Gazebo Harmonic + Unitree Go2 + ros2_control — the quadruped plant (ML3.5 F3).

Runs on: x86 workstation ONLY, in the `sim` container. Gazebo is OGRE 2 /
desktop OpenGL and must never be placed on the Aquila AM69 (CLAUDE.md rule 1).

    ros2 launch demo_simulation quadruped.launch.py

This is the legged counterpart of simulation.launch.py. Same responsibility —
own the simulator, the robot instance and the ROS<->gz bridge, nothing else —
with the plant swapped. Nav2 and perception compose on top through demo_bringup
and must not be able to tell the difference (topic contract, CLAUDE.md).

Derived from unitree_guide_controller/launch/gazebo.launch.py upstream, with
three deliberate differences, all of them load-bearing:

1. NO RViz2. Upstream starts an `rviz_ocs2` node inside this same launch. That
   is OGRE 2 inside the sim container; in this project RViz2 lives in the `viz`
   container on the host (rule 1). The F2 spike already proved the plant comes
   up without it.
2. Controllers are chained on spawn completion, not on a wall-clock timer.
   Same reasoning as wait_for_clock in F1: a timer measures the wrong thing.
3. The gait FSM is driven by the contract bridge (twist_to_inputs), not by a
   keyboard node. Upstream expects a human at a terminal.

WHY THERE IS NO DIFF-DRIVE-STYLE PLUGIN HERE
The diff-drive plant gets odometry, joint states and cmd_vel from Gazebo system
plugins declared in the URDF. A quadruped has no equivalent native plugin: the
joints are driven by a ros2_control controller (unitree_guide_controller) whose
hardware interface (gz_quadruped_hardware) is loaded INSIDE the gz sim process.
That is why sim is one container and cannot be split (guia-ml35-docker.md §1),
and it is why ML3.5 reverses the ML2 decision against gz_ros2_control.
"""

from demo_simulation.scenarios import missing_models, spawn_pose
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


class ScenarioPose(Substitution):
    """
    One field of the spawn pose: the argument, otherwise the scenario table.

    It is a Substitution and not an OpaqueFunction because the destination is a
    Node's `arguments`, which accepts substitutions and does NOT accept actions
    -- an OpaqueFunction there is accepted at assembly and fails at execution,
    with an error that talks about types and not about the argument.

    Resolving at execution time is also what lets `world:=` and `yaw:=` arrive
    from SIM_ARGS: neither exists when the description is assembled.
    """

    def __init__(self, field: str) -> None:
        super().__init__()
        self._field = field

    def perform(self, context) -> str:
        explicit = LaunchConfiguration(self._field).perform(context)
        if explicit:
            return explicit
        world = LaunchConfiguration('world').perform(context)
        return str(spawn_pose(world)[self._field])


def _check_external_models(context, *args, **kwargs) -> list:
    """
    Fail LOUDLY when the world asks for a mesh that is not mounted.

    Without this check the failure mode is the worst possible: Gazebo loads the
    world, the maze model ends up with no visual and no collision, and the
    result is an empty plane. The lidar sees no wall, Nav2 plans a straight
    line, and the goal ends SUCCEEDED -- with numbers BETTER than the real ones.
    The run passes and nobody discovers the maze was not there.

    It is checked here and not in compose because the launch is what knows which
    world is involved, and because the default path of
    `MAZE_MODELS:-./models-extra` makes Docker CREATE an empty directory instead
    of refusing the mount.
    """
    world = LaunchConfiguration('world').perform(context)
    missing = missing_models(world)
    if missing:
        raise RuntimeError(
            'world %s loads model(s) external to the repository that are not '
            'mounted: %s.\n'
            'The maze mesh has a TODO licence upstream '
            '(github.com/cafemesa/ros_maze_worlds) and therefore is NOT '
            'vendored -- the same blocker that made the project swap the A1 for '
            'the Go2.\n'
            'Point MAZE_MODELS at the models/ directory of that repository:\n'
            '  MAZE_MODELS=/path/to/ros_maze_worlds/models '
            'docker compose -f compose.host.yml up -d sim\n'
            'Without it Gazebo starts an EMPTY plane without reporting anything, '
            'and any navigation measurement taken on it is fiction.'
            % (world, ', '.join(missing))
        )
    return []


def generate_launch_description() -> LaunchDescription:
    # The quadruped's OFFICIAL SCENARIO is the maze, not the empty world.
    #
    # It was `quadruped_empty.sdf` since the F2 spike, when the simulation image
    # did not even have Nav2: the empty world was the only one that came up. It
    # stayed as the default by inertia, and the side effect is that the shortest
    # command there is (`ros2 launch ... quadruped.launch.py`) brought up an
    # infinite floor with nothing to navigate -- so EVERY navigation measurement
    # required passing `world:=` by hand, and a measurement taken without
    # passing it measured navigation in open field.
    #
    # The small worlds remain one argument away:
    #   ros2 launch demo_simulation quadruped.launch.py \
    #     world:=$(ros2 pkg prefix demo_simulation)/share/demo_simulation/\
    #       worlds/quadruped_empty.sdf
    #
    # The spawn pose and the camera framing follow the world through
    # `scenarios.py`; there is no way to switch worlds and forget the other nine
    # numbers. See the header of that file.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'worlds',
            'quadruped_maze11.sdf',
        ]),
        description='Absolute path to the SDF world to load. The default is the '
                    'official scenario (the maze).',
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='demo_robot',
        description='Model name in Gazebo. Kept identical to the diff-drive '
                    'plant so anything keyed on the model name still matches.',
    )

    # Gait tuning, as a controller param file rather than compiled-in literals.
    # Overridable so a sweep is `gait_params:=/tmp/try.yaml`, not a rebuild of
    # the sim image — which is what every gait experiment before ML3.5 phase A
    # cost. See the file's own header for why the spawner is the injection point
    # instead of go2_description/config/gazebo.yaml.
    gait_params_arg = DeclareLaunchArgument(
        'gait_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'config', 'gait_go2.yaml',
        ]),
        description='Controller param file with the trot tuning, loaded into '
                    'unitree_guide_controller before it is configured.',
    )

    # Decimation of the joint_state_broadcaster, by the same mechanism and for
    # the same reason as gait_params: the package that declares the controller
    # is vendored. The file explains the measurement that motivated the 50 Hz;
    # in short, the broadcaster inherited the controller_manager's 1000 Hz and
    # filled /tf at 1090 Hz for eleven subscribers, of which the navigation ones
    # are on the other side of the Ethernet. Empty turns decimation off and goes
    # back to the inherited behaviour.
    jsb_params_arg = DeclareLaunchArgument(
        'jsb_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'config',
            'joint_state_broadcaster.yaml',
        ]),
        description='Param file with the joint_state_broadcaster rate. Affects '
                    'ONLY the broadcaster: the control loop, the gait and the '
                    'physics stay at 1000/200/1000 Hz.',
    )

    # Empty = taken from scenarios.py for the world. A numeric default here is
    # indistinguishable from an operator choice, and that is how the robot came
    # to spawn facing the wall when the scenario became the maze: yaw 0 is
    # correct in the warehouse and wrong in maze11, and nothing flags the
    # difference -- the robot just spends its first seconds turning inside a
    # 1.20 m corridor.
    x_arg = DeclareLaunchArgument('x', default_value='')
    y_arg = DeclareLaunchArgument('y', default_value='')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='')

    # Spawn height. NOT cosmetic and NOT the same as the diff-drive's -z 0.1.
    # A quadruped spawned at floor level starts with its legs already
    # interpenetrating the ground plane; the solver pushes it out violently and
    # the robot is on its back before the controller ever loads. Upstream's own
    # README uses height:=0.43 for the A1 for this reason. The F2 spike used
    # 0.5 for the Go2 and the robot settled to a 0.353 m standing height, so
    # 0.5 leaves a small drop with no interpenetration.
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Spawn height in metres. A quadruped spawned at floor '
                    'level interpenetrates the ground and tips over before '
                    'the controller loads.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless runs.',
    )

    scene_cameras_arg = DeclareLaunchArgument(
        'scene_cameras',
        default_value='true',
        description='Spawn the cockpit scene cameras. Set false for navigation '
                    'measurements without the extra render cost.',
    )

    # GAZEBO:=true selects xacro/gazebo.xacro, which declares the
    # <ros2_control> block with the gz_quadruped_hardware/GazeboSimSystem
    # plugin and includes the foot force sensors. Without it the xacro expands
    # to the real-hardware interface (xacro/ros2_control.xacro) and the plant
    # comes up with no simulated actuators — a robot that loads and never
    # moves, with no error naming the cause.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('demo_simulation'), 'urdf',
                'go2_sim.urdf.xacro',
            ]),
            ' GAZEBO:=true',
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 3'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -s -v 3'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # Publishes the fixed TF edges and animates the 12 leg joints from
    # /joint_states. Unlike the diff-drive plant, nothing here publishes
    # odom -> base_link yet: legged odometry is F5 work. Nav2 is not started by
    # this launch, so the missing edge does not break anything in F3.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            # Upstream sets this. The quadruped's joint states come from
            # ros2_control rather than a Gazebo plugin, and the timestamps do
            # not line up with /clock closely enough for the default check.
            'ignore_timestamp': True,
        }],
    )

    # NO TimerAction HERE, and this is the single most expensive thing to get
    # wrong in this file.
    #
    # The diff-drive plant delays its spawn by 12 s so that `create` does not
    # call Gazebo's "list world names" service before it exists. Copying that
    # delay here produces a robot that is spawned at height and then falls,
    # unattended, for the whole gap between the spawn and the controllers
    # activating. Measured, with the 12 s timer in place:
    #
    #   spawn at z=0.49999 -> z=0.0677 less than 1 s later -> controllers
    #   activate ~3 s after that. Final state: collapsed on the floor with legs
    #   splayed, three controllers reporting `active`, zero errors in the log,
    #   and the gait FSM cheerfully walking passive -> trotting on top of a
    #   robot that is already down.
    #
    # Nothing in that failure names its cause. It is visible only by reading
    # the pose out of gz directly, which is why the F3 gate is measured that
    # way and never from log output.
    #
    # `create` is safe to start immediately because it retries the world-names
    # service rather than failing, and it blocks on the robot_description topic
    # anyway. The cost of starting it early is a few retry lines in the log;
    # the cost of starting it late is the silent collapse above. Upstream's
    # gazebo.launch.py also spawns with no delay at all.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_demo_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-allow_renaming', 'true',
            '-z', LaunchConfiguration('height'),
            '-x', ScenarioPose('x'),
            '-y', ScenarioPose('y'),
            '-Y', ScenarioPose('yaw'),
        ],
    )

    # The controller_manager only exists once gz_quadruped_hardware has loaded
    # inside the gz sim process, which happens when the model is spawned.
    # Chained on spawn exit rather than on a timer: the timer would be
    # measuring container uptime, which is the F1 lesson (wait_for_clock).
    #
    # `--param-file` here is what decimates the broadcaster to 50 Hz. Same
    # rationale as the gait controller's `--param-file` below: the `update_rate`
    # parameter lives on the controller node inside the gz process, and the
    # spawner is what applies it before load_controller. Setting it afterwards
    # would be accepted and never read.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', LaunchConfiguration('jsb_params')],
        output='screen',
    )

    imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # The gait controller. Classic PD (unitree_guide), no RL policy and no .pt
    # weights — which is why the RL configs were dropped when go2_description
    # was vendored (see its README).
    #
    # --param-file, not `parameters=[...]`: the controller's parameters live on
    # the controller_manager node inside the gz process, not on this spawner
    # node. The spawner applies the file before load_controller, which is when
    # the controller's on_init() reads it.
    unitree_guide_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['unitree_guide_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', LaunchConfiguration('gait_params')],
        output='screen',
    )

    # F4 contract boundary for clock, odometry, lidar, camera and IMU. cmd_vel
    # stays ROS-native: twist_to_inputs consumes /demo/cmd_vel directly and
    # translates it to the gait controller's private /control_input message.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare('demo_simulation'),
                'config', 'bridge_quadruped.yaml',
            ]),
            'use_sim_time': True,
        }],
    )

    # Translates /demo/cmd_vel into the controller's Inputs message and walks
    # the gait FSM up to TROTTING. See the node's own docstring for why this is
    # not just a message cast.
    twist_to_inputs = Node(
        package='demo_simulation',
        executable='twist_to_inputs',
        name='twist_to_inputs',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Ground-truth is an acceptance oracle only.  The explorer running on the
    # Aquila never subscribes to this topic and receives no exit coordinates.
    maze_escape_validator = Node(
        package='demo_simulation',
        executable='maze_escape_validator',
        name='maze_escape_validator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world': LaunchConfiguration('world'),
        }],
    )

    # External views of the web cockpit (blue panel): two static cameras
    # spawned into the world, isometric and top-down. Fragment SHARED with the
    # diff-drive plant, so the panel does not go dark when ROBOT_TYPE changes.
    # All the framing arithmetic and the reason they are spawned instead of
    # written into worlds/*.sdf are in scene_cameras.launch.py.
    scene_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch',
            'scene_cameras.launch.py',
        ])),
        launch_arguments={
            'scene_cameras': LaunchConfiguration('scene_cameras'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Gazebo service bridge: the cockpit's play/pause/reset, and the set_pose
    # that scene_view_controller uses to move the cameras. Shared fragment, for
    # the same reason as the previous one.
    sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch',
            'sim_control.launch.py',
        ])),
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        gait_params_arg,
        jsb_params_arg,
        x_arg,
        y_arg,
        yaw_arg,
        height_arg,
        gui_arg,
        scene_cameras_arg,
        # BEFORE any node: if the external mesh is not there, there is no valid
        # run to make, and the silent failure mode is expensive (see the
        # docstring).
        OpaqueFunction(function=_check_external_models),
        bridge,
        gz_sim,
        gz_sim_headless,
        scene_cameras,
        sim_control,
        robot_state_publisher,
        spawn_robot,
        twist_to_inputs,
        maze_escape_validator,
        # Startup chain, each link gated on the previous one actually finishing
        # rather than on elapsed time: spawn -> broadcasters -> gait controller.
        # The gait FSM waits on top of that, inside twist_to_inputs.
        # Broadcasters first, gait controller last: unitree_guide_controller
        # reads joint states and IMU on activation, and activating it before
        # its inputs exist leaves it in an inactive state that looks like a
        # silent hang rather than an error.
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster, imu_sensor_broadcaster],
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[unitree_guide_controller],
        )),
    ])
