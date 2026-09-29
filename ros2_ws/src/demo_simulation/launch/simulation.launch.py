"""
Gazebo Harmonic + robot spawn + ros_gz_bridge — the simulation half of ML3.

Runs on: x86 workstation ONLY. Gazebo is an OGRE 2 / desktop-OpenGL application
and must never be placed on the Aquila AM69 (CLAUDE.md rule 1). In `target` mode
this exact launch still runs on the host; only the navigation stack moves to the
module.

    ros2 launch demo_simulation simulation.launch.py

This file owns the simulator, the robot instance and the ROS<->gz bridge, and
nothing else. Nav2 and perception are composed on top by demo_bringup.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Resolved from nav2_minimal_tb4_sim, NOT from this package's worlds/ dir.
    # The world is not vendored (see README): it ships with
    # ros-jazzy-nav2-minimal-tb4-sim, is Apache-2.0 and ~40 MB of meshes that
    # have no business in this repo. Pointing the default at a local worlds/
    # file that is not committed means a clean clone fails with a Gazebo error
    # that does not name the missing file.
    #
    # To use your own world, drop it anywhere and pass world:=/abs/path.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_minimal_tb4_sim'), 'worlds', 'warehouse.sdf',
        ]),
        description='Absolute path to the SDF world to load.',
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='demo_robot',
        description=(
            'Model name in Gazebo. Note that the DiffDrive plugin does NOT '
            'scope its gz topics under /model/<robot_name>/ — its <topic>, '
            '<odom_topic> and <tf_topic> are literal. See the note in '
            'config/bridge_warehouse.yaml before changing this.'
        ),
    )

    # Spawn pose. The warehouse world has open floor near the origin; override
    # these when starting the robot inside a different aisle.
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless CI runs.',
    )

    # Forwarded to the xacro. Collision, inertia, TF and odometry are identical
    # either way — only rendering changes. See demo_description/_visuals.xacro.
    use_meshes_arg = DeclareLaunchArgument(
        'use_meshes',
        default_value='true',
        description=(
            'Render the robot from nav2_minimal_tb4_description meshes. '
            'Set false to fall back to primitives.'
        ),
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_description'), 'urdf', 'demo_robot.urdf.xacro',
        ]),
        description='Absolute path to the robot xacro to spawn.',
    )

    # The expanded URDF is piped through weld_fixed_joints.py before it reaches
    # either robot_state_publisher or Gazebo.
    #
    # THIS PIPE IS LOAD-BEARING — do not simplify it back to a bare `xacro` call.
    # Upstream tags 21 fixed joints with <preserveFixedJoint>, which makes Gazebo
    # spawn the robot as 13 separate physics bodies instead of one rigid body. The
    # shell, tower standoffs, sensor plate, lidar and camera then drift apart under
    # gravity and the robot visibly comes to pieces. See the long note in
    # demo_description/urdf/demo_robot.urdf.xacro.
    #
    # The script writes its own status line to stderr and exits non-zero if any
    # tag survives, so a silent partial weld is not possible.
    #
    # It is invoked INSTEAD of xacro, not piped into it: Command runs its string
    # through shlex.split rather than a shell, so a `|` here would reach xacro as
    # a literal argument. The script takes the xacro path plus its args, runs
    # xacro itself, and filters the output — one process, no shell needed.
    #
    # value_type=str is required — without it the expanded URDF is parsed as
    # YAML and robot_state_publisher rejects it. Same reasoning as in
    # demo_description/launch/view_robot.launch.py.
    weld_script = PathJoinSubstitution([
        FindPackageShare('demo_description'), 'scripts', 'weld_fixed_joints.py',
    ])

    robot_description = ParameterValue(
        Command([
            'python3 ', weld_script, ' ', LaunchConfiguration('model'),
            ' robot_name:=', LaunchConfiguration('robot_name'),
            ' use_meshes:=', LaunchConfiguration('use_meshes'),
        ]),
        value_type=str,
    )

    # `-r` starts the world unpaused; without it every downstream node blocks
    # forever waiting for /clock to advance.
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

    # Publishes the fixed TF edges (base_footprint -> base_link -> sensors) and
    # animates the wheel joints from /joint_states. It does NOT publish
    # odom -> base_footprint; the DiffDrive plugin owns that edge.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawns from the /robot_description topic rather than a file, so the model
    # in Gazebo is byte-identical to the one robot_state_publisher is using.
    #
    # DELAYED ON PURPOSE — do not remove the TimerAction.
    # `create` first calls Gazebo's "list of world names" service. Started at
    # t=0 that service does not exist yet, and the client retries every 5 s
    # forever instead of failing. The warehouse world takes ~10 s to load its
    # 50+ meshes, so a t=0 spawner never completes its handshake.
    #
    # The robot still appears in the world (the async create service accepts
    # it), which makes this look like it worked: sensors publish, `gz model
    # --list` shows demo_robot. But the DiffDrive and JointStatePublisher
    # system plugins never initialize, so /model/demo_robot/odom and
    # .../joint_state advertise and then stay permanently silent — no
    # odometry, no wheel TF, and Nav2 with nothing to localize against.
    spawn_robot = TimerAction(
        period=12.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_demo_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', LaunchConfiguration('robot_name'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', '0.1',
                '-Y', LaunchConfiguration('yaw'),
            ],
        )],
    )

    # Started after the spawn for the same reason: the model-scoped gz topics
    # (/model/demo_robot/...) only exist once the robot is in the world.
    bridge = TimerAction(
        period=15.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': PathJoinSubstitution([
                    FindPackageShare('demo_simulation'),
                    'config', 'bridge_warehouse.yaml',
                ]),
                'use_sim_time': True,
            }],
        )],
    )

    # External views of the web cockpit (blue panel): two static cameras
    # spawned into the world, isometric and top-down. Fragment SHARED with the
    # quadruped plant, so the panel does not go dark when ROBOT_TYPE changes.
    # All the framing arithmetic and the reason they are spawned instead of
    # written into worlds/*.sdf are in scene_cameras.launch.py.
    #
    # The odom -> world seed IS PASSED EXPLICITLY, and that is the only
    # difference of this include from the quadruped plant's. The odometry here
    # comes from the DiffDrive plugin, which integrates encoders from ZERO: the
    # odom origin is the SPAWN pose, not the world's. Without this seed, an
    # `x:=5` makes both views follow a point 5 m beside the robot -- wrong by a
    # constant offset, with no error anywhere.
    #
    # The quadruped plant passes nothing because there /go2/odom is Gazebo
    # ground truth (already the pose in the world) and a non-zero seed would add
    # the spawn pose twice.
    scene_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch',
            'scene_cameras.launch.py',
        ])),
        launch_arguments={
            'follow_offset_x': LaunchConfiguration('x'),
            'follow_offset_y': LaunchConfiguration('y'),
            'follow_offset_yaw': LaunchConfiguration('yaw'),
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
        x_arg,
        y_arg,
        yaw_arg,
        gui_arg,
        use_meshes_arg,
        model_arg,
        gz_sim,
        gz_sim_headless,
        scene_cameras,
        sim_control,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
