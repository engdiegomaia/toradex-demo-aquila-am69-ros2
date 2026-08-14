"""
learn mode — the whole demo on one x86 workstation.

Runs on: x86 workstation, everything native, no containers. This is the mode
phases L1-L3 use: containers enter at L4, and ROS 2 on an unfamiliar system is
hard enough to debug without Docker in the way.

    ros2 launch demo_bringup learn.launch.py

Composition order matters and is enforced with timers rather than luck:

    t=0s    simulation  — Gazebo starts loading the world
    t=12s   (robot spawn, inside simulation.launch.py)
    t=15s   (ros_gz_bridge, inside simulation.launch.py)
    t=20s   perception  — needs /demo/camera/image_raw to be bridged
    t=25s   navigation  — needs /clock, /demo/scan and TF to be bridged

The internal 12 s and 15 s delays are explained in simulation.launch.py; the
short version is that the warehouse world takes ~10 s to load and a spawner
started before it is ready retries forever without ever completing its
handshake. Everything here has to clear those two marks.

Nav2 started too early comes up before /clock is publishing and every lifecycle
node stalls waiting for a transform that has no timestamps yet. The delays are
generous on purpose; this is a demo, not a boot-time benchmark. On a slower
machine, or with a heavier world, raise all of these together.

There is one launch file per mode, with no cross-mode conditionals — project
convention. emul.launch.py and target.launch.py arrive with ML4, when the same
services move into containers.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Same source as simulation.launch.py's default — see the note there.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_minimal_tb4_sim'), 'worlds', 'warehouse.sdf',
        ]),
        description='SDF world to load.',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'maps', 'warehouse.yaml',
        ]),
        description='Static map for AMCL and the global costmap.',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2. x86 only — never set true on the module.',
    )

    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Start Nav2. Set false to teleoperate without navigation.',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch', 'simulation.launch.py',
        ])),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    perception = TimerAction(
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('demo_perception'), 'launch', 'perception.launch.py',
            ])),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )],
    )

    navigation = TimerAction(
        period=25.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('demo_navigation'), 'launch', 'navigation.launch.py',
            ])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('navigation')),
        )],
    )

    # This package's own RViz config — a copy of Nav2's with the RobotModel
    # display ENABLED and the TF display DISABLED.
    #
    # DO NOT point this back at nav2_bringup/rviz/nav2_default_view.rviz.
    # That file ships with RobotModel disabled and TF enabled with Show Axes +
    # Show Names, so RViz draws no robot body and 33 labelled axis triads
    # floating where the robot should be. It looks precisely like a robot that
    # has come apart, and it is not — see the header of rviz/demo_view.rviz.
    #
    # demo_description's config is for viewing the model alone and has no map
    # frame, so it is not a substitute here.
    rviz = TimerAction(
        period=25.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('demo_bringup'), 'rviz', 'demo_view.rviz',
            ])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        )],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        rviz_arg,
        navigation_arg,
        simulation,
        perception,
        navigation,
        rviz,
    ])
