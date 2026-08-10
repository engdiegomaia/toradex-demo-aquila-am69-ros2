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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'worlds', 'warehouse.sdf',
        ]),
        description='Absolute path to the SDF world to load.',
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='demo_robot',
        description=(
            'Model name in Gazebo. Must match the robot_name baked into the '
            'bridge config: the DiffDrive plugin scopes its gz topics under '
            '/model/<robot_name>/.'
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

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_description'), 'urdf', 'demo_robot.urdf.xacro',
        ]),
        description='Absolute path to the robot xacro to spawn.',
    )

    # value_type=str is required — without it the expanded URDF is parsed as
    # YAML and robot_state_publisher rejects it. Same reasoning as in
    # demo_description/launch/view_robot.launch.py.
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('model'),
            ' robot_name:=', LaunchConfiguration('robot_name'),
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
    spawn_robot = Node(
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
    )

    bridge = Node(
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
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        yaw_arg,
        gui_arg,
        model_arg,
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
