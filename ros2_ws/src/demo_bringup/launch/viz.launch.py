"""
Visualization container entrypoint — RViz2.

Runs on: x86 workstation ONLY, in the `viz` container. RViz2 is an OGRE 2 /
desktop-OpenGL application and the AM69 exposes only OpenGL ES 3.2 and Vulkan
1.2. It must never be placed on the module (CLAUDE.md rule 1), which is why
compose.module.yml has no viz service at all.

    ros2 launch demo_bringup viz.launch.py

ML3.5 F1 decomposition of learn.launch.py.

No /clock gate here, unlike nav and perception. RViz tolerates starting before
the simulator: displays sit empty and populate once data arrives, which is
visible on screen rather than silent. Gating it would only delay the window a
human uses to watch the rest of the stack come up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # This package's own RViz config — a copy of Nav2's with the RobotModel
    # display ENABLED and the TF display DISABLED.
    #
    # DO NOT point this back at nav2_bringup/rviz/nav2_default_view.rviz.
    # That file ships with RobotModel disabled and TF enabled with Show Axes +
    # Show Names, so RViz draws no robot body and 33 labelled axis triads
    # floating where the robot should be. It looks precisely like a robot that
    # has come apart, and it is not — see the header of rviz/demo_view.rviz.
    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_bringup'), 'rviz', 'demo_view.rviz',
        ]),
        description='RViz2 config file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        rviz,
    ])
