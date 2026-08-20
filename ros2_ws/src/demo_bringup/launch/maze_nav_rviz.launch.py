"""Interactive quadruped navigation: Nav2 plus RViz click-to-goal view.

The Gazebo plant runs separately in ``run_quadruped_sim.sh``.  This launch is
host-side and starts the TF/odometry bridge, Nav2 and RViz with the Go2 sensor
topics already enabled.  Select ``Nav2 Goal`` (the arrow target icon) and click
the destination, then drag to set the desired final heading.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start Nav2 and RViz configured for click-to-goal maze navigation."""
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_bringup'), 'rviz', 'demo_view.rviz',
        ]),
        description='RViz configuration with Go2 TF, lidar and camera displays.',
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_bringup'), 'launch',
            'nav_quadruped.launch.py',
        ])),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='maze_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([rviz_config, nav, rviz])
