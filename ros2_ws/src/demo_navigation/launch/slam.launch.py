r"""
SLAM mapping run — one-off, used to produce the static map ML3 then navigates on.

Runs on: x86 host. This is a development tool, not part of the demo. The module
runs AMCL over the saved map, never SLAM.

Procedure (three terminals):

    # 1. simulator
    ros2 launch demo_simulation simulation.launch.py

    # 2. this file
    ros2 launch demo_navigation slam.launch.py

    # 3. drive the robot over the whole warehouse
    ros2 launch demo_simulation teleop.launch.py

Then save the result into the package source tree (not install/, which is a
symlink farm that gets wiped by a clean rebuild):

    ros2 run nav2_map_server map_saver_cli -f \\
        ros2_ws/src/demo_navigation/maps/warehouse

Commit both warehouse.yaml and warehouse.pgm. After that this launch file is
only needed if the world changes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    # Parameters are inline rather than in a YAML file: this is a throwaway
    # mapping run, and the project's "config in YAML" rule targets the Nav2
    # parameters that ship with the demo.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/demo/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'transform_publish_period': 0.02,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node,
    ])
