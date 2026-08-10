"""
Launch turtlesim + teleop side by side.

Demonstrates launch composition of two third-party nodes. Not namespaced under
/demo on purpose — turtlesim publishes /turtle1/* and remapping it would
obscure the topic names the beginner is trying to learn.

Usage:
    ros2 launch demo_tutorials turtlesim_demo.launch.py

The teleop node needs a terminal with keyboard focus. Launching with
`output='screen'` gives you that.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    teleop = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop_turtle',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([turtlesim, teleop])
