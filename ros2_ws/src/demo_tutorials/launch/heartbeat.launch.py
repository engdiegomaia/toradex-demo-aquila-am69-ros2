"""
Launch the heartbeat publisher + subscriber together.

The L1 exit exercise (ML1 in .ai/AGENTS.md §9):
    ros2 launch demo_tutorials heartbeat.launch.py rate_hz:=2.0

Both nodes come up under the /demo namespace; the subscriber logs receipt at
the configured rate with no gaps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='1.0',
        description='Publish rate for heartbeat_publisher, in Hz (> 0).',
    )

    rate_hz = LaunchConfiguration('rate_hz')

    publisher_node = Node(
        package='demo_tutorials',
        executable='heartbeat_publisher',
        name='heartbeat_publisher',
        namespace='demo',
        output='screen',
        parameters=[{'rate_hz': PythonExpression(['float(', rate_hz, ')'])}],
    )

    subscriber_node = Node(
        package='demo_tutorials',
        executable='heartbeat_subscriber',
        name='heartbeat_subscriber',
        namespace='demo',
        output='screen',
    )

    return LaunchDescription([rate_hz_arg, publisher_node, subscriber_node])
