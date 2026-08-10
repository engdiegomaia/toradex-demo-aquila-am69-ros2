"""
Perception stub + costmap adapter — the perception half of ML3.

Runs on: Aquila AM69 (arm64) in target mode, x86 host in learn mode. Identical
code and identical launch either way — this node pair is CPU-only.

    ros2 launch demo_perception perception.launch.py

Both nodes live in one launch file because they are one deployable unit: the
container that gets swapped for TIDL inference later replaces both.
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

    assumed_range_arg = DeclareLaunchArgument(
        'assumed_range_m',
        default_value='2.0',
        description=(
            'Ground-plane distance assumed for every detection. A 2D box has '
            'no depth; see detections_to_cloud.py for why this is a stub-only '
            'approximation.'
        ),
    )

    detection_stub_node = Node(
        package='demo_perception',
        executable='detection_stub',
        name='detection_stub',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    detections_to_cloud_node = Node(
        package='demo_perception',
        executable='detections_to_cloud',
        name='detections_to_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'assumed_range_m': LaunchConfiguration('assumed_range_m'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        assumed_range_arg,
        detection_stub_node,
        detections_to_cloud_node,
    ])
