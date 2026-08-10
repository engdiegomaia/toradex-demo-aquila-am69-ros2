r"""
Keyboard teleoperation — ML3 acceptance ("robot can be teleoperated").

Runs on: x86 workstation. Start it in its own terminal, after
simulation.launch.py is already up:

    ros2 launch demo_simulation teleop.launch.py

teleop_twist_keyboard only reads keys from the terminal that has focus, so it
needs `prefix='xterm -e'` when launched this way — a launch-started node has no
attached TTY. If xterm is not installed, run the node directly instead:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/demo/cmd_vel

Publishes to /demo/cmd_vel, the same topic Nav2 drives. Do not run teleop and
Nav2 at the same time unless you want to watch them fight over the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.4',
        description='Initial linear speed in m/s.',
    )

    turn_arg = DeclareLaunchArgument(
        'turn',
        default_value='1.0',
        description='Initial angular speed in rad/s.',
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'turn': LaunchConfiguration('turn'),
            'use_sim_time': True,
        }],
        remappings=[('/cmd_vel', '/demo/cmd_vel')],
    )

    return LaunchDescription([
        speed_arg,
        turn_arg,
        teleop_node,
    ])
