"""
Exhibition role: drive the robot through a looping choreography.

One launch file per container role, as the rest of demo_bringup. This role is a
*commander* -- it produces /demo/cmd_vel and therefore must never run at the
same time as nav.launch.py, which produces the same topic. Two publishers on
/demo/cmd_vel do not error; they interleave, and the robot follows whichever
message arrived last at 20 Hz. Pick one.

Runs on: either machine. It speaks only the public contract.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Declare the tuning surface of the exhibition loop and start the node."""
    arguments = [
        # Above ~7 Hz or twist_to_inputs' 0.3 s watchdog makes the robot
        # stutter between walking and stopping. 20 Hz is the tested value.
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        # Seconds of no publishing after each movement. This is the posture
        # settle: silence is the stop command, and HOLD is where the settle
        # runs. Shorter than ~3 s does not leave time to see it.
        DeclareLaunchArgument('settle_s', default_value='5.0'),
        DeclareLaunchArgument('move_s', default_value='8.0'),
        # Measured operating points, not envelope edges. See the node.
        DeclareLaunchArgument('cruise_mps', default_value='0.10'),
        DeclareLaunchArgument('strafe_mps', default_value='0.08'),
        DeclareLaunchArgument('turn_rps', default_value='0.10'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('min_z', default_value='0.28'),
        DeclareLaunchArgument('stand_z', default_value='0.30'),
    ]

    routine = Node(
        package='demo_bringup',
        executable='demo_routine',
        name='demo_routine',
        output='screen',
        parameters=[{
            'rate_hz': LaunchConfiguration('rate_hz'),
            'settle_s': LaunchConfiguration('settle_s'),
            'move_s': LaunchConfiguration('move_s'),
            'cruise_mps': LaunchConfiguration('cruise_mps'),
            'strafe_mps': LaunchConfiguration('strafe_mps'),
            'turn_rps': LaunchConfiguration('turn_rps'),
            'loop': LaunchConfiguration('loop'),
            'min_z': LaunchConfiguration('min_z'),
            'stand_z': LaunchConfiguration('stand_z'),
            # The routine paces itself off /clock when a simulator is running,
            # so a settle of 5 s is 5 s of simulated time whatever the RTF.
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([*arguments, routine])
