"""
Select the matching Nav2 launch for ``robot_type``.

This file is deliberately only a dispatcher. The two navigation stacks remain
explicit and separate: ``nav.launch.py`` owns static-map/AMCL navigation for the
diff-drive robot, while ``nav_quadruped.launch.py`` owns reactive rolling-map
navigation and the Go2 unit/TF adapters. Keeping the mapping beside the plant
mapping prevents a valid but dangerous mixed stack.
"""

from demo_bringup.robot_selection import launch_file, ROBOT_LAUNCH_FILES
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


SUPPORTED_ROBOTS = tuple(sorted(ROBOT_LAUNCH_FILES))
ROBOT_DEFAULT_PARAMS = '__robot_default__'


def _check_robot_type(context, *args, **kwargs):
    """Reject an unknown robot before any Nav2 process is started."""
    robot_type = LaunchConfiguration('robot_type').perform(context)
    try:
        launch_file(robot_type, 'navigation')
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    return []


def _launch_navigation(context, *args, **kwargs):
    """Include exactly one of the two explicit navigation entrypoints."""
    robot_type = LaunchConfiguration('robot_type').perform(context)
    params_override = LaunchConfiguration('params_override').perform(context)
    launch_arguments = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    if params_override and params_override != ROBOT_DEFAULT_PARAMS:
        launch_arguments['params_file'] = params_override
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_bringup'),
            'launch',
            launch_file(robot_type, 'navigation'),
        ])),
        launch_arguments=launch_arguments.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    """Build the robot-aware navigation dispatcher."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='quadruped',
            description='Robot stack: ' + ' | '.join(SUPPORTED_ROBOTS) + '.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Follow the Gazebo /clock in learn and HIL modes.',
        ),
        DeclareLaunchArgument(
            'params_override',
            default_value='',
            description=(
                'Optional Nav2 parameters file. Empty keeps the selected '
                'robot launch default.'
            ),
        ),
        OpaqueFunction(function=_check_robot_type),
        OpaqueFunction(function=_launch_navigation),
    ])
