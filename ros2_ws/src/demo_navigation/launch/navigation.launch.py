"""
Nav2 stack over a static map — the navigation half of ML3.

Runs on: Aquila AM69 (arm64) in target mode, x86 host in learn mode. This is the
half of the demo that migrates to the module; the simulator never does.

    ros2 launch demo_navigation navigation.launch.py

Delegates to Nav2's bringup_launch.py rather than instantiating each server by
hand: the lifecycle-manager wiring and node ordering are exactly what upstream
maintains, and duplicating it here would rot. What this file owns is the
project's parameter file, map, and namespace choices.

That delegation now targets a VENDORED copy under launch/nav2_vendored/ instead
of the installed nav2_bringup package. The launch logic is upstream's, unchanged;
only the package-root paths were re-rooted. The reason is CLAUDE.md rule 1: the
ros-jazzy-nav2-bringup *package* hard-depends on nav2-minimal-tb3/tb4-sim and
ros-gz-sim, which drag OGRE 2 and the whole Gazebo stack into the `nav` container
— an image that ships to the Aquila AM69, where desktop OpenGL does not exist.
See launch/nav2_vendored/README.md for the full provenance and the exact edits.

RViz is deliberately absent — it is an OGRE 2 application and must stay on the
x86 host (CLAUDE.md rule 1). demo_bringup's learn launch starts it separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'maps', 'warehouse.yaml',
        ]),
        description=(
            'Static map for AMCL and the global costmap. Generate it with '
            'slam.launch.py before the first navigation run.'
        ),
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'config', 'nav2_params.yaml',
        ]),
        description='Nav2 parameter file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description=(
            'Transition the lifecycle nodes to active automatically. Required '
            'by the ML3 acceptance criterion "Nav2 reaches active state".'
        ),
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description=(
            'Load the Nav2 servers into one component container. Keeps the '
            'process count and IPC overhead down, which matters on the module.'
        ),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'),
            'launch', 'nav2_vendored', 'bringup_launch.py',
        ])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        autostart_arg,
        use_composition_arg,
        nav2_bringup,
    ])
