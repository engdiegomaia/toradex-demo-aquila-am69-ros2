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
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'config', 'slam_params.yaml',
        ]),
        description='slam_toolbox parameter file.',
    )

    # The bulk of the tuning comes from the params file; the scan topic is
    # remapped instead of being passed as a parameter.
    #
    # slam_toolbox reads `scan_topic` when it declares parameters, and setting
    # it from an external params file does not reliably take effect — the node
    # comes up subscribed to nothing but /clock, logs no error, and never
    # publishes a map or map->odom. A remap is applied by rclcpp before the
    # node's own subscription is created, so it always wins. Verified: with the
    # parameter alone, `ros2 node info /slam_toolbox` showed no scan
    # subscriber at all.
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/scan', '/demo/scan')],
    )

    # slam_toolbox is a LIFECYCLE node on Jazzy and comes up `unconfigured`.
    # Left there it looks alive — the process runs and logs "Node using stack
    # size" — but it creates no scan subscription and publishes no map or
    # map->odom, silently. These two transitions are what actually start it.
    #
    # Chained deliberately: activate is emitted only after configure reports
    # success, instead of firing both on a timer and hoping.
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_arg,
        slam_toolbox_node,
        activate_slam_on_configure,
        configure_slam,
    ])
