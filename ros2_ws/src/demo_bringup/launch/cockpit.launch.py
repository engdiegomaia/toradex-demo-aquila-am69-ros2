"""
Cockpit transport container entrypoint — rosbridge + web_video_server.

Runs on: x86 workstation today, in the `cockpit` container. Unlike `viz`, this
role has NO desktop-OpenGL dependency: rosbridge_server and web_video_server are
pure transport, so the image builds for arm64 as well and can move to the Aquila
AM69 in M3 without a rewrite. That portability is the whole reason the cockpit
lives here and not inside the (amd64-only) viz image.

    ros2 launch demo_bringup cockpit.launch.py

This is the ML3.5 cockpit-web F1 entrypoint. It publishes two network surfaces
and nothing else:

  * rosbridge WebSocket (default 9090) — every panel that draws from data;
  * web_video_server HTTP MJPEG (default 8080) — the camera and scene panels.

The static HTML/CSS/JS bundle is NOT served here. It is a separate `hmi` service
(nginx) so that a UI change does not rebuild a ROS image, and so the bundle can
later be baked into a Chromium kiosk image for the module.

use_sim_time is deliberately NOT set on either node. Both are transport
processes with no control loop, and a node with use_sim_time:=true whose /clock
never arrives sits at time zero forever — here that would silently freeze the
MJPEG stream's timestamps while Gazebo is still loading. Panel-side staleness is
measured against the browser's own wall clock instead (hmi/js/ros/freshness.js).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Ports are declared, not hardcoded at the call site, because every service in
# compose.host.yml runs with network_mode: host — nothing is mapped, so these
# bind straight onto the operator's workstation and must be overridable from
# .env when they collide (CLAUDE.md: no hard-coded IPs/ports in committed files).
DEFAULT_ROSBRIDGE_PORT = '9090'
DEFAULT_VIDEO_PORT = '8080'


def generate_launch_description() -> LaunchDescription:
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value=DEFAULT_ROSBRIDGE_PORT,
        description='TCP port for the rosbridge WebSocket server.',
    )

    video_port_arg = DeclareLaunchArgument(
        'video_port',
        default_value=DEFAULT_VIDEO_PORT,
        description='TCP port for the web_video_server MJPEG endpoint.',
    )

    # 0.0.0.0 and not 127.0.0.1: in hil mode the browser may run on a different
    # machine than this container, and in M3 the bundle is served from the
    # module. Binding to loopback would work on the developer workstation and
    # fail on the bench, which is the failure mode this project keeps paying
    # for.
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Bind address for both servers.',
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
            'address': LaunchConfiguration('address'),
            # Without these two, a service call or an action goal from the
            # browser blocks rosbridge's single asyncio thread until it
            # returns. A NavigateToPose goal runs for tens of seconds, so the
            # whole cockpit — every subscription, not just the goal — would
            # stop updating while the robot drives. The panels would look
            # frozen and nothing would name the cause.
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
            # Sends the map as a PNG-compressed payload instead of a raw JSON
            # array of ints. maze11 is ~11.6 m at 0.05 m/cell, so an
            # uncompressed OccupancyGrid crosses the socket as tens of
            # thousands of comma-separated numbers on every republish.
            'png_compression': True,
        }],
    )

    # /rosapi/* — topic and type introspection. rosbridge itself does not answer
    # those; the upstream launch file always pairs the two, and the panels use
    # /rosapi/topics to tell "topic absent" apart from "topic silent", which are
    # different operator problems.
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    # MJPEG over plain HTTP. The camera stream is 640x480 RGB at sim rate; sent
    # through rosbridge as JSON it would be base64 of a raw frame per message,
    # which is what makes a data-axis cockpit feel slow. Images go over HTTP,
    # everything else goes over the WebSocket.
    video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('video_port'),
            'address': LaunchConfiguration('address'),
            # Publishers in this project use reliable QoS for the camera on
            # purpose (see bridge_quadruped.yaml). A best-effort subscriber is
            # compatible with a reliable publisher, so the default works — but
            # state it rather than inherit it, because the reverse combination
            # silently drops every frame and looks identical.
            'default_stream_type': 'mjpeg',
        }],
    )

    return LaunchDescription([
        rosbridge_port_arg,
        video_port_arg,
        address_arg,
        rosbridge,
        rosapi,
        video,
    ])
