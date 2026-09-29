"""
Gazebo service bridge -- what gives the cockpit its simulation buttons.

Runs on: x86 workstation ONLY, in the `sim` container. Exposes as ROS services
two things that exist only inside the simulator process:

    /demo/sim/control            ros_gz_interfaces/srv/ControlWorld
    /demo/sim/set_entity_pose    ros_gz_interfaces/srv/SetEntityPose

The first is play/pause/reset. The second moves the scene cameras, and its
caller is `scene_view_controller` -- not the browser; see that node's header.

THE SIMULATOR DOES NOT RUN ON THE MODULE, AND THAT DOES NOT CHANGE HERE

The button lives in the cockpit, and in M3 the cockpit is served BY the Aquila.
But whoever runs Gazebo is still the x86 workstation: the AM69 exposes only
OpenGL ES 3.2 and Vulkan 1.2, and OGRE 2 needs desktop OpenGL (rule 1 of
CLAUDE.md). What crosses over is the service call, through the ROS graph,
exactly as the Nav2 goal already does today. "Start the simulation from the
cockpit" is supported; "start the simulation ON the module" is not, and no
amount of UI code changes that.

WHY THE WORLD NAME IS READ FROM THE FILE

Gazebo services live under `/world/<name>/...`, and `<name>` is the attribute of
the `<world>` element -- not the file name. `quadruped_maze11.sdf` declares
`<world name="quadruped_maze11">`, but the `nav2_minimal_tb4_sim` warehouse
declares `<world name='warehouse'>`. Guessing from the file name is right in one
case and wrong in the other, and the error is silent: the bridge comes up,
advertises the ROS services, and every call times out on a gz service that does
not exist.

So the name comes from parsing the SDF, and a file that does not open or has no
`<world>` brings the launch down with a message saying which file it was --
instead of delivering buttons that do nothing.
"""

import xml.etree.ElementTree as ElementTree

from demo_simulation.scenarios import spawn_pose
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Reset height when the plant does not declare `height`.
#
# Only the quadruped declares this argument; the diff-drive plant is spawned with
# `-z 0.1` hard-coded in the `create`. Matching the two values matters:
# resetting a diff-drive at 0.5 m is a gratuitous fall, and resetting a
# quadruped at 0.1 m pushes its legs into the floor -- which is precisely what
# `height_arg` exists to prevent.
DEFAULT_RESET_Z = 0.1

# Remappings: the cockpit must not know the world name. If it did, switching
# scenario would require editing the JavaScript, which is exactly what the
# project avoids by separating scenario (SIM_ARGS) from code.
CONTROL_SERVICE = '/demo/sim/control'
SET_POSE_SERVICE = '/demo/sim/set_entity_pose'


def world_name_of(path: str) -> str:
    """
    Return the declared name of the first `<world>` in the SDF.

    Raises RuntimeError naming the file, because this is the kind of failure
    that would otherwise turn into "the button does nothing" three days later.
    """
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RuntimeError(
            f'could not read world {path} to find out the Gazebo world '
            f'name: {error}'
        ) from error

    world = root.find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(
            f'{path} does not declare <world name="...">; without it there is '
            'no way to build the /world/<name>/control and '
            '/world/<name>/set_pose services'
        )
    return world.get('name')


def _reset_pose(context) -> dict:
    """
    Pose to which the reset button returns the robot.

    Same precedence as the `create` that spawned it (see `ScenarioPose` in
    quadruped.launch.py): the explicit argument wins, and empty means "ask the
    scenario table". Without this the reset would return the robot to the origin
    in any world whose usable area is not at the origin -- the maze is that
    case, and the robot would reappear inside a wall with no error at all.

    `context.launch_configurations` and not `LaunchConfiguration(...).perform`:
    `height` exists only on the quadruped plant, and performing an undeclared
    argument raises instead of returning the default.
    """
    world_path = LaunchConfiguration('world').perform(context)
    table = spawn_pose(world_path)
    declared = context.launch_configurations

    def field(name):
        explicit = declared.get(name, '')
        return float(explicit) if explicit else float(table[name])

    return {
        'robot_name': declared.get('robot_name', 'demo_robot'),
        'spawn_x': field('x'),
        'spawn_y': field('y'),
        'spawn_yaw': field('yaw'),
        'spawn_z': float(declared.get('height') or DEFAULT_RESET_Z),
    }


def _bridge(context, *args, **kwargs):
    world = world_name_of(LaunchConfiguration('world').perform(context))

    # The parameter_bridge service syntax is
    # <service>@<ROS srv>[@<gz req>@<gz rep>], and the direction is always
    # gz->ROS: the bridge exposes a gz service AS a ROS service, never the
    # other way round.
    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_control_bridge',
        output='screen',
        arguments=[
            f'/world/{world}/control@ros_gz_interfaces/srv/ControlWorld',
            f'/world/{world}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        remappings=[
            (f'/world/{world}/control', CONTROL_SERVICE),
            (f'/world/{world}/set_pose', SET_POSE_SERVICE),
        ],
    ), Node(
        # The std_srvs facade. It comes up together with the bridge because
        # without the bridge it has nowhere to forward to, and separating them
        # would only create the chance of one coming up without the other. See
        # the header of sim_control_relay.py for why the browser does not speak
        # ControlWorld directly.
        package='demo_simulation',
        executable='sim_control_relay',
        name='sim_control_relay',
        output='screen',
        # The reset teleports the robot instead of resetting the world, and so
        # needs to know WHERE to. See the header of sim_control_relay.py:
        # `reset.all` deletes the robot, measured on 2026-08-26.
        parameters=[_reset_pose(context)],
    )]


def generate_launch_description() -> LaunchDescription:
    # `world` is NOT declared here: this fragment is always included by a plant
    # that already declared it, and declaring it again would create a second
    # default able to diverge from the plant's.
    return LaunchDescription([OpaqueFunction(function=_bridge)])
