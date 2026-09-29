"""
Cockpit scene cameras -- spawn of the two external views (ML3.5 F3b).

Runs on: x86 workstation ONLY, inside the `sim` container. They are render
sensors inside the Gazebo process; none of this exists on the module (rule 1 of
CLAUDE.md).

Included by quadruped.launch.py and by simulation.launch.py. It is not a
container entrypoint and does not appear in compose: it is a shared fragment, so
that the cockpit's blue panel does not depend on which plant ROBOT_TYPE selected.
A screen that goes dark when the robot is switched is exactly the kind of silent
failure this project has already paid dearly for.

    ros2 launch demo_simulation quadruped.launch.py scene_cameras:=false

turns both off, to measure the render cost without them.

THE FRAMING IS DERIVED FROM THE WORLD, AND THAT IS THE DESIGN

The ten numbers of the two cameras come from `demo_simulation/scenarios.py`,
indexed by the world file. There is no framing default here: choosing the world
ALREADY chooses the framing.

The reason is in the header of that file, and the failure it removes is worth
repeating: switching worlds without switching the framing points the camera at
empty floor, with no error and no log, and whoever looks at the blue panel
concludes the robot is not walking. While the two were independent arguments,
the wrong combination was the easiest to produce -- it was enough to forget half
of a six-line SIM_ARGS.

Every `scene_*` accepts an explicit value, which beats the table. The default is
EMPTY, meaning "derive from the world" -- and not a number, because a number
here is indistinguishable from an operator choice.

    # probe the maze from higher up, keeping the other nine numbers
    ros2 launch demo_bringup sim.launch.py scene_top_z:=20.0

FRAMING -- the arithmetic, and the measurements that corrected it

hfov = 1.20 rad and image 800x600, so:

    horizontal half-width   tan(0.60)            = 0.684
    vertical half-width     tan(0.60) * 600/800  = 0.513

The coverage of a top view at height h is therefore 1.368*h by 1.026*h.

  WAREHOUSE, top (0, 0, 6.0): covers 8.2 x 6.2 m centred on the origin.

  The first version used 12 m, from the arithmetic of covering 16.4 x 12.3 m.
  Measured in warehouse.sdf: at 12 m the camera is ABOVE the roof beams and the
  whole image is an orange beam across the frame, with the robot hidden behind
  it. There is no error anywhere -- just a beam. At 6 m the camera is below the
  structure and the robot appears clean on the floor. DO NOT raise this height
  without looking at the warehouse image.

  It is not a view of the whole world, and that is deliberate: the warehouse is
  about 28 x 45 m and framing it entirely would require h ~ 44 m, a height at
  which the robot becomes a handful of pixels. Whoever wants the whole map looks
  at the green panel.

  WAREHOUSE, iso (-3.0, 3.0, 2.4): looks at the origin. The direction
  (3, -3, -2.4) gives
      yaw   = atan2(-3, 3)               = -0.7854
      pitch = atan2(2.4, sqrt(3^2+3^2))  = 0.5150
  at 4.87 m distance, covering about 6.7 m of width in the origin plane -- the
  robot fills a useful fraction of the frame instead of being a white dot.

  The quadrant matters. The first three attempts started from (-x, -y) and all
  landed inside the warehouse shelf aisle: half the frame becomes shelving and
  the robot is behind it. From (-x, +y) the line of sight is clear and the
  background still has enough warehouse for the scene not to look like an empty
  studio.

  MAZE11 -- the OFFICIAL scenario, and the only world in the project whose
  usable area is not centred on the origin. The bbox of maze11.stl was read from
  the binary and multiplied by the 0.002 scale: local x[1.017, 12.617]
  y[-12.594, -0.994]. With the link pose (-11.672, 11.649) the maze occupies, in
  the world, x[-10.655, 0.945] and y[-0.945, 10.655], centre (-4.855, 4.855),
  11.60 x 11.60 m -- which matches the world header. The robot spawns at (0, 0),
  the lower right corner. The maze has no ceiling, so here the top view can go
  higher: 13 m covers 17.8 x 13.3 m, the 11.6 m with margin.

FOLLOWING THE ROBOT, AND THE odom -> WORLD SEED

Both views follow the robot by default (`follow:=false` turns it off).
scene_view_controller uses /demo/odom as the robot pose, and it needs that pose
in the WORLD frame, which is the only one Gazebo's set_pose understands.

The two plants differ here:

  quadruped    /go2/odom is ground truth from gz-sim-odometry-publisher-system:
               already the pose in the world. Seed = 0, and quadruped.launch.py
               passes nothing.

  diff-drive   /odom is integrated from the encoders by the DiffDrive plugin,
               with its origin at the SPAWN pose. Seed = the spawn x/y/yaw, and
               simulation.launch.py wires all three explicitly.

The `follow_offset_*` arguments exist for this. They are NOT read from `x`, `y`
and `yaw` in here by scoping accident: both plants declare those three names and
the include would inherit them from both, which would make the quadruped camera
follow a ghost shifted by the spawn pose (in maze11, shifted AND rotated by 90
degrees). Whoever knows whether the odometry is ground truth is the plant, and
it is the one that passes them.

WHY THE TWO `create` CALLS ARE NOT DELAYED BY A TIMER
Same reason as the robot spawn in quadruped.launch.py: `create` already retries
the world-name service instead of failing, so starting early costs a few retry
lines in the log. The robot had an additional reason not to wait (it falls while
nobody controls it); the camera does not fall, but gains nothing from waiting
either.
"""

from demo_simulation.scenarios import camera_pose
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Order of the five numbers of a camera pose in scenarios.py.
POSE_FIELDS = ('x', 'y', 'z', 'pitch', 'yaw')


def _pose_args(name: str) -> list:
    """
    Declare the five arguments of a camera, all EMPTY by default.

    Empty means "derive from the world". There is deliberately no number here: a
    numeric default is indistinguishable from a value the operator chose, and
    that is how the warehouse framing survived the switch to the maze.
    """
    return [
        DeclareLaunchArgument(
            f'scene_{name}_{field}',
            default_value='',
            description=f'{field} of camera {name}. Empty = taken from '
                        'scenarios.py for the selected world.',
        )
        for field in POSE_FIELDS
    ]


def _resolve(context, name: str) -> dict:
    """Effective camera pose: what was passed, otherwise the world table."""
    world = LaunchConfiguration('world').perform(context)
    table = camera_pose(world, name)
    resolved = {}
    for index, field in enumerate(POSE_FIELDS):
        explicit = LaunchConfiguration(f'scene_{name}_{field}').perform(context)
        resolved[field] = explicit if explicit else str(table[index])
    return resolved


def _spawn(name: str, pose: dict) -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_cockpit_scene_{name}',
        output='screen',
        condition=IfCondition(LaunchConfiguration('scene_cameras')),
        arguments=[
            '-file', PathJoinSubstitution([
                FindPackageShare('demo_simulation'), 'models',
                f'cockpit_scene_{name}.sdf',
            ]),
            '-name', f'cockpit_scene_{name}',
            '-allow_renaming', 'true',
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-P', pose['pitch'],
            '-Y', pose['yaw'],
        ],
    )


def controller_params(iso: dict, top: dict) -> dict:
    """
    Parameters of scene_view_controller: the ten poses plus following.

    Public (no underscore) so the launch test can assert on the dictionary
    directly. Reaching `Node._Node__parameters` from the test side does not
    work: Node normalises keys and values into tuples of TextSubstitution, and
    an assertion on that breaks when `launch` changes version, without anything
    in the project having changed.
    """
    params = {'use_sim_time': True}
    for name, pose in (('iso', iso), ('top', top)):
        for field, value in pose.items():
            params[f'{name}_{field}'] = float(value)

    params['follow'] = ParameterValue(
        LaunchConfiguration('follow'), value_type=bool)
    for axis in ('x', 'y', 'yaw'):
        params[f'follow_offset_{axis}'] = ParameterValue(
            LaunchConfiguration(f'follow_offset_{axis}'), value_type=float)
    return params


def _view_controller(iso: dict, top: dict) -> Node:
    """
    Build the node that the cockpit view buttons move.

    The parameters come from the SAME poses that positioned the cameras above.
    That is the reason it lives in this file and not in sim_control.launch.py:
    the initial framing is resolved only once, and the controller is born
    knowing where the camera started -- even when the scenario reframed
    everything.

    The poses are already RESOLVED here (the OpaqueFunction read them from the
    scenario or from the argument), so they become float in Python and do not go
    through ParameterValue. ParameterValue(value_type=float) receiving a plain
    str fails at assembly with "value='-13.0' is not an instance of <class
    'float'>" -- it converts a SUBSTITUTION to float, not text to float.
    Measured on 2026-08-25.

    The three below remain substitutions, because `follow` and the
    `follow_offset_*` arrive as LaunchConfiguration: there the converter is
    necessary, or the node dies with "Wrong parameter type" on activation, since
    it declares the offsets as double.
    """
    return Node(
        package='demo_simulation',
        executable='scene_view_controller',
        name='scene_view_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('scene_cameras')),
        parameters=[controller_params(iso, top)],
    )


def _cameras(context, *args, **kwargs) -> list:
    """Resolve the framing and return the three nodes that depend on it."""
    iso = _resolve(context, 'iso')
    top = _resolve(context, 'top')
    return [_spawn('iso', iso), _spawn('top', top), _view_controller(iso, top)]


def _follow_args() -> list:
    return [
        DeclareLaunchArgument(
            'follow',
            default_value='true',
            description='Scene views follow the robot. false freezes both '
                        'at the scenario framing (the wide view).',
        ),
        DeclareLaunchArgument(
            'follow_offset_x',
            default_value='0.0',
            description='odom -> world, X. Zero for ground-truth odometry '
                        '(quadruped); the spawn pose for the diff-drive.',
        ),
        DeclareLaunchArgument('follow_offset_y', default_value='0.0'),
        DeclareLaunchArgument('follow_offset_yaw', default_value='0.0'),
    ]


def generate_launch_description() -> LaunchDescription:
    enabled_arg = DeclareLaunchArgument(
        'scene_cameras',
        default_value='true',
        description='Spawn the cockpit scene cameras (blue panel). Set false '
                    'to run the plant without the extra render cost.',
    )

    # Declared here too, and not just inherited from the plant: this fragment
    # READS the world to resolve the framing, and an argument read without being
    # declared blows up with "LaunchConfiguration not found" instead of saying
    # what is missing.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Selected world. Empty falls back to the generic framing '
                    '(cameras looking at the origin).',
    )

    return LaunchDescription(
        [enabled_arg, world_arg]
        + _pose_args('iso')
        + _pose_args('top')
        + _follow_args()
        + [OpaqueFunction(function=_cameras)]
    )
