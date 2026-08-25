"""
Plant container entrypoint — Gazebo, the robot, and the ROS<->gz bridge.

Runs on: x86 workstation ONLY, in the `sim` container. Gazebo is an OGRE 2 /
desktop-OpenGL application and must never be placed on the Aquila AM69
(CLAUDE.md rule 1). In hil mode this container still runs on the host; only nav
and perception move to the module.

    ros2 launch demo_bringup sim.launch.py

This is the ML3.5 F1 decomposition of learn.launch.py: one launch file per
container role, each the entrypoint of exactly one compose service. It wraps
demo_simulation/simulation.launch.py and adds nothing to it. The 12 s spawn and
15 s bridge delays that the demo depends on live inside that file and are
unchanged — see the long notes there for why they are load-bearing.

`robot_type` selects the plant: `diffdrive` (ML1-ML3, Gazebo's native
gz-sim-diff-drive-system) or `quadruped` (ML3.5 F3, Unitree Go2 on
ros2_control). Each maps to exactly one launch file in demo_simulation; an
unknown value fails loudly rather than silently starting the wrong plant.

The ML3.5 default is `quadruped`; `diffdrive` remains the tested fallback. The
same selector is consumed by `nav_select.launch.py`, so plant and navigation
cannot silently select different robots.
"""

from demo_bringup.robot_selection import (
    launch_file,
    official_world,
    ROBOT_LAUNCH_FILES,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Each entry maps to exactly one launch file in demo_simulation. Adding a plant
# means adding a launch file, not adding a conditional to an existing one
# (CLAUDE.md: no single launch file full of conditionals).
SUPPORTED_ROBOTS = tuple(sorted(ROBOT_LAUNCH_FILES))


def _check_robot_type(context, *args, **kwargs):
    """
    Fail loudly on an unimplemented robot_type instead of ignoring it.

    A launch argument that is declared and then never read is the silent kind
    of failure this project keeps paying for: `robot_type:=quadruped` would
    start the diff-drive plant and look like it worked.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    try:
        launch_file(robot_type, 'plant')
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    return []


def _launch_plant(context, *args, **kwargs):
    """
    Include the launch file for the selected plant.

    Resolved here rather than with a substitution so that an unknown
    robot_type fails in _check_robot_type with a message that names the
    problem, instead of failing later as a missing-file error that does not.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)

    # O mundo e resolvido AQUI e passado explicitamente, e nao deixado para o
    # default da planta. A razao e escopo de launch: `world` declarado neste
    # arquivo entra no contexto, e um DeclareLaunchArgument na descricao incluida
    # NAO sobrepoe um valor herdado -- o valor do pai vence. Se este arquivo
    # declarasse `world` vazio e confiasse no default da planta, o fragmento
    # scene_cameras.launch.py herdaria o vazio, cairia no enquadramento generico,
    # e as cameras do cockpit apontariam para a origem num mundo cuja area util
    # esta em (-4,855; 4,855). Sem erro, sem log: painel azul olhando para chao.
    world = LaunchConfiguration('world').perform(context)
    if not world:
        package, *parts = official_world(robot_type)
        world = PathJoinSubstitution(
            [FindPackageShare(package)] + parts).perform(context)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'),
            'launch',
            launch_file(robot_type, 'plant'),
        ])),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    # VAZIO = o cenario oficial do robot_type selecionado (robot_selection.py):
    # labirinto para o quadrupede, armazem para o diff-drive.
    #
    # Era o armazem cravado aqui para os dois. O quadrupede passou a ser o robo
    # padrao no F6 e a sintonia do Nav2 dele foi medida no labirinto, entao o
    # default cravado deixou o caminho mais curto rodando o robo oficial no mundo
    # errado -- com a inflacao de 0,85 pensada para corredor de 1,20 m aplicada
    # num armazem aberto, e as cameras de cena enquadrando outro lugar.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='SDF world to load. Vazio = cenario oficial do robo '
                    '(labirinto no quadrupede, armazem no diff-drive).',
    )

    # Default true: the sim container gets /dev/dri and the X socket, so the
    # GUI is the normal case. Set false for headless CI runs.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless runs.',
    )

    # F5 validated the quadruped plant with Nav2 on the host and the Aquila.
    # F6 promotes it while preserving diffdrive as an explicit fallback.
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='quadruped',
        description=(
            'Which plant to simulate: ' + ' | '.join(SUPPORTED_ROBOTS) + '.'
        ),
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        robot_type_arg,
        OpaqueFunction(function=_check_robot_type),
        OpaqueFunction(function=_launch_plant),
    ])
