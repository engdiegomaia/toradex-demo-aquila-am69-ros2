"""
Gazebo Harmonic + Unitree Go2 + ros2_control — the quadruped plant (ML3.5 F3).

Runs on: x86 workstation ONLY, in the `sim` container. Gazebo is OGRE 2 /
desktop OpenGL and must never be placed on the Aquila AM69 (CLAUDE.md rule 1).

    ros2 launch demo_simulation quadruped.launch.py

This is the legged counterpart of simulation.launch.py. Same responsibility —
own the simulator, the robot instance and the ROS<->gz bridge, nothing else —
with the plant swapped. Nav2 and perception compose on top through demo_bringup
and must not be able to tell the difference (topic contract, CLAUDE.md).

Derived from unitree_guide_controller/launch/gazebo.launch.py upstream, with
three deliberate differences, all of them load-bearing:

1. NO RViz2. Upstream starts an `rviz_ocs2` node inside this same launch. That
   is OGRE 2 inside the sim container; in this project RViz2 lives in the `viz`
   container on the host (rule 1). The F2 spike already proved the plant comes
   up without it.
2. Controllers are chained on spawn completion, not on a wall-clock timer.
   Same reasoning as wait_for_clock in F1: a timer measures the wrong thing.
3. The gait FSM is driven by the contract bridge (twist_to_inputs), not by a
   keyboard node. Upstream expects a human at a terminal.

WHY THERE IS NO DIFF-DRIVE-STYLE PLUGIN HERE
The diff-drive plant gets odometry, joint states and cmd_vel from Gazebo system
plugins declared in the URDF. A quadruped has no equivalent native plugin: the
joints are driven by a ros2_control controller (unitree_guide_controller) whose
hardware interface (gz_quadruped_hardware) is loaded INSIDE the gz sim process.
That is why sim is one container and cannot be split (guia-ml35-docker.md §1),
and it is why ML3.5 reverses the ML2 decision against gz_ros2_control.
"""

from demo_simulation.scenarios import missing_models, spawn_pose
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


class ScenarioPose(Substitution):
    """
    Um campo da pose de nascimento: o argumento, senao a tabela do cenario.

    E uma Substituicao e nao um OpaqueFunction porque o destino e `arguments` de
    um Node, que aceita substituicoes e NAO aceita acoes -- um OpaqueFunction ali
    e aceito na montagem e falha na execucao, com erro que fala de tipos e nao do
    argumento.

    Resolver na hora da execucao tambem e o que permite `world:=` e `yaw:=`
    chegarem de SIM_ARGS: nenhum dos dois existe quando a descricao e montada.
    """

    def __init__(self, field: str) -> None:
        super().__init__()
        self._field = field

    def perform(self, context) -> str:
        explicit = LaunchConfiguration(self._field).perform(context)
        if explicit:
            return explicit
        world = LaunchConfiguration('world').perform(context)
        return str(spawn_pose(world)[self._field])


def _check_external_models(context, *args, **kwargs) -> list:
    """
    Falha ALTO quando o mundo pede uma malha que nao esta montada.

    Sem esta checagem o modo de falha e o pior possivel: o Gazebo carrega o
    mundo, o modelo do labirinto fica sem visual e sem colisao, e o resultado e
    um plano vazio. O lidar nao ve parede nenhuma, o Nav2 planeja em linha reta,
    e a meta termina SUCCEEDED -- com numeros MELHORES que os reais. O ensaio
    passa e ninguem descobre que o labirinto nao estava la.

    Mede-se aqui e nao no compose porque quem sabe de qual mundo se trata e o
    launch, e porque o caminho default de `MAZE_MODELS:-./models-extra` faz o
    Docker CRIAR um diretorio vazio em vez de recusar o mount.
    """
    world = LaunchConfiguration('world').perform(context)
    faltando = missing_models(world)
    if faltando:
        raise RuntimeError(
            'o mundo %s carrega modelo(s) externo(s) ao repositorio que nao '
            'estao montados: %s.\n'
            'A malha do labirinto tem licenca TODO no upstream '
            '(github.com/cafemesa/ros_maze_worlds) e por isso NAO e vendorizada '
            '-- mesmo bloqueio que fez o projeto trocar o A1 pelo Go2.\n'
            'Aponte MAZE_MODELS para o diretorio models/ daquele repositorio:\n'
            '  MAZE_MODELS=/caminho/ros_maze_worlds/models '
            'docker compose -f compose.host.yml up -d sim\n'
            'Sem isso o Gazebo sobe um plano VAZIO sem acusar nada, e qualquer '
            'medicao de navegacao feita nele e ficcao.'
            % (world, ', '.join(faltando))
        )
    return []


def generate_launch_description() -> LaunchDescription:
    # O CENARIO OFICIAL do quadrupede e o labirinto, e nao o mundo vazio.
    #
    # Era `quadruped_empty.sdf` desde o spike do F2, quando a imagem de simulacao
    # nao tinha nem Nav2: o mundo vazio era o unico que subia. Ficou como default
    # por inercia, e o efeito colateral e que o comando mais curto que existe
    # (`ros2 launch ... quadruped.launch.py`) subia um chao infinito sem nada
    # para navegar -- entao TODA medicao de navegacao exigia passar `world:=` a
    # mao, e uma medicao feita sem passar media navegacao em campo aberto.
    #
    # Os mundos pequenos continuam a um argumento de distancia:
    #   ros2 launch demo_simulation quadruped.launch.py \
    #     world:=$(ros2 pkg prefix demo_simulation)/share/demo_simulation/worlds/quadruped_empty.sdf
    #
    # A pose de nascimento e o enquadramento das cameras acompanham o mundo por
    # `scenarios.py`; nao ha como trocar de mundo e esquecer os outros nove
    # numeros. Ver o cabecalho daquele arquivo.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'worlds',
            'quadruped_maze11.sdf',
        ]),
        description='Absolute path to the SDF world to load. O default e o '
                    'cenario oficial (labirinto).',
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='demo_robot',
        description='Model name in Gazebo. Kept identical to the diff-drive '
                    'plant so anything keyed on the model name still matches.',
    )

    # Gait tuning, as a controller param file rather than compiled-in literals.
    # Overridable so a sweep is `gait_params:=/tmp/try.yaml`, not a rebuild of
    # the sim image — which is what every gait experiment before ML3.5 phase A
    # cost. See the file's own header for why the spawner is the injection point
    # instead of go2_description/config/gazebo.yaml.
    gait_params_arg = DeclareLaunchArgument(
        'gait_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'config', 'gait_go2.yaml',
        ]),
        description='Controller param file with the trot tuning, loaded into '
                    'unitree_guide_controller before it is configured.',
    )

    # Vazio = pega de scenarios.py pelo mundo. Um default numerico aqui e
    # indistinguivel de uma escolha do operador, e foi assim que o robo passou a
    # nascer olhando para a parede quando o cenario virou o labirinto: yaw 0 e
    # correto no armazem e errado no maze11, e nada acusa a diferenca -- o robo
    # so gasta os primeiros segundos girando dentro de um corredor de 1,20 m.
    x_arg = DeclareLaunchArgument('x', default_value='')
    y_arg = DeclareLaunchArgument('y', default_value='')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='')

    # Spawn height. NOT cosmetic and NOT the same as the diff-drive's -z 0.1.
    # A quadruped spawned at floor level starts with its legs already
    # interpenetrating the ground plane; the solver pushes it out violently and
    # the robot is on its back before the controller ever loads. Upstream's own
    # README uses height:=0.43 for the A1 for this reason. The F2 spike used
    # 0.5 for the Go2 and the robot settled to a 0.353 m standing height, so
    # 0.5 leaves a small drop with no interpenetration.
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Spawn height in metres. A quadruped spawned at floor '
                    'level interpenetrates the ground and tips over before '
                    'the controller loads.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with its GUI. Set false for headless runs.',
    )

    scene_cameras_arg = DeclareLaunchArgument(
        'scene_cameras',
        default_value='true',
        description='Spawn the cockpit scene cameras. Set false for navigation '
                    'measurements without the extra render cost.',
    )

    # GAZEBO:=true selects xacro/gazebo.xacro, which declares the
    # <ros2_control> block with the gz_quadruped_hardware/GazeboSimSystem
    # plugin and includes the foot force sensors. Without it the xacro expands
    # to the real-hardware interface (xacro/ros2_control.xacro) and the plant
    # comes up with no simulated actuators — a robot that loads and never
    # moves, with no error naming the cause.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('demo_simulation'), 'urdf',
                'go2_sim.urdf.xacro',
            ]),
            ' GAZEBO:=true',
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 3'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -s -v 3'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # Publishes the fixed TF edges and animates the 12 leg joints from
    # /joint_states. Unlike the diff-drive plant, nothing here publishes
    # odom -> base_link yet: legged odometry is F5 work. Nav2 is not started by
    # this launch, so the missing edge does not break anything in F3.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            # Upstream sets this. The quadruped's joint states come from
            # ros2_control rather than a Gazebo plugin, and the timestamps do
            # not line up with /clock closely enough for the default check.
            'ignore_timestamp': True,
        }],
    )

    # NO TimerAction HERE, and this is the single most expensive thing to get
    # wrong in this file.
    #
    # The diff-drive plant delays its spawn by 12 s so that `create` does not
    # call Gazebo's "list world names" service before it exists. Copying that
    # delay here produces a robot that is spawned at height and then falls,
    # unattended, for the whole gap between the spawn and the controllers
    # activating. Measured, with the 12 s timer in place:
    #
    #   spawn at z=0.49999 -> z=0.0677 less than 1 s later -> controllers
    #   activate ~3 s after that. Final state: collapsed on the floor with legs
    #   splayed, three controllers reporting `active`, zero errors in the log,
    #   and the gait FSM cheerfully walking passive -> trotting on top of a
    #   robot that is already down.
    #
    # Nothing in that failure names its cause. It is visible only by reading
    # the pose out of gz directly, which is why the F3 gate is measured that
    # way and never from log output.
    #
    # `create` is safe to start immediately because it retries the world-names
    # service rather than failing, and it blocks on the robot_description topic
    # anyway. The cost of starting it early is a few retry lines in the log;
    # the cost of starting it late is the silent collapse above. Upstream's
    # gazebo.launch.py also spawns with no delay at all.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_demo_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-allow_renaming', 'true',
            '-z', LaunchConfiguration('height'),
            '-x', ScenarioPose('x'),
            '-y', ScenarioPose('y'),
            '-Y', ScenarioPose('yaw'),
        ],
    )

    # The controller_manager only exists once gz_quadruped_hardware has loaded
    # inside the gz sim process, which happens when the model is spawned.
    # Chained on spawn exit rather than on a timer: the timer would be
    # measuring container uptime, which is the F1 lesson (wait_for_clock).
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # The gait controller. Classic PD (unitree_guide), no RL policy and no .pt
    # weights — which is why the RL configs were dropped when go2_description
    # was vendored (see its README).
    #
    # --param-file, not `parameters=[...]`: the controller's parameters live on
    # the controller_manager node inside the gz process, not on this spawner
    # node. The spawner applies the file before load_controller, which is when
    # the controller's on_init() reads it.
    unitree_guide_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['unitree_guide_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', LaunchConfiguration('gait_params')],
        output='screen',
    )

    # F4 contract boundary for clock, odometry, lidar, camera and IMU. cmd_vel
    # stays ROS-native: twist_to_inputs consumes /demo/cmd_vel directly and
    # translates it to the gait controller's private /control_input message.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare('demo_simulation'),
                'config', 'bridge_quadruped.yaml',
            ]),
            'use_sim_time': True,
        }],
    )

    # Translates /demo/cmd_vel into the controller's Inputs message and walks
    # the gait FSM up to TROTTING. See the node's own docstring for why this is
    # not just a message cast.
    twist_to_inputs = Node(
        package='demo_simulation',
        executable='twist_to_inputs',
        name='twist_to_inputs',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Vistas externas do cockpit web (painel azul): duas cameras estaticas
    # spawnadas no mundo, isometrica e de topo. Fragmento COMPARTILHADO com a
    # planta diff-drive, para que o painel nao apague ao trocar ROBOT_TYPE.
    # Toda a conta de enquadramento e o motivo de serem spawnadas em vez de
    # escritas nos worlds/*.sdf estao em scene_cameras.launch.py.
    scene_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch',
            'scene_cameras.launch.py',
        ])),
        launch_arguments={
            'scene_cameras': LaunchConfiguration('scene_cameras'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Ponte de serviços do Gazebo: play/pause/reset do cockpit, e o set_pose que
    # o scene_view_controller usa para mover as câmeras. Fragmento
    # compartilhado, pela mesma razão do anterior.
    sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_simulation'), 'launch',
            'sim_control.launch.py',
        ])),
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        gait_params_arg,
        x_arg,
        y_arg,
        yaw_arg,
        height_arg,
        gui_arg,
        scene_cameras_arg,
        # ANTES de qualquer no: se a malha externa nao esta la, nao ha ensaio
        # valido a fazer, e o modo de falha silenciosa e caro (ver o docstring).
        OpaqueFunction(function=_check_external_models),
        bridge,
        gz_sim,
        gz_sim_headless,
        scene_cameras,
        sim_control,
        robot_state_publisher,
        spawn_robot,
        twist_to_inputs,
        # Startup chain, each link gated on the previous one actually finishing
        # rather than on elapsed time: spawn -> broadcasters -> gait controller.
        # The gait FSM waits on top of that, inside twist_to_inputs.
        # Broadcasters first, gait controller last: unitree_guide_controller
        # reads joint states and IMU on activation, and activating it before
        # its inputs exist leaves it in an inactive state that looks like a
        # silent hang rather than an error.
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster, imu_sensor_broadcaster],
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[unitree_guide_controller],
        )),
    ])
