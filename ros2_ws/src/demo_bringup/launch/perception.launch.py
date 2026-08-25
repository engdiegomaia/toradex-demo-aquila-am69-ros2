"""
Perception container entrypoint — detection stub + costmap adapter.

Runs on: x86 host in learn mode, Aquila AM69 (arm64) in hil mode. Identical
image and identical launch either way; this node pair is CPU-only.

    ros2 launch demo_bringup perception.launch.py use_sim_time:=true

ML3.5 F1 decomposition of learn.launch.py. Wraps
demo_perception/perception.launch.py and adds nothing to it — demo_perception
itself is not touched by any ML3.5 phase (rule 6).

Like nav.launch.py, this gates on /clock advancing instead of carrying over
learn.launch.py's t=20s timer. The reason is the same: across a container
boundary the timer counts from the wrong start. The consequence here is milder
than Nav2's — the stub subscribes with sensor QoS and simply receives nothing
until the bridge is up — but starting with use_sim_time:=true against a clock
that does not yet exist gives every published detection a zero timestamp, and
the costmap silently discards those.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow /clock. True whenever Gazebo drives the demo.',
    )

    assumed_range_arg = DeclareLaunchArgument(
        'assumed_range_m',
        default_value='2.0',
        description=(
            'Ground-plane distance assumed for every detection. A 2D box has '
            'no depth; see detections_to_cloud.py.'
        ),
    )

    clock_timeout_arg = DeclareLaunchArgument(
        'clock_timeout_s',
        default_value='120.0',
        description='How long to wait for /clock before giving up.',
    )

    wait_for_clock = Node(
        package='demo_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        parameters=[{
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Not use_sim_time — this node runs before sim time exists.
            'use_sim_time': False,
        }],
    )

    # A IMAGEM CHEGA COMPRIMIDA E E ABERTA AQUI, DO LADO DE QUEM CONSOME.
    #
    # `sim.launch.py` publica /demo/camera/image_raw/compressed alem do RAW. Este
    # no assina a comprimida e devolve sensor_msgs/Image em
    # /demo/perception/image_in, que e local a esta maquina. O SetRemap abaixo
    # religa o detection_stub a ela.
    #
    # POR QUE O NOME MUDA, E POR QUE ISSO NAO E OPCIONAL: publicar o descomprimido
    # de volta em /demo/camera/image_raw poria DOIS publicadores no mesmo topico
    # do mesmo dominio -- o do bridge no host e este. Isso nao da erro nenhum: o
    # assinante obedece a ultima mensagem que chegou, e o resultado e imagem
    # alternando entre duas fontes sem nada em log. O projeto ja pagou por essa
    # classe de falha em /demo/cmd_vel (ver o docstring de nav_trial.py).
    #
    # O CONTRATO NAO MUDA. demo_perception continua consumindo sensor_msgs/Image
    # e continua sem saber a origem do quadro (regra 6 do CLAUDE.md) -- ele nem e
    # tocado: a religacao e por remap de launch, nao por edicao do no. Quando o
    # TIDL substituir o stub, ele recebe o mesmo tipo no mesmo lugar.
    #
    # CUSTO QUE ISTO ACRESCENTA, DITO EXPLICITAMENTE: decodificar JPEG gasta CPU
    # no modulo, que e justamente o recurso em falta. A aposta e que decodificar
    # 640x480 custa menos que remontar 75 Mbit/s de RAW fragmentado, e ela e
    # MEDIDA, nao assumida. Se a decodificacao comer a economia, o proximo passo e
    # o stub assinar CompressedImage direto e tirar as dimensoes de
    # /demo/camera/camera_info -- ele so usa header, width e height, nunca os
    # pixels (detection_stub.py:91,100,119).
    camera_decompressor = Node(
        package='image_transport',
        executable='republish',
        name='camera_decompressor',
        # Parametros, nao posicionais -- ver a armadilha documentada em
        # sim.launch.py, que custou um ciclo de build. Aqui out_transport vazio
        # nao faz laco (o topico de saida tem outro nome), mas faz coisa pior de
        # achar: o decompressor nao publica nada, o detection_stub fica sem
        # imagem nenhuma, e a percepcao morre em silencio.
        parameters=[{
            'in_transport': 'compressed',
            'out_transport': 'raw',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # `in` carrega o sufixo do transporte e `out` nao -- ver a armadilha
        # documentada em sim.launch.py. Aqui os papeis se invertem em relacao ao
        # compressor, porque aqui o comprimido e a ENTRADA.
        remappings=[
            ('in/compressed', '/demo/camera/image_raw/compressed'),
            ('out', '/demo/perception/image_in'),
        ],
        output='screen',
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_perception'), 'launch', 'perception.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'assumed_range_m': LaunchConfiguration('assumed_range_m'),
        }.items(),
    )

    # SetRemap vale para o escopo do grupo, e SO o include entra nele: o
    # camera_decompressor fica de fora de proposito, senao o proprio `in` dele
    # seria reescrito e ele passaria a assinar a si mesmo.
    perception_group = GroupAction([
        SetRemap(src='/demo/camera/image_raw', dst='/demo/perception/image_in'),
        perception,
    ])

    return LaunchDescription([
        use_sim_time_arg,
        assumed_range_arg,
        clock_timeout_arg,
        wait_for_clock,
        camera_decompressor,
        perception_group,
    ])
