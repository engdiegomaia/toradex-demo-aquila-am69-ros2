"""
Ponte de serviços do Gazebo — o que dá ao cockpit os botões de simulação.

Roda em: workstation x86 SOMENTE, no container `sim`. Expõe como serviços ROS
duas coisas que só existem dentro do processo do simulador:

    /demo/sim/control            ros_gz_interfaces/srv/ControlWorld
    /demo/sim/set_entity_pose    ros_gz_interfaces/srv/SetEntityPose

O primeiro é play/pause/reset. O segundo move as câmeras de cena, e quem o
chama é o `scene_view_controller` — não o navegador; ver o cabeçalho daquele nó.

O SIMULADOR NÃO RODA NO MÓDULO, E ISSO NÃO MUDA AQUI

O botão fica no cockpit, e no M3 o cockpit é servido PELO Aquila. Mas quem
executa o Gazebo continua sendo a workstation x86: o AM69 expõe apenas OpenGL
ES 3.2 e Vulkan 1.2, e o OGRE 2 precisa de OpenGL de desktop (regra 1 do
CLAUDE.md). O que atravessa é a chamada de serviço, pelo grafo ROS, exatamente
como a meta do Nav2 já atravessa hoje. "Iniciar a simulação pelo cockpit" é
suportado; "iniciar a simulação NO módulo" não é, e nenhuma quantidade de
código na UI muda isso.

POR QUE O NOME DO MUNDO É LIDO DO ARQUIVO

Os serviços do Gazebo moram em `/world/<nome>/...`, e `<nome>` é o atributo do
elemento `<world>` — não o nome do arquivo. `quadruped_maze11.sdf` declara
`<world name="quadruped_maze11">`, mas o warehouse de `nav2_minimal_tb4_sim`
declara `<world name='warehouse'>`. Adivinhar pelo nome do arquivo acerta num
caso e erra no outro, e o erro é silencioso: a ponte sobe, anuncia os serviços
ROS, e cada chamada expira em um serviço gz que não existe.

Por isso o nome sai de um parse do SDF, e um arquivo que não abre ou não tem
`<world>` derruba o launch com uma mensagem que diz qual arquivo era — em vez
de entregar botões que não fazem nada.
"""

import xml.etree.ElementTree as ElementTree

from demo_simulation.scenarios import spawn_pose
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Altura de reposição quando a planta não declara `height`.
#
# Só o quadrúpede declara esse argumento; a planta diff-drive nasce com `-z 0.1`
# cravado no `create`. Casar os dois valores importa: repor um diff-drive a
# 0,5 m é uma queda gratuita, e repor um quadrúpede a 0,1 m mete as pernas no
# chão — que é justamente o que `height_arg` existe para evitar.
DEFAULT_RESET_Z = 0.1

# Remapeamentos: o cockpit não deve conhecer o nome do mundo. Se conhecesse,
# trocar de cenário exigiria editar o JavaScript, que é exatamente o que o
# projeto evita ao separar cenário (SIM_ARGS) de código.
CONTROL_SERVICE = '/demo/sim/control'
SET_POSE_SERVICE = '/demo/sim/set_entity_pose'


def world_name_of(path: str) -> str:
    """
    Nome declarado do primeiro `<world>` do SDF.

    Levanta RuntimeError nomeando o arquivo, porque este é o tipo de falha que
    de outra forma vira "o botão não faz nada" três dias depois.
    """
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise RuntimeError(
            f'não consegui ler o mundo {path} para descobrir o nome do mundo '
            f'do Gazebo: {error}'
        ) from error

    world = root.find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(
            f'{path} não declara <world name="...">; sem isso não há como '
            'montar os serviços /world/<nome>/control e /world/<nome>/set_pose'
        )
    return world.get('name')


def _reset_pose(context) -> dict:
    """
    Pose a que o botão de reset devolve o robô.

    Mesma precedência do `create` que o nasceu (ver `ScenarioPose` em
    quadruped.launch.py): o argumento explícito vence, e vazio significa
    "pergunte à tabela do cenário". Sem isso o reset devolveria o robô para a
    origem em qualquer mundo cuja área útil não está na origem — o labirinto é
    esse caso, e o robô reapareceria dentro de uma parede sem erro nenhum.

    `context.launch_configurations` e não `LaunchConfiguration(...).perform`:
    `height` só existe na planta do quadrúpede, e performar um argumento não
    declarado levanta em vez de devolver o default.
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

    # A sintaxe de serviço do parameter_bridge é
    # <serviço>@<srv ROS>[@<req gz>@<rep gz>], e a direção é sempre gz->ROS:
    # o bridge expõe um serviço gz COMO serviço ROS, nunca o contrário.
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
        # A fachada std_srvs. Sobe junto com a ponte porque sem a ponte ela não
        # tem para onde encaminhar, e separá-las só criaria a chance de subir
        # uma sem a outra. Ver o cabeçalho de sim_control_relay.py para por que
        # o navegador não fala ControlWorld direto.
        package='demo_simulation',
        executable='sim_control_relay',
        name='sim_control_relay',
        output='screen',
        # O reset teleporta o robô em vez de resetar o mundo, e por isso precisa
        # saber para ONDE. Ver o cabeçalho de sim_control_relay.py: `reset.all`
        # apaga o robô, medido em 26/08/2026.
        parameters=[_reset_pose(context)],
    )]


def generate_launch_description() -> LaunchDescription:
    # `world` NÃO é declarado aqui: este fragmento é sempre incluído por uma
    # planta que já o declarou, e declarar de novo criaria um segundo default
    # capaz de divergir do da planta.
    return LaunchDescription([OpaqueFunction(function=_bridge)])
