"""
Camaras de cena do cockpit — spawn das duas vistas externas (ML3.5 F3b).

Roda em: x86 workstation SOMENTE, dentro do container `sim`. Sao sensores de
render dentro do processo do Gazebo; nada disso existe no modulo (regra 1 do
CLAUDE.md).

Incluido por quadruped.launch.py e por simulation.launch.py. Nao e um entrypoint
de container e nao aparece em compose: e um fragmento compartilhado, para que o
painel azul do cockpit nao dependa de qual planta o ROBOT_TYPE selecionou. Uma
tela que apaga quando se troca de robo e exatamente o tipo de falha silenciosa
que este projeto ja pagou caro.

    ros2 launch demo_simulation quadruped.launch.py scene_cameras:=false

desliga as duas, para medir o custo de render sem elas.

ENQUADRAMENTO — as contas, e as medidas que as corrigiram

hfov = 1,20 rad e imagem 800x600, entao:

    meia-largura horizontal   tan(0,60)            = 0,684
    meia-largura vertical     tan(0,60) * 600/800  = 0,513

A cobertura de uma vista de topo a altura h e, portanto, 1,368*h por 1,026*h.

  TOPO, default (0, 0, 6,0): cobre 8,2 x 6,2 m centrados na origem.

  A primeira versao usava 12 m, pela conta de cobrir 16,4 x 12,3 m. Medido no
  warehouse.sdf, que e o mundo default do compose: a 12 m a camera esta ACIMA
  das vigas do telhado e a imagem inteira e uma viga laranja atravessando o
  quadro, com o robo escondido atras dela. Nao ha erro em lugar nenhum — so uma
  viga. A 6 m a camera esta abaixo da estrutura e o robo aparece limpo sobre o
  chao. NAO suba esta altura sem olhar a imagem do warehouse.

  Nao e uma vista do mundo inteiro, e isso e deliberado: o warehouse tem cerca
  de 28 x 45 m e enquadra-lo por completo exigiria h ~ 44 m, altura em que o
  robo vira um punhado de pixels. Quem quer o mapa inteiro olha o painel verde.

  ISO, default (-3,0 ; 3,0 ; 2,4): olha para a origem. A direcao (3, -3, -2,4) da
      yaw   = atan2(-3, 3)             = -0,7854
      pitch = atan2(2,4 ; sqrt(3^2+3^2)) = 0,5150
  a 4,87 m de distancia, cobrindo cerca de 6,7 m de largura no plano da origem —
  o robo ocupa fracao util do quadro em vez de ser um ponto branco.

  O quadrante importa. As tres primeiras tentativas partiram de (-x, -y) e todas
  cairam dentro do corredor de prateleiras do warehouse: metade do quadro vira
  estante e o robo fica atras dela. De (-x, +y) a linha de visada esta livre e o
  fundo ainda tem armazem suficiente para a cena nao parecer um estudio vazio.

  MAZE11 — o unico mundo do projeto cuja area util nao esta centrada na origem.
  A bbox do maze11.stl foi lida do binario e multiplicada pela escala 0,002:
  local x[1,017; 12,617] y[-12,594; -0,994]. Com a pose do link (-11,672 ;
  11,649) o labirinto ocupa, no mundo, x[-10,655; 0,945] e y[-0,945; 10,655],
  centro (-4,855 ; 4,855), 11,60 x 11,60 m — o que confere com o cabecalho do
  mundo. O robo nasce em (0,0), o canto inferior direito. O labirinto nao tem
  teto, entao aqui a vista de topo pode subir:

      ros2 launch demo_bringup sim.launch.py \
        world:=$(ros2 pkg prefix demo_simulation)/share/demo_simulation/worlds/quadruped_maze11.sdf \
        scene_top_x:=-4.855 scene_top_y:=4.855 scene_top_z:=13.0 \
        scene_iso_x:=-13.0 scene_iso_y:=-3.0 scene_iso_z:=9.0 \
        scene_iso_pitch:=0.671 scene_iso_yaw:=0.768

  (topo a 13 m cobre 17,8 x 13,3 m, os 11,6 m com margem).

POR QUE OS DOIS `create` NAO SAO ADIADOS POR TIMER
Mesma razao do spawn do robo em quadruped.launch.py: `create` ja repete o
servico de nomes de mundo em vez de falhar, entao comecar cedo custa algumas
linhas de retry no log. O robo tinha um motivo adicional para nao esperar (cai
enquanto ninguem o controla); a camera nao cai, mas tambem nao ganha nada
esperando.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Defaults por camara: (x, y, z, pitch, yaw). Roll e sempre 0 — uma camara de
# cena inclinada lateralmente nao tem uso e so confunde quem compara com o mapa.
#
# O pitch de 1,5708 da vista de topo aponta o +X da camara para baixo; o yaw de
# 1,5708 poe o +Y do mundo para cima na imagem, a mesma orientacao do canvas do
# painel verde. Sem o yaw as duas telas discordam sobre onde e o norte.
DEFAULTS = {
    'iso': ('-3.0', '3.0', '2.4', '0.5150', '-0.7854'),
    'top': ('0.0', '0.0', '6.0', '1.5708', '1.5708'),
}


def _pose_args(name: str) -> list:
    x, y, z, pitch, yaw = DEFAULTS[name]
    return [
        DeclareLaunchArgument(f'scene_{name}_x', default_value=x),
        DeclareLaunchArgument(f'scene_{name}_y', default_value=y),
        DeclareLaunchArgument(f'scene_{name}_z', default_value=z),
        DeclareLaunchArgument(f'scene_{name}_pitch', default_value=pitch),
        DeclareLaunchArgument(f'scene_{name}_yaw', default_value=yaw),
    ]


def _spawn(name: str) -> Node:
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
            '-x', LaunchConfiguration(f'scene_{name}_x'),
            '-y', LaunchConfiguration(f'scene_{name}_y'),
            '-z', LaunchConfiguration(f'scene_{name}_z'),
            '-P', LaunchConfiguration(f'scene_{name}_pitch'),
            '-Y', LaunchConfiguration(f'scene_{name}_yaw'),
        ],
    )


def _view_controller() -> Node:
    """
    O nó que os botões de vista do cockpit movem.

    Os parâmetros saem dos MESMOS argumentos que posicionaram as câmeras acima.
    Essa é a razão de ele morar neste arquivo e não em sim_control.launch.py: o
    enquadramento inicial é declarado uma vez só, e o controlador nasce sabendo
    de onde a câmera partiu — inclusive quando SIM_ARGS reenquadrou tudo para
    outro cenário.

    ParameterValue com value_type=float não é cerimônia: LaunchConfiguration
    entrega STRING, o nó declara os parâmetros como double, e a incompatibilidade
    derruba o nó com "Wrong parameter type" na ativação.
    """
    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return Node(
        package='demo_simulation',
        executable='scene_view_controller',
        name='scene_view_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('scene_cameras')),
        parameters=[{
            'iso_x': as_float('scene_iso_x'),
            'iso_y': as_float('scene_iso_y'),
            'iso_z': as_float('scene_iso_z'),
            'iso_pitch': as_float('scene_iso_pitch'),
            'iso_yaw': as_float('scene_iso_yaw'),
            'top_x': as_float('scene_top_x'),
            'top_y': as_float('scene_top_y'),
            'top_z': as_float('scene_top_z'),
            'top_pitch': as_float('scene_top_pitch'),
            'top_yaw': as_float('scene_top_yaw'),
            'use_sim_time': True,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    enabled_arg = DeclareLaunchArgument(
        'scene_cameras',
        default_value='true',
        description='Spawn the cockpit scene cameras (blue panel). Set false '
                    'to run the plant without the extra render cost.',
    )

    return LaunchDescription(
        [enabled_arg]
        + _pose_args('iso')
        + _pose_args('top')
        + [_spawn('iso'), _spawn('top'), _view_controller()]
    )
