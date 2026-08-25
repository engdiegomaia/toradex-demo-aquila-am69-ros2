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

O ENQUADRAMENTO E DERIVADO DO MUNDO, E ISSO E O DESENHO

Os dez numeros das duas cameras saem de `demo_simulation/scenarios.py`, indexados
pelo arquivo de mundo. Nao ha default de enquadramento aqui: escolher o mundo JA
escolhe o enquadramento.

A razao esta no cabecalho daquele arquivo, e vale repetir a falha que ela remove:
trocar de mundo sem trocar o enquadramento aponta a camera para chao vazio, sem
erro e sem log, e quem olha o painel azul conclui que o robo nao esta andando.
Enquanto os dois eram argumentos independentes, a combinacao errada era a mais
facil de produzir -- bastava esquecer metade de um SIM_ARGS de seis linhas.

Todo `scene_*` aceita valor explicito, que vence a tabela. O default e VAZIO, que
significa "derive do mundo" -- e nao um numero, porque um numero aqui e
indistinguivel de uma escolha do operador.

    # sondar o labirinto de mais alto, mantendo os outros nove numeros
    ros2 launch demo_bringup sim.launch.py scene_top_z:=20.0

ENQUADRAMENTO — as contas, e as medidas que as corrigiram

hfov = 1,20 rad e imagem 800x600, entao:

    meia-largura horizontal   tan(0,60)            = 0,684
    meia-largura vertical     tan(0,60) * 600/800  = 0,513

A cobertura de uma vista de topo a altura h e, portanto, 1,368*h por 1,026*h.

  ARMAZEM, topo (0, 0, 6,0): cobre 8,2 x 6,2 m centrados na origem.

  A primeira versao usava 12 m, pela conta de cobrir 16,4 x 12,3 m. Medido no
  warehouse.sdf: a 12 m a camera esta ACIMA das vigas do telhado e a imagem
  inteira e uma viga laranja atravessando o quadro, com o robo escondido atras
  dela. Nao ha erro em lugar nenhum — so uma viga. A 6 m a camera esta abaixo da
  estrutura e o robo aparece limpo sobre o chao. NAO suba esta altura sem olhar a
  imagem do warehouse.

  Nao e uma vista do mundo inteiro, e isso e deliberado: o warehouse tem cerca
  de 28 x 45 m e enquadra-lo por completo exigiria h ~ 44 m, altura em que o
  robo vira um punhado de pixels. Quem quer o mapa inteiro olha o painel verde.

  ARMAZEM, iso (-3,0 ; 3,0 ; 2,4): olha para a origem. A direcao (3, -3, -2,4) da
      yaw   = atan2(-3, 3)             = -0,7854
      pitch = atan2(2,4 ; sqrt(3^2+3^2)) = 0,5150
  a 4,87 m de distancia, cobrindo cerca de 6,7 m de largura no plano da origem —
  o robo ocupa fracao util do quadro em vez de ser um ponto branco.

  O quadrante importa. As tres primeiras tentativas partiram de (-x, -y) e todas
  cairam dentro do corredor de prateleiras do warehouse: metade do quadro vira
  estante e o robo fica atras dela. De (-x, +y) a linha de visada esta livre e o
  fundo ainda tem armazem suficiente para a cena nao parecer um estudio vazio.

  MAZE11 — o cenario OFICIAL, e o unico mundo do projeto cuja area util nao esta
  centrada na origem. A bbox do maze11.stl foi lida do binario e multiplicada
  pela escala 0,002: local x[1,017; 12,617] y[-12,594; -0,994]. Com a pose do
  link (-11,672 ; 11,649) o labirinto ocupa, no mundo, x[-10,655; 0,945] e
  y[-0,945; 10,655], centro (-4,855 ; 4,855), 11,60 x 11,60 m — o que confere com
  o cabecalho do mundo. O robo nasce em (0,0), o canto inferior direito. O
  labirinto nao tem teto, entao aqui a vista de topo pode subir: 13 m cobrem
  17,8 x 13,3 m, os 11,6 m com margem.

SEGUIR O ROBO, E O SEED odom -> MUNDO

As duas vistas seguem o robo por default (`follow:=false` desliga). O
scene_view_controller usa /demo/odom como pose do robo, e ele precisa dessa pose
no referencial do MUNDO, que e o unico que o set_pose do Gazebo entende.

As duas plantas divergem nisso:

  quadrupede   /go2/odom e ground truth do gz-sim-odometry-publisher-system:
               ja e a pose no mundo. Seed = 0, e quadruped.launch.py nao passa
               nada.

  diff-drive   /odom e integrado dos encoders pelo plugin DiffDrive, com origem
               na pose de SPAWN. Seed = x/y/yaw do spawn, e simulation.launch.py
               liga os tres explicitamente.

Os argumentos `follow_offset_*` existem para isso. Eles NAO sao lidos de `x`,
`y` e `yaw` aqui dentro por acidente de escopo: as duas plantas declaram esses
tres nomes e o include os herdaria dos dois, o que faria a camera do quadrupede
seguir um fantasma deslocado pela pose de spawn (no maze11, deslocado E girado
de 90 graus). Quem sabe se a odometria e ground truth e a planta, e e ela que
passa.

POR QUE OS DOIS `create` NAO SAO ADIADOS POR TIMER
Mesma razao do spawn do robo em quadruped.launch.py: `create` ja repete o
servico de nomes de mundo em vez de falhar, entao comecar cedo custa algumas
linhas de retry no log. O robo tinha um motivo adicional para nao esperar (cai
enquanto ninguem o controla); a camera nao cai, mas tambem nao ganha nada
esperando.
"""

from demo_simulation.scenarios import camera_pose
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Ordem dos cinco numeros de uma pose de camera em scenarios.py.
POSE_FIELDS = ('x', 'y', 'z', 'pitch', 'yaw')


def _pose_args(name: str) -> list:
    """
    Declara os cinco argumentos de uma camera, todos VAZIOS por default.

    Vazio significa "derive do mundo". Nao ha numero aqui de proposito: um
    default numerico e indistinguivel de um valor que o operador escolheu, e foi
    assim que o enquadramento do armazem sobreviveu a troca para o labirinto.
    """
    return [
        DeclareLaunchArgument(
            f'scene_{name}_{field}',
            default_value='',
            description=f'{field} da camara {name}. Vazio = pega de '
                        'scenarios.py pelo mundo selecionado.',
        )
        for field in POSE_FIELDS
    ]


def _resolve(context, name: str) -> dict:
    """Pose efetiva da camera: o que foi passado, senao a tabela do mundo."""
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
    Parametros do scene_view_controller: as dez poses mais o seguimento.

    Publica (sem underscore) para que o teste de launch possa afirmar sobre o
    dicionario direto. Alcançar `Node._Node__parameters` do lado do teste nao
    serve: o Node normaliza chaves e valores para tuplas de TextSubstitution, e
    uma assercao sobre isso quebra quando o `launch` muda de versao, sem que nada
    do projeto tenha mudado.
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
    O no que os botoes de vista do cockpit movem.

    Os parametros saem das MESMAS poses que posicionaram as cameras acima. Essa e
    a razao de ele morar neste arquivo e nao em sim_control.launch.py: o
    enquadramento inicial e resolvido uma vez so, e o controlador nasce sabendo
    de onde a camera partiu — inclusive quando o cenario reenquadrou tudo.

    As poses ja estao RESOLVIDAS aqui (o OpaqueFunction as leu do cenario ou do
    argumento), entao elas viram float em Python e nao passam por ParameterValue.
    ParameterValue(value_type=float) recebendo uma str simples falha na montagem
    com "value='-13.0' is not an instance of <class 'float'>" -- ele converte
    SUBSTITUICAO para float, nao texto para float. Medido em 25/08/2026.

    Os tres de baixo continuam sendo substituicao, porque `follow` e os
    `follow_offset_*` chegam como LaunchConfiguration: ali o conversor e
    necessario, ou o no cai com "Wrong parameter type" na ativacao, ja que ele
    declara os offsets como double.
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
    """Resolve o enquadramento e devolve os tres nos que dependem dele."""
    iso = _resolve(context, 'iso')
    top = _resolve(context, 'top')
    return [_spawn('iso', iso), _spawn('top', top), _view_controller(iso, top)]


def _follow_args() -> list:
    return [
        DeclareLaunchArgument(
            'follow',
            default_value='true',
            description='Vistas de cena seguem o robo. false congela as duas '
                        'no enquadramento do cenario (a vista larga).',
        ),
        DeclareLaunchArgument(
            'follow_offset_x',
            default_value='0.0',
            description='odom -> mundo, X. Zero para odometria ground truth '
                        '(quadrupede); a pose de spawn para o diff-drive.',
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

    # Declarado aqui tambem, e nao só herdado da planta: este fragmento LE o
    # mundo para resolver o enquadramento, e um argumento lido sem ser declarado
    # explode com "LaunchConfiguration not found" em vez de dizer o que falta.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Mundo selecionado. Vazio cai no enquadramento generico '
                    '(cameras olhando para a origem).',
    )

    return LaunchDescription(
        [enabled_arg, world_arg]
        + _pose_args('iso')
        + _pose_args('top')
        + _follow_args()
        + [OpaqueFunction(function=_cameras)]
    )
