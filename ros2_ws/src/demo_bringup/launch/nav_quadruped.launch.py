"""
Nav2 para o quadrupede com mapa vivo persistido por slam_toolbox.

Roda na estacao x86 (amd64) em learn e no Aquila AM69 (arm64) em HIL. O Nav2 nao
tem dependencia grafica; Gazebo e RViz continuam exclusivamente no host. A
execucao arm64 composta no modulo foi medida em 21/08/2026; ver
docs/results/ml35-hil-aquila.md.

    ros2 launch demo_bringup nav_quadruped.launch.py

## Por que este arquivo existe em vez de um argumento em nav.launch.py

`nav.launch.py` navega sobre MAPA ESTATICO: ele inclui `bringup_launch.py`, que
carrega map_server e AMCL, e passa `nav2_params.yaml`, que e do TurtleBot 4.
Este arquivo navega sem mapa nenhum. As duas coisas divergem em quatro pontos
que nao sao um parametro:

- a pilha incluida e `navigation_launch.py`, sem localizacao nem map_server;
- quem publica `map -> odom` e o `odom_tf`, com identidade, e nao o AMCL;
- o costmap global e ROLANTE e sem `static_layer`;
- o arquivo de parametros e `nav2_params_go2.yaml`.

Um so launch com condicionais para cobrir os dois casos e exatamente o que
CLAUDE.md proibe ("No single launch file full of conditionals"), e aqui a
proibicao tem conteudo: metade dos erros deste caminho e subir a combinacao
errada dos quatro pontos acima, e um `if` esconde qual combinacao esta ativa.

## O QUE NAO PODE RODAR JUNTO

`demo_routine`. Os dois publicam `/demo/cmd_vel` -- o Nav2 pelo
`collision_monitor`, a rotina direto. Dois publicadores no mesmo topico nao dao
erro: o `twist_to_inputs` recebe as duas mensagens e obedece a ultima que
chegou, alternando entre o desvio e a coreografia a 20 Hz. O robo anda em
espasmos e nada no log diz por que. Escolha um.

Para mover o robo sob Nav2 use `patrol_commander`, que manda METAS e nao
velocidades.

## A armadilha do mapa vazio

Sem `static_layer` e com `track_unknown_space: true`, tudo fora do alcance do
lidar e desconhecido, e o `NavfnPlanner` planeja atraves do desconhecido porque
`allow_unknown: true`. Isso e proposital -- e o que permite pedir uma meta a
5 m sem mapa. A consequencia e que o Nav2 ACEITA meta fora da janela rolante de
20 m e depois falha ao chegar perto da borda. Mantenha as metas dentro de ~8 m
da origem; `patrol_commander` ja faz isso.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_navigation'), 'config',
            'nav2_params_go2.yaml',
        ]),
        description=(
            'Arquivo de parametros. O default e o do Go2; nav2_params.yaml e do '
            'TurtleBot 4 e NAO serve aqui -- ver a lista de deltas no cabecalho '
            'dele.'
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Seguir /clock. Verdadeiro sempre que o Gazebo comanda.',
    )

    clock_timeout_arg = DeclareLaunchArgument(
        'clock_timeout_s',
        default_value='120.0',
        description='Quanto esperar por /clock antes de desistir.',
    )

    # Verdadeiro no caminho reativo, que e o que este launch faz. Falso se voce
    # subir AMCL ou SLAM: os dois publicam `map -> odom`, e dois publicadores na
    # mesma aresta da TF nao dao erro -- o consumidor alterna entre as duas
    # crencas. Ver o cabecalho de odom_tf.py.
    map_identity_arg = DeclareLaunchArgument(
        'publish_map_identity',
        default_value='false',
        description=(
            'Publicar map -> odom como identidade. Ponha false se subir AMCL '
            'ou SLAM, ou havera dois publicadores nessa aresta.'
        ),
    )

    wait_for_clock = Node(
        package='demo_bringup',
        executable='wait_for_clock',
        name='wait_for_clock',
        output='screen',
        parameters=[{
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Deliberadamente NAO use_sim_time: a funcao deste no e rodar antes
            # de existir tempo de simulacao.
            'use_sim_time': False,
        }],
    )

    # Fecha `odom -> base` e, opcionalmente, `map -> odom`. Sem isso o Nav2 nao
    # coloca o scan em costmap nenhum e falha sem citar TF.
    odom_tf = Node(
        package='demo_bringup',
        executable='odom_tf',
        name='odom_tf',
        output='screen',
        parameters=[{
            'base_frame': 'base',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'publish_map_identity': LaunchConfiguration('publish_map_identity'),
            # NAO segue LaunchConfiguration('use_sim_time'), e isso e deliberado.
            # Ver o bloco PISO OCIOSO DE CPU abaixo do cmd_vel_adapter.
            #
            # O caminho quente deste no (`_on_odom`) COPIA o stamp da mensagem de
            # odometria -- o cabecalho de odom_tf.py explica por que, e continua
            # valendo. A unica chamada a get_clock() esta em `_identity()`, que
            # carimba a aresta map -> odom publicada em /tf_static UMA vez. O
            # buffer estatico do tf2 devolve transformada estatica para qualquer
            # instante consultado: o stamp dela nao entra em lookup nenhum.
            #
            # Ou seja: nada que este no publica muda de valor por causa desta
            # linha. O que muda e ele parar de receber ~870 mensagens de /clock
            # por segundo para nao usar nenhuma.
            'use_sim_time': False,
        }],
    )

    # Fronteira de unidades. O Nav2 publica SI em /demo/cmd_vel_si e este no
    # converte para o manche de /demo/cmd_vel. Sem ele o robo anda a 40% do
    # pedido e nada acusa -- ver o cabecalho de cmd_vel_si_to_stick.py.
    cmd_vel_adapter = Node(
        package='demo_bringup',
        executable='cmd_vel_si_to_stick',
        name='cmd_vel_si_to_stick',
        output='screen',
        # ================= PISO OCIOSO DE CPU DO MODULO =================
        #
        # Este no converte Twist em Twist. Nao tem header, nao tem timer, nao
        # chama get_clock() em lugar nenhum -- verificavel por grep, e ha teste
        # que trava isso. Com use_sim_time: true ele assinava /clock assim mesmo,
        # porque quem cria a assinatura e o rclpy, nao o codigo do no.
        #
        # MEDIDO NO AQUILA AM69 EM 25/08/2026, pilha de pe e SEM META ATIVA,
        # amostrando /proc/<tid>/stat por thread dentro do container `nav`:
        #
        #   piso ocioso total          367% de 800%
        #   component_container (Nav2) 215%
        #   odom_tf                     40%   <- republicador trivial
        #   recvUC (recepcao Cyclone)   38%
        #   cmd_vel_si_to_stick         36%   <- este no
        #   nav_control_rel             35%   <- relay, tambem sem relogio
        #
        # Tres republicadores em Python gastando 111% de 800% -- 14% da maquina
        # -- com o robo PARADO. O trabalho util deles cabe em ~1%; o resto e
        # entrega de /clock a ~870 Hz, que o Gazebo publica nessa taxa porque o
        # passo de fisica da marcha e 1 ms.
        #
        # Isto NAO e o estrangulamento de /clock que foi ensaiado e REPROVADO em
        # 21/08 (ver demo_simulation/clock_throttle.py): la a taxa caia para
        # TODO mundo, inclusive para o MPPI, e a navegacao morreu. Aqui a taxa
        # nao muda para ninguem. Muda quem assina -- e sao tres nos que nao
        # tinham o que fazer com a mensagem.
        parameters=[{'use_sim_time': False}],
    )

    # Canal operacional para o cockpit. Fica no target junto do Nav2: em HIL os
    # recursos e a temperatura exibidos são do Aquila, e os logs de eixos vêm da
    # fronteira real Nav2(SI) -> manche, não do /rosout bruto misturado ao host.
    target_monitor = Node(
        package='demo_bringup',
        executable='target_monitor',
        name='target_monitor',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # O lidar do Go2 tem 16 aneis. O LaserScan de um anel publicado pelo bridge
    # nao ve os obstaculos do maze; o SLAM precisa da nuvem completa achatada.
    # Transformar para `base` tambem torna os cortes de altura relativos ao
    # robo, descartando o piso sem depender da pose no mundo.
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/demo/scan_cloud'),
            ('scan', '/demo/scan_slam'),
        ],
        parameters=[{
            'target_frame': 'base',
            'transform_tolerance': 0.10,
            'min_height': 0.12,
            'max_height': 1.00,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.008726646259972,
            'scan_time': 0.10,
            'range_min': 0.10,
            'range_max': 9.00,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Nao deixe este include herdar `params_file` do launch pai. Neste arquivo,
    # esse argumento e o YAML do Nav2 (`nav2_params_go2.yaml`), enquanto o
    # slam_toolbox exige seu proprio namespace e seus frames em
    # `slam_params.yaml`. LaunchConfiguration tem escopo compartilhado entre
    # includes; sem a passagem explicita abaixo o SLAM recebe silenciosamente o
    # arquivo do Nav2 e volta ao default `base_footprint`.
    slam_params = PathJoinSubstitution([
        FindPackageShare('demo_navigation'), 'config', 'slam_params.yaml',
    ])
    slam = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('demo_navigation'), 'launch', 'slam.launch.py',
            ])),
            launch_arguments={
                'params_file': slam_params,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': '/demo/scan_slam',
            }.items(),
        )],
    )

    # O caminho da arvore de comportamento tem de ser ABSOLUTO, e nao pode ficar
    # cravado no YAML: quem sabe o prefixo de instalacao e o FindPackageShare.
    #
    # A reescrita e feita AQUI, e nao em `navigation_launch.py`, porque aquele
    # arquivo e copia vendorizada do Nav2 e a proveniencia depende de ele seguir
    # identico ao upstream (ver launch/nav2_vendored/README.md). Reescrevemos
    # antes e passamos o resultado como params_file; o navigation_launch faz a
    # propria reescrita de `autostart` sobre este arquivo, o que compoe bem.
    smoothed_bt = PathJoinSubstitution([
        FindPackageShare('demo_navigation'),
        'behavior_trees', 'nav_to_pose_smoothed.xml',
    ])
    exploration_bt = PathJoinSubstitution([
        FindPackageShare('demo_navigation'),
        'behavior_trees', 'nav_to_pose_exploration.xml',
    ])
    maze_explorer = Node(
        package='demo_navigation',
        executable='maze_explorer',
        name='maze_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'exploration_bt_xml': exploration_bt,
        }],
    )
    # RewrittenYaml JA e uma substituicao que resolve para o caminho do arquivo
    # reescrito, entao vai direto em launch_arguments. Envolver em ParameterFile
    # aqui nao funciona: launch_arguments aceita string ou substituicao, e
    # ParameterFile nao e nem um nem outro.
    params_with_bt = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': smoothed_bt},
        convert_types=True,
    )

    # `navigation_launch.py` e nao `bringup_launch.py`: o segundo arrasta
    # map_server e AMCL, que e o caminho do mapa estatico.
    #
    # O container que `navigation_launch.py` espera e nao cria. O nome
    # `nav2_container` casa com o default do argumento `container_name` de la; se
    # um dos dois mudar, os nos sao carregados em lugar nenhum, sem erro.
    #
    # `component_container_isolated` e nao `component_container`: cada no ganha
    # seu proprio executor de thread unica dentro do processo. Um executor
    # compartilhado deixaria um callback longo do MPPI atrasar o heartbeat do
    # `lifecycle_manager`, e o sintoma seria o gerenciador declarando os nos
    # mortos no meio da navegacao.
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_with_bt,
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'autostart': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    # COMPOSICAO LIGADA, e o container e criado LOGO ACIMA (`nav2_container`).
    #
    # `navigation_launch.py` com composicao usa LoadComposableNodes para carregar
    # os servidores dentro de `/nav2_container`, mas NAO cria esse container --
    # quem o cria upstream e `bringup_launch.py`, que e justamente o arquivo que
    # nao estamos incluindo. Ligar `use_composition` sem criar o container e uma
    # falha SILENCIOSA: os nos sao carregados num container que ninguem criou,
    # NADA sobe e NADA imprime erro. Medido em 20/08/2026 -- o log do launch
    # termina com `wait_for_clock` e `odom_tf` e mais nada, e `ros2 action list`
    # nunca mostra navigate_to_pose. O sintoma visivel e "o Nav2 nao ativou", que
    # nao aponta para este parametro. Se voce mexer no bloco `nav2_container`,
    # este e o modo como isso quebra.
    #
    # POR QUE COMPOR, medido no Aquila AM69 em 21/08/2026, modo hil:
    #
    #   13 processos separados     container demo-nav a 470% de CPU (de 800%)
    #                              odom_tf e cmd_vel_si_to_stick, republicadores
    #                              triviais em Python, a ~87% de um nucleo cada
    #
    # O custo nao esta no algoritmo, esta na multiplicacao: `use_sim_time` faz
    # CADA no assinar `/clock`, que o Gazebo publica a ~880 Hz porque o passo de
    # fisica da marcha e 1 ms. Treze assinantes x 880 Hz = ~11 mil entregas por
    # segundo entre processos, num Cortex-A72.
    #
    # Composto, os servidores dividem UM processo: uma assinatura de `/clock` em
    # vez de treze, e comunicacao intraprocesso em vez de DDS para os topicos
    # internos. Ataca o mesmo custo sem tocar no relogio que os algoritmos veem.
    #
    # A ALTERNATIVA QUE FOI TENTADA E REJEITADA: estrangular o `/clock` a 100 Hz
    # (demo_simulation/clock_throttle.py). Baixou a CPU do container de 470% para
    # 324%, e a navegacao PAROU -- 0.0039 m/s contra 0.0251 m/s, com `cmd_vx` de
    # pico caindo de 0.138 para 0.003. Economia de CPU que nao compra nada nao e
    # otimizacao. Veja docs/results/ml35-hil-aquila.md para o A/B.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'),
            'launch', 'nav2_vendored', 'navigation_launch.py',
        ])),
        launch_arguments={
            'params_file': params_with_bt,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items(),
    )

    # Fachada std_srvs do "resetar meta" do cockpit: cancela a meta, limpa os
    # costmaps e recicla por PAUSE/RESUME o `lifecycle_manager_navigation` que o
    # include acima sobe. RESET + STARTUP seria o obvio e mata o container --
    # o cabecalho de nav_control_relay.py tem a medicao. Fragmento
    # compartilhado com o caminho de mapa estatico, por isso e um include —
    # o cabecalho de nav_control.launch.py diz por que duplicar o Node seria
    # pior. Depois do `navigation` na lista de proposito: o relay tolera o
    # gerenciador ainda nao existir (`wait_for_service`), mas a ordem de leitura
    # deve dizer quem depende de quem.
    nav_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('demo_navigation'),
            'launch', 'nav_control.launch.py',
        ])),
        # Sem launch_arguments: o relay nao declara mais use_sim_time, porque
        # nao chama o relogio. Passar aqui agora e erro de launch, e essa e a
        # intencao -- ver nav_control.launch.py.
    )

    # Ultimo elo antes do Nav2, e o que faltava: a aresta odom -> base so
    # existe quando a PRIMEIRA /demo/odom atravessa a fronteira de container.
    # Sem este portao o Nav2 aposta na velocidade de descoberta do DDS -- e em
    # 26/08 a aposta perdeu, o local_costmap nao ativou em 60 s e o gerenciador
    # ABORTOU o bringup em definitivo. Ver o cabecalho de wait_for_tf.py.
    wait_for_tf = Node(
        package='demo_bringup',
        executable='wait_for_tf',
        name='wait_for_tf',
        output='screen',
        parameters=[{
            'parent_frame': 'odom',
            'child_frame': 'base',
            'timeout_s': LaunchConfiguration('clock_timeout_s'),
            # Consulta com Time() (instante comum mais recente), que nao usa o
            # relogio do no -- entao ele nao precisa assinar /clock.
            'use_sim_time': False,
        }],
    )

    # Falha ALTO. Emitir o Nav2 mesmo assim reproduz exatamente o defeito que
    # este portao existe para impedir, e o sintoma seria de novo "meta recusada"
    # sem ninguem citar TF nem relogio.
    def _gate(following, what):
        def _on_exit(event, context):
            if event.returncode == 0:
                return following
            return [
                LogInfo(msg=f'[nav_quadruped] {what} falhou (codigo '
                            f'{event.returncode}). Nav2 NAO sera iniciado.'),
                Shutdown(reason=f'{what} nao satisfeito'),
            ]
        return _on_exit

    # Cadeia de subida, cada elo condicionado ao anterior TERMINAR e nao a tempo
    # decorrido -- mesma disciplina de quadruped.launch.py, que este arquivo nao
    # seguia. Antes de 26/08/2026 os cinco nos abaixo eram emitidos JUNTOS, e
    # `wait_for_clock` nao condicionava coisa alguma apesar do que o docstring
    # dele promete. Era corrida, e ela foi perdida no AM69.
    #
    #   wait_for_clock  ->  wait_for_tf  ->  Nav2
    #
    # `odom_tf` e `cmd_vel_adapter` sobem de imediato, de proposito: e o odom_tf
    # que PRODUZ a aresta que o wait_for_tf espera.
    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        clock_timeout_arg,
        map_identity_arg,
        odom_tf,
        cmd_vel_adapter,
        target_monitor,
        wait_for_clock,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=wait_for_clock,
            on_exit=_gate([wait_for_tf], 'wait_for_clock'),
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=wait_for_tf,
            on_exit=_gate([
                cloud_to_scan, slam, nav2_container, navigation, nav_control,
                maze_explorer,
            ],
                          'wait_for_tf'),
        )),
    ])
