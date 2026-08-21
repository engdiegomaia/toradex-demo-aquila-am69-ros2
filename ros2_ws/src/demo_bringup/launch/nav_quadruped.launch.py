"""
Nav2 para o quadrupede: desvio reativo, sem mapa e sem AMCL.

Roda na estacao x86 (amd64). O Nav2 em si nao tem dependencia grafica e no modo
hil vai para o Aquila AM69, mas a imagem `demo-sim:spike-go2` usada hoje NAO tem
Nav2 dentro (verificado: zero pacotes nav2 em /opt/ros/jazzy/lib). Enquanto essa
imagem for a do spike, este launch sobe nativo no host e conversa com o
simulador pelo DDS -- dominio 69, mesma rede.

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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        default_value='true',
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        clock_timeout_arg,
        map_identity_arg,
        wait_for_clock,
        odom_tf,
        cmd_vel_adapter,
        nav2_container,
        navigation,
    ])
