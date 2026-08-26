"""
Fachada de controle da navegacao — o "resetar meta" do cockpit.

Roda em: Aquila AM69 (arm64) no modo hil, workstation x86 no modo learn. Sempre
no MESMO container que o Nav2, porque o que ela faz e mexer no ciclo de vida
dele.

Fragmento COMPARTILHADO, includado por:

    demo_navigation/navigation.launch.py   caminho de mapa estatico + AMCL
                                           (nav.launch.py e learn.launch.py)
    demo_bringup/nav_quadruped.launch.py   caminho reativo do Go2

Nao e entrypoint de container e nao aparece em compose — mesmo papel que
demo_simulation/launch/scene_cameras.launch.py cumpre do lado do simulador.

POR QUE UM FRAGMENTO EM VEZ DE DUAS COPIAS DO Node

Os dois caminhos de navegacao sao deliberadamente separados
(nav_quadruped.launch.py explica por que), mas esta fachada e identica nos dois:
ela fala com `lifecycle_manager_navigation`, que os dois sobem. Duplicar o bloco
`Node` seria duas coisas para manter em sincronia, e o modo como isso quebra e o
pior possivel — o botao do cockpit funciona num ROBOT_TYPE e nao no outro, sem
erro em lugar nenhum.

    ros2 launch ... nav_control:=false

desliga a fachada, para subir a navegacao sem nenhum servico de reinicio exposto.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enabled_arg = DeclareLaunchArgument(
        'nav_control',
        default_value='true',
        description='Expor /demo/nav/{reset,cancel} para o cockpit.',
    )

    # O argumento use_sim_time FOI REMOVIDO em 25/08/2026, de proposito.
    #
    # Manter um argumento declarado que ninguem consome e falha silenciosa: quem
    # passasse use_sim_time:=true veria o valor aceito e ignorado, sem uma linha
    # de log. Removido, a mesma chamada falha ALTO -- "is not a valid launch
    # argument" -- e quem chamou descobre na hora, nao numa medicao de CPU.
    #
    # Por que ninguem consome: ver o bloco no `relay` abaixo.

    relay = Node(
        package='demo_navigation',
        executable='nav_control_relay',
        name='nav_control_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav_control')),
        # use_sim_time NAO, e a inversao e de 25/08/2026.
        #
        # O comentario anterior dizia "SIM, para que o no viva no mesmo tempo que
        # o resto da pilha" -- e admitia na frase seguinte que os timeouts internos
        # NAO usam esse relogio (o docstring de `_wait` explica: um Nav2 desativado
        # no meio de um RESET coexiste com um /clock parado, e medir timeout ali
        # seria esperar para sempre). Isso continua verdade, e o resto tambem:
        # este no nao tem UMA chamada a get_clock(), nem timer, nem stamp. Viver
        # "no mesmo tempo" nao comprava nada, porque ele nunca pergunta as horas.
        #
        # O que comprava era custo: 35% de um nucleo no AM69 so recebendo /clock a
        # ~870 Hz. Ver o bloco PISO OCIOSO DE CPU em
        # demo_bringup/launch/nav_quadruped.launch.py para a medicao completa.
        #
        # Vale para os DOIS robos, porque este launch e compartilhado. Nao ha
        # caminho por modo aqui, e nao deve haver.
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([enabled_arg, relay])
