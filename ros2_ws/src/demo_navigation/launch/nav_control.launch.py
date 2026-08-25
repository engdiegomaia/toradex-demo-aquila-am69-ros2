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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Seguir /clock. Verdadeiro sempre que o Gazebo comanda.',
    )

    relay = Node(
        package='demo_navigation',
        executable='nav_control_relay',
        name='nav_control_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav_control')),
        # use_sim_time SIM, para que o no viva no mesmo tempo que o resto da
        # pilha. Os timeouts internos dele NAO usam esse relogio, e a razao esta
        # no docstring de `_wait`: um Nav2 desativado no meio de um RESET
        # coexiste com um /clock parado, e medir timeout ali seria esperar para
        # sempre.
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([enabled_arg, use_sim_time_arg, relay])
