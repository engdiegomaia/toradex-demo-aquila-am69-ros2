"""
Fachada std_srvs para reiniciar a navegacao — o "resetar meta" do cockpit.

Roda em: Aquila AM69 (arm64) no modo hil, workstation x86 no modo learn. Ele
mora do lado do Nav2, no MESMO container, e e por isso que reiniciar a
navegacao "no Aquila" e uma chamada de servico e nao um acesso SSH ao modulo.

    /demo/nav/reset   std_srvs/Trigger   (entra)
    /demo/nav/cancel  std_srvs/Trigger   (entra)
    /navigate_to_pose/_action/cancel_goal              action_msgs/CancelGoal
    /{global,local}_costmap/clear_entirely_*_costmap   nav2_msgs/ClearEntireCostmap
    /lifecycle_manager_navigation/manage_nodes         nav2_msgs/ManageLifecycleNodes

O QUE "RESETAR" FAZ, E POR QUE NAO E SO CANCELAR A META

Cancelar a meta para o robo e deixa tudo o mais no lugar: o costmap acumulado, os
servidores no estado em que estavam. E o suficiente quando o operador so mudou de
ideia — e para esse caso existe o `cancelar meta` do painel verde, que fala
direto com a acao.

Nao e suficiente no caso que motivou este no: o Nav2 preso. No HIL de 24/08 as
duas metas de 8 m expiraram no protocolo de 420/200 s, com o robo gastando o
tempo em recuperacoes sobre um costmap sujo. Ali o que se quer e a pilha limpa, e
o que este no faz, em ordem, e:

    1. CANCELAR    toda meta ativa de navigate_to_pose;
    2. LIMPAR      os dois costmaps por inteiro, o global e o local;
    3. PAUSE       desativa todos os nos gerenciados, em ordem inversa;
    4. RESUME      reativa todos, na ordem certa.

Depois disso o robo esta parado, o costmap nao carrega mais nenhum obstaculo
fantasma, o bt_navigator perdeu qualquer estado interno preso e o MPPI foi
reinicializado.

Medido em 24/08/2026 na workstation, com meta ativa: 6,6 s de ponta a ponta
(PAUSE 2,4 s, RESUME 2,4 s, o resto em cancelamento e limpeza). A meta em curso
terminou com status CANCELED e uma meta nova foi aceita imediatamente depois. No
Cortex-A72 do AM69 isso e mais lento e NAO foi medido (regra 5 do CLAUDE.md).

POR QUE NAO E RESET + STARTUP, QUE SERIA O OBVIO

Porque isso QUEBRA a pilha, de forma reproduzivel. Medido em 24/08/2026, no modo
learn, caminho quadrupede:

    ManageLifecycleNodes RESET (3) seguido de STARTUP (0) mata o processo
    `component_container_isolated` com SIGSEGV (exit code -11), sempre no mesmo
    ponto do segundo CONFIGURE:

        [route_server]: Configuring Rerouting service operation.
        [ERROR] process has died [pid 40, exit code -11, ...]

Duas tentativas, duas mortes identicas — com meta ativa e sem. Depois disso NAO
existe navegacao nenhuma: os nos nao estao inativos, o processo que os continha
sumiu, e so `docker compose restart nav` traz de volta. Um botao de "consertar a
navegacao" que mata a navegacao e pior que nenhum botao.

A causa esta no `nav2_route`: o `route_server` e reconfigurado do zero pelo
STARTUP (RESET faz CLEANUP, que destroi o no logicamente) e a operacao
`ReroutingService` nao sobrevive ao ciclo. O `route_server` esta na lista
`lifecycle_nodes` do `navigation_launch.py` vendorizado — que e copia upstream e
tem de seguir identica (ver launch/nav2_vendored/README.md) — e este projeto NAO
usa rota nenhuma: a arvore de comportamento e NavigateToPose com NavfnPlanner e
MPPI, e `route_server` nem aparece em nav2_params_go2.yaml. Ele sobe, configura,
ativa e nunca e chamado.

PAUSE + RESUME nao passa por CONFIGURE, entao nao toca nesse caminho. O que se
perde em relacao ao RESET e o que o CONFIGURE refaria: a arvore de comportamento
nao e relida do XML e os plugins nao sao reinstanciados. Nenhum dos dois muda
durante uma demo. O que se perde de verdade e a destruicao do costmap — e e por
isso que o passo 2 existe: `clear_entirely_*_costmap` esvazia o mesmo estado sem
passar pelo ciclo de vida.

Se algum dia o `route_server` sair da lista gerenciada, RESET + STARTUP volta a
ser a opcao mais forte. Ate la, isto.

POR QUE A LOCALIZACAO FICA DE FORA

Existe um segundo gerenciador, `lifecycle_manager_localization`, no caminho de
mapa estatico (nav.launch.py, com AMCL). Ele NAO e tocado aqui, de proposito:
pausar o AMCL no meio de uma demo transforma "a navegacao travou" em "o robo nao
sabe mais onde esta". O caminho quadrupede nem tem esse gerenciador — quem
publica map -> odom la e o `odom_tf`, com identidade, e ele nao e um no de ciclo
de vida.

Consequencia pratica: depois de um reset o robo continua localizado e volta a
aceitar meta. Se a localizacao e que estiver errada, este botao nao conserta.

POR QUE O NAVEGADOR NAO CHAMA manage_nodes DIRETO

NAO e o motivo da armadilha 2 do docs/guia-completo.md (Parte II). Ali o navegador nao PODE
chamar o servico do Gazebo, porque o rosbridge monta o pedido importando o pacote
de interfaces dentro do container `cockpit` e ali nao existe `ros_gz_interfaces`.
Aqui existe: `demo_bringup` declara `<depend>nav2_msgs</depend>` e a chave nao
esta na lista de --skip-keys do docker/base/Dockerfile, entao `ros-jazzy-nav2-msgs`
esta instalado na imagem base e, por heranca, na do cockpit. Verificado em
24/08/2026:

    docker run --rm --entrypoint bash local/demo-aquila-cockpit:dev -lc \
      'source /opt/ros/jazzy/setup.bash; python3 -c "import nav2_msgs.srv"'

E o mesmo motivo pelo qual o clique-para-meta funciona: NavigateToPose e do
`nav2_msgs`. Se fosse ausente, nem a meta sairia.

O motivo real e outro, e e mais forte:

1. NAO E UMA CHAMADA, E QUATRO PASSOS COM ESTADO INTERMEDIARIO INVALIDO. Entre o
   PAUSE e o RESUME a pilha esta inativa: nenhuma meta e aceita e nada a levanta
   sozinha. Se a sequencia morasse no navegador, um F5, uma aba fechada ou uma
   queda de WebSocket no meio dela deixariam o Nav2 desativado sem ninguem para
   terminar o trabalho — e o sintoma seria "a navegacao morreu depois que eu
   apertei o botao de consertar a navegacao". Aqui a sequencia roda inteira num
   processo que nao depende da pagina.

2. QUAL COMANDO USAR E DECISAO DA PILHA, NAO DA TELA. A secao acima e uma
   descoberta medida sobre este Nav2 nesta configuracao. Escrita em JavaScript,
   ela viveria longe do arquivo que sobe os nos gerenciados e divergiria na
   primeira mudanca de topologia da pilha.

3. E a mesma escolha que o `scene_view_controller` faz do lado do simulador: o
   cockpit manda INTENCAO, quem tem o estado tem a maquina.
"""

import time

from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

# Nome default do gerenciador da PILHA DE NAVEGACAO (nao o da localizacao). Sai
# de `name='lifecycle_manager_navigation'` em
# demo_navigation/launch/nav2_vendored/navigation_launch.py, nos dois ramos —
# composto e nao composto. Parametrizado porque um deploy com namespace o
# prefixaria, e um servico inexistente aqui e um botao morto.
DEFAULT_MANAGER = '/lifecycle_manager_navigation/manage_nodes'

# A acao que o clique-para-meta do cockpit usa. Cancelar por aqui, com
# `goal_info` zerado, cancela TODAS as metas ativas — que e o que "resetar o
# alvo" quer dizer, e o que o handle de um cockpit recarregado nao consegue mais
# fazer por ter perdido o uuid.
CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'

# Os dois costmaps. O global acumula os obstaculos que sujam uma demo longa; o
# local e o que o MPPI ve. Limpar um e nao o outro deixa o robo desviando de um
# fantasma que so metade da pilha conhece.
COSTMAP_SERVICES = (
    '/global_costmap/clear_entirely_global_costmap',
    '/local_costmap/clear_entirely_local_costmap',
)

# Medido: PAUSE e RESUME levam ~3 s cada na workstation. O limite e generoso
# porque no Cortex-A72 do AM69 e mais lento, e porque expirar no meio de um
# RESUME deixa a pilha inativa — pior que esperar.
TRANSITION_TIMEOUT_S = 60.0

# Cancelar e limpar sao baratos e nao podem segurar o ciclo: se a acao nem existe
# (pilha ja inativa), o PAUSE/RESUME a seguir resolve de qualquer forma.
SHORT_TIMEOUT_S = 5.0


class NavControlRelay(Node):
    """Traduz Trigger em ciclo de vida do Nav2."""

    def __init__(self) -> None:
        super().__init__('nav_control_relay')

        self.declare_parameter('manager_service', DEFAULT_MANAGER)
        self.declare_parameter('cancel_service', CANCEL_SERVICE)
        self.declare_parameter('costmap_services', list(COSTMAP_SERVICES))
        self._manager_name = self.get_parameter('manager_service').value
        self._cancel_name = self.get_parameter('cancel_service').value
        costmap_names = self.get_parameter('costmap_services').value

        # Grupo reentrante, exatamente como no sim_control_relay: os callbacks de
        # Trigger BLOQUEIAM esperando a resposta do manage_nodes. No grupo
        # mutuamente exclusivo default essa espera impede o executor de processar
        # a propria resposta, e toda chamada expira.
        self._group = ReentrantCallbackGroup()

        self._manager = self.create_client(
            ManageLifecycleNodes, self._manager_name,
            callback_group=self._group,
        )
        self._cancel = self.create_client(
            CancelGoal, self._cancel_name, callback_group=self._group,
        )
        self._costmaps = {
            name: self.create_client(
                ClearEntireCostmap, name, callback_group=self._group)
            for name in costmap_names
        }

        self.create_service(
            Trigger, '/demo/nav/reset', self._on_reset,
            callback_group=self._group,
        )
        self.create_service(
            Trigger, '/demo/nav/cancel', self._on_cancel,
            callback_group=self._group,
        )

        self.get_logger().info(
            'fachada de navegacao pronta: /demo/nav/{reset,cancel} -> '
            f'{self._manager_name}'
        )

    # --- entrada -----------------------------------------------------------

    def _on_reset(self, request, response):
        del request

        if not self._manager.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
            response.success = False
            response.message = (
                f'{self._manager_name} nao respondeu; o container nav esta no ar?'
            )
            self.get_logger().error(response.message)
            return response

        # Cancelar ANTES de desativar. Nao e redundante: sem isto o bt_navigator
        # e desativado com uma meta em curso, o que a aborta sem que o cliente
        # receba um resultado limpo, e o cockpit fica com o HUD em "navegando"
        # sobre uma pilha que nao esta mais correndo.
        self._cancel_all()

        # Limpar com os costmaps AINDA ATIVOS: `clear_entirely_*` e um servico
        # dos proprios nos de costmap, e um no inativo nao atende servico.
        cleared = self._clear_costmaps()

        paused, pause_message = self._transition(
            ManageLifecycleNodes.Request.PAUSE, 'PAUSE')
        if not paused:
            # NAO paramos aqui. Um PAUSE que falhou no meio deixa parte da pilha
            # inativa, e devolver o erro sem tentar levantar de novo daria ao
            # operador uma navegacao morta e um botao que "nao funcionou".
            self.get_logger().warning(
                f'PAUSE falhou ({pause_message}); tentando RESUME')

        resumed, resume_message = self._transition(
            ManageLifecycleNodes.Request.RESUME, 'RESUME')

        response.success = bool(resumed)
        if resumed and paused and cleared:
            response.message = (
                'navegacao reiniciada: meta descartada, costmaps limpos, '
                'servidores reativados'
            )
        elif resumed:
            details = ', '.join(
                part for part in (
                    None if paused else pause_message,
                    None if cleared else 'os costmaps nao foram limpos',
                ) if part
            )
            response.message = f'navegacao ativa, com ressalvas: {details}'
        else:
            response.message = (
                f'a navegacao NAO voltou a ativar: {resume_message}. '
                'Reinicie o container nav.'
            )
        self.get_logger().info(response.message)
        return response

    def _on_cancel(self, request, response):
        del request
        cancelled = self._cancel_all()
        response.success = cancelled
        response.message = (
            'metas canceladas' if cancelled
            else f'{self._cancel_name} nao respondeu; ha meta ativa?'
        )
        return response

    # --- saida -------------------------------------------------------------

    def _transition(self, command, label):
        """Uma transicao do gerenciador. Devolve (ok, motivo)."""
        future = self._manager.call_async(
            ManageLifecycleNodes.Request(command=command))
        if not _wait(future, TRANSITION_TIMEOUT_S):
            return False, f'{label} expirou depois de {TRANSITION_TIMEOUT_S:.0f} s'
        result = future.result()
        if result is None:
            return False, f'{label} nao devolveu resposta'
        if not result.success:
            return False, f'o gerenciador recusou {label}'
        return True, f'{label} aplicado'

    def _cancel_all(self):
        """
        Cancela toda meta ativa de navigate_to_pose.

        `goal_info` fica zerado de proposito: no protocolo de acoes do ROS 2, id
        vazio e stamp zero significam "todas". Preencher o uuid exigiria conhecer
        a meta, que e justamente o que ninguem sabe depois de recarregar a pagina.
        """
        if not self._cancel.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
            self.get_logger().warning(
                f'{self._cancel_name} ainda nao existe; nada a cancelar')
            return False
        future = self._cancel.call_async(CancelGoal.Request())
        if not _wait(future, SHORT_TIMEOUT_S):
            self.get_logger().warning('o cancelamento expirou')
            return False
        return future.result() is not None

    def _clear_costmaps(self):
        """Esvazia os dois costmaps. Devolve True so se os dois responderam."""
        ok = True
        for name, client in self._costmaps.items():
            if not client.wait_for_service(timeout_sec=SHORT_TIMEOUT_S):
                # Nao e fatal: no caminho de mapa estatico o global tem
                # static_layer e volta sozinho, e o reset ainda vale pelo resto.
                self.get_logger().warning(f'{name} nao respondeu')
                ok = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not _wait(future, SHORT_TIMEOUT_S):
                self.get_logger().warning(f'{name} expirou')
                ok = False
        return ok


def _wait(future, timeout_s: float) -> bool:
    """
    Espera o future sem girar o executor.

    `spin_until_future_complete` NAO serve aqui, pela mesma razao registrada em
    demo_simulation/sim_control_relay.py: ja estamos dentro de um callback do
    executor, e gira-lo de novo sobre o mesmo no e reentrancia sobre o objeto
    errado. Quem completa este future e outra thread do MultiThreadedExecutor —
    e o grupo reentrante existe para permitir isso.

    O relogio e o de PAREDE, e aqui isso importa mais que no relay do simulador:
    este no roda com use_sim_time, e um Nav2 desativado no meio de um PAUSE pode
    perfeitamente coexistir com um /clock parado. Medir o timeout no tempo
    simulado transformaria "expirou" em "espera para sempre".
    """
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.02)
    return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavControlRelay()
    # Multithreaded para valer o grupo reentrante: com uma thread so, o callback
    # bloqueado seria a unica disponivel e a resposta nunca chegaria.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
