#!/usr/bin/env python3
"""
Republica `/clock` a taxa fixa, a partir do clock cru do Gazebo.

Roda no host x86, dentro do container do simulador, ao lado do Gazebo.

LEIA ISTO PRIMEIRO: O ESTRANGULAMENTO ESTA DESLIGADO POR DEFAULT
================================================================

`rate_hz: 0` (o default do launch) e passagem direta. Este no nasceu para
estrangular o `/clock` e reduzir a carga do Aquila, e o ensaio REFUTOU a ideia.
Medido em 21/08/2026, modo hil, mesmo mundo e mesmas metas:

    /clock     CPU do container nav    velocidade media    cmd_vx de pico
    ~750 Hz    470% de 800%            0.0251 m/s          0.138 m/s
     100 Hz    324% de 800%            0.0039 m/s          0.003 m/s

A CPU caiu de verdade. A navegacao morreu junto: o robo passou 180 s girando no
lugar, `cmd_wz` ativo em 1721 de 1800 amostras e `cmd_vx` em zero. O mecanismo
exato de como a granularidade de 10 ms quebra o MPPI nao esta isolado -- o que
esta medido e a relacao de causa. Economia de CPU que faz o robo parar de andar
nao e otimizacao.

O ataque certo ao MESMO custo e compor o Nav2 num processo unico: uma assinatura
de `/clock` em vez de treze, e comunicacao intraprocesso no lugar de DDS. Isso
esta em `demo_bringup/launch/nav_quadruped.launch.py`, no bloco `nav2_container`.

ESTE NO NAO ESTA EM NENHUM LAUNCH. Nao basta rodar `ros2 run` para usa-lo: o
`bridge_quadruped.yaml` publica `/clock` DIRETO, entao subir este no sem mudar o
bridge cria dois publicadores no mesmo topico -- falha silenciosa, o robo anda
estranho e nada em log nomeia o relogio. Para repetir o A/B sao duas mudancas:

    1. em demo_simulation/config/bridge_quadruped.yaml, trocar o
       `ros_topic_name` do clock de "/clock" para "/demo/clock_raw";
    2. subir este no com `rate_hz` no valor a ensaiar (0 = passagem direta,
       que reproduz o basal com o hop extra ja no lugar).

Ele ficou fora do caminho default porque e intermediario sem funcao depois do
A/B, nao porque o hop tenha sido medido como caro: em passagem direta a
diferenca ficou DENTRO do ruido de corrida (0.0202 m/s com o hop, 0.0232 sem, na
mesma configuracao e no mesmo mundo). Quem procurar aqui a explicacao para uma
queda de velocidade nao vai encontrar -- o hop nao e ela.

O diagnostico abaixo continua valido -- e o custo que existe.

O CUSTO QUE O CLOCK DE 1 kHz REALMENTE IMPOE
============================================

O mundo do Go2 usa `<max_step_size>0.001</max_step_size>` porque a marcha
precisa de 1 ms de passo de fisica. O Gazebo publica `/clock` a cada passo,
entao o bridge entrega ~1000 mensagens por segundo.

No host x86 isso e absorvivel. No Aquila AM69 nao e, e a falha NAO se parece com
falha de clock. Medido em 21/08/2026, modo hil, Nav2 no modulo:

    /clock no host        989 Hz
    /clock no modulo      870 Hz
    CPU do container nav  660% de 800% disponiveis
    load average          15 a 23, com 8 nucleos
    odom_tf               87% de um nucleo
    cmd_vel_si_to_stick   89% de um nucleo

`odom_tf` e `cmd_vel_si_to_stick` sao republicadores triviais em Python. A unica
coisa de alta taxa que ambos processam e `/clock`, porque `use_sim_time: true`
faz TODO no do Nav2 assinar esse topico: ~13 nos x 870 Hz = ~11 mil entregas por
segundo num Cortex-A72. O sintoma visivel e o robo navegando devagar, com
`cmd_vx` de pico normal (0.138) e MEDIA quase zero (0.0067) -- o controlador
esta faminto, nao mal sintonizado.

Nada em log nomeia o clock. Os nos apenas ficam lentos.

POR QUE ESTRANGULAR AQUI NAO AMEACA A MARCHA
============================================

O `controller_manager` deste robo roda DENTRO do processo do Gazebo, via
`gz_quadruped_hardware`, e e passado pelo loop de fisica -- nao por `/clock`.
Verificado em `quadruped.launch.py`, que comenta exatamente isso. Portanto a
malha de 1 kHz da marcha continua a 1 kHz com o clock publicado a 100 Hz.

Quem consome `/clock` sao os nos com `use_sim_time`, e para eles 100 Hz da 10 ms
de granularidade, folgado para Nav2, TF e para os ensaios (que amostram a 10 Hz).

O QoS DE SAIDA E BEST-EFFORT, DE PROPOSITO
==========================================

`rclcpp::ClockQoS` do proprio ROS 2 e KeepLast(1) best-effort, e e o certo aqui:
perder uma mensagem de clock e inofensivo, porque a proxima vem em 10 ms.
Entrega CONFIAVEL de clock sobre Wi-Fi custa retransmissao e ACK por mensagem,
para nenhum ganho. Este no publica com a mesma politica.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from rosgraph_msgs.msg import Clock

# Equivalente ao ClockQoS do ROS 2: o ultimo valor e o unico que importa.
CLOCK_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ClockThrottle(Node):
    """Guarda o ultimo clock cru e o republica num timer de tempo REAL."""

    def __init__(self) -> None:
        super().__init__('clock_throttle')
        # NUNCA use_sim_time verdadeiro neste no: ele PRODUZ o /clock. Seguir o
        # proprio relogio simulado o deixaria esperando por si mesmo, e o
        # sintoma seria a simulacao subir sem tempo nenhum -- o mesmo travamento
        # que `wait_for_clock` existe para diagnosticar. Quem passa `False` e o
        # launch; declarar aqui levanta ParameterAlreadyDeclaredException,
        # porque o Node do rclpy JA declara use_sim_time sozinho.
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('input_topic', '/demo/clock_raw')

        rate = float(self.get_parameter('rate_hz').value)
        source = str(self.get_parameter('input_topic').value)

        self._latest = None
        self._in = 0
        self._out = 0
        # rate_hz = 0 e PASSAGEM DIRETA: cada mensagem sai como entrou, com o
        # timestamp original. Existe para o braco A de um A/B -- provar que uma
        # mudanca de comportamento veio, ou nao veio, do estrangulamento. Nao da
        # para imitar isso com um rate alto: um timer a 2000 Hz republica a
        # ULTIMA mensagem repetidamente e entrega MAIS trafego que o Gazebo
        # produz, o que mede outra coisa.
        self._bypass = rate <= 0.0

        self.create_subscription(Clock, source, self._on_clock, CLOCK_QOS)
        self.pub = self.create_publisher(Clock, '/clock', CLOCK_QOS)
        if not self._bypass:
            # Timer de tempo real: com use_sim_time falso, create_timer usa o
            # relogio do sistema, que e o que se quer para cadenciar a saida.
            self.create_timer(1.0 / rate, self._tick)
        self.create_timer(10.0, self._report)

        if self._bypass:
            self.get_logger().warning(
                'PASSAGEM DIRETA de %s -> /clock (rate_hz=0). Sem '
                'estrangulamento: o Aquila recebe os ~880 Hz do Gazebo. Modo de '
                'ensaio, nao de operacao.' % source)
        else:
            self.get_logger().info(
                'republicando %s -> /clock a %.0f Hz. O passo de fisica NAO '
                'muda: a marcha e passada pelo loop do Gazebo, nao pelo /clock.'
                % (source, rate))

    def _on_clock(self, msg: Clock) -> None:
        self._latest = msg
        self._in += 1
        if self._bypass:
            self.pub.publish(msg)
            self._out += 1

    def _tick(self) -> None:
        # Sem clock cru ainda: nao publique nada. Publicar zero faria os nos com
        # use_sim_time saltarem para t=0 e a TF inteira ficaria no passado.
        if self._latest is None:
            return
        self.pub.publish(self._latest)
        self._out += 1

    def _report(self) -> None:
        # A razao entrada/saida e o que prova que o estrangulamento agiu. Sem
        # isto, "mudei a taxa e nada aconteceu" nao tem como ser distinguido de
        # "a taxa nunca mudou".
        self.get_logger().info(
            'clock: entrada %d msg, saida %d msg nos ultimos 10 s '
            '(~%.0f Hz -> ~%.0f Hz)'
            % (self._in, self._out, self._in / 10.0, self._out / 10.0))
        self._in = 0
        self._out = 0


def main(args=None) -> None:
    """Sobe o no e roda ate ser interrompido."""
    rclpy.init(args=args)
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
