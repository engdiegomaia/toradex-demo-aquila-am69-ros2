"""
Converte o Twist em SI que o Nav2 produz para as unidades de manche do contrato.

Roda na mesma máquina que o Nav2 (estação x86 hoje, módulo no modo hil).

    ros2 run demo_bringup cmd_vel_si_to_stick

## Por que este nó tem de existir

`/demo/cmd_vel` **não está em SI**, apesar de ser um `geometry_msgs/Twist`. Ele
carrega posição normalizada de manche, e o controlador de marcha aplica o próprio
ganho ao recebê-la. A prova é aritmética, na fonte vendorizada:

    StateTrotting.cpp:192  v_cmd = invNormalize(ly, -0.4, +0.4)
    mathTools.h:10         invNormalize(v, min, max) = 0.4 * v   (minLim=-1, maxLim=1)
    twist_to_inputs.py:283 ly = linear.x                          (ganho UNITÁRIO)

Logo `linear.x` chega ao robô multiplicado por **0,4**. O mesmo para guinada, com
ganho **0,5**. O projeto sempre soube disso e trabalha assim: `demo_routine.to_twist`
divide pelo ganho antes de publicar, e `docs/results/ml35-f4-parcial.md` registra
"comando `linear.x = 0.25` (→ `v_cmd = 0,1 m/s`)".

O Nav2 **não pode** trabalhar assim. Ele não é só um publicador de velocidade: o
MPPI integra `vx` como metros por segundo para prever onde o robô estará. Se o
número publicado vale 0,4× do que ele modela, todo rollout erra a distância por
2,5×, e o horizonte que se calibrou em 1,44 m vale 0,58 m na planta.

Medido em 20/08/2026, com `vx_max: 0.15` interpretado como manche: em 300 s o
robô comandou no máximo 0,058 (média 0,014), ou seja **0,006 m/s reais** — abaixo
do mínimo de ~0,05 m/s em que a marcha se mantém estável, e 17× menor que o ponto
validado de 0,10 m/s. Nenhuma meta cabia no prazo.

## Onde este nó entra, e o que ele deliberadamente NÃO faz

    Nav2 (collision_monitor)  --/demo/cmd_vel_si-->  ESTE NÓ  --/demo/cmd_vel-->  planta
                                   SI                             manche

Ele **não muda o contrato** de `/demo/cmd_vel` e **não toca a planta**.
`demo_routine` e `gait_trial.sh` continuam publicando manche direto em
`/demo/cmd_vel`, sem alteração, e todo número já registrado nos resultados
continua significando o que significava.

A alternativa — fazer `twist_to_inputs` aceitar SI — deixaria `/demo/cmd_vel`
honesto, e foi rejeitada por escopo: mudaria a planta, os dois comandantes
existentes, e o significado de cada `--v-cmd` já gravado em `docs/results/`.

## A armadilha que este nó cria

Passam a existir dois tópicos `Twist` com unidades diferentes. `_si` no nome é a
única defesa, e ela é fraca. Se alguém ligar o Nav2 direto em `/demo/cmd_vel`, o
robô anda a 40% do pedido e nada acusa. O sintoma é exatamente o medido acima:
robô lento que nunca chega, sem erro em log nenhum.

`/demo/cmd_vel` continua tendo **um** publicador por vez. Este nó é um deles.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Ganhos do controlador, de `trot.v_x_limit` / `v_y_limit` / `w_yaw_limit` em
# `demo_simulation/config/gait_go2.yaml`, que são [-0.4, 0.4], [-0.3, 0.3] e
# [-0.5, 0.5]. `invNormalize` com esses limites reduz a multiplicar pelo limite
# superior. Se você mudar o YAML, mude aqui -- não há como o nó descobrir sozinho,
# porque o parâmetro pertence ao controlador e não a ele.
VX_PER_STICK = 0.4
VY_PER_STICK = 0.3
WZ_PER_STICK = 0.5

# Envelope de manche comprovadamente estável (`_SAFE_STICK_LIMIT` em
# `twist_to_inputs.py`). Clampar aqui em vez de deixar a planta clampar em
# silêncio: um pedido acima do envelope fica visível neste nó.
STICK_CLAMP = 0.5


def to_stick(value: float, gain: float, clamp: float = STICK_CLAMP) -> float:
    """Divide uma velocidade SI pelo ganho do controlador e limita ao envelope."""
    return max(-clamp, min(clamp, value / gain))


def convert(si: Twist) -> Twist:
    """Monta o Twist em manche equivalente a um Twist em SI."""
    out = Twist()
    out.linear.x = to_stick(si.linear.x, VX_PER_STICK)
    out.linear.y = to_stick(si.linear.y, VY_PER_STICK)
    # Sem inversão de sinal aqui: `twist_to_inputs` já nega lx e rx, porque o
    # controlador upstream nega esses eixos internamente. Negar de novo faria o
    # robô virar para o lado errado, e o Nav2 corrigiria aumentando o erro.
    out.angular.z = to_stick(si.angular.z, WZ_PER_STICK)
    return out


class CmdVelSiToStick(Node):
    """Republica `/demo/cmd_vel_si` como `/demo/cmd_vel` em unidades de manche."""

    def __init__(self) -> None:
        """Abre a assinatura em SI e a publicação em manche."""
        super().__init__('cmd_vel_si_to_stick')
        self._publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.create_subscription(Twist, '/demo/cmd_vel_si', self._on_si, 10)
        self._forwarded = 0
        self._saturated = 0
        self.get_logger().info(
            'convertendo /demo/cmd_vel_si (SI) para /demo/cmd_vel (manche) com '
            'ganhos vx=%.2f vy=%.2f wz=%.2f e clamp %.2f. NAO ligue o Nav2 '
            'direto em /demo/cmd_vel: o robo andaria a %.0f%% do pedido sem '
            'nenhum erro em log.'
            % (VX_PER_STICK, VY_PER_STICK, WZ_PER_STICK, STICK_CLAMP,
               100.0 * VX_PER_STICK))

    def _on_si(self, message: Twist) -> None:
        """Republica uma mensagem em manche, avisando quando satura."""
        out = convert(message)
        self._publisher.publish(out)

        self._forwarded += 1
        if abs(out.linear.x) >= STICK_CLAMP or abs(out.angular.z) >= STICK_CLAMP:
            self._saturated += 1
            # Saturar significa que o Nav2 pede acima do envelope da marcha. Não
            # é erro deste nó, é sinal de que os limites do MPPI estão largos.
            self.get_logger().warning(
                'manche saturado: pedido vx=%.3f wz=%.3f SI excede o envelope. '
                'Aperte vx_max/wz_max em nav2_params_go2.yaml (%d de %d '
                'mensagens)' % (message.linear.x, message.angular.z,
                                self._saturated, self._forwarded),
                throttle_duration_sec=10.0)


def main(args=None) -> None:
    """Roda o conversor até ser interrompido."""
    rclpy.init(args=args)
    node = CmdVelSiToStick()
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
