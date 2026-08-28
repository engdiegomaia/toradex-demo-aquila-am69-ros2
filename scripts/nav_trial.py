#!/usr/bin/env python3
"""
Ensaio de navegação Nav2 sobre o quadrúpede: dirige por METAS e grava evidência.

Roda no host x86, contra a simulação em container e o Nav2 já ativo
(`ROS_DOMAIN_ID=69`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

    ros2 launch demo_bringup nav_quadruped.launch.py     # terminal 2
    python3 scripts/nav_trial.py out.csv --seconds 180   # terminal 3

POR QUE ESTE SCRIPT EXISTE, E POR QUE NÃO É O `gait_trial.py`

`gait_trial.py` publica `/demo/cmd_vel` — é malha aberta, e não pode coexistir
com o Nav2: dois publicadores no tópico que comanda o robô **não dão erro**,
`twist_to_inputs` obedece a última mensagem que chegou, e o robô anda em
espasmos sem nada em log explicando.

Este script **nunca publica `/demo/cmd_vel`**. Ele manda meta pela ação
`navigate_to_pose` e deixa o Nav2 decidir a velocidade, que é exatamente o que
se quer medir quando a pergunta é "o robô atravessa mais rápido?".

`patrol_commander` também manda metas, e é o nó de exposição: uma meta que falha
é abandonada e o ciclo segue, então ele não sabe dizer se o percurso foi
cumprido. O docstring dele manda usar um script de teste para isso. É este.

O QUE ELE MEDE, E POR QUE CADA COISA ESTÁ AQUI

- **RTF no mesmo intervalo das amostras.** Cada linha carrega tempo de simulação
  e tempo monotônico de parede. Corridas com carga ou enlace diferentes deixam
  de parecer comparáveis só porque `/clock` continuou publicando rapidamente.
- **Velocidade média, não o pico.** Nas corridas de 20/08 o pico foi 0,119 m/s e
  a média 0,021 m/s: o MPPI gasta a maior parte do tempo corrigindo rumo a
  0,12 rad/s de teto de guinada. Subir `vx_max` mexe no pico; o que se sente é
  a média. Comparar picos entre condições não decide nada.
- **`/demo/cmd_vel_si`**, não `/demo/cmd_vel`. É a saída do `collision_monitor`,
  em SI. `/demo/cmd_vel` carrega posição de manche (ganho 0,4 / 0,5), e comparar
  manche entre condições que mudam `vx_max` compara a coisa errada.
- **Folga mínima medida pelo lidar**, não contra posições de obstáculo
  conhecidas. Num labirinto não existe "posição do obstáculo": a parede está em
  toda volta. Corredor de 1,20 m com margem de 21,7 cm por lado é o que torna
  esse número o critério de parada de qualquer aumento de velocidade.
- **Quedas por `z`**, imediatamente e com abortar. Qualquer número medido depois
  que o robô caiu descreve um corpo sendo arrastado.
- **Estatísticas do supervisor de marcha** (`mode`, tilt, `yawSat`) vêm do log
  do container, porque `StateTrotting.cpp:589` as emite como linha de log e não
  como tópico. Passe `--sim-log` e elas entram no resumo; sem isso o resumo diz
  que não as tem, em vez de omitir silenciosamente.

No HIL, o script mede a planta simulada no host e o Nav2 que roda no Aquila. Os
números valem para essa pilha distribuída, mas não validam localização por
pernas, um Go2 físico, térmica ou desempenho isolado do módulo (regras 5 e 7 do
`CLAUDE.md`).
"""

import argparse
import csv
import math
import os
import re
import sys
import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from trial_timing import (goals_csv_path, path_metrics, plan_switch_count,
                          timing_spans, vx_metrics)


# Raio circunscrito do tronco do Go2. A folga do lidar até a parede menos isto é
# a folga real da carcaça.
TRUNK_RADIUS_M = 0.383

# Banda de altura que o costmap trata como obstáculo
# (`min_obstacle_height` / `max_obstacle_height` em nav2_params_go2.yaml).
OBSTACLE_Z_MIN_M = 0.12
OBSTACLE_Z_MAX_M = 1.0

# Metas default: centros de corredor do quadruped_maze11.sdf, dentro dos 8 m de
# MAX_GOAL_RADIUS_M. Geradas por
#   maze_fit.py --models <models> maze11 --start se --goals 4
# e não escolhidas a olho: meta sobre parede é ACEITA pelo Nav2 e falha depois,
# perto da borda, onde o erro já não tem nome. A partida é o canto inferior
# direito, então todo o labirinto fica em -x / +y.
MAZE11_GOALS = ((0.00, 8.00), (-8.00, 0.00), (-1.60, 1.60), (-5.83, 4.91))
# Stability gate before autonomous exploration.  These are prefixes of the
# measured connected exit route, split into short steps on the initial aisle.
MAZE11_SHORT_GOALS = ((-0.50, 0.02), (-1.00, 0.03), (-1.50, 0.05))

SUPERVISOR = re.compile(
    r'trot supervisor: mode=(?P<mode>\w+).*?tilt=(?P<tilt>[-\d.]+)deg'
    r'.*?yawSat=(?P<yawsat>[-\d.]+)%')


def yaw_and_tilt(q) -> tuple[float, float]:
    """Yaw em rad e inclinação do eixo z do corpo em graus."""
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    # z do corpo no referencial do mundo; o ângulo dele com o z do mundo é o tilt.
    z_axis_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return yaw, math.degrees(math.acos(max(-1.0, min(1.0, z_axis_z))))


class NavTrial(Node):
    """Manda metas, amostra o estado, e nunca toca em /demo/cmd_vel."""

    def __init__(self, args) -> None:
        super().__init__('nav_trial')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.args = args
        self.pose = None
        self.cmd = Twist()
        self.min_range = float('inf')
        self.plan_length_m = math.nan
        self.plan_heading_deg = math.nan
        self.plan_received_s = None
        self.rows: list[dict] = []
        # Uma entrada por meta ENCERRADA. Nao e log de tela: e a evidencia que o
        # portao de estabilidade exige (desfecho e codigo de erro de cada acao),
        # e por isso ela vai para um CSV irmao em vez de morrer no stdout.
        self.goal_log: list[dict] = []
        self._wall_start: float | None = None
        # Uma meta por vez, com EPOCA. O servidor navigate_to_pose aceita uma
        # meta so: mandar outra PREEMPTA a anterior, que devolve ABORTED, cujo
        # callback chegaria depois e sobrescreveria o estado da meta nova. Isso
        # realimenta -- cada callback obsoleto libera outro envio -- e o ensaio
        # vira uma enxurrada de metas na taxa do laco. Ja aconteceu antes neste
        # projeto (F5, "metas concorrentes"). A epoca e o que corta a
        # realimentacao: callback de epoca velha e descartado.
        self._epoch = 0
        self._active = False
        self._result = None
        # Identidade e desfecho da meta em voo. `_goal_index` tambem carimba
        # cada amostra de telemetria, que e o que permite recortar odometria,
        # plano e trocas de rota POR META depois -- sem isso o CSV mistura as
        # tres metas do portao numa serie so e a corrida nao prova nada.
        self._goal_index: int | None = None
        self._goal_target: tuple[float, float] | None = None
        self._goal_sent_s = math.nan
        self._status: int | None = None
        self._error_code: int | None = None
        self._error_msg = ''

        self.create_subscription(Odometry, '/demo/odom', self._on_odom,
                                 QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, 10)
        self.create_subscription(PointCloud2, '/demo/scan_cloud', self._on_cloud,
                                 QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # -- entradas ----------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        if points.size == 0:
            return
        band = ((points[:, 2] > OBSTACLE_Z_MIN_M)
                & (points[:, 2] < OBSTACLE_Z_MAX_M))
        if not band.any():
            return
        distances = np.hypot(points[band, 0], points[band, 1])
        finite = distances[np.isfinite(distances)]
        if finite.size:
            self.min_range = float(finite.min())

    def _on_plan(self, msg: Path) -> None:
        points = [(entry.pose.position.x, entry.pose.position.y)
                  for entry in msg.poses]
        self.plan_length_m, self.plan_heading_deg = path_metrics(points)
        self.plan_received_s = self.sim_s()

    # -- relógios ----------------------------------------------------------

    def sim_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- metas -------------------------------------------------------------

    def _send(self, x: float, y: float, yaw: float, index: int) -> None:
        """
        Envia uma meta. O yaw é o rumo de CHEGADA, deliberadamente.

        Exigir um yaw arbitrário na chegada custou 110-139 graus de giro parado
        nas corridas de 20/08, e girar parado não fica parado: o robô derivou
        0,78 m em y e saiu da tolerância de posição que já havia satisfeito.
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.args.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # A plan still in memory belongs to the preceding goal. Mark it absent
        # until planner_server publishes the route for this epoch.
        self.plan_length_m = math.nan
        self.plan_heading_deg = math.nan
        self.plan_received_s = None
        self._epoch += 1
        self._active = True
        self._result = None
        self._goal_index = index
        self._goal_target = (x, y)
        self._goal_sent_s = self.sim_s()
        self._status = None
        self._error_code = None
        self._error_msg = ''
        epoch = self._epoch
        self._handle = None
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_accepted(done, epoch))

    def _on_accepted(self, future, epoch: int) -> None:
        if epoch != self._epoch:
            return
        handle = future.result()
        if not handle.accepted:
            self._active = False
            self._result = 'rejeitada'
            # Recusa nao produz Result, entao nao ha error_code do Nav2 aqui: a
            # ausencia e informacao, e nao deve virar um zero que se confunde
            # com NONE. Fica vazia no CSV.
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._on_finished(done, epoch))

    def _on_finished(self, future, epoch: int) -> None:
        if epoch != self._epoch:
            return
        self._active = False
        self._handle = None
        wrapped = future.result()
        # status 4 == SUCCEEDED em action_msgs/GoalStatus
        self._status = int(wrapped.status)
        self._result = 'ok' if wrapped.status == 4 else 'falhou'
        # NavigateToPose.Result em Jazzy carrega error_code/error_msg. E a
        # diferenca entre "falhou" e "falhou POR QUE", que e a primeira pergunta
        # da ordem de investigacao do portao.
        self._error_code = int(getattr(wrapped.result, 'error_code', 0))
        self._error_msg = str(getattr(wrapped.result, 'error_msg', ''))

    def cancel_goal(self) -> None:
        """Cancela a meta corrente e aposenta a epoca dela."""
        if self._handle is not None:
            self._handle.cancel_goal_async()
        self._epoch += 1
        self._handle = None
        self._active = False

    def close_goal(self, elapsed_s: float) -> None:
        """Fecha a meta em voo e a arquiva com desfecho e codigo de erro."""
        if self._goal_index is None or self._result is None:
            return
        target = self._goal_target or (math.nan, math.nan)
        # Trocas de rota DESTA meta, nao do ensaio inteiro: o criterio do portao
        # e por meta, e somar o ensaio esconde qual delas oscilou.
        mine = [row for row in self.rows if row.get('goal_index')
                == self._goal_index]
        self.goal_log.append({
            'goal_index': self._goal_index,
            'goal_x': round(target[0], 3),
            'goal_y': round(target[1], 3),
            'sent_sim_s': round(self._goal_sent_s, 1),
            'elapsed_s': round(elapsed_s, 1),
            'outcome': self._result,
            'status': '' if self._status is None else self._status,
            'error_code': '' if self._error_code is None else self._error_code,
            'error_msg': self._error_msg,
            'samples': len(mine),
            'plan_switches': plan_switch_count(mine),
        })
        self._result = None
        self._goal_index = None

    # -- amostragem --------------------------------------------------------

    def sample(self) -> None:
        yaw, tilt = yaw_and_tilt(self.pose.orientation)
        if self._wall_start is None:
            raise RuntimeError('wall clock not initialized')
        sim_now = self.sim_s()
        plan_age = (sim_now - self.plan_received_s
                    if self.plan_received_s is not None else math.nan)
        self.rows.append({
            'goal_index': '' if self._goal_index is None else self._goal_index,
            'sim_s': round(sim_now, 3),
            'wall_s': round(time.monotonic() - self._wall_start, 3),
            'x': round(self.pose.position.x, 4),
            'y': round(self.pose.position.y, 4),
            'z': round(self.pose.position.z, 4),
            'yaw_deg': round(math.degrees(yaw), 2),
            'tilt_deg': round(tilt, 2),
            'cmd_vx': round(self.cmd.linear.x, 4),
            'cmd_wz': round(self.cmd.angular.z, 4),
            'min_range_m': round(self.min_range, 3),
            'plan_length_m': round(self.plan_length_m, 3),
            'plan_heading_deg': round(self.plan_heading_deg, 2),
            'plan_age_s': round(max(0.0, plan_age), 3)
            if math.isfinite(plan_age) else math.nan,
        })

    def wait_for_stack(self) -> None:
        """
        Espera odometria E o servidor de ação, com erro que diz qual faltou.

        `ros2 action list` BLOQUEIA indefinidamente com o grafo incompleto -- não
        devolve vazio, não expira. Por isso o portão aqui é
        `wait_for_server` com prazo, e não uma listagem.
        """
        deadline = time.time() + self.args.wait_stack
        while time.time() < deadline and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise SystemExit(
                'sem /demo/odom em %.0fs: a simulação está de pé, e o '
                'ROS_DOMAIN_ID é 69?' % self.args.wait_stack)

        remaining = max(1.0, deadline - time.time())
        if not self.client.wait_for_server(timeout_sec=remaining):
            raise SystemExit(
                'navigate_to_pose não apareceu: o Nav2 subiu e chegou a '
                '"Managed nodes are active"? Sem isso não há o que medir.')

        # DE PÉ É z E INCLINAÇÃO, NÃO SÓ z.
        #
        # Um Go2 tombado de lado mantém o tronco a ~0.25 m do chão, acima de
        # qualquer --min-z razoável. Medido em 21/08/2026 no HIL: o robô caiu num
        # ensaio, ficou caído, e o ensaio SEGUINTE passou por esta verificação e
        # mediu 240 s de robô no chão como se fosse navegação -- 808 linhas de
        # supervisor, todas em RECOVER, tilt 136 graus, e um relatório completo
        # com velocidade média 0.0000 m/s. Nada acusou que a medição era lixo.
        def upright() -> bool:
            _, tilt = yaw_and_tilt(self.pose.orientation)
            return (self.pose.position.z > self.args.min_z
                    and tilt < self.args.max_tilt)

        while not upright() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not upright():
            _, tilt = yaw_and_tilt(self.pose.orientation)
            raise SystemExit(
                'robô não está de pé (z = %.3f m, inclinação = %.1f graus; '
                'limites --min-z %.2f m e --max-tilt %.0f graus): não há nada '
                'a medir. Reinicie a simulação.'
                % (self.pose.position.z, tilt, self.args.min_z,
                   self.args.max_tilt))

    def run(self, goals) -> str:
        """Cicla as metas por --seconds de tempo de SIMULAÇÃO."""
        start = self.sim_s()
        self._wall_start = time.monotonic()
        next_sample = start
        index = 0
        goal_started = start
        previous = (self.pose.position.x, self.pose.position.y)
        verdict = 'concluído'

        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.sim_s()
            if now - start >= self.args.seconds:
                self.cancel_goal()
                break

            # Mesmo critério da pré-condição: z E inclinação. Tombo lateral
            # mantém z acima do limiar, então checar z sozinho deixa o ensaio
            # seguir medindo um corpo deitado no chão.
            _, tilt_now = yaw_and_tilt(self.pose.orientation)
            if (self.pose.position.z <= self.args.min_z
                    or tilt_now >= self.args.max_tilt):
                verdict = ('ABORTADO: robô caiu (z = %.3f m, inclinação = '
                           '%.1f graus)' % (self.pose.position.z, tilt_now))
                self.cancel_goal()
                break

            # Prazo por meta. Sem ele uma única meta inalcançável consome o
            # ensaio inteiro e a comparação entre condições perde a perna que
            # ela devia medir. Cancelar e seguir é o que o Nav2 permite.
            if self._active and now - goal_started >= self.args.goal_timeout:
                self.cancel_goal()
                self._result = 'prazo'

            # `settle` é a defesa contra realimentação: mesmo que o estado da
            # meta fique inconsistente por algum motivo não previsto, o ensaio
            # não pode virar uma enxurrada de metas. Duas por segundo já é mais
            # rápido do que qualquer meta real termina.
            if not self._active and now - goal_started >= self.args.goal_settle:
                self.close_goal(now - start)
                target = goals[index % len(goals)]
                heading = math.atan2(target[1] - previous[1],
                                     target[0] - previous[0])
                self._send(target[0], target[1], heading, index)
                previous = target
                index += 1
                goal_started = now

            if now >= next_sample:
                self.sample()
                next_sample += 1.0 / self.args.sample_rate

        # A meta em voo quando o ensaio termina TEM de ser arquivada. Sem isto a
        # ultima meta some do log -- ela so era registrada quando a seguinte
        # partia -- e um portao de 3 metas terminava mostrando 2. Custou a
        # leitura da corrida de 28/08/2026.
        if self._result is None:
            # Meta ainda em voo quando o prazo do ensaio (ou a queda) chegou.
            # Nao e 'falhou': ninguem a deixou terminar.
            self._result = 'cancelada'
        self.close_goal(self.sim_s() - start)
        return verdict


def supervisor_stats(path: str, offset: int) -> dict | None:
    """
    Lê as linhas do supervisor de marcha escritas DEPOIS de `offset`.

    Casar pela posição no arquivo, e não por timestamp, porque a linha de log
    carrega o relógio do nó e o ensaio mede tempo de simulação -- correlacionar
    os dois introduz um erro que ninguém percebe.
    """
    if not path or not os.path.isfile(path):
        return None
    with open(path, 'r', encoding='utf-8', errors='ignore') as handle:
        handle.seek(offset)
        modes, tilts, saturation = [], [], []
        for line in handle:
            found = SUPERVISOR.search(line)
            if found:
                modes.append(found.group('mode'))
                tilts.append(float(found.group('tilt')))
                saturation.append(float(found.group('yawsat')))
    if not modes:
        return {'lines': 0}
    return {
        'lines': len(modes),
        'recover': sum(1 for m in modes if m.upper() == 'RECOVER'),
        'tilt_max': max(tilts),
        'yawsat_mean': sum(saturation) / len(saturation),
        'yawsat_max': max(saturation),
    }


def summarise(trial: NavTrial, verdict: str, stats: dict | None) -> None:
    """Imprime o resumo. A média é o número que decide, não o pico."""
    rows = trial.rows
    if len(rows) < 2:
        print('menos de duas amostras: nada a resumir', file=sys.stderr)
        return

    xs = np.array([r['x'] for r in rows])
    ys = np.array([r['y'] for r in rows])
    path = float(np.hypot(np.diff(xs), np.diff(ys)).sum())
    net = float(math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))
    elapsed, wall_elapsed, rtf = timing_spans(rows)
    cmd_vx = np.array([r['cmd_vx'] for r in rows])
    vx_zero, vx_duty = vx_metrics(rows, trial.args.vx_zero_threshold,
                                   trial.args.vx_work_threshold)
    ranges = np.array([r['min_range_m'] for r in rows])
    ranges = ranges[np.isfinite(ranges)]

    print()
    print(f'veredito                 {verdict}')
    print(f'tempo de simulação       {elapsed:.1f} s  ({len(rows)} amostras)')
    print(f'tempo de parede          {wall_elapsed:.1f} s')
    print(f'fator de tempo real      {rtf:.3f}')
    print(f'caminho percorrido       {path:.2f} m')
    print(f'deslocamento líquido     {net:.2f} m')
    print(f'velocidade média         {path / elapsed:.4f} m/s')
    print(f'cmd_vx pico / médio      {cmd_vx.max():.3f} / {cmd_vx.mean():.4f} m/s')
    print(f'cmd_vx ~ 0               {100.0 * vx_zero:.1f}% das amostras')
    print(f'razão de trabalho vx     {100.0 * vx_duty:.1f}% '
          f'(cmd_vx > {trial.args.vx_work_threshold:.3f} m/s)   <- métrica primária')
    print(f'cmd_vx negativo          {100.0 * (cmd_vx < 0).mean():.0f}% das amostras')
    print(f'tilt pico                {max(r["tilt_deg"] for r in rows):.2f} deg')
    if ranges.size:
        print(f'folga mínima (lidar)     {ranges.min():.3f} m'
              f'   -> carcaça {ranges.min() - TRUNK_RADIUS_M:+.3f} m')
    valid_plans = [r for r in rows
                   if math.isfinite(float(r.get('plan_length_m', math.nan)))
                   and math.isfinite(float(r.get('plan_heading_deg', math.nan)))
                   and float(r.get('plan_age_s', math.inf)) <= 2.5]
    if valid_plans:
        lengths = np.array([r['plan_length_m'] for r in valid_plans])
        print(f'plano comprimento       {np.median(lengths):.2f} m mediana '
              f'[{lengths.min():.2f}, {lengths.max():.2f}]')
        print(f'trocas grandes de rota   {plan_switch_count(rows)} '
              '(>1 m ou >45 graus)')
    done = sum(1 for entry in trial.goal_log if entry['outcome'] == 'ok')
    print(f'metas                    {done} cumprida(s) de '
          f'{len(trial.goal_log)} encerrada(s)')
    tally: dict = {}
    for entry in trial.goal_log:
        tally[entry['outcome']] = tally.get(entry['outcome'], 0) + 1
    if tally:
        print('   por desfecho            ' + '  '.join(
            f'{name}={count}' for name, count in sorted(tally.items())))
    for entry in trial.goal_log[:12]:
        code = entry['error_code']
        detail = f' código {code}' if code not in ('', 0) else ''
        if entry['error_msg']:
            detail += f' ({entry["error_msg"]})'
        print(f'   meta {entry["goal_index"]}: {entry["outcome"]} em '
              f't={entry["elapsed_s"]:.0f}s{detail}, '
              f'{entry["plan_switches"]} troca(s) de rota')
    if len(trial.goal_log) > 12:
        print(f'   ... e {len(trial.goal_log) - 12} outras')

    if stats is None:
        print('supervisor de marcha     SEM DADOS (passe --sim-log)')
    elif stats['lines'] == 0:
        print('supervisor de marcha     0 linhas: o robô entrou em trote?')
    else:
        print(f'supervisor de marcha     {stats["lines"]} linhas, '
              f'RECOVER={stats["recover"]}, tilt_max={stats["tilt_max"]:.1f} deg, '
              f'yawSat média={stats["yawsat_mean"]:.0f}% '
              f'pico={stats["yawsat_max"]:.0f}%')


def main(argv=None) -> int:
    """Roda um ensaio e grava o CSV. Devolve 1 se o robô caiu."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='arquivo CSV de saída')
    parser.add_argument('--seconds', type=float, default=180.0,
                        help='duração em tempo de SIMULAÇÃO')
    # USE `--goals=`, COM SINAL DE IGUAL. Toda meta do maze11 tem x negativo, e
    # `--goals -1.50,0.05` e lido pelo argparse como uma FLAG desconhecida: o
    # script imprime `usage` e sai 0. Num pipe com `2>/dev/null` -- que e o uso
    # normal aqui, porque o CycloneDDS enche o stderr -- isso vira uma corrida
    # silenciosa que nao faz nada e nao acusa. Custou uma corrida em 27/08/2026.
    parser.add_argument('--goals', default='maze11',
                        help='"maze11", "maze11-short" ou "x,y;x,y;...". '
                             'Passe com sinal de igual (--goals="-1.5,0.05;...") '
                             ': coordenada negativa sem o "=" vira flag e o '
                             'script sai 0 sem rodar nada.')
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--cmd-topic', default='/demo/cmd_vel_si',
                        help='saída SI do Nav2; NAO use /demo/cmd_vel (manche)')
    # 0.20, nao 0.30. O robo em pe fica em ~0.35 m em trote e afunda a 0.29 num
    # tropeco -- medido no HIL em 21/08/2026, onde 0.30 abortou um ensaio por
    # AFUNDAMENTO e chamou isso de queda. Um Go2 caido fica em ~0.10 m, entao
    # 0.20 separa os dois casos com folga nos dois lados.
    parser.add_argument('--min-z', type=float, default=0.20,
                        help='abaixo disto o robô está no chão e o ensaio aborta')
    # Um Go2 em pé fica abaixo de 4 graus mesmo em trote (medido: pico 3.3).
    # 30 graus e folga de quase uma ordem de grandeza e ainda pega tombo, que
    # passa de 100 graus.
    parser.add_argument('--max-tilt', type=float, default=30.0,
                        help='acima disto o robô tombou e o ensaio aborta')
    parser.add_argument('--goal-settle', type=float, default=2.0,
                        help='intervalo mínimo entre envios de meta; e o teto '
                             'que impede o ensaio de virar enxurrada de metas')
    parser.add_argument('--goal-timeout', type=float, default=90.0,
                        help='prazo por meta em tempo de simulação; ao expirar '
                             'a meta é cancelada e o ciclo segue')
    parser.add_argument('--wait-stack', type=float, default=120.0)
    parser.add_argument('--sample-rate', type=float, default=10.0)
    parser.add_argument('--vx-zero-threshold', type=float, default=0.005,
                        help='|cmd_vx| até este valor conta como zero (m/s)')
    parser.add_argument('--vx-work-threshold', type=float, default=0.05,
                        help='cmd_vx acima deste valor conta como trabalho para frente (m/s)')
    parser.add_argument('--sim-log',
                        help='log do container da simulação, para as '
                             'estatísticas do supervisor de marcha')
    args = parser.parse_args(argv)

    if args.goals == 'maze11':
        goals = list(MAZE11_GOALS)
    elif args.goals == 'maze11-short':
        goals = list(MAZE11_SHORT_GOALS)
    else:
        goals = [tuple(float(v) for v in pair.split(',')[:2])
                 for pair in args.goals.split(';') if pair.strip()]
    if not goals:
        print('nenhuma meta: --goals ficou vazio', file=sys.stderr)
        return 2

    offset = os.path.getsize(args.sim_log) if (
        args.sim_log and os.path.isfile(args.sim_log)) else 0

    rclpy.init()
    trial = NavTrial(args)
    try:
        trial.wait_for_stack()
        print(f'pilha de pé. {len(goals)} metas, {args.seconds:.0f} s de '
              'tempo de simulação.')
        verdict = trial.run(goals)
    finally:
        with open(args.csv, 'w', newline='', encoding='utf-8') as handle:
            if trial.rows:
                writer = csv.DictWriter(handle, fieldnames=list(trial.rows[0]))
                writer.writeheader()
                writer.writerows(trial.rows)
        with open(goals_csv_path(args.csv), 'w', newline='',
                  encoding='utf-8') as handle:
            if trial.goal_log:
                writer = csv.DictWriter(
                    handle, fieldnames=list(trial.goal_log[0]))
                writer.writeheader()
                writer.writerows(trial.goal_log)
        trial.destroy_node()
        rclpy.shutdown()

    summarise(trial, verdict, supervisor_stats(args.sim_log, offset))
    print(f'\namostras em {args.csv}')
    print(f'metas em     {goals_csv_path(args.csv)}')
    return 1 if verdict.startswith('ABORTADO') else 0


if __name__ == '__main__':
    sys.exit(main())
