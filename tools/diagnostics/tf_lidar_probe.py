#!/usr/bin/env python3
"""
Sonda temporal do lidar: mede IDADE e DISPONIBILIDADE de TF, sem comandar nada.

Roda no host x86, contra a pilha já ativa (`ROS_DOMAIN_ID=69`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`):

    python3 tools/diagnostics/tf_lidar_probe.py artifacts/tf-lidar-baseline.csv --seconds 180

POR QUE ESTA SONDA EXISTE

`restamp_tf: true` carimba `map -> odom` com o tempo ATUAL em vez do tempo do
scan que gerou a estimativa. Ele destravou o robô congelado (0,0% -> 6,2% de
trabalho em vx, `artifacts/maze11-short-gate*.csv`), mas o custo é um erro de
pose proporcional ao atraso que ele compensa. Sem medir esse atraso não há como
separar "restamp introduz erro" de "o atraso do transporte já era grande" — e
qualquer mexida em `restamp_tf` antes dessa medição é chute.

Esta sonda NÃO publica `/demo/cmd_vel`, não manda meta e não entra em launch de
produção. Ela só escuta.

O QUE ELA MEDE, E POR QUE CADA COISA ESTÁ AQUI

- **Idade contra o relógio de SIMULAÇÃO, não o de parede.** É o relógio que o
  buffer de TF, o costmap e o `collision_monitor` usam. Idade medida em tempo de
  parede responderia a outra pergunta.
- **Uma linha por NUVEM, não por amostra de taxa fixa.** A nuvem é o sujeito da
  medição; amostrar a 20 Hz um tópico de 10 Hz duplicaria cada carimbo e
  transformaria metade dos intervalos em zero.
- **OS TRÊS PARES, na mesma linha e no mesmo carimbo.** `base <- lidar`
  (estática, do `robot_state_publisher`), `odom <- base` (dinâmica, do
  `odom_tf`) e `odom <- lidar` (a cadeia composta, que é o que a
  `ObstacleLayer` realmente faz com a nuvem). Medir os três na mesma consulta é
  o que permite atribuir uma falha da cadeia a uma aresta.

  Isto ANTES era feito com três execuções separadas da sonda, com durações
  diferentes, e a comparação não valia: uma corrida de 15 s e outra de 180 s
  vêem regimes de carga diferentes, e a diferença entre 95,30% e 78,11% podia
  ser inteiramente disso. Um número plausível e incomparável é pior que um
  número ausente — o mesmo defeito que a taxa da odometria já tinha tido.

- **`odom_tf_age_ms`: a idade da aresta que falha.** É `carimbo_da_nuvem -
  carimbo_mais_recente_de_odom->base`. Separa "a árvore inteira está atrasada"
  de "a aresta dinâmica está atrasada", que são hipóteses diferentes com
  correções diferentes.

- **Regularidade de `odom -> base`, amostrada a 200 Hz.** Um publicador que
  carimba regularmente mas ENTREGA em rajadas produz exatamente a falha bimodal
  medida em 28/08. Por isso a sonda registra as duas coisas: o intervalo entre
  carimbos DISTINTOS (o que o `odom_tf` diz) e o intervalo entre as PRIMEIRAS
  observações de cada carimbo (quando ele de fato chegou). Divergência entre os
  dois é a assinatura da rajada.

  200 Hz para um sinal de 50 Hz: rápido o bastante para não perder carimbo,
  e é leitura de buffer, não deserialização de mensagem — não acrescenta a
  carga que a sonda existe para medir.
- **`transform_latency_ms` com sinal.** É `carimbo_da_nuvem - carimbo_da_TF_mais
  recente`. POSITIVO significa que a árvore de TF está ATRÁS da nuvem e o
  consumidor precisa extrapolar para frente — que é o modo de falha que derruba
  `FollowPath`. Compare direto com `transform_timeout: 0.2` do `slam_params.yaml`.
- **`cloud_age_ms` com sinal.** Negativo é carimbo no FUTURO, que é sintoma
  próprio (relógio do módulo saltando, ou restamp exagerado) e não pode ser
  escondido por um `abs()`.

POR QUE `rclpy` É IMPORTADO TARDE

`tests/test_tf_lidar_probe.py` roda sem ROS no ambiente (assim como
`tests/test_nav_trial_metrics.py`, que importa `trial_timing` justamente por
isso). Aqui o plano pede um arquivo só, então a separação é feita no tempo do
import: tudo acima de `build_probe()` é Python puro e testável; `rclpy` só entra
dentro dela.

No HIL a sonda mede a planta simulada no host e a pilha que roda no Aquila. Os
números valem para essa pilha distribuída; não validam térmica nem desempenho
isolado do módulo (regras 5 e 7 do `CLAUDE.md`).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections.abc import Sequence
from typing import Any


FIELDS = (
    'wall_s',
    'sim_s',
    'cloud_stamp_s',
    'cloud_age_ms',
    'odom_stamp_s',
    'odom_age_ms',
    'cloud_interval_ms',
    'odom_interval_ms',
    'transform_available',
    'transform_latency_ms',
    # Os tres pares, consultados no MESMO carimbo da nuvem, na mesma linha.
    'tf_base_lidar',
    'tf_odom_base',
    'tf_odom_lidar',
    'odom_tf_stamp_s',
    'odom_tf_age_ms',
    'cloud_points',
)

# Critérios da Etapa 1 do plano de fechamento. Ficam aqui, e não espalhados pelo
# resumo, para que mudar um critério seja uma linha só.
CLOUD_RATE_HZ_RANGE = (9.0, 10.0)
ODOM_RATE_HZ_RANGE = (49.0, 50.0)
# As faixas acima sao nominais. A taxa medida oscila em torno do nominal por
# jitter de publicacao, e 10,004 Hz nao e uma reprovacao de uma faixa que termina
# em 10 -- foi o que o primeiro ensaio na bancada marcou como XX. A tolerancia e
# relativa para servir as duas faixas, que estao a uma ordem de grandeza de
# distancia uma da outra.
RATE_TOLERANCE = 0.02
CLOUD_AGE_MEDIAN_MAX_MS = 150.0
TRANSFORM_AVAILABLE_MIN_PCT = 99.5


def age_ms(now_s: float, stamp_s: float) -> float:
    """Return signed age in milliseconds: positive is stale, negative is future.

    The sign is load-bearing. A negative age means the message carries a
    timestamp ahead of the clock reading it, which is a different fault from a
    late message and must not be folded into the same number.
    """
    if not math.isfinite(now_s) or not math.isfinite(stamp_s):
        raise ValueError('timestamps must be finite')
    return (now_s - stamp_s) * 1000.0


def is_future(age_value_ms: float, tolerance_ms: float = 0.0) -> bool:
    """Return True when the sample carries a timestamp ahead of the clock."""
    if tolerance_ms < 0.0:
        raise ValueError('tolerance must be non-negative')
    return age_value_ms < -tolerance_ms


def percentile(values: Sequence[float], q: float) -> float:
    """Return the linearly interpolated ``q``-th percentile of ``values``.

    Dependency-free on purpose: these tests run without ROS and without numpy.
    The interpolation matches numpy's default so the numbers stay comparable to
    anything analysed later with pandas.
    """
    if not values:
        raise ValueError('at least one value is required')
    if not 0.0 <= q <= 100.0:
        raise ValueError('percentile must be within [0, 100]')
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * (q / 100.0)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] + (ordered[upper] - ordered[lower]) * weight)


def unique_stamps(stamps: Sequence[float]) -> list[float]:
    """Drop consecutive repeats, keeping order.

    A stamp repeats whenever a row was written before a fresh message replaced
    the previous one. Counting those repeats as arrivals would inflate the rate
    and report a zero-length gap that never happened.
    """
    result: list[float] = []
    for stamp in stamps:
        if not result or stamp != result[-1]:
            result.append(float(stamp))
    return result


def intervals_ms(stamps: Sequence[float]) -> list[float]:
    """Return consecutive gaps, in milliseconds, between distinct stamps."""
    distinct = unique_stamps(stamps)
    return [(second - first) * 1000.0
            for first, second in zip(distinct, distinct[1:])]


def rate_hz(stamps: Sequence[float]) -> float:
    """Return the mean arrival rate implied by distinct stamps.

    Derived from the stamp span rather than from the row count so that a probe
    interrupted mid-run still reports the rate it actually observed.
    """
    distinct = unique_stamps(stamps)
    if len(distinct) < 2:
        raise ValueError('at least two distinct stamps are required')
    span_s = distinct[-1] - distinct[0]
    if span_s <= 0.0:
        raise ValueError('stamps did not advance')
    return (len(distinct) - 1) / span_s


def _summarise_tf_samples(
        samples: Sequence[tuple[float, float]]) -> dict[str, Any]:
    """Reduce the 200 Hz `odom -> base` sampler to regularity statistics.

    Two series, deliberately not one:

    - the STAMP intervals are what `odom_tf` claims about its own cadence;
    - the ARRIVAL intervals are when this process first saw each stamp.

    A publisher that stamps every 20 ms but delivers in bursts shows a tight
    stamp series next to a ragged arrival series. That is precisely the shape
    that produces the bimodal availability measured on 28/08/2026, and folding
    the two into one number would hide it.
    """
    result: dict[str, Any] = {'odom_tf_samples': len(samples)}
    if len(samples) < 2:
        return result
    arrivals = [float(wall) for wall, _ in samples]
    stamps = [float(stamp) for _, stamp in samples]

    stamp_gaps = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
    arrival_gaps = [(b - a) * 1000.0 for a, b in zip(arrivals, arrivals[1:])]
    for gaps, prefix in ((stamp_gaps, 'odom_tf_stamp_interval'),
                         (arrival_gaps, 'odom_tf_arrival_interval')):
        result[f'{prefix}_median_ms'] = percentile(gaps, 50.0)
        result[f'{prefix}_p99_ms'] = percentile(gaps, 99.0)
        result[f'{prefix}_max_ms'] = max(gaps)

    # Rate from the stamp span, for the same reason `rate_hz` does it: a run cut
    # short still reports the rate it observed rather than a rate diluted by
    # time it never sampled.
    span_s = stamps[-1] - stamps[0]
    if span_s > 0.0:
        result['odom_tf_rate_hz'] = (len(stamps) - 1) / span_s
    return result


def _available_pct(rows: Sequence[dict[str, Any]], field: str) -> float | None:
    """Return the percentage of rows where ``field`` reported an available TF.

    Missing on purpose rather than zero when the column is absent: CSVs written
    before the three pairs were measured together have no such column, and
    reporting 0% for "not measured" would invent a regression.
    """
    values = [row[field] for row in rows if row.get(field, '') != '']
    if not values:
        return None
    return 100.0 * sum(int(value) for value in values) / len(values)


def summarise(rows: Sequence[dict[str, Any]],
              tf_samples: Sequence[tuple[float, float]] | None = None,
              ) -> dict[str, Any]:
    """Reduce sampled rows to the summary the probe prints once, at the end.

    Returns ``samples == 0`` with every metric ``None`` for an empty run rather
    than raising: a probe that captured nothing must still be able to say so.

    ``tf_samples`` are ``(wall_s, stamp_s)`` pairs from the 200 Hz sampler, one
    per DISTINCT ``odom -> base`` stamp observed. They do not belong in ``rows``
    because they are not per-cloud: a cloud arrives at 10 Hz and this edge is
    published at 50 Hz, so folding them into the same table would either drop
    four samples out of five or repeat each cloud five times.
    """
    summary: dict[str, Any] = {
        'samples': len(rows),
        'cloud_rate_hz': None,
        'odom_rate_hz': None,
        'cloud_age_median_ms': None,
        'cloud_age_p95_ms': None,
        'cloud_age_p99_ms': None,
        'cloud_age_max_ms': None,
        'cloud_gap_max_ms': None,
        'transform_available_pct': None,
        'transform_latency_p99_ms': None,
        'future_stamp_pct': None,
        'real_time_factor': None,
        'tf_base_lidar_pct': None,
        'tf_odom_base_pct': None,
        'tf_odom_lidar_pct': None,
        'odom_tf_age_median_ms': None,
        'odom_tf_age_p99_ms': None,
        'odom_tf_samples': 0,
        'odom_tf_rate_hz': None,
        'odom_tf_stamp_interval_median_ms': None,
        'odom_tf_stamp_interval_p99_ms': None,
        'odom_tf_stamp_interval_max_ms': None,
        'odom_tf_arrival_interval_median_ms': None,
        'odom_tf_arrival_interval_p99_ms': None,
        'odom_tf_arrival_interval_max_ms': None,
    }
    summary.update(_summarise_tf_samples(tf_samples or ()))
    if not rows:
        return summary

    ages = [float(row['cloud_age_ms']) for row in rows]
    summary['cloud_age_median_ms'] = percentile(ages, 50.0)
    summary['cloud_age_p95_ms'] = percentile(ages, 95.0)
    summary['cloud_age_p99_ms'] = percentile(ages, 99.0)
    summary['cloud_age_max_ms'] = max(ages)
    summary['future_stamp_pct'] = 100.0 * sum(
        is_future(value) for value in ages) / len(ages)

    available = [int(row['transform_available']) for row in rows]
    summary['transform_available_pct'] = 100.0 * sum(available) / len(available)

    latencies = [float(row['transform_latency_ms']) for row in rows
                 if row['transform_latency_ms'] != '']
    if latencies:
        summary['transform_latency_p99_ms'] = percentile(latencies, 99.0)

    cloud_stamps = [float(row['cloud_stamp_s']) for row in rows]
    gaps = intervals_ms(cloud_stamps)
    if gaps:
        summary['cloud_gap_max_ms'] = max(gaps)
    try:
        summary['cloud_rate_hz'] = rate_hz(cloud_stamps)
    except ValueError:
        pass

    # A taxa da odometria SAI DOS INTERVALOS, nao da coluna de carimbos.
    #
    # Ha uma linha por NUVEM (~10 Hz) e a coluna `odom_stamp_s` guarda o ultimo
    # carimbo de odometria visto naquele instante. Derivar a taxa dela limita o
    # resultado a taxa da nuvem por construcao: medido contra a bancada em
    # 28/08/2026, dava 10,00 Hz para uma odometria que corre a ~50 Hz. O erro nao
    # era do robo, era da sonda -- e um numero plausivel e errado e pior que um
    # numero ausente.
    #
    # `odom_interval_ms` e escrito no callback da odometria, entre carimbos
    # DISTINTOS consecutivos, e nao sofre desse teto. A mediana dele e robusta a
    # uma perda isolada de mensagem, que a media nao seria.
    odom_intervals = [float(row['odom_interval_ms']) for row in rows
                      if row['odom_interval_ms'] != '']
    if odom_intervals:
        median_ms = percentile(odom_intervals, 50.0)
        if median_ms > 0.0:
            summary['odom_rate_hz'] = 1000.0 / median_ms

    for field, key in (('tf_base_lidar', 'tf_base_lidar_pct'),
                       ('tf_odom_base', 'tf_odom_base_pct'),
                       ('tf_odom_lidar', 'tf_odom_lidar_pct')):
        summary[key] = _available_pct(rows, field)

    odom_tf_ages = [float(row['odom_tf_age_ms']) for row in rows
                    if row.get('odom_tf_age_ms', '') != '']
    if odom_tf_ages:
        summary['odom_tf_age_median_ms'] = percentile(odom_tf_ages, 50.0)
        summary['odom_tf_age_p99_ms'] = percentile(odom_tf_ages, 99.0)

    sim_span = float(rows[-1]['sim_s']) - float(rows[0]['sim_s'])
    wall_span = float(rows[-1]['wall_s']) - float(rows[0]['wall_s'])
    if wall_span > 0.0:
        summary['real_time_factor'] = sim_span / wall_span
    return summary


def _verdict(value: float | None, low: float, high: float,
             tolerance: float = 0.0) -> str:
    """Render a pass/fail marker for a value against a nominal band."""
    if value is None:
        return '  ?'
    margin_low = low * (1.0 - tolerance)
    margin_high = high * (1.0 + tolerance) if math.isfinite(high) else high
    return ' ok' if margin_low <= value <= margin_high else ' XX'


def format_summary(summary: dict[str, Any]) -> str:
    """Render the single block the probe prints when it stops."""
    if not summary['samples']:
        return ('sonda temporal: NENHUMA amostra capturada.\n'
                '  Verifique ROS_DOMAIN_ID, RMW e se a nuvem está publicando.')

    def number(key: str, digits: int = 1) -> str:
        value = summary[key]
        return '  n/d' if value is None else f'{value:.{digits}f}'

    lines = [
        f'sonda temporal: {summary["samples"]} amostras',
        f'  taxa da nuvem          {number("cloud_rate_hz", 2)} Hz'
        f'{_verdict(summary["cloud_rate_hz"], *CLOUD_RATE_HZ_RANGE, RATE_TOLERANCE)}'
        f'   (esperado {CLOUD_RATE_HZ_RANGE[0]:.0f}-{CLOUD_RATE_HZ_RANGE[1]:.0f})',
        f'  taxa da odometria      {number("odom_rate_hz", 2)} Hz'
        f'{_verdict(summary["odom_rate_hz"], *ODOM_RATE_HZ_RANGE, RATE_TOLERANCE)}'
        f'   (esperado {ODOM_RATE_HZ_RANGE[0]:.0f}-{ODOM_RATE_HZ_RANGE[1]:.0f})',
        f'  idade mediana          {number("cloud_age_median_ms")} ms'
        f'{_verdict(summary["cloud_age_median_ms"], -math.inf, CLOUD_AGE_MEDIAN_MAX_MS)}'
        f'   (limite {CLOUD_AGE_MEDIAN_MAX_MS:.0f})',
        f'  idade p95              {number("cloud_age_p95_ms")} ms',
        f'  idade p99              {number("cloud_age_p99_ms")} ms',
        f'  idade máxima           {number("cloud_age_max_ms")} ms',
        f'  maior lacuna de nuvem  {number("cloud_gap_max_ms")} ms',
        f'  TF disponível          {number("transform_available_pct", 2)} %'
        f'{_verdict(summary["transform_available_pct"], TRANSFORM_AVAILABLE_MIN_PCT, 100.0)}'
        f'   (mínimo {TRANSFORM_AVAILABLE_MIN_PCT})',
        f'  TF atrás da nuvem p99  {number("transform_latency_p99_ms")} ms',
        f'  carimbos no futuro     {number("future_stamp_pct", 2)} %',
        f'  fator de tempo real    {number("real_time_factor", 3)}',
        '  --- os três pares, mesma corrida, mesmo carimbo ---',
        f'  base <- lidar          {number("tf_base_lidar_pct", 2)} %'
        '   (estática; espera-se 100)',
        f'  odom <- base           {number("tf_odom_base_pct", 2)} %'
        f'{_verdict(summary["tf_odom_base_pct"], TRANSFORM_AVAILABLE_MIN_PCT, 100.0)}'
        '   (dinâmica, odom_tf)',
        f'  odom <- lidar          {number("tf_odom_lidar_pct", 2)} %'
        f'{_verdict(summary["tf_odom_lidar_pct"], TRANSFORM_AVAILABLE_MIN_PCT, 100.0)}'
        '   (composta, ObstacleLayer)',
        f'  idade de odom->base    {number("odom_tf_age_median_ms")} ms'
        f'  (p99 {number("odom_tf_age_p99_ms")})',
        '  --- regularidade de odom->base (amostrador 200 Hz) ---',
        f'  amostras / taxa        {summary["odom_tf_samples"]}'
        f'  /  {number("odom_tf_rate_hz", 2)} Hz',
        f'  intervalo por carimbo  {number("odom_tf_stamp_interval_median_ms")} ms'
        f'  (p99 {number("odom_tf_stamp_interval_p99_ms")},'
        f' máx {number("odom_tf_stamp_interval_max_ms")})',
        f'  intervalo por chegada  {number("odom_tf_arrival_interval_median_ms")} ms'
        f'  (p99 {number("odom_tf_arrival_interval_p99_ms")},'
        f' máx {number("odom_tf_arrival_interval_max_ms")})',
    ]
    return '\n'.join(lines)


def write_csv(path: str, rows: Sequence[dict[str, Any]]) -> None:
    """Write ``rows`` under the fixed ``FIELDS`` header.

    The header is written even for an empty run, so a failed capture leaves a
    file that says "nothing arrived" instead of no file at all.
    """
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(FIELDS))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in FIELDS})


def build_probe(args: argparse.Namespace):
    """Construct the rclpy node. Imports ROS lazily — see the module docstring."""
    import rclpy
    from rclpy.clock import Clock
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    import tf2_ros

    def stamp_seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec / 1e9

    class Probe(Node):
        """Listens only. Never publishes a command, a goal or a transform."""

        def __init__(self) -> None:
            super().__init__('tf_lidar_probe')
            # Sim time is not optional here: the ages this probe exists to
            # measure are the ones TF and the costmap see, and those run on
            # /clock. Reading them against wall time answers another question.
            self.set_parameters([rclpy.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

            self.rows: list[dict[str, Any]] = []
            self._wall_start: float | None = None
            self._sim_start: float | None = None
            self._last_cloud_stamp: float | None = None
            self._last_odom_stamp: float | None = None
            self._odom_stamp: float | None = None
            self._odom_interval_ms: float | None = None
            self._target = args.target_frame
            self._odom_frame = args.odom_frame
            self._base_frame = args.base_frame

            # (wall_s, stamp_s) por carimbo DISTINTO de odom -> base. Ver
            # `_summarise_tf_samples` para por que as duas colunas.
            self.tf_samples: list[tuple[float, float]] = []
            self._last_tf_stamp: float | None = None

            self._buffer = tf2_ros.Buffer()
            self._listener = tf2_ros.TransformListener(self._buffer, self)

            self.create_subscription(
                Odometry, args.odom_topic, self._on_odom,
                QoSPresetProfiles.SENSOR_DATA.value)
            self.create_subscription(
                PointCloud2, args.cloud_topic, self._on_cloud,
                QoSPresetProfiles.SENSOR_DATA.value)

            # Amostrador da aresta dinamica. Timer de PARADE, nao de simulacao:
            # a pergunta e quando a mensagem chegou a este processo, e uma pausa
            # do /clock nao pode ser lida como uma rajada do publicador.
            self._tf_timer = self.create_timer(
                1.0 / args.tf_sample_hz, self._sample_odom_tf,
                clock=Clock())

        def _sim_now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _odom_tf_stamp(self) -> float | None:
            """Latest `odom -> base` stamp in the buffer, or None if absent."""
            try:
                latest = self._buffer.lookup_transform(
                    self._odom_frame, self._base_frame, Time())
            except tf2_ros.TransformException:
                return None
            return stamp_seconds(latest.header.stamp)

        def _sample_odom_tf(self) -> None:
            stamp = self._odom_tf_stamp()
            if stamp is None or stamp == self._last_tf_stamp:
                return
            self._last_tf_stamp = stamp
            self.tf_samples.append((time.monotonic(), stamp))

        def _on_odom(self, message: Odometry) -> None:
            stamp = stamp_seconds(message.header.stamp)
            if self._last_odom_stamp is not None and stamp != self._last_odom_stamp:
                self._odom_interval_ms = (stamp - self._last_odom_stamp) * 1000.0
            self._last_odom_stamp = stamp
            self._odom_stamp = stamp

        def _on_cloud(self, message: PointCloud2) -> None:
            # Odom has to be seen first, or the row would carry an empty column
            # that every downstream awk one-liner would have to special-case.
            if self._odom_stamp is None:
                return

            wall = time.monotonic()
            sim = self._sim_now()
            if self._wall_start is None:
                self._wall_start = wall
                self._sim_start = sim

            stamp = stamp_seconds(message.header.stamp)
            interval = ''
            if self._last_cloud_stamp is not None:
                interval = round((stamp - self._last_cloud_stamp) * 1000.0, 1)
            self._last_cloud_stamp = stamp

            source = args.source_frame or message.header.frame_id
            query = Time.from_msg(message.header.stamp)
            available = self._buffer.can_transform(self._target, source, query)
            latency: float | str = ''
            try:
                latest = self._buffer.lookup_transform(
                    self._target, source, Time())
                latency = round(
                    (stamp - stamp_seconds(latest.header.stamp)) * 1000.0, 1)
            except tf2_ros.TransformException:
                pass

            # Os tres pares, no MESMO carimbo e na mesma volta do callback. E o
            # que torna a atribuicao valida: qualquer diferenca entre eles e da
            # aresta, nao do instante em que cada um foi perguntado.
            pairs = {
                'tf_base_lidar': (self._base_frame, source),
                'tf_odom_base': (self._odom_frame, self._base_frame),
                'tf_odom_lidar': (self._odom_frame, source),
            }
            availability = {
                field: int(self._buffer.can_transform(target, child, query))
                for field, (target, child) in pairs.items()
            }

            odom_tf_stamp = self._odom_tf_stamp()

            self.rows.append({
                'wall_s': round(wall - self._wall_start, 3),
                'sim_s': round(sim - self._sim_start, 3),
                'cloud_stamp_s': round(stamp, 3),
                'cloud_age_ms': round(age_ms(sim, stamp), 1),
                'odom_stamp_s': round(self._odom_stamp, 3),
                'odom_age_ms': round(age_ms(sim, self._odom_stamp), 1),
                'cloud_interval_ms': interval,
                'odom_interval_ms': ('' if self._odom_interval_ms is None
                                     else round(self._odom_interval_ms, 1)),
                'transform_available': int(available),
                'transform_latency_ms': latency,
                **availability,
                'odom_tf_stamp_s': ('' if odom_tf_stamp is None
                                    else round(odom_tf_stamp, 3)),
                'odom_tf_age_ms': ('' if odom_tf_stamp is None else
                                   round(age_ms(stamp, odom_tf_stamp), 1)),
                'cloud_points': message.width * message.height,
            })

    return rclpy, Probe


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('csv', help='arquivo CSV de saída')
    parser.add_argument('--seconds', type=float, default=60.0,
                        help='duração da captura em tempo de parede (default 60)')
    parser.add_argument('--cloud-topic', default='/demo/scan_cloud')
    parser.add_argument('--odom-topic', default='/demo/odom')
    parser.add_argument('--target-frame', default='odom',
                        help='destino da transformação medida (default odom)')
    parser.add_argument('--source-frame', default='',
                        help='origem; vazio usa o frame_id da própria nuvem')
    parser.add_argument('--odom-frame', default='odom',
                        help='quadro de odometria (default odom)')
    parser.add_argument('--base-frame', default='base',
                        help='quadro do corpo (default base)')
    parser.add_argument('--tf-sample-hz', type=float, default=200.0,
                        help='taxa do amostrador de odom->base (default 200); '
                             'precisa ficar acima da taxa de publicação para '
                             'não perder carimbo')
    parser.add_argument('--odom-tf-csv', default='',
                        help='opcional: série completa (chegada, carimbo) do '
                             'amostrador. Só é necessária quando o resumo '
                             'estatístico não decidir a questão')
    args = parser.parse_args(argv)
    if args.seconds <= 0.0:
        parser.error('--seconds deve ser positivo')
    if args.tf_sample_hz <= 0.0:
        parser.error('--tf-sample-hz deve ser positivo')

    rclpy, probe_class = build_probe(args)
    rclpy.init()
    probe = probe_class()
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rows = list(probe.rows)
        tf_samples = list(probe.tf_samples)
        probe.destroy_node()
        rclpy.shutdown()

    write_csv(args.csv, rows)
    if args.odom_tf_csv:
        with open(args.odom_tf_csv, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['arrival_wall_s', 'odom_tf_stamp_s'])
            writer.writerows(
                (round(wall, 4), round(stamp, 4)) for wall, stamp in tf_samples)
    print(format_summary(summarise(rows, tf_samples)))
    return 0 if rows else 1


if __name__ == '__main__':
    sys.exit(main())
