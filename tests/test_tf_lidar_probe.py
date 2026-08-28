"""Unit tests for the pure helpers of scripts/tf_lidar_probe.py.

These run WITHOUT ROS. That is the reason `tf_lidar_probe` imports `rclpy`
inside `build_probe()` instead of at module scope — importing the module here
must not require a sourced Jazzy environment.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from tf_lidar_probe import (  # noqa: E402
    FIELDS,
    age_ms,
    format_summary,
    intervals_ms,
    is_future,
    percentile,
    rate_hz,
    summarise,
    unique_stamps,
    write_csv,
)


def _row(**overrides):
    row = {
        'wall_s': 0.0,
        'sim_s': 0.0,
        'cloud_stamp_s': 0.0,
        'cloud_age_ms': 0.0,
        'odom_stamp_s': 0.0,
        'odom_age_ms': 0.0,
        'cloud_interval_ms': '',
        'odom_interval_ms': 20.0,
        'transform_available': 1,
        'transform_latency_ms': 0.0,
        'tf_base_lidar': 1,
        'tf_odom_base': 1,
        'tf_odom_lidar': 1,
        'odom_tf_stamp_s': 0.0,
        'odom_tf_age_ms': 0.0,
        'cloud_points': 10240,
    }
    row.update(overrides)
    return row


# --------------------------------------------------------------------------
# cálculo de idade
# --------------------------------------------------------------------------

def test_age_is_reported_in_milliseconds_and_signed() -> None:
    assert age_ms(100.150, 100.000) == pytest.approx(150.0)


def test_age_of_a_future_stamp_stays_negative_instead_of_being_folded() -> None:
    # A stamp ahead of the clock is a different fault from a late message.
    # abs() here would report a 40 ms delay that never happened.
    assert age_ms(100.000, 100.040) == pytest.approx(-40.0)


@pytest.mark.parametrize('now_s, stamp_s', [
    (float('nan'), 1.0),
    (1.0, float('inf')),
])
def test_age_rejects_non_finite_timestamps(now_s, stamp_s) -> None:
    with pytest.raises(ValueError):
        age_ms(now_s, stamp_s)


# --------------------------------------------------------------------------
# detecção de timestamp futuro
# --------------------------------------------------------------------------

def test_future_stamp_is_detected_from_a_negative_age() -> None:
    assert is_future(-0.5) is True
    assert is_future(0.0) is False
    assert is_future(120.0) is False


def test_future_tolerance_absorbs_sub_millisecond_clock_jitter() -> None:
    assert is_future(-0.4, tolerance_ms=1.0) is False
    assert is_future(-1.5, tolerance_ms=1.0) is True


def test_future_tolerance_must_be_non_negative() -> None:
    with pytest.raises(ValueError):
        is_future(-1.0, tolerance_ms=-1.0)


# --------------------------------------------------------------------------
# percentis
# --------------------------------------------------------------------------

def test_percentile_interpolates_linearly_between_neighbours() -> None:
    values = [10.0, 20.0, 30.0, 40.0]

    assert percentile(values, 50.0) == pytest.approx(25.0)
    assert percentile(values, 0.0) == pytest.approx(10.0)
    assert percentile(values, 100.0) == pytest.approx(40.0)


def test_percentile_does_not_depend_on_input_order() -> None:
    assert percentile([40.0, 10.0, 30.0, 20.0], 75.0) == pytest.approx(32.5)


def test_percentile_of_a_single_sample_is_that_sample() -> None:
    assert percentile([7.5], 99.0) == pytest.approx(7.5)


@pytest.mark.parametrize('values, q', [
    ([], 50.0),
    ([1.0], -1.0),
    ([1.0], 101.0),
])
def test_percentile_rejects_empty_input_and_out_of_range_quantiles(values, q) -> None:
    with pytest.raises(ValueError):
        percentile(values, q)


# --------------------------------------------------------------------------
# intervalos
# --------------------------------------------------------------------------

def test_intervals_are_consecutive_gaps_in_milliseconds() -> None:
    assert intervals_ms([1.0, 1.1, 1.2]) == pytest.approx([100.0, 100.0])


def test_repeated_stamps_do_not_become_zero_length_gaps() -> None:
    # A stamp repeats while no fresh cloud has replaced the previous one.
    # Counting the repeat would report a 0 ms arrival that never happened.
    assert unique_stamps([1.0, 1.0, 1.1, 1.1, 1.1, 1.2]) == [1.0, 1.1, 1.2]
    assert intervals_ms([1.0, 1.0, 1.1, 1.1, 1.1, 1.2]) == pytest.approx(
        [100.0, 100.0])


def test_a_dropout_shows_up_as_one_long_gap() -> None:
    gaps = intervals_ms([1.0, 1.1, 4.6, 4.7])

    assert max(gaps) == pytest.approx(3500.0)


@pytest.mark.parametrize('stamps', [[], [1.0], [1.0, 1.0]])
def test_intervals_of_fewer_than_two_distinct_stamps_is_empty(stamps) -> None:
    assert intervals_ms(stamps) == []


def test_rate_is_derived_from_the_stamp_span_not_the_row_count() -> None:
    # Ten distinct stamps 0,1 s apart span 0,9 s -> 10 Hz, and the four repeated
    # rows must not inflate it.
    stamps = [round(0.1 * i, 3) for i in range(10)]

    assert rate_hz(stamps) == pytest.approx(10.0)
    assert rate_hz(stamps + [stamps[-1]] * 4) == pytest.approx(10.0)


@pytest.mark.parametrize('stamps', [[], [5.0], [5.0, 5.0]])
def test_rate_needs_two_distinct_advancing_stamps(stamps) -> None:
    with pytest.raises(ValueError):
        rate_hz(stamps)


# --------------------------------------------------------------------------
# resumo
# --------------------------------------------------------------------------

def test_empty_summary_reports_no_samples_instead_of_raising() -> None:
    summary = summarise([])

    assert summary['samples'] == 0
    assert summary['cloud_rate_hz'] is None
    assert summary['cloud_age_median_ms'] is None
    assert summary['real_time_factor'] is None


def test_empty_summary_renders_a_diagnosable_message() -> None:
    rendered = format_summary(summarise([]))

    assert 'NENHUMA amostra' in rendered


def test_odom_rate_comes_from_intervals_not_from_the_sampled_column() -> None:
    # Ha uma linha por NUVEM (~10 Hz) e `odom_stamp_s` guarda o ultimo carimbo
    # visto naquele instante. Derivar a taxa dessa coluna limita o resultado a
    # taxa da nuvem: na bancada isso reportou 10,00 Hz para uma odometria de
    # ~50 Hz. O intervalo, escrito no callback da odometria, nao tem esse teto.
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=1.0, cloud_age_ms=10.0,
             odom_stamp_s=1.00, odom_interval_ms=20.0),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=1.1, cloud_age_ms=10.0,
             odom_stamp_s=1.10, odom_interval_ms=20.0),
        _row(wall_s=0.2, sim_s=0.2, cloud_stamp_s=1.2, cloud_age_ms=10.0,
             odom_stamp_s=1.20, odom_interval_ms=20.0),
    ]

    summary = summarise(rows)

    assert summary['cloud_rate_hz'] == pytest.approx(10.0)
    assert summary['odom_rate_hz'] == pytest.approx(50.0)


def test_odom_rate_uses_the_median_so_one_dropout_does_not_move_it() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=1.0, cloud_age_ms=0.0,
             odom_stamp_s=1.0, odom_interval_ms=20.0),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=1.1, cloud_age_ms=0.0,
             odom_stamp_s=1.1, odom_interval_ms=400.0),
        _row(wall_s=0.2, sim_s=0.2, cloud_stamp_s=1.2, cloud_age_ms=0.0,
             odom_stamp_s=1.2, odom_interval_ms=20.0),
    ]

    assert summarise(rows)['odom_rate_hz'] == pytest.approx(50.0)


def test_odom_rate_is_absent_when_no_interval_was_ever_recorded() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=1.0, cloud_age_ms=0.0,
             odom_interval_ms=''),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=1.1, cloud_age_ms=0.0,
             odom_interval_ms=''),
    ]

    assert summarise(rows)['odom_rate_hz'] is None


def test_rate_verdict_tolerates_jitter_around_the_nominal_band() -> None:
    # 10,004 Hz nao e uma reprovacao de uma faixa nominal que termina em 10.
    rows = [_row(wall_s=0.1 * i, sim_s=0.1 * i,
                 cloud_stamp_s=1.0 + 0.09996 * i, cloud_age_ms=10.0,
                 odom_stamp_s=1.0 + 0.09996 * i, odom_interval_ms=20.0)
            for i in range(20)]

    rendered = format_summary(summarise(rows))
    rate_line = [line for line in rendered.splitlines()
                 if 'taxa da nuvem' in line][0]

    assert 'XX' not in rate_line, rate_line


def test_summary_reports_rates_ages_gaps_and_real_time_factor() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=100.0, cloud_age_ms=50.0,
             odom_stamp_s=100.00, odom_age_ms=5.0, transform_available=1,
             odom_interval_ms=20.0, transform_latency_ms=10.0),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=100.1, cloud_age_ms=60.0,
             odom_stamp_s=100.02, odom_age_ms=6.0, transform_available=1,
             odom_interval_ms=20.0, transform_latency_ms=20.0),
        _row(wall_s=0.2, sim_s=0.2, cloud_stamp_s=100.2, cloud_age_ms=250.0,
             odom_stamp_s=100.04, odom_age_ms=7.0, transform_available=0,
             odom_interval_ms=20.0, transform_latency_ms=30.0),
    ]

    summary = summarise(rows)

    assert summary['samples'] == 3
    assert summary['cloud_rate_hz'] == pytest.approx(10.0)
    assert summary['odom_rate_hz'] == pytest.approx(50.0)
    assert summary['cloud_age_median_ms'] == pytest.approx(60.0)
    assert summary['cloud_age_max_ms'] == pytest.approx(250.0)
    assert summary['cloud_gap_max_ms'] == pytest.approx(100.0)
    assert summary['transform_available_pct'] == pytest.approx(200.0 / 3.0)
    assert summary['real_time_factor'] == pytest.approx(1.0)
    assert summary['future_stamp_pct'] == pytest.approx(0.0)


def test_summary_counts_future_stamps_separately_from_late_ones() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=1.0, cloud_age_ms=-20.0),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=1.1, cloud_age_ms=40.0),
        _row(wall_s=0.2, sim_s=0.2, cloud_stamp_s=1.2, cloud_age_ms=40.0),
        _row(wall_s=0.3, sim_s=0.3, cloud_stamp_s=1.3, cloud_age_ms=40.0),
    ]

    assert summarise(rows)['future_stamp_pct'] == pytest.approx(25.0)


def test_summary_tolerates_rows_without_a_transform_latency() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=1.0, cloud_age_ms=10.0,
             odom_stamp_s=1.0, transform_available=0, transform_latency_ms=''),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=1.1, cloud_age_ms=10.0,
             odom_stamp_s=1.02, transform_available=0, transform_latency_ms=''),
    ]

    summary = summarise(rows)

    assert summary['transform_latency_p99_ms'] is None
    assert summary['transform_available_pct'] == pytest.approx(0.0)


def test_stalled_simulation_clock_gives_a_zero_real_time_factor() -> None:
    rows = [
        _row(wall_s=0.0, sim_s=5.0, cloud_stamp_s=1.0, cloud_age_ms=0.0,
             odom_stamp_s=1.0),
        _row(wall_s=9.0, sim_s=5.0, cloud_stamp_s=1.1, cloud_age_ms=0.0,
             odom_stamp_s=1.1),
    ]

    assert summarise(rows)['real_time_factor'] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# serialização CSV
# --------------------------------------------------------------------------

def test_csv_round_trips_through_summarise(tmp_path) -> None:
    rows = [
        _row(wall_s=0.0, sim_s=0.0, cloud_stamp_s=100.0, cloud_age_ms=50.0,
             odom_stamp_s=100.00, cloud_interval_ms='',
             transform_latency_ms=10.0),
        _row(wall_s=0.1, sim_s=0.1, cloud_stamp_s=100.1, cloud_age_ms=60.0,
             odom_stamp_s=100.02, cloud_interval_ms=100.0,
             transform_latency_ms=20.0),
    ]
    path = tmp_path / 'probe.csv'

    write_csv(str(path), rows)
    with open(path, newline='') as handle:
        reloaded = list(csv.DictReader(handle))

    assert list(reloaded[0]) == list(FIELDS)
    # Reading the file back must yield the same summary as the in-memory rows,
    # or the CSV is evidence of something other than what the probe measured.
    assert summarise(reloaded)['cloud_age_median_ms'] == pytest.approx(
        summarise(rows)['cloud_age_median_ms'])
    assert summarise(reloaded)['transform_available_pct'] == pytest.approx(100.0)


def test_csv_of_an_empty_run_still_carries_the_header(tmp_path) -> None:
    # A capture that caught nothing must leave a file saying so, not no file.
    path = tmp_path / 'empty.csv'

    write_csv(str(path), [])

    assert path.read_text().strip() == ','.join(FIELDS)


def test_csv_columns_match_the_documented_contract() -> None:
    assert FIELDS == (
        'wall_s', 'sim_s', 'cloud_stamp_s', 'cloud_age_ms', 'odom_stamp_s',
        'odom_age_ms', 'cloud_interval_ms', 'odom_interval_ms',
        'transform_available', 'transform_latency_ms',
        'tf_base_lidar', 'tf_odom_base', 'tf_odom_lidar',
        'odom_tf_stamp_s', 'odom_tf_age_ms',
        'cloud_points',
    )


# --------------------------------------------------------------------------
# os três pares na mesma corrida
# --------------------------------------------------------------------------

def test_each_pair_is_reduced_independently() -> None:
    # O caso que interessa: a estática passa sempre, a dinâmica falha às vezes,
    # e a composta não pode passar mais do que a pior das duas.
    rows = [
        _row(tf_base_lidar=1, tf_odom_base=1, tf_odom_lidar=1),
        _row(tf_base_lidar=1, tf_odom_base=0, tf_odom_lidar=0),
        _row(tf_base_lidar=1, tf_odom_base=1, tf_odom_lidar=0),
        _row(tf_base_lidar=1, tf_odom_base=1, tf_odom_lidar=1),
    ]
    summary = summarise(rows)
    assert summary['tf_base_lidar_pct'] == pytest.approx(100.0)
    assert summary['tf_odom_base_pct'] == pytest.approx(75.0)
    assert summary['tf_odom_lidar_pct'] == pytest.approx(50.0)


def test_a_csv_written_before_the_three_pairs_reports_them_as_unmeasured(
) -> None:
    # Os CSVs de 28/08 não têm essas colunas. Reduzi-las a 0% transformaria
    # "não medido" em "reprovou 100% das vezes", que é uma regressão inventada
    # -- e seria lida como tal na comparação A/B.
    legacy = [{key: value for key, value in _row().items()
               if not key.startswith(('tf_', 'odom_tf_'))}]
    summary = summarise(legacy)
    assert summary['tf_base_lidar_pct'] is None
    assert summary['tf_odom_base_pct'] is None
    assert summary['tf_odom_lidar_pct'] is None
    assert summary['odom_tf_age_median_ms'] is None
    # E o que a corrida antiga MEDIU continua sendo lido.
    assert summary['transform_available_pct'] == pytest.approx(100.0)


def test_the_dynamic_edge_age_is_positive_when_the_edge_is_behind() -> None:
    rows = [_row(odom_tf_age_ms=60.0), _row(odom_tf_age_ms=160.0)]
    summary = summarise(rows)
    assert summary['odom_tf_age_median_ms'] == pytest.approx(110.0)
    assert summary['odom_tf_age_p99_ms'] == pytest.approx(159.0)


# --------------------------------------------------------------------------
# regularidade de odom -> base
# --------------------------------------------------------------------------

def _samples(pairs):
    return [(float(wall), float(stamp)) for wall, stamp in pairs]


def test_a_regular_publisher_shows_matching_stamp_and_arrival_series() -> None:
    samples = _samples([(i * 0.020, 100.0 + i * 0.020) for i in range(51)])
    summary = summarise([_row()], samples)
    assert summary['odom_tf_samples'] == 51
    assert summary['odom_tf_rate_hz'] == pytest.approx(50.0)
    assert summary['odom_tf_stamp_interval_median_ms'] == pytest.approx(20.0)
    assert summary['odom_tf_arrival_interval_median_ms'] == pytest.approx(20.0)
    assert summary['odom_tf_arrival_interval_max_ms'] == pytest.approx(20.0)


def test_a_bursting_publisher_shows_a_tight_stamp_and_a_ragged_arrival(
) -> None:
    # Esta é a assinatura que o A/B do TF precisa distinguir: o `odom_tf`
    # carimba a cada 20 ms e ENTREGA em rajadas de cinco. A série de carimbos
    # continua perfeita; só a de chegada acusa. Um resumo que colapsasse as duas
    # num número só declararia o publicador saudável.
    pairs = []
    for burst in range(10):
        arrival = burst * 0.100
        for index in range(5):
            pairs.append((arrival, 100.0 + (burst * 5 + index) * 0.020))
    summary = summarise([_row()], _samples(pairs))

    assert summary['odom_tf_stamp_interval_median_ms'] == pytest.approx(20.0)
    assert summary['odom_tf_stamp_interval_max_ms'] == pytest.approx(20.0)
    assert summary['odom_tf_arrival_interval_median_ms'] == pytest.approx(0.0)
    assert summary['odom_tf_arrival_interval_max_ms'] == pytest.approx(100.0)


def test_a_run_that_saw_one_stamp_reports_no_intervals_rather_than_zero(
) -> None:
    summary = summarise([_row()], _samples([(0.0, 100.0)]))
    assert summary['odom_tf_samples'] == 1
    assert summary['odom_tf_rate_hz'] is None
    assert summary['odom_tf_stamp_interval_median_ms'] is None


def test_the_sampler_series_is_absent_by_default_and_says_so() -> None:
    summary = summarise([_row()])
    assert summary['odom_tf_samples'] == 0
    assert summary['odom_tf_arrival_interval_p99_ms'] is None


def test_an_empty_run_still_carries_the_sampler_keys() -> None:
    # `summarise([])` volta cedo; as chaves novas têm de existir mesmo assim,
    # ou `format_summary` explode justamente na corrida que falhou.
    summary = summarise([])
    for key in ('tf_odom_base_pct', 'odom_tf_samples',
                'odom_tf_arrival_interval_max_ms'):
        assert key in summary


def test_the_summary_renders_every_new_line_without_a_sampler() -> None:
    text = format_summary(summarise([_row()]))
    assert 'odom <- base' in text
    assert 'intervalo por chegada' in text
    assert 'n/d' in text
