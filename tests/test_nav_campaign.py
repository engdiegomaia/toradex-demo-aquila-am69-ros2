"""
Locks down the navigation campaign's PROTOCOL.

Only one thing here is testable without a simulator, and it is exactly the
thing that breaks silently: the ORDER of the legs. A blocked campaign
(`A A A B B B`) runs just the same, writes the same files, produces a
summary that looks identical — and attributes to the condition any thermal,
cache, or network drift that occurred over the course of the campaign. With a
measured spread of 2.4x under IDENTICAL configuration
(`docs/results/ml35-f5-clock-fanout.md`), this is not theoretical rigor: it is
the difference between measuring the effect and measuring the time of day.
"""

import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'evaluation' / 'nav_campaign.py'


def _load():
    spec = importlib.util.spec_from_file_location('nav_campaign', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def campaign():
    return _load()


def test_order_is_interleaved_not_blocked(campaign):
    conditions = [('baseline', None), ('align8', 'cmd')]
    names = [name for _, name, _ in campaign.leg_order(conditions, 3)]
    assert names == ['baseline', 'align8'] * 3
    # The opposite of what is wanted, written so it doesn't pass by accident:
    assert names != ['baseline'] * 3 + ['align8'] * 3


def test_every_condition_gets_the_same_number_of_replicates(campaign):
    conditions = [('a', None), ('b', 'x'), ('c', 'y')]
    names = [name for _, name, _ in campaign.leg_order(conditions, 4)]
    assert len(names) == 12
    assert {names.count(n) for n in ('a', 'b', 'c')} == {4}


def test_replicates_are_numbered_in_order(campaign):
    reps = [rep for rep, _, _ in campaign.leg_order([('a', None)], 3)]
    assert reps == [1, 2, 3]


@pytest.mark.parametrize('raw, expected', [
    ('baseline', ('baseline', None)),
    ('align8=ros2 param set /x y 8.0', ('align8', 'ros2 param set /x y 8.0')),
    # An empty `=` means "apply an empty command", which is different from
    # "apply nothing" — and the difference has to survive the parse.
    ('empty=', ('empty', '')),
])
def test_parse_condition(campaign, raw, expected):
    assert campaign.parse_condition(raw) == expected


def test_condition_without_name_is_an_error(campaign):
    with pytest.raises(Exception):
        campaign.parse_condition('=command')


def test_comparison_requires_reapplying_every_condition(campaign):
    with pytest.raises(ValueError, match='baseline'):
        campaign.validate_conditions([
            ('baseline', None),
            ('align8', 'apply-align8'),
        ])


def test_comparison_accepts_explicit_commands(campaign):
    campaign.validate_conditions([
        ('baseline', 'apply-baseline'),
        ('align8', 'apply-align8'),
    ])


def test_single_condition_can_use_current_state(campaign):
    campaign.validate_conditions([('smoke', None)])


def test_route_with_negative_coordinate_is_forwarded_with_equals(campaign, monkeypatch,
                                                                  tmp_path):
    calls = []

    monkeypatch.setattr(campaign, 'reset_between_legs', lambda **_: True)
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes', lambda *a, **k: (True, 0.0))

    def fake_run(command, timeout):
        calls.append(command)
        if 'nav_trial.py' in ' '.join(command):
            Path(command[2]).touch()
        return 0, ''

    monkeypatch.setattr(campaign, '_run', fake_run)

    assert campaign.main([
        str(tmp_path), '--condition', 'route', '--reps', '1',
        '--seconds', '1', '--goals=-1.50,0.05',
    ]) == 0
    trial = next(command for command in calls if 'nav_trial.py' in ' '.join(command))
    assert '--goals=-1.50,0.05' in trial


def test_goal_timeout_is_forwarded_to_each_trial(campaign, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(campaign, 'reset_between_legs', lambda **_: True)
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes', lambda *a, **k: (True, 0.0))

    def fake_run(command, timeout):
        calls.append(command)
        if 'nav_trial.py' in ' '.join(command):
            Path(command[2]).touch()
        return 0, ''

    monkeypatch.setattr(campaign, '_run', fake_run)
    assert campaign.main([
        str(tmp_path), '--condition', 'warm', '--reps', '1',
        '--seconds', '1', '--goal-timeout', '300',
    ]) == 0
    trial = next(command for command in calls if 'nav_trial.py' in ' '.join(command))
    assert trial[-2:] == ['--goal-timeout', '300.0']


def test_simulator_reset_is_required(campaign, monkeypatch):
    """
    A refusal during the reset has to ABORT, not just warn.

    A leg that starts from a different state than the others is not a
    replicate, and including it in the median ruins the single number the
    campaign produces.
    """
    monkeypatch.setattr(campaign, 'SETTLE_S', 0.0)
    monkeypatch.setattr(
        campaign, 'call_trigger',
        lambda service, timeout=30.0: (False, 'refused'),
    )
    assert campaign.reset_between_legs(skip_nav=True, verbose=False) is False


def test_successful_reset_returns_true(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'SETTLE_S', 0.0)
    calls = []

    def fake(service, timeout=30.0):
        calls.append(service)
        return True, 'success=True'

    monkeypatch.setattr(campaign, 'call_trigger', fake)
    assert campaign.reset_between_legs(skip_nav=False, verbose=False) is True
    # Both, and in this order: reset the robot before clearing the costmap,
    # so the new costmap is already born from the new pose.
    assert calls == [campaign.SIM_RESET, campaign.NAV_RESET]


def test_success_false_does_not_count_as_accepted(campaign, monkeypatch):
    """
    `ros2 service call` exits 0 even when the service responded success=False.

    Trusting the exit code would let the campaign continue after a reset that
    never happened — and a reset that doesn't happen is invisible on screen.
    """
    monkeypatch.setattr(
        campaign, '_run',
        lambda command, timeout: (0, 'response:\nTrigger_Response(success=False'),
    )
    accepted, _ = campaign.call_trigger('/demo/sim/reset')
    assert accepted is False


# --- readiness gate: waits for the managed nodes before collecting --------
#
# `docker compose up -d --force-recreate` returns before bt_navigator,
# controller_server and planner_server become active. Without waiting for
# that, the campaign measures "Action server is inactive" instead of the
# condition's behavior -- and nothing in the CSV tells the two apart.

def test_lifecycle_state_reads_the_first_output_word(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (0, 'active [3]\n'))
    assert campaign.lifecycle_state('/bt_navigator') == 'active'


def test_lifecycle_state_recognizes_inactive(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (0, 'inactive [2]\n'))
    assert campaign.lifecycle_state('/bt_navigator') == 'inactive'


def test_lifecycle_state_is_empty_when_command_fails(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (1, ''))
    assert campaign.lifecycle_state('/bt_navigator') == ''


def test_readiness_is_ready_when_all_nodes_are_active(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'lifecycle_state', lambda node, timeout=10.0: 'active')
    ready, elapsed = campaign.wait_for_managed_nodes(
        ('/bt_navigator', '/controller_server'), timeout=1.0, poll_interval=0.01,
        verbose=False)
    assert ready is True
    assert elapsed >= 0.0


def test_readiness_times_out_when_node_stays_inactive(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'lifecycle_state', lambda node, timeout=10.0: 'inactive')
    ready, elapsed = campaign.wait_for_managed_nodes(
        ('/bt_navigator',), timeout=0.05, poll_interval=0.01, verbose=False)
    assert ready is False
    assert elapsed >= 0.05


def test_readiness_waits_for_a_slow_node_to_activate(campaign, monkeypatch):
    """A node that takes its time but activates before the timeout must not
    count as a failure -- giving up too early is as wrong as never giving up."""
    states = iter(['inactive', 'inactive', 'active'])
    monkeypatch.setattr(campaign, 'lifecycle_state',
                         lambda node, timeout=10.0: next(states))
    ready, _ = campaign.wait_for_managed_nodes(
        ('/bt_navigator',), timeout=5.0, poll_interval=0.0, verbose=False)
    assert ready is True


def test_readiness_checks_three_managed_nodes_by_default(campaign):
    assert campaign.READINESS_NODES == (
        '/bt_navigator', '/controller_server', '/planner_server')


def test_trial_does_not_start_before_the_readiness_gate(campaign, monkeypatch, tmp_path):
    """If the lifecycle never confirms active, neither reset nor nav_trial may
    have run -- otherwise the leg collects data from a still-inactive Nav2."""
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (False, 90.0))
    called = {'reset': False, 'trial': False}

    def fake_reset(**_):
        called['reset'] = True
        return True
    monkeypatch.setattr(campaign, 'reset_between_legs', fake_reset)

    def fake_run(command, timeout):
        if 'nav_trial.py' in ' '.join(command):
            called['trial'] = True
        return 0, ''
    monkeypatch.setattr(campaign, '_run', fake_run)

    result = campaign.main([
        str(tmp_path), '--condition', 'baseline=noop', '--reps', '1',
        '--seconds', '1',
    ])
    assert result == 1
    assert called['reset'] is False
    assert called['trial'] is False


def test_failed_condition_command_aborts_before_readiness(campaign, monkeypatch,
                                                          tmp_path):
    readiness_calls = []
    monkeypatch.setattr(
        campaign, 'wait_for_managed_nodes',
        lambda *a, **k: readiness_calls.append(1) or (True, 0.0))
    monkeypatch.setattr(campaign, '_run', lambda command, timeout: (1, 'boom'))

    result = campaign.main([
        str(tmp_path), '--condition', 'baseline=command-that-fails', '--reps', '1',
        '--seconds', '1',
    ])
    assert result == 1
    assert readiness_calls == []


def test_full_campaign_records_readiness_in_manifest(campaign, monkeypatch, tmp_path):
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (True, 7.5))
    monkeypatch.setattr(campaign, 'reset_between_legs', lambda **_: True)

    def fake_run(command, timeout):
        joined = ' '.join(command)
        if 'nav_trial.py' in joined:
            Path(command[2]).touch()
        return 0, ''
    monkeypatch.setattr(campaign, '_run', fake_run)

    result = campaign.main([
        str(tmp_path), '--condition', 'baseline=aplica-baseline', '--reps', '1',
        '--seconds', '1',
    ])
    assert result == 0

    linhas = (tmp_path / 'manifesto.jsonl').read_text(encoding='utf-8').strip().splitlines()
    assert len(linhas) == 1
    registro = json.loads(linhas[0])
    assert registro == {
        'rep': 1, 'condition': 'baseline', 'command_applied': True,
        'readiness_ready': True, 'readiness_seconds': 7.5,
    }


def test_manifest_records_leg_aborted_by_timeout(campaign, monkeypatch, tmp_path):
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (False, 90.0))
    monkeypatch.setattr(campaign, '_run', lambda command, timeout: (0, ''))

    result = campaign.main([
        str(tmp_path), '--condition', 'baseline=aplica-baseline', '--reps', '1',
        '--seconds', '1',
    ])
    assert result == 1

    linhas = (tmp_path / 'manifesto.jsonl').read_text(encoding='utf-8').strip().splitlines()
    registro = json.loads(linhas[0])
    assert registro['readiness_ready'] is False
    assert registro['readiness_seconds'] == 90.0
