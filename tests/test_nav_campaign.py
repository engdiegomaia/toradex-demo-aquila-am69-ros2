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


def test_a_ordem_intercala_e_nao_bloca(campaign):
    conditions = [('baseline', None), ('align8', 'cmd')]
    nomes = [name for _, name, _ in campaign.leg_order(conditions, 3)]
    assert nomes == ['baseline', 'align8'] * 3
    # The opposite of what is wanted, written so it doesn't pass by accident:
    assert nomes != ['baseline'] * 3 + ['align8'] * 3


def test_toda_condicao_recebe_o_mesmo_n(campaign):
    conditions = [('a', None), ('b', 'x'), ('c', 'y')]
    nomes = [name for _, name, _ in campaign.leg_order(conditions, 4)]
    assert len(nomes) == 12
    assert {nomes.count(n) for n in ('a', 'b', 'c')} == {4}


def test_as_replicatas_sao_numeradas_em_ordem(campaign):
    reps = [rep for rep, _, _ in campaign.leg_order([('a', None)], 3)]
    assert reps == [1, 2, 3]


@pytest.mark.parametrize('raw, esperado', [
    ('baseline', ('baseline', None)),
    ('align8=ros2 param set /x y 8.0', ('align8', 'ros2 param set /x y 8.0')),
    # An empty `=` means "apply an empty command", which is different from
    # "apply nothing" — and the difference has to survive the parse.
    ('vazio=', ('vazio', '')),
])
def test_parse_de_condicao(campaign, raw, esperado):
    assert campaign.parse_condition(raw) == esperado


def test_condicao_sem_nome_e_erro(campaign):
    with pytest.raises(Exception):
        campaign.parse_condition('=comando')


def test_comparacao_exige_reaplicar_todas_as_condicoes(campaign):
    with pytest.raises(ValueError, match='baseline'):
        campaign.validate_conditions([
            ('baseline', None),
            ('align8', 'aplica-align8'),
        ])


def test_comparacao_aceita_comandos_explicitos(campaign):
    campaign.validate_conditions([
        ('baseline', 'aplica-baseline'),
        ('align8', 'aplica-align8'),
    ])


def test_condicao_unica_pode_usar_estado_atual(campaign):
    campaign.validate_conditions([('smoke', None)])


def test_rota_com_coordenada_negativa_e_repassada_com_equals(campaign, monkeypatch,
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


def test_o_reset_do_simulador_e_obrigatorio(campaign, monkeypatch):
    """
    A refusal during the reset has to ABORT, not just warn.

    A leg that starts from a different state than the others is not a
    replicate, and including it in the median ruins the single number the
    campaign produces.
    """
    monkeypatch.setattr(campaign, 'SETTLE_S', 0.0)
    monkeypatch.setattr(
        campaign, 'call_trigger',
        lambda service, timeout=30.0: (False, 'recusado'),
    )
    assert campaign.reset_between_legs(skip_nav=True, verbose=False) is False


def test_reposicao_ok_devolve_true(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'SETTLE_S', 0.0)
    chamados = []

    def fake(service, timeout=30.0):
        chamados.append(service)
        return True, 'success=True'

    monkeypatch.setattr(campaign, 'call_trigger', fake)
    assert campaign.reset_between_legs(skip_nav=False, verbose=False) is True
    # Both, and in this order: reset the robot before clearing the costmap,
    # so the new costmap is already born from the new pose.
    assert chamados == [campaign.SIM_RESET, campaign.NAV_RESET]


def test_success_false_nao_conta_como_aceito(campaign, monkeypatch):
    """
    `ros2 service call` exits 0 even when the service responded success=False.

    Trusting the exit code would let the campaign continue after a reset that
    never happened — and a reset that doesn't happen is invisible on screen.
    """
    monkeypatch.setattr(
        campaign, '_run',
        lambda command, timeout: (0, 'response:\nTrigger_Response(success=False'),
    )
    aceito, _ = campaign.call_trigger('/demo/sim/reset')
    assert aceito is False


# --- readiness gate: waits for the managed nodes before collecting --------
#
# `docker compose up -d --force-recreate` returns before bt_navigator,
# controller_server and planner_server become active. Without waiting for
# that, the campaign measures "Action server is inactive" instead of the
# condition's behavior -- and nothing in the CSV tells the two apart.

def test_lifecycle_state_le_a_primeira_palavra_da_saida(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (0, 'active [3]\n'))
    assert campaign.lifecycle_state('/bt_navigator') == 'active'


def test_lifecycle_state_reconhece_inactive(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (0, 'inactive [2]\n'))
    assert campaign.lifecycle_state('/bt_navigator') == 'inactive'


def test_lifecycle_state_vazio_quando_comando_falha(campaign, monkeypatch):
    monkeypatch.setattr(campaign, '_run', lambda cmd, timeout: (1, ''))
    assert campaign.lifecycle_state('/bt_navigator') == ''


def test_readiness_pronta_quando_todos_nodes_ativos(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'lifecycle_state', lambda node, timeout=10.0: 'active')
    pronto, decorrido = campaign.wait_for_managed_nodes(
        ('/bt_navigator', '/controller_server'), timeout=1.0, poll_interval=0.01,
        verbose=False)
    assert pronto is True
    assert decorrido >= 0.0


def test_readiness_timeout_quando_node_fica_inactive(campaign, monkeypatch):
    monkeypatch.setattr(campaign, 'lifecycle_state', lambda node, timeout=10.0: 'inactive')
    pronto, decorrido = campaign.wait_for_managed_nodes(
        ('/bt_navigator',), timeout=0.05, poll_interval=0.01, verbose=False)
    assert pronto is False
    assert decorrido >= 0.05


def test_readiness_espera_node_lento_ate_ativar(campaign, monkeypatch):
    """A node that takes its time but activates before the timeout must not
    count as a failure -- giving up too early is as wrong as never giving up."""
    estados = iter(['inactive', 'inactive', 'active'])
    monkeypatch.setattr(campaign, 'lifecycle_state',
                         lambda node, timeout=10.0: next(estados))
    pronto, _ = campaign.wait_for_managed_nodes(
        ('/bt_navigator',), timeout=5.0, poll_interval=0.0, verbose=False)
    assert pronto is True


def test_readiness_verifica_os_tres_managed_nodes_por_padrao(campaign):
    assert campaign.READINESS_NODES == (
        '/bt_navigator', '/controller_server', '/planner_server')


def test_trial_nao_comeca_antes_do_readiness_gate(campaign, monkeypatch, tmp_path):
    """If the lifecycle never confirms active, neither reset nor nav_trial may
    have run -- otherwise the leg collects data from a still-inactive Nav2."""
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (False, 90.0))
    chamado = {'reset': False, 'trial': False}

    def fake_reset(**_):
        chamado['reset'] = True
        return True
    monkeypatch.setattr(campaign, 'reset_between_legs', fake_reset)

    def fake_run(command, timeout):
        if 'nav_trial.py' in ' '.join(command):
            chamado['trial'] = True
        return 0, ''
    monkeypatch.setattr(campaign, '_run', fake_run)

    resultado = campaign.main([
        str(tmp_path), '--condition', 'baseline=noop', '--reps', '1',
        '--seconds', '1',
    ])
    assert resultado == 1
    assert chamado['reset'] is False
    assert chamado['trial'] is False


def test_comando_de_condicao_que_falha_aborta_antes_do_readiness(campaign, monkeypatch,
                                                                   tmp_path):
    chamado_readiness = []
    monkeypatch.setattr(
        campaign, 'wait_for_managed_nodes',
        lambda *a, **k: chamado_readiness.append(1) or (True, 0.0))
    monkeypatch.setattr(campaign, '_run', lambda command, timeout: (1, 'boom'))

    resultado = campaign.main([
        str(tmp_path), '--condition', 'baseline=comando-que-falha', '--reps', '1',
        '--seconds', '1',
    ])
    assert resultado == 1
    assert chamado_readiness == []


def test_campanha_completa_grava_readiness_no_manifesto(campaign, monkeypatch, tmp_path):
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (True, 7.5))
    monkeypatch.setattr(campaign, 'reset_between_legs', lambda **_: True)

    def fake_run(command, timeout):
        joined = ' '.join(command)
        if 'nav_trial.py' in joined:
            Path(command[2]).touch()
        return 0, ''
    monkeypatch.setattr(campaign, '_run', fake_run)

    resultado = campaign.main([
        str(tmp_path), '--condition', 'baseline=aplica-baseline', '--reps', '1',
        '--seconds', '1',
    ])
    assert resultado == 0

    linhas = (tmp_path / 'manifesto.jsonl').read_text(encoding='utf-8').strip().splitlines()
    assert len(linhas) == 1
    registro = json.loads(linhas[0])
    assert registro == {
        'rep': 1, 'condition': 'baseline', 'command_applied': True,
        'readiness_ready': True, 'readiness_seconds': 7.5,
    }


def test_manifesto_registra_perna_que_abortou_por_timeout(campaign, monkeypatch, tmp_path):
    monkeypatch.setattr(campaign, 'wait_for_managed_nodes',
                         lambda *a, **k: (False, 90.0))
    monkeypatch.setattr(campaign, '_run', lambda command, timeout: (0, ''))

    resultado = campaign.main([
        str(tmp_path), '--condition', 'baseline=aplica-baseline', '--reps', '1',
        '--seconds', '1',
    ])
    assert resultado == 1

    linhas = (tmp_path / 'manifesto.jsonl').read_text(encoding='utf-8').strip().splitlines()
    registro = json.loads(linhas[0])
    assert registro['readiness_ready'] is False
    assert registro['readiness_seconds'] == 90.0
