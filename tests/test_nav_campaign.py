"""
Trava o PROTOCOLO da campanha de navegação.

Só uma coisa aqui é testável sem simulador, e é justamente a que quebra em
silêncio: a ORDEM das pernas. Uma campanha blocada (`A A A B B B`) roda igual,
grava os mesmos arquivos, produz um resumo de aparência idêntica — e atribui à
condição qualquer deriva térmica, de cache ou de rede que tenha ocorrido ao
longo da campanha. Com dispersão medida de 2,4x em configuração IDÊNTICA
(`docs/results/ml35-f5-clock-fanout.md`), isso não é rigor teórico: é a
diferença entre medir o efeito e medir a hora do dia.
"""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'nav_campaign.py'


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
    # O contrário do que se quer, escrito para não passar por acidente:
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
    # Um `=` vazio significa "aplica comando vazio", que é diferente de "não
    # aplica nada" — e a diferença tem de sobreviver ao parse.
    ('vazio=', ('vazio', '')),
])
def test_parse_de_condicao(campaign, raw, esperado):
    assert campaign.parse_condition(raw) == esperado


def test_condicao_sem_nome_e_erro(campaign):
    with pytest.raises(Exception):
        campaign.parse_condition('=comando')


def test_o_reset_do_simulador_e_obrigatorio(campaign, monkeypatch):
    """
    Recusa na reposição tem de ABORTAR, não avisar.

    Uma perna que começa de um estado diferente das outras não é replicata, e
    incluí-la na mediana estraga o único número que a campanha produz.
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
    # Os dois, e nesta ordem: repor o robô antes de esvaziar o costmap, para o
    # costmap novo já nascer da pose nova.
    assert chamados == [campaign.SIM_RESET, campaign.NAV_RESET]


def test_success_false_nao_conta_como_aceito(campaign, monkeypatch):
    """
    `ros2 service call` sai 0 mesmo quando o serviço respondeu success=False.

    Confiar no código de saída faria a campanha seguir depois de um reset que
    não aconteceu — e o reset que não acontece é invisível na tela.
    """
    monkeypatch.setattr(
        campaign, '_run',
        lambda command, timeout: (0, 'response:\nTrigger_Response(success=False'),
    )
    aceito, _ = campaign.call_trigger('/demo/sim/reset')
    assert aceito is False
