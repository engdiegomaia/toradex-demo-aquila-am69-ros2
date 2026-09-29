"""
Invariantes do transporte comprimido da camera entre host e modulo.

Testes estruturais: leem os launch files com `ast`, nao sobem ROS. Casar string
crua seria fragil justamente aqui -- a primeira versao destes testes passou
enquanto o launch estava quebrado, porque afirmava a forma posicional que era o
proprio defeito.

O que eles protegem custou bancada, nao opiniao:
docs/results/ml35-f5-ethernet0-repeticao.md.
"""

from __future__ import annotations

import ast
from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
CONTRACT_TOPIC = '/demo/camera/image_raw'
LOCAL_TOPIC = '/demo/perception/image_in'


def _republish_nodes(launch_file: str) -> dict[str, dict]:
    """Extrai {nome: {params, remaps}} de cada Node(executable='republish')."""
    tree = ast.parse((LAUNCH / launch_file).read_text(encoding='utf-8'))
    found: dict[str, dict] = {}

    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        executable = kw.get('executable')
        if not isinstance(executable, ast.Constant) or executable.value != 'republish':
            continue

        params: dict[str, str] = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    params[key.value] = value.value

        remaps = {}
        for pair in getattr(kw.get('remappings'), 'elts', []):
            if isinstance(pair, ast.Tuple) and len(pair.elts) == 2:
                src, dst = pair.elts
                if isinstance(src, ast.Constant) and isinstance(dst, ast.Constant):
                    remaps[src.value] = dst.value

        name = kw['name'].value
        found[name] = {'params': params, 'remaps': remaps}

    return found


HOST = _republish_nodes('sim.launch.py')
MODULE = _republish_nodes('perception.launch.py')


def _expected_remap_key(side: str, transport: str) -> str:
    """
    Nomeie o topico como `<side>/<transporte>`, exceto em raw.

    `raw` e o transporte default e usa o topico base sem sufixo. Qualquer outro
    acrescenta o sufixo, e o remap precisa casar o nome COMPLETO.
    """
    return side if transport == 'raw' else f'{side}/{transport}'


def _effective_topic(node: dict, side: str) -> str:
    transport = node['params'][f'{side}_transport']
    return node['remaps'][_expected_remap_key(side, transport)]


def test_both_sides_declare_a_republish_node() -> None:
    assert 'camera_compressor' in HOST
    assert 'camera_decompressor' in MODULE


def test_every_republish_sets_both_transports_explicitly() -> None:
    """
    `out_transport` ausente nao da erro -- da um no mudo, ou um laco.

    Escrito como arguments=['raw', 'compressed'], o Jazzy le o primeiro como
    in_transport e deixa out_transport vazio, registrando
    "The 'out_transport' parameter is set to:" com o valor em branco. Nenhuma
    linha de erro. Foi assim que a camera do contrato foi a 118 Hz.
    """
    for name, node in {**HOST, **MODULE}.items():
        for key in ('in_transport', 'out_transport'):
            assert node['params'].get(key), \
                f'{name}: {key} precisa ser parametro explicito e nao-vazio'


def test_remap_keys_carry_the_transport_suffix() -> None:
    """
    Remapear `out` quando o topico se chama `out/compressed` NAO casa.

    A regra e ignorada em silencio: o no sobe, loga os transportes certos,
    assina a entrada, e publica em `/out/compressed` na raiz -- um topico que
    ninguem procura. Medido em 25/08/2026; so `ros2 node info` denunciava.
    """
    for name, node in {**HOST, **MODULE}.items():
        for side in ('in', 'out'):
            expected = _expected_remap_key(side, node['params'][f'{side}_transport'])
            assert expected in node['remaps'], (
                f'{name}: falta remap de {expected!r}; '
                f'as chaves presentes sao {sorted(node["remaps"])}'
            )


def test_no_republish_can_feed_itself() -> None:
    """
    Publicar no topico que se assina nao da erro -- da realimentacao.

    Medido: Publisher count 2 no topico do contrato e a camera a 118 Hz em vez
    de 10 Hz, com o detection_stub do modulo recebendo o fluxo inflado pelo fio.
    """
    for name, node in {**HOST, **MODULE}.items():
        assert _effective_topic(node, 'in') != _effective_topic(node, 'out'), \
            f'{name}: publica no mesmo topico que assina'


def test_the_two_sides_agree_on_the_wire_topic() -> None:
    """
    O que o host publica precisa ser exatamente o que o modulo assina.

    Sao arquivos diferentes, em maquinas diferentes, e nada em runtime reclama
    se divergirem: o decompressor apenas nunca recebe nada, e a percepcao morre
    calada.
    """
    assert _effective_topic(HOST['camera_compressor'], 'out') == \
        _effective_topic(MODULE['camera_decompressor'], 'in')


def test_module_output_never_reuses_the_contract_topic_name() -> None:
    """Dois publicadores no mesmo topico nao dao erro -- dao fonte alternando."""
    assert _effective_topic(MODULE['camera_decompressor'], 'out') == LOCAL_TOPIC
    assert _effective_topic(HOST['camera_compressor'], 'in') == CONTRACT_TOPIC


def test_perception_is_rewired_by_remap_and_not_by_editing_the_node() -> None:
    """
    Regra 6: demo_perception nao sabe a origem do quadro.

    A religacao mora no launch. Se virar edicao em detection_stub.py, o no passa
    a conhecer a topologia de transporte e a troca pelo TIDL deixa de ser troca
    de container.
    """
    perception = (LAUNCH / 'perception.launch.py').read_text(encoding='utf-8')
    assert f"SetRemap(src='{CONTRACT_TOPIC}', dst='{LOCAL_TOPIC}')" in perception

    stub = (LAUNCH.parents[1] / 'demo_perception' / 'demo_perception'
            / 'detection_stub.py').read_text(encoding='utf-8')
    assert 'CompressedImage' not in stub
    assert f"IMAGE_TOPIC = '{CONTRACT_TOPIC}'" in stub


def test_decompressor_stays_outside_the_remap_scope() -> None:
    """No escopo do SetRemap, o `in` do decompressor viraria ele mesmo."""
    perception = (LAUNCH / 'perception.launch.py').read_text(encoding='utf-8')
    group_start = perception.index('perception_group = GroupAction([')
    assert perception.index('camera_decompressor = Node(') < group_start
    group_body = perception[group_start:perception.index('])', group_start)]
    assert 'camera_decompressor' not in group_body
