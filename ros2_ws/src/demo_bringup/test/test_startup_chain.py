"""A cadeia de subida do Nav2 quadrupede: ordem, nao tempo decorrido.

Estruturais: leem launch e YAML com `ast`/`yaml`, nao sobem ROS.

O que protegem custou bancada. Em 26/08/2026, no Aquila AM69, os cinco nos do
`nav_quadruped.launch.py` eram emitidos JUNTOS. A aresta `odom -> base` demorou
mais de 60 s para atravessar a fronteira de container, o `local_costmap` nao
ativou, e o gerenciador de ciclo de vida ABORTOU o bringup em definitivo. O
container ficou de pe, todos os topicos apareceram, `module.sh verify` retornou
0, e toda meta foi recusada com "Action server is inactive".
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / 'demo_bringup' / 'launch' / 'nav_quadruped.launch.py'
PARAMS = SRC / 'demo_navigation' / 'config' / 'nav2_params_go2.yaml'

# Emitir estes junto com o resto e exatamente o defeito medido.
MUST_BE_GATED = ('nav2_container', 'navigation', 'nav_control')


def _tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _launch_description_elements() -> list[str]:
    """Nomes emitidos DIRETAMENTE em LaunchDescription([...])."""
    for call in ast.walk(_tree()):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'LaunchDescription' and call.args):
            return [e.id for e in call.args[0].elts if isinstance(e, ast.Name)]
    raise AssertionError('LaunchDescription([...]) nao encontrado')


def _node_params(executable: str) -> dict:
    for call in ast.walk(_tree()):
        if not isinstance(call, ast.Call):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        found = kw.get('executable')
        if not isinstance(found, ast.Constant) or found.value != executable:
            continue
        params: dict = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if isinstance(key, ast.Constant):
                    params[key.value] = (value.value
                                         if isinstance(value, ast.Constant)
                                         else 'dinamico')
        return params
    raise AssertionError(f'Node(executable={executable!r}) nao existe')


def test_nav2_is_not_emitted_alongside_everything_else() -> None:
    emitted = _launch_description_elements()
    for name in MUST_BE_GATED:
        assert name not in emitted, (
            f'{name} e emitido direto em LaunchDescription. Isso o coloca em '
            f'corrida com a TF odom -> base: se ela demorar, o local_costmap '
            f'nao ativa e o gerenciador aborta o bringup PARA SEMPRE.'
        )


def test_the_edge_producer_starts_immediately() -> None:
    """`odom_tf` PRODUZ a aresta que o portao espera; condiciona-lo trava tudo."""
    assert 'odom_tf' in _launch_description_elements()


def test_nav2_is_gated_on_the_tf_gate_finishing() -> None:
    handlers = [
        call for call in ast.walk(_tree())
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == 'OnProcessExit'
    ]
    assert len(handlers) >= 2, 'a cadeia wait_for_clock -> wait_for_tf -> Nav2 sumiu'

    gated_on_tf = [
        h for h in handlers
        if any(isinstance(n, ast.Name) and n.id == 'wait_for_tf'
               for kw in h.keywords if kw.arg == 'target_action'
               for n in ast.walk(kw.value))
    ]
    assert gated_on_tf, 'nada e condicionado ao wait_for_tf terminar'

    following = ast.unparse(gated_on_tf[0])
    for name in MUST_BE_GATED:
        assert name in following, f'{name} nao esta atras do portao de TF'


def test_the_gate_watches_the_frames_the_costmap_demands() -> None:
    """O portao tem de esperar a MESMA aresta que faz o costmap ativar.

    Se alguem trocar `robot_base_frame` no YAML e nao aqui, o portao libera com
    a aresta errada disponivel e o defeito volta inteiro, agora com um teste
    verde por perto.
    """
    params = _node_params('wait_for_tf')
    costmap = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    local = costmap['local_costmap']['local_costmap']['ros__parameters']

    assert params['parent_frame'] == local['global_frame']
    assert params['child_frame'] == local['robot_base_frame']


def test_the_gate_fails_loud_instead_of_starting_nav2_anyway() -> None:
    """`OnProcessExit` dispara em QUALQUER saida, inclusive erro.

    Pelo AST, e nao por `'returncode' in source`: a primeira versao deste teste
    fazia isso e SOBREVIVEU a mutacao que trocava o `if` por `if True`, porque a
    palavra continuava aparecendo na mensagem de log ao lado.
    """
    handler = next(
        (fn for fn in ast.walk(_tree())
         if isinstance(fn, ast.FunctionDef) and fn.name == '_on_exit'), None)
    assert handler, 'o callable de saida do portao sumiu'

    branch = next((n for n in ast.walk(handler) if isinstance(n, ast.If)), None)
    assert branch, 'o portao nao ramifica: libera o Nav2 em qualquer saida'

    condition = ast.unparse(branch.test)
    assert 'returncode' in condition and '0' in condition, (
        f'a condicao do portao e {condition!r}. Ela tem de olhar o codigo de '
        f'saida: um wait_for_tf que ESTOUROU o prazo nao pode liberar o Nav2, '
        f'que e exatamente o defeito original.'
    )

    # O ramo de falha pode estar num `else` ou logo depois do `if` que retorna
    # cedo. As duas formas sao corretas; o que nao pode e o Shutdown estar no
    # caminho de SUCESSO, ou nao existir.
    success_path = ast.unparse(ast.Module(body=branch.body, type_ignores=[]))
    whole = ast.unparse(handler)
    assert 'Shutdown(' in whole, (
        'o ramo de falha nao derruba o launch, entao a falha volta a ser muda'
    )
    assert 'Shutdown(' not in success_path, (
        'o portao derruba o launch tambem quando o pre-requisito FOI satisfeito'
    )
