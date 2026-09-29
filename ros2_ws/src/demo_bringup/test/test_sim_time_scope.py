"""
Quem nao pergunta as horas nao assina o relogio.

`use_sim_time: true` NAO e uma declaracao de intencao inofensiva: o rclpy cria
uma assinatura de `/clock` por no, independentemente de o codigo do no chamar o
relogio. No Aquila AM69 o Gazebo publica `/clock` a ~870 Hz (passo de fisica de
1 ms, exigido pela marcha), e o preco medido por no que so recebe e descarta
essas mensagens foi 35-40% de um nucleo.

Medicao de 25/08/2026, container `nav` no AM69, pilha de pe e SEM META ATIVA,
amostrando /proc/<tid>/stat por thread: piso ocioso de 367% de 800%, dos quais
111% eram tres republicadores em Python que nao chamam o relogio uma unica vez.

Estes testes sao estruturais -- leem launch e fonte com `ast`, nao sobem ROS.
Eles travam a invariante nos DOIS sentidos, e e o segundo que importa:

  1. os nos listados sobem com use_sim_time literalmente False;
  2. esses mesmos nos NAO chamam get_clock() no caminho quente.

Sem (2) o teste viraria carimbo: alguem acrescenta um timer que le o relogio, o
no passa a ler tempo de PAREDE achando que le tempo simulado, e nada acusa --
os stamps ficam anos no futuro e o costmap descarta leitura com "message filter
dropping message", que nao nomeia a causa.
"""

from __future__ import annotations

import ast
from pathlib import Path


SRC = Path(__file__).resolve().parents[2]
QUADRUPED_LAUNCH = SRC / 'demo_bringup' / 'launch' / 'nav_quadruped.launch.py'
NAV_CONTROL_LAUNCH = SRC / 'demo_navigation' / 'launch' / 'nav_control.launch.py'

# executavel -> fonte que o implementa.
CLOCKLESS_NODES = {
    'cmd_vel_si_to_stick':
        SRC / 'demo_bringup' / 'demo_bringup' / 'cmd_vel_si_to_stick.py',
    'nav_control_relay':
        SRC / 'demo_navigation' / 'demo_navigation' / 'nav_control_relay.py',
    'odom_tf':
        SRC / 'demo_bringup' / 'demo_bringup' / 'odom_tf.py',
}

# odom_tf e a excecao justificada: carimba a aresta ESTATICA map -> odom com o
# relogio, e o buffer estatico do tf2 devolve transformada estatica para
# qualquer instante consultado -- esse stamp nao entra em lookup nenhum.
CLOCK_ALLOWED_IN = {'odom_tf': {'_identity'}}


def _node_params(launch_path: Path, executable: str) -> dict:
    """Extrai os parametros constantes do Node(executable=...) pedido."""
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))

    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        found = kw.get('executable')
        if not isinstance(found, ast.Constant) or found.value != executable:
            continue

        params: dict[str, object] = {}
        for entry in getattr(kw.get('parameters'), 'elts', []):
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if not isinstance(key, ast.Constant):
                    continue
                # Constante vira valor; LaunchConfiguration(...) vira o marcador
                # 'dinamico', que e exatamente o que este teste precisa reprovar.
                params[key.value] = (value.value if isinstance(value, ast.Constant)
                                     else 'dinamico')
        return params

    raise AssertionError(f'Node(executable={executable!r}) nao existe em {launch_path.name}')


def _clock_calls(source: Path) -> set[str]:
    """Funcoes do arquivo que chamam self.get_clock()."""
    tree = ast.parse(source.read_text(encoding='utf-8'))
    callers: set[str] = set()

    for func in (n for n in ast.walk(tree)
                 if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))):
        for node in ast.walk(func):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'get_clock'):
                callers.add(func.name)
    return callers


def test_glue_nodes_do_not_follow_sim_time() -> None:
    for executable, launch in (('odom_tf', QUADRUPED_LAUNCH),
                               ('cmd_vel_si_to_stick', QUADRUPED_LAUNCH),
                               ('nav_control_relay', NAV_CONTROL_LAUNCH)):
        params = _node_params(launch, executable)
        assert params.get('use_sim_time') is False, (
            f'{executable} sobe com use_sim_time={params.get("use_sim_time")!r}. '
            f'Ele nao usa o relogio, e assinar /clock a ~870 Hz custou 35-40% '
            f'de um nucleo no AM69.'
        )


def test_nodes_without_sim_time_do_not_read_the_clock() -> None:
    for executable, source in CLOCKLESS_NODES.items():
        offenders = _clock_calls(source) - CLOCK_ALLOWED_IN.get(executable, set())
        assert not offenders, (
            f'{executable} sobe sem use_sim_time mas chama get_clock() em '
            f'{sorted(offenders)}. Esse relogio e o de PAREDE: ou o no volta a '
            f'seguir /clock, ou a chamada sai.'
        )


def test_odom_tf_hot_path_copies_the_message_stamp() -> None:
    """A aresta odom -> base tem de herdar o stamp da odometria, nao do relogio."""
    tree = ast.parse(CLOCKLESS_NODES['odom_tf'].read_text(encoding='utf-8'))
    on_odom = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == '_on_odom')

    assigns = [ast.unparse(n.value) for n in ast.walk(on_odom)
               if isinstance(n, ast.Assign)
               and any(ast.unparse(t).endswith('header.stamp') for t in n.targets)]
    assert assigns == ['message.header.stamp'], (
        f'_on_odom carimba a TF com {assigns}. Tem de ser o stamp da mensagem: '
        f'relogio local aqui adianta ou atrasa a TF em relacao ao scan e o '
        f'costmap descarta a leitura sem dizer por que.'
    )


def test_nav_control_launch_rejects_a_sim_time_argument() -> None:
    """Argumento aceito e ignorado e falha silenciosa; aqui tem de falhar alto."""
    tree = ast.parse(NAV_CONTROL_LAUNCH.read_text(encoding='utf-8'))
    declared = {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args and isinstance(call.args[0], ast.Constant)
    }
    assert 'use_sim_time' not in declared

    # Pelo AST, e nao por fatia de texto: a primeira versao deste teste casava
    # o COMENTARIO que explica a remocao e reprovava a arvore correta.
    for caller in (QUADRUPED_LAUNCH,
                   SRC / 'demo_navigation' / 'launch' / 'navigation.launch.py'):
        includes = [
            call for call in ast.walk(ast.parse(caller.read_text(encoding='utf-8')))
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'IncludeLaunchDescription'
            and any(isinstance(n, ast.Constant) and n.value == 'nav_control.launch.py'
                    for n in ast.walk(call))
        ]
        assert len(includes) == 1, f'{caller.name}: {len(includes)} includes de nav_control'
        passed = {k.arg for k in includes[0].keywords if k.arg}
        assert 'launch_arguments' not in passed, (
            f'{caller.name} ainda passa launch_arguments para nav_control.launch.py, '
            f'que nao declara mais use_sim_time.'
        )
