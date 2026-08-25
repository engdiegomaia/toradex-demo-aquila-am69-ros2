"""
Constroi de verdade a descricao de launch das cameras de cena.

POR QUE ESTE ARQUIVO EXISTE

Os testes de `scenarios.py` verificam a TABELA; este verifica a FIACAO. A
distincao nao e academica: em 25/08/2026 a tabela estava certa, os guardas
estruturais passavam, e o container `sim` morria na partida com

    [ERROR] [launch]: Caught exception in launch (see debug for traceback):
    value='-13.0' is not an instance of <class 'float'>

porque `ParameterValue(value_type=float)` converte SUBSTITUICAO para float e nao
texto para float, e as poses resolvidas pelo OpaqueFunction ja eram texto. Nada
disso e visivel em leitura estatica, e o custo de descobrir foi um ciclo inteiro
de build de imagem.

Rodar a descricao aqui custa milissegundos e pega toda essa classe: tipo de
parametro, argumento nao declarado, substituicao no lugar de acao.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def _load(name: str):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides) -> LaunchContext:
    """Contexto com os defaults declarados, como o launch faria."""
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            value = ''
            if entity.default_value:
                value = entity.default_value[0].perform(context)
            context.launch_configurations[entity.name] = value
    context.launch_configurations.update(overrides)
    return context


@pytest.fixture(scope='module')
def scene_cameras():
    return _load('scene_cameras.launch.py')


def _params(module, world, **overrides) -> dict:
    """Parametros que o scene_view_controller receberia para este mundo."""
    context = _context(module, world=world, **overrides)
    return module.controller_params(module._resolve(context, 'iso'),
                                    module._resolve(context, 'top'))


WORLDS = ('quadruped_maze11.sdf', 'warehouse.sdf', 'quadruped_empty.sdf', '')


@pytest.mark.parametrize('world', WORLDS)
def test_description_builds_for_every_world(scene_cameras, world):
    """Inclui o mundo vazio: o generico tem de ser um caminho valido."""
    nodes = scene_cameras._cameras(_context(scene_cameras, world=world))
    assert len(nodes) == 3, 'duas cameras e o controlador de vista'


def test_view_controller_params_are_real_floats(scene_cameras):
    """
    O defeito exato de 25/08: pose como texto dentro de ParameterValue.

    O no declara as dez poses como double. Texto ali derruba a descricao ANTES de
    qualquer no subir, e a mensagem fala de tipo, nao de camera.
    """
    context = _context(scene_cameras, world='quadruped_maze11.sdf')
    params = scene_cameras.controller_params(
        scene_cameras._resolve(context, 'iso'),
        scene_cameras._resolve(context, 'top'))
    for name in ('iso_x', 'iso_y', 'iso_z', 'iso_pitch', 'iso_yaw',
                 'top_x', 'top_y', 'top_z', 'top_pitch', 'top_yaw'):
        assert isinstance(params[name], float), f'{name} nao e float'


def test_maze_framing_reaches_the_view_controller(scene_cameras):
    """
    A tabela certa nao garante que ela CHEGOU ao no.

    Este e o par do teste estrutural: la se verifica que os numeros existem em
    scenarios.py, aqui que sao eles que o controlador recebe. O enquadramento do
    armazem num mundo de labirinto aponta o painel azul para chao vazio, sem erro
    e sem log.
    """
    params = _params(scene_cameras, '/algum/lugar/quadruped_maze11.sdf')
    assert params['top_x'] == pytest.approx(-4.855)
    assert params['top_y'] == pytest.approx(4.855)
    assert params['top_z'] == pytest.approx(13.0)
    assert params['iso_x'] == pytest.approx(-13.0)


def test_explicit_argument_beats_the_table(scene_cameras):
    """Sondar de mais alto sem perder os outros nove numeros."""
    params = _params(scene_cameras, 'quadruped_maze11.sdf',
                     scene_top_z='20.0')
    assert params['top_z'] == pytest.approx(20.0)
    # os outros seguem do cenario
    assert params['top_x'] == pytest.approx(-4.855)


def test_warehouse_keeps_its_measured_framing(scene_cameras):
    """
    6 m e nao 12: a 12 m a camera fica acima das vigas do telhado do armazem e a
    imagem inteira vira uma viga, com o robo escondido atras dela.
    """
    params = _params(scene_cameras, 'warehouse.sdf')
    assert params['top_z'] == pytest.approx(6.0)
