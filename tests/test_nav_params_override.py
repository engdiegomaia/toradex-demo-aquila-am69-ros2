"""Contract for applying Nav2 parameter variants in A/B campaigns.

The selector default stays empty for direct launch use. Compose cannot emit a
valid ``params_override:=`` argument from an empty value, so it passes an explicit
sentinel which the selector also interprets as "use the robot default".
"""

from __future__ import annotations

import ast
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
NAV_SELECT = ROOT / 'ros2_ws/src/demo_bringup/launch/nav_select.launch.py'
HOST_COMPOSE = ROOT / 'docker/compose.host.yml'
MODULE_COMPOSE = ROOT / 'docker/compose.module.yml'
NAV_CAMPAIGN = ROOT / 'tools/evaluation/nav_campaign.py'
EVALUATION_GUIDE = ROOT / 'docs/evaluation.md'
GO2_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2.yaml'
ALIGN8_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/params-align8.yaml'
FOOTPRINT_PARAMS = (
    ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2_footprint.yaml')

# Tronco do Go2, medido: 0,70 x 0,31 m.
TRUNK_LENGTH_M = 0.70
TRUNK_WIDTH_M = 0.31


def _tree() -> ast.Module:
    return ast.parse(NAV_SELECT.read_text(encoding='utf-8'))


def _launch_arg(name: str) -> ast.Call:
    for call in ast.walk(_tree()):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name) or call.func.id != 'DeclareLaunchArgument':
            continue
        if call.args and isinstance(call.args[0], ast.Constant) and call.args[0].value == name:
            return call
    raise AssertionError(f'DeclareLaunchArgument({name!r}) not found')


def _function(name: str) -> ast.FunctionDef:
    for node in ast.walk(_tree()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f'{name} not found')


def test_nav_select_default_params_file_keeps_robot_launch_default() -> None:
    params_arg = _launch_arg('params_override')
    keywords = {kw.arg: kw.value for kw in params_arg.keywords if kw.arg}

    default = keywords.get('default_value')
    assert isinstance(default, ast.Constant)
    assert default.value == ''


def test_nav_select_only_passes_params_file_when_non_empty() -> None:
    fn = _function('_launch_navigation')
    source = ast.unparse(fn)

    assert "LaunchConfiguration('params_override').perform(context)" in source
    assert 'params_override != ROBOT_DEFAULT_PARAMS' in source
    assert "launch_arguments['params_file'] = params_override" in source


def test_compose_exposes_the_same_nav_params_override_on_host_and_module() -> None:
    for path in (HOST_COMPOSE, MODULE_COMPOSE):
        text = path.read_text(encoding='utf-8')
        assert 'params_override:=${NAV2_PARAMS:-__robot_default__}' in text, path
        assert 'params_file:=${NAV2_PARAMS:-}' not in text, path


def test_campaign_docs_use_a_path_inside_the_nav_image() -> None:
    expected = '/ws/src/demo_navigation/config/params-align8.yaml'
    command = f'NAV2_PARAMS={expected} \\'

    assert command in NAV_CAMPAIGN.read_text(encoding='utf-8')
    assert command in EVALUATION_GUIDE.read_text(encoding='utf-8')


def test_align8_changes_only_the_path_alignment_weight() -> None:
    baseline = yaml.safe_load(GO2_PARAMS.read_text(encoding='utf-8'))
    align8 = yaml.safe_load(ALIGN8_PARAMS.read_text(encoding='utf-8'))
    path = ('controller_server', 'ros__parameters', 'FollowPath',
            'PathAlignCritic', 'cost_weight')

    cursor = align8
    for key in path[:-1]:
        cursor = cursor[key]
    assert cursor[path[-1]] == 8.0

    cursor[path[-1]] = 14.0
    assert align8 == baseline


def _costmap_params(document: dict, name: str) -> dict:
    return document[name][name]['ros__parameters']


def test_footprint_variant_matches_the_promoted_default() -> None:
    """
    The footprint experiment was promoted into the default on 29/08/2026
    (`docs/results/ml35-f5-footprint-ab.md`; HIL under the new default still
    PENDING).

    `nav2_params_go2_footprint.yaml` is kept only because Compose, docs and
    prior campaign commands still reference `NAV2_PARAMS=...footprint.yaml`;
    it must describe the exact same robot as the default now, not a second
    shape. If this fails, the two files drifted apart again.
    """
    baseline = yaml.safe_load(GO2_PARAMS.read_text(encoding='utf-8'))
    variant = yaml.safe_load(FOOTPRINT_PARAMS.read_text(encoding='utf-8'))

    assert variant == baseline


def test_default_uses_the_footprint_polygon_not_a_circle() -> None:
    """Promoted 29/08/2026: the default no longer ships `robot_radius`."""
    baseline = yaml.safe_load(GO2_PARAMS.read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        params = _costmap_params(baseline, name)
        assert 'footprint' in params, name
        assert 'robot_radius' not in params, name


def test_local_and_global_costmaps_share_the_same_default_footprint() -> None:
    baseline = yaml.safe_load(GO2_PARAMS.read_text(encoding='utf-8'))
    local = _costmap_params(baseline, 'local_costmap')['footprint']
    glob = _costmap_params(baseline, 'global_costmap')['footprint']
    assert local == glob


def test_footprint_variant_never_declares_a_radius_beside_the_polygon() -> None:
    """
    `robot_radius` e `footprint` juntos deixam a forma efetiva ambigua.

    O costmap aceita os dois e usa um deles; qual, depende da ordem de leitura
    dos parametros. Medir uma rodada nesse estado nao mede nada.
    """
    variant = yaml.safe_load(FOOTPRINT_PARAMS.read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        params = _costmap_params(variant, name)
        assert 'robot_radius' not in params, name


def test_footprint_encloses_the_measured_trunk() -> None:
    """A pegada tem de conter o tronco medido, senao ela nao descreve o robo."""
    variant = yaml.safe_load(FOOTPRINT_PARAMS.read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        points = ast.literal_eval(_costmap_params(variant, name)['footprint'])
        assert len(points) == 4, name
        length = max(x for x, _ in points) - min(x for x, _ in points)
        width = max(y for _, y in points) - min(y for _, y in points)
        assert length >= TRUNK_LENGTH_M, (name, length)
        assert width >= TRUNK_WIDTH_M, (name, width)


def test_inflation_still_covers_the_circumscribed_footprint() -> None:
    """
    Inflacao menor que a pegada deixa o planejador raspar o canto na parede.

    A regra ja estava no arquivo default contra `robot_radius`; com um poligono
    o piso passa a ser o raio CIRCUNSCRITO, que e maior que o inscrito.
    """
    variant = yaml.safe_load(FOOTPRINT_PARAMS.read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        params = _costmap_params(variant, name)
        points = ast.literal_eval(params['footprint'])
        circumscribed = max(math.hypot(x, y) for x, y in points)
        inflation = params['inflation_layer']['inflation_radius']
        assert inflation >= circumscribed, (name, inflation, circumscribed)


def test_footprint_variant_uses_polygon_collision_checking() -> None:
    """
    Promovido em 30/08/2026 após a pegada explícita virar o default.

    O alias histórico não pode reativar a amostragem apenas pelo ponto central;
    sem polígono publicado este modo derrubaria o nav2_container com SIGSEGV,
    por isso a igualdade integral com o default também é testada acima.
    """
    variant = yaml.safe_load(FOOTPRINT_PARAMS.read_text(encoding='utf-8'))
    critic = variant['controller_server']['ros__parameters']['FollowPath']
    assert critic['CostCritic']['consider_footprint'] is True
