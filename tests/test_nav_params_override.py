"""Contract for applying Nav2 parameter variants in A/B campaigns.

The selector default stays empty for direct launch use. Compose cannot emit a
valid ``params_override:=`` argument from an empty value, so it passes an explicit
sentinel which the selector also interprets as "use the robot default".
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
NAV_SELECT = ROOT / 'ros2_ws/src/demo_bringup/launch/nav_select.launch.py'
HOST_COMPOSE = ROOT / 'docker/compose.host.yml'
MODULE_COMPOSE = ROOT / 'docker/compose.module.yml'
NAV_CAMPAIGN = ROOT / 'scripts/nav_campaign.py'
NAV_ROADMAP = ROOT / 'docs/ml35/proximos-passos-navegacao.md'
GO2_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/nav2_params_go2.yaml'
ALIGN8_PARAMS = ROOT / 'ros2_ws/src/demo_navigation/config/params-align8.yaml'


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
    assert command in NAV_ROADMAP.read_text(encoding='utf-8')


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
