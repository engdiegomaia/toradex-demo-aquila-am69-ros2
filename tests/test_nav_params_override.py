"""Contract for applying Nav2 parameter variants in A/B campaigns.

The selector default must stay empty: an empty override means "let the selected
robot launch use its own default". Passing an empty params_file through to the
child launch would override that default with the empty string and break both
robot stacks.
"""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAV_SELECT = ROOT / 'ros2_ws/src/demo_bringup/launch/nav_select.launch.py'
HOST_COMPOSE = ROOT / 'docker/compose.host.yml'
MODULE_COMPOSE = ROOT / 'docker/compose.module.yml'
NAV_CAMPAIGN = ROOT / 'scripts/nav_campaign.py'
NAV_ROADMAP = ROOT / 'docs/ml35/proximos-passos-navegacao.md'


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
    params_arg = _launch_arg('params_file')
    keywords = {kw.arg: kw.value for kw in params_arg.keywords if kw.arg}

    default = keywords.get('default_value')
    assert isinstance(default, ast.Constant)
    assert default.value == ''


def test_nav_select_only_passes_params_file_when_non_empty() -> None:
    fn = _function('_launch_navigation')
    source = ast.unparse(fn)

    assert "LaunchConfiguration('params_file').perform(context)" in source
    assert 'if params_file:' in source
    assert "launch_arguments['params_file'] = params_file" in source


def test_compose_exposes_the_same_nav_params_override_on_host_and_module() -> None:
    for path in (HOST_COMPOSE, MODULE_COMPOSE):
        text = path.read_text(encoding='utf-8')
        assert 'params_file:=${NAV2_PARAMS:-}' in text, path


def test_campaign_docs_use_a_path_inside_the_nav_image() -> None:
    expected = '/ws/src/demo_navigation/config/params-align8.yaml'
    command = f'NAV2_PARAMS={expected} \\'

    assert command in NAV_CAMPAIGN.read_text(encoding='utf-8')
    assert command in NAV_ROADMAP.read_text(encoding='utf-8')
