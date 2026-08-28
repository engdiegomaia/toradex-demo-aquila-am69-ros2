from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
NAV = ROOT / 'ros2_ws/src/demo_navigation'
SIM = ROOT / 'ros2_ws/src/demo_simulation'


def test_exploration_planner_never_crosses_unknown():
    for name in ('nav2_params_go2.yaml', 'params-align8.yaml'):
        params = yaml.safe_load((NAV / 'config' / name).read_text())
        planner = params['planner_server']['ros__parameters']
        assert planner['GridBased']['allow_unknown'] is True
        assert planner['ExplorationGrid']['allow_unknown'] is False
        assert 'ExplorationGrid' in planner['planner_plugins']


def test_slam_tf_is_restamped_for_distributed_hil_clock():
    params = yaml.safe_load((NAV / 'config/slam_params.yaml').read_text())
    slam = params['slam_toolbox']['ros__parameters']
    assert slam['restamp_tf'] is True
    assert slam['transform_timeout'] == 0.2


def test_exploration_bt_hardcodes_safe_planner_and_smoothing():
    path = NAV / 'behavior_trees/nav_to_pose_exploration.xml'
    root = ET.parse(path).getroot()
    compute = root.find('.//ComputePathToPose')
    assert compute is not None
    assert compute.attrib['planner_id'] == 'ExplorationGrid'
    smooth = root.find('.//SmoothPath')
    assert smooth is not None
    assert smooth.attrib['unsmoothed_path'] != smooth.attrib['smoothed_path']


def test_explorer_has_no_runtime_knowledge_of_maze_geometry():
    source = (NAV / 'demo_navigation/maze_explorer.py').read_text()
    for forbidden in ('maze_route', 'maze11', 'STL', '-4.90', '-0.90'):
        assert forbidden not in source


def test_exit_marker_is_visual_only_and_outside_opening():
    world = ET.parse(SIM / 'worlds/quadruped_maze11.sdf').getroot()
    marker = world.find(".//model[@name='maze_exit_marker']")
    assert marker is not None
    assert marker.find('.//collision') is None
    pose = [float(value) for value in marker.findtext('pose').split()]
    assert pose[:2] == [-4.90, -2.60]


def test_public_exploration_interfaces_are_stable():
    source = (NAV / 'demo_navigation/maze_explorer.py').read_text()
    for name in (
        '/demo/exploration/start', '/demo/exploration/cancel',
        '/demo/exploration/status', '/demo/perception/maze_exit/pose',
    ):
        assert name in source


def test_short_goal_gate_is_connected_and_bounded():
    source = (ROOT / 'scripts/nav_trial.py').read_text()
    assert 'MAZE11_SHORT_GOALS' in source
    assert "args.goals == 'maze11-short'" in source


def test_cockpit_owns_start_cancel_and_ground_truth_display():
    html = (ROOT / 'hmi/index.html').read_text()
    panel = (ROOT / 'hmi/js/panels/nav-panel.js').read_text()
    # A decisao de busca mora num modulo sem DOM para poder ser testada pelo
    # `node --test` sem dublar um contexto 2D. O contrato vale sobre os dois.
    store = (ROOT / 'hmi/js/panels/exploration.js').read_text()
    config = (ROOT / 'hmi/js/config.js').read_text()
    assert 'data-role="exploration-start"' in html
    assert 'data-role="exploration-cancel"' in html
    assert '/demo/exploration/start' in panel
    assert '/demo/exploration/cancel' in panel
    # Duas portas para a meta manual -- o clique no canvas e o envio -- e as
    # duas tem de estar fechadas enquanto a busca corre.
    assert panel.count('if (explorationActive()) return') == 2
    assert '/demo/maze/escaped' in config
    assert 'SAÍDA CONFIRMADA' in store
    # O rotulo de sucesso so pode sair do ground truth, nunca do estado do
    # explorador: 'completed' diz que ele chegou perto do marcador, nao que o
    # robo atravessou a abertura.
    assert 'mazeEscaped' in store


def test_gate_persists_outcome_and_error_code_per_goal():
    """O veredito do portao nao pode viver so no stdout de quem rodou."""
    source = (ROOT / 'scripts/nav_trial.py').read_text()
    # Desfecho, codigo de erro do Nav2 e trocas de rota, por meta.
    for field in ('outcome', 'error_code', 'error_msg', 'plan_switches'):
        assert f"'{field}'" in source
    # A meta em voo no fim do ensaio tem de ser arquivada: sem esta chamada um
    # portao de 3 metas termina relatando 2.
    assert source.count('self.close_goal(') >= 2
    assert 'goals_csv_path' in source


def test_telemetry_rows_carry_the_goal_they_belong_to():
    """Sem o carimbo, as tres metas do portao viram uma serie so."""
    source = (ROOT / 'scripts/nav_trial.py').read_text()
    assert "'goal_index'" in source
