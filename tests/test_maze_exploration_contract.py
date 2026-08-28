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


def test_slam_publishes_the_grid_at_the_upstream_default_period():
    # 5.0 e o default do upstream
    # (`/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`).
    # Ficou em 1.0 ate 28/08/2026, pagando 5x a rasterizacao do pose-graph no
    # AM69 sem que nada exigisse essa taxa.
    #
    # As outras quatro chaves estao aqui como TRAVA de A/B, nao por gosto: a
    # rodada que mede o efeito de `map_update_interval` so significa alguma
    # coisa se elas nao tiverem se mexido junto.
    params = yaml.safe_load((NAV / 'config/slam_params.yaml').read_text())
    slam = params['slam_toolbox']['ros__parameters']
    assert slam['map_update_interval'] == 5.0
    assert slam['restamp_tf'] is True
    assert slam['transform_timeout'] == 0.2
    assert slam['transform_publish_period'] == 0.02
    assert slam['minimum_time_interval'] == 0.5


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


# --- fiacao de launch e dependencias ---------------------------------------
#
# Estruturais, com `ast`: casar string crua passa com o launch quebrado. E o que
# eles pegam so aparece DEPOIS, no container arm64 -- um no que ninguem inicia,
# ou um import que o package.xml nao declara e que o rosdep do build nao instala.

def _node_launches(path):
    """Devolve {executable: {kwargs crus do Node(...)}} de um launch file."""
    import ast
    tree = ast.parse((ROOT / path).read_text(encoding='utf-8'))
    found = {}
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        kwargs = {k.arg: k.value for k in call.keywords if k.arg}
        executable = kwargs.get('executable')
        if isinstance(executable, ast.Constant):
            found[executable.value] = ast.dump(call)
    return found


NAV_LAUNCH = 'ros2_ws/src/demo_bringup/launch/nav_quadruped.launch.py'
PERCEPTION_LAUNCH = 'ros2_ws/src/demo_perception/launch/perception.launch.py'
SIM_LAUNCH = 'ros2_ws/src/demo_simulation/launch/quadruped.launch.py'


def test_explorer_is_started_by_the_quadruped_navigation_launch():
    """Um no que ninguem inicia e um no que nao existe."""
    nodes = _node_launches(NAV_LAUNCH)
    assert 'maze_explorer' in nodes
    # Sem o caminho da arvore de exploracao ele cairia na arvore padrao, que usa
    # o GridBased com allow_unknown -- e o caminho passaria pelo desconhecido.
    assert 'exploration_bt_xml' in nodes['maze_explorer']


def test_detector_runs_where_the_camera_is_consumed():
    """A percepcao roda no modulo; o detector tem de subir com ela."""
    assert 'maze_exit_detector' in _node_launches(PERCEPTION_LAUNCH)


def test_ground_truth_validator_never_leaves_the_simulation():
    """
    O validador le odometria ground truth: ele NAO pode rodar do lado do robo.

    Se subisse junto com a navegacao, o explorador teria acesso indireto a
    verdade que ele deveria descobrir sozinho, e a aceitacao nao mediria nada.
    """
    assert 'maze_escape_validator' in _node_launches(SIM_LAUNCH)
    assert 'maze_escape_validator' not in _node_launches(NAV_LAUNCH)
    assert 'maze_escape_validator' not in _node_launches(PERCEPTION_LAUNCH)


def test_explorer_and_detector_never_run_on_the_simulation_side():
    """Regra 1 ao contrario: o que decide navegacao mora no modulo."""
    sim_nodes = _node_launches(SIM_LAUNCH)
    assert 'maze_explorer' not in sim_nodes
    assert 'maze_exit_detector' not in sim_nodes


def test_every_new_node_has_a_console_script():
    """Sem entry point o launch encontra o pacote e nao encontra o executavel."""
    for package, executable in (
        ('demo_navigation', 'maze_explorer'),
        ('demo_perception', 'maze_exit_detector'),
        ('demo_simulation', 'maze_escape_validator'),
    ):
        setup = (ROOT / f'ros2_ws/src/{package}/setup.py').read_text()
        assert executable in setup, f'{package}: {executable}'


def test_package_manifests_declare_what_the_new_modules_import():
    """
    Import nao declarado no package.xml quebra no container, nao no host.

    No host o overlay do ROS ja tem tudo; a imagem arm64 instala exatamente o
    que o manifesto pede. Este e o teste que separa "funciona aqui" de
    "funciona no Aquila".
    """
    import re
    expected = {
        'demo_navigation': ('std_msgs', 'geometry_msgs', 'nav_msgs',
                            'tf2_ros', 'nav2_msgs', 'action_msgs',
                            'std_srvs'),
        'demo_perception': ('geometry_msgs', 'sensor_msgs', 'vision_msgs'),
        'demo_simulation': ('std_msgs', 'nav_msgs'),
    }
    for package, dependencies in expected.items():
        manifest = (ROOT / f'ros2_ws/src/{package}/package.xml').read_text()
        declared = set(re.findall(r'<(?:exec_)?depend>([^<]+)</', manifest))
        missing = [name for name in dependencies if name not in declared]
        assert not missing, f'{package} nao declara {missing}'


def test_the_exploration_tree_is_installed_with_the_package():
    """A arvore e lida em runtime pelo bt_navigator dentro do container."""
    setup = (ROOT / 'ros2_ws/src/demo_navigation/setup.py').read_text()
    assert 'behavior_trees' in setup


def test_perception_keeps_the_marker_out_of_the_costmap_pipeline():
    """
    O painel e uma pista visual; vira-lo obstaculo tapa a propria saida.

    `detections_to_cloud` assina o topico do contrato do projeto. O detector da
    saida publica noutro, e essa separacao e o que impede o marcador de aparecer
    como obstaculo exatamente em frente a abertura.
    """
    detector = (ROOT / 'ros2_ws/src/demo_perception/demo_perception'
                / 'maze_exit_detector.py').read_text()
    assert "'/demo/perception/maze_exit/detections'" in detector
    assert "'/demo/perception/detections'" not in detector


def test_neither_perception_nor_frontier_knows_the_maze():
    """O isolamento vale para toda a cadeia, nao so para o executivo."""
    for path in ('ros2_ws/src/demo_perception/demo_perception/maze_exit_detector.py',
                 'ros2_ws/src/demo_navigation/demo_navigation/frontier.py'):
        source = (ROOT / path).read_text()
        for forbidden in ('maze_route', 'maze11', '-4.90', '-0.90', 'waypoint'):
            assert forbidden not in source, f'{path}: {forbidden}'
