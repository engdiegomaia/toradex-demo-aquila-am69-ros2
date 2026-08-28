"""
Contratos da decimacao do joint_state_broadcaster.

Existe porque a mudanca que estes testes protegem tem um modo de falha barato de
cometer e caro de diagnosticar: baixar a taxa do LACO em vez da taxa do
BROADCASTER. As duas edicoes sao de uma linha, ficam no mesmo arquivo, e a
errada quebra a marcha -- cujo comportamento foi medido ao longo de quatro
experimentos revertidos (docs/results/ml35-f4-parcial.md) contra um
controller_manager a 1000 Hz.

O que se decima e o publicador de /joint_states, medido a ~1090 Hz em /tf no HIL
de 28/08/2026 (docs/results/ml35-f5-tf-cpu-baseline.md §6). O que NAO se decima e
todo o resto: laco a 1000, marcha a 200, fisica a 1 ms, IMU e odometria como
estavam.
"""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'ros2_ws/src/demo_simulation/launch/quadruped.launch.py'
JSB_PARAMS = (ROOT / 'ros2_ws/src/demo_simulation/config'
              / 'joint_state_broadcaster.yaml')
GAIT_PARAMS = ROOT / 'ros2_ws/src/demo_simulation/config/gait_go2.yaml'
GAZEBO_CONFIG = ROOT / 'ros2_ws/src/go2_description/config/gazebo.yaml'
ROBOT_CONFIG = ROOT / 'ros2_ws/src/go2_description/config/robot_control.yaml'
WORLDS = sorted((ROOT / 'ros2_ws/src/demo_simulation/worlds').glob('*.sdf'))

# A taxa do laco de controle, que e a mesma do passo de fisica de 1 ms.
LOOP_RATE_HZ = 1000
# A taxa do broadcaster depois da decimacao. Casada com /demo/odom, que e a
# aresta dinamica que a navegacao espera a 50 Hz.
BROADCASTER_RATE_HZ = 50
# A taxa da marcha, medida sob ML3.5 F2/F4.
GAIT_RATE_HZ = 200


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _launch_tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _assignment(name: str) -> str:
    node = next(
        item for item in ast.walk(_launch_tree())
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name
                for target in item.targets)
    )
    return ast.unparse(node)


# --- a taxa global nao se move -------------------------------------------

def test_the_control_loop_still_runs_at_one_kilohertz() -> None:
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        params = _yaml(path)['controller_manager']['ros__parameters']
        assert params['update_rate'] == LOOP_RATE_HZ, path.name


def test_physics_still_steps_at_one_millisecond() -> None:
    # Decimar o broadcaster nao pode virar, por engano, um mundo mais lento:
    # seria a mesma economia de CPU pelo caminho errado, e invalidaria toda a
    # marcha medida.
    assert WORLDS, 'nenhum mundo encontrado'
    for world in WORLDS:
        assert '<max_step_size>0.001</max_step_size>' in world.read_text(
            encoding='utf-8'), world.name


def test_the_gait_controller_keeps_the_rate_its_tuning_was_measured_against(
) -> None:
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        params = _yaml(path)['unitree_guide_controller']['ros__parameters']
        assert params['update_rate'] == GAIT_RATE_HZ, path.name


def test_the_two_vendored_configs_agree_on_the_rates_this_demo_uses() -> None:
    # gazebo.yaml e robot_control.yaml sao a mesma pilha em simulacao e em
    # hardware. Uma taxa que divergir entre os dois e um bug que so aparece do
    # lado que ninguem rodou naquela semana.
    #
    # A lista e restrita ao que ESTE projeto carrega. `ocs2_quadruped_controller`
    # ja diverge no upstream (500 Hz em simulacao contra 200 em hardware) e
    # `rl_quadruped_controller` depende de pesos .pt que foram descartados quando
    # go2_description foi vendorizado. Incluir os dois aqui seria adotar uma
    # divergencia que nao e nossa e que ninguem deste lado pode resolver.
    gazebo, robot = _yaml(GAZEBO_CONFIG), _yaml(ROBOT_CONFIG)
    for section in ('controller_manager', 'unitree_guide_controller'):
        left = gazebo[section]['ros__parameters'].get('update_rate')
        right = robot[section]['ros__parameters'].get('update_rate')
        assert left == right, section


# --- so o broadcaster e decimado -----------------------------------------

def test_the_broadcaster_is_decimated_to_fifty_hertz() -> None:
    params = _yaml(JSB_PARAMS)['joint_state_broadcaster']['ros__parameters']
    assert params['update_rate'] == BROADCASTER_RATE_HZ


def test_the_broadcaster_rate_divides_the_loop_rate_exactly() -> None:
    # O ControllerManager decima por fator inteiro. Uma taxa que nao divide
    # 1000 nao e recusada -- ela e arredondada, e o /joint_states sai numa
    # cadencia que nao e a que o YAML diz.
    assert LOOP_RATE_HZ % BROADCASTER_RATE_HZ == 0


def test_the_broadcaster_param_file_touches_nothing_but_the_broadcaster(
) -> None:
    document = _yaml(JSB_PARAMS)
    assert list(document) == ['joint_state_broadcaster']
    assert list(document['joint_state_broadcaster']['ros__parameters']) == [
        'update_rate']


def test_no_project_param_file_decimates_the_imu_or_the_gait() -> None:
    # A varredura e sobre o diretorio inteiro de config do demo_simulation, e
    # nao sobre uma lista de arquivos conhecidos, porque o defeito que ela
    # persegue e alguem acrescentar um arquivo novo.
    config_dir = ROOT / 'ros2_ws/src/demo_simulation/config'
    for path in sorted(config_dir.glob('*.yaml')):
        document = _yaml(path)
        # bridge_quadruped.yaml e uma LISTA de pontes, nao um mapa de
        # controladores. Pular em vez de explodir mantem a varredura util
        # quando o diretorio ganhar mais arquivos que nao sao param files.
        if not isinstance(document, dict):
            continue
        for controller, block in document.items():
            if controller == 'joint_state_broadcaster':
                continue
            params = (block or {}).get('ros__parameters', {})
            assert 'update_rate' not in params, f'{path.name}:{controller}'


def test_the_gait_tuning_file_carries_no_rate_at_all() -> None:
    # O gait_go2.yaml e o outro param file aplicado pelo mesmo mecanismo. Ele
    # existe para a sintonia da marcha; uma taxa que aparecesse ali passaria a
    # competir com gazebo.yaml sem que nada acuse a diferenca.
    params = _yaml(GAIT_PARAMS)['unitree_guide_controller']['ros__parameters']
    assert 'update_rate' not in params
    assert 'joint_state_broadcaster' not in _yaml(GAIT_PARAMS)


def test_the_vendored_config_never_declares_a_broadcaster_rate() -> None:
    # go2_description e vendorizado e byte-identico ao upstream, e essa garantia
    # e o que sustenta o argumento de licenca. A decimacao mora no projeto.
    for path in (GAZEBO_CONFIG, ROBOT_CONFIG):
        document = _yaml(path)
        block = document.get('joint_state_broadcaster')
        assert block is None or 'ros__parameters' not in block, path.name


# --- o launch aplica o arquivo, e so nele --------------------------------

def test_the_spawner_applies_the_broadcaster_param_file() -> None:
    source = _assignment('joint_state_broadcaster')
    assert "'joint_state_broadcaster'" in source
    assert "'--param-file'" in source
    assert "LaunchConfiguration('jsb_params')" in source


def test_the_imu_broadcaster_is_left_at_the_inherited_rate() -> None:
    source = _assignment('imu_sensor_broadcaster')
    assert "'--param-file'" not in source


def test_the_gait_spawner_still_loads_only_its_own_tuning() -> None:
    source = _assignment('unitree_guide_controller')
    assert "LaunchConfiguration('gait_params')" in source
    assert "jsb_params" not in source


def test_the_decimation_is_overridable_and_defaults_to_the_project_file(
) -> None:
    source = _assignment('jsb_params_arg')
    assert "'jsb_params'" in source
    assert "'joint_state_broadcaster.yaml'" in source
    assert "'demo_simulation'" in source


def test_the_new_argument_is_actually_registered() -> None:
    # Um DeclareLaunchArgument que nao entra na LaunchDescription nao existe: o
    # LaunchConfiguration falha em tempo de execucao, dentro do container.
    description = next(
        ast.unparse(node) for node in ast.walk(_launch_tree())
        if isinstance(node, ast.Return)
        and isinstance(node.value, ast.Call)
        and getattr(node.value.func, 'id', None) == 'LaunchDescription'
    )
    assert 'jsb_params_arg' in description
