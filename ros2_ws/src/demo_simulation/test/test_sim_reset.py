"""
Trava a semantica do botao de reset do cockpit.

POR QUE ESTE ARQUIVO EXISTE

O reset ja foi `ControlWorld.reset.all`, e essa variante APAGA O ROBO. Medido em
26/08/2026 no mundo `quadruped_maze11`, com uma unica chamada a /demo/sim/reset:

    /joint_states  999 Hz -> morto        gz model -m demo_robot
    /demo/imu      996 Hz -> morto        =>  No model named <demo_robot>
    /demo/odom    49,6 Hz -> morto
    /demo/scan      10 Hz -> 10 Hz        (sensor orfao, segue publicando)
    /clock         999 Hz -> 997 Hz

O robo e INSERIDO depois da carga do mundo (`ros_gz_sim create`), e `reset.all`
devolve o mundo ao SDF de origem -- que nao o contem. O modo de falha e o pior
que este projeto conhece: o cockpit fica inteiro verde (relogio, camera, cena)
apontando para uma planta que nao existe mais, sem uma linha de log.

Dois guardas, e nenhum precisa de Gazebo:

  1. o relay nao sabe montar `reset` no WorldControl -- se alguem reabrir esse
     caminho, `_request('reset')` volta a existir e o teste cai;
  2. o launch resolve a pose de reposicao pela TABELA DO CENARIO, nao por (0,0).
     No labirinto (0,0) nao e a origem da area util, e repor ali devolveria o
     robo para dentro de uma parede -- em silencio.
"""

import importlib.util
from pathlib import Path

from demo_simulation.scenarios import spawn_pose
from demo_simulation.sim_control_relay import _request

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
WORLDS_DIR = Path(__file__).resolve().parents[1] / 'worlds'


def _load(name: str):
    path = LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def sim_control():
    return _load('sim_control.launch.py')


def _context(world: str, **overrides) -> LaunchContext:
    """Contexto como a planta do quadrupede o entrega ao fragmento incluido."""
    context = LaunchContext()
    context.launch_configurations.update({
        'world': str(WORLDS_DIR / world),
        'robot_name': 'demo_robot',
        # Vazio = "pergunte a tabela", igual ao ScenarioPose da planta.
        'x': '', 'y': '', 'yaw': '',
        'height': '0.5',
    })
    context.launch_configurations.update(overrides)
    return context


# --- guarda 1: o WorldControl nao aceita mais reset ------------------------

def test_world_control_nao_monta_reset():
    with pytest.raises(ValueError):
        _request('reset')


@pytest.mark.parametrize('action, paused', [('play', False), ('pause', True)])
def test_play_e_pause_seguem_no_world_control(action, paused):
    request = _request(action)
    assert request.world_control.pause is paused
    # O que nao pode acontecer nunca: pausar/retomar reiniciando o mundo.
    assert request.world_control.reset.all is False
    assert request.world_control.reset.model_only is False
    assert request.world_control.reset.time_only is False


# --- guarda 2: a pose de reposicao vem da tabela do cenario ----------------

def test_reposicao_usa_a_pose_do_cenario_do_labirinto(sim_control):
    world = 'quadruped_maze11.sdf'
    esperado = spawn_pose(str(WORLDS_DIR / world))
    params = sim_control._reset_pose(_context(world))

    assert params['spawn_x'] == pytest.approx(esperado['x'])
    assert params['spawn_y'] == pytest.approx(esperado['y'])
    # O yaw do labirinto NAO e zero: nasce olhando para o corredor. Repor com
    # yaw 0 poe o robo de frente para a parede.
    assert params['spawn_yaw'] == pytest.approx(esperado['yaw'])
    assert params['spawn_yaw'] != 0.0
    assert params['robot_name'] == 'demo_robot'


def test_argumento_explicito_vence_a_tabela(sim_control):
    params = sim_control._reset_pose(
        _context('quadruped_maze11.sdf', x='2.5', y='-1.25', yaw='0.75'),
    )
    assert params['spawn_x'] == pytest.approx(2.5)
    assert params['spawn_y'] == pytest.approx(-1.25)
    assert params['spawn_yaw'] == pytest.approx(0.75)


def test_altura_de_reposicao_segue_a_da_planta(sim_control):
    """O quadrupede repoe na altura de NASCIMENTO, nao na de marcha."""
    assert sim_control._reset_pose(
        _context('quadruped_maze11.sdf'))['spawn_z'] == pytest.approx(0.5)
    assert sim_control._reset_pose(
        _context('quadruped_maze11.sdf', height='0.43'),
    )['spawn_z'] == pytest.approx(0.43)


def test_planta_sem_height_cai_no_default_do_diffdrive(sim_control):
    """
    `height` so existe na planta do quadrupede.

    A diff-drive nasce com `-z 0.1` cravado no `create`; repo-la a 0,5 m seria
    uma queda gratuita, e repor um quadrupede a 0,1 m mete as pernas no chao.
    """
    context = _context('quadruped_maze11.sdf')
    del context.launch_configurations['height']
    params = sim_control._reset_pose(context)
    assert params['spawn_z'] == pytest.approx(sim_control.DEFAULT_RESET_Z)
    assert params['spawn_z'] == pytest.approx(0.1)
