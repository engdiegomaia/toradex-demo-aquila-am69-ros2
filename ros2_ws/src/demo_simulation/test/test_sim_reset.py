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
     robo para dentro de uma parede -- em silencio;
  3. o reset PARA o robo antes de teleportar e o reancora depois. Os dois
     defeitos seguintes, medidos no mesmo dia, e os dois silenciosos:
       - teleportar sem reancorar: o StateTrotting segue perseguindo a pose
         anterior, o eixo de guinada satura em 100% dos ticks, o robo se
         arrasta 0,87 m e COLAPSA a z=0,131 m contra 0,353 m de marcha;
       - teleportar sem parar: `SetEntityPose` preserva a VELOCIDADE, e um robo
         em marcha e solto de 0,15 m ainda viajando -- z de 0,337 m para
         0,162 m em um segundo, com o fluxo de cmd_vel vivo.
"""

import importlib.util
from pathlib import Path

from demo_simulation.scenarios import spawn_pose
from demo_simulation.sim_control_relay import _request

from launch import LaunchContext

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


# --- guarda 3: teleportar sem reancorar o gait ---------------------------

RELAY = (
    Path(__file__).resolve().parents[1]
    / 'demo_simulation' / 'sim_control_relay.py'
).read_text(encoding='utf-8')


def test_o_robo_para_antes_do_teleporte_e_retoma_depois():
    """
    A ordem e o conteudo desta correcao, nao um detalhe de estilo.

    Parar DEPOIS de teleportar nao serve: o teleporte preserva a velocidade, e
    quem cai e o robo em marcha. Reancorar ANTES nao serve: o
    StateTrotting::enter() le a pose corrente, que ainda e a velha.
    """
    assert 'HOLD_SERVICE' in RELAY
    assert 'RESUME_SERVICE' in RELAY

    parada = RELAY.index('gait = self._hold_gait()')
    teleporte = RELAY.index('pose_request.entity.name')
    retomada = RELAY.index('self._resume_gait(gait)}')
    assert parada < teleporte < retomada


def test_o_reset_espera_o_robo_parar_de_verdade():
    """Sem espera, o hold e so uma chamada: o robo ainda esta em movimento."""
    assert 'GAIT_STOP_S' in RELAY
    assert 'time.sleep(GAIT_STOP_S)' in RELAY

    parada = RELAY.index('gait = self._hold_gait()')
    espera = RELAY.index('time.sleep(GAIT_STOP_S)')
    teleporte = RELAY.index('pose_request.entity.name')
    assert parada < espera < teleporte


def test_a_ausencia_do_gait_nao_reprova_o_reset():
    """
    Na planta diferencial nao existe gait, e isso e caminho normal.

    Se a reancoragem virar obrigatoria, o reset do diffdrive passa a falhar --
    e o cockpit passa a mostrar erro num reset que funcionou.
    """
    assert 'GAIT_TIMEOUT_S' in RELAY
    assert 'nothing to stop' in RELAY


def test_a_falha_de_teleporte_nao_deixa_o_robo_preso_em_fixed_stand():
    """
    Todo caminho de saida depois do hold tem de retomar.

    Um robo deixado em FIXEDSTAND nao aceita comando nenhum, e nada em log diz
    por que a demo parou de responder.
    """
    saidas = RELAY.count('self._resume_gait(gait)')
    assert saidas == 3, (
        f'esperados 3 caminhos de retomada (timeout, recusa, sucesso), '
        f'encontrados {saidas}'
    )


def test_a_falha_de_reancoragem_nao_pode_ser_silenciosa():
    """Um reset que teleporta e nao reancora deixa o robo se arrastando."""
    assert 'the gait was NOT re-anchored' in RELAY
