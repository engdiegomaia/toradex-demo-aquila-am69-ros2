"""
Structural guards for the official scenario (labirinto).

Static checks on committed files, same reasoning as the other files in this
directory: each invariant here is cheap to break in a one-line edit and the
failure mode is a measurement that PASSES with numbers better than reality.

Run with:  python3 -m pytest tests/ -q
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SIMULATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation'
BRINGUP = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_bringup'
SCENARIOS = SIMULATION / 'demo_simulation' / 'scenarios.py'
QUADRUPED_LAUNCH = SIMULATION / 'launch' / 'quadruped.launch.py'
SCENE_CAMERAS = SIMULATION / 'launch' / 'scene_cameras.launch.py'
SIM_LAUNCH = BRINGUP / 'launch' / 'sim.launch.py'
ROBOT_SELECTION = BRINGUP / 'demo_bringup' / 'robot_selection.py'
COMPOSE_HOST = REPO_ROOT / 'docker' / 'compose.host.yml'
MODELS_EXTRA_README = REPO_ROOT / 'docker' / 'models-extra' / 'README.md'

MAZE_WORLD = 'quadruped_maze11.sdf'


def _load(path: Path, name: str):
    """Import a module by path, so the tables are checked as VALUES."""
    sys.path.insert(0, str(path.parent.parent))
    try:
        module = __import__(f'{path.parent.name}.{name}', fromlist=[name])
    finally:
        sys.path.pop(0)
    return module


@pytest.fixture(scope='module')
def scenarios():
    return _load(SCENARIOS, 'scenarios')


@pytest.fixture(scope='module')
def selection():
    return _load(ROBOT_SELECTION, 'robot_selection')


def _code(path: Path) -> str:
    """Source with comments and docstrings stripped.

    A guard that matches its own documentation is worse than no guard: it passes
    while the behaviour is gone. Same helper reasoning as
    test_cockpit_web_contract.py.
    """
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.FunctionDef,
                                 ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            body.pop(0)
    return ast.unparse(tree)


# --- o cenario oficial ----------------------------------------------------

def test_maze_is_the_official_world_for_the_quadruped(selection):
    """O labirinto e o cenario oficial, e o pedido do operador em 25/08."""
    package, *parts = selection.official_world('quadruped')
    assert parts[-1] == MAZE_WORLD
    assert package == 'demo_simulation'


def test_diffdrive_keeps_the_warehouse_where_it_was_validated(selection):
    """
    O fallback NAO herda o labirinto.

    O portao do F6 do diff-drive e uma meta de x=0 para x=1, medida no armazem.
    No labirinto essa meta cai numa parede: promover o labirinto para os dois
    trocaria o cenario de um teste que passou por um em que ele nunca rodou.
    """
    package, *parts = selection.official_world('diffdrive')
    assert parts[-1] == 'warehouse.sdf'
    assert package == 'nav2_minimal_tb4_sim'


def test_unknown_robot_has_no_official_world(selection):
    with pytest.raises(ValueError):
        selection.official_world('hexapod')


def test_quadruped_plant_defaults_to_the_maze():
    """
    O caminho mais curto ja sobe o cenario oficial.

    Era `quadruped_empty.sdf`, herdado do spike do F2. Com o quadrupede virando
    o robo padrao no F6, o default antigo fazia o comando mais curto medir
    navegacao em campo aberto com a sintonia pensada para corredor de 1,20 m.
    """
    code = _code(QUADRUPED_LAUNCH)
    assert MAZE_WORLD in code
    assert 'quadruped_empty.sdf' not in code


# --- enquadramento derivado do mundo -------------------------------------

def test_maze_framing_matches_the_measured_geometry(scenarios):
    """
    Os numeros do labirinto sao os medidos, nao arredondamentos novos.

    Centro (-4,855; 4,855) saiu da bbox do STL lida do binario e multiplicada
    pela escala 0,002 (tools/maze/maze_fit.py), nao do nome do arquivo. A vista de
    topo a 13 m cobre 17,8 x 13,3 m, os 11,6 m do labirinto com margem.
    """
    top = scenarios.camera_pose(MAZE_WORLD, 'top')
    assert top[:3] == (-4.855, 4.855, 13.0)
    iso = scenarios.camera_pose(MAZE_WORLD, 'iso')
    assert iso[:3] == (-13.0, -3.0, 9.0)


def test_maze_spawn_faces_the_corridor(scenarios):
    """
    yaw 1,5708 nasce o robo olhando para o corredor, nao para a parede.

    Com yaw 0 a primeira coisa que o Nav2 tem de fazer e um giro de 90 graus
    dentro de um corredor de 1,20 m. Isso gasta os primeiros segundos de todo
    ensaio e polui a comparacao entre condicoes.
    """
    assert scenarios.spawn_pose(MAZE_WORLD)['yaw'] == pytest.approx(1.5708)


def test_scene_camera_defaults_are_empty_not_numeric():
    """
    Vazio = derive do mundo. Um numero aqui e a falha silenciosa original.

    Enquanto mundo e enquadramento eram argumentos independentes, a combinacao
    errada era a mais facil de produzir: mundo do labirinto com camera do
    armazem aponta o painel azul para chao vazio, sem erro e sem log.
    """
    code = _code(SCENE_CAMERAS)
    assert "default_value=''" in code
    # As poses nao podem estar cravadas aqui: quem as guarda e scenarios.py.
    for number in ('-4.855', '13.0', '0.5150'):
        assert number not in code, (
            f'{number} voltou a ser default de launch; o enquadramento tem de '
            'vir de scenarios.py, senao mundo e camera divergem de novo'
        )


def test_scene_cameras_reads_the_scenario_table():
    assert 'camera_pose' in _code(SCENE_CAMERAS)


def test_sim_launch_resolves_the_world_before_including_the_plant():
    """
    Escopo de launch: o pai vence, entao o mundo NAO pode ficar vazio aqui.

    Um DeclareLaunchArgument na descricao incluida nao sobrepoe valor herdado. Se
    sim.launch.py passasse vazio, scene_cameras.launch.py herdaria o vazio e
    cairia no enquadramento generico num mundo cuja area util nao esta na origem.
    """
    code = _code(SIM_LAUNCH)
    assert 'official_world' in code


# --- a guarda da malha externa -------------------------------------------

def test_missing_mesh_aborts_and_names_the_variable():
    """
    Malha ausente e WARNING no Gazebo, nunca erro.

    Sem guarda: o mundo carrega, o labirinto nao esta la, o lidar nao ve parede,
    o Nav2 planeja em linha reta e a meta termina SUCCEEDED mais rapido que o
    real. O ensaio PASSA com numeros melhores que a verdade.
    """
    code = _code(QUADRUPED_LAUNCH)
    assert 'missing_models' in code
    assert 'MAZE_MODELS' in code
    assert 'RuntimeError' in code


def test_maze_mesh_is_not_vendored(scenarios):
    """
    Licenca TODO no upstream, mesmo bloqueio que trocou o A1 pelo Go2.

    Se algum dia a malha for vendorizada, ela sai de `needs_models` e este teste
    deve ser reescrito junto com a base legal -- nao apagado.
    """
    assert scenarios.external_models(MAZE_WORLD) == ('maze11',)
    assert not list(SIMULATION.glob('models/maze11/**/*.stl'))


def test_models_extra_readme_exists():
    """
    O comentario do compose promete este arquivo.

    A promessa esteve falsa: o comentario dizia "The default is an EMPTY
    directory in the repo ... See models-extra/README.md" e o diretorio nao
    existia. Documentacao que aponta para o vazio e pior que ausente.
    """
    assert MODELS_EXTRA_README.is_file()
    text = MODELS_EXTRA_README.read_text()
    assert 'MAZE_MODELS' in text
    assert 'ros_maze_worlds' in text


def test_compose_no_longer_pastes_the_framing_by_hand():
    """
    A quarta copia dos dez numeros saiu do compose.

    Era um SIM_ARGS de seis linhas, e colar metade dele dava mundo do labirinto
    com camera do armazem.
    """
    text = COMPOSE_HOST.read_text()
    assert 'scene_iso_pitch:=' not in text
    assert 'MAZE_MODELS' in text


# --- estabilidade: o segfault do route_server ----------------------------

NAV_PARAMS = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation' / 'config'
              / 'nav2_params_go2.yaml')
VENDORED_NAV = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation' / 'launch'
                / 'nav2_vendored' / 'navigation_launch.py')


def test_route_server_does_not_build_the_rerouting_service():
    """
    Configurar `ReroutingService` derruba o nav2_container com SIGSEGV.

    Medido em tres caminhos: RESET+STARTUP pelo lifecycle_manager e **cold start
    normal**, este de forma intermitente. Quando acontece morrem TODOS os
    servidores do Nav2 de uma vez e o container `nav` fica com o processo `ros2`
    vivo e nenhum filho -- `docker compose ps` diz "running" e o robo nao navega.

    A demo nao usa roteamento por grafo, entao listar so `AdjustSpeedLimit` em
    `operations` faz o plugin problematico nunca ser construido.
    """
    import yaml
    params = yaml.safe_load(NAV_PARAMS.read_text())
    operations = params['route_server']['ros__parameters']['operations']
    assert 'ReroutingService' not in operations
    # Lista vazia derruba o launch com "Expected 'value' to be one of [...]
    # but got '()'" -- armadilha 5 do guia-operacao.
    assert operations, 'lista YAML vazia quebra o launch; deixe um plugin'


def test_every_declared_route_operation_declares_its_plugin_type():
    """
    Sobrepor `operations` obriga a declarar o TIPO de cada plugin da lista.

    Enquanto a lista vem do default, o nav2_route conhece os tipos dos proprios
    defaults. No instante em que ela e sobreposta, ele passa a exigir
    `<nome>.plugin` -- e reprova a subida inteira com uma mensagem que nao diz de
    onde falta o parametro:

        [FATAL] [route_server]: Can not get 'plugin' param value for AdjustSpeedLimit
        [FATAL] [route_server]: Failed to configure route server: No 'plugin' param
        [ERROR] [lifecycle_manager_navigation]: Failed to bring up all requested nodes.

    Medido em 25/08/2026, no primeiro cold start com o contorno do segfault
    aplicado. Este teste existe para que a proxima operacao acrescentada a lista
    nao repita a mesma subida reprovada.
    """
    import yaml
    section = yaml.safe_load(NAV_PARAMS.read_text())['route_server']
    params = section['ros__parameters']
    for name in params['operations']:
        assert name in params, (
            f'{name} esta em operations e nao tem secao propria')
        assert params[name].get('plugin'), (
            f'{name} precisa de `plugin:` com o tipo, ex. nav2_route::{name}')


def test_vendored_navigation_launch_still_owns_the_lifecycle_list():
    """
    O contorno acima existe PORQUE a lista nao e sobreponivel pelo YAML.

    `navigation_launch.py` passa `{'node_names': lifecycle_nodes}` inline, e
    parametro inline vence arquivo de parametros. Se algum dia o upstream expuser
    a lista como argumento de launch, o contorno pode virar "nao suba o
    route_server" e este teste deve mudar junto.
    """
    text = VENDORED_NAV.read_text()
    assert "'node_names': lifecycle_nodes" in text
    assert "'route_server'" in text


# --- estabilidade: descoberta DDS ---------------------------------------

DDS_HOST = REPO_ROOT / 'docker' / 'cyclonedds' / 'host.xml'
ENV_SH = REPO_ROOT / 'scripts' / 'env.sh'
COMPOSE_HOST_TEXT = COMPOSE_HOST.read_text()


def test_host_dds_pins_loopback_alongside_autodetermine():
    """
    Mesma maquina nao pode depender do que o autodetermine escolher.

    Com docker0, duas bridges e tailscale0 no ar, a escolha pode cair numa
    interface que nao roteia para os outros participantes. O resultado medido em
    25/08/2026 foi entrega ASSIMETRICA: o `nav` recebia odom e scan do `sim`, e o
    `sim` NAO recebia /demo/cmd_vel do `nav`. O robo trota parado com
    `sticks=(lx=0.0000 ...)` enquanto o Nav2 comanda guinada em 95,6% das
    amostras, e nenhuma linha de log nomeia DDS.
    """
    text = DDS_HOST.read_text()
    assert 'name="lo"' in text, (
        'sem interface de loopback explicita, o trafego entre containers na '
        'mesma maquina volta a depender do autodetermine'
    )
    assert 'autodetermine="true"' in text, (
        'a interface real tem de continuar na lista, ou o modo hil perde o '
        'caminho para o Aquila'
    )


def test_rendered_dds_config_is_not_older_than_its_template():
    """
    Editar host.xml nao surte efeito nos containers ate `module.sh sync` rodar.

    O compose monta `host.rendered.xml`, gerado a partir de host.xml com o
    endereco do modulo e a interface deste host (endereco nao entra em git). Na
    bancada de 25/08/2026 o rendered havia sido gerado antes, era byte-identico
    ao template, e ficou parado enquanto o template mudava: a correcao de
    loopback estava commitada e os containers seguiam sem ela.

    Este teste e SKIP quando o rendered nao existe -- num clone novo ele nao
    existe ainda, e isso nao e um defeito do commit.
    """
    rendered = REPO_ROOT / 'docker' / 'cyclonedds' / 'host.rendered.xml'
    if not rendered.is_file():
        pytest.skip('host.rendered.xml ainda nao foi gerado (module.sh sync)')
    assert rendered.stat().st_mtime >= DDS_HOST.stat().st_mtime, (
        'host.rendered.xml e mais antigo que host.xml: rode '
        '`scripts/module.sh sync` e recrie os containers, senao a config '
        'revisada nao e a que roda'
    )
    assert 'name="lo"' in rendered.read_text(), (
        'o rendered perdeu a interface de loopback'
    )


def test_render_step_keeps_loopback_and_replaces_autodetermine():
    """
    O awk de `render_host_config` troca so a linha do autodetermine.

    Se ele passar a reescrever o bloco <Interfaces> inteiro, a linha do loopback
    desaparece do hil e a assimetria volta -- la, onde e mais caro descobrir.
    """
    module_sh = (REPO_ROOT / 'scripts' / 'module.sh').read_text()
    assert 'NetworkInterface autodetermine=' in module_sh
    assert 'autodetermine' in module_sh


def test_host_tools_use_the_same_dds_config_as_the_containers():
    """
    `nav_trial.py` abortava com "navigate_to_pose nao apareceu" enquanto o log
    do nav dizia "Managed nodes are active" e /demo/odom chegava a 49 Hz: metade
    do grafo visivel, metade nao.
    """
    text = ENV_SH.read_text()
    assert 'CYCLONEDDS_URI' in text
    assert 'cyclonedds/host.xml' in text


# --- estabilidade: carga que a navegacao paga sem aparecer -----------------

SCENE_MODELS = (REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation' / 'models')


def test_perception_layer_clears_in_both_costmaps():
    """
    `clearing: false` na camada de percepcao deixa marca PERMANENTE.

    `observation_persistence` esvazia o buffer de observacoes, nao as celulas ja
    escritas. Medido em 25/08/2026 com a percepcao parada ha um minuto: 169
    celulas letais antes de limpar, 108 depois de `/demo/nav/reset` -- 61 celulas
    que nenhuma observacao viva sustentava. Num corredor de 1,20 m com
    `robot_radius: 0.38`, e a diferenca entre ter e nao ter caminho, e numa demo
    longa e degradacao monotonica sem nenhum log.

    Os DOIS costmaps precisam casar: planejar num mapa que limpa e controlar num
    que nao limpa da rota valida com o controlador recusando segui-la.
    """
    import yaml
    params = yaml.safe_load(NAV_PARAMS.read_text())
    found = 0
    for costmap in ('global_costmap', 'local_costmap'):
        layers = params[costmap][costmap]['ros__parameters']
        layer = layers.get('perception_layer')
        if layer is None:
            continue
        found += 1
        source = layer['observation_sources']
        assert layers['perception_layer'][source]['clearing'] is True, (
            f'{costmap}.perception_layer.{source}.clearing deve ser true')
    assert found == 2, 'as duas camadas de percepcao tem de existir'


def test_scene_cameras_keep_the_measured_aspect_and_match_each_other():
    """
    O enquadramento das duas cenas foi MEDIDO em 4:3.

    `horizontal_fov` fixo com outra proporcao corta vertical e desenquadra as
    duas de uma vez, sem erro. E as duas camaras tem de ter a mesma resolucao:
    resolucoes diferentes fazem um painel do cockpit chegar mais nitido que o
    outro por motivo nenhum, e mudam o custo de render de cada um.
    """
    import re
    sizes = {}
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        text = (SCENE_MODELS / name).read_text()
        width = int(re.search(r'<width>(\d+)</width>', text).group(1))
        height = int(re.search(r'<height>(\d+)</height>', text).group(1))
        assert width * 3 == height * 4, f'{name}: {width}x{height} nao e 4:3'
        sizes[name] = (width, height)
    assert len(set(sizes.values())) == 1, f'resolucoes divergentes: {sizes}'
