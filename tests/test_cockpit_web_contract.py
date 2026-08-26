"""
Structural guards for the web cockpit (ML3.5 cockpit-web).

These are static checks on committed files, not runtime tests. They exist
because each invariant below is cheap to break in a one-line edit and expensive
to discover: the placement ones only fail on the Aquila, weeks later.

Run with:  python3 -m pytest tests/ -q
"""

from __future__ import annotations

import ast
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
COCKPIT_DOCKERFILE = REPO_ROOT / 'docker' / 'cockpit' / 'Dockerfile'
HMI_DOCKERFILE = REPO_ROOT / 'docker' / 'hmi' / 'Dockerfile'
NGINX_TEMPLATE = REPO_ROOT / 'docker' / 'hmi' / 'default.conf.template'
COMPOSE_HOST = REPO_ROOT / 'docker' / 'compose.host.yml'
BUNDLE = REPO_ROOT / 'hmi'
SIMULATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_simulation'
SCENE_CAMERAS_LAUNCH = SIMULATION / 'launch' / 'scene_cameras.launch.py'
SCENARIOS_TABLE = SIMULATION / 'demo_simulation' / 'scenarios.py'
PLANT_LAUNCHES = (
    SIMULATION / 'launch' / 'quadruped.launch.py',
    SIMULATION / 'launch' / 'simulation.launch.py',
)
BRIDGE_CONFIGS = (
    SIMULATION / 'config' / 'bridge_quadruped.yaml',
    SIMULATION / 'config' / 'bridge_warehouse.yaml',
)
GO2_DESCRIPTION = REPO_ROOT / 'ros2_ws' / 'src' / 'go2_description'

# Desktop-OpenGL / OGRE 2 packages. The cockpit image is the ONE role that is
# allowed to build for arm64 and reach the module, so it is the one place where
# adding a convenient debugging package silently violates CLAUDE.md rule 1.
DESKTOP_GL_PACKAGES = (
    'rviz2',
    'rviz-common',
    'ros-gz-sim',
    'gz-sim',
    'gazebo',
    'rqt-common-plugins',
    'rqt-image-view',
)


def test_cockpit_image_stays_free_of_desktop_opengl():
    """The one image allowed on the AM69 must not acquire an OGRE 2 dependency."""
    dockerfile = COCKPIT_DOCKERFILE.read_text()
    installs = '\n'.join(
        line for line in dockerfile.splitlines() if not line.lstrip().startswith('#')
    )
    for package in DESKTOP_GL_PACKAGES:
        assert package not in installs, (
            f'{package} would put desktop OpenGL on an arm64-capable image '
            '(CLAUDE.md rule 1)'
        )


def test_cockpit_image_carries_the_two_transports():
    """rosbridge without web_video_server means a camera panel that never fills."""
    dockerfile = COCKPIT_DOCKERFILE.read_text()
    assert 'rosbridge-suite' in dockerfile
    assert 'web-video-server' in dockerfile


def test_hmi_image_has_no_javascript_build_step():
    """plano-cockpit-web.md Decisão 5: no npm supply chain travels to arm64."""
    dockerfile = _code_of(HMI_DOCKERFILE)
    for forbidden in ('npm ', 'yarn ', 'pnpm ', 'node_modules'):
        assert forbidden not in dockerfile, (
            f'{forbidden.strip()!r} reopens Decisão 5 (bundle without build step)'
        )


def test_bundle_ships_no_vendored_dependencies():
    """A node_modules or a vendored roslib in the tree is the same decision."""
    assert not (BUNDLE / 'node_modules').exists()
    assert not (BUNDLE / 'package-lock.json').exists()
    vendored = [path.name for path in BUNDLE.rglob('roslib*.js')]
    assert vendored == [], f'vendored roslibjs found: {vendored}'


def test_bundle_has_no_hardcoded_hostname():
    """AGENTS.md §5.7. A literal host works on this desk and fails on the bench."""
    sources = [
        path
        for path in BUNDLE.rglob('*')
        if path.suffix in {'.js', '.html', '.css'} and 'test' not in path.parts
    ]
    assert sources, 'bundle sources not found — did the tree move?'
    for path in sources:
        text = path.read_text()
        for forbidden in ('ws://localhost', 'http://localhost', '127.0.0.1'):
            assert forbidden not in text, f'{path.name} hardcodes {forbidden}'


def test_cockpit_ports_are_configurable():
    """network_mode: host means these bind on the operator's workstation."""
    compose = COMPOSE_HOST.read_text()
    for variable in (
        'COCKPIT_ROSBRIDGE_PORT',
        'COCKPIT_VIDEO_PORT',
        'COCKPIT_HMI_PORT',
    ):
        assert f'${{{variable}:-' in compose, f'{variable} must have a compose default'


def test_nginx_template_restricts_envsubst():
    """
    Without the filter, envsubst blanks nginx's own $uri/$host.

    The result is a site that reports a valid configuration and 404s every
    request — a failure whose cause is not named anywhere in the logs.
    """
    assert 'NGINX_ENVSUBST_FILTER=^COCKPIT_' in HMI_DOCKERFILE.read_text()
    assert '$uri' in NGINX_TEMPLATE.read_text()


def _code_of(path: Path) -> str:
    """Source text with comment lines removed, so prose does not trip a check."""
    return '\n'.join(
        line for line in path.read_text().splitlines()
        if not line.lstrip().startswith('#')
    )


# --- F3b: scene cameras ----------------------------------------------------
#
# The blue panel's cameras belong to the world, never to the robot, and they
# must reach EVERY world rather than only the two that happen to be committed
# here. Both invariants are one careless edit away from being lost, and both
# fail silently: a dead panel, or a licence argument that no longer holds.


def test_scene_cameras_are_spawned_not_written_into_worlds():
    """
    The cameras must be spawnable models, not blocks inside worlds/*.sdf.

    docker/compose.host.yml does not pass `world:=`, so the default is
    nav2_minimal_tb4_sim's warehouse.sdf — an installed package file this repo
    cannot edit. Cameras written into the project's own worlds leave the blue
    panel dark in exactly the configuration the demo runs in.
    """
    for world in (SIMULATION / 'worlds').glob('*.sdf'):
        assert 'cockpit_scene' not in world.read_text(), (
            f'{world.name} carries a scene camera inline; it belongs in '
            'models/ and is spawned by scene_cameras.launch.py'
        )

    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        assert (SIMULATION / 'models' / name).is_file(), f'modelo {name} ausente'


def test_scene_camera_models_are_installed():
    """A model that setup.py does not install is a file the launch cannot find."""
    setup = (SIMULATION / 'setup.py').read_text()
    assert "glob('models/*.sdf')" in setup


def test_both_plants_include_the_scene_cameras():
    """
    The panel must not go dark when ROBOT_TYPE flips.

    Both plants are supported paths (CLAUDE.md), and a cockpit that only works
    for one of them is the kind of mode-dependent behaviour the project bans.
    """
    for launch in PLANT_LAUNCHES:
        assert 'scene_cameras.launch.py' in launch.read_text(), (
            f'{launch.name} does not include the shared scene-camera fragment'
        )


def test_scene_cameras_are_never_hung_on_the_vendored_robot():
    """
    go2_description is vendorized byte-for-byte to sustain the licence
    argument (its own README). Hanging a chase camera on the trunk would be the
    obvious shortcut and would quietly invalidate that.
    """
    if not GO2_DESCRIPTION.is_dir():
        return
    for source in GO2_DESCRIPTION.rglob('*'):
        if source.is_file() and source.suffix in {'.xacro', '.urdf', '.sdf', '.yaml'}:
            assert 'cockpit' not in source.read_text(errors='ignore'), (
                f'{source.relative_to(REPO_ROOT)} mentions the cockpit; the '
                'vendored package must stay untouched'
            )


def test_scene_topics_are_bridged_on_every_plant():
    """
    Image AND camera_info, on both bridges.

    The camera_info is not geometry here: it is the panel's heartbeat. An <img>
    on a multipart stream never reports its own liveness, so without it the
    panel shows "sem sinal" over running video.
    """
    for config in BRIDGE_CONFIGS:
        text = config.read_text()
        for view in ('scene_iso', 'scene_top'):
            for suffix in ('image_raw', 'camera_info'):
                topic = f'/demo/cockpit/{view}/{suffix}'
                assert topic in text, f'{topic} não está em {config.name}'


def test_scene_camera_framing_is_overridable():
    """
    No pose literal may be the only way to frame a world.

    maze11 is the one project world whose usable area is not centred on the
    origin, so a single set of defaults cannot suit every world. The framing
    therefore lives in demo_simulation/scenarios.py, indexed BY WORLD, and this
    launch file declares the five axes as arguments that override it.

    The table moved out of this file on 25/08/2026 (it used to be a `DEFAULTS`
    dict here). The invariant did not move: every axis stays reachable from the
    command line, and no world is stuck with another world's framing.

    Read the axis declarations through the source rather than by importing: the
    argument names are built with f-strings, so a grep for the literal name
    would pass on a file that declares nothing.
    """
    poses = ast.literal_eval(
        next(
            node.value
            for node in ast.parse(SCENARIOS_TABLE.read_text()).body
            if isinstance(node, ast.Assign)
            and any(getattr(t, 'id', None) == 'GENERIC' for t in node.targets)
        )
    )

    assert {'scene_iso', 'scene_top'} <= set(poses), (
        'as duas vistas precisam de enquadramento generico'
    )
    for name in ('scene_iso', 'scene_top'):
        # x, y, z, pitch, yaw. Roll is deliberately absent.
        assert len(poses[name]) == 5, f'pose de {name} incompleta'
        for value in poses[name]:
            float(value)

    launch = SCENE_CAMERAS_LAUNCH.read_text()
    for axis in ('x', 'y', 'z', 'pitch', 'yaw'):
        assert f"scene_{{name}}_{{axis}}" in launch or (
            f"'{axis}'" in launch
        ), f'o eixo {axis} não é declarado como argumento de launch'
    assert 'POSE_FIELDS' in launch, (
        'os cinco eixos têm de ser declarados a partir de uma lista única; '
        'declarar um por um é como um deles some sem ninguém notar'
    )


def test_scene_camera_models_carry_no_desktop_gl_assumption():
    """
    A camera sensor is render work inside the Gazebo process, which is host
    only. The models must not acquire anything that suggests otherwise, and the
    world dependency must stay written down where someone will read it.
    """
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        text = (SIMULATION / 'models' / name).read_text()
        assert '<static>true</static>' in text, f'{name} deve ser estático'
        assert 'gz-sim-sensors-system' in text, (
            f'{name} não documenta a dependência do plugin de sensores; sem ele '
            'o sensor existe e não publica nada, sem erro no log'
        )


# --------------------------------------------------------------------------
# Controle da simulação e das câmeras a partir do cockpit (pedido do operador,
# 24/08/2026: "iniciar a simulação pela cockpit, resetá-la, mover, girar e dar
# zoom na tela da simulação").
# --------------------------------------------------------------------------

SIM_CONTROL_LAUNCH = SIMULATION / 'launch' / 'sim_control.launch.py'
SIM_CONTROL_RELAY = SIMULATION / 'demo_simulation' / 'sim_control_relay.py'
SIM_CONTROLS_JS = BUNDLE / 'js' / 'panels' / 'sim-controls.js'
VIEW_CONTROLS_JS = BUNDLE / 'js' / 'panels' / 'view-controls.js'
TOKENS_CSS = BUNDLE / 'css' / 'tokens.css'
INDEX_HTML = BUNDLE / 'index.html'


def _code_lines(source: str) -> list[str]:
    """
    Linhas de código, sem comentário.

    Os cabeçalhos destes arquivos EXPLICAM por que o navegador não fala o
    vocabulário do Gazebo, e citam os nomes ao fazê-lo. Uma busca ingênua pelo
    texto proibiria justamente a documentação da regra.
    """
    lines, in_block = [], False
    for raw in source.splitlines():
        line = raw.strip()
        if in_block:
            if '*/' in line:
                in_block = False
            continue
        if line.startswith('/*'):
            in_block = '*/' not in line
            continue
        if line.startswith('//'):
            continue
        lines.append(raw)
    return lines


def test_browser_never_speaks_gazebo_interfaces():
    """
    O navegador chama std_srvs, nunca ros_gz_interfaces.

    O rosbridge monta o pedido importando o pacote de interfaces dentro do
    próprio container, e o container do cockpit não tem ros_gz_interfaces — no
    modo `deploy`, sem Gazebo nenhum, nem faria sentido ter. A falha é uma
    InvalidModuleException em tempo de clique, longe de qualquer teste.
    """
    for path in (SIM_CONTROLS_JS, VIEW_CONTROLS_JS):
        code = '\n'.join(_code_lines(path.read_text(encoding='utf-8')))
        assert 'callService(' in code, f'{path.name} não chama nenhum serviço'
        assert 'ros_gz_interfaces' not in code, (
            f'{path.name} referencia ros_gz_interfaces em código'
        )
        assert '/demo/sim/control' not in code, (
            f'{path.name} chama o serviço com tipo do Gazebo direto; use a '
            'fachada std_srvs do sim_control_relay'
        )
        assert 'world_control' not in code, (
            f'{path.name} monta um WorldControl; esse vocabulário é do simulador'
        )


def test_sim_control_relay_ships_with_the_bridge():
    """A fachada e a ponte sobem juntas: uma sem a outra é um botão morto."""
    launch = SIM_CONTROL_LAUNCH.read_text(encoding='utf-8')
    assert "executable='sim_control_relay'" in launch
    assert 'ros_gz_bridge' in launch

    entry_points = (SIMULATION / 'setup.py').read_text(encoding='utf-8')
    assert 'sim_control_relay = demo_simulation.sim_control_relay:main' in entry_points


def test_sim_control_relay_serves_the_three_actions():
    """Os três botões do cockpit precisam de serviço do outro lado."""
    relay = SIM_CONTROL_RELAY.read_text(encoding='utf-8')
    # Os CAMINHOS, não os nomes: `play` e `pause` saem de um laço e `reset` tem
    # handler próprio, e o navegador só conhece o caminho.
    assert "f'/demo/sim/{action}'" in relay
    assert "for action in ('play', 'pause')" in relay
    assert "'/demo/sim/reset'" in relay, 'reset não é servido pela fachada'


def test_o_reset_nao_pode_voltar_a_apagar_o_robo():
    """
    `reset.all` APAGA a planta, e o cockpit não tem como perceber.

    O robô e as duas câmeras de cena são inseridos depois da carga do mundo
    (`ros_gz_sim create`); `reset.all` devolve o mundo ao SDF de origem, que não
    os contém. Medido em 26/08/2026: `/joint_states` 999 Hz -> morto,
    `/demo/imu` 996 Hz -> morto, `/demo/odom` 49,6 Hz -> morto, e
    `gz model -m demo_robot` respondendo `No model named <demo_robot>`.

    O que torna isto digno de um guarda é a APARÊNCIA: o relógio segue a 999 Hz
    e o Gazebo deixa os sensores órfãos publicando a 10 Hz, então todo painel do
    cockpit fica verde apontando para uma planta que não existe. Nada em log
    acusa. Evidência em `docs/results/cockpit-reset-nao-destrutivo.md`.
    """
    relay = SIM_CONTROL_RELAY.read_text(encoding='utf-8')
    # O campo, não a palavra: o cabeçalho CITA `reset.all` de propósito, para
    # que a próxima pessoa saiba por que ele não está sendo usado. O que não
    # pode existir é a atribuição.
    assert 'request.world_control.reset' not in relay, (
        'a fachada não pode escrever em nenhum campo de reset do WorldControl; '
        'reset.all apaga o robô e time_only salta o relógio para trás'
    )
    assert 'reset.all = True' not in relay
    # O caminho novo: teleporta pela mesma fachada que as câmeras de cena usam.
    assert 'SetEntityPose' in relay
    assert "'/demo/sim/set_entity_pose'" in relay


def test_sim_state_label_comes_from_the_clock():
    """
    O rótulo de estado não pode ser o eco do último clique.

    Um simulador morto, uma chamada expirada ou uma pausa feita pela GUI do
    Gazebo produzem exatamente o caso em que o eco mente.
    """
    source = SIM_CONTROLS_JS.read_text(encoding='utf-8')
    assert "'/clock'" in source
    assert 'throttleRate' in source, (
        '/clock publica a ~1 kHz; sem throttle o tráfego vai inteiro ao navegador'
    )


def test_view_pad_buttons_all_have_a_step():
    """Um data-command sem passo é um botão que não faz nada e não reclama."""
    html = INDEX_HTML.read_text(encoding='utf-8')
    commands = set(re.findall(r'data-role="view" data-command="([a-z-]+)"', html))
    steps = set(re.findall(r"^  '([a-z-]+)':", VIEW_CONTROLS_JS.read_text(encoding='utf-8'),
                           flags=re.MULTILINE))
    assert commands, 'nenhum botão de câmera no HTML'
    assert commands == steps, f'sem passo: {commands - steps}; sem botão: {steps - commands}'


def test_brand_palette_is_the_toradex_one():
    """
    Os três valores da marca são exatos e vieram do time. Um ajuste "só um
    tom mais escuro" num deles é a forma como uma identidade se perde.
    """
    tokens = TOKENS_CSS.read_text(encoding='utf-8')
    for name, value in (
        ('--brand-blue', '#00508c'),
        ('--brand-green', '#96c837'),
        ('--brand-orange', '#ff5a00'),
    ):
        assert f'{name}: {value};' in tokens, f'{name} deixou de ser {value}'
    assert '--bg-root: #ffffff;' in tokens, 'o fundo do cockpit é branco'


def test_canvas_colours_come_from_the_tokens():
    """
    Nenhum painel de canvas guarda cor própria.

    Quando o cockpit trocou o fundo preto pelo branco da marca, o ciano do
    plano e o amarelo do laser sumiram junto. Cor duplicada em JavaScript é o
    que faz a próxima troca de tema consertar quatro painéis e esquecer dois.
    """
    for name in ('nav-panel.js', 'detection-overlay.js'):
        source = (BUNDLE / 'js' / 'panels' / name).read_text(encoding='utf-8')
        literals = re.findall(r"= '(#[0-9a-fA-F]{3,8})'", source)
        assert not literals, f'{name} ainda tem cor literal: {literals}'
        assert 'readMapPalette' in source


def test_logos_are_present_and_have_alpha():
    """
    As marcas são tinta branca com alfa, e é por isso que a barra é azul.

    Um PNG opaco aqui viraria um retângulo branco sobre o azul — e o teste
    existe porque o arquivo é fácil de substituir por um JPG achatado.
    """
    for name in ('toradex.png', 'ros.png'):
        blob = (BUNDLE / 'img' / name).read_bytes()
        assert blob[:8] == b'\x89PNG\r\n\x1a\n', f'{name} não é PNG'
        # Byte 25 do IHDR é o color type; 6 = RGBA, 4 = cinza+alfa.
        assert blob[25] in (4, 6), f'{name} não tem canal alfa'

    html = INDEX_HTML.read_text(encoding='utf-8')
    assert 'img/toradex.png' in html and 'img/ros.png' in html
    assert 'alt="Toradex"' in html and 'alt="ROS 2"' in html, (
        'sem alt, um kiosk sem imagem não diz de quem é a demo'
    )


def test_stub_detections_are_not_drawn_over_the_video():
    """
    Nada desenha as detecções do stub sobre a câmera.

    O `demo_perception` de hoje varre uma caixa sintética pela imagem quer haja
    objeto ali ou não. Sobre o vídeo isso vira um retângulo passeando de um lado
    para o outro, e numa demo o espectador lê aquilo como detecção de verdade.

    O que NÃO está sob teste: as detecções continuarem chegando e alimentando a
    perception_layer do costmap. Isso é o contrato de tópicos do CLAUDE.md e não
    mudou — o que saiu foi só o desenho. O módulo de overlay segue no bundle,
    testado, para quando o TIDL substituir o stub.
    """
    html = INDEX_HTML.read_text(encoding='utf-8')
    assert 'data-role="detections"' not in html, (
        'o canvas de detecções voltou ao painel da câmera'
    )
    main = (BUNDLE / 'js' / 'main.js').read_text(encoding='utf-8')
    assert 'createDetectionOverlay' not in main
    assert (BUNDLE / 'js' / 'panels' / 'detection-overlay.js').exists(), (
        'o overlay deve continuar no bundle: ele volta com o TIDL'
    )


def test_scene_cameras_keep_the_measured_aspect_ratio():
    """
    A resolução pode subir; a proporção 4:3 não pode mudar.

    O horizontal_fov e as poses das duas câmeras foram medidos nesta proporção
    (ver o cabeçalho de scene_cameras.launch.py). Ir para 16:9 mantendo o hfov
    corta vertical e desenquadra as duas cenas de uma vez — sem erro nenhum,
    só um robô fora do quadro.
    """
    for name in ('cockpit_scene_iso.sdf', 'cockpit_scene_top.sdf'):
        sdf = (SIMULATION / 'models' / name).read_text(encoding='utf-8')
        width = int(re.search(r'<width>(\d+)</width>', sdf).group(1))
        height = int(re.search(r'<height>(\d+)</height>', sdf).group(1))
        assert width * 3 == height * 4, f'{name} deixou de ser 4:3 ({width}x{height})'


# --------------------------------------------------------------------------
# Câmera seguindo o robô, e reinício da navegação a partir do cockpit
# (pedido do operador, 24/08/2026: logo maior, resetar o alvo reiniciando o ROS
# de navegação no Aquila, e vista trackeada ao robô nas duas câmeras).
# --------------------------------------------------------------------------

SCENE_VIEW_CONTROLLER = SIMULATION / 'demo_simulation' / 'scene_view_controller.py'
NAVIGATION = REPO_ROOT / 'ros2_ws' / 'src' / 'demo_navigation'
NAV_CONTROL_RELAY = NAVIGATION / 'demo_navigation' / 'nav_control_relay.py'
NAV_CONTROL_LAUNCH = NAVIGATION / 'launch' / 'nav_control.launch.py'
NAV_PANEL_JS = BUNDLE / 'js' / 'panels' / 'nav-panel.js'
PANELS_CSS = BUNDLE / 'css' / 'panels.css'
LAYOUT_CSS = BUNDLE / 'css' / 'layout.css'
# Os dois entrypoints explícitos de navegação. Os dois sobem o
# lifecycle_manager_navigation, então os dois têm de expor a fachada de reinício
# — o modo como isso quebra é o botão funcionar num ROBOT_TYPE e não no outro.
NAV_ENTRYPOINTS = (
    NAVIGATION / 'launch' / 'navigation.launch.py',
    REPO_ROOT / 'ros2_ws' / 'src' / 'demo_bringup' / 'launch'
    / 'nav_quadruped.launch.py',
)


def _python_code(source: str) -> str:
    """
    Código Python sem comentário e sem docstring.

    Mesma razão do `_code_lines` acima, um andar mais fundo: os cabeçalhos destes
    nós EXPLICAM por que a localização não é resetada, e citam o nome do
    gerenciador ao fazê-lo. Uma busca ingênua proibiria a documentação da regra.

    Via ast, e não por regex: uma docstring com aspas triplas dentro de uma
    f-string é exatamente o caso que o regex erra em silêncio.
    """
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    return ast.unparse(ast.fix_missing_locations(tree))


def test_scene_views_follow_the_robot_on_both_cameras():
    """
    Seguir vale para as DUAS vistas, e o alvo é o robô.

    Uma implementação que seguisse só a iso passaria por qualquer teste de
    "existe seguimento" e falharia exatamente no botão iso/topo: a vista de topo
    é a que o operador usa para ver o robô caminhar pelo labirinto.
    """
    code = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    assert 'def follow(self, anchor)' in code, (
        'a Orbit não tem como seguir um alvo móvel'
    )
    # O laço do tique percorre self._orbits, que contém as duas câmeras. Um
    # `self._orbits['scene_iso']` literal aqui seria a regressão.
    assert 'for name, orbit in self._orbits.items()' in code
    assert "'scene_iso'" in code and "'scene_top'" in code


def test_scene_follow_state_is_published_not_echoed():
    """
    O botão `seguir` é pintado pelo nó, não pelo próprio clique.

    Mesma regra do rótulo de simulação: recarregar a página, abrir o cockpit numa
    segunda tela ou desligar o seguimento por linha de comando são três casos em
    que o clique local não sabe a resposta.
    """
    node = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    assert "'/demo/cockpit/scene/following'" in node, (
        'o nó não publica o estado do seguimento'
    )
    assert 'TRANSIENT_LOCAL' in node, (
        'sem durabilidade latched uma aba nova fica sem valor até a próxima '
        'mudança'
    )

    code = '\n'.join(_code_lines(VIEW_CONTROLS_JS.read_text(encoding='utf-8')))
    assert 'FOLLOWING_TOPIC' in code and 'client.subscribe(' in code, (
        'o cockpit não lê o estado do seguimento de tópico nenhum'
    )
    assert 'following = wanted' not in code, (
        'o botão está sendo pintado pelo clique; o valor tem de vir do nó'
    )


def test_follow_anchor_carries_the_odom_to_world_seed():
    """
    /demo/odom não é a pose no mundo nas duas plantas, e a diferença é silenciosa.

    No quadrúpede é ground truth do Gazebo. No diff-drive o plugin DiffDrive
    integra encoders a partir de zero, então a origem do odom é a pose de SPAWN.
    Sem o seed, um `x:=5` faz a câmera seguir um ponto 5 m ao lado do robô — e um
    `yaw:=` faz o erro crescer com a distância.
    """
    node = SCENE_VIEW_CONTROLLER.read_text(encoding='utf-8')
    for name in ('follow_offset_x', 'follow_offset_y', 'follow_offset_yaw'):
        assert f"'{name}'" in node, f'{name} não é declarado pelo nó'
    # O yaw tem de ser APLICADO, não apenas declarado: uma rotação ignorada é
    # exatamente o erro que cresce com a distância percorrida.
    assert 'math.cos(syaw)' in node and 'math.sin(syaw)' in node, (
        'o seed de yaw é declarado e não usado'
    )

    # A planta diff-drive passa a pose de spawn; a quadrúpede NÃO passa nada,
    # porque somaria a pose duas vezes sobre uma odometria que já é do mundo.
    diffdrive = (SIMULATION / 'launch' / 'simulation.launch.py').read_text(
        encoding='utf-8')
    assert "'follow_offset_x': LaunchConfiguration('x')" in diffdrive
    assert "'follow_offset_yaw': LaunchConfiguration('yaw')" in diffdrive

    quadruped = (SIMULATION / 'launch' / 'quadruped.launch.py').read_text(
        encoding='utf-8')
    assert 'follow_offset' not in quadruped, (
        'a planta quadrúpede tem odometria ground truth: um seed aqui somaria a '
        'pose de spawn duas vezes'
    )


def test_nav_reset_facade_ships_with_both_navigation_paths():
    """
    A fachada de reinício sobe nos DOIS caminhos de navegação.

    Os dois sobem o `lifecycle_manager_navigation`, então os dois podem ser
    reiniciados. Se ela subisse só num, o botão do cockpit funcionaria com um
    ROBOT_TYPE e não com o outro — sem erro em lugar nenhum, porque o serviço
    simplesmente não existiria.
    """
    for path in NAV_ENTRYPOINTS:
        source = path.read_text(encoding='utf-8')
        assert 'nav_control.launch.py' in source, (
            f'{path.name} sobe o Nav2 sem a fachada de reinício'
        )

    launch = NAV_CONTROL_LAUNCH.read_text(encoding='utf-8')
    assert "executable='nav_control_relay'" in launch

    entry_points = (NAVIGATION / 'setup.py').read_text(encoding='utf-8')
    assert (
        'nav_control_relay = demo_navigation.nav_control_relay:main'
        in entry_points
    ), 'o executável não está registrado; o launch falha ao encontrá-lo'


def test_nav_reset_never_uses_reset_startup_because_it_segfaults():
    """
    Reiniciar é PAUSE + RESUME, nunca RESET + STARTUP.

    Medido em 24/08/2026, learn, caminho quadrúpede: RESET seguido de STARTUP
    mata o `component_container_isolated` com SIGSEGV (exit code -11), sempre no
    segundo CONFIGURE do `route_server`, em "Configuring Rerouting service
    operation". Duas tentativas, duas mortes idênticas. Depois disso não existe
    navegação nenhuma — só `docker compose restart nav` traz de volta.

    `route_server` está na lista `lifecycle_nodes` do navigation_launch.py
    vendorizado, que é cópia upstream e tem de seguir idêntica, e este projeto
    não usa rota nenhuma. Enquanto ele estiver na lista gerenciada, RESET é
    proibido: um botão de consertar a navegação que mata a navegação é pior que
    nenhum botão.
    """
    relay = NAV_CONTROL_RELAY.read_text(encoding='utf-8')
    code = _python_code(relay)
    assert 'ManageLifecycleNodes.Request.PAUSE' in code
    assert 'ManageLifecycleNodes.Request.RESUME' in code
    for forbidden in ('Request.RESET', 'Request.STARTUP'):
        assert forbidden not in code, (
            f'{forbidden} volta a passar pelo CONFIGURE do route_server, que '
            'mata o container — ver o docstring deste teste'
        )
    # A alternativa ao CONFIGURE: o costmap é esvaziado pelos serviços dos
    # próprios nós de costmap, sem passar pelo ciclo de vida. Sem isto o reset
    # deixaria o obstáculo fantasma que suja uma demo longa.
    assert 'clear_entirely_global_costmap' in code
    assert 'clear_entirely_local_costmap' in code

    assert 'lifecycle_manager_localization' not in code, (
        'a fachada está mexendo na localização junto; ver o cabeçalho dela'
    )
    # O timeout é medido no relógio de PAREDE. Este nó roda com use_sim_time e um
    # Nav2 desativado coexiste com um /clock parado: medir no tempo simulado
    # transforma "expirou" em "espera para sempre".
    assert 'time.monotonic()' in relay
    assert 'self.get_clock()' not in relay


def test_nav_reset_needs_two_clicks_and_the_browser_speaks_std_srvs():
    """
    Reiniciar é destrutivo e leva dezenas de segundos: dois cliques, como o
    reset da simulação.

    E a chamada é a fachada std_srvs, não o manage_nodes. Aqui `nav2_msgs` até
    existe no container do cockpit — é o pacote do NavigateToPose que a meta
    usa — então o motivo não é o da armadilha do Gazebo: é que a sequência tem um
    estado inválido no meio e não pode depender da página continuar aberta.
    """
    code = '\n'.join(_code_lines(NAV_PANEL_JS.read_text(encoding='utf-8')))
    assert "'/demo/nav/reset'" in code, 'o painel não chama a fachada'
    assert 'manage_nodes' not in code, (
        'o navegador está conduzindo o ciclo de vida do Nav2 direto; um F5 no '
        'meio deixa a pilha desativada'
    )
    assert 'RESET_ARM_MS' in code and "dataset.armed = 'true'" in code, (
        'reiniciar a navegação não está atrás de confirmação'
    )


def test_toradex_logo_doubled_and_the_bar_grew_with_it():
    """
    A marca da Toradex é 2x a original, e a faixa reserva altura para ela.

    O 34 px original foi ajustado na tela contra o logo do ROS; o pedido era
    dobrar a Toradex. O que este teste guarda não é o número: é o acoplamento.
    `.bar` tem overflow-x e não -y, então subir o logo sem subir o mínimo da
    linha da grade CORTA a marca, sem barra de rolagem e sem erro nenhum.
    """
    tokens = TOKENS_CSS.read_text(encoding='utf-8')
    toradex = int(re.search(r'--logo-toradex:\s*(\d+)px', tokens).group(1))
    bar_min = int(re.search(r'--bar-min-height:\s*(\d+)px', tokens).group(1))

    assert toradex >= 68, (
        f'a marca da Toradex voltou a {toradex}px; o pedido era ao menos 2x os '
        '34px originais'
    )
    assert bar_min >= toradex + 10, (
        f'a faixa reserva {bar_min}px para um logo de {toradex}px mais 10px de '
        'padding: a marca é cortada'
    )

    # A altura vem do token nos dois lados. Um literal em px em qualquer um
    # deles é exatamente como os dois números divergem.
    panels = PANELS_CSS.read_text(encoding='utf-8')
    assert 'height: var(--logo-toradex)' in panels
    layout = LAYOUT_CSS.read_text(encoding='utf-8')
    assert 'minmax(var(--bar-min-height)' in layout
    assert 'minmax(48px' not in layout, (
        'a linha da barra voltou a um mínimo literal, dessincronizado do logo'
    )
