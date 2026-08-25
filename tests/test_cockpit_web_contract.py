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
    origin, and the defaults here cannot suit it. Read through the AST rather
    than by substring: the argument names are built with f-strings, so a
    grep-style check would pass on a file that declares nothing.
    """
    tree = ast.parse(SCENE_CAMERAS_LAUNCH.read_text())
    defaults = next(
        node.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(getattr(t, 'id', None) == 'DEFAULTS' for t in node.targets)
    )
    poses = ast.literal_eval(defaults)

    assert set(poses) == {'iso', 'top'}, 'as duas vistas precisam de default'
    for name, pose in poses.items():
        # x, y, z, pitch, yaw. Roll is deliberately absent.
        assert len(pose) == 5, f'pose de {name} incompleta'
        for value in pose:
            float(value)

    launch = SCENE_CAMERAS_LAUNCH.read_text()
    for axis in ('x', 'y', 'z', 'pitch', 'yaw'):
        assert f"scene_{{name}}_{axis}" in launch, (
            f'o eixo {axis} não é declarado como argumento de launch'
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
    relay = SIM_CONTROL_RELAY.read_text(encoding='utf-8')
    for action in ('play', 'pause', 'reset'):
        assert f"'{action}'" in relay, f'{action} não é servido pela fachada'
    # reset.all e não time_only: só ele devolve o robô à pose inicial.
    assert 'reset.all = True' in relay


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
