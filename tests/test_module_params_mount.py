"""Contract for iterating Nav2 parameters on the module without a rebuild.

``compose.module.yml`` bind-mounts the synced ``demo_navigation/config`` over
the copy baked into the arm64 image. Two things make that work, and both fail
silently if broken:

* the mount target must be the **final target** of the symlink colcon installs,
  not the installed path. Mounting over the installed path replaces a symlink
  with a directory and the Nav2 servers keep reading the image's copy — same
  parameters, no error, and an A/B campaign that compares a condition with
  itself. That exact class of false-positive is what §3 of
  ``docs/ml35/proximos-passos-navegacao.md`` was written about.
* the source must be the path ``module.sh sync`` actually populates, or the
  mount lands an empty directory over the config and every managed node fails
  to configure with a file-not-found that names the container path, not the
  cause.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_COMPOSE = ROOT / 'docker/compose.module.yml'
MODULE_SH = ROOT / 'scripts/module.sh'
CONFIG_DIR = ROOT / 'ros2_ws/src/demo_navigation/config'

# O alvo final do symlink que o colcon instala, verificado dentro do container
# em 27/08/2026. Se o layout do colcon mudar, este teste e o que avisa.
CONTAINER_CONFIG = '/ws/src/demo_navigation/config'
HOST_CONFIG = './ros2_ws/src/demo_navigation/config'

# O mesmo, para o Python do explorador. Verificado dentro do container em
# 29/08/2026: `import demo_navigation.maze_explorer` carrega
# /ws/build/demo_navigation/demo_navigation/maze_explorer.py, e
# /ws/build/demo_navigation/demo_navigation e um SYMLINK de diretorio para
# /ws/src/demo_navigation/demo_navigation. O alvo final e o mesmo padrao da
# config: montar no caminho instalado ou no de build seria ignorado.
CONTAINER_PKG = '/ws/src/demo_navigation/demo_navigation'
HOST_PKG = './ros2_ws/src/demo_navigation/demo_navigation'
PKG_DIR = ROOT / 'ros2_ws/src/demo_navigation/demo_navigation'


def _compose() -> dict:
    return yaml.safe_load(MODULE_COMPOSE.read_text(encoding='utf-8'))


def _common_volumes() -> list[str]:
    doc = _compose()
    # O bloco ancora `x-common` (ou equivalente) carrega os volumes comuns.
    for key, value in doc.items():
        if key.startswith('x-') and isinstance(value, dict) and 'volumes' in value:
            return list(value['volumes'])
    raise AssertionError('bloco comum com `volumes` sumiu de compose.module.yml')


def test_config_is_mounted_from_the_synced_tree() -> None:
    mounts = [v for v in _common_volumes() if CONTAINER_CONFIG in v]
    assert len(mounts) == 1, (
        f'esperava exatamente uma montagem sobre {CONTAINER_CONFIG}, '
        f'achei {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == HOST_CONFIG, (
        f'origem {source} nao e o que `module.sh sync` popula ({HOST_CONFIG})')
    assert target == CONTAINER_CONFIG
    assert 'ro' in flags, 'a config e lida, nunca escrita pelo container'


def test_explorer_source_is_mounted_from_the_synced_tree() -> None:
    """F5 roda uma variavel por rodada, e toda rodada edita maze_explorer.py.

    Sem esta montagem cada rodada custa um rebuild arm64 NATIVO de `base` e
    depois `nav` no proprio modulo. Com ela custa `module.sh sync` mais um
    `docker compose restart nav`.
    """
    mounts = [v for v in _common_volumes() if CONTAINER_PKG in v]
    assert len(mounts) == 1, (
        f'esperava exatamente uma montagem sobre {CONTAINER_PKG}, '
        f'achei {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == HOST_PKG, (
        f'origem {source} nao e o que `module.sh sync` popula ({HOST_PKG})')
    assert target == CONTAINER_PKG
    assert 'ro' in flags, (
        'o pacote e lido, nunca escrito pelo container -- o interpretador '
        'apenas deixa de gravar __pycache__, sem erro')


def test_explorer_mount_does_not_hide_a_module_that_only_exists_in_the_image(
) -> None:
    """A montagem cobre o pacote INTEIRO, entao ele tem de estar completo."""
    present = {p.name for p in PKG_DIR.glob('*.py')}
    for required in ('__init__.py', 'maze_explorer.py', 'frontier.py'):
        assert required in present, (
            f'{required} sumiu de {PKG_DIR}; a montagem o esconderia da '
            'imagem e o no do explorador nao subiria')


def test_mount_target_is_the_symlink_target_not_the_installed_path() -> None:
    """O inverso, que e o que importa: montar em install/ seria silencioso."""
    mounts = [v for v in _common_volumes() if 'demo_navigation/config' in v]
    for mount in mounts:
        target = mount.split(':')[1]
        assert '/install/' not in target, (
            f'montagem em {target}: o caminho instalado e um SYMLINK, e o Nav2 '
            'abre o alvo. A montagem seria ignorada sem nenhum erro.')
        assert '/build/' not in target, (
            f'montagem em {target}: /ws/build tambem e symlink para /ws/src')


def test_sync_ships_the_directory_the_mount_expects() -> None:
    """Se o rsync parar de mandar ros2_ws/src, a montagem fica vazia."""
    source = MODULE_SH.read_text(encoding='utf-8')
    # O comando ocupa varias linhas com continuacao por barra invertida, entao
    # a busca atravessa a continuacao ate o argumento de origem.
    assert re.search(r'rsync\b(?:[^\n]*\\\n)*[^\n]*\bros2_ws/src\b', source), (
        'module.sh nao sincroniza mais ros2_ws/src -- a montagem de config '
        'passaria a cobrir a config da imagem com um diretorio vazio')


def test_every_params_file_the_mount_shadows_exists_in_the_tree() -> None:
    """Montagem que esconde um arquivo que so existe na imagem quebra tudo."""
    present = {p.name for p in CONFIG_DIR.glob('*.yaml')}
    for required in ('nav2_params.yaml', 'nav2_params_go2.yaml'):
        assert required in present, (
            f'{required} sumiu de {CONFIG_DIR}; a montagem o esconderia da '
            'imagem e os servidores do Nav2 nao configurariam')


def test_raytrace_clearing_stays_on_in_both_costmaps() -> None:
    """Trava um experimento REPROVADO para que ninguém o repita.

    `clearing: false` no costmap global parece "dar memória ao mapa" e é a
    primeira ideia de qualquer um que leia a oscilação de plano medida em
    27/08. Foi medido e reprovado: `clearing` é o raytrace, e o raytrace é o
    único mecanismo que torna célula desconhecida em LIVRE. Desligá-lo deixou
    150 de 161 células da reta até a meta em 255 (desconhecido) e tornou o
    atalho pelo desconhecido MAIS atraente.

    Evidência: docs/results/ml35-f5-memoria-costmap.md.
    """
    params = yaml.safe_load(
        (CONFIG_DIR / 'nav2_params_go2.yaml').read_text(encoding='utf-8'))
    for scope in ('local_costmap', 'global_costmap'):
        cloud = (params[scope][scope]['ros__parameters']
                 ['obstacle_layer']['cloud'])
        assert cloud['clearing'] is True, (
            f'{scope}: `clearing: false` foi medido e REPROVADO -- ele desliga '
            'o raytrace, que e o que estabelece espaco livre. Ver '
            'docs/results/ml35-f5-memoria-costmap.md antes de tentar de novo.')
        assert cloud['marking'] is True


PERCEPTION_CONTAINER_PKG = '/ws/src/demo_perception/demo_perception'
PERCEPTION_HOST_PKG = './ros2_ws/src/demo_perception/demo_perception'
PERCEPTION_PKG_DIR = ROOT / 'ros2_ws/src/demo_perception/demo_perception'


def test_detector_source_is_mounted_from_the_synced_tree() -> None:
    """O detector virou o arquivo que muda por rodada, e ele e arm64.

    R6 mediu a pose da saida publicada em 0,478 da distancia real (12 amostras,
    R5+R6). Corrigir isso e iterar sobre `maze_exit_detector.py`, que vive na
    imagem de percepcao -- cujo rebuild roda sob QEMU. Mesma montagem, mesmo
    argumento de nao-divergencia da montagem do explorador acima.
    """
    mounts = [v for v in _common_volumes() if PERCEPTION_CONTAINER_PKG in v]
    assert len(mounts) == 1, (
        f'esperava exatamente uma montagem sobre {PERCEPTION_CONTAINER_PKG}, '
        f'achei {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == PERCEPTION_HOST_PKG
    assert target == PERCEPTION_CONTAINER_PKG
    assert 'ro' in flags


def test_detector_mount_does_not_hide_a_module_that_only_exists_in_the_image(
) -> None:
    """A montagem cobre o pacote inteiro; faltar um modulo derruba o no."""
    present = {p.name for p in PERCEPTION_PKG_DIR.glob('*.py')}
    for required in ('__init__.py', 'maze_exit_detector.py'):
        assert required in present, (
            f'{required} sumiu de {PERCEPTION_PKG_DIR}; a montagem o esconderia '
            'da imagem e o no de percepcao nao subiria')
