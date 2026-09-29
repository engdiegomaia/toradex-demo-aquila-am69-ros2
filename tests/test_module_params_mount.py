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

# The final target of the symlink colcon installs, verified inside the container
# on 27/08/2026. If colcon's layout changes, this test is what warns.
CONTAINER_CONFIG = '/ws/src/demo_navigation/config'
HOST_CONFIG = './ros2_ws/src/demo_navigation/config'

# The same, for the explorer's Python. Verified inside the container on
# 29/08/2026: `import demo_navigation.maze_explorer` loads
# /ws/build/demo_navigation/demo_navigation/maze_explorer.py, and
# /ws/build/demo_navigation/demo_navigation is a DIRECTORY SYMLINK to
# /ws/src/demo_navigation/demo_navigation. The final target follows the same
# pattern as the config: mounting on the installed or build path would be
# ignored.
CONTAINER_PKG = '/ws/src/demo_navigation/demo_navigation'
HOST_PKG = './ros2_ws/src/demo_navigation/demo_navigation'
PKG_DIR = ROOT / 'ros2_ws/src/demo_navigation/demo_navigation'


def _compose() -> dict:
    return yaml.safe_load(MODULE_COMPOSE.read_text(encoding='utf-8'))


def _common_volumes() -> list[str]:
    doc = _compose()
    # The `x-common` anchor block (or equivalent) carries the common volumes.
    for key, value in doc.items():
        if key.startswith('x-') and isinstance(value, dict) and 'volumes' in value:
            return list(value['volumes'])
    raise AssertionError('common block with `volumes` disappeared from compose.module.yml')


def test_config_is_mounted_from_the_synced_tree() -> None:
    mounts = [v for v in _common_volumes() if CONTAINER_CONFIG in v]
    assert len(mounts) == 1, (
        f'expected exactly one mount over {CONTAINER_CONFIG}, '
        f'found {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == HOST_CONFIG, (
        f'source {source} is not what `module.sh sync` populates ({HOST_CONFIG})')
    assert target == CONTAINER_CONFIG
    assert 'ro' in flags, 'the config is read, never written by the container'


def test_explorer_source_is_mounted_from_the_synced_tree() -> None:
    """F5 runs one variable per round, and every round edits maze_explorer.py.

    Without this mount each round costs a NATIVE arm64 rebuild of `base` and
    then `nav` on the module itself. With it, a round costs `module.sh sync`
    plus a `docker compose restart nav`.
    """
    mounts = [v for v in _common_volumes() if CONTAINER_PKG in v]
    assert len(mounts) == 1, (
        f'expected exactly one mount over {CONTAINER_PKG}, '
        f'found {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == HOST_PKG, (
        f'source {source} is not what `module.sh sync` populates ({HOST_PKG})')
    assert target == CONTAINER_PKG
    assert 'ro' in flags, (
        'the package is read, never written by the container -- the '
        'interpreter just stops writing __pycache__, without error')


def test_explorer_mount_does_not_hide_a_module_that_only_exists_in_the_image(
) -> None:
    """The mount covers the WHOLE package, so it must be complete."""
    present = {p.name for p in PKG_DIR.glob('*.py')}
    for required in ('__init__.py', 'maze_explorer.py', 'frontier.py'):
        assert required in present, (
            f'{required} disappeared from {PKG_DIR}; the mount would hide it '
            'from the image and the explorer node would not start')


def test_mount_target_is_the_symlink_target_not_the_installed_path() -> None:
    """The inverse, which is what matters: mounting on install/ would be silent."""
    mounts = [v for v in _common_volumes() if 'demo_navigation/config' in v]
    for mount in mounts:
        target = mount.split(':')[1]
        assert '/install/' not in target, (
            f'mount on {target}: the installed path is a SYMLINK, and Nav2 '
            'opens the target. The mount would be ignored with no error.')
        assert '/build/' not in target, (
            f'mount on {target}: /ws/build is also a symlink to /ws/src')


def test_sync_ships_the_directory_the_mount_expects() -> None:
    """If rsync stops shipping ros2_ws/src, the mount ends up empty."""
    source = MODULE_SH.read_text(encoding='utf-8')
    # The command spans several lines with backslash continuation, so the
    # search crosses the continuation up to the source argument.
    assert re.search(r'rsync\b(?:[^\n]*\\\n)*[^\n]*\bros2_ws/src\b', source), (
        'module.sh no longer syncs ros2_ws/src -- the config mount would '
        'cover the image config with an empty directory')


def test_every_params_file_the_mount_shadows_exists_in_the_tree() -> None:
    """A mount that hides a file that only exists in the image breaks everything."""
    present = {p.name for p in CONFIG_DIR.glob('*.yaml')}
    for required in ('nav2_params.yaml', 'nav2_params_go2.yaml'):
        assert required in present, (
            f'{required} disappeared from {CONFIG_DIR}; the mount would hide it '
            'from the image and the Nav2 servers would not configure')


def test_raytrace_clearing_stays_on_in_both_costmaps() -> None:
    """Locks a FAILED experiment so that nobody repeats it.

    `clearing: false` in the global costmap looks like "giving the map memory"
    and is the first idea of anyone who reads the plan oscillation measured on
    27/08. It was measured and failed: `clearing` is the raytrace, and the
    raytrace is the only mechanism that turns an unknown cell into FREE.
    Turning it off left 150 of 161 cells on the line to the goal at 255
    (unknown) and made the shortcut through the unknown MORE attractive.

    Evidence: docs/results/ml35-f5-memoria-costmap.md.
    """
    params = yaml.safe_load(
        (CONFIG_DIR / 'nav2_params_go2.yaml').read_text(encoding='utf-8'))
    for scope in ('local_costmap', 'global_costmap'):
        cloud = (params[scope][scope]['ros__parameters']
                 ['obstacle_layer']['cloud'])
        assert cloud['clearing'] is True, (
            f'{scope}: `clearing: false` was measured and FAILED -- it turns off '
            'the raytrace, which is what establishes free space. See '
            'docs/results/ml35-f5-memoria-costmap.md before trying again.')
        assert cloud['marking'] is True


PERCEPTION_CONTAINER_PKG = '/ws/src/demo_perception/demo_perception'
PERCEPTION_HOST_PKG = './ros2_ws/src/demo_perception/demo_perception'
PERCEPTION_PKG_DIR = ROOT / 'ros2_ws/src/demo_perception/demo_perception'


def test_detector_source_is_mounted_from_the_synced_tree() -> None:
    """The detector became the file that changes every round, and it is arm64.

    R6 measured the published exit pose at 0.478 of the true distance (12
    samples, R5+R6). Fixing that means iterating on `maze_exit_detector.py`,
    which lives in the perception image -- whose rebuild runs under QEMU. Same
    mount, same no-divergence argument as the explorer mount above.
    """
    mounts = [v for v in _common_volumes() if PERCEPTION_CONTAINER_PKG in v]
    assert len(mounts) == 1, (
        f'expected exactly one mount over {PERCEPTION_CONTAINER_PKG}, '
        f'found {mounts}')
    source, target, *flags = mounts[0].split(':')
    assert source == PERCEPTION_HOST_PKG
    assert target == PERCEPTION_CONTAINER_PKG
    assert 'ro' in flags


def test_detector_mount_does_not_hide_a_module_that_only_exists_in_the_image(
) -> None:
    """The mount covers the whole package; a missing module takes the node down."""
    present = {p.name for p in PERCEPTION_PKG_DIR.glob('*.py')}
    for required in ('__init__.py', 'maze_exit_detector.py'):
        assert required in present, (
            f'{required} disappeared from {PERCEPTION_PKG_DIR}; the mount would '
            'hide it from the image and the perception node would not start')
