"""Behavioral tests for scripts/module.sh configuration precedence."""

from __future__ import annotations

import os
import re
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_SH = REPO_ROOT / 'scripts' / 'module.sh'
CONFIG_KEYS = (
    'MODULE_HOST',
    'MODULE_IP',
    'HOST_IP',
    'ROS_DOMAIN_ID',
    'TAG',
    'REGISTRY',
    'ROBOT_TYPE',
)


def load_config(env_file: Path, **overrides: str) -> dict[str, str]:
    """Source module.sh without dispatching a command or contacting the target."""
    env = os.environ.copy()
    for key in CONFIG_KEYS:
        env.pop(key, None)
    env.update(overrides)
    env['MODULE_ENV_FILE'] = str(env_file)

    fields = ' '.join(f'"${{{key}-}}"' for key in CONFIG_KEYS)
    command = (
        f'source "{MODULE_SH}"; '
        f'printf "%s\\037%s\\037%s\\037%s\\037%s\\037%s\\037%s" {fields}'
    )
    result = subprocess.run(
        ['bash', '-c', command],
        cwd=REPO_ROOT,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    return dict(zip(CONFIG_KEYS, result.stdout.split('\x1f'), strict=True))


def test_environment_wins_over_dotenv(tmp_path: Path) -> None:
    env_file = tmp_path / '.env'
    env_file.write_text(
        'HOST_IP=192.0.2.6\n'
        'MODULE_IP=192.0.2.5\n'
        'TAG=stale\n',
        encoding='utf-8',
    )

    config = load_config(
        env_file,
        HOST_IP='192.0.2.4',
        MODULE_IP='192.0.2.3',
        TAG='candidate',
    )

    assert config['HOST_IP'] == '192.0.2.4'
    assert config['MODULE_IP'] == '192.0.2.3'
    assert config['TAG'] == 'candidate'


def test_explicit_empty_environment_value_is_not_replaced(tmp_path: Path) -> None:
    env_file = tmp_path / '.env'
    env_file.write_text('HOST_IP=192.0.2.6\n', encoding='utf-8')

    config = load_config(env_file, HOST_IP='')

    assert config['HOST_IP'] == ''


def test_dotenv_fills_unset_values_and_defaults_fill_the_rest(
        tmp_path: Path) -> None:
    env_file = tmp_path / '.env'
    env_file.write_text(
        '  export MODULE_IP="192.0.2.3"\n'
        "REGISTRY='bench'\n"
        'not-a-key=ignored\n',
        encoding='utf-8',
    )

    config = load_config(env_file)

    assert config['MODULE_IP'] == '192.0.2.3'
    assert config['REGISTRY'] == 'bench'
    assert config['MODULE_HOST'] == 'aquila-am69-12593525.local'
    assert config['ROS_DOMAIN_ID'] == '69'
    assert config['TAG'] == 'dev'
    assert config['ROBOT_TYPE'] == 'quadruped'


def test_verify_starts_subscriber_before_remote_publisher() -> None:
    script = MODULE_SH.read_text(encoding='utf-8')
    subscriber = 'ros2 topic echo --once /demo/system/heartbeat'
    publisher = 'ros2 run demo_tutorials heartbeat_publisher'

    assert script.index(subscriber) < script.index(publisher)
    assert 'return "${verify_failed}"' in script


def test_verify_requires_the_host_to_module_topic_contract() -> None:
    script = MODULE_SH.read_text(encoding='utf-8')

    for topic in ('/clock', '/demo/odom', '/demo/scan',
                  '/demo/camera/image_raw'):
        assert topic in script


def test_module_up_recreates_containers_after_dds_render() -> None:
    script = MODULE_SH.read_text(encoding='utf-8')

    assert 'compose.module.yml up -d --force-recreate' in script


def test_every_required_topic_survives_the_collection_filter() -> None:
    """The filter that collects topics must not drop a topic the check demands.

    The weaker sibling test above only asserts that each topic name appears
    somewhere in the script. That passed while `verify` was collecting with a
    bare `grep /demo/` and then demanding /clock, which is not under /demo/ and
    could never match: the step reported "AUSENTE: /clock" against a module
    that was reading /clock at 616 Hz. Presence of a string is not evidence
    that the pipeline can produce it.
    """
    script = MODULE_SH.read_text(encoding='utf-8')

    loop = re.search(r'for required_topic in ([^;]+); do', script)
    assert loop, 'required-topic loop not found'
    required = loop.group(1).split()
    assert '/clock' in required

    collector = re.search(r'ros2 topic list \| grep ((?:-e \S+ ?)+)', script)
    assert collector, 'topic collection filter not found'
    patterns = re.findall(r'-e (\S+)', collector.group(1))
    assert patterns

    for topic in required:
        assert any(pat in topic for pat in patterns), (
            f'{topic} is required by verify but the collection filter '
            f'{patterns} drops it'
        )
