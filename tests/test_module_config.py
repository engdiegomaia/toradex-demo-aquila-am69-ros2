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
        'HOST_IP=192.0.2.190\n'
        'MODULE_IP=192.0.2.130\n'
        'TAG=stale\n',
        encoding='utf-8',
    )

    config = load_config(
        env_file,
        HOST_IP='192.0.2.109',
        MODULE_IP='192.0.2.67',
        TAG='candidate',
    )

    assert config['HOST_IP'] == '192.0.2.109'
    assert config['MODULE_IP'] == '192.0.2.67'
    assert config['TAG'] == 'candidate'


def test_explicit_empty_environment_value_is_not_replaced(tmp_path: Path) -> None:
    env_file = tmp_path / '.env'
    env_file.write_text('HOST_IP=192.0.2.190\n', encoding='utf-8')

    config = load_config(env_file, HOST_IP='')

    assert config['HOST_IP'] == ''


def test_dotenv_fills_unset_values_and_defaults_fill_the_rest(
        tmp_path: Path) -> None:
    env_file = tmp_path / '.env'
    env_file.write_text(
        '  export MODULE_IP="192.0.2.67"\n'
        "REGISTRY='bench'\n"
        'not-a-key=ignored\n',
        encoding='utf-8',
    )

    config = load_config(env_file)

    assert config['MODULE_IP'] == '192.0.2.67'
    assert config['REGISTRY'] == 'bench'
    assert config['MODULE_HOST'] == 'aquila-am69.local'
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


def test_verify_requires_a_real_clock_sample_not_only_discovery() -> None:
    script = MODULE_SH.read_text(encoding='utf-8')

    assert 'ros2 topic echo --once /clock rosgraph_msgs/msg/Clock' in script
    assert "grep -q '^clock:'" in script
    assert '/clock: SEM MENSAGEM' in script


def test_hil_render_prefers_routed_interface_over_loopback() -> None:
    """HIL external unicast must not be bound to the preferred loopback."""
    script = MODULE_SH.read_text(encoding='utf-8')

    assert '<NetworkInterface name=\\"lo\\" priority=\\"default\\"' in script
    assert 'priority=\\"10\\"' in script
    assert 'did not downgrade loopback in HIL mode' in script
    assert 'did not prioritize the routed interface in HIL mode' in script


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


def test_verify_checks_that_nav2_is_active_and_not_merely_running() -> None:
    """Topico existir nao e servico funcionar.

    Em 26/08/2026 as tres etapas anteriores passaram inteiras contra um Nav2
    cujo bringup havia ABORTADO: o `local_costmap` nao ativou porque a TF
    `odom -> base` nao atravessou a fronteira em 60 s, e o gerenciador desistiu
    em definitivo. `verify` retornou 0 e toda meta era recusada com
    "Action server is inactive".
    """
    script = MODULE_SH.read_text(encoding='utf-8')

    assert 'ros2 lifecycle get /bt_navigator' in script, (
        'verify nao pergunta o estado de ciclo de vida do Nav2'
    )

    # A resposta tem de DECIDIR o resultado, nao so ser impressa.
    tail = script[script.index('ros2 lifecycle get /bt_navigator'):]
    decision = tail[:tail.index('return "${verify_failed}"')]
    assert "grep -Eq '^active([[:space:]]|$)'" in decision
    assert 'grep -q active' not in decision
    assert "tail -1 || true" in decision
    assert 'verify_failed=1' in decision

    # E as etapas precisam estar renumeradas, senao o operador le "3/3" e
    # conclui que a bateria acabou antes da etapa que importa.
    for step in ('1/4', '2/4', '3/4', '4/4'):
        assert f'say "{step}' in script, f'etapa {step} ausente'


def test_nav2_lifecycle_match_rejects_inactive() -> None:
    pattern = r'^active([[:space:]]|$)'

    for state in ('active', 'active [3]'):
        result = subprocess.run(
            ['grep', '-Eq', pattern], input=state, text=True, check=False)
        assert result.returncode == 0, f'{state!r} deveria ser aceito'

    for state in ('inactive', 'inactive [2]', 'unconfigured [1]', ''):
        result = subprocess.run(
            ['grep', '-Eq', pattern], input=state, text=True, check=False)
        assert result.returncode != 0, f'{state!r} nao deveria ser aceito'
