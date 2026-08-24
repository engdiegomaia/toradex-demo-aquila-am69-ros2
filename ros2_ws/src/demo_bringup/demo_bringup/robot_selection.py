"""Map each supported robot to its matching plant and navigation launches."""


ROBOT_LAUNCH_FILES = {
    'diffdrive': {
        'plant': 'simulation.launch.py',
        'navigation': 'nav.launch.py',
    },
    'quadruped': {
        'plant': 'quadruped.launch.py',
        'navigation': 'nav_quadruped.launch.py',
    },
}


def launch_file(robot_type: str, role: str) -> str:
    """Return the launch for a robot/role pair, failing loudly on bad input."""
    if robot_type not in ROBOT_LAUNCH_FILES:
        supported = ', '.join(sorted(ROBOT_LAUNCH_FILES))
        raise ValueError(
            f'robot_type:={robot_type!r} is not a known robot. '
            f'Supported: {supported}.'
        )

    if role not in ('plant', 'navigation'):
        raise ValueError(
            f'role:={role!r} is not known. Supported: navigation, plant.'
        )

    return ROBOT_LAUNCH_FILES[robot_type][role]
