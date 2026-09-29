"""Map each supported robot to its matching plant, navigation and scenario."""


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


# OFFICIAL scenario for each robot: (package, subdirectory, file).
#
# WHY THIS IS PER ROBOT AND NOT A SINGLE DEFAULT
#
# The maze is the demo's official scenario, and it is in the maze that the
# quadruped's Nav2 tuning was measured: 1.20 m corridor, 0.85 global
# inflation, zero `vx_min`, behavior tree with SmoothPath. See
# docs/results/ml35-navegacao-maze11.md.
#
# The diff-drive does NOT inherit this. It is the fallback, and what has been
# measured for it is in the warehouse -- including the F6 gate (goal from
# x=0 to x=1). Changing its world along with the quadruped's would swap the
# scenario of a test that already passed for one it never ran on, and the
# x=1 goal in the maze lands inside a wall. A fallback that breaks on the day
# it is needed is not a fallback.
#
# That is why the table has two rows instead of one constant: the question
# "what is the official world" has no single answer, and pretending it does
# is what produces the wrong combination silently.
OFFICIAL_WORLD = {
    'quadruped': ('demo_simulation', 'worlds', 'quadruped_maze11.sdf'),
    'diffdrive': ('nav2_minimal_tb4_sim', 'worlds', 'warehouse.sdf'),
}


def official_world(robot_type: str) -> tuple:
    """
    Official scenario for a robot, as (package, subdirectory, file).

    Returns the three parts instead of a ready-made path because the launch
    file is what knows how to build the path, with FindPackageShare: the
    install prefix does not exist at import time, and hardcoding one here
    would give a path that only works on the machine it was written on.
    """
    if robot_type not in OFFICIAL_WORLD:
        supported = ', '.join(sorted(OFFICIAL_WORLD))
        raise ValueError(
            f'robot_type:={robot_type!r} is not a known robot. '
            f'Supported: {supported}.'
        )
    return OFFICIAL_WORLD[robot_type]
