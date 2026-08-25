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


# Cenario OFICIAL de cada robo: (pacote, subdiretorio, arquivo).
#
# POR QUE E POR ROBO E NAO UM DEFAULT UNICO
#
# O labirinto e o cenario oficial da demo, e e no labirinto que a sintonia do
# Nav2 do quadrupede foi medida: corredor de 1,20 m, inflacao global 0,85,
# `vx_min` zero, arvore de comportamento com SmoothPath. Ver
# docs/results/ml35-navegacao-maze11.md.
#
# O diff-drive NAO herda isso. Ele e o fallback, e o que existe medido dele e no
# armazem -- inclusive o portao do F6 (meta de x=0 para x=1). Mudar o mundo dele
# junto trocaria o cenario de um teste que ja passou por um em que ele nunca
# rodou, e a meta de x=1 no labirinto cai numa parede. Um fallback que se quebra
# no dia em que se precisa dele nao e um fallback.
#
# Por isso a tabela tem duas linhas em vez de uma constante: a pergunta "qual e o
# mundo oficial" nao tem resposta unica, e fingir que tem e o que produz a
# combinacao errada em silencio.
OFFICIAL_WORLD = {
    'quadruped': ('demo_simulation', 'worlds', 'quadruped_maze11.sdf'),
    'diffdrive': ('nav2_minimal_tb4_sim', 'worlds', 'warehouse.sdf'),
}


def official_world(robot_type: str) -> tuple:
    """
    Cenario oficial de um robo, como (pacote, subdiretorio, arquivo).

    Devolve as tres partes em vez de um caminho pronto porque quem sabe montar o
    caminho e o launch, com FindPackageShare: o prefixo de instalacao nao existe
    em tempo de import, e cravar um aqui daria um caminho que so funciona na
    maquina onde foi escrito.
    """
    if robot_type not in OFFICIAL_WORLD:
        supported = ', '.join(sorted(OFFICIAL_WORLD))
        raise ValueError(
            f'robot_type:={robot_type!r} is not a known robot. '
            f'Supported: {supported}.'
        )
    return OFFICIAL_WORLD[robot_type]
