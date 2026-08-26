#!/usr/bin/env python3
"""
Deriva do STL a ROTA DE SAIDA do labirinto: waypoints internos + meta externa.

Roda no host x86, offline, sem ROS e sem Gazebo. So leitura.

    python3 scripts/maze_route.py --models ~/ros_maze_worlds/models maze11

POR QUE ESTE SCRIPT EXISTE

`maze_fit.py` responde "este labirinto serve, e o robo nasce aqui". Ele sugere
metas ESPALHADAS dentro de um raio, que e o que uma patrulha quer. Nao e o que
uma TRAVESSIA quer: para sair, a sequencia de metas tem de seguir um caminho
conectado ate a abertura, na ordem certa, e terminar FORA.

A abertura nao pode ser escolhida a olho no Gazebo. Meta sobre parede e ACEITA
pelo Nav2 e falha depois, perto da borda, onde o erro ja nao tem nome -- e o
mesmo motivo pelo qual `MAZE11_GOALS` foi gerada e nao chutada.

O CRITERIO DE SUCESSO QUE ISTO TORNA POSSIVEL

`SUCCEEDED` na ultima meta NAO e "saiu do labirinto": o Nav2 declara sucesso
dentro de `xy_goal_tolerance` (0.25 m), e a tolerancia sozinha nao diz de que
LADO da parede o robo parou. Este script emite tambem a fronteira de saida, para
que o ensaio verifique CRUZAMENTO, que e geometrico e nao depende do Nav2
concordar consigo mesmo.

CONVENCAO DE COORDENADAS

Iguais as de `maze_fit.py` e do `patrol_commander`: origem na celula de
nascimento do robo, +x para frente, +y para a esquerda. A pose do SDF ja coloca
o labirinto nessa relacao -- ver o cabecalho de `quadruped_maze11.sdf`.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path

import numpy as np

import maze_fit


# Espacamento alvo entre waypoints. Nao e estetica.
#
# O horizonte do MPPI e 96 x 0.1 x 0.15 = 1.44 m (ver nav2_params_go2.yaml).
# Meta mais distante que o horizonte deixa o campo de custo quase plano perto do
# robo, que foi medido em 20/08 como a causa de "comando sai como ruido em torno
# de zero". Meta muito mais proxima que isso faz o goal_checker disparar antes de
# o robo ganhar velocidade, e a travessia vira uma sequencia de arranques.
WAYPOINT_SPACING_M = 1.4

# Quanto a meta externa fica ALEM da abertura. 1.0 m e mais que o raio
# circunscrito do tronco (0.383) somado a xy_goal_tolerance (0.25): garante que
# satisfazer a meta exige o corpo inteiro fora, nao a tolerancia encostada nela.
EXTERNAL_MARGIN_M = 1.0

# Peso do desvio de centro de corredor no custo do caminho. Com 0 o Dijkstra
# corta quina e raspa parede; alto demais e ele recusa corredor estreito legitimo.
CENTRE_WEIGHT = 2.0

NEIGHBOURS = tuple(
    (dr, dc, math.hypot(dr, dc))
    for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)
)


def boundary_openings(component: np.ndarray) -> list[tuple[int, int]]:
    """Celulas navegaveis encostadas na borda da grade: sao aberturas reais."""
    rows, cols = component.shape
    found = []
    for c in range(cols):
        if component[0, c]:
            found.append((0, c))
        if component[rows - 1, c]:
            found.append((rows - 1, c))
    for r in range(rows):
        if component[r, 0]:
            found.append((r, 0))
        if component[r, cols - 1]:
            found.append((r, cols - 1))
    return found


def group_openings(cells: list) -> list:
    """
    Agrupa celulas de borda contiguas num vao so.

    Contar CELULA e nao vao faria o relatorio dizer "10 aberturas" onde ha uma
    de 0.50 m, e quem lesse concluiria que existe rota alternativa.
    """
    groups: list = []
    for cell in sorted(cells):
        for group in groups:
            if any(abs(cell[0] - other[0]) <= 1 and abs(cell[1] - other[1]) <= 1
                   for other in group):
                group.append(cell)
                break
        else:
            groups.append([cell])
    return groups


def dijkstra(component, clearance, resolution, start):
    """Menor caminho ponderado por afastamento de parede. Devolve (custo, pai)."""
    best_clear = float(clearance[component].max())
    cost = {start: 0.0}
    parent: dict = {start: None}
    queue = [(0.0, start)]
    rows, cols = component.shape

    while queue:
        here, cell = heapq.heappop(queue)
        if here > cost.get(cell, math.inf):
            continue
        row, col = cell
        for drow, dcol, step in NEIGHBOURS:
            nrow, ncol = row + drow, col + dcol
            if not (0 <= nrow < rows and 0 <= ncol < cols):
                continue
            if not component[nrow, ncol]:
                continue
            # Penaliza proximidade de parede sem proibi-la.
            penalty = 1.0 + CENTRE_WEIGHT * (
                1.0 - float(clearance[nrow, ncol]) / best_clear)
            candidate = here + step * resolution * penalty
            if candidate < cost.get((nrow, ncol), math.inf):
                cost[(nrow, ncol)] = candidate
                parent[(nrow, ncol)] = cell
                heapq.heappush(queue, (candidate, (nrow, ncol)))
    return cost, parent


def trace(parent, target):
    path = []
    cell = target
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    return path[::-1]


def sample(path, clearance, resolution, spacing):
    """
    Amostra o caminho a cada `spacing` metros, encostando no centro do corredor.

    Sem o reencosto os waypoints caem onde o Dijkstra passou, que ja e razoavel
    mas nao e o melhor ponto local: uma meta 5 cm mais para o centro custa nada
    e tira o robo da faixa onde o CostCritic o penaliza.
    """
    picked = [path[0]]
    walked = 0.0
    for previous, cell in zip(path, path[1:]):
        walked += math.hypot(cell[0] - previous[0], cell[1] - previous[1]) * resolution
        if walked < spacing:
            continue
        walked = 0.0
        row, col = cell
        window = clearance[max(row - 2, 0):row + 3, max(col - 2, 0):col + 3]
        offset = np.unravel_index(int(np.argmax(window)), window.shape)
        picked.append((max(row - 2, 0) + offset[0], max(col - 2, 0) + offset[1]))
    if picked[-1] != path[-1]:
        picked.append(path[-1])
    return picked


def build(name: str, models: Path, scale: float, resolution: float,
          spacing: float) -> dict:
    result = maze_fit.analyse(name, models, scale, resolution, start='se')
    component, _centred, clearance, gx, gy, srow, scol, res = result['_grid']

    def to_robot(cell):
        return (float(gx[cell[1]] - gx[scol]), float(gy[cell[0]] - gy[srow]))

    openings = boundary_openings(component)
    gaps = group_openings(openings)
    if not openings:
        raise SystemExit(f'{name}: nenhuma abertura navegavel na borda da grade')

    cost, parent = dijkstra(component, clearance, res, (srow, scol))
    reachable = [cell for cell in openings if cell in cost]
    if not reachable:
        raise SystemExit(f'{name}: a abertura existe mas nao e alcancavel da partida')
    exit_cell = min(reachable, key=lambda cell: cost[cell])

    path = trace(parent, exit_cell)
    raw_m = sum(math.hypot(b[0] - a[0], b[1] - a[1]) * res
                for a, b in zip(path, path[1:]))
    waypoints = [to_robot(cell) for cell in sample(path, clearance, res, spacing)]

    # A meta externa sai na direcao da borda que a abertura toca.
    rows, cols = component.shape
    if exit_cell[0] == 0:
        heading = (0.0, -1.0)
    elif exit_cell[0] == rows - 1:
        heading = (0.0, +1.0)
    elif exit_cell[1] == 0:
        heading = (-1.0, 0.0)
    else:
        heading = (+1.0, 0.0)

    opening = to_robot(exit_cell)
    external = (opening[0] + heading[0] * EXTERNAL_MARGIN_M,
                opening[1] + heading[1] * EXTERNAL_MARGIN_M)

    return {
        'name': name, 'grid': component.shape, 'resolution': res,
        'start_cell': (srow, scol), 'exit_cell': exit_cell,
        'gaps': len(gaps), 'gap_width_m': max(len(g) for g in gaps) * res,
        'reachable_cells': len(reachable),
        'weighted_cost_m': cost[exit_cell], 'path_m': raw_m,
        'opening': opening, 'opening_clearance': float(clearance[exit_cell]),
        'external': external, 'heading': heading,
        'waypoints': waypoints[1:],  # o primeiro e a propria partida
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mazes', nargs='+')
    parser.add_argument('--models', type=Path,
                        default=Path.home() / 'ros_maze_worlds' / 'models')
    parser.add_argument('--scale', type=float, default=0.002)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--spacing', type=float, default=WAYPOINT_SPACING_M)
    args = parser.parse_args()

    for name in args.mazes:
        r = build(name, args.models, args.scale, args.resolution, args.spacing)
        print(f"=== {r['name']} ===")
        print(f"grade {r['grid'][0]}x{r['grid'][1]} @ {r['resolution']} m")
        print(f"partida celula {r['start_cell']}  saida celula {r['exit_cell']}")
        print(f"vaos na borda: {r['gaps']} "
              f"(o maior com {r['gap_width_m']:.2f} m de liberdade de centro; "
              f"{r['reachable_cells']} celulas alcancaveis da partida)")
        print(f"caminho interno: {r['path_m']:.2f} m "
              f"(custo ponderado {r['weighted_cost_m']:.2f})")
        print(f"abertura em ({r['opening'][0]:.2f}, {r['opening'][1]:.2f}) "
              f"folga {r['opening_clearance']:.2f} m")
        print(f"meta externa ({r['external'][0]:.2f}, {r['external'][1]:.2f})")
        print(f"\n{len(r['waypoints'])} waypoints, prontos para --goals:")
        print(';'.join(f'{x:.2f},{y:.2f}' for x, y in r['waypoints']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
