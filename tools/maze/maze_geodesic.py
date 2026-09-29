#!/usr/bin/env python3
"""
Mede, offline, QUANTO o Nav2 nao sabe quando recebe uma meta neste labirinto.

Roda no host x86, sem ROS e sem Gazebo. So leitura do STL.

    python3 tools/maze/maze_geodesic.py --models ~/ros_maze_worlds/models maze11

POR QUE ESTE SCRIPT EXISTE

`maze_fit.py` responde "este labirinto serve e as metas caem em corredor".
`maze_route.py` responde "esta e a sequencia para SAIR". Nenhum dos dois responde
a pergunta que a investigacao de F5 precisava:

    a meta que o ensaio manda e alcancavel pela LINHA RETA, ou o planejador
    global precisa inventar um caminho por espaco que o robo nunca viu?

A pergunta importa porque o costmap global do quadrupede e JANELA ROLANTE sem
camada estatica e sem mapa (`nav2_params_go2.yaml`, global_costmap), e o NavFn
roda com `allow_unknown: true`. As duas coisas juntas significam que uma meta
atras de uma parede ainda nao observada produz um plano ATRAVES da parede, sem
erro, sem aviso, e com aparencia perfeita em `/plan`.

O modo de falha e o pior deste projeto: nada acusa. O `compute_path_to_pose`
devolve `SUCCEEDED`, o caminho aparece no RViz e no cockpit, e o controlador
passa o ensaio inteiro tentando seguir uma reta que atravessa alvenaria.

O QUE ELE MEDE

  reta        distancia euclidiana spawn -> meta
  geodesica   caminho mais curto pelo espaco NAVEGAVEL (erodido por robot_radius)
  razao       geodesica / reta.  1.0 = a reta serve.  >1 = a reta mente.
  1a parede   onde a reta encosta na primeira parede
  visivel     fracao do espaco livre no alcance do lidar que o robo enxerga do
              spawn, com oclusao.  E o tamanho da ignorancia no instante zero.

CONVENCAO DE COORDENADAS

A mesma de `maze_fit.py` e de `MAZE11_GOALS` em `nav_trial.py`: origem na celula
de nascimento do robo, eixos do frame `map`. A verificacao no comeco de
`analyse_goals` falha alto se essa relacao quebrar -- frame trocado em silencio
e a classe de erro que este arquivo existe para nao cometer.
"""

from __future__ import annotations

import argparse
import heapq
import math
from pathlib import Path

import numpy as np
from scipy import ndimage

import maze_fit


# Raio circunscrito do tronco do Go2, igual ao `robot_radius` dos dois costmaps.
# O espaco navegavel e o costmap erodido por ele, nao o espaco livre cru: uma
# fresta de 0,20 m e livre e nao e navegavel, e uma geodesica que passasse por
# ela seria uma rota que o planejador nunca escolhe.
ROBOT_RADIUS_M = 0.383

# `obstacle_max_range` do global_costmap. NAO e o alcance declarado do L1
# (10 m): o que nao e marcado nao entra no costmap, entao o alcance que importa
# aqui e o do costmap, nao o do sensor.
LIDAR_RANGE_M = 8.0

# Resolucao dos dois costmaps. Medir noutra resolucao daria outra geodesica.
COSTMAP_RESOLUTION_M = 0.05

# Raios do raycast de visibilidade. 2880 = um a cada 0,125 grau; a 8 m isso da
# 1,7 cm de arco, abaixo da celula de 5 cm, entao nenhum corredor escapa por
# amostragem angular esparsa.
VISIBILITY_RAYS = 2880


def maze11_goals() -> list[tuple[float, float]]:
    """
    Le `MAZE11_GOALS` do fonte de `nav_trial.py`, por AST e nao por import.

    `nav_trial` importa `geometry_msgs` na primeira linha util, entao importa-lo
    aqui obrigaria este script a ter ROS -- e ele existe justamente para rodar
    offline. Copiar a tupla criaria duas fontes da verdade que divergem em
    silencio no dia em que alguem regenerar as metas com `maze_fit.py`.
    """
    import ast

    source = (Path(__file__).resolve().parents[1] / 'evaluation' / 'nav_trial.py').read_text()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if 'MAZE11_GOALS' in names:
                return [tuple(v) for v in ast.literal_eval(node.value)]
    raise SystemExit('MAZE11_GOALS nao encontrada em nav_trial.py')


def navigable_mask(clearance: np.ndarray) -> np.ndarray:
    """Celulas em que o centro do robo cabe."""
    return clearance >= ROBOT_RADIUS_M


def geodesic_field(mask: np.ndarray, row: int, col: int,
                   resolution: float) -> np.ndarray:
    """Dijkstra 8-conectado sobre `mask`, em metros, a partir de (row, col)."""
    dist = np.full(mask.shape, np.inf)
    dist[row, col] = 0.0
    diag = math.sqrt(2.0)
    steps = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag))
    queue = [(0.0, row, col)]
    rows, cols = mask.shape
    while queue:
        here, r, c = heapq.heappop(queue)
        if here > dist[r, c]:
            continue
        for dr, dc, weight in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < rows and 0 <= cc < cols and mask[rr, cc]:
                there = here + weight * resolution
                if there < dist[rr, cc]:
                    dist[rr, cc] = there
                    heapq.heappush(queue, (there, rr, cc))
    return dist


def descend(dist: np.ndarray, mask: np.ndarray,
            row: int, col: int) -> list[tuple[int, int]]:
    """Retrocaminha o gradiente da geodesica ate a origem."""
    path = [(row, col)]
    rows, cols = dist.shape
    while math.isfinite(dist[row, col]) and dist[row, col] > 0.0:
        best, br, bc = dist[row, col], row, col
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = row + dr, col + dc
                if 0 <= rr < rows and 0 <= cc < cols and mask[rr, cc]:
                    if dist[rr, cc] < best:
                        best, br, bc = dist[rr, cc], rr, cc
        if (br, bc) == (row, col):
            break
        row, col = br, bc
        path.append((row, col))
    return path


def visibility(free: np.ndarray, to_rc, resolution: float, reach: float,
               x0: float = 0.0, y0: float = 0.0) -> np.ndarray:
    """Raycast 2D com oclusao a partir de (x0, y0). Marca a celula batida."""
    seen = np.zeros_like(free)
    rows, cols = free.shape
    samples = int(reach / (resolution / 2.0))
    for k in range(VISIBILITY_RAYS):
        angle = 2.0 * math.pi * k / VISIBILITY_RAYS
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        for i in range(1, samples + 1):
            d = i * resolution / 2.0
            r, c = to_rc(x0 + d * cos_a, y0 + d * sin_a)
            if not (0 <= r < rows and 0 <= c < cols):
                break
            seen[r, c] = True
            if not free[r, c]:
                break
    return seen


def first_wall_from(free: np.ndarray, to_rc, resolution: float,
                    x0: float, y0: float,
                    x1: float, y1: float) -> tuple[float, str]:
    """Distancia ate a primeira celula NAO livre na reta (x0,y0) -> (x1,y1)."""
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    rows, cols = free.shape
    samples = max(1, int(length / (resolution / 2.0)))
    for i in range(1, samples + 1):
        t = i / samples
        r, c = to_rc(x0 + dx * t, y0 + dy * t)
        if not (0 <= r < rows and 0 <= c < cols):
            return t * length, 'fora da grade'
        if not free[r, c]:
            return t * length, 'parede'
    return length, 'livre'


def analyse_goals(name: str, models: Path, scale: float,
                  goals: list[tuple[float, float]],
                  chain: bool = False) -> dict:
    """
    Mede cada meta a partir do spawn, ou -- com `chain` -- da meta anterior.

    A distincao nao e cosmetica. `MAZE11_GOALS` e patrulha: cada meta e enviada
    com o robo onde a anterior o deixou, mas elas foram escolhidas por
    espalhamento e nao por conectividade, entao medir do spawn descreve bem a
    primeira e mal as outras. Uma rota do `maze_route.py` e o oposto: so faz
    sentido encadeada, e medi-la do spawn inventa paredes que a perna real nunca
    encontra. Chamar o modo errado produz uma tabela plausivel e falsa.
    """
    result = maze_fit.analyse(name, models, scale, COSTMAP_RESOLUTION_M,
                              start='se')
    component, _centred, clearance, gx, gy, row0, col0, res = result['_grid']
    free = clearance > 0.0

    def to_rc(x: float, y: float) -> tuple[int, int]:
        return (int(round(row0 + y / res)), int(round(col0 + x / res)))

    def to_map(r: int, c: int) -> tuple[float, float]:
        return (float(gx[c] - gx[col0]), float(gy[r] - gy[row0]))

    # O spawn TEM de ser a origem do frame das metas. Se um dia `maze_fit`
    # mudar de convencao, esta linha para o script em vez de publicar numeros
    # medidos no frame errado -- que passariam despercebidos.
    origin = to_map(row0, col0)
    if abs(origin[0]) > 1e-9 or abs(origin[1]) > 1e-9:
        raise SystemExit(f'convencao de frame quebrou: spawn em {origin}, '
                         'esperado (0, 0)')

    navigable = navigable_mask(clearance)
    if not navigable[row0, col0]:
        raise SystemExit('spawn nao e navegavel para robot_radius '
                         f'{ROBOT_RADIUS_M} m -- pose ou escala erradas')
    labels, _ = ndimage.label(navigable)
    reachable = labels == labels[row0, col0]

    dist = geodesic_field(reachable, row0, col0, res)
    spawn_seen = visibility(free, to_rc, res, LIDAR_RANGE_M)
    views = {(row0, col0): spawn_seen}

    rows_f, cols_f = np.nonzero(free)
    coords = np.array([to_map(r, c) for r, c in zip(rows_f, cols_f)])
    in_range = np.hypot(coords[:, 0], coords[:, 1]) <= LIDAR_RANGE_M

    rows_out = []
    from_x, from_y = 0.0, 0.0
    fields = {(row0, col0): dist}
    for x, y in goals:
        r, c = to_rc(x, y)
        r_from, c_from = to_rc(from_x, from_y)
        if (r_from, c_from) not in fields:
            fields[(r_from, c_from)] = geodesic_field(
                reachable, r_from, c_from, res)
            views[(r_from, c_from)] = visibility(
                free, to_rc, res, LIDAR_RANGE_M, from_x, from_y)
        field = fields[(r_from, c_from)]
        seen = views[(r_from, c_from)]

        dx, dy = x - from_x, y - from_y
        straight = math.hypot(dx, dy)
        inside = 0 <= r < field.shape[0] and 0 <= c < field.shape[1]
        geo = float(field[r, c]) if inside else math.inf
        # A reta e sempre medida da origem da PERNA, entao o raycast anda
        # deslocado: `first_wall` percorre de (0,0) ate o delta, e o resultado
        # so vale se a origem for o spawn. Para pernas encadeadas o raycast
        # precisa partir da perna anterior.
        wall_at, why = first_wall_from(free, to_rc, res,
                                       from_x, from_y, x, y)
        if math.isfinite(geo):
            path = descend(field, reachable, r, c)
            visible_frac = sum(1 for p in path if seen[p]) / len(path)
        else:
            path, visible_frac = [], float('nan')
        if chain:
            from_x, from_y = x, y
        rows_out.append({
            'goal': (x, y), 'straight': straight, 'geodesic': geo,
            'ratio': geo / straight if straight else math.nan,
            'wall_at': wall_at, 'why': why,
            'cells': len(path), 'visible_frac': visible_frac,
        })

    return {
        'name': name, 'scale': scale, 'resolution': res, 'chain': chain,
        'free_cells': int(free.sum()),
        'in_range_cells': int(in_range.sum()),
        'seen_cells': int((spawn_seen & free).sum()),
        'goals': rows_out,
    }


def report(data: dict) -> None:
    print(f"== {data['name']} @ escala {data['scale']}  "
          f"res {data['resolution']} m  robot_radius {ROBOT_RADIUS_M} m")
    seen_frac = 100.0 * data['seen_cells'] / max(1, data['in_range_cells'])
    print(f"   espaco livre                {data['free_cells']} celulas")
    print(f"   dentro de {LIDAR_RANGE_M:.0f} m do spawn      "
          f"{data['in_range_cells']} celulas")
    print(f"   VISIVEL do spawn            {data['seen_cells']} celulas "
          f"({seen_frac:.1f}% do que esta no alcance)")
    print()
    origem = 'da meta anterior' if data['chain'] else 'do spawn'
    print(f"   reta/geodesica/parede medidas {origem}")
    print(f"   {'meta':>16} {'reta':>7} {'geodesica':>10} {'razao':>6} "
          f"{'1a parede':>10}  {'rota visivel':>12}")
    for row in data['goals']:
        x, y = row['goal']
        geo = row['geodesic']
        geo_s = f'{geo:10.2f}' if math.isfinite(geo) else f"{'inalcanc.':>10}"
        vis = row['visible_frac']
        vis_s = f'{100 * vis:11.1f}%' if vis == vis else f"{'-':>12}"
        print(f"   ({x:6.2f},{y:6.2f}) {row['straight']:7.2f} {geo_s} "
              f"{row['ratio']:6.2f} {row['wall_at']:8.2f} m {vis_s}")
    print()
    worst = max(r['ratio'] for r in data['goals'] if math.isfinite(r['ratio']))
    blocked = sum(1 for r in data['goals'] if r['why'] == 'parede')
    print(f"   VEREDITO: {blocked} de {len(data['goals'])} metas tem parede na "
          f"reta; pior razao geodesica/reta {worst:.2f}x")
    if blocked:
        print("   -> com `allow_unknown: true` e costmap rolante sem mapa, o "
              "plano global\n"
              "      dessas metas atravessa parede nao observada, sem erro.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('maze', nargs='?', default='maze11')
    parser.add_argument('--models', type=Path,
                        default=Path.home() / 'ros_maze_worlds' / 'models')
    parser.add_argument('--scale', type=float, default=0.002,
                        help='mesma escala do SDF (0.002 no maze11)')
    parser.add_argument('--goals', default='',
                        help='"x,y;x,y;..."; vazio usa MAZE11_GOALS')
    parser.add_argument('--chain', action='store_true',
                        help='mede cada perna a partir da meta ANTERIOR; use '
                             'para rotas do maze_route.py, nunca para patrulha')
    args = parser.parse_args()

    if args.goals:
        goals = [tuple(float(v) for v in pair.split(','))
                 for pair in args.goals.split(';')]
    else:
        goals = maze11_goals()

    report(analyse_goals(args.maze, args.models, args.scale, goals,
                         chain=args.chain))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
