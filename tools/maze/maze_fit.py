#!/usr/bin/env python3
"""
Mede se um labirinto do ros_maze_worlds serve para o Go2, e onde nascer nele.

Roda no host x86, offline, sem ROS e sem Gazebo: lê o STL direto. Só leitura.

    python3 tools/maze/maze_fit.py --models /caminho/ros_maze_worlds/models maze11
    python3 tools/maze/maze_fit.py --models ... maze10 maze11 --scale 0.002

Existe porque a escala e a pose de cada labirinto são números MEDIDOS, e sem
este script eles voltariam a ser chute. Reaproveitar a pose de um labirinto em
outro coloca o robô dentro de uma parede, e o Gazebo não reclama disso.

O QUE ELE MEDE, E POR QUE CADA COISA IMPORTA

- **Largura de corredor.** O tronco do Go2 é 0.70 x 0.31 m, raio circunscrito
  0.383 m, logo precisa de 0.77 m para passar girando. Na escala 0.001 do
  upstream NENHUMA célula dos labirintos cabe o robô.
- **Altura da parede.** O lidar L1 assenta a ~0.306 m do chão. Parede na altura
  exata do plano de varredura entra e sai do scan conforme o tronco oscila, e
  isso parece defeito de bridge, não geometria.
- **Área navegável em UM componente conectado.** É o número que decide se existe
  patrulha possível: dois bolsões grandes separados por um corredor estreito
  somam área e não servem para nada.
- **Pose de nascimento.** Escolhida onde há a maior corrida livre em +x (a
  primeira coisa que o robô faz é andar para frente) COM folga de centro de
  corredor. Maximizar só a corrida encosta o robô num canto.

COMO A PEGADA DAS PAREDES É OBTIDA

Os labirintos são caixas de parede extrudadas. As faces horizontais do STL são
o topo e a base dessas caixas, logo a projeção delas em XY é exatamente a
pegada. Rasterizar só as faces horizontais evita ter de fechar sólido.

A escala default é 0.002, o dobro do upstream, pelas duas medidas acima. Ver
o cabeçalho de `demo_simulation/worlds/quadruped_maze.sdf` e
`docs/results/ml35-labirinto.md`.
"""

import argparse
import math
from pathlib import Path
import struct
import sys

import numpy as np
from scipy import ndimage


# Raio circunscrito do tronco do Go2 (0.70 x 0.31 m). O robô só passa por
# vãos maiores que o dobro disto.
TRUNK_RADIUS_M = 0.383

# Folga mínima para considerar uma célula "centro de corredor". Um corredor de
# 1.20 m tem meia-largura 0.60; 0.55 aceita o centro e rejeita quem está
# encostado numa parede.
CORRIDOR_CENTRE_CLEARANCE_M = 0.55

# Resolução da rasterização. 0.01 m resolve a parede de 0.40 m de espessura e
# a margem de 0.217 m sem custo relevante nestas grades.
DEFAULT_RESOLUTION_M = 0.01


def load_triangles(path: Path) -> np.ndarray:
    """Lê um STL binário ou ASCII e devolve (n, 3, 3) de vértices."""
    data = path.read_bytes()
    if data[:5].lower() == b'solid' and b'facet' in data[:2000]:
        vertices = [
            [float(x) for x in parts[1:4]]
            for parts in (line.split() for line in
                          data.decode('utf-8', 'ignore').splitlines())
            if parts[:1] == ['vertex']
        ]
        return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)

    count = struct.unpack('<I', data[80:84])[0]
    out = np.zeros((count, 3, 3))
    for i in range(count):
        base = 84 + i * 50 + 12
        for j in range(3):
            out[i, j] = struct.unpack('<3f', data[base + j * 12:base + j * 12 + 12])
    return out


def wall_footprint(triangles: np.ndarray, resolution: float):
    """Rasteriza as faces horizontais em XY. Devolve (livre, gx, gy)."""
    flat = triangles.reshape(-1, 3)
    low, high = flat.min(axis=0), flat.max(axis=0)
    nx = int(np.ceil((high[0] - low[0]) / resolution)) + 1
    ny = int(np.ceil((high[1] - low[1]) / resolution)) + 1
    gx = low[0] + (np.arange(nx) + 0.5) * resolution
    gy = low[1] + (np.arange(ny) + 0.5) * resolution
    mesh_x, mesh_y = np.meshgrid(gx, gy)

    occupied = np.zeros((ny, nx), dtype=bool)
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        if abs(normal[2]) < 1e-9:
            continue  # face vertical: é o lado da parede, não a pegada
        a, b, c = triangle[:, 0:2]
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(denominator) < 1e-12:
            continue
        bary_a = ((b[1] - c[1]) * (mesh_x - c[0])
                  + (c[0] - b[0]) * (mesh_y - c[1])) / denominator
        bary_b = ((c[1] - a[1]) * (mesh_x - c[0])
                  + (a[0] - c[0]) * (mesh_y - c[1])) / denominator
        inside = (bary_a >= -1e-9) & (bary_b >= -1e-9) & (1 - bary_a - bary_b >= -1e-9)
        occupied |= inside

    return ~occupied, gx, gy, low, high


def corridor_widths(free: np.ndarray, resolution: float, span: float) -> np.ndarray:
    """Vãos contíguos de espaço livre, nas duas direções da grade."""
    runs = []
    for grid in (free, free.T):
        for row in grid:
            length = 0
            for cell in row:
                if cell:
                    length += 1
                elif length:
                    runs.append(length * resolution)
                    length = 0
            if length:
                runs.append(length * resolution)
    widths = np.asarray(runs)
    # Descarta ruído sub-célula e o vão externo ao labirinto, que não é corredor.
    return widths[(widths > 0.05) & (widths < span * 0.5)]


def free_run_east(component: np.ndarray, resolution: float) -> np.ndarray:
    """Corrida livre contígua em +x a partir de cada célula da componente."""
    run = np.zeros(component.shape, dtype=float)
    for row in range(component.shape[0]):
        length = 0.0
        for col in range(component.shape[1] - 1, -1, -1):
            length = length + resolution if component[row, col] else 0.0
            run[row, col] = length
    return run


# Cantos da caixa envolvente, como (sinal em x, sinal em y). "se" e o canto
# inferior direito visto de cima com x para a direita e y para cima.
CORNERS = {'se': (+1, -1), 'ne': (+1, +1), 'nw': (-1, +1), 'sw': (-1, -1)}

# Direcoes de saida, com o yaw de spawn correspondente em graus.
HEADINGS = (('+x', 0, 1, 0), ('+y', 90, 0, 1),
            ('-x', 180, -1, 0), ('-y', -90, 0, -1))


def run_from(component, row, col, step_row, step_col, resolution) -> float:
    """Pista livre contigua a partir de (row, col) na direcao dada."""
    length = 0.0
    r, c = row + step_row, col + step_col
    while (0 <= r < component.shape[0] and 0 <= c < component.shape[1]
            and component[r, c]):
        length += resolution
        r += step_row
        c += step_col
    return length


def exits(component, row, col, resolution) -> list:
    """
    Pista livre nas quatro direcoes, ordenada da maior para a menor.

    Existe porque a pose sozinha nao basta: o robo nasce com yaw 0, olhando
    para +x, e a primeira coisa que ele faz e andar para frente. Num canto do
    labirinto +x costuma ser parede -- e girar parado e justamente o que este
    robo faz pior (teto de guinada de 0.13 rad/s). Nascer virado para a saida
    e de graca: `quadruped.launch.py` aceita `yaw:=`.
    """
    measured = [
        (name, yaw, run_from(component, row, col, dy, dx, resolution))
        for name, yaw, dx, dy in HEADINGS
    ]
    return sorted(measured, key=lambda item: -item[2])


def choose_start(centred, run, gx, gy, low, high, mode: str):
    """
    Escolhe a celula de nascimento: maior corrida em +x, ou um canto.

    `run` maximiza pista livre para frente, que e bom para um ensaio de marcha
    reta. Um canto e o que se quer numa demonstracao: o robo comeca numa ponta
    e atravessa o labirinto inteiro, em vez de nascer no meio dele.

    Em qualquer dos dois a celula sai de `centred`, ou seja folga de centro de
    corredor. Sem isso o robo nasce encostado numa parede e o primeiro passo
    ja raspa.
    """
    if mode == 'run':
        return np.unravel_index(np.argmax(run * centred), run.shape)

    sign_x, sign_y = CORNERS[mode]
    target_x = high[0] if sign_x > 0 else low[0]
    target_y = high[1] if sign_y > 0 else low[1]
    rows, cols = np.nonzero(centred)
    if rows.size == 0:
        return np.unravel_index(np.argmax(run * centred), run.shape)
    reach = np.hypot(gx[cols] - target_x, gy[rows] - target_y)
    nearest = int(np.argmin(reach))
    return rows[nearest], cols[nearest]


def analyse(name: str, models: Path, scale: float, resolution: float,
            start: str = 'run') -> dict:
    """Mede um labirinto e devolve o veredito mais a pose recomendada."""
    stl = models / name / 'meshes' / f'{name}.stl'
    if not stl.is_file():
        raise FileNotFoundError(stl)

    triangles = load_triangles(stl) * scale
    free, gx, gy, low, high = wall_footprint(triangles, resolution)
    span_x, span_y = high[0] - low[0], high[1] - low[1]

    widths = corridor_widths(free, resolution, span_x)
    clearance = ndimage.distance_transform_edt(free) * resolution

    fits = free & (clearance >= TRUNK_RADIUS_M)
    labels, count = ndimage.label(fits)
    if count == 0:
        return {
            'name': name, 'scale': scale, 'span': (span_x, span_y),
            'wall_height': high[2] - low[2], 'widths': widths,
            'components': 0, 'area': 0.0, 'pose': None,
        }
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, count + 1))
    component = labels == 1 + int(np.argmax(sizes))

    run = free_run_east(component, resolution)
    centred = component & (clearance >= CORRIDOR_CENTRE_CLEARANCE_M)
    row, col = choose_start(centred, run, gx, gy, low, high, start)

    rows, cols = np.nonzero(component)
    return {
        'name': name, 'scale': scale, 'span': (span_x, span_y), 'start': start,
        'wall_height': high[2] - low[2], 'widths': widths,
        'components': count, 'area': component.sum() * resolution ** 2,
        'pose': (-gx[col], -gy[row]),
        'run': run[row, col], 'clearance': clearance[row, col],
        'extent_x': (gx[cols.min()] - gx[col], gx[cols.max()] - gx[col]),
        'extent_y': (gy[rows.min()] - gy[row], gy[rows.max()] - gy[row]),
        'exits': exits(component, row, col, resolution),
        '_grid': (component, centred, clearance, gx, gy, row, col, resolution),
    }


def pick_goals(result: dict, count: int, max_radius: float,
               min_radius: float = 2.0) -> list:
    """
    Escolhe metas em centro de corredor, espalhadas e dentro do raio.

    Uma meta em cima de uma parede e ACEITA pelo Nav2 e falha depois, perto da
    borda, onde o erro ja nao tem nome -- entao ela e escolhida aqui, sobre a
    mesma grade que decidiu a pose, e nao a olho no RViz.

    `max_radius` existe porque patrol_commander rejeita metas alem de
    MAX_GOAL_RADIUS_M (8.0 m) e o costmap global e janela rolante.
    """
    component, centred, clearance, gx, gy, row0, col0, resolution = result['_grid']
    rows, cols = np.nonzero(centred)
    if rows.size == 0:
        return []

    # Coordenadas no referencial do ROBO: ele nasce na celula (row0, col0).
    points = np.stack([gx[cols] - gx[col0], gy[rows] - gy[row0]], axis=1)
    reach = np.hypot(points[:, 0], points[:, 1])
    # min_radius: uma meta a menos de 2 m da partida nao e travessia, e numa
    # patrulha ela vira uma parada que nao mede nada.
    keep = (reach <= max_radius) & (reach >= min_radius)
    points, scores = points[keep], clearance[rows, cols][keep]
    if points.size == 0:
        return []

    # Guloso por distancia: pega o ponto mais distante dos ja escolhidos, para
    # que a patrulha atravesse o labirinto em vez de circular numa sala.
    chosen = [points[int(np.argmax(reach[keep]))]]
    while len(chosen) < count:
        spread = np.min(
            [np.hypot(points[:, 0] - c[0], points[:, 1] - c[1]) for c in chosen],
            axis=0)
        candidate = int(np.argmax(spread + 0.05 * scores))
        if spread[candidate] < 1.5:
            break
        chosen.append(points[candidate])
    return [(float(x), float(y)) for x, y in chosen]


def report(result: dict, goals: int = 0, max_radius: float = 8.0,
           min_radius: float = 2.0) -> bool:
    """Imprime o laudo e devolve True se o labirinto serve."""
    name, scale = result['name'], result['scale']
    span_x, span_y = result['span']
    widths = result['widths']
    needed = 2 * TRUNK_RADIUS_M

    print(f'== {name} @ escala {scale}   partida "{result.get("start", "run")}"')
    print(f'   pegada                  {span_x:.2f} x {span_y:.2f} m')
    print(f'   altura da parede        {result["wall_height"]:.2f} m'
          '   (lidar L1 a ~0.306 m)')
    if widths.size:
        median = float(np.median(widths))
        print(f'   corredor mediano        {median:.2f} m'
              f'   (precisa de {needed:.2f} m)')
        print(f'   margem por lado         {(median - needed) / 2 * 100:.1f} cm')
    print(f'   area navegavel          {result["area"]:.1f} m2'
          f'   em {result["components"]} componente(s)')

    if result['pose'] is None:
        print('   VEREDITO: NAO SERVE -- nenhuma celula cabe o robo nesta escala')
        return False

    pose_x, pose_y = result['pose']
    print(f'   folga no nascimento     {result["clearance"]:.2f} m')
    print('   pista livre por direcao ' + '  '.join(
        f'{name}={length:.2f}m' for name, _, length in result['exits']))
    best_name, best_yaw, best_run = result['exits'][0]
    print(f'   >>> yaw:={math.radians(best_yaw):.4f}'
          f'   ({best_yaw:+d} deg, saida {best_name}, {best_run:.2f} m livres)')
    print(f'   extensao (coords robo)  x[{result["extent_x"][0]:+.2f}, '
          f'{result["extent_x"][1]:+.2f}]  '
          f'y[{result["extent_y"][0]:+.2f}, {result["extent_y"][1]:+.2f}]')
    print(f'   >>> <pose>{pose_x:.3f} {pose_y:.3f} 0 0 0 0</pose>')

    # A parede tem de sair do plano de varredura, não ficar nele.
    ok = result['wall_height'] > 0.40 and (
        not widths.size or float(np.median(widths)) >= needed)
    print(f'   VEREDITO: {"SERVE" if ok else "NAO SERVE"}')

    if goals and '_grid' in result:
        picked = pick_goals(result, goals, max_radius, min_radius)
        print(f'   metas em centro de corredor, coords do robo, raio <= '
              f'{max_radius:.1f} m:')
        for index, (x, y) in enumerate(picked):
            print(f'      {index}: x={x:+.2f} y={y:+.2f}'
                  f'   ({np.hypot(x, y):.2f} m da origem)')
        if len(picked) < goals:
            print(f'      (so {len(picked)} de {goals} pedidas cabem espalhadas '
                  'dentro do raio)')
        flat = ', '.join(f'{x:.2f}, {y:.2f}, 0.0' for x, y in picked)
        print(f'   waypoints:="[{flat}]"')

    return ok


def main() -> int:
    """Mede um ou mais labirintos e devolve 1 se algum não servir."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mazes', nargs='+', help='nomes, ex. maze10 maze11')
    parser.add_argument('--models', type=Path,
                        default=Path('/tmp/ros_maze_worlds/models'),
                        help='diretorio models/ do clone do ros_maze_worlds')
    parser.add_argument('--scale', type=float, default=0.002,
                        help='escala uniforme aplicada ao STL (default 0.002)')
    parser.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION_M,
                        help='resolucao da rasterizacao em m')
    parser.add_argument('--start', default='run',
                        choices=['run'] + sorted(CORNERS),
                        help='onde o robo nasce: "run" = maior pista livre em '
                             '+x; "se"/"ne"/"nw"/"sw" = canto do labirinto '
                             '(se = inferior direito)')
    parser.add_argument('--goals', type=int, default=0, metavar='N',
                        help='tambem sugere N metas em centro de corredor, '
                             'prontas para waypoints:= do patrol_commander')
    parser.add_argument('--min-goal-radius', type=float, default=2.0,
                        help='metas mais perto que isto da partida sao '
                             'descartadas: nao sao travessia')
    parser.add_argument('--max-goal-radius', type=float, default=8.0,
                        help='raio maximo das metas sugeridas; casa com '
                             'MAX_GOAL_RADIUS_M do patrol_commander')
    args = parser.parse_args()

    if not args.models.is_dir():
        print(f'--models={args.models} nao e um diretorio. Clone o '
              'ros_maze_worlds e aponte para o subdiretorio models/.',
              file=sys.stderr)
        return 2

    failures = 0
    for maze in args.mazes:
        try:
            measured = analyse(maze, args.models, args.scale, args.resolution,
                               args.start)
            if not report(measured, args.goals, args.max_goal_radius,
                          args.min_goal_radius):
                failures += 1
        except FileNotFoundError as missing:
            print(f'{maze}: STL nao encontrado em {missing}', file=sys.stderr)
            failures += 1
        print()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
