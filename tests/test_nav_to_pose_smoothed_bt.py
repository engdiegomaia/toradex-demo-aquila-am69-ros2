"""
Trava a colisão de blackboard que produz `[follow_path] Aborting handle` a
~1 Hz durante toda a navegação (ver docs/ml35/proximos-passos-navegacao.md).

`SmoothPath` e `FollowPath` vivem como irmãos dentro do mesmo
`PipelineSequence`. Enquanto `FollowPath` retorna RUNNING, o próprio
`PipelineSequence` re-tica os irmãos anteriores -- inclusive o
`RateController` que recomputa e suaviza o caminho. Se `SmoothPath` escreve o
resultado na MESMA chave que `FollowPath` lê (`{path}`), toda re-tick parece
uma meta nova para o `FollowPath`, que aborta o handle em andamento e
recomeça. Confirmado por um mantenedor do Nav2 em
ros-navigation/navigation2#5817: "we expect users to remap the smoothed path
to a different blackboard variable".

Este teste não precisa de ROS nem de simulador: é uma checagem estrutural do
XML.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

BT_FILE = (
    Path(__file__).resolve().parents[1]
    / 'ros2_ws' / 'src' / 'demo_navigation' / 'behavior_trees'
    / 'nav_to_pose_smoothed.xml'
)


def _find_one(root: ET.Element, tag: str) -> ET.Element:
    matches = root.findall(f'.//{tag}')
    assert len(matches) == 1, f'esperava exatamente um <{tag}>, achei {len(matches)}'
    return matches[0]


def test_smooth_path_nao_sobrescreve_a_propria_entrada():
    """
    `unsmoothed_path` e `smoothed_path` no MESMO nó precisam ser chaves
    diferentes. Reusar a mesma chave é exatamente o padrão reproduzido em
    navigation2#5817.
    """
    root = ET.parse(BT_FILE).getroot()
    smooth = _find_one(root, 'SmoothPath')
    assert smooth.get('unsmoothed_path') != smooth.get('smoothed_path'), (
        "SmoothPath reusa a mesma chave de entrada e saida -- isso e o "
        "padrao que causa 'Aborting handle' a ~1 Hz (navigation2#5817)")


def test_follow_path_consome_o_caminho_suavizado():
    """
    O ponto do `SmoothPath` é alimentar o `FollowPath` com o caminho já
    suavizado. Se as chaves não combinarem, `FollowPath` volta a seguir o
    caminho em escada do NavFn, o defeito que `nav_to_pose_smoothed.xml`
    existe para evitar.
    """
    root = ET.parse(BT_FILE).getroot()
    smooth = _find_one(root, 'SmoothPath')
    follow = _find_one(root, 'FollowPath')
    assert follow.get('path') == smooth.get('smoothed_path')
