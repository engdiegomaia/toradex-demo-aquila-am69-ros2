"""
Cenarios do demo: mundo, pose de nascimento e enquadramento das cameras.

POR QUE ESTA TABELA EXISTE

Antes dela os nove numeros que descrevem um cenario viviam em tres lugares: o
`world` no launch, o `yaw` de nascimento em outro launch, e o enquadramento das
duas cameras de cena num terceiro -- com uma QUARTA copia num comentario de
`docker/compose.host.yml`, na forma de um SIM_ARGS de seis linhas que o operador
tinha de colar a mao.

Isso produz uma falha silenciosa especifica, e ela ja aconteceu: trocar de mundo
sem trocar o enquadramento. O labirinto tem area util centrada em (-4,855;
4,855) e o armazem na origem, entao o mundo do labirinto com a camera do armazem
mostra chao vazio ao lado do labirinto. Nada erra, nada loga: o painel azul do
cockpit simplesmente aponta para o lugar errado, e quem olha conclui que o robo
nao esta se movendo.

A correcao e de estrutura, nao de disciplina: o enquadramento passa a ser
DERIVADO do mundo. Nao ha mais como escolher um sem o outro, porque nao ha mais
dois lugares para escolher.

COMO SE SOBREPOE UM VALOR

Todo argumento `scene_*` e `yaw` dos launches aceita vazio (o default), que
significa "pergunte a esta tabela". Qualquer valor nao vazio vence. Entao
sondar um canto do labirinto continua sendo `scene_top_z:=20.0` na linha de
comando, sem editar arquivo nenhum, e sem perder os outros oito numeros.

DE ONDE VEM CADA NUMERO

Nenhum foi escolhido: todos foram medidos ou calculados, e as contas estao em
`launch/scene_cameras.launch.py`. Resumo da origem:

- armazem: iso em (-3, +3) porque os quadrantes (-x,-y) caem dentro do corredor
  de prateleiras; topo a 6 m porque a 12 m a camera fica ACIMA das vigas do
  telhado e a imagem inteira vira uma viga laranja;
- maze11: centro e extensao vieram da bbox do STL lida do binario e multiplicada
  pela escala 0,002 (ver `scripts/maze_fit.py`), nao do nome do arquivo.

O maze11 e o cenario OFICIAL do quadrupede. O armazem continua sendo o do
diff-drive, que e o fallback e foi validado la -- ver `robot_selection.py`.
"""

import os

# Enquadramento generico: as duas cameras olhando para a origem. Serve para os
# mundos pequenos e centrados na origem (empty, corridor, objects, ramp, rough),
# em que a area util E a origem. NAO serve para o maze11, e e exatamente por
# isso que ele tem entrada propria.
GENERIC = {
    'spawn': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    # (x, y, z, pitch, yaw)
    'scene_iso': (-3.0, 3.0, 2.4, 0.5150, -0.7854),
    'scene_top': (0.0, 0.0, 6.0, 1.5708, 1.5708),
}

SCENARIOS = {
    # Cenario OFICIAL do quadrupede. Corredor de 1,20 m, parede de 0,40 m,
    # pegada 11,60 x 11,60 m centrada em (-4,855; 4,855).
    #
    # yaw 1,5708 nasce o robo olhando PARA O CORREDOR e nao para a parede. Sem
    # isso a primeira coisa que o Nav2 tem de fazer e um giro de 90 graus dentro
    # de um corredor de 1,20 m, o que gasta os primeiros segundos de qualquer
    # ensaio e polui a comparacao entre condicoes.
    #
    # O labirinto nao tem teto, entao a vista de topo pode subir: 13 m cobrem
    # 17,8 x 13,3 m, os 11,6 m com margem. A iso vem de fora e de baixo
    # (-13, -3, 9) para nao olhar de dentro de um corredor.
    'quadruped_maze11.sdf': {
        'spawn': {'x': 0.0, 'y': 0.0, 'yaw': 1.5708},
        'scene_iso': (-13.0, -3.0, 9.0, 0.6717, 0.7676),
        'scene_top': (-4.855, 4.855, 13.0, 1.5708, 1.5708),
        # Malha EXTERNA ao repositorio -- ver `external_models` abaixo.
        'needs_models': ('maze11',),
    },
    # Cenario do diff-drive, que e o fallback validado. O warehouse tem cerca de
    # 28 x 45 m e enquadra-lo por completo exigiria h ~ 44 m, altura em que o
    # robo vira um punhado de pixels; o enquadramento generico e proposital.
    'warehouse.sdf': GENERIC,
}

# Onde o compose monta os modelos externos dentro do container `sim`.
# Espelha o volume de docker/compose.host.yml; se um dos dois mudar, a
# verificacao de `missing_models` passa a nao achar nada e volta a ser silenciosa.
EXTERNAL_MODELS_DIR = '/maze/models'


def scenario(world: str) -> dict:
    """
    Devolve o cenario de um mundo, pelo nome do ARQUIVO.

    Pelo nome do arquivo e nao pelo nome do elemento `<world>`: aqui a entrada e
    um caminho de launch, e ler o SDF para descobrir o nome interno custaria uma
    leitura de arquivo em tempo de launch para resolver o que o proprio caminho
    ja diz. (Onde a distincao importa de verdade -- servico do Gazebo -- e em
    sim_control.launch.py, que documenta o caso.)

    Mundo sem entrada cai no generico. Isso e deliberado: um mundo novo funciona
    sem tocar nesta tabela, e so precisa de entrada quando a area util NAO esta
    na origem.
    """
    return SCENARIOS.get(os.path.basename(world), GENERIC)


def camera_pose(world: str, camera: str) -> tuple:
    """Pose (x, y, z, pitch, yaw) de `scene_iso` ou `scene_top`."""
    key = f'scene_{camera}'
    if key not in ('scene_iso', 'scene_top'):
        raise ValueError(f'camera {camera!r} desconhecida: use iso ou top.')
    return scenario(world)[key]


def spawn_pose(world: str) -> dict:
    """Pose de nascimento do robo: x, y, yaw."""
    return scenario(world)['spawn']


def external_models(world: str) -> tuple:
    """Modelos que o mundo carrega e que NAO estao no repositorio."""
    return tuple(scenario(world).get('needs_models', ()))


def missing_models(world: str, root: str = EXTERNAL_MODELS_DIR) -> tuple:
    """
    Quais modelos externos o mundo pede e nao estao montados.

    Existe porque a falha e TOTALMENTE silenciosa. O SDF do maze11 referencia
    `model://maze11/meshes/maze11.stl`; sem a malha o Gazebo carrega o mundo, o
    modelo fica sem visual e sem colisao, e o resultado e um plano vazio. O lidar
    nao ve nada, o Nav2 planeja em linha reta e conclui com SUCCEEDED. Ou seja: o
    ensaio PASSA, com numeros melhores que os reais, e nada na saida diz que o
    labirinto nao estava la.

    E pior ainda no caminho default do compose: `MAZE_MODELS:-./models-extra`
    aponta para um diretorio que nao existe no repositorio, e o Docker cria um
    diretorio VAZIO em vez de falhar.
    """
    return tuple(
        name for name in external_models(world)
        if not os.path.isdir(os.path.join(root, name))
    )
