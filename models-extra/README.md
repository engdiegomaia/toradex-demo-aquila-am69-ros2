# Modelos externos ao repositório

Este diretório existe **vazio de propósito**. Ele é o ponto de montagem default
de `MAZE_MODELS` em `docker/compose.host.yml`:

```yaml
- ${MAZE_MODELS:-./models-extra}:/maze/models:ro
```

Compose não tem montagem condicional. Se a variável ficasse sem default, o
Compose criaria um diretório com o nome literal da variável; se o default
apontasse para um caminho inexistente, o Docker criaria um diretório vazio de
qualquer forma. Ter o diretório aqui, versionado e documentado, torna o caso
default explícito em vez de acidental.

## O que deveria estar montado aqui

O cenário **oficial** da demo é o labirinto (`quadruped_maze11.sdf`), e a malha
dele **não está neste repositório**:

```
maze11/
├── model.config
├── model.sdf
└── meshes/
    └── maze11.stl
```

Vem de [`cafemesa/ros_maze_worlds`](https://github.com/cafemesa/ros_maze_worlds).

```bash
git clone https://github.com/cafemesa/ros_maze_worlds.git ~/ros_maze_worlds
export MAZE_MODELS=~/ros_maze_worlds/models
```

Aponte para um caminho **estável**, não para `/tmp`: um reboot no meio da semana
apaga o labirinto e o sintoma é o do parágrafo seguinte.

## Por que a malha não é vendorizada

`package.xml` do upstream declara `<license>TODO</license>` e não há arquivo de
licença no repositório. É o **mesmo bloqueio** que fez este projeto trocar o
robô A1 pelo Go2 (ver `docs/ml35/estado-fases.md`, "O bloqueador:
`a1_description` declara `<license>TODO</license>`"). A base legal usada para
vendorizar o `go2_description` foi o rastreamento até `unitreerobotics/unitree_ros`
(BSD-3) com malhas provadas bit-idênticas por hash — não existe equivalente aqui.

Sem licença, sem vendorização. Não é cautela excessiva: é a regra que o projeto
já aplicou uma vez a um custo maior.

## A falha silenciosa que isso cria, e a guarda

Malha ausente é **WARNING** no Gazebo, nunca erro. O mundo carrega, o modelo do
labirinto fica sem visual e sem colisão, e o resultado é um plano vazio:

- o lidar não vê parede nenhuma;
- o Nav2 planeja em linha reta;
- a meta termina `SUCCEEDED`, **mais rápido que o real**;
- nada na saída diz que o labirinto não estava lá.

Ou seja, o ensaio *passa* com números melhores que a verdade — o pior modo de
falha possível para uma medição.

Por isso `quadruped.launch.py` verifica antes de subir qualquer nó
(`_check_external_models`, alimentado por
`demo_simulation/scenarios.py::missing_models`) e **aborta** nomeando
`MAZE_MODELS`. Se você vê o labirinto ausente sem essa mensagem, a guarda
regrediu — há um teste estrutural em `tests/` para ela.
