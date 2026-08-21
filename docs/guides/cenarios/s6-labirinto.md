# S6 — Labirinto interativo com metas por clique

Mundo: `quadruped_maze11.sdf` · Estação x86 · Nav2 + RViz2

Este é o fluxo para dirigir o Go2 por metas, sem publicar velocidade manual.
O Gazebo roda no container e o Nav2/RViz no host x86. O `GoalTool` do RViz
transforma cada clique-e-arraste em uma ação `NavigateToPose`; o robô planeja,
desvia pelo costmap e entra em trote sozinho.

## 1. Preparar os modelos do labirinto (uma vez)

O STL do labirinto é uma dependência externa e não está versionado no projeto —
o `package.xml` do upstream declara `<license>TODO</license>` e não há arquivo
LICENSE. Baixe fora do repositório, **num caminho estável**:

```bash
git clone --depth 1 https://github.com/cafemesa/ros_maze_worlds.git \
  ~/ros_maze_worlds
```

Se o diretório já existir, atualize-o com `git -C ~/ros_maze_worlds pull
--ff-only`. O script precisa receber o diretório `models`, não a raiz do clone.

**Não use `/tmp`.** Um reboot no meio de uma demonstração apaga o clone, e o
sintoma não é um erro: o Gazebo sobe, o labirinto simplesmente não aparece.

Confirme que o labirinto serve para este robô antes de rodar:

```bash
python3 scripts/maze_fit.py --models ~/ros_maze_worlds/models maze11
```

Espere `VEREDITO: SERVE`. O script também imprime a `<pose>` recomendada — é de
onde saiu a que está no SDF. Método e números em
[`docs/results/ml35-labirinto.md`](../../results/ml35-labirinto.md).

## 2. Subir o Gazebo

No primeiro terminal, na raiz do repositório:

```bash
export MAZE_MODELS=~/ros_maze_worlds/models
./scripts/run_quadruped_sim.sh quadruped_maze11.sdf
```

O script imprime duas linhas de confirmação antes de subir:

```
Modelos externos: /home/você/ros_maze_worlds/models -> /maze/models
Yaw de nascimento: 1.5708 rad
```

Espere o log chegar a `state=fixed stand`, e o mundo aparecer como
`World [quadruped_maze11] initialized`.

**A partida é o canto inferior direito do labirinto**, para que a demonstração
comece numa ponta e atravesse tudo. Nesse canto `+x` é parede a 16 cm, e o robô
nasce olhando para `+x` — então o mundo declara o próprio yaw de nascimento numa
linha `<!-- go2_spawn_yaw: 1.5708 -->` e o script a lê. **Não há nada a passar na
linha de comando**; `GO2_SPAWN_YAW=<rad>` sobrepõe se você quiser experimentar
outro rumo. Confirme no `scenario_check` que a pose sai com `yaw=90.0 deg`.

Sem `MAZE_MODELS` o `run_quadruped_sim.sh` **recusa** qualquer mundo
`quadruped_maze*.sdf` e explica por quê. Esse guard existe porque a falha que
ele evita é silenciosa: malha que não resolve é uma linha de aviso no meio do
log, não um erro fatal, e o robô então anda em campo aberto — uma corrida que
parece um desvio perfeito.

## 3. Abrir Nav2 e RViz interativo

No segundo terminal, no host x86:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select demo_navigation demo_bringup
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
export ROS_LOG_DIR=/tmp
ros2 launch demo_bringup maze_nav_rviz.launch.py
```

O portão de prontidão é `Managed nodes are active`. Não inicie
`demo_routine`, `patrol_commander` ou outro publicador de `/demo/cmd_vel` junto
com esse launch.

## 4. Mandar o robô para um local

No RViz:

1. Use a vista superior (`TopDownOrtho`) e deixe o `Fixed Frame` em `map`.
2. Selecione a ferramenta **Nav2 Goal** (ícone de alvo/seta).
3. Clique no destino e arraste na direção do heading final; solte para enviar.
4. Clique e arraste outra meta para cancelar/substituir a atual.

As metas devem ficar dentro dos corredores e do alcance do lidar. O costmap é
rolante e não existe mapa estático: clicar numa parede ou numa área ainda não
observada pode fazer o planejador rejeitar a meta.

### Onde as metas cabem no `maze11`

O componente navegável, em coordenadas do robô (que nasce em 0,0, no canto
inferior direito — logo o labirinto todo fica em `−x` e `+y`):

```
x[-9,87 ... +0,16]     y[-0,94 ... +9,87]
```

Metas medidas em centro de corredor, dentro dos 8 m que o `patrol_commander`
aceita — são também as default do `nav_trial.py`:

```
(0.00, 8.00)   (-8.00, 0.00)   (-1.60, 1.60)   (-5.83, 4.91)
```

Duas restrições que não se anunciam:

- **Corredor de 1,20 m com margem de 21,7 cm por lado.** Com `robot_radius` de
  0,38 m e `inflation_radius` de 0,55 m, quase todo o corredor carrega custo; a
  faixa sem custo é a linha de centro. O planejador funciona nisso, mas metas
  colocadas junto à parede são as que ele rejeita.
- **`patrol_commander` rejeita metas além de 8 m** (`MAX_GOAL_RADIUS_M`). A ponta
  `y = −10,38` está fora desse raio. Pelo RViz não há esse limite, mas o
  costmap global é janela rolante: meta muito longe é *aceita* e depois falha
  perto da borda.

### Trocar de labirinto

Os 11 labirintos do upstream não são intercambiáveis: cada um tem a sua pose, e
**reaproveitar a pose de outro coloca o robô dentro de uma parede** — sem erro
do Gazebo.

| labirinto | escala | `<pose>` | yaw | área navegável |
| --- | --- | --- | --- | --- |
| `maze10` (`quadruped_maze.sdf`) | 0,002 | `-2.670 3.061 0 0 0 0` | 0 | 18,4 m² |
| **`maze11`** (`quadruped_maze11.sdf`) | 0,002 | `-11.672 11.649 0 0 0 0` | 1,5708 | 35,4 m² |

Para um labirinto novo, meça em vez de estimar:

```bash
python3 scripts/maze_fit.py --models ~/ros_maze_worlds/models maze7 \
  --start se --goals 4
```

O script imprime a `<pose>`, o `yaw:=` recomendado (medindo a pista livre nas
quatro direções na célula de partida) e metas em centro de corredor. `--start`
aceita `run` (maior pista em `+x`, para ensaio de marcha reta) ou um canto:
`se`, `ne`, `nw`, `sw`.

Os STL de 1 a 7 estão com **Y para cima** e precisariam de `roll 1.5708`; 8 a 11
já estão com Z para cima e usam `rpy 0 0 0`. Os mundos deste projeto referenciam
a malha direto justamente por isso — um `<include>model://mazeN` deita o
labirinto de lado, porque o `model.sdf` do upstream declara roll para todos.

## 5. Displays já abertos

O arquivo `demo_view.rviz`, carregado automaticamente, deixa habilitados:

- **TF:** `map`, `odom`, `base`, `trunk`, `lidar`, `front_camera` e `imu_link`;
- **LaserScan:** `/demo/scan`;
- **Lidar PointCloud:** `/demo/scan_cloud` (a nuvem 3D usada pelo costmap);
- **Go2Camera:** `/demo/camera/image_raw`.

Para conferir pelo terminal, o contrato pode ser medido sem publicar comando:

```bash
cd ..
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69 ROS_LOG_DIR=/tmp
python3 scripts/scenario_check.py --seconds 10
```

O resultado esperado é `PASSOU: 0 problema(s)`, câmera em aproximadamente 10 Hz,
lidar em 10 Hz e TF com raiz `map`.

## 6. Encerrar

Feche o RViz/Nav2 com `Ctrl-C` e depois o terminal do Gazebo com `Ctrl-C`. Não
inicie outra simulação até o container `aquila-go2` desaparecer de `docker ps`.

Essa execução usa `/demo/odom` do Gazebo como ground truth para fechar
`odom → base`; ela valida planejamento e percepção na simulação, não localização
ou o estimador de estado do hardware real.
