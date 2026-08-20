# S6 — Labirinto interativo com metas por clique

Mundo: `quadruped_maze.sdf` · Estação x86 · Nav2 + RViz2

Este é o fluxo para dirigir o Go2 por metas, sem publicar velocidade manual.
O Gazebo roda no container e o Nav2/RViz no host x86. O `GoalTool` do RViz
transforma cada clique-e-arraste em uma ação `NavigateToPose`; o robô planeja,
desvia pelo costmap e entra em trote sozinho.

## 1. Preparar os modelos do labirinto (uma vez)

O STL do labirinto é uma dependência externa e não está versionado no projeto.
Baixe-a fora do repositório:

```bash
git clone --depth 1 https://github.com/cafemesa/ros_maze_worlds.git \
  /tmp/ros_maze_worlds
```

Se o diretório já existir, atualize-o com `git -C /tmp/ros_maze_worlds pull
--ff-only`. O script precisa receber o diretório `models`, não a raiz do clone.

## 2. Subir o Gazebo

No primeiro terminal, na raiz do repositório:

```bash
export MAZE_MODELS=/tmp/ros_maze_worlds/models
./scripts/run_quadruped_sim.sh quadruped_maze.sdf
```

Espere o log chegar a `state=fixed stand`. A linha `Loading SDF world file` deve
conter `quadruped_maze.sdf`; sem `MAZE_MODELS` o Gazebo ainda abre, mas a malha
não resolve e o robô fica em campo aberto.

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
