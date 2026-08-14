# Guia de operação e edição

Como rodar a demo e como mexer nela sem quebrar o que já funciona.

Escrito para quem chega no projeto sem experiência prévia de ROS 2. Onde uma
armadilha conhecida existe, ela está descrita no ponto em que você a
encontraria — não numa seção de troubleshooting no fim.

**Fase atual: L3 concluída.** Tudo aqui roda no host x86, nativo, sem
containers. Containers entram na L4.

---

## Sumário

1. [Antes de começar](#1-antes-de-começar)
2. [Rodar a demo](#2-rodar-a-demo)
3. [Como o projeto se encaixa](#3-como-o-projeto-se-encaixa)
4. [Editar o robô](#4-editar-o-robô)
5. [Editar a navegação](#5-editar-a-navegação)
6. [Editar a percepção](#6-editar-a-percepção)
7. [Editar a simulação](#7-editar-a-simulação)
8. [Diagnóstico](#8-diagnóstico)
9. [As armadilhas que já custaram tempo](#9-as-armadilhas-que-já-custaram-tempo)

---

## 1. Antes de começar

### Onde cada coisa roda

Metade dos erros possíveis neste projeto vem de colocar algo na máquina errada.
A regra não muda quando os containers chegarem:

| Componente | Máquina | Por quê |
| --- | --- | --- |
| Gazebo, RViz2 | **Host x86** | São OGRE 2 / OpenGL desktop. A GPU do AM69 só expõe OpenGL ES 3.2 e Vulkan 1.2. |
| Nav2, percepção, bringup | Aquila (ou host, na L3) | Sem renderização. |
| HMI Chromium | Aquila | GPU acelerada, mas via ES. |

Nunca coloque Gazebo ou RViz2 num serviço que sobe no módulo. Isso não é
preferência de estilo: o módulo não tem o driver para isso.

### Instalação

```bash
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-minimal-tb4-sim \
    ros-jazzy-nav2-minimal-tb4-description \
    ros-jazzy-joint-state-publisher-gui \
    liburdfdom-tools
```

Os dois últimos pacotes `nav2-minimal-tb4-*` não são opcionais para a
aparência: fornecem o mundo do armazém **e** as meshes do robô. Ambos são
Apache-2.0, mantidos pela organização Nav2.

### Compilar

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` faz launch files e YAMLs serem lidos direto de `src/` —
você edita e roda de novo, sem recompilar. **Só vale para arquivos de dados.**
Mudou código Python de um nó, recompile.

> **Nunca commite `build/`, `install/` ou `log/`.** Com `--symlink-install`
> esses diretórios contêm caminhos absolutos da sua máquina. Já aconteceu neste
> repo: quatro symlinks apontando para um diretório que não existia mais
> quebravam o build de quem clonasse. O `.gitignore` cobre isso.

### O `source` que todo mundo esquece

Cada terminal novo precisa de:

```bash
source /opt/ros/jazzy/setup.bash
source ~/toradex/demo/aquila-am69-ros2/ros2_ws/install/setup.bash
```

Sem o segundo, `ros2 launch demo_bringup ...` responde "package not found"
mesmo com o pacote compilado.

---

## 2. Rodar a demo

### Tudo de uma vez

```bash
ros2 launch demo_bringup learn.launch.py
```

Sobe Gazebo, robô, bridge, percepção, Nav2 e RViz2. **Leva ~30 s até o Nav2
ficar pronto** — e isso é de propósito:

```
t=0s    Gazebo começa a carregar o mundo (~10 s de meshes)
t=12s   spawn do robô
t=15s   ros_gz_bridge
t=20s   percepção
t=25s   Nav2 + RViz2
```

Esses timers não são folga preguiçosa; ver [seção 9](#9-as-armadilhas-que-já-custaram-tempo).
Numa máquina mais lenta, aumente **todos juntos**.

Para mandar o robô a um destino: no RViz2, botão **2D Goal Pose**, clique e
arraste no mapa.

### Opções úteis

```bash
# Sem Nav2 — só simulação, para teleoperar
ros2 launch demo_bringup learn.launch.py navigation:=false

# Sem RViz2 (headless, CI)
ros2 launch demo_bringup learn.launch.py rviz:=false

# Outro mundo
ros2 launch demo_bringup learn.launch.py \
    world:=$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim/worlds/depot.sdf
```

### Peças isoladas

Útil para depurar: se algo falha no conjunto, suba uma camada por vez.

```bash
# Só o simulador
ros2 launch demo_simulation simulation.launch.py

# Teleop por teclado (terminal separado, precisa de foco)
ros2 launch demo_simulation teleop.launch.py

# Só o modelo, em RViz — não precisa de Gazebo
ros2 launch demo_description view_robot.launch.py
```

### Gerar um mapa novo

O mapa em `demo_navigation/maps/warehouse.{yaml,pgm}` já está commitado. Para
refazer, ou para mapear outro mundo:

```bash
# Terminal 1
ros2 launch demo_simulation simulation.launch.py
# Terminal 2
ros2 launch demo_navigation slam.launch.py
# Terminal 3 — dirija o robô pelo cenário inteiro
ros2 launch demo_simulation teleop.launch.py
# Terminal 4 — quando o mapa estiver completo
ros2 run nav2_map_server map_saver_cli -f meu_mapa
```

Depois copie os dois arquivos para `demo_navigation/maps/` e passe
`map:=.../meu_mapa.yaml`.

---

## 3. Como o projeto se encaixa

```
demo_description   o robô: geometria, sensores, plugins Gazebo
demo_simulation    Gazebo + spawn + ponte ROS↔gz + teleop
demo_perception    stub de detecção + adaptador para o costmap
demo_navigation    Nav2, parâmetros, SLAM, mapas
demo_bringup       composição: quem sobe, em que ordem
demo_tutorials     exercícios da L1, não faz parte da demo
```

### O contrato de tópicos

Isto é o invariante do projeto. Existe para que a inferência na NPU entre
depois sem refatorar nada:

| Tópico | Tipo | Produtor | Consumidores |
| --- | --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | Gazebo, câmera USB ou rosbag | `demo_perception` |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | `demo_perception` | costmap, HMI |
| `/demo/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Gazebo ou driver real |

Duas consequências práticas ao editar:

- **A percepção nunca pode saber de onde vem a imagem.** Se você escrever no
  código dela algo que assume Gazebo, trocar o stub por TIDL deixa de ser uma
  troca de container e vira refatoração.
- **As detecções alimentam o costmap, não só a tela.** Mesmo sendo stub. Se
  essa costura quebrar, ninguém percebe até o dia em que a inferência real
  chegar.

### Quem publica cada transformada

Uma transformada com dois donos é um robô que treme no RViz. A tabela:

| Aresta TF | Dono |
| --- | --- |
| `map → odom` | `amcl` (Nav2) |
| `odom → base_footprint` | plugin DiffDrive do Gazebo |
| `base_footprint → base_link` → sensores | `robot_state_publisher`, a partir do URDF |

---

## 4. Editar o robô

Arquivos em `ros2_ws/src/demo_description/urdf/`:

| Arquivo | Conteúdo |
| --- | --- |
| `demo_robot.urdf.xacro` | principal: dimensões, montagem, plugins Gazebo |
| `_visuals.xacro` | visuais por mesh e o fallback primitivo |
| `_wheel.xacro` | rodas motrizes e roda-boba |
| `_sensors.xacro` | lidar e câmera (URDF + bloco `<sensor>` do gz) |
| `_inertia.xacro` | tensores de inércia |
| `_materials.xacro` | cores (só RViz) |

### Mudar uma dimensão

Tudo é argumento xacro. Nada é hardcoded:

```bash
# Testar sem editar arquivo
ros2 launch demo_description view_robot.launch.py \
    xacro_args:="wheel_separation:=0.30"
```

Gostou, mude o `default` do `<xacro:arg>` correspondente.

> **`wheel_separation` e `wheel_radius` não são cosméticos.** Vão literalmente
> para o plugin DiffDrive, que integra os dois na odometria. Mudar a mesh sem
> mudar esses valores (ou o contrário) produz um robô que **desliza
> visivelmente enquanto reporta uma linha reta** — e nada dá erro. Mudou um,
> confira o outro.

Mudou o raio do chassi? Atualize `robot_radius` nos dois costmaps em
`demo_navigation/config/nav2_params.yaml`. Eles não leem o URDF.

### Aparência: meshes e o fallback

O robô renderiza a partir de meshes DAE de `nav2_minimal_tb4_description`
(iRobot Create 3 + TurtleBot 4), referenciadas por `package://`. **Não estão
copiadas neste repositório** — são ~25 MB de binários que não precisam estar no
git.

```bash
# Padrão: meshes
ros2 launch demo_description view_robot.launch.py

# Fallback: primitivas
ros2 launch demo_description view_robot.launch.py xacro_args:="use_meshes:=false"
```

Duas regras ao mexer nisso:

1. **Visual e colisão são geometrias diferentes de propósito.** Visual é mesh
   (bonito, caro); colisão é primitiva (cilindro/caixa, barato). Usar mesh como
   colisão deixa a física ordens de grandeza mais cara sem ganho nenhum — um
   cilindro descreve muito bem um chassi redondo.
2. **Mantenha o caminho `use_meshes:=false` funcionando.** É a saída para uma
   máquina sem o pacote instalado, e é o que a imagem arm64 da L4 vai usar se
   as meshes forem removidas dela. Colisão, inércia, TF e odometria são
   idênticas nos dois caminhos; só a renderização muda.

Se trocar as meshes, os `rpy` em `_visuals.xacro` vêm dos macros upstream que
sabidamente renderizam certo. Uma rotação errada desenha o robô deitado — e o
sintoma é só visual, porque TF, odometria e costmaps continuam corretos.

### Depois de qualquer edição no URDF

```bash
# Expande e valida a estrutura
xacro demo_robot.urdf.xacro > /tmp/r.urdf && check_urdf /tmp/r.urdf

# Os dois caminhos precisam expandir
xacro demo_robot.urdf.xacro use_meshes:=false > /dev/null && echo "fallback OK"
```

`check_urdf` deve mostrar `base_footprint` como raiz e a árvore de links.

---

## 5. Editar a navegação

`demo_navigation/config/nav2_params.yaml`, ~13 blocos de servidor. Parâmetros
ficam em YAML, **nunca embutidos em código**.

Os que você mais vai mexer:

| Parâmetro | Onde | Efeito |
| --- | --- | --- |
| `max_vel_x`, `max_vel_theta` | `controller_server` | velocidade da demo |
| `robot_radius` | ambos os costmaps | precisa casar com o chassi |
| `inflation_radius` | `inflation_layer` | distância que ele mantém das paredes |
| `xy_goal_tolerance` | `general_goal_checker` | precisão para dar goal por concluído |

Editou YAML? Não precisa recompilar (`--symlink-install`), mas **precisa
reiniciar o Nav2** — parâmetros são lidos no configure do ciclo de vida.

> **Não esvazie listas YAML.** `docks: []` chega ao launch como tupla Python e
> derruba tudo com `Expected 'value' to be one of [float, int, str, bool,
> bytes], but got '()'`. Se não quer a chave, **omita** — não deixe vazia.

---

## 6. Editar a percepção

```
detection_stub.py       gera detecções sintéticas determinísticas
detections_to_cloud.py  Detection2DArray → PointCloud2 para o costmap
```

O adaptador existe porque a `ObstacleLayer` de fábrica do Nav2 não lê
`Detection2DArray`. A alternativa seria um plugin C++ de costmap; a convenção do
projeto é C++ só com desempenho medido no hardware, e nada foi medido no AM69
ainda. Trocar depois não muda nenhum dos dois lados.

**Limitação assumida:** caixa 2D não tem profundidade. O adaptador assume
distância fixa e usa pinhole para o azimute. Daí `clearing: false` (a projeção
é grosseira demais para apagar um obstáculo real visto pelo lidar) e
`observation_persistence: 1.0` (a detecção expira em vez de virar fantasma).

Ao substituir o stub por inferência real, o alvo é: publicar
`vision_msgs/Detection2DArray` em `/demo/perception/detections`, consumindo
`sensor_msgs/Image` de `/demo/camera/image_raw`. Nada mais precisa mudar.

---

## 7. Editar a simulação

### A ponte ROS ↔ Gazebo

`demo_simulation/config/bridge_warehouse.yaml` é a fronteira inteira. Todo nome
do lado ROS vive sob `/demo`.

> **Nenhuma entrada é escopada por modelo, e isso está correto.** Os elementos
> `<topic>`, `<odom_topic>` e `<tf_topic>` do DiffDrive são **literais**: o
> plugin escuta em `/cmd_vel` e publica em `/odom` e `/tf`, sem prefixo. O
> Gazebo *também* anuncia `/model/demo_robot/{cmd_vel,odom,tf}`, que aparecem
> em `gz topic -l` e **parecem** os nomes certos — mas não têm ninguém
> conectado. Apontar a ponte para eles dá um robô que não anda e odometria que
> não publica, **sem erro nenhum**.

Confirme antes de confiar:

```bash
gz topic -l                    # existe?
gz topic -i -t /cmd_vel        # tem publisher/subscriber?
```

`No subscribers on topic` é o sintoma.

### Trocar o mundo

O default resolve para `nav2_minimal_tb4_sim`. Para usar outro, passe
`world:=/caminho/absoluto.sdf`. Se colocar um `.sdf` em
`demo_simulation/worlds/`, ele é instalado pelo `setup.py` — mas prefira não
versionar mundos pesados.

---

## 8. Diagnóstico

Ordem de baixo para cima. Pare no primeiro que falhar.

```bash
# 1. Os nós estão vivos?
ros2 node list

# 2. Os tópicos existem e publicam?
ros2 topic list | grep demo
ros2 topic hz /demo/scan          # esperado ~10 Hz
ros2 topic hz /demo/odom          # esperado ~30 Hz

# 3. A árvore TF está completa e sem furo?
ros2 run tf2_tools view_frames    # gera frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint

# 4. O Nav2 subiu inteiro? Os 7 servidores têm de estar em `active`
ros2 lifecycle list /planner_server
for n in map_server amcl planner_server controller_server \
         bt_navigator behavior_server velocity_smoother; do
    echo -n "$n: "; ros2 lifecycle get /$n
done

# 5. A percepção chega ao costmap?
ros2 topic info /demo/perception/detection_cloud -v   # Subscription count deve ser 2
```

**Lado Gazebo** (nomes gz não são nomes ROS):

```bash
gz topic -l
gz topic -i -t /odom
gz model --list
```

### Sintomas comuns

| Sintoma | Causa provável |
| --- | --- |
| Robô aparece, sensores publicam, mas não anda e `/demo/odom` está mudo | Spawn cedo demais, ou ponte apontando para nome escopado. Ver seção 9. |
| Nav2 metade `active`, metade `inactive` | Um servidor falhou no configure e o lifecycle manager abortou o resto. Veja qual. |
| `slam_toolbox` roda mas não gera mapa | Ficou em `unconfigured` — é lifecycle node. Ver seção 9. |
| Costmap ignora as detecções | `frame_id` fora da árvore TF, ou `observation_persistence` expirando. |
| Tudo trava esperando TF | Falta `/clock`. A ponte precisa estar de pé antes de qualquer nó com `use_sim_time`. |

---

## 9. As armadilhas que já custaram tempo

Todas têm a mesma assinatura: **nenhuma mensagem de erro, tudo parecendo
funcionar**. Estão aqui porque voltam se alguém "limpar" o código.

### 1. Spawn em t=0 nunca completa

`ros_gz_sim create` chama primeiro o serviço de lista de mundos. Em t=0 esse
serviço não existe, e o cliente **retenta a cada 5 s para sempre** em vez de
falhar. O mundo do armazém leva ~10 s carregando meshes.

O robô ainda aparece no mundo — o create assíncrono aceita —, então
`gz model --list` mostra `demo_robot` e os sensores publicam. Mas os plugins
DiffDrive e JointStatePublisher **nunca inicializam**.

→ Não remova os `TimerAction` de 12 s e 15 s em `simulation.launch.py`.

### 2. Tópicos gz não são escopados

Detalhado na [seção 7](#a-ponte-ros--gazebo). É a que mais parece um bug de
configuração e mais consome tempo.

### 3. `slam_toolbox` é lifecycle node

Sobe em `unconfigured` e fica lá. O processo roda, loga normalmente, e **não
cria assinatura de scan nem publica mapa ou `map→odom`**. `ros2 node info`
mostra só `/clock`.

→ `slam.launch.py` usa `LifecycleNode` com `EmitEvent`/`OnStateTransition`
encadeados: activate só depois do configure dar OK.

### 4. `docking_server` sem `dock_plugins` derruba o Nav2 inteiro

Faz parte da lista padrão do Nav2 Jazzy. Sem configuração, falha no configure e
o lifecycle manager **aborta tudo** — `map_server` e `amcl` ficam `active` e o
resto para em `inactive`. A demo não tem dock; está configurado o mínimo.

### 5. Lista YAML vazia quebra o launch

`docks: []` → tupla Python → launch aborta. Omita a chave.

---

## Referências

- `.ai/CLAUDE.md` — contrato operacional, regras invioláveis, fase atual
- `.ai/AGENTS.md` — contrato de implementação, milestones, Definition of Done
- `.ai/changelog.md` — decisões tomadas e o porquê
- README de cada pacote em `ros2_ws/src/demo_*/`
