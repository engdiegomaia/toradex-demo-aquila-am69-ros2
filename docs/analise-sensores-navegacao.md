# Sensores da simulação e cadeia de navegação

Documento de análise da fase L3. Descreve **exatamente** quais dados de sensor
saem do Gazebo, por onde passam até chegar ao Nav2, e como o comando de
navegação volta até os atuadores do robô.

Todos os números abaixo foram **medidos** na simulação (Gazebo Harmonic 8.14.0,
mundo `nav2_minimal_tb4_sim/worlds/warehouse.sdf`), não copiados de
documentação. Onde um valor não pôde ser verificado, está marcado como tal.

> **Onde isto roda:** tudo neste documento roda na **workstation x86**. Gazebo,
> `ros_gz_bridge` e RViz2 nunca rodam no módulo Aquila AM69 (regra 1 do
> `CLAUDE.md`: a GPU do AM69 só expõe OpenGL ES 3.2 / Vulkan 1.2, e o Gazebo é
> OGRE 2 / OpenGL desktop). Em modo `target`, apenas Nav2, bringup, percepção e
> `rosbridge_server` migram para o módulo — o simulador permanece no host.

---

## 1. O modelo do robô

O robô é o **TurtleBot 4 upstream**, incluído diretamente de
`nav2_minimal_tb4_description` (mantido pela equipe Nav2):

```
/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro
```

`demo_description/urdf/demo_robot.urdf.xacro` é hoje apenas um **wrapper fino**
sobre esse arquivo.

**Por que isso mudou.** A versão anterior montava o robô peça por peça a partir
dos meshes individuais do TB4, com offsets derivados de medição de bounding box —
manutenção difícil e propensa a erro. O modelo upstream já vem montado.

### ⚠️ O robô com "partes separadas" no RViz: a causa mais provável

**Verifique isto ANTES de mexer no modelo.** Duas trocas de modelo e uma correção
de física não resolveram esse sintoma, porque a causa estava na configuração do
RViz — não no robô.

O `nav2_bringup/rviz/nav2_default_view.rviz` (que o `learn.launch.py` usava) vem
com:

| Display | Valor upstream | Efeito |
| --- | --- | --- |
| `RobotModel` | `Enabled: false` | **o corpo do robô não é desenhado** |
| `TF` | `Enabled: true`, Show Axes + Show Names | **33 triedros de eixos** desenhados |

Resultado na tela: nenhum corpo de robô e 33 marcadores de eixo rotulados
flutuando (4 colunas da torre, 4 blocos de peso, 6 zonas de para-choque, 6 frames
da OAK-D, IMU, rodas, rodízio) exatamente onde o robô deveria estar. Isso **parece
idêntico** a um robô com as peças separadas.

Diagnosticado lendo as flags de display, não o URDF.

**Correção:** `demo_bringup/rviz/demo_view.rviz` — cópia do arquivo do Nav2 com
`RobotModel` ligado e `TF` desligado. Para depurar frames, marque `TF` na barra
lateral e desmarque depois.

```bash
# Confirme as flags do arquivo que o RViz realmente carrega
python3 -c "
import yaml
d=yaml.safe_load(open('install/demo_bringup/share/demo_bringup/rviz/demo_view.rviz'))
for x in d['Visualization Manager']['Displays']:
    if x.get('Class','').endswith(('RobotModel','TF')):
        print(x['Class'].split('/')[-1], x.get('Enabled'))
"   # RobotModel True / TF False
```

### ⚠️ Juntas fixas não fundidas (problema distinto, também real)

Trocar o modelo **não** resolveu esse sintoma, e vale registrar por quê, porque a
explicação anterior (offsets de mesh errados) estava **incorreta**.

O upstream marca 22 juntas fixas com:

```xml
<gazebo reference="..."><preserveFixedJoint>true</preserveFixedJoint></gazebo>
```

Essa tag diz ao Gazebo para **não** fundir (weld) o filho da junta no corpo rígido
do pai. Sem fusão, o robô nasce como **13 corpos de física independentes** (casco,
4 colunas da torre, placa de sensores, lidar, câmera, para-choque, rodas, rodízio,
base), ligados apenas por restrições de junta fixa. O solver **não** os mantém
rígidos: sob gravidade e contato as peças acomodam e se afastam — a torre inclina,
a placa e o lidar flutuam acima das colunas.

**Não é bug de geometria.** Os offsets e as rotações `rpy` por visual do modelo
upstream estão corretos. Fundir ou não fundir não muda **onde as peças são
desenhadas**, apenas se a física pode movê-las umas em relação às outras. É por
isso que remedir offsets nunca resolveu — e por isso trocar o modelo do robô
também não resolveu.

**Detalhe que torna isso difícil de atribuir:** o RViz2 **ignora** blocos
`<gazebo>` por completo, então sempre desenhou o robô montado corretamente. O
modelo parece perfeito no RViz e se desmonta no Gazebo.

**Correção.** O xacro não consegue remover uma tag emitida por um `include`, então
a remoção acontece no pipeline que alimenta o Gazebo:

```bash
weld_fixed_joints.py demo_robot.urdf.xacro robot_name:=demo_robot > robot.urdf
```

`demo_simulation/launch/simulation.launch.py` já faz isso. Se você spawnar esse
xacro por qualquer outro caminho, passe pelo script também.

Dois detalhes do script, ambos aprendidos na prática:
- Ele é **silencioso** em caso de sucesso. A substituição `Command` do launch
  aborta o launch inteiro se o comando escrever **qualquer coisa** em stderr
  (`executed command showed stderr output`) — uma mensagem de progresso amigável
  derrubava a simulação. Use `--verbose` ao rodar à mão.
- Ele é chamado **em vez de** `xacro`, não em pipe: `Command` passa a string por
  `shlex.split`, não por um shell, então um `|` chegaria ao xacro como argumento
  literal.

Travado por `test_weld_script_removes_every_preserve_fixed_joint`.

**Efeito colateral positivo:** ganhamos sensores que não existiam antes — câmera
RGBD estéreo (OAK-D Pro) e IMU.

### Árvore TF relevante

```
odom                              ← publicado pelo plugin DiffDrive do Gazebo
  └── base_link                   RAIZ do URDF; robot_base_frame do Nav2
        ├── base_footprint        transform IDENTIDADE (xyz 0 0 0, rpy 0 0 0)
        ├── shell_link
        │     ├── rplidar_link            z = +0.193 m
        │     └── oakd_camera_bracket
        │           └── oakd_link
        │                 └── oakd_rgb_camera_frame → …_optical_frame  z = +0.244 m
        ├── imu_link                      z = +0.084 m
        ├── left_wheel / right_wheel      (juntas continuous)
        └── front_caster_link
```

**Atenção — a árvore está invertida em relação ao modelo antigo:**

| | raiz | filho |
|---|---|---|
| modelo antigo (nosso) | `base_footprint` | `base_link` |
| modelo atual (upstream) | `base_link` | `base_footprint` |

O `DiffDrive` do upstream tem `child_frame_id` **fixo em `base_link`**, definido
em `icreate/create3.urdf.xacro`, e **não existe argumento xacro** para alterá-lo.

Por isso o Nav2 foi apontado para `base_link` (`robot_base_frame: base_link` em
todos os pontos de `nav2_params.yaml`), em vez de tentar forçar `base_footprint`.
Isso é seguro porque `base_footprint_joint` é uma transformação identidade — os
dois frames coincidem numericamente. Existe um teste
(`demo_description/test/test_urdf_parses.py::test_base_footprint_coincides_with_base_link`)
que **falha** se uma atualização futura do upstream der um offset real a essa
junta e quebrar essa equivalência.

> **Armadilha (custou tempo real).** A tentativa óbvia — redeclarar o bloco
> `<gazebo><plugin ...DiffDrive>` no nosso wrapper para trocar só o
> `child_frame_id` — **não funciona**. O xacro **concatena** blocos `<gazebo>`,
> não os substitui. O resultado é um URDF com **dois** plugins DiffDrive
> controlando as mesmas duas juntas, ambos integrando odometria e publicando TF.
> O Gazebo carrega os dois sem reclamar. Verificação:
> ```bash
> xacro src/demo_description/urdf/demo_robot.urdf.xacro | grep -c diff-drive-system   # deve ser 1
> ```
> Há um teste que trava isso: `test_exactly_one_of_each_gz_system_plugin`.

---

## 2. Dados de sensor recebidos da simulação

### 2.1 Tabela completa (valores medidos)

| Sensor | Tópico gz | Tópico ROS | Tipo ROS | Taxa | `frame_id` | Consumidor |
|---|---|---|---|---|---|---|
| Lidar 2D (RPLIDAR A1) | `/scan` | `/demo/scan` | `sensor_msgs/LaserScan` | 10 Hz | `rplidar_link` | Nav2 (ambos costmaps), AMCL, SLAM |
| Câmera RGB (OAK-D Pro) | `/rgbd_camera/image` | `/demo/camera/image_raw` | `sensor_msgs/Image` | 10 Hz | `oakd_rgb_camera_optical_frame` | `demo_perception` |
| Intrínsecos | `/rgbd_camera/camera_info` | `/demo/camera/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | idem | `demo_perception` |
| Profundidade | `/rgbd_camera/depth_image` | `/demo/camera/depth_image` | `sensor_msgs/Image` | 10 Hz | idem | **nenhum ainda** |
| Nuvem de pontos | `/rgbd_camera/points` | `/demo/camera/points` | `sensor_msgs/PointCloud2` | 10 Hz | idem | **nenhum ainda** |
| IMU | `/imu` | `/demo/imu` | `sensor_msgs/Imu` | 200 Hz | `imu_link` | **nenhum ainda** |
| Odometria de roda | `/odom` | `/demo/odom` | `nav_msgs/Odometry` | 30 Hz | `odom` → `base_link` | Nav2 (controller, bt_navigator) |
| Ângulos de junta | `/joint_states` | `/joint_states` | `sensor_msgs/JointState` | 30 Hz | — | `robot_state_publisher` |
| Relógio | `/clock` | `/clock` | `rosgraph_msgs/Clock` | — | — | **todos** os nós |

### 2.2 Lidar — parâmetros medidos

Verificado com `gz topic -e -t /scan -n 1`:

```
frame_id   = rplidar_link
count      = 360          amostras por varredura
range_min  = 0.164 m
range_max  = 20.0 m
update     = 10 Hz
```

O `range_max` de **20 m** é importante: o lidar antigo, feito à mão, declarava
12 m, e o `nav2_params.yaml` ainda estava configurado para 12 m. Ver
"Otimização 2".

### 2.3 IMU — depende do mundo, não só do robô

O sensor IMU está declarado no URDF, mas **só publica se o MUNDO carregar o
sistema `gz-sim-imu-system`**. O `warehouse.sdf` do `nav2_minimal_tb4_sim`
carrega (verificado). Um mundo escrito à mão pode não carregar, e nesse caso o
sensor fica **silencioso, sem erro nenhum**.

```bash
gz topic -l | grep imu      # se vazio, o mundo não tem o plugin imu-system
```

Foi exatamente assim que este documento quase registrou "IMU morta": um mundo de
teste mínimo sem o plugin. O sensor estava correto o tempo todo.

### 2.4 Como o dado atravessa a fronteira Gazebo → ROS

O Gazebo **não fala ROS**. A tradução é feita pelo `ros_gz_bridge`
(`parameter_bridge`), configurado em
`demo_simulation/config/bridge_warehouse.yaml`. Cada linha é um par
(tópico gz, tópico ROS, tipo, direção).

```
Gazebo (gz-transport, protobuf)        ros_gz_bridge         ROS 2 (DDS/CycloneDDS)
  /scan          gz.msgs.LaserScan   ───────────────►   /demo/scan     sensor_msgs/LaserScan
  /rgbd_camera/image  gz.msgs.Image  ───────────────►   /demo/camera/image_raw
  /imu           gz.msgs.IMU         ───────────────►   /demo/imu
  /odom          gz.msgs.Odometry    ───────────────►   /demo/odom
  /cmd_vel       gz.msgs.Twist       ◄───────────────   /demo/cmd_vel  geometry_msgs/Twist
```

**Armadilha de nomes (documentada no próprio YAML).** Todos os nomes do lado gz
são **literais e não escopados** (`/cmd_vel`, `/odom`, `/scan`). O Gazebo
*também* anuncia nomes parecidos e escopados por modelo
(`/model/demo_robot/cmd_vel`) que **aparecem em `gz topic -l` mas não têm
publicador algum por trás**. Fazer bridge para esses nomes produz um robô que
nunca se move e odometria que nunca publica — **sem erro em lugar nenhum**.
Sempre confira com `gz topic -l` com a simulação rodando.

---

## 3. Como a navegação chega aos atuadores

### 3.1 Cadeia completa

```
        ┌─ /demo/scan (10 Hz) ──► costmaps (obstacle_layer)
        │                          ├─ global_costmap  (1 Hz)  ──► planner_server
        │                          └─ local_costmap   (5 Hz)  ──► controller_server
        │
        ├─ /demo/scan ──────────► AMCL ──► TF: map → odom
        │
        ├─ /demo/odom (30 Hz) ──► controller_server (velocidade atual)
        │
        └─ /demo/perception/detection_cloud ──► perception_layer (ambos costmaps)

  objetivo (RViz2 / BT) ──► bt_navigator ──► planner_server (NavFn, 20 Hz)
                                                    │
                                                    ▼ nav_msgs/Path (global)
                                            controller_server (20 Hz)
                                                    │  MPPI: 2000 trajetórias
                                                    ▼
                                          geometry_msgs/Twist
                                                    │
                                            /demo/cmd_vel
                                                    │  ros_gz_bridge (ROS→GZ)
                                                    ▼
                                            /cmd_vel  (Gazebo)
                                                    │
                                    plugin gz-sim-diff-drive-system
                                                    │  cinemática inversa
                                                    ▼
                            torque em left_wheel_joint / right_wheel_joint
                                                    │
                                    ┌───────────────┴────────────────┐
                                    ▼                                ▼
                        /odom (30 Hz) + TF odom→base_link      /joint_states (30 Hz)
                                                                      │
                                                          robot_state_publisher
                                                                      ▼
                                                          TF das rodas (visual)
```

### 3.2 O único ponto de atuação: `/demo/cmd_vel`

Não existem controladores de junta, nem `ros2_control`, nem
`controller_manager`. Toda a atuação passa por **um único tópico**:
`/demo/cmd_vel` (`geometry_msgs/Twist`).

O plugin `gz-sim-diff-drive-system` recebe esse `Twist` e resolve a cinemática
inversa do diferencial:

```
v_esq = (v_linear − ω · L/2) / r
v_dir = (v_linear + ω · L/2) / r

onde  L = wheel_separation = 0.233 m
      r = wheel_radius     = 0.03575 m
```

**`wheel_separation` e `wheel_radius` são os dois valores que nunca podem
divergir do modelo físico.** O plugin os integra para produzir odometria; se
divergirem dos meshes/colisões, o robô **desliza visivelmente enquanto reporta
uma linha reta** — e nada dá erro. Herdar esses valores do upstream (em vez de
redeclará-los) os mantém travados à geometria por definição.

Verificado em execução: comando `linear.x = 0.3` por 2 s deslocou o robô
**0.6046 m** (esperado 0.6 m) com odometria acompanhando.

### 3.3 Limites que o simulador impõe

O `DiffDrive` do upstream **satura internamente**:

| Limite | Valor |
|---|---|
| `max_linear_velocity` | 0.5 m/s |
| `max_angular_velocity` | 2.0 rad/s |
| `max_linear_acceleration` | 2.0 m/s² |
| `max_angular_acceleration` | 3.0 rad/s² |

**Consequência prática:** configurar o Nav2 com velocidade acima desses valores
faz o controlador comandar velocidades que o simulador **silenciosamente recusa
atingir**. O sintoma parece erro de tuning do controlador — e não é. Os limites
em `nav2_params.yaml` foram alinhados a esses valores (`vx_max: 0.5`,
`wz_max: 1.9`, com margem em ω).

### 3.4 Quem publica cada aresta de TF

Duplicar qualquer uma destas produz um robô tremendo, sem mensagem de erro.

| Aresta | Publicador |
|---|---|
| `map → odom` | AMCL (ou `slam_toolbox` em SLAM) |
| `odom → base_link` | plugin DiffDrive do Gazebo (via bridge `/tf`) |
| `base_link → *` (sensores, rodas) | `robot_state_publisher` |

---

## 4. Otimizações

### Aplicadas nesta sessão

#### Otimização 1 — `robot_radius` 0.22 → 0.18 m

O raio real do chassi do TB4 é ~0.17 m (`shell_radius` 0.12 + para-choque). O
valor 0.22 vinha do chassi do modelo antigo e adicionava ~5 cm de largura
fantasma. Efeito: o robô **recusa vãos pelos quais fisicamente passa**, o que num
corredor de galpão se manifesta como falha do planejador.

#### Otimização 2 — alcance do lidar 12 → 20 m

O `obstacle_layer` estava com `raytrace_max_range: 12.0`, herdado do lidar antigo.
O sensor real alcança **20 m** (medido). Quando o raytrace é menor que o alcance
real, a camada **para de limpar células que nunca observou de fato**, deixando
obstáculos obsoletos congelados no costmap.

Também adicionado `obstacle_min_range: 0.17` (acima do `range_min` de 0.164 do
sensor): retornos mais próximos são artefatos de medição, e marcá-los planta
obstáculos **dentro da própria pegada do robô**, disparando comportamentos de
recuperação espúrios.

`obstacle_max_range` (15 m) fica deliberadamente **abaixo** de
`raytrace_max_range` (20 m): marcar obstáculo de forma conservadora, limpar espaço
livre de forma generosa.

#### Otimização 3 — inflação 0.55 → 0.35 m, `cost_scaling_factor` 3.0 → 5.0

0.55 m é **mais de 3× o raio do robô**. Nos corredores estreitos do galpão, uma
"saia" de 0.55 m em cada parede tornava o corredor inteiro quase letal: o
planejador global desviava de corredores por onde o robô passa, e o local
oscilava dentro deles.

0.35 m cobre a pegada de 0.18 m com ~0.17 m de folga. **Nunca reduza abaixo de
`robot_radius`** — inflação menor que a pegada permite caminhos cujas curvas
raspam obstáculos.

`cost_scaling_factor` 5.0 faz o custo cair mais rápido com a distância, deixando
o planejador se comprometer com o **centro** do corredor livre.

Os dois costmaps foram mantidos **idênticos** nesses valores: inflação divergente
entre global e local é causa clássica de "o plano global vai por onde o local se
recusa a seguir" — o robô trava no meio do corredor.

#### Otimização 4 — DWB → MPPI

**Por que.** O DWB pontua uma grade fixa de arcos de curvatura constante. Num
diferencial em corredor estreito isso produz dois artefatos visíveis: ele
**oscila entre amostras vizinhas** (o robô serpenteia num corredor reto) e não
consegue representar manobra que exija inversão de sinal de velocidade no meio da
trajetória, então curvas fechadas degeneram em para-gira-anda.

O MPPI amostra 2000 trajetórias ruidosas por ciclo e tira a média ponderada por
custo, então o comando é **contínuo** em vez de preso a uma amostra da grade. É a
recomendação atual do Nav2 para diferencial.

Detalhes de configuração:
- `motion_model: DiffDrive` — **não** `Omni`. Este robô não anda de lado;
  declarar `Omni` faz o MPPI amostrar velocidades laterais inexecutáveis e então
  perseguir um caminho que nunca consegue seguir.
- `CostCritic` em vez de `ObstaclesCritic`: respeita o gradiente da camada de
  inflação em vez de aplicar o próprio modelo de pegada, então a
  `inflation_radius` ajustada acima é de fato o que guia o robô.
- Os 8 críticos foram **verificados** contra `libmppi_critics.so`. Um crítico com
  nome errado **não é carregado e não gera erro** — simplesmente para de
  contribuir.

> ⚠️ **Custo de CPU — só mensurável no hardware real.** O MPPI é
> significativamente mais pesado que o DWB. No host x86 (modo `learn`) é
> tranquilo. No Aquila AM69 em modo `target` este é o nó mais caro da pilha, e
> **se 20 Hz se sustenta lá não pode ser respondido a partir do modo `learn` nem
> de emulação arm64** (regra 5 do `CLAUDE.md`: emulação não mede performance).
> Precisa ser medido no módulo real. Se não sustentar, reduza `batch_size` para
> 1000 **antes** de baixar `controller_frequency` — metade do lote custa menos
> precisão que metade da taxa de controle.

#### Otimização 5 — expor profundidade, nuvem de pontos e IMU no bridge

Bridge dos novos sensores para `/demo/camera/depth_image`,
`/demo/camera/points` e `/demo/imu`. Ainda **não consumidos** — mas disponíveis e
mensuráveis, e prontos para o trabalho de TIDL (MX-TIDL) sem alterar o bridge.

> ⚠️ **Banda:** `/demo/camera/points` é de longe o tópico mais pesado
> (~640×480 XYZRGB a 10 Hz). Em modo `target` esse tráfego DDS atravessa a rede
> até o módulo. Comente a entrada se o demo não precisar de obstáculos 3D.

### Recomendadas, ainda não aplicadas

Estas ficaram de fora **deliberadamente**, por alterarem estrutura (não apenas
tuning) e merecerem validação isolada:

#### A — Fusão IMU + odometria via EKF (`robot_localization`)

**Ganho.** A odometria de roda pura acumula deriva de rumo em **toda** rotação
(escorregamento de roda). O IMU a 200 Hz corrige exatamente isso. É a melhoria de
localização com melhor relação custo/benefício disponível agora.

**Por que não foi aplicada agora.** Um `ekf_node` passa a ser o dono da aresta
`odom → base_link`, que hoje é do plugin DiffDrive. **Dois publicadores na mesma
aresta produzem um robô tremendo, sem mensagem de erro.** Exige:
1. criar `demo_navigation/config/ekf.yaml`;
2. **desativar** a publicação de TF do DiffDrive (ou remover `/tf` do bridge);
3. revalidar a árvore com `ros2 run tf2_tools view_frames`.

`robot_localization` já está instalado (`ekf_node` verificado).

#### B — Obstáculos 3D via `voxel_layer` na nuvem de pontos

Hoje o robô só vê obstáculos **no plano do lidar** (z ≈ 0.19 m). Ele é cego a
paletes baixos e a saliências acima do plano. A nuvem OAK-D já está no bridge;
falta uma `nav2_costmap_2d::VoxelLayer`. Custa CPU e banda — medir no módulo.

#### C — `RotationShimController` em torno do MPPI

Faz o robô **girar no lugar** para se alinhar ao caminho antes de acelerar, em vez
de sair em arco. Melhora bastante a leitura visual do demo em partidas e curvas
fechadas. Verificado como instalado
(`nav2_rotation_shim_controller::RotationShimController`).

#### D — Trocar NavFn por Smac Planner (`SmacPlannerHybrid`)

NavFn produz caminhos em grade, com quinas de 45°. O Smac Hybrid gera caminhos
cinematicamente viáveis para diferencial. Ganho estético e de suavidade; custo de
CPU maior no planejador global (que roda a 1 Hz, então o impacto é menor que no
controlador).

---

## 4.5 Metas em espaço desconhecido (não é bug de tuning)

Sintoma no log, ao clicar uma meta no RViz:

```
[ERROR] [planner_server]: Failed to create a plan from potential when a legal
        potential was found. This shouldn't happen.
[WARN]  GridBased plugin failed to plan from (4.76, 0.41) to (11.01, 10.61):
        "Failed to create plan with tolerance of: 0.500000"
```

A mensagem "This shouldn't happen" sugere bug interno do planejador. **Não é.**

O `warehouse.pgm` salvo tem **55% das células desconhecidas**, porque o SLAM só
mapeou os corredores por onde o robô passou:

| Classe | Células | % |
| --- | --- | --- |
| FREE | 98 863 | 43.6% |
| OCCUPIED | 3 393 | 1.5% |
| **UNKNOWN** | **124 319** | **54.9%** |

A meta `(11.01, 10.61)` está **inteiramente** em espaço desconhecido — valor 205
no PGM em toda a vizinhança de 1 m. A meta `(-4.84, 3.38)`, que funcionou, estava
em espaço livre (254).

`allow_unknown: true` **não resolve** esse caso: ele permite atravessar células
desconhecidas alcançáveis, não criar um corredor até uma meta cercada de
desconhecido.

**Como checar uma meta antes de clicar:**

```bash
python3 - <<'EOF'
X, Y = 11.01, 10.61      # a meta que você quer testar
res, ox, oy = 0.05, -12.077, -12.215
p='install/demo_navigation/share/demo_navigation/maps/warehouse.pgm'
f=open(p,'rb'); f.readline(); l=f.readline()
while l.startswith(b'#'): l=f.readline()
w,h=map(int,l.split()); f.readline(); d=f.read(w*h)
cx, cy = int((X-ox)/res), int((Y-oy)/res)
v = d[(h-1-cy)*w + cx]
occ = (255-v)/255.0
print(f"pgm={v} ->", 'OCCUPIED' if occ>0.65 else ('FREE' if occ<0.196 else 'UNKNOWN'))
EOF
```

**Soluções, em ordem de esforço:**
1. Clique metas apenas em área branca (livre) do mapa no RViz — o cinza é
   desconhecido.
2. Refaça o mapa cobrindo mais área: rode o SLAM e dirija o robô pelos corredores
   que faltam antes de salvar (`map_saver_cli`).
3. Para o demo, defina metas fixas verificadas em vez de cliques livres.

Isto **não** é regressão das otimizações da seção 4 — a primeira meta do mesmo
run completou com `Reached the goal!` / `Goal succeeded`.

## 5. Como verificar tudo isto

```bash
cd ~/toradex/demo/aquila-am69-ros2/ros2_ws
source install/setup.bash          # obrigatório em CADA terminal novo

# 1) O URDF expande, tem 1 plugin de cada, e a árvore TF é a esperada
python3 -m pytest src/demo_description/test/test_urdf_parses.py -q

# Conte os ELEMENTOS de plugin, não linhas de texto. Qualquer grep aqui é
# inútil: os próprios comentários do xacro citam o nome do plugin e casam com o
# padrão. Parseie o XML.
xacro src/demo_description/urdf/demo_robot.urdf.xacro | python3 -c "
import sys,xml.etree.ElementTree as ET
from collections import Counter
r=ET.fromstring(sys.stdin.read())
print(Counter(p.get('name') for g in r.findall('gazebo') for p in g.findall('plugin')))
"   # DiffDrive: 1, JointStatePublisher: 1

# 2) Nenhuma junta fixa preservada — senão o robô se desmonta no Gazebo
python3 src/demo_description/scripts/weld_fixed_joints.py --verbose \
  src/demo_description/urdf/demo_robot.urdf.xacro | grep -c preserveFixedJoint  # 0

# 2) Lado Gazebo: os nomes literais existem e têm publicador
gz topic -l | sort
gz topic -e -t /scan -n 1 | grep -E "frame|count|range_m"
gz topic -l | grep imu             # vazio ⇒ o mundo não tem gz-sim-imu-system

# 3) Lado ROS: taxa real de cada sensor (não só "o tópico existe")
ros2 topic hz /demo/scan           # ~10 Hz
ros2 topic hz /demo/odom           # ~30 Hz
ros2 topic hz /demo/imu            # ~200 Hz
ros2 topic hz /demo/camera/image_raw

# 4) Árvore TF sem aresta duplicada nem frame órfão
ros2 run tf2_tools view_frames

# 5) Atuação de ponta a ponta
ros2 topic pub --once /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /demo/odom --once  # a posição deve ter mudado
```

**Regra geral deste projeto:** um tópico que existe em `ros2 topic list` **não**
prova que há publicador. Use sempre `ros2 topic hz` / `gz topic -i`. Quase toda
falha silenciosa documentada aqui aparece como um tópico presente e mudo.
