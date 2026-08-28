# Guia completo — operação e cockpit web

Como rodar a demo, como mexer nela sem quebrar o que já funciona, e como usar o
cockpit web que a acompanha. Dois guias que eram arquivos separados
(`guia-operacao.md` e `guia-cockpit.md`) e foram unificados aqui em
26/08/2026 — o conteúdo não mudou, só deixou de estar espalhado.

Escrito para quem chega no projeto sem experiência prévia de ROS 2. Onde uma
armadilha conhecida existe, ela está descrita no ponto em que você a
encontraria — não numa seção de troubleshooting no fim.

---

## Sumário

**[Parte I: Operação da demo](#parte-i-operação-da-demo)**

1. [Antes de começar](#1-antes-de-começar)
2. [Rodar a demo](#2-rodar-a-demo)
3. [Como o projeto se encaixa](#3-como-o-projeto-se-encaixa)
4. [Editar o robô](#4-editar-o-robô)
5. [Editar a navegação](#5-editar-a-navegação)
6. [Editar a percepção](#6-editar-a-percepção)
7. [Editar a simulação](#7-editar-a-simulação)
8. [Diagnóstico](#8-diagnóstico)
9. [As armadilhas que já custaram tempo](#9-as-armadilhas-que-já-custaram-tempo)
10. [O módulo Aquila AM69](#10-o-módulo-aquila-am69)

**[Parte II: Cockpit web](#parte-ii-cockpit-web)**

1. [O que é o cockpit](#1-o-que-é-o-cockpit)
2. [Rodar](#2-rodar)
3. [A tela](#3-a-tela)
4. [Os controles](#4-os-controles)
5. [O que NÃO dá para fazer](#5-o-que-não-dá-para-fazer)
6. [Trocar de cenário](#6-trocar-de-cenário)
7. [Ajustar qualidade de imagem](#7-ajustar-qualidade-de-imagem)
8. [Diagnóstico do cockpit](#8-diagnóstico-do-cockpit)
9. [As armadilhas do cockpit que já custaram tempo](#9-as-armadilhas-do-cockpit-que-já-custaram-tempo)
10. [Como o cockpit é feito](#10-como-o-cockpit-é-feito)
11. [O que está feito e o que falta](#11-o-que-está-feito-e-o-que-falta)

---

# Parte I: Operação da demo

Como rodar a demo e como mexer nela sem quebrar o que já funciona.

Escrito para quem chega no projeto sem experiência prévia de ROS 2. Onde uma
armadilha conhecida existe, ela está descrita no ponto em que você a
encontraria — não numa seção de troubleshooting no fim.

**Fase atual: L3 concluída.** As seções 1 a 9 descrevem a demo rodando no host
x86, nativo, sem containers. A [seção 10](#10-o-módulo-aquila-am69) é a exceção:
cobre o módulo Aquila AM69, que é sempre container e sempre `arm64`.

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

### Cockpit de demonstração

Uma página web com as cinco regiões da demo — cena, navegação, logs, câmera e
barra de controle — servida por container e aberta no navegador:

```bash
cd docker
docker compose -f compose.host.yml --profile learn up -d --build
# abra http://localhost:8081
```

**O guia completo está na [Parte II](#parte-ii-cockpit-web) deste documento**:
o que cada painel mostra, o que os botões fazem, como trocar de cenário, como
ajustar qualidade de imagem, e as armadilhas próprias dele.

> **O caminho antigo foi descartado.** Havia aqui um procedimento que tentava
> **incorporar janelas X11** de Gazebo, RViz e `rqt_image_view` numa aplicação
> única (`./scripts/run_cockpit.sh`). Quatro tentativas, nenhuma aceita: no
> último ensaio RViz e câmera permaneceram externos e os painéis internos
> ficaram vazios. **Não retome esse caminho** — o checkpoint está em
> [`results/cockpit-standalone-parcial.md`](results/cockpit-standalone-parcial.md),
> marcado como superado.
>
> O motivo de fundo não era de implementação: RViz e Gazebo são OGRE 2 e
> precisam de OpenGL de desktop, então nunca poderiam ir para o Aquila (regra 1
> do `CLAUDE.md`). Aquele cockpit jamais viraria o HMI do módulo. O cockpit web,
> que renderiza a partir de tópicos ROS 2, vira.

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

> Reiniciar aqui significa **subir o launch de novo** (ou recriar o serviço
> `nav`). Não serve o `/demo/nav/reset` descrito abaixo: ele não passa por
> `CONFIGURE`, de propósito, então parâmetro novo não é lido. Um reset que
> "não pegou a mudança" é o sintoma.

### Descartar a meta e limpar os costmaps

```bash
ros2 service call /demo/nav/reset std_srvs/srv/Trigger
```

Serve para "a demo travou, quero recomeçar sem derrubar nada": cancela toda meta
em andamento, esvazia os dois costmaps e recicla os servidores por
`PAUSE`/`RESUME`. ~6,6 s no host. É o mesmo serviço do botão **reiniciar nav** do
cockpit. Sobe junto com os dois caminhos de Nav2 (`navigation.launch.py` e
`nav_quadruped.launch.py`); `nav_control:=false` desliga.

`/demo/nav/cancel` faz só a primeira parte, sem tocar em costmap nem em ciclo de
vida.

> **Não troque isso por `RESET`+`STARTUP` no `lifecycle_manager`.** É o caminho
> óbvio e ele derruba o container inteiro com `SIGSEGV` ao configurar o
> `route_server` — medido duas vezes, determinístico. Ver a armadilha 8 do cockpit,
> [aqui](#8-reset--startup-no-nav2-mata-o-container-segfault-no-route_server).

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

### 6. `autodetermine` do CycloneDDS escolhe a bridge do Docker

No Aquila, `ip -br addr` mostra `ethernet0` **e** `br-a00dfb945795`
(192.0.2.9) UP ao mesmo tempo — a stack de easy-pairing da própria Toradex roda
em compose. `autodetermine` ranqueia interfaces e pode escolher a bridge. O
CycloneDDS então transmite num endereço que o host não roteia, e **nenhum log de
nenhum dos lados menciona interface**.

→ `scripts/module.sh sync` fixa a interface, detectada a partir de `MODULE_IP`.
Não troque por `autodetermine` "para simplificar", e não fixe `ethernet0` à mão
— placa diferente ou mudança para `wlan0` deixa o nome obsoleto e falhando
calado.

### 7. `--packages-skip` do colcon não é `--packages-ignore`

O grafo de dependências do `colcon` não distingue `exec_depend` de
`build_depend`. `--packages-skip gz_quadruped_hardware` mantém o pacote no grafo
sem construí-lo, e `demo_simulation` (que o declara como `exec_depend`) falha
pedindo `install/gz_quadruped_hardware/.../package.sh`. O dano real é o
`demo_bringup` que vem depois virar "not processed" — e é lá que moram
`nav.launch.py` e `perception.launch.py`, os entrypoints dos dois serviços do
módulo.

→ Use `--packages-ignore`. A mensagem de erro não menciona a diferença.

### 8. Dois publishers em `/demo/cmd_vel` não geram erro nenhum

`nav` no módulo é o Nav2, e o Nav2 publica `/demo/cmd_vel`. Se a simulação do
host estiver rodando no mesmo `ROS_DOMAIN_ID`, o robô simulado passa a receber
comandos de duas origens. O tópico é válido, os dois publishers estão saudáveis,
o DDS faz exatamente o que foi mandado — e o robô se move sozinho. Um ensaio de
marcha em curso é **corrompido, não interrompido**.

→ `scripts/module.sh up` detecta simulação ativa no host e recusa. As saídas
estão na mensagem de recusa.

### 9. `docker compose exec` não roda o ENTRYPOINT da imagem

```
$ docker compose exec -T tools bash -lc 'which ros2'
                      # (nada)
```

O `entrypoint.sh` é que faz `source` do underlay e do overlay `/ws/install`, e
`exec` não o executa. `bash -lc` não salva: a imagem `ros` não coloca o setup no
`.bashrc`.

O que torna isso venenoso é a combinação usual:

```bash
ros2 topic list 2>/dev/null | grep /demo/ || echo "(nenhum topico visivel)"
```

O `2>/dev/null` engole `command not found`, e o `||` imprime a **mesma linha**
que uma falha real de descoberta imprimiria. Isso já fez duas etapas de
verificação relatarem um problema de DDS que não existia.

→ Sempre `docker compose exec <svc> /usr/local/bin/entrypoint.sh <comando>`.
E não use `2>/dev/null` em comando de diagnóstico. Com `exec -d`, confirme
depois que o nó subiu — `-d` esconde todo erro.

### 10. `ROS_NAMESPACE` não funciona no ROS 2

```
$ ... -e ROS_NAMESPACE=/demo ... printenv ROS_NAMESPACE
/demo
$ ros2 run demo_tutorials heartbeat_publisher              → /system/heartbeat
$ ros2 run ... --ros-args -r __ns:=/demo                   → /demo/system/heartbeat
```

A variável **está** no ambiente do processo. O ROS 2 a ignora — é um resquício
de ROS 1.

→ Use `--ros-args -r __ns:=<ns>`. Note que `scripts/env.sh` exporta
`ROS_NAMESPACE=/demo` como se funcionasse; aquela linha não tem efeito.

### 11. Metade da configuração de DDS falha igual a firewall

Com o módulo configurado certo (multicast off, peers) e um publisher
**comprovadamente rodando** nele, o host não via nada. Duas causas ao mesmo
tempo:

| Direção | Por que falhava |
| --- | --- |
| host → módulo | O default do CycloneDDS anuncia por **multicast**; o módulo tem `AllowMulticast=false` e nunca escuta. |
| módulo → host | O módulo manda SPDP unicast para as portas RTPS do host, mas um participante default **não fixa porta determinística** — usa efêmera e conta com multicast para ser achado. Não há porta para mirar. |

Os dois lados precisam de config **casada**: multicast off,
`ParticipantIndex=auto`, e o outro endereço como `<Peer>`. Configurar só um lado
produz exatamente o sintoma de firewall bloqueando.

→ `scripts/module.sh sync` renderiza os dois: `module.xml` (vai para o módulo) e
`docker/cyclonedds/host.rendered.xml` (fica no host, gitignored). Quem publica no
host precisa apontar `CYCLONEDDS_URI` para o arquivo renderizado.
`scripts/run_quadruped_sim.sh` monta e seleciona esse arquivo quando ele existe
e imprime a interface e o peer usados.

### 12. Matar o `ros2 launch` deixa os nós vivos, e o próximo Nav2 morre acusando o DDS

Sintoma: você reinicia o Nav2 e **todos** os nós morrem na subida, cada um com

```
[rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  failed to initialize rcl node: error not set, at ./src/rcl/node.c:252
```

e o `lifecycle_manager` fica para sempre em `Waiting for service
controller_server/get_state...`.

A mensagem acusa o CycloneDDS. O culpado é a execução **anterior**.
`ros2 launch` é só o pai: um `kill` nele **órfã os nós filhos**, que seguem
vivos segurando índice de participante do domínio. Medido em 21/08/2026:
31 órfãos acumulados de 5 gerações de launch, 14 falhas de domínio na subida
seguinte. Um nó isolado ainda criava domínio sem erro, o que faz parecer que o
DDS está bem — e está; o que acabou foi o espaço de índice.

Como confirmar, antes de mexer em configuração de DDS:

```bash
ps -eo pid,etimes,comm --no-headers | grep -E \
  'odom_tf|controller_serv|bt_navigator|behavior_server|route_server'
```

Se aparecerem PIDs com `etimes` maior que a sua sessão atual, são órfãos.

Como limpar. Use `pkill -x`, que casa o **nome** do processo, e não `pkill -f`:

```bash
for n in odom_tf cmd_vel_si_to_s velocity_smooth waypoint_follow \
         behavior_server smoother_server route_server opennav_docking \
         controller_serv planner_server bt_navigator collision_monit \
         lifecycle_manag; do pkill -9 -x "$n"; done
```

Dois detalhes que custam tempo sozinhos:

- **`pkill -f` casa a própria linha de comando de quem chama.** `pkill -f nav2`
  digitado num shell cujo comando contém `nav2` mata o shell. Aconteceu duas
  vezes aqui, e o sintoma é o comando "falhar" sem imprimir nada. `-x` não tem
  esse problema. Se precisar de `-f`, escreva o padrão com classe de caractere:
  `pkill -f '[n]av2'`.
- **Os nomes em `-x` são truncados em 15 caracteres**, que é o limite de `comm`
  no Linux: é `collision_monit`, não `collision_monitor`.

### 13. No modo HIL o robô fica lento e a culpa não é do Aquila

O sintoma: em `hil` o robô navega muito mais devagar que em `learn`, e a
tentação é dizer que o AM69 é fraco. Medido em 21/08/2026, **não é**.

O que atravessa o Wi-Fi é que pesa. O maior item, medido no fio:

```
/demo/camera/image_raw: 640x480 rgb8, 921600 bytes/quadro, 10,1 Hz
                        -> 74,2 Mbit/s
```

É `sensor_msgs/Image` **cru**, sem compressão, com QoS confiável — cada perda
vira retransmissão, e retransmissão vira contrapressão no publicador **dentro do
simulador**. Por isso o efeito aparece na velocidade do robô e não num erro de
rede. Nada em log nomeia a câmera.

Subindo só `nav` no módulo, sem `perception`, ninguém assina a câmera, o
CycloneDDS não a transmite, e a velocidade média sobe 2,2× (0,0197 → 0,0427
m/s). Com o módulo inteiro parado, 0,0725 m/s.

Como conferir antes de culpar o hardware:

```bash
# banda real do tópico, do lado do host
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node('cam'); got = []
n.create_subscription(Image, '/demo/camera/image_raw',
                      lambda m: got.append(len(m.data)), 10)
t = time.monotonic()
while time.monotonic() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
print('%.1f Mbit/s' % (sum(got) * 8 / 1e6 / (time.monotonic() - t)))
EOF
```

E **não** use `ros2 topic hz` para isso: nesta configuração de DDS ele volta sem
imprimir nada, em qualquer tópico, o que parece tópico morto.

Em 21/08/2026, uma sonda e o `detection_stub` não receberam quadros e isso foi
atribuído ao Wi-Fi. O HIL Ethernet de 24/08 localizou a causa: o bridge publica
o frame de 921600 bytes como `RELIABLE`, mas o stub pedia `BEST_EFFORT`; a
descoberta ocorria e todos os frames fragmentados eram perdidos. O stub agora
pede `RELIABLE` e recebe ~10 Hz. Portanto o ensaio antigo não prova que o Wi-Fi
era incapaz de carregar a câmera.

### Trocar o host para Ethernet

O módulo tem duas portas Ethernet na mesma `/24`, mas o kernel roteia os peers
dessa rede por `ethernet0` (métrica 101 contra 102). Use explicitamente
`ethernet0`; fixar DDS em `ethernet1` enquanto a rota sai por
`ethernet0` cria bind e envio divergentes. Não presuma que o hostname mDNS
identifica uma porta.

1. Antes de ligar o cabo, confirme que o PHY do host anuncia gigabit:

   ```bash
   ethtool enp0s31f6 | sed -n '/Supported link modes:/,/Advertised/p'
   ```

   `1000baseT/Full` precisa aparecer. Se o I219 anunciar apenas 10baseT depois
   de uma retomada de suspensão, recarregue `e1000e`; se ainda não aparecer,
   desligue o host de verdade — reboot morno preserva esse estado do PHY.

2. Ligue host e `ethernet0` do módulo no mesmo switch/roteador, nunca ponto a
   ponto para este portão. Confirme link e rota:

   ```bash
   ip -br addr show enp0s31f6
   ethtool enp0s31f6 | grep -E 'Speed:|Duplex:|Link detected:'
   ip route get <IP_ETHERNET0_AQUILA>   # tem de dizer "dev enp0s31f6"
   ```

   O portão exige `Speed: 1000Mb/s`, `Duplex: Full`, `Link detected: yes` e um
   `src` cabeado na mesma sub-rede. Se a rota usar Wi-Fi, não meça.

3. Passe os endereços explicitamente. Variável no ambiente vence
   `docker/.env`, inclusive quando vazia; isso é coberto por teste:

   ```bash
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh sync
   ```

4. **Re-renderize a configuração de DDS.** Este é o passo que se esquece:

   ```bash
   ./scripts/module.sh sync
   ```

   O `sync` deriva a interface de `ip route get`, então ele passa a fixar a
   Ethernet nos dois lados. Sem isso o `host.rendered.xml` continua fixando o
   Wi-Fi, o CycloneDDS transmite num endereço que o módulo não responde, e o
   sintoma é idêntico a firewall. O `run_quadruped_sim.sh` avisa quando os dois
   divergem — leia a linha `DDS:` na subida.

5. Reinicie simulador e containers do módulo e exija as três etapas do
   instrumento:

   ```bash
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh up
   MODULE_HOST=<IP_ETHERNET0_AQUILA> MODULE_IP=<IP_ETHERNET0_AQUILA> \
     HOST_IP=<IP_ETHERNET_DO_HOST> ./scripts/module.sh verify
   ```

   `verify` agora cria o assinante do host antes do heartbeat remoto, espera a
   descoberta unicast com prazo limitado e retorna falha se UDP, contrato de
   tópicos ou heartbeat não passarem.

6. Execute três corridas sem mudar carga, imagem ou parâmetros:

   ```bash
   python3 scripts/nav_trial.py docs/results/ml35-f5-ethernet0-run1.csv \
     --seconds 420 --goal-timeout 200 --sim-log <LOG_DA_SIMULACAO>
   # reinicie a planta no mesmo estado e repita como run2 e run3
   ```

   Cada CSV registra `sim_s` e `wall_s`; o resumo imprime o RTF calculado no
   mesmo intervalo. F5 fecha apenas se as três corridas concluírem ao menos uma
   meta de 8 m, sem queda e com enlace/RTF comparáveis.

Se a Ethernet não for possível, os outros caminhos são reduzir taxa ou resolução
em `demo_simulation/urdf/go2_sim.urdf.xacro` (muda o que a demo mostra) ou
`image_transport` comprimido. Detalhe em `docs/results/ml35-hil-aquila.md`.

### Switch isolado de exposição (sem roteador/DHCP)

Um switch Ethernet comum é somente camada 2: ele encaminha os quadros, mas não
atribui endereços nem fornece rota padrão. Portanto a demonstração funciona sem
roteador desde que host e Aquila recebam IPs estáticos na mesma sub-rede. A
configuração da bancada de exposição é:

| Equipamento | Interface | Endereço | Gateway |
| --- | --- | --- | --- |
| Host | `enp0s31f6` | `192.0.2.6/24` | nenhum |
| Aquila | porta `ethernet0` ligada ao switch | `192.0.2.5/24` | nenhum |

No host, o perfil NetworkManager persistente deve manter o Wi-Fi como rota
padrão e usar somente a rota conectada da Ethernet. Isto evita que desligar o
roteador externo derrube a comunicação host--Aquila:

```bash
nmcli connection modify ethernet \
  ipv4.method manual ipv4.addresses 192.0.2.6/24 \
  ipv4.gateway '' ipv4.never-default yes ipv4.route-metric 700 \
  ipv4.routes 192.0.2.5/32 ipv6.method disabled
nmcli connection up ethernet ifname enp0s31f6
```

Antes de subir a demo, este é o portão mínimo. `ip route get` tem de mencionar
`enp0s31f6`; se `ip neigh` ficar `FAILED`, o Aquila não está no switch nessa
sub-rede (cabo, porta, alimentação ou IP do módulo), e sincronizar DDS não
resolverá a ausência de conectividade L2.

```bash
ip -br -4 addr show enp0s31f6
ip route get 192.0.2.5
ping -c 2 -I enp0s31f6 192.0.2.5
```

Quando o Aquila responder, renderize novamente os peers unicast e valide o
contrato, sem depender de multicast nem do roteador:

```bash
./scripts/module.sh sync
./scripts/module.sh up
./scripts/module.sh verify
```

### 14. `use_composition` sem container é falha 100% silenciosa

`navigation_launch.py` com composição usa `LoadComposableNodes` para carregar os
servidores dentro de `/nav2_container`, mas **não cria** esse container — quem o
cria upstream é `bringup_launch.py`, que este projeto não inclui.

Ligar `use_composition: 'True'` sem criar o container carrega os nós num
container que ninguém criou: **nada sobe e nada imprime erro**. O log do launch
termina em `wait_for_clock` e `odom_tf` e para ali. O sintoma legível é "o Nav2
não ativou", que não aponta para este parâmetro.

O container está criado em `nav_quadruped.launch.py`, no bloco `nav2_container`,
com o nome casando o default do argumento `container_name` do launch vendorizado.
Se um dos dois nomes mudar, volta a falhar assim.

### 15. `down` sem `--profile` deixa o `nav` vivo com a imagem antiga

`nav` e `perception` estão atrás de `profiles: ["learn"]` no
`compose.host.yml`. Compose **ignora serviços com perfil** em qualquer comando
que não declare o perfil, e isso inclui o `down`:

```bash
docker compose -f compose.host.yml down --remove-orphans   # NAO derruba nav nem perception
docker compose -f compose.host.yml --profile learn down     # derruba
```

O `down` sem perfil imprime uma lista de containers removidos que **parece
completa** — ele lista o que removeu, nunca o que pulou. O `up -d sim` seguinte
também não recria o `nav`, porque para o Compose ele já está no estado desejado.
Resultado: o `nav` atravessa reconstruções de imagem indefinidamente rodando o
código de quando subiu.

O sintoma é o pior possível, porque não há sintoma: `docker compose ps` diz
`running`, os tópicos existem, e o comportamento é o de uma versão antiga do
código. Medido em 25/08/2026 — um `nav` de 10 horas antes sobreviveu a dois
ciclos de build e continuou construindo o `ReroutingService` que a correção
recém-compilada já não construía, o que fez a correção parecer não ter
funcionado.

Como conferir, quando um resultado não bate com o código:

```bash
# ID da imagem que o container esta rodando vs. ID da tag atual
docker inspect docker-nav-1 --format '{{.Image}}'
docker images --no-trunc --format '{{.Repository}}:{{.Tag}} {{.ID}}' | grep nav:dev
```

IDs diferentes = container obsoleto. É a mesma família da armadilha 4: ali o
`/ws/src` assado na imagem estava velho, aqui a imagem inteira está velha.

---

### 16. O builder multi-arch não enxerga as imagens locais, e o build "não existe"

O `CLAUDE.md` manda criar o builder multi-arch uma vez por host:

```bash
docker buildx create --use --name multiarch
```

O `--use` deixa esse builder **ativo para tudo**, e ele usa o driver
`docker-container`. Esse driver tem um store de imagens próprio e **não lê o
store local do Docker**. Consequência: qualquer imagem deste projeto cujo pai
seja outra imagem local para de construir, porque o builder tenta *puxar* o pai
do Docker Hub:

```text
failed to solve: local/demo-aquila-base:dev: failed to resolve source metadata
for docker.io/local/demo-aquila-base:dev: pull access denied, repository does
not exist or may require authorization
```

A mensagem fala em autorização e repositório inexistente, então ela **manda o
leitor procurar credencial, VPN ou registry** — e o problema não é nenhum dos
três. A imagem existe, na máquina, construída minutos antes.

A `base` engana porque **ela continua funcionando**: o pai dela é
`ros:jazzy-ros-base`, que de fato é puxável. Só `sim`, `nav`, `perception`,
`tools`, `viz`, `cockpit` e `hmi` quebram — o que faz parecer defeito dos
Dockerfiles dessas imagens.

Como conferir e como resolver:

```bash
docker buildx ls          # o builder com * e o ativo; driver docker-container e o problema

docker buildx use default                                   # driver `docker`, le o store local
docker compose -f compose.host.yml build base               # em SERIE: os filhos
docker compose -f compose.host.yml build sim                # precisam do pai ja construido
docker buildx use armbuilder                                # restaure, o multi-arch depende dele
```

Duas notas que economizam tempo:

- **`docker compose ... --builder default` não existe** nesta versão do Compose;
  ele responde `unknown flag: --builder`. Trocar o builder ativo é o caminho.
- **Construir `base sim perception` num comando só falha por corrida**, não pelo
  builder: o Compose dispara os três em paralelo e os filhos não encontram o pai
  que ainda está sendo construído. Em série sempre.

O build **nativo no módulo** não passa por nada disso: ele roda `docker build`
no próprio Aquila, com o daemon local.

---

### 17. `use_sim_time: true` custa CPU mesmo em nó que nunca olha o relógio

O sintoma: o módulo fica com carga alta, o `collision_monitor` recusa a nuvem do
LiDAR dizendo que a fonte está velha, e o robô para. Você olha o `top`, vê o
Nav2 no topo e conclui que Nav2 é caro. **Pode não ser ele.**

`use_sim_time: true` não é uma declaração de intenção. É o **rclpy** que cria uma
assinatura de `/clock` por nó, independentemente de o código do nó chamar o
relógio. No mundo do Go2 o Gazebo publica `/clock` a ~870 Hz, porque o passo de
física da marcha é 1 ms. Cada nó que assina paga por mensagem, useando-a ou não.

Medido em 26/08/2026 no AM69, pilha de pé e **sem meta ativa**: 367% de 800%, dos
quais 111% em três republicadores em Python que não têm uma única chamada a
`get_clock()`.

Como conferir, em vez de supor. `top` mostra o processo; o que você quer é a
thread, e num container composto o nome da thread é quem entrega o culpado:

```bash
# no módulo. Soma utime+stime por thread de todo o cgroup do container.
CID=$(docker inspect -f '{{.Id}}' demo-nav-1)
for p in $(cat /sys/fs/cgroup/system.slice/docker-$CID.scope/cgroup.procs); do
  for t in /proc/$p/task/*; do
    read -r comm < $t/comm
    set -- $(cat $t/stat); echo "${t##*/} $comm $(( $14 + $15 ))"
  done
done
```

Rode duas vezes com ~20 s de intervalo e tire a diferença: `(ticks2 - ticks1) /
100 / dt * 100` é a porcentagem de um núcleo. Um `component_container` espalhado
em ~30 threads a ~7% cada **não** é custo algorítmico — é entrega de mensagem.

E confirme de fora, sem inferir:

```bash
ros2 topic info /clock -v | grep 'Node name'   # quem realmente assina
```

**Antes de tirar `use_sim_time` de um nó, verifique que ele não lê o relógio:**

```bash
grep -n 'get_clock' <arquivo_do_no>.py
```

Se ele lê e você tira, o nó passa a ler **tempo de parede** achando que lê tempo
simulado. Os carimbos saem anos no futuro, o costmap descarta a leitura com
`message filter dropping message`, e nada nomeia a causa. `demo_bringup/test/`
`test_sim_time_scope.py` trava essa invariante nos dois sentidos.

**O que isto NÃO é:** estrangular `/clock` (`demo_simulation/clock_throttle.py`).
Aquilo baixa a taxa para **todo mundo**, o MPPI incluso, foi ensaiado em 21/08 e
**matou a navegação** (0,0039 m/s contra 0,0251). Aqui a taxa não muda para
ninguém; muda quem assina.

### 18. `docker/.env` tem endereço de bancada antigo, e `ssh` esconde isso

O sintoma: `scripts/module.sh sync` ou `build` falha com

```
[module.sh] ERRO: nao identifiquei a interface do modulo que carrega 192.0.2.5
```

enquanto `ssh`, `verify` e `status` funcionam normalmente.

A causa: a precedência é ambiente → `docker/.env` → defaults. O `.env` da bancada
carrega um `MODULE_IP` de outra rede, e ele **vence os defaults**. O `ssh` não
percebe porque usa `MODULE_HOST` (nome mDNS), não o IP — só os comandos que
precisam casar o IP com uma interface é que quebram.

Saída imediata:

```bash
MODULE_IP=<ip real> HOST_IP=<ip real> ./scripts/module.sh sync
```

Saída permanente: conserte o `.env`. Confira o valor real com
`getent hosts <MODULE_HOST>` e `ip route get <ip do módulo>`.

### 19. `/demo/sim/reset` num quadrúpede em marcha o derrubava

`/demo/sim/reset` (`std_srvs/Trigger`, servido pelo `sim_control_relay`)
teleporta o robô de volta à pose de nascimento do cenário. Chamado direto por
`ros2 service call`, sem o cockpit, num quadrúpede que está andando: até
26/08/2026 ele podia colapsar (a altura caía de 0,337 m para 0,162 m em 1 s) ou
arrastar-se girando por dezenas de metros perseguindo a orientação de antes do
reset — os dois defeitos são silenciosos, nada em log ou na resposta do serviço
acusava.

A causa e a correção estão detalhadas na armadilha 9 do cockpit,
[aqui](#9-o-reset-derrubava-o-quadrúpede-em-marcha) (`SetEntityPose` preserva
velocidade; `StateTrotting` reancora sua postura uma única vez). O reset agora para o gait antes de teleportar e o retoma depois —
por isso, num quadrúpede, a chamada leva alguns segundos a mais do que num
diff-drive antes de responder. Se o `Trigger` voltar `success=True` mas a
mensagem mencionar `NAO foi reancorado`, o robô ficou preso em pé sem aceitar
comando: chame `ros2 service call /demo/gait/resume std_srvs/srv/Trigger`
manualmente. Esse serviço só existe na planta quadrúpede — no diff-drive a
mensagem do reset diz `sem /demo/gait/hold (planta sem gait)`, e isso não é
falha.

Evidência: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

## 10. O módulo Aquila AM69

Tudo aqui é `arm64` sobre Torizon OS, e **nada gráfico** (regra 1: o AM69 expõe
só OpenGL ES 3.2 e Vulkan 1.2, então Gazebo e RViz2 ficam no host x86).

A interface é `scripts/module.sh`. Ele resolve os dois endereços em vez de
adivinhar: `MODULE_IP` por resolução do nome, `HOST_IP` pela **rota até o
módulo** — pegar o primeiro endereço da primeira interface UP escolheria
`docker0` ou `tailscale0` no workstation e produziria um peer que ninguém
alcança.

```bash
scripts/module.sh inventory   # OS, docker, disco, links — antes de qualquer coisa
scripts/module.sh sync        # fontes + config renderizada para ~/demo no módulo
scripts/module.sh build       # constrói as imagens arm64 NO módulo
scripts/module.sh verify      # 3 etapas: UDP, módulo vê host, host recebe módulo
scripts/module.sh up          # nav + perception (recusa se houver sim no host)
scripts/module.sh shell       # shell no container tools
```

### Por que o build roda no módulo e não sob QEMU

`CLAUDE.md` documenta `docker buildx --platform linux/arm64` a partir do host, e
esse caminho é o certo quando houver registry e manifest multi-arch. Para
bring-up não é:

- QEMU arm64 pode nem estar habilitado no workstation (`binfmt_misc` sem handler
  aarch64, `docker buildx ls` sem `linux/arm64` nas plataformas);
- o módulo tem 8 × Cortex-A72 e 31 GiB ociosos, e compila `ros2_control` e
  `unitree_guide_controller` em minutos, não horas;
- **o workstation é onde os ensaios de marcha rodam.** Eles medem *quando* o robô
  cai. Um build QEMU satura a CPU e corrompe a medição em vez de só atrasá-la.

O que se perde: imagem local só-arm64, sem manifest multi-arch. E nenhum dos
dois caminhos mede desempenho (regra 5).

### O que vai para o módulo e o que não vai

`sync` envia `ros2_ws/src`, `docker/entrypoint.sh`, `compose.module.yml` e
apenas os Dockerfiles de `base`, `nav`, `perception` e `tools`. **`sim/` e
`viz/` não são enviados** — são OGRE 2. A ausência deles no módulo é parte da
guarda, junto com a verificação de regra 1 que o `build` roda nas imagens
prontas (procurando `ogre|gz-rendering|gz-sim|gz-gui|rviz`, e não um `gz`
genérico: `gz-cmake/math/tools/utils-vendor` entram via `sdformat`, são CPU puro
e não violam nada).

### Configuração de DDS, renderizada e não commitada

Nenhum endereço entra no git. `sync` renderiza
`docker/cyclonedds/module.xml` — fixa a interface, injeta
`<Peer address="${HOST_IP}"/>` — e escreve em `~/demo/cyclonedds/module.xml`,
validando o XML no fim. O peer `127.0.0.1` continua lá e é *load-bearing*: com
`AllowMulticast=false`, `nav` e `perception` no módulo não se veem entre si sem
ele.

Se `ros2 topic list` vier vazio no módulo, confira nesta ordem:

```bash
ssh torizon@<módulo> 'grep -E "<Peer |<NetworkInterface " ~/demo/cyclonedds/module.xml'
ssh torizon@<módulo> 'cat ~/demo/.env'          # ROS_DOMAIN_ID igual nos dois lados?
scripts/module.sh verify                         # etapa 1 separa firewall de DDS
```

### O que ainda falta para o modo `hil` completo

- O portão funcional passa: TF fecha, Nav2 e percepção arm64 sobem no Aquila e
  o contrato atravessa a fronteira.
- O portão de estabilidade ainda depende do enlace físico gigabit e das três
  corridas de 420/200 s descritas acima. Nenhum número por Wi-Fi fecha F5.
- `odom_tf` ainda republica ground truth da simulação; portanto HIL não valida
  localização por pernas nem um Go2 físico.

---

---

# Parte II: Cockpit web

Como rodar, o que cada região da tela mostra, o que os botões fazem, e o que
não é possível fazer por eles.

Este documento é **operacional**. A evidência de bancada (medições, capturas,
falhas encontradas) está em [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md)
e [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md); as decisões de
projeto e as fases em [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md).

> **Nada aqui foi executado no Aquila AM69.** Tudo abaixo é o modo `learn`, com
> todos os containers na workstation x86. O kiosk no módulo é o F2 do plano e
> continua em aberto.

---

## 1. O que é o cockpit

Uma página web que mostra o robô e deixa comandá-lo, sem RViz e sem a janela do
Gazebo. Ela substituiu quatro tentativas de **incorporar janelas X11** numa
aplicação única — todas falharam, e o checkpoint delas
([`results/cockpit-standalone-parcial.md`](results/cockpit-standalone-parcial.md))
está marcado como superado. **Não retome aquele caminho.**

O eixo agora é: renderizar **a partir de tópicos ROS 2**. Isso muda o que é
possível. RViz e Gazebo são OGRE 2 e precisam de OpenGL de desktop, então nunca
poderiam ir para o Aquila (regra 1 do `CLAUDE.md` — o AM69 só expõe OpenGL ES
3.2 e Vulkan 1.2). Um navegador desenhando `nav_msgs/OccupancyGrid` num
`<canvas>`, não. É por isso que **este cockpit vira o HMI do módulo no M3**, e a
tentativa anterior não viraria.

Duas peças:

| Serviço | O que é | Onde roda hoje | Onde roda no M3 |
| --- | --- | --- | --- |
| `cockpit` | `rosbridge_server` + `web_video_server` | host | Aquila |
| `hmi` | nginx servindo o bundle de `hmi/` | host | Aquila |

O bundle é HTML/CSS/ES modules **sem etapa de build** — sem npm, sem bundler,
sem `node_modules`. Editar um arquivo e recarregar a página é o ciclo inteiro.
Isso é decisão de projeto (Decisão 5 do plano): a cadeia npm é justamente o que
não deve viajar para o módulo.

---

## 2. Rodar

### Pré-requisitos

Uma vez por máquina:

```bash
cd docker
cp .env.example .env      # e edite MAZE_MODELS, se for usar o labirinto
```

### Subir

```bash
cd docker
docker compose -f compose.host.yml --profile learn up -d --build
```

Isso sobe cinco serviços: `sim` (Gazebo + robô + pontes), `nav` (Nav2),
`perception` (stub de detecção), `cockpit` (rosbridge + vídeo) e `hmi` (nginx).

**Abra <http://localhost:8081>.**

O Nav2 leva ~30 s para ficar pronto. A página pode ser aberta antes: os painéis
sobem vazios e se preenchem sozinhos, e o ponto de frescor de cada um diz se
aquele tópico já chegou. É de propósito — um cockpit que trava até tudo estar
pronto não mostra o que está faltando.

### Portas

Todos os containers usam `network_mode: host`, então **nada aqui é mapeamento de
porta**: os números abaixo abrem direto na workstation. Se colidirem com algo
que você já roda (8080 é popular), troque em `docker/.env`.

| Variável | Padrão | Serve |
| --- | --- | --- |
| `COCKPIT_HMI_PORT` | 8081 | a página — **é esta que você abre** |
| `COCKPIT_ROSBRIDGE_PORT` | 9090 | WebSocket que o navegador consome |
| `COCKPIT_VIDEO_PORT` | 8080 | MJPEG das câmeras |

### Derrubar

```bash
docker compose -f compose.host.yml --profile learn down
```

### Editar o bundle

Mudança em `hmi/` (HTML, CSS, JS) não precisa de `colcon`, mas precisa de
rebuild da imagem, porque o nginx serve uma cópia:

```bash
cd <raiz do repo>
DOCKER_BUILDKIT=0 docker build -f docker/hmi/Dockerfile -t local/demo-aquila-hmi:dev .
cd docker && docker compose -f compose.host.yml up -d --no-build --force-recreate hmi
```

Antes de recarregar, rode os testes do bundle — eles não precisam de ROS nem de
container:

```bash
cd hmi && node --test "test/**/*.test.js"
```

As **aspas no glob são obrigatórias**: sem elas o shell expande, o `node --test`
recebe um diretório, resolve para nada e sai com código 0 — verde sem ter
rodado teste nenhum.

---

## 3. A tela

Cinco regiões, na disposição de [`ml35/cockpit-division-view.png`](ml35/cockpit-division-view.png):

```
+---------------------------+--------+
|          CENA             |  NAV   |
+---------------------------+--------+
|          LOGS             | CÂMERA |
+---------------------------+--------+
|          BARRA DE CONTROLE         |
+------------------------------------+
```

| Região | Fonte | Responde a que pergunta |
| --- | --- | --- |
| **Cena** | `/demo/cockpit/scene_{iso,top}/image_raw` | o robô se mexeu? |
| **Navegação** | `/global_costmap/costmap`, `/plan`, `/demo/scan`, TF | ele sabe para onde ir? |
| **Logs** | `/rosout` + telemetria de `/demo/cmd_vel` e `/demo/odom` | o que ele está tentando fazer? |
| **Câmera** | `/demo/camera/image_raw` | o que ele está vendo? |
| **Barra** | estado do WebSocket | o cockpit ainda está falando com o robô? |

### O painel de Cena é o mais importante e é o menos óbvio

São **duas câmeras estáticas do mundo** — não do robô — alternadas pelos botões
`iso` / `topo` no cabeçalho. Elas existem porque respondem "o robô se mexeu?"
sem depender de nenhum nó do stack: se o Nav2, a TF e a odometria mentirem
juntos, esta imagem continua honesta. É a testemunha independente da tela.

### O ponto de frescor

Cada painel tem uma bolinha no canto do cabeçalho, com três estados que **não
são a mesma coisa**:

| Cor | Estado | Significa |
| --- | --- | --- |
| cinza | `never` | esse tópico nunca produziu nada desde que a página abriu |
| verde | `live` | chegou amostra dentro da janela daquele painel |
| laranja | `stale` | chegou antes e **parou** |

`stale` é o que pega um simulador morto, e por isso não pode parecer `never`.

### Cores

A paleta é a identidade Toradex: fundo branco, azul `#00508c`, verde `#96c837`,
laranja `#ff5a00`. O **vermelho de falha (`#c0261b`) não é da marca, de
propósito**: "parado há tempo demais" e "morto" precisam parecer coisas
diferentes através da sala, e usar laranja nos dois faria o pior caso
desaparecer dentro do caso comum.

No mapa, a mesma lógica: azul é o plano global, verde é o robô, laranja é
obstáculo, e a meta é violeta — uma cor que **não** é da marca, justamente para
não poder ser confundida com um estado.

Cor de canvas não existe em JavaScript. `hmi/js/panels/palette.js` lê os tokens
`--map-*` de `hmi/css/tokens.css` uma vez, na montagem. Mudar tema é mexer num
arquivo.

---

## 4. Os controles

### Clicar no mapa manda meta

Clique no painel verde. Vai um `NavigateToPose` para o Nav2, e o HUD no rodapé
do painel passa a mostrar o estado (`navegando · 5,04 m restantes`), incluindo a
contagem de recuperações. Um botão **cancelar meta** aparece no cabeçalho
enquanto há meta ativa.

Clique perto demais do robô (< 0,25 m) é tratado como engano e ignorado.

**Durante uma busca autônoma o clique não manda nada** — veja a seção seguinte.

### Busca autônoma: iniciar e cancelar

Dois botões no cabeçalho do painel verde, `iniciar busca` e `cancelar busca`.
Chamam `/demo/exploration/{start,cancel}` (`std_srvs/Trigger`), servidos pelo
`maze_explorer`, que roda **junto com o Nav2** — no Aquila, no modo `hil`.

O que ele faz sem nenhuma ajuda: parte sem mapa prévio, extrai fronteiras do
mapa vivo do `slam_toolbox`, navega até a melhor delas, e quando a percepção
reconhece o painel magenta da saída, cancela a fronteira e se aproxima em passos
de 0,5 m. Ele **não** conhece o labirinto: não há waypoint, não há coordenada da
saída, e há teste garantindo que o código não menciona nenhuma.

O HUD passa a mostrar o estado da busca no lugar do estado da meta:

| Estado | Significa |
|---|---|
| `waiting_map` | esperando mapa, TF e os servidores do Nav2 |
| `selecting` | avaliando candidatos com o planner `ExplorationGrid` |
| `navigating` | indo para uma fronteira |
| `homing_exit` | vendo o marcador e se aproximando dele |
| `completed` | chegou ao marcador |
| `failed` | prazo total (600 s) ou falha declarada |
| `cancelled` | o operador parou |

Junto vão o tempo decorrido, quantas fronteiras existem, `saída detectada`
quando a percepção está vendo o painel, e a última mensagem de falha.

Três coisas que parecem detalhe e não são:

- **O clique no mapa fica desabilitado enquanto a busca corre.** Duas fontes de
  meta no mesmo `navigate_to_pose` se preemptam sem erro nenhum no log — é a
  mesma classe de falha dos dois publicadores em `/demo/cmd_vel`.
- **`iniciar busca` recusa a segunda chamada** enquanto uma está em andamento, e
  a recusa aparece no HUD.
- **`reiniciar nav` cancela a busca antes** de ciclar a pilha. Sem isso o
  explorador mandaria uma meta nova no meio do ciclo de reset.

### `SAÍDA CONFIRMADA` não vem do explorador

O rótulo só aparece quando `/demo/maze/escaped` é `true`, e quem publica isso é
o `maze_escape_validator`, **do lado da simulação**, olhando a odometria ground
truth: ele exige o cruzamento da abertura *e* o robô inteiro do lado de fora.

O explorador não assina esse tópico. Ele é o árbitro da demonstração, não uma
entrada dela — `state: completed` diz apenas que o robô chegou perto do painel,
o que não é a mesma coisa que ter saído.

### Simulação: ▶ ⏸ ⟲

Na barra. Chamam `/demo/sim/{play,pause,reset}` (`std_srvs/Trigger`).

O **reset exige dois cliques**: o primeiro arma o botão (ele vira `confirmar` em
laranja por 4 s), o segundo executa. Ele devolve o robô à pose inicial. **Não**
apaga o costmap — quem limpa o costmap é [reiniciar a navegação](#reiniciar-a-navegação),
botão separado, de propósito: repor o robô sem derrubar o que o Nav2 já sabe do
mundo é o que permite os dois em sequência sem perder trabalho.

No cenário quadrúpede o clique leva alguns segundos a mais do que no
diff-drive, e isso é esperado: o robô é **parado** antes de ser reposicionado e
só volta a andar depois — ver a [armadilha 9](#9-o-reset-derrubava-o-quadrúpede-em-marcha).

O rótulo ao lado (`rodando` / `pausado` / `sem simulador`) **não é o eco do
último clique**: ele vem do `/clock`. Um eco mentiria em todos os casos que
importam — container `sim` morto, chamada expirada, pausa feita pela GUI do
Gazebo, mundo resetado por outra pessoa. Se o botão diz uma coisa e o rótulo
diz outra, **o rótulo está certo**.

A diferença entre `pausado` e `sem simulador` é a que custa: amostras chegando
com o mesmo tempo simulado é pausa; amostras **parando** é ausência.

### Câmera de cena: o pad sobre a imagem

Canto inferior direito do painel azul. Três fileiras:

```
 ↺  ▲  ▼  ↻      girar e inclinar
 ◀  △  ▽  ▶      mover
 +  −  recentrar  aproximar / afastar / voltar ao enquadramento inicial
 seguir robô      liga/desliga o acompanhamento (vale para as DUAS vistas)
```

Segurar o botão repete. Os comandos agem sobre **a câmera que está na tela** —
trocar entre `iso` e `topo` troca o alvo junto. O `seguir robô` é a exceção: ele
vale para as duas de uma vez, porque o que ele muda é o **alvo da órbita**, e não
há versão disso que faça sentido para uma câmera só.

O navegador publica **deltas** em `/demo/cockpit/scene/cmd_view`
(`geometry_msgs/TwistStamped`, com o `header.frame_id` escolhendo a câmera). Quem
guarda a órbita, satura os limites e escreve a pose no Gazebo é o nó
`scene_view_controller`. Por isso recarregar a página **não** mexe no
enquadramento, e dois cockpits abertos não brigam pela pose.

### Seguir o robô

Ligado por default. As duas câmeras acompanham `/demo/odom` a 10 Hz, e o que se
move é só o **alvo** da órbita — azimute, elevação e distância ficam onde
estavam. Consequência prática: o enquadramento medido do armazém (a diagonal de
`(-3, +3, 2,4)` na iso, os 6 m de altura na topo) continua valendo enquanto o
robô caminha, em vez de o robô sair do quadro em quinze segundos.

Os botões de mover (`◀ △ ▽ ▶`) continuam úteis com o seguimento ligado: com ele
ligado o pan vira um **deslocamento relativo ao robô**, saturado em 15 m, e não
um ponto fixo do mundo. Serve para olhar o robô de lado, ou um pouco à frente
dele, sem perder o acompanhamento. `recentrar` zera esse deslocamento junto com
o resto.

O estado do botão vem de `/demo/cockpit/scene/following`
(`std_msgs/Bool`, latched), publicado pelo nó, e **não do próprio clique** — o
mesmo raciocínio do rótulo de simulação, e pelos mesmos três casos: F5, segundo
cockpit aberto, e alguém que desligou o seguimento por `ros2 service call`.

Desligar (`follow:=false` no launch, ou o botão) devolve a vista larga do
cenário, que é o que se quer para conferir o mundo inteiro ou para comparar com
um enquadramento anterior.

### Reiniciar a navegação

**reiniciar nav**, no cabeçalho do painel verde. Chama `/demo/nav/reset`
(`std_srvs/Trigger`), servido pelo `nav_control_relay` **dentro do container que
roda o Nav2** — no `hil` isso é o Aquila.

Como o reset da simulação, **exige dois cliques** (o primeiro arma por 4 s). Não
por simetria: ele descarta a meta em andamento, e um clique por engano no meio de
uma demo custa a demo.

A sequência, na ordem, e cada passo existe:

1. `CancelGoal` em `/navigate_to_pose` com `goal_info` zerado — cancela **todas**
   as metas, inclusive a que o cockpit não sabe que existe (mandada pelo
   `patrol_commander`, ou por outra aba);
2. `ClearEntireCostmap` no global e no local, **com os servidores ainda ativos**
   — um nó desativado não responde serviço, então limpar depois de pausar não
   limparia nada e não daria erro;
3. `PAUSE` no `lifecycle_manager_navigation`;
4. `RESUME`.

Medido em 6,6 s no host. A meta interrompida termina `CANCELED`, os servidores
voltam `active [3]`, e uma meta nova é aceita em seguida.

O HUD do painel mostra `reiniciando…` durante a sequência e `reiniciado` no fim.
Sem timeout do lado do navegador: quem tem o timeout é o nó (60 s por transição),
e fechar a aba no meio **não** interrompe o reset.

**Por que não `RESET` + `STARTUP`**, que é o caminho óbvio: ele derruba o
container. Ver a armadilha 8.

A localização **não** é tocada. `lifecycle_manager_localization` fica fora da
sequência de propósito: no caminho de mapa estático, reciclar o AMCL joga a pose
fora e o robô "se perde" num reset que era só para descartar a meta.

### Controle manual: desligado de propósito

As setas e o E-STOP da barra estão desenhados, alcançáveis por teclado, e
**inertes**, com o motivo no `title`. Hoje o Nav2 é o único publicador em
`/demo/cmd_vel`; um botão de teleop publicando ali daria dois escritores não
arbitrados no mesmo tópico — o último a escrever ganha, e nenhum dos dois sabe
que perdeu. Fecha no **F4**, com `twist_mux`, não com um mux escrito à mão no
navegador.

---

## 5. O que NÃO dá para fazer

### Rodar a simulação no Aquila

Não é limitação de implementação, é a regra 1 do `CLAUDE.md`: o Gazebo é OGRE 2
e precisa de OpenGL de desktop; o AM69 expõe apenas OpenGL ES 3.2 e Vulkan 1.2.
**Nenhuma quantidade de código na UI muda isso.**

O que existe é **controlar a simulação a partir do cockpit**. No M3, com o
cockpit servido pelo Aquila, o clique sai do módulo e a chamada de serviço
atravessa o grafo ROS — exatamente como a meta do Nav2 já atravessa hoje. O
processo do simulador continua na workstation x86.

Se a distinção parecer sutil na hora da demo: o que está na tela do módulo é
uma **imagem** vinda do host, e um **botão** que fala com o host.

### Chamar serviço com tipo do Gazebo pelo navegador

Também não é limitação temporária. O `rosbridge` monta o pedido importando o
pacote de interfaces **dentro do próprio container**, e o container `cockpit`
não tem `ros_gz_interfaces` — nem deve ter, porque no M3 ele roda no Aquila e no
modo `deploy` não existe Gazebo nenhum. Ver a armadilha 2 na
[seção 9](#9-as-armadilhas-do-cockpit-que-já-custaram-tempo).

---

## 6. Trocar de cenário

O mundo padrão é o armazém do `nav2_minimal_tb4_sim`. Para o labirinto de 11,6 ×
11,6 m, duas variáveis de ambiente (em `docker/.env` ou exportadas antes do
`up`):

```bash
export MAZE_MODELS=/caminho/para/ros_maze_worlds/models

export SIM_ARGS="world:=/ws/install/demo_simulation/share/demo_simulation/worlds/quadruped_maze11.sdf \
  yaw:=1.5708 \
  scene_top_x:=-4.855 scene_top_y:=4.855 scene_top_z:=13.0 \
  scene_iso_x:=-13.0 scene_iso_y:=-3.0 scene_iso_z:=9.0 \
  scene_iso_pitch:=0.6717 scene_iso_yaw:=0.7676"

docker compose -f compose.host.yml up -d --force-recreate sim
```

O que cada grupo faz:

- **`MAZE_MODELS`** — o `sim` monta esse diretório em `/maze/models` e aponta
  `GZ_SIM_RESOURCE_PATH` para lá. **Sem ele o mundo carrega e o labirinto
  simplesmente não está lá** — sem erro nenhum. Os modelos vêm de
  `github.com/cafemesa/ros_maze_worlds` e nenhum asset externo é copiado para o
  repositório.
- **`yaw:=1.5708`** — o canto de partida do labirinto tem parede no `+x`
  default; sem girar, o robô nasce de cara na parede.
- **`scene_*`** — enquadramento das duas câmeras de cena. Os valores acima foram
  **medidos**, não escolhidos: são os que pegam o labirinto inteiro
  (`scene_top`) e uma diagonal legível (`scene_iso`).

`SIM_ARGS` é livre e passa direto para `sim.launch.py`. É onde mora tudo que é
do **cenário** e não do **modo**. Vazio = armazém.

### Por que as câmeras de cena são modelos e não estão nos mundos

Duas razões que só aparecem depois:

1. o mundo default é de terceiros e **não é editável** — e é ele que sobe quando
   ninguém passa `world:=`;
2. pendurar a câmera no `go2_description` vendorizado quebraria a **garantia
   byte a byte** que sustenta o argumento de licença.

Por isso elas são modelos spawnáveis
(`demo_simulation/models/cockpit_scene_{iso,top}.sdf`), plantados em qualquer
mundo por `scene_cameras.launch.py`.

---

## 7. Ajustar qualidade de imagem

Padrão atual: **1600 × 1200 a 10 Hz**, anti-aliasing 8, JPEG de qualidade 95.

**A taxa é 10 e não 15 por medição.** As duas câmeras renderizam no mesmo
processo do Gazebo, e nesta resolução a workstation não entrega 15 Hz de
qualquer forma. Medido na bancada, mesmo mundo (`maze11`), janela de 10 s:

| `update_rate` | entregue no tópico | fator de tempo real |
| --- | --- | --- |
| 15 | 9,43 Hz | **0,59** |
| 10 | 9,77 Hz | **0,97** |

Pedir 15 não rendia um quadro a mais e custava 40% da velocidade da simulação —
o que estica cada meta do Nav2 na mesma proporção.

**Se precisar economizar, mexa nesta ordem:**

1. **qualidade JPEG**, em `hmi/js/config.js` (`STREAM_QUALITY`). Degrada
   suavemente e não muda nada do lado do ROS. É o primeiro lugar a mexer se o
   gargalo for **banda** — modo `hil`, stream atravessando a Ethernet até o
   Aquila.
2. **`update_rate`** nos dois SDF, se o gargalo for **render**.
3. **resolução**, por último. Ela desloca o enquadramento em pixels; e a
   **proporção 4:3 não pode mudar** — o `horizontal_fov` e as poses das duas
   câmeras foram medidos nela, e ir para 16:9 mantendo o hfov corta vertical e
   desenquadra as duas cenas de uma vez, sem erro nenhum.

Qualquer mudança nos SDF exige rebuild de `base` **e** de `sim` (o workspace é
compilado em `base`), e o cache do Docker mente aqui — ver armadilha 4.

### Para medir o fator de tempo real

```bash
docker compose -f compose.host.yml exec -T sim bash -lc 'source /ws/install/setup.bash
  ros2 topic echo --once /clock; sleep 10; ros2 topic echo --once /clock'
```

Divida o avanço do tempo simulado pelos 10 s de parede.

---

## 8. Diagnóstico do cockpit

O cockpit foi feito para responder sozinho "esse tópico está chegando?" — é para
isso que o nome do tópico está impresso em cada cabeçalho e que existe o ponto de
frescor. Quando não bastar:

```bash
cd docker

# A página existe?
curl -o /dev/null -w '%{http_code}\n' localhost:8081/

# O rosbridge subiu?
docker compose -f compose.host.yml logs cockpit | grep -i rosbridge

# As câmeras de cena estão publicando?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 topic hz /demo/cockpit/scene_iso/image_raw'

# O MJPEG responde?
curl -o /dev/null -w '%{http_code}\n' \
  'localhost:8080/snapshot?topic=/demo/cockpit/scene_iso/image_raw'

# Os serviços de simulação existem?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 service list | grep /demo/sim'
```

**O console do navegador é fonte de primeira classe aqui.** Falhas de chamada de
serviço aparecem lá (`[cockpit] falha ao pausar a simulação: ...`) e, com mais
detalhe, no log do container `cockpit`.

### Sintomas comuns do cockpit

| Sintoma | Causa provável |
| --- | --- |
| Painel verde vazio, HUD diz `sem TF map→base` | armadilha 1 |
| Botão de simulação não faz nada | armadilha 2 — olhe o log do `cockpit` |
| Painel azul preto, ponto cinza | mundo sem as câmeras: `sim` recriado sem `SIM_ARGS`? |
| Labirinto não aparece, chão vazio | `MAZE_MODELS` não exportado |
| Editei o bundle e nada mudou | a imagem `hmi` não foi reconstruída |
| Editei um SDF e nada mudou | armadilha 4 |
| Tudo cinza, badge `desconectado` | `cockpit` caiu, ou a porta 9090 colidiu |

---

## 9. As armadilhas do cockpit que já custaram tempo

### 1. O `/tf_static` chega **uma vez só**, e qual mensagem é sorte

Sintoma: o painel verde ficava em `sem TF map→base` em cerca de metade dos
carregamentos.

O `rosbridge` entrega **uma** mensagem latched por inscrição. Medido em três
inscrições novas e consecutivas: a primeira trouxe as arestas do robô, a segunda
`map→odom`, a terceira `map→odom`. `queue_length: 16` não muda nada — a perda é
**acima** da fila do cliente.

Correção, já no código: `nav-panel.js` reinscreve em `/tf_static` a cada 1,5 s,
no máximo 8 vezes, e para em definitivo assim que `lookup('map','base')`
resolve. Converge em 2 a 4 rodadas.

### 2. O navegador não pode chamar serviço com tipo do Gazebo

Sintoma: os botões de simulação não faziam absolutamente nada. A UI não
mostrava erro; a causa só aparecia no log do container `cockpit`:

```
call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
from package ros_gz_interfaces. Caused by: No module named 'ros_gz_interfaces'
```

O `rosbridge` monta o pedido importando o pacote de interfaces **dentro do
próprio container**, e o `cockpit` não tem `ros_gz_interfaces`. E não deve ter:
no M3 ele roda no Aquila, e no modo `deploy` não há Gazebo nenhum.

Correção: o nó **`sim_control_relay`** (lado do simulador) expõe
`/demo/sim/{play,pause,reset}` como `std_srvs/Trigger` e traduz para
`ControlWorld`. A fronteira do navegador só fala tipos de núcleo do ROS.
Guardado por `tests/test_cockpit_web_contract.py::test_browser_never_speaks_gazebo_interfaces`.

### 3. O nome do mundo não é o nome do arquivo

Os serviços do Gazebo moram em `/world/<nome>/...`, e `<nome>` é o atributo do
elemento `<world>`. `quadruped_maze11.sdf` declara `<world name="quadruped_maze11">`,
mas o armazém do `nav2_minimal_tb4_sim` declara `<world name='warehouse'>`.
Adivinhar pelo nome do arquivo acerta num caso e erra no outro, **em silêncio**:
a ponte sobe, anuncia os serviços ROS, e cada chamada expira num serviço gz que
não existe.

Por isso `sim_control.launch.py` faz parse do SDF, e um arquivo sem `<world>`
derruba o launch dizendo qual arquivo era.

### 4. O cache do Docker mente sobre `COPY ros2_ws/src`

Sintoma: você edita um SDF ou um `.py` do workspace, reconstrói, e a mudança não
está no container. O build reporta `CACHED` para a camada de `COPY`.

Aconteceu três vezes em uma sessão. **Contorno: rodar `docker compose build base`
duas vezes** — a segunda pega. E o workspace é compilado em `base`, então
qualquer mudança em `ros2_ws/` exige `base` **e depois** `sim`:

```bash
export DOCKER_BUILDKIT=0
docker compose -f compose.host.yml build base
docker compose -f compose.host.yml build base   # sim, de novo
docker compose -f compose.host.yml build sim
```

Sempre confirme antes de concluir que a mudança não funcionou:

```bash
docker compose -f compose.host.yml exec -T sim \
  grep update_rate /ws/install/demo_simulation/share/demo_simulation/models/cockpit_scene_iso.sdf
```

**A variante pior é rodar a suíte de testes de um pacote dentro do container.**
`/ws/src` é uma cópia da imagem, não um bind mount do seu diretório de trabalho:
uma imagem de antes da sua edição roda os testes **antigos** e passa. Um teste que
você acabou de escrever simplesmente não é coletado, e a saída é verde. Aconteceu
em 25/08: `32 passed` na imagem reconstruída contra `22 passed` na anterior, com o
arquivo novo ausente da lista de coleta. Confira a contagem, ou confira a
coleta:

```bash
docker compose -f compose.host.yml run --rm -T tools \
  bash -lc 'ls /ws/src/demo_simulation/test/'
```

### 5. Buildx não enxerga imagens locais

`docker compose build` com o builder `armbuilder` ativo falha com
`pull access denied ... local/demo-aquila-base:dev`. O builder multi-arquitetura
não vê o daemon local. Para builds de bancada:

```bash
export DOCKER_BUILDKIT=0 BUILDX_BUILDER=default
```

### 6. `node --test` com glob sem aspas passa sem rodar nada

```bash
node --test "test/**/*.test.js"    # certo
node --test test/**/*.test.js      # verde sem ter rodado teste
```

Sem aspas o shell expande, o Node recebe um diretório, resolve para nada e sai
com código 0.

### 7. Enquadramento de câmera se mede, não se chuta

A primeira tentativa (iso em `-7,-7,5`) caía **dentro** dos corredores de
prateleira do armazém — o robô virava um ponto branco atrás de uma prateleira. A
de topo a 12 m batia numa viga do telhado exatamente sobre o robô. Os dois
enquadramentos atuais saíram de tentativas medidas contra a pegada real do
cenário.

Ao testar comando de órbita pela linha de comando, use `ros2 topic pub -t 1 -w 1`
e não `-r 3`: seis segundos a 3 Hz aplicam ~18 passos de 0,35 rad ≈ 2π, a câmera
volta ao ponto de partida, e parece que nada aconteceu.

### 8. `RESET` + `STARTUP` no Nav2 mata o container (segfault no `route_server`)

Este era o desenho natural do "reiniciar nav": o `lifecycle_manager` do Nav2 tem
`RESET` (desativa e desconfigura tudo) e `STARTUP` (configura e ativa tudo), e
nenhuma outra dupla de transições descreve tão bem "reinicie a pilha".

Medido em 24/08/2026, no host, modo `learn`, o `nav2_container` morre:

```
[component_container_isolated-4] [INFO] [route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [exit code -11]
```

`-11` é `SIGSEGV`. Reproduzido **duas** vezes — com meta ativa e sem meta ativa.
Não é o "aconteceu uma vez" que este guia registrava antes: é determinístico, e
está no caminho do `CONFIGURE`, que é justamente o que `STARTUP` faz.

O `route_server` está na lista de `lifecycle_nodes` do `navigation_launch.py`
vendorizado — que tem de seguir **idêntico ao upstream** (ver
`launch/nav2_vendored/README.md`), então tirá-lo da lista não é opção. Esta demo
não usa roteamento, e ele não tem seção em `nav2_params_go2.yaml`; a hipótese é
que ele reconfigure sobre estado que não sobrevive ao `CLEANUP`, mas isso não foi
confirmado no fonte do Nav2. **Candidato a issue upstream.**

O que o `nav_control_relay` faz em vez disso — cancelar, limpar costmaps,
`PAUSE`, `RESUME` — nunca passa por `CONFIGURE`, e por isso nunca chega perto
disso. Se algum dia alguém "simplificar" a sequência para `RESET`+`STARTUP`, o
sintoma será o container `nav` reiniciando e o cockpit perdendo o link no meio da
demo. Existe um guarda estrutural em `tests/` exatamente para isso.

#### Não é só no reset: acontece no boot normal (25/08/2026)

A frase acima, "está no caminho do `CONFIGURE`", estava certa e era estreita
demais. O `CONFIGURE` do `route_server` também acontece na **subida normal** do
`nav`, e lá o mesmo segfault aparece — de forma **intermitente**: o mesmo
`docker compose up nav` subiu numa vez e derrubou a pilha na seguinte.

O estado que ele deixa é o que faz isso caro:

```
$ docker compose ps
nav   running                       <- e mentira

$ docker exec docker-nav-1 ps -eo comm
ros2                                <- so o pai
odom_tf
nav_control_rel
cmd_vel_si_to_s                     <- nenhum servidor do Nav2
```

Morrem **todos** os servidores de uma vez, o container segue `running`, e de
dentro dele `get_node_names()` lista só os nós do `sim`. Quem olha o Compose
conclui "o Nav2 está de pé"; quem olha o robô conclui "a navegação piorou". Foi
metade da regressão investigada em `docs/results/ml35-regressao-navegacao.md`.

A correção vive em `nav2_params_go2.yaml`: `route_server.operations` lista só
`AdjustSpeedLimit`, então o plugin que estoura nunca é construído. Sobrepor essa
lista obriga a declarar também o **tipo** de cada plugin dela
(`AdjustSpeedLimit.plugin`), senão a subida é reprovada com
`Can not get 'plugin' param value` — falha alta, e nisso melhor que o segfault.

### 9. O reset derrubava o quadrúpede em marcha

O reset da simulação teleporta o robô de volta à pose inicial via
`SetEntityPose` (ver armadilha 2 sobre por que não é uma chamada direta do
Gazebo pelo navegador). Isso resolve um defeito pior — `reset.all` **apagava**
o robô inteiro, ver `docs/results/cockpit-reset-nao-destrutivo.md` §1-2 — mas
sozinho não bastava para o quadrúpede, e por dois motivos diferentes,
descobertos em sequência:

1. **teleportar sem reancorar o controlador de marcha.** O `StateTrotting`
   (controlador C++ do gait) captura sua referência de postura uma única vez, e
   um teleporte muda a pose sem passar por essa captura. Medido sem nenhum
   comando de velocidade publicado por 26 s: mesmo assim o robô se arrastou
   0,87 m e girou 135° sozinho, perseguindo a pose de ANTES do reset;
2. **teleportar sem parar.** `SetEntityPose` reposiciona o corpo e **preserva a
   velocidade**. Um robô em marcha, teleportado, é solto ainda viajando com as
   pernas em balanço — e cai. Medido com o Nav2 conduzindo de verdade durante o
   reset: a altura do robô caiu de 0,337 m para 0,162 m em 1 segundo.

A correção para o robô ANTES de teleportar e o reancora DEPOIS — dois serviços
internos (`/demo/gait/hold`, `/demo/gait/resume`), servidos pelo mesmo nó que já
traduzia `/demo/cmd_vel` para os eixos do gait. Nenhum dos dois é exposto ao
cockpit; o operador só vê o efeito, que é o clique de reset levar ~2-7 s a mais
no quadrúpede do que no diff-drive. Verificado inclusive com o robô **caído**
(tombado, preso no modo de recuperação do controlador — que não sai sozinho de
cabeça para baixo): o reset o devolve de pé.

Evidência completa: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

---

## 10. Como o cockpit é feito

### Árvore

```
hmi/
├── index.html            as cinco regiões
├── css/
│   ├── tokens.css        paleta, tipografia, movimento — fonte ÚNICA de cor
│   ├── layout.css        a grade de cinco regiões
│   └── panels.css        cromo dos painéis, barra, pad de câmera
├── img/                  marcas Toradex e ROS (PNG branco com alfa)
├── js/
│   ├── main.js           só fiação: resolve config, monta painéis, um timer
│   ├── config.js         endpoints, tópicos, URL do MJPEG
│   ├── ros/              transporte
│   │   ├── rosbridge-client.js   WebSocket, reconexão, ações, serviços
│   │   ├── png-decompress.js     costmap comprimido (33x menor que JSON)
│   │   ├── tf-tree.js            cache de TF
│   │   └── freshness.js          os pontos verde/laranja/cinza
│   ├── panels/           renderização
│   │   ├── stream-panel.js       painéis de imagem (cena e câmera)
│   │   ├── nav-panel.js          canvas do mapa, clique-para-meta
│   │   ├── map-view.js           mundo <-> tela, LUT de custo (puro, testado)
│   │   ├── palette.js            lê os tokens --map-* do CSS
│   │   ├── log-panel.js          /rosout + telemetria
│   │   ├── control-bar.js        estado do link
│   │   ├── sim-controls.js       play/pause/reset
│   │   ├── view-controls.js      pad de câmera
│   │   └── detection-overlay.js  NÃO montado hoje — ver abaixo
│   └── ...
└── test/                 node --test, sem navegador
```

### Do lado do ROS

| Arquivo | Papel | Roda em |
| --- | --- | --- |
| `demo_bringup/launch/cockpit.launch.py` | rosbridge + web_video_server | host hoje, Aquila no M3 |
| `demo_simulation/launch/scene_cameras.launch.py` | planta as duas câmeras de cena | host |
| `demo_simulation/launch/sim_control.launch.py` | ponte de serviços gz + fachada | host |
| `demo_simulation/scene_view_controller.py` | órbita das câmeras de cena | host |
| `demo_simulation/sim_control_relay.py` | fachada `std_srvs` para play/pause/reset | host |
| `demo_navigation/launch/nav_control.launch.py` | sobe a fachada de reset do Nav2 | host ou Aquila |
| `demo_navigation/nav_control_relay.py` | fachada `std_srvs` para reiniciar o Nav2 | onde o Nav2 roda |

As duas últimas linhas são o único par desta tabela que **roda no módulo** no
modo `hil`: elas moram no container `nav`, junto da pilha que reiniciam. A
fachada de simulação fica presa ao host porque o Gazebo fica.

### Um detalhe que parece bug e não é

O painel da câmera **não desenha as caixas de detecção**, e a ausência é
deliberada. O `demo_perception` de hoje é um stub determinístico: varre uma
caixa sintética pela imagem quer haja objeto ali ou não. Sobre o vídeo isso vira
um retângulo passeando de um lado para o outro — pior que nada numa demo, porque
o espectador lê aquilo como detecção de verdade.

**As detecções continuam publicadas e continuam alimentando a
`perception_layer` do costmap.** O contrato de tópicos do `CLAUDE.md` está
intacto; saiu só o desenho. `detection-overlay.js` segue no bundle, testado,
para voltar quando o TIDL substituir o stub.

### Testes

```bash
cd hmi && node --test "test/**/*.test.js"    # 138 — lógica do bundle
cd .. && python3 -m pytest tests/ -q          # 36 — guardas estruturais
```

Os guardas estruturais são checagens estáticas em arquivos commitados, não
testes de runtime. Existem porque cada invariante que eles cobrem é barata de
quebrar numa edição de uma linha e cara de descobrir — as de colocação só falham
no Aquila, semanas depois.

---

## 11. O que está feito e o que falta

### Feito (24/08/2026, só no host)

- **F1** — transporte e esqueleto: serviços `cockpit` e `hmi`, bundle, câmera ao
  vivo, reconexão automática. Evidência: [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md).
- **F3b** — painel de cena (duas câmeras alternáveis) e painel de navegação
  (costmap, plano, laser, pegada, clique-para-meta). Portão cumprido: meta
  clicada aceita e executada pelo Nav2.
- **Controle de simulação** pelo cockpit: play, pause, reset.
- **Controle de câmera**: girar, inclinar, mover, zoom, recentrar.
- **Identidade Toradex** e repaletização do mapa para fundo claro.
- **Qualidade de imagem**: 800×600@5 Hz → 1600×1200@10 Hz, JPEG 70 → 95.

Evidência do conjunto: [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md).

### Ajustes de UI (25/08/2026, só no host)

Três pedidos de bancada, fora da numeração de fases:

- **Marca Toradex ao dobro** (34 → 68 px de altura; a do ROS, 22 → 44). A altura
  mínima da faixa saiu de 48 para 80 px **pelo mesmo token** (`--bar-min-height`
  em `tokens.css`) — a barra tem `overflow-x`, não `-y`, então as duas medidas
  divergirem cortaria a marca sem avisar.
- **Seguir o robô** nas duas vistas de cena. Verificado no host: `scene_top` em
  `(-1,552 ; 0,151 ; 6,0)` contra robô em `(-1,598 ; 0,130)` — acompanhamento
  dentro de ~5 cm com o `z` preservado; `scene_iso` em `(-4,551 ; 3,15 ; 2,4)`,
  isto é, o deslocamento medido `(-3, +3, 2,4)` mantido enquanto desliza com o
  robô. Desligar congelou a pose por 6 s; religar recentrou.
- **Reiniciar a navegação** pelo cockpit, em 6,6 s, com a meta interrompida
  terminando `CANCELED` e os servidores voltando `active [3]`.

Nada disso foi visto num navegador com captura de tela: o Chrome não está
instalado nesta máquina e o Firefox snap em modo headless não respondeu. O que
existe é a página servida com o HTML e o CSS corretos (`curl` 200, tokens e os
dois botões presentes no que o nginx entrega) mais a verificação pelo lado do
ROS. **Nada disso rodou no Aquila AM69** (regra 7 do `CLAUDE.md`); as medidas são
todas do host x86 em `learn`.

### Falta

- **F4 — controle manual.** `twist_mux` arbitrando contra o Nav2. É a única
  região da tela que ainda mente, e por isso os botões estão desligados.
- **F2 — kiosk no módulo.** Chromium no Aquila servindo este mesmo bundle. Três
  coisas que só o módulo responde:
  1. o bundle foi verificado no **Firefox**; o kiosk é Chromium, e a ressalva de
     cache do `<img>` em `config.js` vem da literatura, não de medição;
  2. o MJPEG a 1600×1200 atravessando a Ethernet — nada disso foi medido lá;
  3. no modo `deploy` não existe Gazebo: os botões de simulação precisam sumir
     ou dizer por que não valem. **Ainda não foi tratado.**

### Uma pendência conhecida, sem relação com o cockpit

O segfault do `route_server` ao configurar, agora caracterizado e
determinístico — ver a [armadilha 8](#8-reset--startup-no-nav2-mata-o-container-segfault-no-route_server).
Não é regressão do cockpit; é o motivo pelo qual o reset de navegação usa
`PAUSE`/`RESUME` em vez de `RESET`/`STARTUP`.

---

---

## Referências

- `.ai/CLAUDE.md` — contrato operacional, regras invioláveis, fase atual
- `.ai/AGENTS.md` — contrato de implementação, milestones, Definition of Done
- `.ai/changelog.md` — decisões tomadas e o porquê
- `CLAUDE.md` na raiz — as regras invioláveis, principalmente a 1
- README de cada pacote em `ros2_ws/src/demo_*/`
- [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md) — decisões e fases do cockpit web
- [`ml35/estado-fases.md`](ml35/estado-fases.md) — estado do ML3.5, leia primeiro numa sessão nova
- [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md), [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md) — evidência de bancada do cockpit
