# ML3.5, arquitetura containerizada e guia de execução

Spec de implementação. Vale sobre o plano original onde houver divergência.

Alvo: ROS 2 Jazzy, Gazebo Harmonic, quadrúpede A1, host x86 de simulação mais módulo Aquila AM69 rodando Torizon OS.

---

## 1. Eixo de modularidade

A decomposição não é por pacote ROS. É por resposta a uma pergunta: o que precisa ser trocado quando a simulação vira hardware real?

A resposta é uma coisa só, a planta. Tudo acima dela consome o mesmo contrato de tópicos e não sabe se está falando com Gazebo ou com um A1 físico.

```
        planta (trocável)                consumidores (fixos)
  ┌──────────────────────────┐     ┌─────────────────────────────┐
  │ sim   (Gazebo + control) │     │ nav        (Nav2)           │
  │   ou                     │ ==> │ perception (demo_perception)│
  │ hw    (A1 real, futuro)  │     │ viz        (RViz2)          │
  └──────────────────────────┘     └─────────────────────────────┘
            contrato: /demo/cmd_vel  /demo/odom  /demo/scan  /demo/camera/image_raw
```

Por que `sim` é um container só e não dois: `gz_ros2_control` é plugin do Gazebo e carrega o `controller_manager` dentro do processo do `gz sim`. Separar Gazebo dos controladores em containers diferentes não é possível sem reescrever a integração. Então a planta simulada é uma unidade: Gazebo, `gz_ros2_control`, controladores de marcha, `robot_state_publisher` e a ponte `ros_gz_bridge`.

---

## 2. Containers

| Container | Arquitetura | Onde roda | Conteúdo |
|---|---|---|---|
| `sim` | x86_64 apenas | host | Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, controladores quadrúpede, descrição do robô, mundo |
| `nav` | x86_64 e arm64 | host ou módulo | Nav2, mapa, params, costmaps |
| `perception` | x86_64 e arm64 | host ou módulo | `demo_perception`, sem alteração |
| `viz` | x86_64 apenas | host apenas | RViz2, rqt |
| `tools` | x86_64 e arm64 | qualquer | teleop, CLI ROS 2, colcon, execução dos testes |
| `hw` | arm64 apenas | módulo | placeholder, A1 físico, fora do escopo do ML3.5 |

`sim` e `viz` nunca vão para arm64. Regra 1 do projeto, Gazebo é OGRE 2 e RViz2 quer GL de desktop. A build multi-arch é seletiva, não uniforme.

`hw` existe no repo desde F1, vazio, com um README de uma linha. É onde a colisão de invariantes registrada no plano vai bater: a base de `quadruped_ros2_control` documenta conflito entre CycloneDDS e `unitree_sdk2` e recomenda FastDDS, enquanto a regra 2 do projeto é CycloneDDS sempre. Enquanto o A1 for simulado, o SDK não entra e não há colisão. O container vazio serve para o problema ficar visível no lugar certo em vez de aparecer como surpresa.

---

## 3. Layout no repositório

```
docker/
  base/Dockerfile           # ros:jazzy-ros-base, cyclonedds, usuário não root, entrypoint
  sim/Dockerfile            # FROM base, ros-jazzy-ros-gz, gz_ros2_control, controladores
  nav/Dockerfile            # FROM base, nav2
  perception/Dockerfile     # FROM base, deps de demo_perception
  viz/Dockerfile            # FROM base, rviz2, rqt
  tools/Dockerfile          # FROM base, teleop, colcon, pytest
  hw/README.md              # placeholder, A1 físico
  compose.host.yml
  compose.module.yml
  cyclonedds/host.xml
  cyclonedds/module.xml
  entrypoint.sh
  .env.example
docs/ml35/
  guia-ml35-docker.md       # este arquivo
```

`base` centraliza a configuração de RMW e o `source` do overlay. Um lugar para mudar, não seis.

---

## 4. Imagem base

`ros:jazzy-ros-base` tem tag arm64 e serve host e módulo. Torizon OS é um host Docker, então a imagem oficial roda no módulo sem adaptação de userspace.

Duas restrições que valem para as imagens arm64:

- O storage de containers no Torizon fica na partição de dados. Camada gorda custa espaço real. Use `--no-install-recommends`, limpe `/var/lib/apt/lists` na mesma layer e mantenha `nav` e `perception` sem nada de gráfico.
- Acesso ao acelerador do AM69 a partir do container exige os device nodes e o runtime da TI, que a imagem ROS genérica não traz. Se `demo_perception` for para inferência acelerada em algum momento, confirme os nodes e a stack na documentação Toradex e TI antes de assumir qualquer coisa. Liste `/dev` no host primeiro. Enquanto a percepção for CPU e OpenCV, nada disso é necessário.

Build arm64 a partir do host x86:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --use --name demo || docker buildx use demo
docker buildx build --platform linux/arm64 \
  -f docker/nav/Dockerfile -t ${REGISTRY}/demo-nav:${TAG} --push .
```

QEMU aqui constrói imagem. Não mede nada. Regra 5.

---

## 5. DDS entre containers e entre máquinas

Todos os containers usam `network_mode: host`. Isso resolve descoberta entre containers da mesma máquina sem configuração adicional e evita a classe de problema de DDS atrás de bridge NAT.

Entre host e módulo, multicast costuma morrer em Wi-Fi e em switch gerenciado. Não dependa dele. Use peers explícitos.

`docker/cyclonedds/host.xml`:

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="${HOST_IP}"/>
        <Peer address="${MODULE_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

`module.xml` é o mesmo arquivo com a interface do módulo. Ajuste `NetworkInterface name` ao que existe na máquina, confira com `ip -br link`.

`.env.example`:

```
REGISTRY=registry.local/demo
TAG=ml35
ROS_DOMAIN_ID=42
HOST_IP=192.0.2.10
MODULE_IP=192.0.2.11
DISPLAY=:0
```

`ROS_DOMAIN_ID` igual nas duas máquinas. Domínio diferente é a causa mais comum de "os tópicos não aparecem" e não gera erro nenhum.

---

## 6. Compose do host

`docker/compose.host.yml`, trecho:

```yaml
x-common: &common
  network_mode: host
  environment:
    - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
  volumes:
    - ./cyclonedds/host.xml:/cfg/cyclonedds.xml:ro

services:
  sim:
    <<: *common
    image: ${REGISTRY}/demo-sim:${TAG}
    build:
      context: ..
      dockerfile: docker/sim/Dockerfile
    ipc: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
      - DISPLAY=${DISPLAY}
    volumes:
      - ./cyclonedds/host.xml:/cfg/cyclonedds.xml:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    devices:
      - /dev/dri:/dev/dri
    command: >
      ros2 launch demo_bringup sim.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}

  nav:
    <<: *common
    image: ${REGISTRY}/demo-nav:${TAG}
    profiles: ["learn"]
    command: >
      ros2 launch demo_bringup nav_select.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}
      use_sim_time:=true

  perception:
    <<: *common
    image: ${REGISTRY}/demo-perception:${TAG}
    profiles: ["learn"]
    command: ros2 launch demo_bringup perception.launch.py use_sim_time:=true

  viz:
    <<: *common
    image: ${REGISTRY}/demo-viz:${TAG}
    ipc: host
    command: ros2 launch demo_bringup viz.launch.py
```

`nav` e `perception` ficam sob o profile `learn`. No modo HIL eles simplesmente não sobem no host, sem edição de arquivo.

Para NVIDIA no host, troque o mapeamento de `/dev/dri` por `gpus: all` com o nvidia-container-toolkit instalado. Para Intel e AMD, `/dev/dri` basta.

`use_sim_time` é `true` em tudo que consome a simulação, inclusive no módulo em modo HIL, e a ponte precisa publicar `/clock`. Relógio errado no módulo produz TF extrapolando e Nav2 recusando goal sem mensagem óbvia.

---

## 7. Compose do módulo

`docker/compose.module.yml`:

```yaml
x-common: &common
  network_mode: host
  restart: unless-stopped
  environment:
    - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    - CYCLONEDDS_URI=file:///cfg/cyclonedds.xml
  volumes:
    - ./cyclonedds/module.xml:/cfg/cyclonedds.xml:ro

services:
  nav:
    <<: *common
    image: ${REGISTRY}/demo-nav:${TAG}
    command: >
      ros2 launch demo_bringup nav_select.launch.py
      robot_type:=${ROBOT_TYPE:-quadruped}
      use_sim_time:=true

  perception:
    <<: *common
    image: ${REGISTRY}/demo-perception:${TAG}
    command: ros2 launch demo_bringup perception.launch.py use_sim_time:=true
```

Sem `sim`, sem `viz`, sem X11, sem `/dev/dri`. Nada gráfico chega ao módulo.
O mesmo `ROBOT_TYPE` seleciona a planta no host e o Nav2 correspondente nos dois
Compose. No quadrúpede, `odom_tf` ainda deriva a TF do ground truth do Gazebo;
estimativa por pernas permanece fora do fechamento da ML3.5.

Quando houver câmera real, no modo deploy, `perception` ganha o mapeamento do device:

```yaml
    devices:
      - "/dev/video0:/dev/video0"
```

Antes de debugar permissão de container, confirme que o node existe no host. Se o Device Tree não instanciou o sensor, não há mapeamento que resolva.

---

## 8. Modos de execução

| Modo | Host x86 | Módulo Aquila AM69 | Para que serve |
|---|---|---|---|
| `learn` | `sim`, `nav`, `perception`, `viz` | desligado | desenvolvimento e o portão de F2 a F5 |
| `hil` | `sim`, `viz` | `nav`, `perception` | prova que a divisão funciona, é o entregável do ML3.5 |
| `deploy` | nada | `nav`, `perception`, `hw` | A1 físico, fora do escopo |

`learn` é o que existe hoje, containerizado. `hil` é o modo que vale a pena demonstrar: a simulação roda onde tem GPU, a navegação e a percepção rodam onde vão rodar em produção.

---

## 9. Como rodar

### Modo learn, tudo no host

```bash
cd docker
cp .env.example .env          # ajuste IPs e DISPLAY
xhost +local:docker
docker compose -f compose.host.yml --profile learn up --build
```

Verificação, de outro terminal:

```bash
docker compose -f compose.host.yml exec tools bash
ros2 topic list | grep /demo/
ros2 topic hz /demo/scan
ros2 topic echo /demo/odom --once
ros2 control list_controllers
```

Goal de navegação, mesmo critério que o ML3 já bateu:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}}}}"
```

Encerrar:

```bash
docker compose -f compose.host.yml --profile learn down
xhost -local:docker
```

### Modo hil, simulação no host e navegação no módulo

Host, só a planta e a visualização:

```bash
cd docker
docker compose -f compose.host.yml up sim viz
```

Publicar as imagens arm64 e levar o compose para o módulo:

```bash
docker buildx build --platform linux/arm64 -f nav/Dockerfile \
  -t ${REGISTRY}/demo-nav:${TAG} --push ..
docker buildx build --platform linux/arm64 -f perception/Dockerfile \
  -t ${REGISTRY}/demo-perception:${TAG} --push ..

scp compose.module.yml .env cyclonedds/module.xml torizon@${MODULE_IP}:~/demo/
```

Sem registry acessível, transfira a imagem direto:

```bash
docker save ${REGISTRY}/demo-nav:${TAG} | ssh torizon@${MODULE_IP} docker load
```

Módulo:

```bash
ssh torizon@${MODULE_IP}
cd ~/demo && docker compose -f compose.module.yml up -d
docker compose -f compose.module.yml logs -f nav
```

Verificação de que as duas máquinas se enxergam, do módulo:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list | grep /demo/
ros2 topic hz /demo/scan
```

Se a lista vier vazia, a ordem de checagem é: `ROS_DOMAIN_ID` igual nos dois lados, `ip -br link` batendo com o `NetworkInterface name` do XML, IPs do `Peers` corretos, firewall do host liberando UDP 7400 e adjacentes.

### Modo deploy

Fora do escopo do ML3.5. O container `hw` fica vazio até existir A1 físico, e é onde a decisão CycloneDDS contra FastDDS vai precisar ser tomada.

---

## 10. Onde cada fase toca a infraestrutura

| Fase | Docker | ROS |
|---|---|---|
| F0 | nada | changelog do ML3.1, commit das 1047 linhas |
| F1 | cria `docker/` inteiro, `base`, `sim`, `nav`, `perception`, `viz`, `tools`, os dois composes, o XML de DDS | nenhuma mudança de comportamento, só empacotamento do diff-drive atual |
| F2 | tag descartável `demo-sim:spike-go2` | clone da base upstream, Go2 sem modificação |
| F3 | mesma imagem `sim`, muda o conteúdo | descrição do A1, cinemática, massas, limites de junta, malhas |
| F4 | nada | remaps para `/demo/*`, `demo_perception` intocado |
| F5 | promove `nav` e `perception` para arm64, valida `hil` | odometria de pernas contra costmap, `robot_radius`, footprint, tolerâncias |
| F6 | profile e variável `ROBOT_TYPE` no compose | `robot_type:=quadruped\|diffdrive`, testes estendidos |

F1 antes de F2 é deliberado. Se o compose quebrar depois que o quadrúpede entrar, você não sabe se foi Docker, DDS ou marcha. Containerizando o que já funciona, F2 falha por um motivo só.

---

## 11. Pontos a confirmar, não a assumir

O plano original afirma coisas sobre `quadruped_ros2_control` que precisam ser verificadas no momento do clone, em F2:

- Suporte real a Jazzy e a Harmonic na branch default.
- Licença, antes de vendorizar qualquer descrição derivada de `unitree_ros`.
- Ausência de config do A1 e o que exatamente vem de `chvmp/robots`.

Sobre o módulo:

- Device nodes e runtime necessários para o acelerador do AM69 dentro de container. Confirme na documentação Toradex e TI, liste `/dev` no host antes.
- Espaço livre na partição de dados do Torizon antes de subir as imagens.

Risco silencioso que continua valendo do plano original: parâmetro de marcha sintonizado para Go2 rodando num A1 produz robô que anda mal sem gerar erro. Mesma classe da armadilha de escala que o ML3.1 já pagou. O portão de F3 é robô em pé e estável respondendo a `cmd_vel`, não build limpo.
