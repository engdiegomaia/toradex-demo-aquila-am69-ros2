# CLAUDE.md

Demo de robótica ROS 2 rodando em Torizon OS sobre Aquila AM69, com Gazebo substituindo o robô físico.
O simulador roda no host x86. A stack de navegação e a interface rodam no módulo, em containers.

Detalhamento completo em `.ai/projeto.md`. Este arquivo é operacional: leia antes de qualquer tarefa.

---

## Regras invioláveis

Estas existem porque violá-las custa dias. Não são preferências.

1. **Nada que dependa de OpenGL desktop roda no alvo.** A GPU do AM69 expõe apenas OpenGL ES 3.2 e Vulkan 1.2. Gazebo (OGRE 2), RViz2 e qualquer coisa sobre OGRE 2 ficam no host x86. Nunca gere launch file, compose ou Dockerfile que coloque essas ferramentas no serviço do módulo.
2. **RMW é sempre `rmw_cyclonedds_cpp`.** Definido na imagem base via `ENV`, nunca em runtime. Não troque para Fast DDS nem remova a variável: há falha conhecida de descoberta entre containers com o padrão.
3. **Instalação do Torizon OS só por Toradex Easy Installer.** Nunca documente, script ou sugira atualização remota para chegar à 7.4.0 neste módulo. O módulo V1.0 com bootloader antigo carrega a device tree da V1.1 e para de bootar.
4. **`/opt` não é modificável por TorizonCore Builder.** Qualquer proposta que dependa de escrever em `/opt` no OS está errada. Vai para dentro do container ou para build Yocto.
5. **Emulação arm64 não mede desempenho.** Nunca tire conclusão de CPU, latência ou FPS de execução sob QEMU. Números só valem no hardware.
6. **Percepção fala pelo contrato, nunca direto.** Ver seção Contrato de tópicos.

---

## Três modos de execução, um só código

A mesma imagem e o mesmo código funcionam nos três modos. O que muda é `platform` e em qual máquina cada serviço sobe. Se uma mudança exigir código diferente por modo, o desenho está errado.

| Modo | Invocação | Simulador | Stack | Uso |
| --- | --- | --- | --- | --- |
| `learn` | `compose.host.yml --profile learn` | amd64 nativo, host | amd64, host | Aprendizado e desenvolvimento |
| `hil` | `compose.host.yml` (host) + `compose.module.yml` (módulo) | amd64 no host | arm64 no AM69 | Hardware-in-the-loop; entregável do ML3.5 |
| `deploy` | `compose.module.yml` | nenhum | arm64 no AM69 | A1 físico; fora de escopo |

Os composes são divididos por **máquina**, não por modo: `docker/compose.host.yml`
e `docker/compose.module.yml`. O modo é escolhido por profile do Compose e por
qual arquivo se invoca em qual máquina. O layout antigo `compose/{learn,emul,target}.yaml`
foi substituído no ML3.5 — ver `docs/ml35/guia-ml35-docker.md`. O modo `emul` caiu
junto: imagens arm64 continuam sendo construídas sob QEMU, mas não há mais um
compose dedicado para rodar a stack emulada.

---

## Estrutura

```
docker/base|sim|nav|perception|viz|tools|hw
docker/compose.host.yml|compose.module.yml
docker/cyclonedds/host.xml|module.xml
ros2_ws/src/demo_description|demo_bringup|demo_navigation|demo_perception|demo_simulation
docs/ml35/guia-ml35-docker.md
.ai/
```

---

## Contrato de tópicos

Invariante do projeto. Existe para que a NPU entre depois sem refatoração.

| Tópico | Tipo | Produtor | Consumidores |
| --- | --- | --- | --- |
| `/demo/camera/image_raw` | `sensor_msgs/Image` | Gazebo, câmera USB ou rosbag | `demo_perception` |
| `/demo/perception/detections` | `vision_msgs/Detection2DArray` | `demo_perception` | costmap layer, HMI |
| `/demo/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Gazebo ou driver real |

`demo_perception` hoje é stub e publica detecções sintéticas. Ele **nunca** deve saber a origem da imagem, e nenhum consumidor deve saber se a detecção veio de stub ou de inferência real. Trocar o stub por TIDL precisa ser troca de container, não mudança de interface.

As detecções alimentam uma camada de costmap no Nav2, não apenas a tela. Mantenha essa costura mesmo com o stub.

---

## Comandos

```bash
# workspace
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose

# habilitar emulacao arm64 (uma vez por maquina)
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx create --use --name multiarch

# imagens multi-arch
docker buildx build --platform linux/amd64,linux/arm64 -t <reg>/<img>:<tag> --push docker/<dir>

# execucao (a partir de docker/)
docker compose -f compose.host.yml --profile learn up --build   # learn: tudo no host
docker compose -f compose.host.yml up sim viz                   # hil: lado host
docker compose -f compose.module.yml up -d                      # hil: no modulo

# diagnostico ROS 2
ros2 topic list && ros2 topic hz <topic> && ros2 node list
ros2 run tf2_tools view_frames

# diagnostico do modulo
cat /etc/os-release && ostree admin status && tdx-info
```

---

## Convenções

Pacotes ROS 2 usam prefixo `demo_`. Python com `ament_python` por padrão; C++ apenas onde houver requisito de desempenho comprovado no hardware.

Um launch por modo em `demo_bringup`, sem arquivo único cheio de condicionais. Legibilidade importa mais que reuso aqui.

Parâmetros do Nav2 ficam em YAML versionado em `demo_navigation`, nunca embutidos em código.

Cada container tem responsabilidade única. `perception` é separado desde já, mesmo sendo stub, porque define a granularidade de atualização OTA.

Imagens são multi-arch. `platform` é declarado explicitamente no compose, nunca inferido.

---

## Onde estamos

Fase atual: **ML3.5, quadrúpede real e containerização.** L1, L2, L3 e ML3.1 concluídas.

- **L1 (ML1)** — `demo_tutorials`: heartbeat pub/sub, serviço, launch, testes. Concluída 31/07/2026.
- **L2 (ML2)** — `demo_description`: xacro diff-drive parametrizado, árvore TF, RViz. Concluída 07/08/2026. Pendente do operador: `sudo apt install liburdfdom-tools` e confirmação visual no RViz.
- **L3 (ML3)** — `demo_simulation`, `demo_navigation`, `demo_perception`, `demo_bringup`. Concluída 10/08/2026, **com os quatro critérios de aceitação executados de verdade**: teleop move o robô por `/demo/cmd_vel`; odom/scan/TF/comandos trocam mensagens; os sete servidores do Nav2 chegam a `active`; e um goal terminou `SUCCEEDED` (0,0 → 2.43,0.20). Mapa do armazém gerado por SLAM e commitado. Ver `changelog.md` para as cinco falhas silenciosas encontradas no caminho.
- **ML3.1** — aparência do robô e guia. Concluída 10/08/2026. `demo_robot.urdf.xacro` virou **wrapper fino sobre o TurtleBot 4 upstream** (`nav2_minimal_tb4_description`, via `package://`, sem binários no repo); a montagem peça a peça por mesh foi tentada e abandonada. A causa real do "robô com partes separadas" era o **RViz** (`RobotModel: Enabled: false` + 33 triedros de TF), não o modelo — daí `demo_bringup/rviz/demo_view.rviz`. Mais três pendências do ML3 e dois documentos em `docs/`. Pendente do operador: confirmação visual em RViz2/Gazebo com GUI.
- **ML3.5** — em curso: locomoção quadrúpede **Go2** real + containerização.
  **Estado por fase e próximo passo: `docs/ml35/estado-fases.md`** (leia primeiro
  numa sessão nova). Spec: `docs/ml35/guia-ml35-docker.md`. Fases F0 a F6, com
  portão em cada uma; F0 (`3885f2e`), F1 (`5d95934`), F2 (14/08/2026, spike não
  commitado) e **F3 (`db4e6f3`, `ae3d9a1`, 17/08/2026) concluídas. F4 é a
  próxima** — o contrato atravessando fronteira de container.
  Base de locomoção: `legubiao/quadruped_ros2_control` — confirmado na árvore em
  F2, não pelo README. Robô-alvo é **Go2**, não A1: `a1_description` declara
  licença `TODO`. Atenção, a justificativa de F2 ("Go2 declara BSD") era
  **incompleta**: o pacote também não tinha texto de licença nem titular. F3
  rastreou até `unitreerobotics/unitree_ros` (BSD-3, Unitree Robotics) e provou
  as malhas bit-idênticas por hash — é essa a base legal da vendorização, não a
  string do `package.xml`. Ver `ros2_ws/src/go2_description/README.md`.
  F3 **não foi retarget de cinemática**: a troca de alvo eliminou esse trabalho.
  Aparência do Go2 confirmada pelo operador no Gazebo GUI em 17/08/2026, com
  `world:=empty.sdf`; RobotModel/TF no RViz2 ainda pendente.
  A containerização entra em F1, **antes** da troca do robô, para separar risco
  de Docker/DDS de risco de marcha.
- **L4 (ML4)** — depois: absorvida em grande parte pelo F1 do ML3.5.

**Robô:** diff-drive hoje, quadrúpede A1 em curso no ML3.5. Continua verdade (verificado em 14/08/2026) que **ninguém entrega quadrúpede + Jazzy + Harmonic + Nav2 funcionando**: CHAMP upstream é ROS 1, e os dois forks Go2 em Jazzy listam Nav2 como "coming soon" **e não declaram licença** — bloqueador para demo comercial, mesmo critério que eliminou o Tugbot no ML3.1. A base escolhida é `legubiao/quadruped_ros2_control` (Apache-2.0, `ros2_control` nativo, branch default Jazzy); a integração com Nav2 é **nossa**, ninguém entrega pronta. O diff-drive validado permanece selecionável por launch arg. Justificativa completa em `changelog.md`.

**Cenário:** `warehouse.sdf` de `nav2_minimal_tb4_sim` (mantido pela org do Nav2, SDF nativo Harmonic). O world do AWS RoboMaker foi arquivado em jul/2026 e é Gazebo Classic — não usar.

A equipe não tem experiência prévia com ROS 2. As fases L1 a L3 são feitas com ROS 2 **instalado nativamente no host, sem container**, de propósito: container sobre um sistema desconhecido torna impossível separar falha do ROS de falha do Docker. A containerização entra na L4.

Ordem: L1 fundamentos, L2 URDF/TF2/RViz2, L3 Gazebo e Nav2, L4 containers e emulação arm64. Depois disso, fases 0 a 5 com o hardware.

Ao concluir uma fase, atualize esta seção e `.ai/changelog.md`.

---

## Ao propor solução

Diga sempre em qual das duas máquinas o código roda, host ou módulo. Metade dos erros possíveis neste projeto vem de colocar algo no lado errado.

Se a proposta envolve GPU, acesso a device, desempenho ou térmico, marque explicitamente que só pode ser validada no hardware real.

Distinga fato verificado de hipótese. Não invente binding de device tree, nome de overlay, opção `CONFIG_` do kernel nem versão de BSP.

Prefira a mudança menor que resolve. Não refatore estrutura existente sem pedido.
