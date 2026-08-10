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

| Modo | Arquivo | Simulador | Stack | Uso |
| --- | --- | --- | --- | --- |
| `learn` | `compose/learn.yaml` | amd64 nativo, host | amd64, host | Aprendizado e desenvolvimento |
| `emul` | `compose/emul.yaml` | amd64 nativo, host | arm64 emulado, host | Validar build e grafo antes do hardware |
| `target` | `compose/target.yaml` | amd64 no host | arm64 no AM69 | Demo real |

---

## Estrutura

```
docker/base|navigation|perception|hmi|simulation
ros2_ws/src/demo_description|demo_bringup|demo_navigation|demo_perception|demo_simulation
compose/learn.yaml|emul.yaml|target.yaml
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

# execucao
docker compose -f compose/learn.yaml up
docker compose -f compose/emul.yaml up
docker compose -f compose/target.yaml up

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

Fase atual: **L4, containers e emulação arm64.** L1, L2 e L3 concluídas.

- **L1 (ML1)** — `demo_tutorials`: heartbeat pub/sub, serviço, launch, testes. Concluída 31/07/2026.
- **L2 (ML2)** — `demo_description`: xacro diff-drive parametrizado, árvore TF, RViz. Concluída 07/08/2026. Pendente do operador: `sudo apt install liburdfdom-tools` e confirmação visual no RViz.
- **L3 (ML3)** — `demo_simulation`, `demo_navigation`, `demo_perception`, `demo_bringup`. Concluída 10/08/2026, **com os quatro critérios de aceitação executados de verdade**: teleop move o robô por `/demo/cmd_vel`; odom/scan/TF/comandos trocam mensagens; os sete servidores do Nav2 chegam a `active`; e um goal terminou `SUCCEEDED` (0,0 → 2.43,0.20). Mapa do armazém gerado por SLAM e commitado. Ver `changelog.md` para as cinco falhas silenciosas encontradas no caminho.
- **L4 (ML4)** — próxima: containers e emulação arm64.

**Robô:** diff-drive, não quadrúpede. Nenhum projeto mantido entrega quadrúpede + Jazzy + Harmonic + Nav2 funcionando hoje (CHAMP upstream é ROS 1; o melhor fork Jazzy tem Nav2 "coming soon" desde mai/2025). O contrato `/demo/cmd_vel` torna a troca posterior barata — rastreado como ML3.5. Justificativa completa em `changelog.md`.

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
