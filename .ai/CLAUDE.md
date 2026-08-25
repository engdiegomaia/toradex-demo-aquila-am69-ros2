# CLAUDE.md

Demo de robótica ROS 2 rodando em Torizon OS sobre Aquila AM69, com Gazebo substituindo o robô físico.
O simulador roda no host x86. A stack de navegação e a interface rodam no módulo, em containers.

Detalhamento completo em `.ai/projeto.md`. Este arquivo é operacional: leia antes de qualquer tarefa.

---

## Regras invioláveis

Estas existem porque violá-las custa dias. Não são preferências.

1. **Nada que dependa de OpenGL desktop roda no alvo.** A GPU do AM69 expõe apenas OpenGL ES 3.2 e Vulkan 1.2. Gazebo (OGRE 2), RViz2 e qualquer coisa sobre OGRE 2 ficam no host x86. Nunca gere launch file, compose ou Dockerfile que coloque essas ferramentas no serviço do módulo.
2. **RMW é sempre `rmw_cyclonedds_cpp`.** Definido na imagem base via `ENV`, nunca em runtime. Não troque para Fast DDS nem remova a variável: há falha conhecida de descoberta entre containers com o padrão.
3. **Instalação do Torizon OS só por Toradex Easy Installer.** O baseline do hardware é 7.7.0. Nunca documente, script ou sugira atualização remota para chegar a ele neste módulo. O módulo V1.0 com bootloader antigo carrega a device tree da V1.1 e para de bootar.
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
  commitado) e **F3 (`db4e6f3`, `ae3d9a1`, 17/08/2026) concluídas**. A marcha,
  HOLD longo e navegação no host estão corrigidos e medidos. A árvore TF fecha
  por `odom_tf`, mas usa **ground truth do Gazebo**, não estimativa por pernas.
  Em 21/08 o Nav2 arm64 composto rodou no Aquila AM69; o gargalo medido foi a
  câmera de 74,2 Mbit/s no Wi-Fi. F4 e F6 fecharam em 24/08: contrato completo
  revalidado e cold start + goal `SUCCEEDED` tanto no Go2 quanto no diff-drive,
  pelo seletor acoplado `ROBOT_TYPE=quadruped|diffdrive`. O HIL Ethernet foi
  executado em 24/08 com Nav2 + perception no AM69: câmera e LiDAR exigiram
  correções de QoS para amostras fragmentadas, e uma meta curta fechou em 28 s.
  Em 25/08 o portão **funcional** passou de novo no AM69 (Nav2 arm64 sem
  segfault, percepção produzindo detecções, contrato atravessando a fronteira),
  e três falhas silenciosas foram corrigidas: `docker/.env` sobrescrevendo o
  ambiente explícito em `module.sh`, `host.rendered.xml` que nunca havia sido
  renderizado, e o DDS do módulo fixado em `ethernet1` quando a rota do kernel
  usa `ethernet0` (as duas portas estão na mesma /24, métricas 101/102).
  **Use `MODULE_IP` explícito e `ethernet0`.** Ainda em 25/08, à noite, o enlace
  cabeado gigabit foi estabelecido e comprovado (1000 Mb/s full, RTT 0,400 ms,
  rota simétrica, `verify` 3/3) e o protocolo de estabilidade **foi executado**.
  **O portão de 8 m reprovou, e a rede não é a causa** — a hipótese
  `ethernet1`/`ethernet0` está refutada por medição. As duas causas medidas são
  **CPU do módulo** (Nav2 sozinho a 600–727% de 800%; a percepção soma ~187% e
  passa da capacidade, o `collision_monitor` recusa a nuvem com 1,0–1,2 s de
  defasagem contra 42 ms com o Nav2 ocioso, custo de 2,8× na velocidade média) e
  **decisão de trajeto** (`vx` em zero em 79% das amostras, giro em 93,9%: o robô
  gira em vez de transladar; 28,9% de eficiência de rota contra 57% no host).
  Tirar a câmera do fio **não** faz a meta passar. Ver
  `docs/results/ml35-f5-ethernet0-repeticao.md` e a seção
  "Sessão 25/08 (noite)" de `docs/ml35/estado-fases.md`.
- **Cockpit web** — trilha paralela, plano em `docs/ml35/plano-cockpit-web.md`.
  **F1 e F3b fechados em 24/08/2026**: as cinco regiões da tela ao vivo, clique
  no mapa vira meta aceita pelo Nav2, play/pause/reset da simulação e controle
  das câmeras de cena a partir do cockpit, e a identidade Toradex aplicada
  (fundo branco, `#00508c`, `#96c837`, `#ff5a00`). Evidência em
  `docs/results/cockpit-web-f3b.md`. Em 25/08 entraram três **ajustes de UI** de
  bancada, fora da numeração de fases: marca Toradex ao dobro (com
  `--bar-min-height` acoplada ao mesmo token, senão a faixa corta a marca sem
  sintoma), câmeras de cena **seguindo o robô** nas duas vistas (só o alvo da
  órbita se move, então o enquadramento medido do armazém continua valendo), e
  **`/demo/nav/reset`** — reiniciar o Nav2 pelo cockpit, por um relay que roda no
  container do Nav2, isto é, no Aquila no modo `hil`. Evidência em
  `docs/results/cockpit-web-ui-ajustes.md`. Achado com peso próprio:
  **`RESET`+`STARTUP` no `lifecycle_manager` do Nav2 derruba o container** com
  `SIGSEGV` ao configurar o `route_server`, reproduzido duas vezes — determinístico,
  não o "aconteceu uma vez" que os docs registravam; daí a sequência ser cancelar
  + limpar costmaps + `PAUSE`/`RESUME`, que nunca passa por `CONFIGURE`.
  **Próximo: F4**, controle manual atrás do `twist_mux` — as setas da barra estão
  desligadas de propósito até lá.
  Duas invariantes que saíram do F3b, cada uma paga com uma sessão de
  depuração: (a) o navegador **não** pode chamar serviço com tipo do Gazebo —
  o rosbridge importa o pacote de interfaces dentro do container do cockpit,
  que não tem `ros_gz_interfaces` e no módulo nunca terá; use a fachada
  `std_srvs` do `sim_control_relay`; (b) "iniciar a simulação **no target**"
  não existe sob a regra 1 — só "controlar do cockpit a simulação do host".
  Em 24/08 o **cockpit web F1** fechou no host: serviços `cockpit`
  (rosbridge + web_video_server) e `hmi` (nginx) em `compose.host.yml`, bundle em
  `hmi/` sem etapa de build, câmera ao vivo e reconexão automática. Plano e
  fases em `docs/ml35/plano-cockpit-web.md`; evidência em
  `docs/results/cockpit-web-f1.md`. Próxima: F3b.
  O portão de 8 m segue aberto: 0 metas no protocolo 420/200 s, apesar de 8,31 m
  percorridos sem queda. Evidência em `docs/results/ml35-hil-ethernet.md` e
  estado exato em `docs/ml35/estado-fases.md`.
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

**Robô:** quadrúpede Go2 é o padrão da ML3.5; o diff-drive validado permanece
como fallback por `ROBOT_TYPE`. Continua verdade (verificado em 14/08/2026) que
ninguém entrega quadrúpede + Jazzy + Harmonic + Nav2 pronto: a integração é do
projeto. Base: `legubiao/quadruped_ros2_control` (Apache-2.0).

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
