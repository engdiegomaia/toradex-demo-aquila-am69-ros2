# ML3.5 — estado das fases

Documento de continuidade. Quem pegar este projeto numa sessão nova lê **este
arquivo primeiro**, depois `guia-ml35-docker.md` (a spec).

Atualize a tabela e a seção da fase ao fechar cada portão.

---

## Objetivo do ML3.5

Substituir o diff-drive por um **quadrúpede A1 com locomoção por pernas real**
(ROS 2 Jazzy + Gazebo Harmonic), com cada parte do sistema em container próprio
e a divisão host x86 / módulo Aquila AM69 explícita desde a primeira fase.

Isto é a **opção C** de uma escolha de três, feita com o custo declarado:
semanas de trabalho, resultado incerto. As alternativas descartadas estão em
"Decisões" abaixo.

---

## Situação atual

| Fase | Nome | Estado | Commit |
|---|---|---|---|
| **F0** | Ponto de retorno, commit do ML3.1 | ✅ **concluída** 14/08/2026 | `3885f2e` |
| **F1** | Containerizar a baseline diff-drive | ⬜ próxima | — |
| **F2** | Spike Go2 dentro do container `sim` | ⬜ | — |
| **F3** | Retarget A1 | ⬜ | — |
| **F4** | Contrato atravessando fronteira de container | ⬜ | — |
| **F5** | Nav2 sobre pernas + modo HIL | ⬜ | — |
| **F6** | Fallback selecionável e testes | ⬜ | — |

**Próximo passo: F1.** Aguardando ordem do operador.

---

## Regras invioláveis desta task

Somam-se às do `CLAUDE.md` do projeto, não as substituem.

1. Gazebo é OGRE 2. Roda no host x86, nunca no módulo. Nenhum container com
   `ros-jazzy-ros-gz` vai para arm64.
2. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` em todos os containers, host e módulo,
   sem exceção.
3. Contrato de tópicos preservado byte a byte: `/demo/cmd_vel`, `/demo/odom`,
   `/demo/scan`, `/demo/camera/image_raw`. Nav2 e percepção não podem saber que o
   robô tem pernas.
4. Nada aqui mede desempenho. QEMU constrói imagem arm64 e nada além disso.
   Latência, jitter e estabilidade de marcha só valem medidos no hardware, e isso
   está fora do escopo do ML3.5.
5. Diff-drive continua selecionável por launch arg, no padrão do `use_meshes`.
6. `demo_perception` não é tocado em nenhuma fase.

**Comportamento:** mudança cirúrgica. Cada linha alterada rastreia até uma fase.
Não melhorar código adjacente, não refatorar o que não está quebrado. Quando uma
premissa cair, **parar e dizer** — premissa que cai em silêncio no meio de
retarget de cinemática é a classe de erro que o ML3.1 já pagou.

---

## Portões

Cada fase para no portão e espera. Não emendar fases.

- **F0** — `colcon build` limpo, 39 testes, árvore limpa, changelog do ML3.1
  descrevendo o que está na árvore. ✅
- **F1** — demo diff-drive de hoje roda inteira em containers, com o mesmo
  resultado de goal Nav2 `SUCCEEDED`. Nenhuma mudança de comportamento.
- **F2** — Go2 upstream, sem modificação, em pé e andando por `cmd_vel` dentro do
  container `sim`. **Falhou aqui, C morre** e voltamos para B (quadrúpede visual
  sobre diff-drive), com F0 e F1 já commitados e válidos.
- **F3** — A1 em pé, estável, responde a `cmd_vel` sem cair.
- **F4** — contrato idêntico ao de hoje, verificado por `ros2 topic list` e por
  tipo de mensagem, com `demo_perception` intocado.
- **F5** — goal Nav2 `SUCCEEDED` com o robô de pernas: primeiro tudo no host,
  depois com Nav2 rodando no módulo.
- **F6** — `robot_type:=quadruped|diffdrive` funcionando nos dois sentidos,
  testes estendidos passando.

---

## F0 — concluída (commit `3885f2e`)

29 arquivos, +2870/−655. Portão batido: `colcon build` limpo (6 pacotes),
`colcon test` 39 testes 0 falhas, árvore limpa.

**Entregue:**

- Changelog do ML3.1 reescrito. A entrada anterior descrevia a montagem peça a
  peça por mesh (com medição via `pycollada` e um `_visuals.xacro`) que **não
  existe na árvore** — foi tentada e abandonada. O que existe é o wrapper sobre
  o TurtleBot 4 upstream.
- Reconciliação de documentação para o eixo host/módulo: `CLAUDE.md` e
  `.ai/CLAUDE.md` atualizados; `compose/{learn,emul,target}.yaml` (os três
  vazios, verificado antes de remover) deletados.
- O modo `emul` foi **descartado** junto. Imagens arm64 seguem construídas sob
  QEMU, mas não há mais compose dedicado para rodar a stack emulada.

**Pendente do operador, herdado do ML3.1:** confirmação visual em RViz2/Gazebo
com GUI (exige sessão gráfica interativa). Não bloqueia F1.

---

## F1 — próxima

Containerizar o diff-drive que já funciona, **sem trocar o robô**. Isso separa
risco de infraestrutura de risco de locomoção: se o compose quebrar aqui,
quebrou por Docker ou DDS, não por marcha. E se F2 falhar, o trabalho de F1
continua valendo para a demo diff-drive.

**Criar:** `docker/{base,sim,nav,perception,viz,tools}/Dockerfile`,
`docker/hw/README.md`, `docker/compose.host.yml`, `docker/compose.module.yml`,
`docker/cyclonedds/{host,module}.xml`, `docker/entrypoint.sh`,
`docker/.env.example`.

**Tocar:** `demo_bringup/launch/{sim,nav,perception,viz}.launch.py` — o guia
invoca quatro launch files que **não existem**; hoje há só `learn.launch.py`
monolítico. Decompor por container. `demo_bringup/setup.py` para instalá-los.

**Renomear:** `docker/` hoje tem os nomes do scaffold antigo (`navigation`,
`simulation`, `hmi`) contra os do guia (`nav`, `sim`, `viz`, `tools`, `hw`).
Todos vazios.

**Não tocar:** `demo_perception` (regra 6), `demo_description`, `demo_navigation`.

### Risco número um de F1

Os delays por timer do `learn.launch.py` (12 s spawn, 15 s bridge, 20 s
perception, 25 s nav) foram calibrados para **processos num só host**.
Atravessando fronteira de container a ordem de subida muda. É o candidato mais
provável a quebrar F1 — e é exatamente o que F1 existe para isolar.

---

## Decisões tomadas

### Base de locomoção: `legubiao/quadruped_ros2_control`

Apache-2.0, `ros2_control` nativo, branch default em ROS 2 Jazzy, suporta
Harmonic. **Todas essas afirmações vêm do README e devem ser confirmadas na
árvore em F2** (ver "A confirmar em F2").

Descartadas:

- **`chvmp/champ`** (BSD-3): ROS 1 apenas (Kinetic/Melodic), último update
  ~jul/2024. Portar seria reescrever middleware + build + camada de controle.
- **`khaledgabr77/unitree_go2_ros2`** e **`RobInLabUJI/unitree_go2_ros2_jazzy`**:
  Jazzy + Harmonic + CHAMP, mas Nav2 marcado "coming soon" **e licença não
  declarada** — bloqueador para demo comercial, mesmo critério que eliminou o
  Tugbot do Fuel no ML3.1.
- **`arjun-sadananda/go2_nav2_ros2`** (registrado no ML2): único CHAMP+Nav2
  demonstrado, mas Humble + Gazebo **Classic**, e compensa erro de odometria
  dobrando a velocidade linear no estimador de estado. Contorno, não calibração.

Verificado em 14/08/2026: continua **não existindo** quadrúpede A1 pronto em
Jazzy + Harmonic + Nav2. A integração com Nav2 (F5) é nossa; ninguém entrega.

### Layout de compose: eixo máquina, não modo

`docker/compose.{host,module}.yml` em vez de `compose/{learn,emul,target}.yaml`.
Decisão do operador. Modos viram profiles do Compose + qual arquivo se invoca em
qual máquina. O modo `emul` foi descartado.

### F1 inserida antes do spike

Adição do operador ao plano original. Justificativa no topo da seção F1.

### F2/F3 revertem uma decisão do ML2

O ML2 decidiu **contra** `gz_ros2_control`, a favor do plugin nativo
`gz-sim-diff-drive-system`, justamente porque o primeiro arrastaria
`ros2_control` + `controller_manager`. F2/F3 revertem isso, e com razão:
quadrúpede não tem plugin nativo equivalente. **Registrar a reversão no
changelog quando F2 fechar**, para não parecer que a decisão do ML2 foi
esquecida.

---

## A confirmar em F2, na árvore clonada — não pelo README

Nada da descrição de `quadruped_ros2_control` entra como fato:

| Afirmação | Como verificar |
|---|---|
| Branch default é Jazzy | `package.xml` / CI na árvore, não o README |
| Suporta Harmonic | dependência `gz-*` real e versão (Harmonic é `gz-sim8`) |
| Licença Apache-2.0 | arquivo `LICENSE` na raiz **e** headers dos fontes vendorizados |
| Não tem config do A1 | `find`/`ls` por `a1` em descrição e config |
| O que vem de `chvmp/robots` | licença da descrição do A1 **e** a licença original de `unitree_ros` de onde deriva |

Licença de repo-pai **não se herda por suposição**. Foi licença não declarada que
matou o Tugbot no ML3.1 e os dois repos Go2 aqui.

Confirmar também: se a base carrega `controller_manager` dentro do processo do
`gz sim` (é o que justifica `sim` ser um container só), e se `gz_ros2_control`
1.2.19 casa com a versão que a base espera.

---

## Ambiente verificado (14/08/2026, host x86)

| Item | Estado |
|---|---|
| ROS 2 Jazzy | instalado nativamente |
| Gazebo Sim | 8.14.0 (Harmonic) |
| `ros_gz`, `ros_gz_bridge`, `ros_gz_sim` | instalados |
| `ros2_control` | **não instalado** — apt tem 4.45.2 |
| `ros2_controllers` | **não instalado** — apt tem 4.40.1 |
| `gz_ros2_control` | **não instalado** — apt tem 1.2.19 |
| Módulo Aquila AM69 | **não acessível nesta sessão** |

Os três de `ros2_control` são pré-requisito de F2 e entram na imagem `sim` em F1.

---

## Colisão de invariantes registrada

`quadruped_ros2_control` documenta que **CycloneDDS conflita com `unitree_sdk2`**
e recomenda FastDDS. A regra inviolável 2 do projeto é `rmw_cyclonedds_cpp`
sempre.

**Não bloqueia o ML3.5:** o SDK só entra com A1 físico, que está fora do escopo.
O container `hw` existe vazio desde F1 para o problema ficar visível no lugar
certo em vez de aparecer como surpresa no bring-up de hardware.

---

## Premissas em vigor

- A spec é `guia-ml35-docker.md`. Onde ela e o plano original divergirem, **o
  guia vence**.
- O módulo não está acessível: nenhum comando com `MODULE_IP`, `scp` ou
  `ssh torizon@` roda até F5, e nada de arm64 é declarado validado sem execução
  real (regra 7 do projeto).
- `eth0` nos XMLs de DDS é **placeholder**. Confirmar com `ip -br link` antes de
  usar; não fixar nome de interface sem verificar.
- `tools` aparece em `docker compose exec tools` no guia §9 mas não está
  declarado no compose §6. Será declarado com `profiles: ["tools"]` e um
  `command` que não encerra.
