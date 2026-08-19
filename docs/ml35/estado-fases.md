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
| **F1** | Containerizar a baseline diff-drive | ✅ **concluída** 14/08/2026 | `5d95934` |
| **F2** | Spike Go2 dentro do container `sim` | ✅ **concluída** 14/08/2026 | (spike descartável, não commitado) |
| **F3** | Go2 na árvore do projeto (era "retarget A1") | ✅ **concluída** 17/08/2026 | `db4e6f3`, `ae3d9a1` |
| **F4** | Contrato atravessando fronteira de container | 🟡 **em andamento** | marcha ativa, trote dinâmico ainda cai |
| **F5** | Nav2 sobre pernas + modo HIL | ⬜ | — |
| **F6** | Fallback selecionável e testes | ⬜ | — |

**Decisão tomada: alvo trocado de A1 para Go2** (ver "F2 — verificação
executada"; a justificativa de licença dada em F2 estava incompleta e foi
corrigida em F3 — ver "F3 — o rastreamento de licença"). **F3 rodou e o portão
bateu**: Go2 em pé, estável, andando por `/demo/cmd_vel` com os pacotes e o
launch do projeto, não mais com o spike. **F4 está em andamento**; checkpoint,
evidências, falha de sintonia e próximos passos em
`docs/results/ml35-f4-parcial.md`.

Plano de movimentação vigente: **`docs/ml35/plano-movimentacao.md`** (19/08/2026).
Substitui `plano-proximos-passos.md`, cujas Fases 1-3 já foram executadas, e
corrige duas conclusões daquele documento — em particular: a árvore TF **não**
fecha, não existe frame `odom`, e isso é bloqueador confirmado de F5. Fase A
(parametrizar a marcha + banco de ensaio versionado) concluída em 19/08/2026;
próxima é a Fase B, o Defeito 2.

Note que F3 **não foi retarget de cinemática**. A troca A1→Go2 eliminou esse
trabalho: o Go2 é o robô nativo da base upstream. F3 virou vendorização
criteriosa + integração.

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
- **F3** — Go2 (não A1, ver decisão de F2) em pé, estável, responde a `cmd_vel`
  sem cair, com os pacotes e o launch do projeto. ✅
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

## F1 — concluída 14/08/2026

Portão batido: **goal Nav2 `SUCCEEDED`** (`error_code: 0`) com a demo inteira em
containers, enviado do container `tools`. `colcon build` limpo (6 pacotes),
`colcon test` **46 testes 0 falhas** (eram 39; +7 do `wait_for_clock`).
Evidência de execução em `docs/results/ml35-f1-execucao.md`.

Taxas medidas, learn mode, host x86: `/clock` 334 Hz, `/demo/odom` 27,8 Hz,
`/demo/scan` 10,0 Hz, `/demo/camera/image_raw` 10,0 Hz,
`/demo/perception/detections` 10,0 Hz. AMCL, bt_navigator, controller_server e
planner_server todos `active`. Contrato de tópicos preservado, `demo_perception`
intocado (regra 6).

**Criado:** `docker/{base,sim,nav,perception,viz,tools}/Dockerfile`,
`docker/hw/README.md`, `docker/compose.{host,module}.yml`,
`docker/cyclonedds/{host,module}.xml`, `docker/entrypoint.sh`,
`docker/.env.example`. Os diretórios do scaffold antigo (`navigation`,
`simulation`, `hmi`) eram todos vazios e não rastreados pelo git — não houve
rename, foi criação.

**Tocado:** `demo_bringup/launch/{sim,nav,perception,viz}.launch.py` (novos),
`demo_bringup/{setup.py,package.xml}`, `demo_bringup/demo_bringup/wait_for_clock.py`
(novo) e seu teste. `demo_navigation/{setup.py,package.xml}` e
`navigation.launch.py` — ver "vendorização" abaixo.

### O risco número um se confirmou, e a decisão foi trocar os timers

Os delays por timer do `learn.launch.py` (20 s perception, 25 s nav) medem o
tempo desde a subida do **próprio** container, que não tem relação fixa com o
momento em que o Gazebo terminou de carregar o mundo. `docker compose up` sobe
tudo junto.

Substituídos pelo nó **`wait_for_clock`** (`demo_bringup`), que aguarda `/clock`
existir **e avançar** antes de liberar Nav2 e perception. Exige duas amostras com
timestamp estritamente crescente: uma só amostra passaria com Gazebo pausado
(`gz sim` sem `-r` inicia pausado), trocando uma falha silenciosa por outra.
Timeout de 120 s, sai != 0 — container que espera para sempre parece travamento,
não falha.

Isto **amplia o escopo de F1** em relação ao "nenhuma mudança de comportamento"
do portão: é código novo, não só empacotamento. Decisão do operador, tomada com
a alternativa (portar os timers como estavam) na mesa. Os 12 s de spawn e 15 s de
bridge **dentro** de `simulation.launch.py` continuam intocados — são
intra-container e o timer ali ainda mede o que deve.

### Três armadilhas encontradas na execução, todas silenciosas

Nenhuma destas aparece como erro que nomeie a causa. Ficam registradas porque
custaram tempo e vão reaparecer.

**1. `${HOST_IP}` em arquivo bind-mounted nunca expande.** O XML do guia §5 usa
`<Peer address="${HOST_IP}"/>`. Docker não substitui variáveis dentro de arquivo
montado, então o CycloneDDS recebe a string literal `${HOST_IP}` como endereço.
Combinado com `AllowMulticast=false`, resultado: **nenhum mecanismo de descoberta
sobrou**, nem entre processos do mesmo container. Sintoma: `ros2 node list` vazio,
`ros2 topic list` só com `/rosout`, e o spawner do Gazebo em
`Waiting messages on topic [robot_description]` para sempre — enquanto o
`robot_state_publisher` logava `Robot initialized` no mesmo container. Correção:
`<Peer address="127.0.0.1"/>`, que é **load-bearing**, não redundante. O IP do
módulo entra em F5 (ver `module.xml`, que hoje só fala consigo mesmo, de
propósito).

Não confundir com forçar `<NetworkInterface name="lo"/>`: isso foi testado e é
**errado** — isola o cliente dos nós que já selecionaram a interface real
(aqui `wlp0s20f3`). Mantém-se `autodetermine`; o peer localhost só adiciona
endereço de descoberta.

**2. `eth0` do guia não existe nesta máquina.** `ip -br link` no host dá `lo`,
`enp0s31f6` (DOWN), `wlp0s20f3` (UP, Wi-Fi), `tailscale0`, `docker0`. Os dois
XMLs usam `autodetermine` em vez de nome fixo, com o procedimento de verificação
comentado no arquivo. Como o host está em Wi-Fi, o caso "multicast morre" é o
esperado, não o excepcional.

**3. `GZ_SIM_RESOURCE_PATH` vazio no container.** `demo_description` referencia
as malhas como `model://nav2_minimal_tb4_description/meshes/*.dae`. Nativamente
resolve pelo ambiente ROS ambiente; no container não. Sintoma: robô spawna com
colisão e inércia corretas — física e navegação funcionam — e **sem corpo
visível**. Robô invisível na GUI mas presente para o planner. Corrigido com
`ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share` no `sim/Dockerfile`; erros de
mesh de N para 0.

### Vendorização de quatro launch files do Nav2 (regra 1)

`ros-jazzy-nav2-bringup` **hard-depends** de `nav2-minimal-tb3-sim`,
`nav2-minimal-tb4-sim`, `ros-gz-sim` e `navigation2`. Medido: colocou
`libogre-1.9`, `gz-ogre-next-vendor`, `gz-rendering`, `gz-gui` e 30+ pacotes na
imagem `nav` — **3,7 GB e OGRE 2 numa imagem que vai para o AM69**, violação
direta da regra 1. `--no-install-recommends` não ajuda: são `Depends`.

`ros-jazzy-navigation2` (o metapacote) tem o mesmo problema um nível abaixo, via
`nav2-rviz-plugins` → `rviz-ogre-vendor`.

Solução: os quatro launch files que `navigation.launch.py` precisa
(`bringup`, `localization`, `navigation`, `slam`) estão vendorizados em
`demo_navigation/launch/nav2_vendored/`, Apache-2.0, cabeçalhos de copyright
intactos, **só os caminhos de raiz de pacote re-rooteados**. Os servidores Nav2
entram individualmente no Dockerfile. Provenance e as edições exatas em
`nav2_vendored/README.md`.

Resultado: `nav` de 3,7 GB → **2,48 GB**, e **zero** pacotes OGRE/RViz/Gazebo.
Verificados os 18 plugins declarados em `nav2_params.yaml` — todos resolvem via
pluginlib na imagem (plugin ausente não é erro de build: falha na transição de
lifecycle).

**Custo aceito:** a lista de servidores no `nav/Dockerfile` e em
`demo_navigation/package.xml` agora acopla com `nav2_params.yaml`. Plugin novo
de pacote não listado exige crescer as duas listas. Está comentado nos dois
lugares.

### Tamanhos das imagens

| Imagem | Tamanho | Vai para o módulo? |
|---|---|---|
| `base` | 912 MB | é a base de todas |
| `perception` | 912 MB | **sim** (arm64) |
| `tools` | 952 MB | sim (arm64) |
| `nav` | 2,48 GB | **sim** (arm64) |
| `sim` | 2,47 GB | não, x86 apenas |
| `viz` | 2,7 GB | não, x86 apenas |

`nav` a 2,48 GB continua gordo para partição de dados do Torizon. Não é violação
de regra nenhuma (não há mais nada gráfico), é peso. Dieta adicional, se
necessária, é trabalho de F5 — é quando `nav` de fato promove para arm64 e vai
ao módulo.

### Não validado nesta fase

- **Nada em arm64.** Nenhuma imagem arm64 foi construída em F1; os `platform:`
  estão declarados e o `compose.module.yml` está escrito, mas não executado.
  Regras 5 e 7.
- **Módulo inacessível** nesta sessão. `compose.module.yml` e
  `cyclonedds/module.xml` são código não executado.
- **Confirmação visual em GUI.** Mapear `/dev/dri` não bastava: `renderD128` é do
  grupo `render` (gid 992 neste host) e o usuário `ubuntu` do container está em
  `video`. Sintoma: `libEGL warning: failed to open /dev/dri/renderD128:
  Permission denied` e queda silenciosa para render em software — a demo roda, só
  devagar. Corrigido com `group_add: ["${RENDER_GID:-992}"]` em `sim` e `viz`;
  verificado que com o gid o device é legível e sem ele o open falha. Após a
  correção: 0 erros de libEGL, 0 erros de mesh, goal `SUCCEEDED`.

  **`RENDER_GID` é específico do host** (`getent group render | cut -d: -f3`). O
  default 992 no compose vale para esta máquina. **Pendência aberta:** o
  `docker/.env.example` não documenta a variável. O arquivo está bloqueado por
  regra de permissão do ambiente (`.env*` é negado para leitura e para shell),
  confirmado em duas sessões — não é transiente. **Correção é manual, do
  operador:** acrescentar ao `docker/.env.example`

  ```
  # gid do grupo `render` DESTE host: getent group render | cut -d: -f3
  # Sem isto, sim e viz caem para render em software sem erro que nomeie a causa.
  RENDER_GID=992
  ```

  O comentário explicativo já está nos dois serviços do `compose.host.yml`, que
  é onde a variável é consumida.

  O que continua **não verificado por olho humano**: se o robô aparece correto no
  Gazebo e no RViz2. Os erros de mesh zeraram e o render é acelerado, mas
  ninguém olhou a tela. Herdado do ML3.1 e ainda pendente do operador.

---

## F2 — verificação executada 14/08/2026, parada em bloqueador

Clone raso de `legubiao/quadruped_ros2_control` em `/tmp/f2-spike`, HEAD
`5434c58` ("x30 repaint"). A tabela "A confirmar em F2" foi percorrida inteira
**antes** de escrever qualquer código, e é bom que tenha sido: duas afirmações do
plano caíram, e uma delas bloqueia.

### Resultado da tabela de verificação

| Afirmação do plano | Verificado na árvore | Veredito |
|---|---|---|
| Branch default é Jazzy | branch default é `main`; README linha 10 diz "developed under ROS2 Jazzy", Humble tem branch própria | ✅ na prática |
| Suporta Harmonic | `gz_quadruped_hardware` depende de `gz_sim_vendor`/`gz_plugin_vendor`; `descriptions/README.md` §2 pede `ros-jazzy-ros-gz` + `ros-jazzy-gz-ros2-control` | ✅ |
| Licença Apache-2.0 | raiz é Apache-2.0, e **todo o código** (controllers, commands, libraries, hardwares) declara Apache-2.0 | ✅ **só para o código** |
| Não tem config do A1 | **falso.** `descriptions/unitree/a1_description/` existe, completa | ❌ premissa caiu |
| O que vem de `chvmp/robots` | **nada.** As descrições não vêm de `chvmp/robots`; o A1 tem maintainer `laikago@unitree.cc`, ou seja, origem Unitree direta | ❌ premissa caiu |

### O bloqueador: `a1_description` declara `<license>TODO</license>`

Este é o achado que para a fase. A licença **por pacote**, medida no
`package.xml` de cada um dos 24 pacotes:

| Pacote | Licença declarada |
|---|---|
| todo o código (11 pacotes: controllers, commands, libraries, hardwares) | `Apache-2.0` |
| `go2_description` | `BSD` |
| `b2_description`, `magicdog_description` | `BSD` |
| `anymal_c_description` | `BSD-3` |
| `lite3_description`, `x30_description` | `MIT` |
| **`a1_description`** | **`TODO`** |
| `go1_description`, `aliengo_description`, `cyberdog_description` | `TODO` |

A raiz ser Apache-2.0 **não cobre** o `a1_description`: licença de repo-pai não
se herda por suposição — é a regra que já matou o Tugbot no ML3.1 e os dois repos
Go2 na escolha da base. Não há header de copyright em nenhum arquivo do
`a1_description` (nem no `robot.xacro`, nem no `robot.urdf` autogerado). O único
sinal de origem é o maintainer `laikago@unitree.cc`. `LICENSES/` na raiz cobre só
`legged_control` e `unitree_guide` — nenhuma descrição de robô.

**Consequência prática:** o A1 é o robô-alvo da demo. F3 vendoriza justamente
essa descrição. Vendorizar arquivo sem licença declarada numa demo comercial da
Toradex é exatamente o risco que o projeto já decidiu não correr, duas vezes.

### O que isso *não* bloqueia

O portão de F2 é **Go2**, e `go2_description` declara **BSD** — licença válida,
e é a descrição que o spike usaria. O bloqueio é de F3 em diante, não do spike em
si. Mas rodar F2 sem resolver isto significa gastar a fase de spike para provar
uma base cujo destino (A1) está juridicamente indefinido.

Não escrevi código porque a decisão muda o alvo do trabalho, não só a ordem dele.

### Caminhos possíveis — **decisão tomada: caminho 1**

1. **[ESCOLHIDO] Trocar o robô-alvo de A1 para Go2.** `go2_description` é BSD,
   tem config de `ocs2`, `legged_gym`, `himloco` e `robot_lab`, e é o robô mais
   exercitado do repo — inclusive com `gazebo_rl_control.launch.py` próprio, que
   o A1 não tem. Elimina o bloqueador e reduziu o risco de F3, que é retarget.
   Custo aceito: o pedido original nomeia A1; a demo passa a chamar-se
   quadrúpede Go2.
2. Rastrear a licença real do A1 upstream (`unitree_ros`) e seguir se for
   BSD-3 — não seguido, custo de rastreamento não compensava com Go2 disponível.
3. Aceitar o risco explicitamente — não seguido.
4. Voltar para a opção B (quadrúpede visual sobre diff-drive) — não seguido.

Se o A1 for requisito duro de nome da demo, o caminho 2 vira pré-requisito antes
de F3 vendorizar a descrição — mas nada em F2/F3 tecnicamente exige A1
especificamente; o contrato de tópicos (F4) não distingue os dois.

## F2 — spike executado 14/08/2026, portão batido

Imagem descartável `demo-sim:spike-go2` (Dockerfile em `/tmp/f2-spike`, **não
commitado** — é spike, não entra na árvore). `ros:jazzy-ros-base` +
`ros-gz-sim`/`ros-gz-bridge`/`ros2-control`/`ros2-controllers`/`gz-ros2-control`
do apt (só para satisfazer headers de build; o plugin que roda de fato é o
`gz_quadruped_hardware` **do próprio clone**, não o do apt — ver achado abaixo),
clone raso de `quadruped_ros2_control` com os pacotes que o spike não builda
removidos antes do `rosdep install` (só remoção do que não se builda: nada do
que o spike usa foi tocado). Build via `colcon build --packages-up-to
go2_description unitree_guide_controller keyboard_input gz_quadruped_playground`.
7 pacotes, build limpo.

**Launch de spike** (`/spike/spike_go2.launch.py`, também não commitado) reflete
`unitree_guide_controller/launch/gazebo.launch.py` upstream sem modificá-lo,
com duas mudanças deliberadas: RViz2 removido (regra 1 — o `viz` do projeto real
fica no host, fora do container `sim`) e Gazebo headless (`-s`, sem GUI). Uma
**ponte de spike** (`twist_to_inputs.py`, idem) traduz `/demo/cmd_vel`
(`geometry_msgs/Twist`, o nome real do contrato) para `/control_input`
(`control_input_msgs/Inputs`), que é o que o controlador de fato aceita —
achado já registrado abaixo. A ponte também percorre a máquina de estados
(`PASSIVE → FIXEDDOWN → FIXEDSTAND → TROTTING`) com 5 s de espera real entre
cada comando — insuficiente na primeira tentativa (ver "armadilha" abaixo).

### Resultado, medido por `gz topic -e -t .../dynamic_pose/info`, não por log

| Momento | z (altura) | Orientação | Interpretação |
|---|---|---|---|
| Antes de qualquer comando (spawn) | ~0.5 (spawn height) | — | — |
| Após FIXEDSTAND, antes de TROTTING | **0.353 m** | quase identidade | **em pé, estável** |
| Em TROTTING parado (`cmd_vel`=0) | 0.15 m | identidade | marcha em posição mais baixa, mas não caiu |
| Andando, `linear.x=0.03` (ganho baixo), 8 s contínuos | **0.343 m sustentado** | quase identidade | **anda de pé, estável, sem cair** |
| Andando, `linear.x=0.15–0.3` (ganho alto) | cai para 0.07–0.24 m, orientação tomba | robô perde equilíbrio | **sintonia, não falha estrutural** |

**Portão batido no ganho baixo**: Go2 upstream, sem modificação, em pé e
andando por `cmd_vel` dentro de um container moldado como `sim`. Zero erros no
log da execução inteira (`grep -c "Err\]"` = 0).

**A queda em ganho alto não bloqueia o portão.** O guia já registrava o risco
antes de rodar: *"parâmetro de marcha sintonizado... produz robô que anda mal
sem gerar erro... O portão de F3 é robô em pé e estável respondendo a `cmd_vel`,
não build limpo."* A causa provável é a ponte de spike ser um mapeamento linear
ingênuo de `Twist` para o joystick normalizado `-1..1` do `Inputs`, sem os
limites de velocidade (`v_x_limit_`) que a UI de joystick real respeitaria —
**isto é responsabilidade de F4** (a ponte real do contrato), não de F2.

### Uma armadilha silenciosa nesta fase também

Testar a máquina de estados manualmente via `ros2 topic pub .../control_input`
**enquanto a ponte de spike do launch ainda rodava em paralelo** produziu dois
publishers competindo pelo mesmo tópico e um retrocesso de estado
(`trotting → fixed stand → fixed down`) que parecia instabilidade do
controlador e não era — era dois processos de teste disputando o mesmo
`/control_input`. Diagnosticado lendo `StateTrotting::checkChange()`
diretamente (linha 76-84 do arquivo): `command==2` força volta a
`FIXEDSTAND` mesmo em trote estável. Corrigido isolando um único publisher por
teste. Registrado porque é o tipo de falha que "parece o robô caindo" quando na
verdade é o harness de teste.

### Outros fatos coletados no clone, para F3 em diante

- **`gz_quadruped_hardware` é do próprio repo**, versão 2.0.6, licença
  `Apache 2`, mantido por Alejandro Hernández / Bence Magyar (é um fork do
  `gz_ros2_control` upstream). O plano supunha usar `gz_ros2_control` 1.2.19 do
  apt — **não é isso que a base usa**. Confirmar qual dos dois entra na imagem
  `sim` antes de F2 rodar; instalar o do apt e esperar que a base o use é uma
  suposição não verificada.
- `unitree_guide_controller/launch/gazebo.launch.py` sobe **RViz2 dentro do mesmo
  launch** (nó `rviz_ocs2`). Isso é OGRE 2: na nossa arquitetura o RViz vive no
  container `viz`, não no `sim`. O spike terá de desligar esse nó — é regra 1.
- O launch aceita `pkg_description:=<pacote>` e `height:=<z inicial>`. O README do
  A1 usa `height:=0.43`; o parâmetro é o z de spawn, e existe porque quadrúpede
  spawnado no chão cai.
- A colisão CycloneDDS × `unitree_sdk2` está **confirmada no README** (linhas
  37-40), recomendando FastDDS. Segue não bloqueando o ML3.5 — o SDK só entra com
  A1 físico, fora do escopo — e o container `hw` já existe para isso.

---

## F3 — concluída 17/08/2026 (commits `db4e6f3`, `ae3d9a1`)

Portão batido, medido por `gz topic -e -t .../dynamic_pose/info`, nunca por log:

| Momento | z (altura) | x | Interpretação |
|---|---|---|---|
| Em pé após a FSM | **0,352 m** | 0,041 | em pé, estável |
| Andando, `linear.x=0.03`, 12 s | **0,351 m sustentado** | 0,041 → **0,216** | anda de verdade, sem perder altura |
| Orientação final | `-7,2e-05` | — | praticamente nivelado |

0 erros no Gazebo, 3 controladores `active`. **Melhor que F2**, que via a altura
cair de 0,353 para 0,343 durante a marcha — a troca de timers por cadeia de
eventos deixou a subida mais limpa.

### F3.0 — o spike de F2 tinha sumido

`/tmp/f2-spike` foi levado pela limpeza do `/tmp`. Os três arquivos nunca foram
commitados (decisão de F2: spike não entra na árvore). Recuperados da imagem
`demo-sim:spike-go2`, que sobreviveu: os dois fontes por `docker cp`, e o
Dockerfile reconstruído camada a camada de `docker history --no-trunc`. Cópia em
`scratchpad/f2-recovered/`.

**Lição:** conhecimento que só existe em `/tmp` não existe. Se um spike futuro
importar, ou se commita, ou se aceita perdê-lo.

### O rastreamento de licença — a justificativa de F2 estava incompleta

F2 trocou A1 por Go2 registrando que "`go2_description` declara **BSD** —
licença válida". Verdadeiro, mas insuficiente, e medido de novo na hora de
vendorizar:

| Evidência | `a1_description` (rejeitado em F2) | `go2_description` (escolhido) |
|---|---|---|
| `<license>` | `TODO` | `BSD` |
| Arquivo `LICENSE` | ausente | **ausente** |
| Header de copyright | ausente | **ausente** |
| Autor / maintainer | `laikago@unitree.cc` | **`TODO` / `TODO@email.com`** |
| Coberto por `LICENSES/` da raiz | não | **não** |

O Go2 era melhor que o A1 em **um** campo, e pior em outro (o A1 ao menos
apontava um maintainer rastreável). "BSD" sozinho não identifica a variante, e
todas exigem reproduzir um aviso de copyright que não existia no pacote.

**Resolvido rastreando até a origem real:** `unitreerobotics/unitree_ros`,
BSD 3-Clause com texto completo e titular identificado (HangZhou YuShu
TECHNOLOGY CO.,LTD., 2016-2022). As **7 malhas são bit-idênticas** ao upstream,
provado por hash git blob contra a API do GitHub. Tabela completa e comandos de
reprodução em `ros2_ws/src/go2_description/README.md`.

A camada xacro **não** bate com o upstream (é port ROS 1 → ROS 2 do `legubiao`).
Adotada como obra derivada coberta pelo BSD-3, com o risco residual registrado
explicitamente no README em vez de apagado.

Mesma coisa na camada de controle: os `package.xml` declaram Apache-2.0, mas os
três pacotes derivados do `unitree_guide` são cobertos por
`LICENSES/unitree_guide/LICENSE.txt` da raiz upstream, que é **BSD-3 da
Unitree** — mesmo titular das malhas. Declarações corrigidas e o texto copiado
para dentro de cada pacote. Ver `unitree_guide_controller/PROVENANCE.md`.

### A armadilha silenciosa desta fase

Copiei do plant diff-drive a `TimerAction` de 12 s antes do spawn. Medido:

```
spawn em z=0.49999 → z=0.0677 em menos de 1 s → controladores ativam ~3 s depois
```

O robô passa a janela inteira em **queda livre sem controlador** e desaba.
Estado final: colapsado no chão, **três controladores reportando `active`, zero
erros no log**, e a FSM de marcha percorrendo `passive → trotting` em cima de um
robô já caído.

Nenhum sinal de log denuncia isso. Só a pose lida direto do `gz`. É exatamente o
que o portão de F3 existe para pegar — *"robô em pé e estável respondendo a
`cmd_vel`, não build limpo"* — e valida a decisão de medir por pose.

**Correção:** spawn imediato, encadeado por `OnProcessExit`
(`spawn → broadcasters → controlador de marcha`), sem timer. É o que o
`gazebo.launch.py` upstream já faz. O comentário longo em
`quadruped.launch.py` explica por que ali não pode haver `TimerAction`.

### Criado / tocado

**Vendorizados** (5 pacotes, nomes upstream preservados para que os `$(find)`
resolvam sem edição): `go2_description` (25 MB), `control_input_msgs`,
`controller_common`, `unitree_guide_controller`, `gz_quadruped_hardware`.
Procedência em `go2_description/README.md` e
`unitree_guide_controller/PROVENANCE.md`.

**Nosso:** `demo_simulation/launch/quadruped.launch.py`,
`demo_simulation/demo_simulation/twist_to_inputs.py` (promovido do spike),
`demo_bringup/launch/sim.launch.py` (roteia `robot_type` → um launch por plant,
sem conditionals), `docker/sim/Dockerfile`.

**Lint de estilo desativado** nos dois pacotes C++ vendorizados: `ament_lint_auto`
rodava sobre código de terceiro e produzia 98 falhas em código que a política
manda não editar. Corrigir destruiria o byte-idêntico; deixar torna
`colcon test` vermelho para sempre. Nossos pacotes mantêm os seus linters.

### Não validado nesta fase

- **Nada em arm64, nada no módulo.** Regras 5 e 7.
- **RViz2 continua pendente.** A execução quantitativa do portão foi headless.
  Em 17/08/2026 o operador repetiu o launch com `gui:=true` na imagem do spike
  e confirmou o modelo Go2 visível no Gazebo, em `empty.sdf`. Isso fecha a
  visualização do modelo no Gazebo, mas não valida a árvore TF e o RobotModel
  no RViz2.
- **O quadrúpede no mundo `warehouse.sdf`.** O portão rodou em `empty.sdf`. O
  mundo do projeto carrega ~10 s e tem 50+ malhas; o spawn agora é imediato, o
  que é seguro (`create` faz retry), mas não foi exercitado ali.
- **Marcha em ganho alto.** Continua o que F2 mediu: acima de ~0,15 m/s o robô
  perde equilíbrio. É sintonia do mapeamento em `twist_to_inputs`, e é **F4**.
- **Nav2 sobre pernas.** F5. O plant não publica `odom → base_link`.

---

## F4 — em andamento 17/08/2026, atualizada 18/08/2026

Checkpoint detalhado em `docs/results/ml35-f4-parcial.md`. Nomes, tipos e
mensagens reais de odom, scan e imagem atravessaram dois containers por DDS. O
primeiro mapeamento SI → stick derrubou o Go2 e foi substituído por clamp no
envelope `0.03` comprovado em F3, mas essa última edição ainda não foi
revalidada em runtime. Perception, warehouse oficial e regressão diff-drive
seguem pendentes; portanto o portão de F4 permanece aberto.

**18/08/2026 — a marcha passou a existir.** `StateTrotting` foi separado em
`WALK`, `HOLD` e `RECOVER`, e `twist_to_inputs` ganhou watchdog de comando. Três
falhas estavam sobrepostas e uma escondia as outras:

1. o gate de passada do upstream pedia `|v| > 0.03 m/s` e o caminho de comando
   inteiro entrega no máximo `0.012 m/s` — nenhuma passada era pedida, e o
   `contact=[1 1 1 1]` registrado antes era isso, não dinâmica;
2. `pcd_` é referência integrada e não era recapturada ao parar, então o QP
   continuava acelerando o corpo depois do comando zerar;
3. `Inputs` não tem timeout: um publisher que simplesmente para deixava o robô
   andando com um comando que ninguém enviava.

Medido: `mode=WALK` já em `Twist linear.x=0.01`, pares diagonais alternando,
`HOLD` estável por mais de 35 s com `posErrXY ≈ 0,005 m`.

**18/08/2026 — o robô anda.** 30 s de trote contínuo a `v_cmd = 0,1 m/s`, 3,00 m
percorridos, nenhuma entrada em `RECOVER`, tilt máximo 2,3°, velocidade média
medida 0,106 m/s. Duas causas, ambas medidas antes de qualquer ajuste:

1. **O comando estava fora do regime da marcha.** `_SAFE_STICK_LIMIT = 0.03`
   estava documentado como "envelope estável de F3", mas foi medido enquanto a
   marcha nunca ativava — descrevia o empurrão sobre um robô de pés plantados,
   não velocidade de caminhada. A `v_cmd = 0,004 m/s` o passo pedido é de 4 mm
   sob elevação de pé de 8 cm: o robô marchava no lugar. Elevado para `0.5`.
2. **O rumo não é controlável pelo QP neste robô.** Instrumentando `bd_` contra
   `A_ * F_`, o momento de guinada pedido ficava travado em ±5,3 N·m = o batente
   `d_wbd(2) ±10 rad/s²` vezes `Izz`. Com `kp_w_ = 780` esse batente satura com
   **0,73°** de erro de guinada, e acima disso o sinal passa a ser escolhido pela
   ondulação do giroscópio, não pelo erro. Guinada em robô com pernas se controla
   com onde o pé pousa: `k_yaw_` em `FeetEndCalc` valia 0,005 contra os 0,1125
   que precisa cancelar no instante do pouso, então o padrão de apoio era
   assentado girado e as pernas cruzavam para o centro. `k_yaw_ = 0.15` resolve.

Alargar o batente (±25) e desmembrar os ganhos de atitude por eixo foram
ensaiados e **rejeitados por medição** — evidência em `ml35-f4-parcial.md`.

F4 segue aberta: deriva de guinada de ~3°/s parado em `HOLD`, viés de 24 mm no
`z` estimado (`foot_radius` contra `feet_h_ = 0`), warehouse, RViz2/TF e
regressão diff-drive. Os gates de `linear.x = 0.01` e `0.03` herdaram a premissa
nula do item 1 e precisam ser reescritos em termos de `v_cmd`.

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

## A confirmar em F2 — ✅ EXECUTADO, ver "F2 — verificação executada" acima

A tabela abaixo é o que se pretendia verificar. Foi verificada em 14/08/2026 e
**duas afirmações caíram**. Mantida como registro do que se perguntou; os
resultados estão na seção de F2.



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
