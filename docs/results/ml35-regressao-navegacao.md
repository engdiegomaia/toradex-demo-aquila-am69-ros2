# ML3.5 — regressão de navegação: causa raiz e correções (25/08/2026)

**Máquina:** estação x86 (amd64), modo `learn`, Gazebo + Nav2 + percepção em
container. **Mundo:** `quadruped_maze11.sdf`.

> **Nada aqui vale para o Aquila AM69.** Regras 5 e 7 do `CLAUDE.md`. As duas
> causas raiz abaixo são de configuração e de plugin, não de desempenho, então
> valem para os dois lados — mas os *números* são todos do host.

## O que se pediu

> "o mapa oficial deve ser o labirinto. a navegação parece ter piorado, revise os
> commits e verifique as melhorias de navegação e estabilidade realizadas.
> compile uma versão final melhorada que será usada para validar na integração
> com o hardware."

## Resultado em uma linha

A navegação piorou de verdade, e **não** por perda de sintonia: a sintonia V3
está intacta no repositório. Foram **duas falhas de infraestrutura**, ambas
silenciosas, ambas encontradas por medição:

1. **`route_server` derruba o `nav2_container` com SIGSEGV** — e não só em
   `RESET`+`STARTUP` como se acreditava, mas também em **cold start normal**, de
   forma intermitente. Quando acontece, morrem todos os servidores do Nav2 de uma
   vez e `docker compose ps` continua dizendo `running`.
2. **Descoberta DDS assimétrica entre containers** — o `nav` recebia do `sim`, o
   `sim` não recebia do `nav`. O robô trota parado com o Nav2 comandando.

## Primeiro: a sintonia V3 não regrediu

Conferido contra `ml35-navegacao-maze11.md`, valor por valor:

| | documentado (V3) | no repositório hoje |
|---|---|---|
| MPPI `vx_min` | 0.0 | **0.0** |
| MPPI `wz_max` | 0.20 | **0.20** |
| MPPI `vx_max` | 0.15 | **0.15** |
| inflação global | 0.85 / 2.0 | **0.85 / 2.0** |
| inflação local | 0.55 / 5.0 | **0.55 / 5.0** |
| `velocity_smoother.min_velocity[0]` | 0.0 | **0.0** |
| `robot_radius` | 0.38 | **0.38** |
| marcha `k_yaw` | 0.35 | **0.35** |
| `SmoothPath` no BT | sim | **sim** (injetado por `RewrittenYaml`) |

`git log` de `nav2_params_go2.yaml`: último toque em `ff9ae4b`, que é o commit da
própria sintonia. Nada a recuperar.

## A medição que mostrou o tamanho do problema

Protocolo idêntico ao da tabela V0/V3: `nav_trial.py`, 180 s de tempo de
simulação, `--goal-timeout 90`, as 4 metas de patrulha do maze11.

| Métrica | V3 (21/08, n=3) | 25/08, antes das correções |
|---|---|---|
| velocidade média | **0,0650 m/s** | **0,0044 m/s** |
| caminho percorrido | 11,04–12,47 m | **0,79 m** |
| `cmd_vx` pico / médio | 0,143 / +0,052 | **0,038 / +0,0002** |
| `cmd_vx` negativo | 0% | 0% |
| metas concluídas | — | **0 de 1** (prazo em t=92 s) |
| tilt pico | 0,5–0,9° | 0,95° |
| quedas | 0 | 0 |

15× mais lento. E o descarte de hipóteses foi mais informativo que o número:

- **fator de tempo real = 0,962** (medido em `/clock`: 14,93 s de simulação em
  15,52 s de parede). Não é falta de CPU.
- **percepção não estava envenenando o costmap.**
  `/demo/perception/detection_cloud` publicou **0 mensagens em 60 s**. A hipótese
  era plausível — a `perception_layer` tem `clearing: false, marking: true`, e um
  stub que detecta sempre encheria o costmap de fantasmas permanentes — e está
  **descartada por medição**. (O risco continua existindo no dia em que o stub
  voltar a publicar; fica registrado.)
- **a marcha estava em trote**, 317 amostras de `state=trotting`, tilt < 1°.

O que a série temporal mostrou:

```
cmd_vx  não-nulo em    3,6% das amostras
cmd_wz  não-nulo em   95,6% das amostras   (pico 0,120 rad/s)
yaw_deg  90 71 47 47 47 49 49 64 65 59 59 59 59 59 59 59 59 100
posição  início (0,00; 0,04)   fim (-0,07; -0,07)
```

O robô **girou 150° no total e ficou congelado em yaw 59° de t≈70 s a t≈170 s**,
enquanto o Nav2 comandava guinada em 95,6% das amostras. E no log da marcha,
durante todo o ensaio:

```
sticks=(lx=0.0000 ly=0.0000 rx=0.0000)   contact=[1 1 1 1]
```

Comando saindo do Nav2, comando **não chegando** ao robô. Daí "Failed to make
progress" e recuperação `Spin`, que também não chegou
(`Exceeded time allowance before reaching the Spin goal`).

## Causa raiz 1 — descoberta DDS assimétrica

`docker/cyclonedds/host.xml` selecionava a interface com
`<NetworkInterface autodetermine="true"/>`. Este host tem quatro: `192.0.2.4`
(LAN), `100.64.184.75` (Tailscale), `192.0.2.7` e `192.0.2.9` (bridges Docker).

O próprio arquivo já previa a falha, no cabeçalho:

> autodetermine picks the highest-priority interface CycloneDDS can find. That is
> usually right on a single-NIC machine and NOT reliably right here, where
> docker0 and tailscale0 are also up and can be chosen ahead of the intended
> link.

E a armadilha 6 do `guia-operacao.md` já documentava exatamente isto — **só para
o lado do módulo**. O lado do host ficou com `autodetermine`.

O sintoma medido é entrega **assimétrica**: o `nav` assinava `/demo/odom` e
`/demo/scan_cloud` do `sim` e recebia (os locators do `sim` eram alcançáveis); o
`sim` assinava `/demo/cmd_vel` do `nav` e **não** recebia. Por isso o Nav2 tinha
odometria e costmap suficientes para planejar e comandar, e o robô não se movia.

O log tinha, sim, uma pista — e ela é ilegível na prática:

```
ddsi_udp_conn_write to udp/192.0.2.4:24672 failed with retcode -1
```

**81 248 linhas** disso num único container, misturadas à saída do Gazebo. Não é
um erro que alguém lê; é ruído que esconde o resto (evictou o log da marcha
inteiro da janela de retenção do Docker).

Havia também um par **obsoleto**, `192.0.2.15`, de quando este host estava em
outra rede: os containers estavam de pé desde antes da troca de IP. Reiniciar a
pilha removeu esse par e **não** resolveu a assimetria — o que isolou a causa na
seleção de interface, não no par velho.

### Correção

`host.xml` passa a listar o **loopback explicitamente e primeiro**:

```xml
<Interfaces>
  <NetworkInterface name="lo" priority="10" multicast="true"/>
  <NetworkInterface autodetermine="true" priority="default"/>
</Interfaces>
```

Toda a pilha do modo `learn` roda com `network_mode: host` na mesma máquina, então
o caminho correto entre containers é `lo`, e ele deixa de depender de sorteio.
`lo` existe em qualquer máquina — é o único nome de interface seguro de cravar, e
por isso isto não é o "nome cravado" que o cabeçalho do arquivo condena. Multicast
fica ligado **só** nele: em loopback não atravessa switch, então a objeção de
"multicast morre no Wi-Fi" não se aplica. A interface real continua na lista, para
o modo `hil` alcançar o Aquila.

O `awk` de `render_host_config` (`scripts/module.sh`) substitui apenas a linha do
`autodetermine`, então a linha do loopback **sobrevive à renderização** e a
correção chega ao `hil` também.

### Dois defeitos de processo achados no caminho

- **`scripts/env.sh` não definia `CYCLONEDDS_URI`.** Ferramenta nativa no host
  usava o *default* do CycloneDDS (multicast ON, interface automática) enquanto os
  containers usavam `host.xml` (multicast OFF, peer explícito). Duas políticas de
  descoberta no mesmo domínio funcionam enquanto a interface escolhida por
  acidente coincide. Sintoma: `nav_trial.py` abortando com "navigate_to_pose não
  apareceu" enquanto `docker compose logs nav` dizia `Managed nodes are active` e
  `/demo/odom` chegava a 49 Hz — **metade do grafo visível, metade não**.
  Corrigido: `env.sh` aponta para a mesma config (o renderizado quando existe, o
  template quando não).

- **Editar `host.xml` não surtia efeito.** O compose monta `host.rendered.xml`,
  gerado por `module.sh sync` (o endereço do módulo não entra em git). O
  renderizado desta bancada era byte-idêntico ao template e ficou parado enquanto
  o template mudava. Em `learn` não havia caminho para regenerá-lo, porque `sync`
  exige `MODULE_IP` e faz push para o módulo. Corrigido: novo
  **`scripts/module.sh render-local`**, e um teste que reprova um renderizado mais
  antigo que o template.

## Causa raiz 2 — o `route_server` mata o Nav2 no boot

O segfault do `route_server` estava registrado como "acontece em `RESET`+`STARTUP`"
(descoberto em 24/08 ao implementar o reset do cockpit). Medido agora em **cold
start normal**:

```
[route_server]: Created route operation ReroutingService of type nav2_route::ReroutingService
[route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [pid 40, exit code -11, ...]
```

`-11` é `SIGSEGV`. É **intermitente**: o mesmo `docker compose up nav` subiu numa
vez e derrubou a pilha na seguinte.

O que torna isto grave para uma validação de hardware é o estado que ele deixa:

```
$ docker compose ps
nav   running                      <- mentira útil
$ docker exec <nav> ps -eo comm,etime
ros2   09:07:53                    <- só o pai; nenhum nó filho
```

Morrem **todos** os servidores do Nav2 de uma vez, o container segue `running`, e
de dentro dele `get_node_names()` lista apenas os 12 nós do `sim`. Quem estiver
olhando o compose conclui "o Nav2 está de pé" e quem estiver olhando o robô
conclui "a navegação piorou".

### Por que não dá para tirar o `route_server` da lista

`navigation_launch.py` é cópia vendorizada e tem de seguir idêntica ao upstream
(proveniência — `launch/nav2_vendored/README.md`). Ele passa a lista ao
gerenciador **cravada no launch**:

```python
parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}]
```

Parâmetro inline vence arquivo de parâmetros, então `node_names` não é
sobreponível pelo YAML. O `route_server` vai ser configurado de qualquer forma.

### Correção (contorno, e está marcado como tal)

O `route_server` **lê a própria seção** do arquivo de parâmetros, e essa não é
sobreposta. `operations` é a lista de plugins de operação de rota; o default inclui
`ReroutingService`. Nova seção em `nav2_params_go2.yaml`:

```yaml
route_server:
  ros__parameters:
    operations: ["AdjustSpeedLimit"]
```

O plugin que estoura nunca é construído. Esta demo não usa roteamento por grafo,
então não se perde função nenhuma.

Não é `[]`: lista YAML vazia chega ao launch como tupla Python e derruba tudo com
`Expected 'value' to be one of [float, int, str, bool, bytes], but got '()'` —
armadilha 5 do `guia-operacao.md`.

**A causa está no `nav2_route` e é candidata a issue upstream.** Isto é contorno.

## O labirinto como cenário oficial

Pedido explícito, e ele também remove uma falha silenciosa.

Antes: o default de `sim.launch.py` era o armazém para os **dois** robôs, e o
labirinto exigia um `SIM_ARGS` de **seis linhas** com dez números colados à mão —
a **quarta** cópia deles no repositório. Colar metade do bloco dá mundo do
labirinto com câmera do armazém, e o painel azul do cockpit passa a olhar chão
vazio ao lado do labirinto, sem erro e sem log.

Agora:

- **`demo_simulation/scenarios.py`** guarda pose de nascimento e enquadramento das
  duas câmeras, **indexados pelo arquivo de mundo**. O enquadramento é *derivado*
  do mundo: não há mais dois lugares para escolher, então não há mais como
  escolher a combinação errada.
- **`robot_selection.py`** ganhou o cenário oficial **por robô**: labirinto para o
  quadrúpede, armazém para o diff-drive. Duas linhas em vez de uma constante,
  porque o diff-drive é o fallback e o portão do F6 dele (meta de x=0 para x=1) foi
  medido no armazém — no labirinto essa meta cai numa parede.
- Todo `scene_*` e o `yaw` aceitam valor explícito, que vence a tabela. O default é
  **vazio**, não um número: um número ali é indistinguível de uma escolha do
  operador, e foi assim que o enquadramento do armazém sobreviveu à troca de mundo.

```bash
# o cenário oficial, sem SIM_ARGS nenhum
MAZE_MODELS=/caminho/ros_maze_worlds/models docker compose -f compose.host.yml up -d sim

# desvio: olhar de mais alto, mantendo os outros nove números
SIM_ARGS="scene_top_z:=20.0" docker compose -f compose.host.yml up -d sim
```

### A malha do labirinto continua fora do repositório, e agora falha alto

`quadruped_maze11.sdf` referencia `model://maze11/meshes/maze11.stl`, que vem de
`cafemesa/ros_maze_worlds`. Esse repositório declara **`<license>TODO</license>`** e
não tem arquivo de licença — o **mesmo bloqueio** que fez este projeto trocar o
robô A1 pelo Go2. Sem licença, sem vendorização.

A consequência é uma falha silenciosa cara: malha ausente é **WARNING** no Gazebo,
nunca erro. O mundo carrega, o labirinto fica sem visual e sem colisão, o lidar não
vê parede, o Nav2 planeja em linha reta e a meta termina `SUCCEEDED` **mais rápido
que o real**. O ensaio *passa* com números melhores que a verdade. E o caminho
default do compose (`MAZE_MODELS:-./models-extra`) faz o Docker **criar um
diretório vazio** em vez de recusar.

`quadruped.launch.py` agora verifica antes de subir qualquer nó
(`_check_external_models`) e **aborta** nomeando `MAZE_MODELS`. E
`models-extra/README.md` passou a existir: o comentário do compose prometia esse
arquivo e ele não estava lá.

## Um defeito introduzido e corrigido na mesma sessão

A primeira versão do enquadramento derivado passava as poses resolvidas por
`ParameterValue(value_type=float)`. O container `sim` morria na partida:

```
[ERROR] [launch]: Caught exception in launch: value='-13.0' is not an instance of <class 'float'>
```

`ParameterValue` converte **substituição** para float, não texto para float. As
poses resolvidas pelo `OpaqueFunction` já eram texto. Custou um ciclo de build de
imagem para descobrir, e por isso entrou
`demo_simulation/test/test_scene_cameras_launch.py`: ele **constrói a descrição de
launch de verdade** para os quatro mundos e afirma sobre os parâmetros. Custa
milissegundos e pega a classe inteira — tipo de parâmetro, argumento não declarado,
substituição no lugar de ação.

## Duas descobertas do primeiro boot a frio com as correções

### O `nav` sobrevivia ao `down` e rodava imagem de dez horas antes

Primeira tentativa de validar a correção do `route_server`: o log ainda mostrava
`Created route operation ReroutingService`, como se o YAML não tivesse efeito.

Não era o YAML. `nav` e `perception` estão atrás de `profiles: ["learn"]`, e o
Compose **ignora serviço com perfil** em comando que não declara o perfil — o
`down` inclusive. O `down` lista o que removeu e nunca o que pulou, então a saída
parece completa. E o `up -d sim` seguinte não recria o `nav`, porque para o
Compose ele já está no estado desejado.

```
$ docker inspect docker-nav-1 --format '{{.Created}} {{.Image}}'
2026-08-25T03:49:25Z  sha256:46f4c7ac...     <- container de 10h antes
$ docker images --no-trunc | grep nav:dev
local/demo-aquila-nav:dev  sha256:8e71b29a... <- imagem recem-construida
```

O container atravessou **dois ciclos de build** rodando o código de quando subiu,
com `docker compose ps` dizendo `running` o tempo todo. É a mesma família da
armadilha 4 — lá o `/ws/src` assado na imagem estava velho, aqui a imagem inteira
está velha — e vale para qualquer conclusão tirada nesta bancada nas últimas
horas de trabalho antes desta descoberta.

O caminho documentado em `guia-cockpit.md` e `guia-ml35-docker.md` sempre foi
`--profile learn down`; foi a invocação desta sessão que estava errada. Virou a
**armadilha 15** do `guia-operacao.md`, com o par de comandos para conferir ID de
imagem do container contra ID da tag.

### Sobrepor `operations` exige declarar o tipo de cada plugin

Com o `nav` de verdade reconstruído, o segfault **desapareceu** — o
`ReroutingService` não é mais construído. No lugar dele:

```
[FATAL] [route_server]: Can not get 'plugin' param value for AdjustSpeedLimit
[FATAL] [route_server]: Failed to configure route server: No 'plugin' param for param ns!
[ERROR] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

Enquanto `operations` vem do default, o `nav2_route` conhece os tipos dos próprios
defaults. No instante em que a lista é sobreposta, ele passa a exigir
`<nome>.plugin`. Corrigido declarando o tipo junto:

```yaml
route_server:
  ros__parameters:
    operations: ["AdjustSpeedLimit"]
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
```

Vale registrar que esta falha é **melhor** que a que ela substituiu: o
gerenciador aborta a subida inteira, nenhum servidor fica meio vivo, e o container
não mente `running` com o Nav2 morto dentro. `test_official_scenario.py` ganhou
`test_every_declared_route_operation_declares_its_plugin_type` para que a próxima
operação acrescentada à lista não repita a subida reprovada.

## A causa maior: carga que a navegação paga sem aparecer em métrica nenhuma

Com o DDS e o `route_server` corrigidos, o `cmd_vel` passou a chegar (os sticks
saíram de zero) e a velocidade subiu de 0,0044 para 0,0139 m/s. Ainda 4,7× abaixo
do V3. O que fechou o vão não estava na sintonia — estava na **carga adicionada
depois do V3**.

`git diff --stat ff9ae4b..HEAD` sobre `demo_{navigation,simulation,bringup}`
mostra 2 331 linhas novas, e entre elas duas coisas que rodam **junto com a
navegação** e não existiam quando a V3 foi medida: as **duas câmeras de cena do
cockpit** e a **percepção em container**. O protocolo do V3 é nativo, três
terminais (`run_quadruped_sim.sh`, `nav_quadruped.launch.py`, `nav_trial.py`) —
sem cockpit e sem percepção.

Matriz 2×2, mesmo protocolo do V3 (180 s de simulação, prazo de meta 90 s,
maze11, host x86), n = 1 por célula:

| câmeras de cena | percepção | velocidade média | caminho | `cmd_vx` pico |
| --- | --- | --- | --- | --- |
| ligadas | ligada | 0,0139 m/s | 2,50 m | 0,063 |
| ligadas | parada | 0,0155 m/s | 2,78 m | 0,012 |
| paradas | ligada | 0,0200 m/s | 3,59 m | 0,107 |
| **paradas** | **parada** | **0,0515 m/s** | **9,26 m** | **0,139** |
| referência V3 (nativo) | — | 0,0650 m/s | 11,04–12,47 m | 0,143 |

A célula de baixo é território V3: 0,0515 contra 0,0650 nas corridas de 180 s e
0,0525 na corrida longa do próprio V3. **A sintonia sempre esteve boa.** O que
mudou é o que roda ao lado dela.

Com n = 1 por célula a dispersão é grande e as duas linhas do meio não sustentam
atribuir um número a cada contribuinte separadamente — o pico de `cmd_vx` cai a
0,012 com câmeras e sem percepção, e sobe a 0,107 sem câmeras e com percepção,
o que não fecha num modelo aditivo. O que **é** inequívoco é o extremo: qualquer
uma das duas cargas presente derruba a navegação para a faixa 0,014–0,020 m/s, e
as duas ausentes devolvem 0,0515.

### O fator de tempo real não vê nada disso

Este é o achado que explica por que a carga entrou sem ninguém notar. O
cabeçalho de `cockpit_scene_iso.sdf` registra uma medição cuidadosa de
`update_rate` **por RTF**:

```
update_rate 15  ->  9,4 Hz entregues,  fator de tempo real 0,59
update_rate 10  ->  9,8 Hz entregues,  fator de tempo real 0,97
```

10 Hz foi escolhido porque o RTF ficava em 0,97, isto é, "praticamente de
graça". Medido agora: **RTF 0,96 nas duas pontas** da comparação que difere 3,3×
em velocidade de navegação. Taxa do lidar também intacta, **9,9 Hz com e sem as
câmeras**, e odometria a 49,7 Hz. Nenhuma das três métricas que alguém olharia
mostra o problema.

Consequência prática, e é a mais importante deste documento: **comparar corridas
com cargas diferentes fabrica regressões que não existem.** Foi exatamente o que
aconteceu — 0,0139 contra 0,0650 parecia perda de sintonia de 15×, e a sintonia
estava byte a byte no lugar.

## As duas correções de carga

### Camada de percepção: `clearing: false` deixava marca permanente

O bloco tinha `observation_persistence: 1.0` com o comentário
*"expire them rather than accumulating a trail of phantom obstacles"* — a
intenção certa — e `clearing: false` logo acima, entregando o oposto:
`observation_persistence` esvazia o **buffer de observações**, não as células já
escritas.

Medido com a percepção **já parada há cerca de um minuto**:

| | células letais | faixa livre no eixo do robô |
| --- | --- | --- |
| antes de limpar | 169 | 0,10 m |
| depois de `/demo/nav/reset` | 108 | 0,15 m |

61 células letais que nenhuma observação viva sustentava, e que só saíram por
limpeza explícita. Num corredor de 1,20 m com `robot_radius: 0.38` isso é a
diferença entre ter e não ter caminho — e numa demo longa é degradação
monotônica: quanto mais tempo rodando, menos costmap navegável, sem log.

Havia um motivo declarado para o `false`, e ele aponta para o mecanismo errado:

> marking: true / clearing: false is deliberate. The stub's assumed-range
> projection is too coarse to trust for clearing — letting it clear cells would
> let a bad projection erase real lidar obstacles.

Cada `ObstacleLayer` tem grade **própria** e entra no mestre por
`combination_method`, cujo default no nav2 é `Max` — verificado no header
instalado, não suposto (`nav2_costmap_2d/cost_values.hpp`: *"default to
maximum"*, `Max = 1`). O mestre fica com `max(parede_do_obstacle_layer, 0)`, que
é a parede. Uma projeção ruim na camada de percepção **não tem como** apagar o
que o lidar viu.

Corrigido nos **dois** costmaps. Os dois têm de casar: planejar num mapa que
limpa e controlar num que não limpa dá rota válida com o controlador recusando
segui-la, sem erro nenhum. Guarda em
`test_perception_layer_clears_in_both_costmaps`.

Vale igual para o TIDL: obstáculo detectado que saiu de campo tem de poder sair
do costmap. É sintonia de costmap, não contrato de percepção — tópico e tipo não
mudam.

### Câmeras de cena: 1600×1200 → 1200×900

O painel de cena ocupa ~1200 px de largura na tela. O histórico registrado no
próprio SDF é: 800×600 foi reprovado porque esticava e sujava as paredes do
labirinto no JPEG; a resposta foi 1600×1200, que resolveu o visual e passou de
largo — 1600 px num painel de 1200 px é redução, não nitidez.

1200×900 casa sensor e painel **exatamente**, atendendo o mesmo critério que
reprovou 800×600 (nada de esticar) com **44% menos pixels**. A proporção 4:3 é
preservada, o que é obrigatório: o `horizontal_fov` e o enquadramento das duas
cenas foram medidos nessa proporção, e ir para 16:9 mantendo o hfov desenquadra
as duas de uma vez. A regra registrada no SDF passou a ser "sensor = largura do
painel", não um valor decorado, e há guarda de proporção e de igualdade entre as
duas câmeras em `test_scene_cameras_keep_the_measured_aspect_and_match_each_other`.

**A conferência visual é do operador.** A reprovação de 800×600 foi visual, e
nenhuma métrica desta sessão substitui olhar o painel.

## Aberto e não resolvido: quedas e colapso do RTF na bancada

A sessão foi encerrada aqui por decisão do operador, para ir à integração com o
Aquila AM69. Fica registrado o que **não** fechou, porque é o que pode morder na
próxima corrida.

**O robô caiu duas vezes**, nas duas últimas corridas da sessão:

- na partida da pilha completa, com 26 `RECOVER` do supervisor de marcha e o robô
  deitado (`z = 0,162 m`, inclinação 133,7°);
- e numa corrida que ia bem — 0,0933 m/s, mais rápido que o V3 — abortada em
  t = 62 s com inclinação 31,6°.

**Zero quedas é critério do portão V3** (`quedas: 0 em 3 corridas ✅`). Isto é uma
regressão de estabilidade, não de velocidade, e por isso vale mais que qualquer
número de m/s deste documento.

Não está atribuída. O que se sabe:

- o **fator de tempo real da bancada desabou para ~0,5** (lidar entregando
  5,0 Hz em vez de 9,9 Hz, odometria 24,8 Hz em vez de 49,7 Hz);
- e continuou em **0,54 com a percepção parada** e com as câmeras em resolução
  *menor* que a das corridas rápidas — ou seja, na direção oposta à carga que eu
  havia acabado de reduzir;
- carga do host 15–16 em 22 núcleos, com a soma dos containers em ~7,8 núcleos.
  O processo `ruby` a 146% que aparece no `ps` **é** o lançador do `gz sim`, não
  um intruso;
- as corridas anteriores da mesma sessão, com imagens antigas, tiveram pico de
  inclinação entre 0,69° e 1,62° e nenhuma queda.

A leitura mais provável é **estado acumulado da bancada** ao longo de muitos
`down`/`up`/`restart` e várias reconstruções de imagem, não o diff. Mas é
hipótese.

Por isso a redução de resolução das câmeras (1600×1200 → 1200×900) foi
**revertida**: era a mudança com a evidência mais fraca (n = 1, carga variável) e
a única que desfazia uma decisão visual já validada. O raciocínio e a medição
ficaram registrados no cabeçalho de `cockpit_scene_iso.sdf` para quem retomar.

O `clearing: true` da camada de percepção **foi mantido**: o mecanismo está
provado (61 células letais que nenhuma observação viva sustentava) e marca
permanente é estritamente pior numa corrida longa de HIL.

### O que fazer antes de confiar em qualquer número novo

1. Reiniciar a bancada (a máquina, não os containers) e refazer a medição
   pivotal com `n >= 3`.
2. Registrar **RTF e carga junto de cada corrida**. Sem isso as corridas não são
   comparáveis, e foi essa ausência que fez a sintonia V3 intacta parecer 15×
   pior.
3. Só então decidir sobre a resolução das câmeras.
