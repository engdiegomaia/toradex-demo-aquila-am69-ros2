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
| **F4** | Contrato atravessando fronteira de container | ✅ **concluída** 24/08/2026 | contrato e perception revalidados no Go2 headless |
| **F5** | Nav2 sobre pernas + modo HIL | 🟡 **em andamento** 28/08/2026 (tarde) | portão de TF **APROVADO** (99,94%) e portão de metas **PARCIAL** — 5 de 6 critérios passam, a velocidade média reprova a 0,0391 m/s contra 0,05. Aceitação da busca autônoma NÃO executada. Ver "Sessão 28/08 (tarde)", `docs/results/ml35-f5-ab-joint-states.md` e `ml35-f5-portao-tres-metas.md` |
| **F6** | Fallback selecionável e testes | ✅ **concluída** 24/08/2026 | cold start + goal `SUCCEEDED` nos dois robôs |

### Sessão 28/08 (tarde) — o portão de TF fechou; a velocidade não

Evidência: **`docs/results/ml35-f5-ab-joint-states.md`** e
**`ml35-f5-portao-tres-metas.md`**, com os CSVs ao lado. Commits `a7dc097`
(decimação), `235ac1f` (sonda), `d7efd30` (reversão do mapa), `becac77` e
`fa1d012` (evidência).

**A causa raiz era `/tf` a 1090 Hz.** O `controller_manager` roda a 1000 Hz
porque a física roda a 1000 Hz, e um controlador sem `update_rate` próprio herda
essa taxa. O `joint_state_broadcaster` publicava `/joint_states` a 1 kHz, o
`robot_state_publisher` convertia cada amostra num `TFMessage`, e onze
assinantes deserializavam o resultado — atravessando a Ethernet, porque o
`robot_state_publisher` roda no HOST e a navegação roda no Aquila. Nav2 não
consome nada disso: as arestas que ela usa são juntas FIXAS e já saem uma vez em
`/tf_static`. Os 1090 Hz eram as doze juntas das PERNAS.

A correção é `update_rate: 50` no broadcaster, pelo spawner
(`demo_simulation/config/joint_state_broadcaster.yaml`), decimação exata de
fator 20. Laço, marcha, física, IMU e odometria intocados, e há teste estrutural
para cada um deles — o modo de falha barato é baixar a taxa do LAÇO em vez da
do BROADCASTER, duas edições de uma linha no mesmo arquivo.

**A/B pareado, uma variável, mesmo protocolo nos dois braços** (recria `sim` →
reinicia módulo → espera SLAM → estabiliza → mede):

| | 1000 Hz | 50 Hz |
| --- | ---: | ---: |
| `/joint_states` | 986,1 Hz | 45,1 Hz |
| `/tf` | 1054,5 Hz | 144,6 Hz (−86,3%) |
| `odom <- lidar` disponível | 94,75% | **99,94%** |
| carga do módulo | 26,90 | 18,52 |
| `nav2_container` | 298% | 240% |
| `maze_explorer` | 67,6% | **76,0%** |

**A atribuição da aresta ficou fechada:** `odom <- base` e `odom <- lidar` deram
exatamente o mesmo número nos dois braços. A cadeia composta não perde nada além
do que a aresta dinâmica perde. E o amostrador de 200 Hz dá o mecanismo: no
braço A o máximo entre carimbos distintos era 120 ms num publicador de 20 ms —
cinco ciclos perdidos de uma vez. Não era rajada de entrega; era o `odom_tf` não
sendo escalonado a tempo de carimbar. Isto também corrige a atribuição da manhã
(100,00% / 95,30% / 78,11%), que comparava três execuções separadas de durações
diferentes: a conclusão qualitativa estava certa, os números não eram
comparáveis entre si.

**Uma hipótese minha foi REPROVADA e está registrada como tal.** Eu havia dito
que os ~68% de um `maze_explorer` OCIOSO eram o `TransformListener` dele
deserializando 1090 mensagens por segundo. Com o fluxo 86% menor ele SUBIU, para
76,0%. A explicação plausível é estrangulamento — com a carga caindo de 26,9
para 18,5, um nó antes disputado passa a rodar à vontade — mas isso é hipótese,
não medição. Perfilar as threads dele (método de `ml35-f5-clock-fanout.md`) é o
próximo passo se o objetivo for CPU.

**`map_update_interval` voltou para 1.0**, em rodada independente. Ele tinha ido
a 5.0 nesta mesma sessão por economia de CPU; a economia foi medida e não
existia (31,4% → 30,3% no `async_slam_toolbox`, dentro do ruído). Na volta o
custo é 1 pp, simétrico, o que confirma que o número é ruído. `/map` sobe de 0,2
para 1,000 Hz e a `static_layer` deixa de ficar até 5 s atrás da parede que o
SLAM já conhece.

**Portão de metas, 3 corridas de 180 s em `maze11-short`:**

| | 1 | 2 | 3 |
| --- | ---: | ---: | ---: |
| metas cumpridas | 9/10 | 8/10 | 9/10 |
| primeiras três | ok ok ok | ok ok ok | ok ok ok |
| tilt de pico | 1,17° | 1,06° | 1,18° |
| folga mínima | 0,448 m | 0,448 m | 0,448 m |
| velocidade média | 0,0383 | 0,0342 | 0,0447 m/s |

Varredura de log limitada, 12 min: **zero** `worldToMap`, **zero**
`invalid source`, **zero** extrapolação de TF. Cinco de seis critérios passam.

**A velocidade reprova, e não é regressão.** A linha de base do maze11 em
`gait_go2.yaml` é 0,0399 m/s; estas três dão média 0,0391. O que melhorou é a
razão de trabalho em vx, de 6,2% para 15,6–22,3% — o robô passa duas a três
vezes mais tempo com avanço efetivo, e isso NÃO virou velocidade média. É
exatamente a distância entre o limite de MÁQUINA, que esta sessão atacou, e o
limite de DECISÃO DE TRAJETO, isolado em `ml35-f5-clock-fanout.md` e ainda de pé.

Ressalva de método, registrada para não virar precedente: `maze11-short` são
metas a 0,5 m e boa parte de cada ciclo é reaquisição, não travessia. O critério
de 0,05 m/s não separa travessia de reaquisição.

**Onde a progressão parou, e por quê.** Nenhuma condição de parada ocorreu — o
estouro de prazo da corrida 2 é prazo, não meta recusada. O que falta é o smoke
de exploração e as três partidas frias de até 600 s, e antes de gastá-las há
duas decisões em aberto:

1. o critério de 0,05 m/s vale para `maze11-short`? Se o alvo é travessia, o
   conjunto certo é `maze11` (metas de 8 m) e o número a bater é outro;
2. o `maze_explorer` a 76% ocioso é o segundo maior consumidor do container
   `nav` e nada aplicado nesta sessão o toca.

**Armadilha nova, que custou uma corrida inteira de 180 s.** Logo depois de
recriar o container `sim`, `/clock` aparece no grafo mas não entrega mensagem a
assinante NOVO por alguns minutos. Qualquer script com `use_sim_time: True` que
suba nessa janela lê relógio parado: RTF 0,000, idade da nuvem −240 s, 100% de
carimbos "no futuro". As colunas que não usam o relógio do nó continuavam
válidas, mas a corrida foi descartada e refeita. Antes de medir, confirme
entrega de verdade (`ros2 topic hz /clock`), não presença no grafo — e note que
o mapa do SLAM é `/map`, não `/demo/map`.

### Sessão 28/08 — busca autônoma implementada; portão de estabilidade ainda REPROVADO

Evidência: **`docs/results/ml35-f5-busca-autonoma.md`** (`PENDING EXECUTION`).

A demonstração de saída autônoma do labirinto está **implementada de ponta a
ponta e instalada**, e **nenhuma corrida de aceitação foi executada**. As duas
frases valem ao mesmo tempo, e a segunda é a que decide se a fase fecha.

**Fechado (host):**

| Peça | Onde roda |
|---|---|
| `frontier.py` + `maze_explorer` (fronteiras, blacklist, prazos, JSON) | módulo |
| `ExplorationGrid` (`allow_unknown: false`) + `nav_to_pose_exploration.xml` | módulo |
| `maze_exit_detector` (painel magenta, confirmação 3 de 5) | módulo |
| painel magenta no `quadruped_maze11.sdf` | host |
| `maze_escape_validator` → `/demo/maze/escaped` | **host, só simulação** |
| botões e HUD de busca no cockpit | cockpit |

Suítes de host: contrato **150**, `demo_navigation` **25**, `demo_perception`
**33**, cockpit **169**. Nenhuma delas mede navegação.

**O portão continua sendo o bloqueio, e ele reprovou.** Última corrida
(`artifacts/maze11-short-gate.csv`): 37,1 s, 0,69 m, **0,0185 m/s**, `cmd_vx`
não-nulo em 18,7% das amostras — **abaixo do piso de 0,05 m/s**, e sem 3/3
metas. A corrida anterior, antes de `restamp_tf: true`, tinha o robô
**congelado** (`cmd_vx` zero em 150 s). O parâmetro destravou o comando e **não
fechou o portão**.

`restamp_tf` foi verificado como parâmetro real do `slam_toolbox` do Jazzy
(`slam_toolbox_common.hpp:177`, e `restamp_tf: false` nos cinco
`mapper_params_*.yaml` de `/opt/ros/jazzy/share`) — não é YAML ignorado em
silêncio. `transform_timeout` fica em 0,2 como exigido.

**`nav_trial.py` passou a arquivar a evidência por meta.** Antes o desfecho de
cada ação morria no stdout e a meta em voo no fim do ensaio nunca era
registrada — um portão de 3 metas relatava 2. Agora sai um CSV irmão
`<csv>-metas.csv` com alvo, desfecho, `status`, `error_code`/`error_msg` do
Nav2 e trocas de rota **daquela** meta, e cada amostra de telemetria carrega
`goal_index`.

**Risco aberto que precede qualquer conclusão sobre percepção:** o RAW da câmera
não atravessa mais o fio desde `ml35-f5-camera-comprimida.md`. O `SetRemap` de
`demo_bringup/launch/perception.launch.py` religa a imagem do detector
automaticamente, **mas `/demo/camera/camera_info` não é remapeado**. Sem ele o
detector publica detecção e nunca publica pose — falha silenciosa. Checar
`ros2 topic hz /demo/camera/camera_info` **no módulo** antes de culpar a visão.

### Sessão 27/08 (parte 3) — mecanismo achado: o plano global alterna a 1 Hz. `clearing: false` REPROVADO

Evidência: **`docs/results/ml35-f5-memoria-costmap.md`**.

**Causa raiz, lendo `/plan` a cada 5 s numa meta presa (0,0) → (0,8):**

| t | comprimento | rumo inicial |
| ---: | ---: | ---: |
| +5 s / +10 s | **11,49 m** | 173° — rota verdadeira |
| +15 s / +20 s | **8,59 m** | 89° — atravessa parede não vista |
| +25 s / +30 s | 8,66 / 8,81 m | 35° / 18° |

Os 11,5 m batem com a geodésica offline (12,23 m). Os 8,6 m só existem porque
`allow_unknown: true` torna o desconhecido barato. **O MPPI recebe um caminho
que inverte 90–180° a cada segundo** — daí girar sem transladar. Completa o
achado da parte 2: lá ficou provado que o sintoma some com meta boa; aqui está
o mecanismo pelo qual a meta ruim o produz.

**Experimento reprovado — não repita.** `clearing: false` no `obstacle_layer`
global ("dar memória ao mapa") melhorou margem (deslocamento 0,19 → 1,04 m,
razão de trabalho 0,0% → 3,9%) e **não mexeu no mecanismo**: o plano continuou
alternando 11,66 ↔ 8,77 m, zero metas. Lendo `costmap_raw`, com ele ligado
**150 de 161 células da reta até a meta ficaram em 255 (desconhecido)**.

`clearing` não é "esquecer obstáculo" — é o raytrace, e o raytrace é o **único**
mecanismo que torna desconhecido em LIVRE nessa camada. Desligá-lo deixa o mapa
permanentemente desconhecido e torna o atalho **mais** atraente. A correção
agrava a causa que ataca. Travado por `tests/test_module_params_mount.py`.

**O que resolve** é persistir ocupado *e* livre, e a camada de obstáculo tem um
botão só para os dois. É mapa de `slam_toolbox` na `static_layer` (já definida e
inerte), com `allow_unknown: true` mantido. Bloqueio real: o
`pointcloud_to_laserscan` exige **rebuild arm64 nativo no módulo**.

**Infraestrutura entregue:** `compose.module.yml` monta
`ros2_ws/src/demo_navigation/config` sobre `/ws/src/demo_navigation/config`
(o **alvo final do symlink**, não o caminho instalado — montar no instalado
seria silencioso). **Parâmetro no módulo passou a custar `sync`, não `build`.**

**Anomalia aberta:** o costmap global marca primeira célula ≥ 253 a **0,55 m em
+y**, onde `maze_fit.py` mede **3,47 m de pista livre**. Medir antes de rodar o
SLAM — um mapa persistente herdaria o erro em definitivo.

### Sessão 27/08 (parte 2) — PROVADO no HIL: 8 de 8 metas cumpridas, razão de trabalho 0,0% → 37,5%

Evidência: **`docs/results/ml35-f5-rota-conectada.md`** + os dois CSVs ao lado.
A/B com minutos de intervalo, **HIL real** (Nav2 no Aquila AM69), mesmas
imagens, mesmos parâmetros, nada reconstruído. **Única variável: a geometria da
meta.**

| métrica | rota conectada | controle — patrulha (0; 8) |
| --- | ---: | ---: |
| **razão de trabalho `vx`** | **37,5%** | **0,0%** |
| `cmd_vx` ≈ 0 | 12,9% | 98,6% |
| deslocamento líquido | **7,11 m** | 0,19 m |
| eficiência de trajeto | 57,2% | 16,2% |
| **metas cumpridas** | **8 de 8** | 0 em 120 s |
| deriva líquida de yaw | **+1,1°** em 240 s | **−186,8°** em 120 s |

A razão de trabalho nunca passou de 8,6% em condição alguma testada neste
projeto (CPU, `/clock`, amostragem do MPPI, câmera comprimida, Ethernet,
correção da BT). Foi a 37,5% sem tocar em nada além da meta. Os 57,2% de
eficiência reproduzem os 57% medidos no host em 21/08 — o módulo sempre foi
capaz disso.

**O giro unidirecional da §10 não é defeito de controlador.** Com plano válido o
`cmd_wz` alterna 45,2% / 51,3% e a deriva é +1,1° em quatro minutos. Com meta
atrás de parede volta a ser unidirecional — **e com o sinal invertido** em
relação à §10, o que mata a família "assimetria de critic" / "erro de sinal na
guinada": erro de sinal não troca de sinal.

**Consequência para o portão.** "Goal Nav2 `SUCCEEDED` com o robô de pernas,
Nav2 no módulo" foi cumprido **oito vezes numa corrida**. O que reprovava era o
protocolo de 8 m sobre metas de patrulha. O portão do F5 tem de ser reescrito
sobre rota conectada ou sobre mapa persistido antes de voltar a ser cobrado.

**Resta déficit real, agora mensurável:** 37,5% e 0,0454 m/s médio ainda estão
abaixo de `vx_max` 0,15 m/s. Sintonia de critic só faz sentido a partir daqui.

**Armadilha nova:** toda meta do maze11 tem `x` negativo, e
`--goals -1.50,...` é lido pelo argparse como flag — o script imprime `usage` e
sai **0**. Com `2>/dev/null` vira corrida silenciosa que não faz nada. Use
sempre `--goals=`. Documentado no próprio `nav_trial.py`.

### Sessão 27/08 (parte 1) — as metas do ensaio estão atrás de parede; o plano global atravessa parede

Evidência e números: `docs/ml35/proximos-passos-navegacao.md` §11. Medido
**offline**, sem bancada, sem ROS e sem Gazebo — só leitura do STL do maze11,
com a ferramenta nova `scripts/maze_geodesic.py` (6 guardas em
`tests/test_maze_geodesic.py`, três verificadas por mutação).

**As quatro `MAZE11_GOALS` têm parede na linha reta.** A geodésica pelo espaço
navegável é 1,53× a 4,22× a reta, e do spawn o robô enxerga, com oclusão,
**19,6%** do espaço livre dentro dos 8 m de `obstacle_max_range`.

Com `global_costmap` rolante **sem `static_layer` e sem mapa** e o NavFn em
`allow_unknown: true`, o plano dessas metas atravessa parede não observada —
`SUCCEEDED`, caminho bonito no RViz e no cockpit, **zero erro ou log**.
Corrobora com dado já no repositório: o caminho medido na §8 tinha ~7,0 m para
uma meta cuja rota real é 12,23 m e cuja própria reta é 8,00 m.

**Consequência de método:** as §§7–10 mediram o MPPI com uma entrada inválida.
Isso não reabre as cinco hipóteses refutadas da §1, mas nenhuma conclusão sobre
critics sobrevive — o teste de desligar critic desce de prioridade.

**A ordenação que a §9 procurou por bearing e não achou** é por distância até a
primeira parede na reta: 0,96 m → 0,00 m de deslocamento; 3,88 m → 0,07 m;
3,90 m → 0,10 m. O corte cai no horizonte do MPPI (1,44 m).

**Próximo passo, barato e decisivo, no host, sem tocar em imagem:** rodar a rota
conectada do `maze_route.py` (0 de 9 pernas com parede na reta, 100% visível em
todas, contra 4 de 4 bloqueadas na patrulha). Comando pronto na §11.

**Persistir o mapa — pedido do operador — não é frente nova: é ligar o que já
está na árvore.** O `static_layer` do Go2 já está definido e inerte com o
procedimento ao lado, e o caminho diff-drive já navega sobre
`maps/warehouse.{pgm,yaml}`. Os dois bloqueios reais: `slam_params.yaml` tem
`base_frame: base_link` (parâmetro, não arquitetura) e o `slam_toolbox` consome
`LaserScan`, enquanto o `/demo/scan` do Go2 é o anel degenerado que o delta 3 do
`nav2_params_go2.yaml` já mediu como **zero obstáculos**. O dado bom é
`/demo/scan_cloud`, e achatá-lo é `ros-jazzy-pointcloud-to-laserscan` (estoque,
2.0.2 no apt do Jazzy, ainda não em imagem nenhuma). **Sem AMCL** nesta
topologia: a odometria do Gazebo é verdade de terreno e `map`→`odom` já é a
identidade do `odom_tf`.

### Sessão 26/08 (tarde) — reset do cockpit, telemetria do alvo, protocolo de campanha

Evidência completa: `docs/results/cockpit-reset-nao-destrutivo.md`.

**Defeito grave fechado: o botão de reset do cockpit apagava o robô.**
`/demo/sim/reset` usava `ControlWorld.reset.all`, que devolve o mundo ao SDF de
origem — e o robô e as duas câmeras de cena são INSERIDOS depois da carga
(`ros_gz_sim create`), logo não estão nele. Medido: `/joint_states` 999 Hz →
morto, `/demo/imu` 996 Hz → morto, `/demo/odom` 49,6 Hz → morto,
`gz model -m demo_robot` → `No model named <demo_robot>`.

O modo de falha era o pior deste projeto: relógio seguia a 999 Hz e os sensores
órfãos a 10 Hz, então **o cockpit ficava inteiro verde apontando para uma planta
inexistente**, sem uma linha de log. Recuperar exigia reiniciar o `sim`.

Agora o reset TELEPORTA o robô para a pose de nascimento do cenário, pelo mesmo
`/demo/sim/set_entity_pose` que as câmeras já usavam. Verificado: robô volta de
(2,0; −1,5) para (0,00003; −0,010), `/joint_states` 1000 Hz, `/demo/imu` 974 Hz,
`/demo/odom` 49,9 Hz, e os cinco modelos seguem no mundo. O relógio **não** volta
a zero, de propósito: um salto de tempo para trás invalidaria o buffer de TF do
Nav2 e o `controller_manager`.

> "reassenta sozinho" estava escrito aqui e era falso — corrigido na sessão
> seguinte, ver abaixo.

**Telemetria do alvo no cockpit, medida no AM69 real.** `target_monitor` publica
`/demo/target/status` (CPU, memória, temperatura, load) e
`/demo/target/ops_log` (eixos comandados em SI, manche e odom, em texto curto).
O painel de logs deixou de depender de `/rosout` bruto — `/rosout` segue
assinado só como reserva filtrada para avisos e erros. Temperatura conferida
contra o sensor: 34,974 °C reportado contra `thermal_zone1/6` lendo 34498
milésimos no mesmo instante; as sete zonas entre 32,1 e 34,5 °C. Custo do nó:
**4,3% de um núcleo** em 800% disponíveis (`use_sim_time: False` mantém isso
barato — ele não assina `/clock`).

**As telas sobrevivem aos dois reinícios.** Sonda com o cliente rosbridge do
próprio cockpit, um assinante por painel: restart do `sim` e restart da
aplicação no alvo não perdem painel nenhum, e o WebSocket não cai (nesta
topologia `cockpit` e `hmi` rodam no host). O único zero é `/demo/cmd_vel_si`
sem meta ativa, que **não é defeito** — o Nav2 subiu `Managed nodes are active`
e o `velocity_smoother` só publica depois da primeira meta; o canal novo diz
isso em texto.

Uma hipótese foi **testada e descartada**: religar o `<img>` do MJPEG depois que
o publicador volta. Medido com `curl` na mesma resposta HTTP através de um
restart do `sim`, os bytes crescem sem interrupção (1,01 MB → 4,61 MB). O
`web_video_server` mantém inscrição e resposta abertas. Não gaste código nisso.

**Protocolo de campanha entregue: `scripts/nav_campaign.py`.** É o elo que
faltava entre `nav_trial.py` (uma corrida) e `summarize_trials.py` (resume
replicatas): decide a ORDEM e o que acontece entre pernas. Intercala
`A B A B A B` em vez de blocar, e repõe robô e costmap antes de cada perna.
**Só é possível por causa do conserto do reset acima** — uma campanha que
chamasse o reset antigo entre pernas mediria, da perna 2 em diante, um mundo sem
robô e sem nada acusando. Dez guardas em `tests/test_nav_campaign.py`, incluindo
o inverso (a ordem blocada tem de reprovar) e o fato de que `ros2 service call`
sai 0 mesmo com `success=False`.

Perna de fumaça de 45 s, para provar o laço: trabalho de `cmd_vx` **0,0%**,
`vx`≈0 em **99,6%**, velocidade 0,0022 m/s. Reproduz o sintoma; n=1 e 45 s não
decidem nada.

**Duas coisas que esta sessão NÃO fez:**

- **O cockpit não foi aberto num navegador.** Não há Chrome nesta máquina e o
  MCP de automação não dirige o Firefox instalado. Tudo acima foi medido no
  caminho de dados. O gate visual continua pendente de passada manual.
- **Controle manual (F4) segue não implementado**, e há um bloqueio novo e
  concreto: `twist_mux` **não está em nenhuma imagem** e o módulo **não tem rota
  default** (só a `192.0.2.1/24`), então `apt` não resolve nada lá. O gateway da
  LAN `192.0.2.2` responde em 0,337 ms e o módulo já tem DNS corporativo — falta
  só `sudo ip route add default via 192.0.2.2 dev ethernet0`, que precisa ser
  rodado por quem tem a permissão.

**Próximo portão, na ordem pedida pelo operador:** (1) rota default no módulo e
F4; (2) campanha A/B real com `nav_campaign.py`, n≥3 intercalado, começando por
`PathAlignCritic` 14,0 — que é o crítico de maior peso e o suspeito nomeado no
próprio YAML como quem torna girar melhor que avançar.

### Sessão 26/08 (noite) — reset não reassentava sozinho: robô colapsava ou se arrastava

Evidência completa: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

Reportado pelo operador: depois do reset o robô fazia guinada de volta à
orientação anterior. Investigado com o robô em movimento — não parado, o único
caso testado na sessão da tarde — e achados DOIS defeitos, ambos silenciosos:

- **teleportar sem reancorar o gait**: `StateTrotting` (controlador C++) captura
  sua referência de postura (`pcd_`, `yaw_cmd_`) uma única vez, atrás de um
  trinco que só um comando de caminhada limpa. Com `/demo/cmd_vel*` zerados e a
  meta cancelada — para excluir o Nav2 como causa — o robô ainda assim se
  arrastou 0,87 m e girou 135° em 26 s sem nenhum comando publicado;
- **teleportar sem parar**: `SetEntityPose` preserva a velocidade. Com um fluxo
  de `/demo/cmd_vel` vivo a 10 Hz durante o reset (o caso real, com o Nav2
  conduzindo), o robô COLAPSA — `z` de 0,337 m para 0,162 m em 1 s — e fica
  contorcendo-se 40 s.

Corrigido com dois serviços novos no `twist_to_inputs` (único escritor de
`/control_input`): `/demo/gait/hold` (trotting → fixed stand, robô imóvel,
`GAIT_STOP_S = 2,0 s` de espera) chamado ANTES do teleporte, `/demo/gait/resume`
(fixed stand → trotting, `StateTrotting::enter()` reancora `pcd_`/`yaw_cmd_` na
pose nova) chamado DEPOIS. Melhor esforço: numa planta diferencial os dois
serviços não existem e isso é caminho normal — mas a ausência entra na própria
mensagem do `Trigger` de reset, nunca fica silenciosa.

Verificado repetindo o caso que falhava (comando vivo a 10 Hz durante o reset):
robô nunca sai de 0,35-0,36 m de altura, volta a obedecer o mesmo comando depois
do reset, um segundo reset re-arma. **Verificado também com o robô CAÍDO**
(tombado 180°, preso em `mode=RECOVER` com `tilt=131°` — esse modo não sai
sozinho de cabeça para baixo): o reset o recupera de pé, `mode=HOLD`,
`tilt=0,2°`, `yawSat=0%`, e ele volta a andar normalmente.

Guardas novos em `test_sim_reset.py` e `test_twist_to_inputs.py`: a ordem
parar→teleportar→retomar tem de estar nessa sequência no fonte, o comando de
descida (`2`) sai exatamente uma vez, o hold não tem prazo próprio, e todo
caminho de saída do handler de reset — inclusive os de erro — tem de retomar o
gait.

### Sessão 26/08 (madrugada) — onde retomar

Evidência completa: `docs/results/ml35-f5-clock-fanout.md`.

**O item 1 da sessão anterior (CPU do módulo) está FECHADO.** Não o reabra pelo
caminho antigo: não é o laço do MPPI (refutado em `ml35-f5-mppi-amostragem.md`) e
não é a taxa do `/clock` (refutado em 21/08 — estrangular matou a navegação).

**Era fan-out de assinatura de `/clock`.** Perfilando `/proc/<tid>/stat` por
thread, o container `nav` gastava **367% de 800% com o robô PARADO**, e 111%
disso eram três republicadores em Python — `odom_tf`, `cmd_vel_si_to_stick` e
`nav_control_relay` — que **não chamam o relógio uma única vez** e assinavam
`/clock` a ~870 Hz só porque `use_sim_time: true` faz o rclpy criar a assinatura.

Corrigido com `use_sim_time: False` nos três. Os três caíram de **111% para
17,7%**, e `ros2 topic info /clock -v` confirma que nenhum deles assina mais.
Quatro testes em `demo_bringup/test/test_sim_time_scope.py`, verificados por
mutação, travam a invariante nos dois sentidos — inclusive o inverso, que é o
que importa: **nó sem `use_sim_time` não pode chamar `get_clock()`**.

**O que isso comprou, medido:**

| | antes | depois |
| --- | ---: | ---: |
| recusas `Ignoring the source` | 16 | **0** |
| `Robot to stop due to invalid source` | 4 | **0** |
| descartes de costmap | — | **0** |
| folga da máquina sob navegação | nenhuma | **307% de 800%** |

**O que isso NÃO comprou: movimento.** A corrida de confirmação deu 0,0246 m/s,
`vx` em zero em **90,7%** das amostras, girando em **90,3%**, e **0 de 2 metas de
8 m**. Dentro da faixa de ruído já conhecida. Zero quedas.

**Portanto o item 2 da sessão anterior — decisão de trajeto — está agora
sozinho e sem confundidor.** Com CPU sobrando e sem uma única recusa de sensor, o
robô continua girando em vez de transladar. Isso não era efeito colateral da CPU.

**Duas armadilhas descobertas nesta sessão:**

- `docker/.env` ainda carrega `MODULE_IP=192.0.2.5`, endereço antigo de
  bancada, e ele **vence os defaults**. `ssh` funciona assim mesmo porque usa o
  nome mDNS, então o erro só aparece em `sync`/`build` (`não identifiquei a
  interface do módulo que carrega 192.0.2.5`). Passe `MODULE_IP=` e `HOST_IP=`
  explícitos, ou conserte o `.env`.
- Uma hipótese foi testada e **descartada**: `inflation_radius` 0,55 num corredor
  de 1,20 m deixaria 10 cm de faixa livre e tornaria girar mais barato que
  avançar. Medido no `local_costmap`: **64,1% das células com custo 0**, e o que
  estava à frente era parede real. Não gaste sintonia nisso sem medir de novo.

**Próximo portão, na ordem:**

1. **Consertar o protocolo antes de sintonizar.** n ≥ 3 por condição,
   intercalado, mediana e faixa. A dispersão de 2,4× em configuração idêntica
   continua valendo e nenhuma corrida única decide — a desta sessão inclusive.
2. **Trocar a métrica primária** para razão de trabalho de `cmd_vx` e fração de
   `vx` ≈ 0. Deram 0,6% e 0,6% em duas corridas distintas, contra 2,4× de
   dispersão na velocidade média, e medem diretamente o sintoma.

**Instrumentação entregue nesta retomada:** `scripts/nav_trial.py` agora
imprime e grava o ensaio com as duas métricas, usando as bandas fixas
`|cmd_vx| <= 0,005 m/s` (quase zero) e `cmd_vx > 0,05 m/s` (trabalho para
frente). `scripts/summarize_trials.py` resume replicatas por condição sem
agrupar amostras, mostrando `n`, mediana e faixa observada. O resumo reproduz
os números históricos (RAW 5,8% de trabalho; comprimido 0,6%), portanto a
definição nova não muda a linha de base.

O rebuild nativo da imagem arm64 foi concluído no Aquila e os containers foram
recriados. `module.sh verify` voltou a **3/3**, e o log do `route_server` da
imagem nova contém somente `AdjustSpeedLimit` (não contém o antigo
`ReroutingService`). O ensaio n≥3/A-B continua deliberadamente pendente de
orçamento: cada perna dura até 420 s e a campanha completa exige seis pernas.
3. **Só então MPPI** (`PathAlignCritic` 14,0 × `PathAngleCritic` 2,0), agora em
   ensaio limpo, com o costmap medido a cada condição.

### Sessão 25/08 (noite) — onde retomar (leia isto antes de tocar em qualquer coisa)

Evidência completa: `docs/results/ml35-f5-ethernet0-repeticao.md`.
A orientação anterior desta seção (consertar PHY, trocar cabo, medir depois)
**foi cumprida e está vencida** — não a repita.

**O enlace está resolvido e comprovado.** `enp0s31f6` a 1000 Mb/s full,
host `192.0.2.14` ↔ Aquila `192.0.2.16` por `ethernet0`, RTT 0,400 ms,
rota simétrica nos dois sentidos, `scripts/module.sh verify` retornando **0** com
as três etapas. A assimetria sumiu de forma estrutural: o Wi-Fi ficou em métrica
600 contra 100 do cabo, então a `/24` inteira prefere o cabo.

**O portão de 8 m continua REPROVADO, e a rede não é a causa.** A hipótese
`ethernet1`/`ethernet0` que estava aberta aqui está **refutada por medição**: com
rota correta e enlace limpo, as duas metas de 8 m estouraram o prazo igual.

**As duas causas medidas, em ordem de tamanho:**

1. **CPU do módulo.** O Nav2 sozinho consome **600–727% de 800%** no AM69. A
   percepção soma ~187% e passa da capacidade. Aí a frescura do sensor colapsa:
   o `collision_monitor` recusou a nuvem do LiDAR 16 vezes com 1,0–1,2 s de
   defasagem. Com o Nav2 ocioso essa defasagem é de 42 ms — ou seja, é
   enfileiramento por contenção, **não** transporte. Custo medido: **2,8×** na
   velocidade média (0,0429 → 0,0155 m/s).
2. **Decisão de trajeto.** No HIL completo o robô tem `vx` em zero em **79%** das
   amostras e gira em **93,9%** delas: ele passa o ensaio **girando em vez de
   transladar**. Sem a câmera o padrão alivia mas não some, e o custo migra para
   a rota — 18,00 m de caminho para 5,20 m líquidos, **28,9% de eficiência**
   contra 57% no host. Bate com a hipótese já registrada de `PathAlignCritic`
   14,0 contra `PathAngleCritic` 2,0, que **segue sem teste de correção**.

**Tirar a câmera do fio NÃO faz a meta passar.** Foi medido: 0,0429 m/s e ainda
assim 0 de 2 metas. São dois limites independentes, e só um é CPU.

**Estabilidade:** zero quedas nas duas corridas, mas o tilt de pico vai de 0,94°
para **15,66°** justamente na corrida em que o robô anda. O valor baixo do HIL
completo descreve um robô quase parado, não um robô estável. Folga de carcaça
segue em **+6,5 cm**.

**Corridas 2 e 3 do protocolo n=3 não foram executadas** — a 1 reprovou e o
mecanismo ficou identificado; repetir gastaria bancada sem informação nova.

**Uma armadilha de método fechada nesta sessão:** `verify` reprovava por
`/clock` ausente contra um módulo que lia `/clock` a 616 Hz. A etapa 2 coletava
com `grep /demo/` e depois exigia `/clock`, que não está sob `/demo/`. O teste
que existia passava o tempo todo porque só checava se a string aparecia no
arquivo. Corrigido, com teste que falha por mutação.

**FASE 2 DO PLANO JÁ FOI EXECUTADA (25/08, noite).** A câmera comprimida está
implementada, validada e medida: `docs/results/ml35-f5-camera-comprimida.md`.
Ela entrega a engenharia (~82× menos fio, CPU do módulo ~711% → ~600%) e **não
move o portão** — velocidade não melhorou de forma confiável e a razão de
trabalho piorou (2,1–2,7% contra 5,8%). O `collision_monitor` segue recusando a
nuvem com ~1,0 s de defasagem e emitindo `Robot to stop due to invalid source`.
A variável dominante é a **presença** da percepção, não o formato do transporte:
com percepção no módulo a razão fica em 2–6% em qualquer formato; sem ela, 16,8%.
Não repita a fase 2 e não volte a discutir formato de imagem.

**FASE 1 DO PLANO TAMBÉM JÁ FOI EXECUTADA (25/08, noite) E REPROVOU.**
`time_steps` 96→64 com `model_dt` 0,10→0,15 (horizonte constante, 33% menos
amostragem) **não reduziu CPU**: ~437% → ~439%. O custo do MPPI aqui não é
dominado por `batch_size × time_steps`. Evidência:
`docs/results/ml35-f5-mppi-amostragem.md`. Não adotada; YAML de volta ao
baseline com o A/B fora do caminho default.

**LEIA ISTO ANTES DE RODAR QUALQUER ENSAIO NOVO — o método atual não decide.**
Duas corridas na configuração **idêntica** deram **0,0109 e 0,0265 m/s**,
dispersão de **2,4×**. O ruído entre corridas é maior que os efeitos procurados,
então **A/B de n=1 nesta bancada é ininterpretável**. Antes de sintonizar
qualquer coisa: n ≥ 3 por condição, intercalado, mediana e faixa reportadas.

**As quedas deixaram de ser variância:** 2 em 3 corridas depois da câmera
comprimida, contra 0 em 2 antes. Sem mecanismo identificado e sem causa
demonstrada, mas é item de investigação, não nota de rodapé — estabilidade é
pré-requisito de qualquer meta.

**Ordem sugerida pelos dados para a próxima sessão:** reduzir CPU do Nav2 no
módulo → tirar a imagem RAW do fio (transporte comprimido até a percepção) →
só então mexer no MPPI → repetir 420 s / 200 s com n=3.
---

**Decisão tomada: alvo trocado de A1 para Go2** (ver "F2 — verificação
executada"; a justificativa de licença dada em F2 estava incompleta e foi
corrigida em F3 — ver "F3 — o rastreamento de licença"). **F3 rodou e o portão
bateu**: Go2 em pé, estável, andando por `/demo/cmd_vel` com os pacotes e o
launch do projeto, não mais com o spike. A falha de HOLD de F4 foi corrigida em
20/08 e o contrato completo com perception foi revalidado em 24/08. Evidência
de marcha em `docs/results/ml35-postura-parada.md` e do fechamento abaixo.

Plano de movimentação vigente: **`docs/ml35/plano-movimentacao.md`** (19/08/2026).
Substitui `plano-proximos-passos.md`, cujas Fases 1-3 já foram executadas.

Trabalho paralelo em aberto — **cockpit unificado**: plano aprovado em
24/08/2026 e **F1 concluído no mesmo dia**. Eixo trocado de "capturar janelas
X11" (quatro tentativas falhas) para "renderizar a partir de tópicos ROS 2", num
cockpit web que depois vira o HMI do M3. Decisões, evidências e fases em
**`docs/ml35/plano-cockpit-web.md`**; evidência do F1 (capturas de tela, taxas,
reconexão) em **`docs/results/cockpit-web-f1.md`**. O checkpoint anterior
(`docs/results/cockpit-standalone-parcial.md`) está marcado como superado; não
retomar a recomendação dele.

Estado do cockpit por fase: **F1 e F3b fechados** (24/08/2026). O F1 subiu os
serviços `cockpit` e `hmi` em `compose.host.yml`, o bundle em `hmi/`, a câmera ao
vivo e a reconexão automática. O **F3b** fechou o painel azul (duas câmeras de
cena estáticas, alternáveis) e o verde (costmap, plano, laser, pegada, e clique
que manda meta), com o portão cumprido: uma meta clicada no canvas foi aceita e
executada pelo Nav2. Evidência em **`docs/results/cockpit-web-f3b.md`**; como rodar e o que cada
painel faz, em **`docs/guia-completo.md`** (Parte II).

Em 25/08/2026, três **ajustes de UI** pedidos na bancada, fora da numeração de
fases e sem abrir fase nova: marca Toradex ao dobro, câmeras de cena seguindo o
robô nas duas vistas, e "reiniciar nav" a partir do cockpit. Evidência em
**`docs/results/cockpit-web-ui-ajustes.md`**. Um achado com peso próprio saiu daí:
`RESET`+`STARTUP` no `lifecycle_manager` do Nav2 **derruba o container** com
`SIGSEGV` ao configurar o `route_server`, reproduzido duas vezes — por isso o
reset usa `PAUSE`/`RESUME`. Candidato a issue upstream; ver a armadilha 8 da
Parte II de `guia-completo.md`. **Próximo do cockpit segue sendo o F4** (controle manual atrás
do `twist_mux`).

Na mesma data entraram, a pedido do operador: controle da simulação pelo cockpit
(play/pause/reset), controle de câmera (girar, inclinar, mover, zoom,
recentrar), a identidade visual Toradex (fundo branco, `#00508c`, `#96c837`,
`#ff5a00`, com as marcas Toradex e ROS na barra) e o aumento de qualidade das
câmeras de cena. Três pontos que valem carregar para a próxima sessão:

1. **"Iniciar a simulação no target" não é possível** e não foi feito. O Gazebo
   é OGRE 2; o AM69 só tem OpenGL ES 3.2/Vulkan 1.2 (regra 1). O que existe é
   play/pause/reset **a partir do** cockpit, agindo sobre o Gazebo do host.
2. **O navegador não fala tipos do Gazebo.** Chamar `ControlWorld` direto pelo
   rosbridge falha com `InvalidModuleException` — o container do cockpit não tem
   `ros_gz_interfaces`, e no M3 ele roda no módulo. A fronteira é `std_srvs`, e a
   tradução mora no nó `sim_control_relay`, do lado do simulador.
3. **Resolução de câmera custa RTF.** Com as duas câmeras de cena a 1600x1200,
   `update_rate 15` entrega 9,43 Hz com fator de tempo real **0,59**, e
   `update_rate 10` entrega 9,77 Hz com **0,97** — pedir 15 não rende um quadro
   a mais e custa 40% da velocidade da simulação. Adotado 10. Ver a seção 5 de
   `cockpit-web-f3b.md`.

Nada disso foi executado em arm64 nem no Aquila. **F2 e F4 do cockpit seguem
abertos** (kiosk no módulo e controle manual com `twist_mux`).

### 21/08/2026 — qualidade de navegação no maze11 (dentro de F5)

Cenário S6 passou para o **`maze11`**, com partida no canto inferior direito
(`docs/results/ml35-labirinto.md`). Em seguida, a qualidade de decisão do Nav2
foi medida e corrigida: **0,0399 → 0,0650 m/s (+63%)**, ré **11–62% → 0%**,
eficiência de trajeto **13% → 57%**, e a **primeira meta cumprida** (8 m em 96 s).
Evidência e limites em **`docs/results/ml35-navegacao-maze11.md`**.

Três defeitos, todos de decisão e nenhum de sensor:

1. `vx_min: -0.10` produzia **deadlock**: o robô recuava, encostava na parede e ré
   continuava ótima. Medido em 100% das amostras com o robô parado em 0,00 m.
   Agora `vx_min: 0.0`, com `wz_max` 0,12 → 0,20 para o giro ser alternativa real.
2. **Nenhum behavior tree do Nav2 Jazzy chama `SmoothPath`**, então o
   `smoother_server` estava ativo e ocioso e o MPPI perseguia a escada crua do
   NavFn. Agora há `demo_navigation/behavior_trees/nav_to_pose_smoothed.xml`.
3. NavFn escolhia rota por comprimento. Inflação do costmap **global** foi para
   0,85 / 2,0 — divergindo do local de propósito, no sentido seguro.

Lidar e odometria foram verificados a pedido e **estão sãos** (odom vs TF com erro
0,0000 m; sem auto-colisão de lidar). Ferramentas novas: `scripts/sensor_check.py`,
`scripts/costmap_probe.py`, `scripts/selfhit.py`.

**Não fechado:** folga de carcaça segue em **+6,5 cm** e é o portão de qualquer
aumento futuro de velocidade. Vem de `robot_radius: 0.38` modelar o tronco como
círculo; a correção é footprint poligonal com `consider_footprint: true`.

### 21/08/2026 — HIL de pé no Aquila AM69 (dentro de F5)

**A aplicação roda no módulo.** Nav2 arm64 ativo no Aquila AM69, composto num
processo único, simulador no host, enlace DDS bidirecional verificado. Build
arm64 **nativo no módulo**, regra 1 verificada nas quatro imagens.
Evidência e limites em **`docs/results/ml35-hil-aquila.md`**.

**O módulo não é o gargalo.** O gargalo é o stream de câmera de **74,2 Mbit/s**
(640×480 rgb8 a 10,1 Hz, medido no fio) atravessando o Wi-Fi:

| Condição | Módulo | Câmera no fio | Vel. média |
|---|---|---|---|
| host-only, DDS multicast default | parado | não | 0,0720 m/s |
| host-only, DDS de HIL | parado | não | **0,0725 m/s** |
| HIL, Nav2 + perception | ativo | sim | 0,0197 m/s |
| HIL, só Nav2 | ativo | não | **0,0427 m/s** |

A configuração de CycloneDDS com peers explícitos **não custa nada** — hipótese
levantada e refutada. Nav2 no módulo custa 1,7×; a câmera custa outros 2,2×.

**Composição do Nav2**: `nav_quadruped.launch.py` passou a criar o
`nav2_container`. Memória do container `nav` **6,89 GiB → 307 MiB**, load **21,9
→ 9,5**, ativação em **~10 s**. CPU total não mudou.

**Estrangular o `/clock` foi tentado, medido e revertido**: a 100 Hz a CPU caiu
de 470% para 324% e a navegação morreu (0,0039 vs 0,0251 m/s). O nó fica no
pacote com o A/B no cabeçalho, fora do caminho default.

**Não fechado, localizado:** a razão de trabalho do `cmd_vx` é baixa nas duas
máquinas — pico normal (0,10–0,14), médio 0,006–0,008. A largada do maze11 exige
giro parado de ~85° e o MPPI comanda `wz = 0,035` rad/s, 17% do teto. Loop de
controle, TF, costmap e `collision_monitor` foram descartados por medição.
Hipótese sem medida: `PathAlignCritic` em 14,0 contra `PathAngleCritic` em 2,0.

**Decisão do operador em 24/08/2026:** preservar 640×480 a 10 Hz e migrar o HIL
para Ethernet. F5 só fecha depois da corrida real nesse enlace; não inferir o
resultado a partir da banda medida no Wi-Fi.

### 24/08/2026 — HIL Ethernet executado, portão longo ainda aberto

O enlace foi executado de verdade: host `enp0s31f6` e Aquila `ethernet1`, com
peers CycloneDDS fixados em `192.0.2.6` e `192.0.2.5`. As duas portas do
Aquila na mesma sub-rede anunciam o mesmo hostname mDNS; deixar `MODULE_IP`
implícito alternou entre os dois endereços. A configuração local agora fixa uma
porta antes de `module.sh sync`.

Dois defeitos de QoS só apareceram com amostras fragmentadas no HIL. A câmera
RAW de 921600 bytes precisava de leitor `RELIABLE`; a nuvem LiDAR precisava de
produtor `SENSOR_DATA` para os leitores `BEST_EFFORT` do Nav2. Depois das duas
correções, câmera, detecções e nuvem de detecções fluíram a ~10 Hz, e o
`collision_monitor` deixou de rejeitar comandos por fonte antiga.

Uma meta curta fechou `SUCCEEDED` em **28 s**, com Nav2 + percepção no AM69,
Gazebo/RViz/câmera no host e zero queda. O protocolo final, porém, não bateu o
portão: **419,9 s, 8,31 m de caminho, 0,0198 m/s, 0 metas de 8 m concluídas**
(dois prazos de 200 s). O valor repete o Wi-Fi com percepção (0,0197 m/s),
refutando a hipótese de que trocar apenas o meio físico removeria o gargalo. O
custo restante está no caminho de processamento/cópias/fragmentação da câmera e
na baixa razão de trabalho do MPPI.

Evidência completa e CSVs em **`docs/results/ml35-hil-ethernet.md`**. F5 segue
aberta até uma meta de 8 m terminar `SUCCEEDED` no protocolo de 420/200 s.

**Atenção ao ler aquele plano:** o bloqueador que ele registra — "a árvore TF não
fecha, não existe frame `odom`" — **foi resolvido em 20/08/2026**. A árvore agora
tem 22 arestas, 9 estáticas, raiz `map`, e o Nav2 planeja e desvia sobre o
quadrúpede. Ver a seção "F5 — Nav2 sobre pernas" abaixo.

Fase A (parametrizar a marcha + banco de ensaio versionado) concluída em
19/08/2026.

Fase B (Defeito 2, queda em `HOLD` prolongado) **concluída em 20/08/2026** com
`hold.settle_rate: 0.02`, que virou default em `gait_go2.yaml`. A correção que
parecia óbvia — baixar só a entrada de guinada de `balance.weight_moment` de 450
para 100 — foi ensaiada e **REJEITADA**: adiantou o colapso de 161,7 s para 91,1 s
e levou `RECOVER` de 74 para 357 na mesma janela. O `gait_go2.yaml` registra isso
ao lado do parâmetro para que ninguém retente. Evidência em
`docs/results/ml35-postura-parada.md`.

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

## F4 — concluída 24/08/2026

> **Continuidade:** esta seção registra o checkpoint de 18/08. A queda em HOLD
> foi corrigida em 20/08 por `hold.settle_rate: 0.02`, com os três critérios de
> marcha verdes; ver `docs/results/ml35-postura-parada.md`. A revalidação
> conjunta do contrato e de perception foi executada em 24/08.

Checkpoint detalhado em `docs/results/ml35-f4-parcial.md`. Nomes, tipos e
mensagens reais de odom, scan e imagem atravessaram dois containers por DDS. O
primeiro mapeamento SI → stick derrubou o Go2 e foi substituído por clamp no
envelope `0.03` comprovado em F3, mas essa última edição ainda não foi
revalidada em runtime. Perception, warehouse oficial e regressão diff-drive
estavam pendentes naquele checkpoint.

**Fechamento em 24/08/2026:** cold start do perfil `learn` com Go2, warehouse,
Nav2 e perception em containers distintos. Os cinco tópicos foram descobertos
com os tipos do contrato: `geometry_msgs/msg/Twist`, `nav_msgs/msg/Odometry`,
`sensor_msgs/msg/LaserScan`, `sensor_msgs/msg/Image` e
`vision_msgs/msg/Detection2DArray`. Foram recebidas mensagens reais de odom,
scan, imagem 640 px e detecção sintética no consumidor. `/clock` avançou, a TF
fechou e o Nav2 chegou a `Managed nodes are active`. Isso fecha o portão F4 sem
fazer alegação de desempenho ou de hardware.

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

Esse era o estado em 18/08. A correção e os critérios substitutos estão no
relatório de 20/08 citado acima; não usar este parágrafo histórico para escolher
o próximo experimento.

---

## Preparação do target — 20/08/2026

**Não é uma fase.** É trabalho de infraestrutura para F5, feito em paralelo aos
ensaios de F4 no host, porque o módulo ficou acessível. Evidência completa em
`docs/results/ml35-target-preparacao.md`.

O que mudou de estado no projeto:

1. **A premissa "o módulo não está acessível" caiu.** Aquila AM69 inventariado:
   Torizon OS 7.7.0+build.40, 8 × Cortex-A72, 31 GiB RAM, 108 G livres, Docker
   25.0.9 arm64, Compose 2.26.0, `torizon` no grupo `docker`. `ethernet0` em
   `192.0.2.16/24`; host x86 em `192.0.2.15` na mesma /24.
2. **A rede DDS foi medida, não assumida.** UDP nas duas direções em três portas
   do domínio 69. `ufw` está ativo no host e **não bloqueia**. Nenhuma mudança de
   firewall é necessária.
3. **A regra 1 estava sendo violada pela árvore atual, em silêncio.** A camada de
   container é de F1 (diff-drive); F3 trouxe `gz_quadruped_hardware`, que declara
   `gz_sim_vendor` e `gz_plugin_vendor` como `<depend>`. Um `colcon build` cego
   do `src` colocaria OGRE 2 nas imagens arm64. Corrigido por dois build args
   (`SKIP_KEYS_EXTRA`, `COLCON_IGNORE_PACKAGES`), ambos default vazio — o lado
   amd64/host não muda.
4. **`autodetermine` no `module.xml` era uma armadilha real**, não teórica: a
   bridge Docker do easy-pairing da Toradex (`br-*`, 192.0.2.9) está UP junto
   com `ethernet0`. A interface agora é fixada em tempo de renderização, e o peer
   do host é injetado ali também, então nenhum endereço entra no git.
5. **`scripts/module.sh`** passou a ser a interface para o módulo:
   `inventory | sync | build | up | down | status | verify | shell`.
6. **Quatro imagens `arm64` existem no módulo**, construídas nativamente lá:
   `base` 1,24 GB, `perception` 1,28 GB, `tools` 1,32 GB, `nav` 2,44 GB. Regra 1
   verificada nas quatro por inspeção das bibliotecas instaladas.
7. **O contrato atravessa a fronteira de máquina nas duas direções, medido.**
   Domínio 69, `/demo/system/heartbeat`: módulo→host `count=11` recebido no host;
   host→módulo `count=14` recebido dentro do container `tools`, com
   `/demo/heartbeat_publisher` visível em `ros2 node list` do módulo. **Isto é o
   pré-requisito de infraestrutura de F4/F5, não o portão deles.**
8. **Descoberto que configurar só um lado do DDS falha idêntico a firewall.**
   O default do CycloneDDS anuncia por multicast (que o módulo ignora) e não fixa
   porta determinística (então o unicast do módulo não tem alvo). Os dois lados
   precisam de config casada. `scripts/module.sh` renderiza os dois:
   `module.xml` para o módulo e `docker/cyclonedds/host.rendered.xml` no host,
   ambos com endereço injetado e gitignored/gerado.
9. **`ROS_NAMESPACE` não funciona no ROS 2 Jazzy.** Verificado: a variável está
   no ambiente do processo (`printenv` confirma) e o ROS a ignora; só
   `--ros-args -r __ns:=` funciona. `scripts/env.sh` exporta
   `ROS_NAMESPACE=/demo` como se funcionasse — **não foi alterado**, o arquivo
   está em uso pelos ensaios de F4. Fica como achado a resolver.

O que **não** mudou, e precisa ficar claro:

- **F5 continua bloqueada pelo mesmo motivo de antes.** O target estar pronto não
  resolve a árvore de TF que não fecha nem a ausência do frame `odom`
  (`plano-movimentacao.md`). Nav2 sobre pernas não passa o portão de F5 por falta
  de `odom`, independentemente de o módulo estar de pé.
- **Nada de desempenho foi medido** (regra 5 e 7). As imagens foram construídas
  nativamente no módulo em vez de sob QEMU, o que não é uma medição de nada.
- **O módulo não vê os tópicos da simulação do host.** Não é defeito do módulo:
  `scripts/run_quadruped_sim.sh` sobe a sim sem `CYCLONEDDS_URI`, então ela
  anuncia por multicast e o módulo (multicast off) não tem como descobri-la. O
  mecanismo está provado nas duas direções com publishers de teste; falta passar
  o config renderizado ao produtor do host. **Não alterado nesta sessão porque
  esse script está em uso pelos ensaios de F4.**
- **`compose.host.yml` continua montando `cyclonedds/host.xml`**, o template sem
  o peer do módulo. Para o `hil` containerizado ele precisa apontar para
  `host.rendered.xml`.
- **`nav` não foi subido no módulo.** Nav2 publica `/demo/cmd_vel`, e a simulação
  do host roda no mesmo domínio 69: dois publishers no tópico que comanda o robô
  corromperiam o ensaio em curso sem nada em log explicando. `scripts/module.sh
  up` detecta a simulação ativa e recusa por default.

---

## F5 — Nav2 sobre pernas: em andamento 20/08/2026

Malha fechada e funcionando: nuvem 3D → costmap → planejador → MPPI → conversão de
unidades → marcha → Gazebo → odometria → TF → costmap. Medido em
`quadruped_objects.sdf`, o robô percorreu 8,36 m, deslocou 3,51 m líquidos, chegou
a **3,8 cm** da meta e passou pelos quatro obstáculos com folga positiva, sem cair.

Evidência completa em **`docs/results/ml35-nav2-quadrupede.md`**; como rodar, em
**`docs/guides/cenarios/s5-nav2-desvio.md`**.

### Os três bloqueadores de F5 estão fechados

| bloqueador | como foi fechado | consequência a lembrar |
| --- | --- | --- |
| árvore TF não fecha | `demo_bringup/odom_tf` publica `odom → base` e `map → odom` | **não é estimativa de estado** — é ground truth do Gazebo virando TF; sai quando o estimador com perna existir |
| nome do frame base | quem cedeu foi o Nav2: `nav2_params_go2.yaml` usa `base` | `go2_description` é vendorizado byte-a-byte e não pode ser editado |
| lidar de um anel | a ponte expõe `/scan/points` como `PointCloud2` em `/demo/scan_cloud` | `/demo/scan` continua existindo e continua inútil para costmap |

O número que fecha o terceiro: no mundo dos objetos, `/demo/scan` dá **zero**
obstáculos — idêntico ao mundo vazio — e `/demo/scan_cloud` dá **249**.

### Seis defeitos encontrados por medição, todos corrigidos

Nenhum deles se anuncia em log. Estão listados porque cada um custaria horas de
novo.

1. **`use_composition` sem container.** `navigation_launch.py` com composição
   carrega os servidores em `/nav2_container`, que só o `bringup_launch.py` cria.
   Incluindo apenas o primeiro: nada sobe, nada dá erro.
2. **Metas concorrentes.** Entre `send_goal_async` e a aceitação, o handle é
   `None`; o supervisor de 1 s reentrava e mandava outra meta.
3. **`progress_checker` do TB4.** 0,5 m em 10 s, contra 13 s de giro a 0,12 rad/s
   sem avanço: 22 abortos com **zero quedas**. Quando o verificador reprova e o
   robô não cai, o suspeito é o verificador.
4. **Horizonte do MPPI medido em tempo.** 2,8 s cobrem 1,4 m no TB4 e 0,42 m no
   Go2 — abaixo da referência de ~1 m do `PathAlignCritic`, que tem o maior peso.
   Horizonte se mede em distância.
5. **`/demo/cmd_vel` não está em SI.** Carrega manche; o controlador multiplica
   `linear.x` por 0,4 e `angular.z` por 0,5 (`StateTrotting.cpp:192` com
   `invNormalize`, e `twist_to_inputs.py:283` com ganho unitário). O Nav2 é o
   primeiro consumidor que não pode viver com isso, porque o MPPI **integra** `vx`
   como m/s. Corrigido com `demo_bringup/cmd_vel_si_to_stick`, um nó de fronteira
   — a planta e os comandantes existentes ficaram intactos.
6. **Yaw da meta como rumo de saída.** Exigia 110–139° de giro parado na chegada,
   e girar parado não fica parado: o robô derivou 0,78 m em y e saiu da tolerância
   de posição que já havia satisfeito. O yaw tem de ser o rumo de **chegada**.

### Duas hipóteses refutadas por medição

Registradas para que ninguém as retente:

- **"O robô está dentro de região inflada."** Medido com o robô parado: custo
  **0** na célula dele, **0** dentro de 0,6 m, 42 células letais nos obstáculos,
  zero desconhecidas. O costmap está correto.
- **"A dispersão de amostragem do MPPI limita a magnitude."** Só a correção de
  unidades levou o comando de 0,006 para 0,119 m/s **com os mesmos desvios**.
  `vx_std` e `wz_std` ficaram como estavam.

### O que F5 ainda não tem

- **Estimativa de estado com perna.** `odom_tf` republica ground truth. Enquanto
  isso, nada aqui valida localização.
- **Meta de 8 m no portão HIL Ethernet.** Ethernet, câmera RAW, percepção e uma
  meta curta já passaram no Aquila. No protocolo de 420 s / 200 s por meta, as
  duas metas longas expiraram. Isolar custo da câmera e razão de trabalho do
  MPPI, sem reduzir 640×480 a 10 Hz, e repetir o mesmo protocolo.

## F6 — fallback selecionável: concluída 24/08/2026

`ROBOT_TYPE=quadruped|diffdrive` agora seleciona em conjunto a planta do host e
o launch Nav2 correspondente, tanto no Compose do host quanto no do módulo.
`quadruped` é o padrão; valor desconhecido falha antes de iniciar Nav2. Os testes
unitários verificam o pareamento e a rejeição de valores inválidos.

Portão executado a partir de subidas limpas do perfil `learn`:

- `quadruped`: TF fechada, Nav2 ativo e meta curta de x≈−1,59 para x=−0,8 com
  `SUCCEEDED`, `error_code: 0`;
- `diffdrive`, com `SIM_GUI=false`: odometria disponível e meta de x=0 para x=1
  com `SUCCEEDED`, `error_code: 0`.

As imagens de base, simulação, navegação, percepção, ferramentas e visualização
foram reconstruídas. O backend `gz_quadruped_hardware` existe apenas na imagem
de simulação; `COLCON_IGNORE` mantém as funções headless isoladas também em
builds e testes incrementais posteriores.

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
- ~~O módulo não está acessível~~ — **PREMISSA CAÍDA em 20/08/2026.** O Aquila
  AM69 respondeu e foi inventariado; ver `docs/results/ml35-target-preparacao.md`
  e a seção "Preparação do target" acima. `ssh torizon@` e `rsync` agora rodam
  por `scripts/module.sh`. A regra 7 continua valendo integralmente: nada de
  desempenho, latência, térmica ou FPS foi medido nem é reivindicado.
- ~~`eth0` nos XMLs de DDS é placeholder~~ — **RESOLVIDO para o módulo.** As
  interfaces verificadas são `ethernet0` e `ethernet1`; no HIL de 24/08 foi
  escolhida explicitamente `ethernet1`, e ela é **fixada em tempo de
  renderização**, detectada a partir de `MODULE_IP`, não escrita à mão:
  `autodetermine` pode escolher a bridge Docker do easy-pairing
  (`br-*`, 192.0.2.9), que está UP ao mesmo tempo. O `host.xml` é template; o
  `module.sh sync` gera `host.rendered.xml` com `enp0s31f6` e o peer escolhido.
  Com as duas portas na mesma sub-rede, `MODULE_IP` deve ser explícito porque o
  mesmo hostname mDNS pode resolver para qualquer uma delas.
- `tools` aparece em `docker compose exec tools` no guia §9 mas não está
  declarado no compose §6. Será declarado com `profiles: ["tools"]` e um
  `command` que não encerra.
