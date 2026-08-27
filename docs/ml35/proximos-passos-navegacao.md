# Próximos passos — decisão de trajeto do Nav2 (F5)

Escrito em 26/08/2026, depois de fechar o reset do cockpit.
Estado autoritativo em `docs/ml35/estado-fases.md`; este arquivo é só o roteiro
da próxima frente.

---

## 1. O sintoma, e o que ele já NÃO é

O robô anda — a marcha está sã — mas não cumpre rota. Última medição
(`docs/results/ml35-f5-clock-fanout.md`):

| métrica | valor |
| --- | ---: |
| `vx` ≈ 0 (`\|cmd_vx\| <= 0,005 m/s`) | **90,7%** das amostras |
| girando | **90,3%** das amostras |
| velocidade média | 0,0246 m/s |
| metas de 8 m cumpridas | **0 de 2** |
| quedas | 0 |

**Cinco hipóteses estão refutadas por medição. Não as reabra:**

| hipótese | como morreu |
| --- | --- |
| CPU do módulo | fechada: 307% de folga de 800%, **zero** recusas do `collision_monitor` (`ml35-f5-clock-fanout.md`) |
| taxa do `/clock` | estrangular derrubou a navegação em 21/08 (`demo_simulation/clock_throttle.py`) |
| amostragem do MPPI | reprovada em `ml35-f5-mppi-amostragem.md` |
| `inflation_radius` fechando o corredor | **64,1%** das células do `local_costmap` com custo 0; o que estava à frente era parede real |
| enlace de rede | gigabit full, RTT 0,400 ms, rota simétrica; as duas metas estouraram igual (`ml35-f5-ethernet0-repeticao.md`) |

Lidar e odometria foram auditados e **estão sãos** (odom vs TF com erro
0,0000 m, sem auto-colisão de lidar). O defeito é de **decisão**.

## 2. Protocolo primeiro. Não sintonize antes disso

O número que invalida qualquer corrida única: em configuração **idêntica** a
velocidade média variou **2,4×** entre corridas. Uma perna por condição não
distingue efeito de ruído — nem quando o efeito é real.

`scripts/nav_campaign.py` existe para isso. Ele intercala `A B A B A B` em vez de
blocar `A A A B B B`, e repõe robô e costmap antes de cada perna:

```bash
. /opt/ros/jazzy/setup.bash && . ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/docker/cyclonedds/host.rendered.xml

python3 scripts/nav_campaign.py docs/results/campanha-align \
    --condition baseline='<comando que reaplica o baseline>' \
    --condition align8='<comando que aplica a condição>' \
    --reps 3 --seconds 420
```

`--dry-run` imprime a ordem e o tempo de pista sem gastar bancada. Com 2
condições × 3 replicatas × 420 s são **42 min de pista**, fora bring-up e
reposição.

Só é possível porque o reset deixou de apagar o robô (commit `7a5e539`). Com o
reset antigo, a perna 2 em diante mediria um mundo sem planta — relógio andando,
sensores órfãos publicando, nada acusando. **Se aquele conserto for revertido,
este laço passa a mentir.**

**Métrica primária é razão de trabalho de `cmd_vx` e fração de `vx` ≈ 0**, não
velocidade média: deram 0,6% e 0,6% em duas corridas distintas, contra 2,4× de
dispersão na média. `summarize_trials.py` já reporta mediana e faixa por
condição, sem agrupar amostras.

## 3. Passo 0 — condição aplicável de forma confiável

**Feito depois deste roteiro inicial.** A primeira campanha ainda depende deste
contrato, então mantenha a invariável: condição é arquivo, não `ros2 param set`.
As duas saídas óbvias falham:

- `ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0`
  é **aceito e lê de volta o valor novo**, mas os pesos dos críticos são lidos no
  `on_configure` do controlador. Uma campanha construída sobre isso compara a
  condição consigo mesma, com evidência de aparência perfeita. É por isso que
  `nav_campaign.py` **não** aplica condição por conta própria: quem aplica é um
  comando fornecido por quem roda.
- `nav_select.launch.py` recebe `params_override` e repassa `params_file`
  **somente quando o valor
  não está vazio**. Vazio quer dizer "use o default do launch do robô
  selecionado"; repassar vazio ao filho esmagaria o default do Go2/TB4 com uma
  string vazia. `tests/test_nav_params_override.py` trava esse contrato.

O YAML instalado é symlink para dentro da imagem, não para o host:

```
/ws/install/demo_navigation/share/demo_navigation/config/nav2_params_go2.yaml
  -> /ws/build/demo_navigation/config/nav2_params_go2.yaml
  -> /ws/src/demo_navigation/config/nav2_params_go2.yaml     (COPY na imagem base)
```

O estado esperado agora é:

- `nav_select.launch.py` declara `params_override` com default vazio para chamadas
  diretas e reconhece o sentinela `__robot_default__` usado pelo Compose;
- `docker/compose.host.yml` e `docker/compose.module.yml` passam
  `params_override:=${NAV2_PARAMS:-__robot_default__}`;
- se `NAV2_PARAMS` estiver vazio, cada robô usa o próprio default;
- se `NAV2_PARAMS` apontar para um YAML dentro da imagem, esse arquivo vira a
  condição da perna.

Com isso, os dois comandos de aplicação ficam:

```bash
--condition baseline='ssh torizon@<modulo> "cd /home/torizon/demo &&
    NAV2_PARAMS=__robot_default__ \
    docker compose -f compose.module.yml up -d --force-recreate nav"' \
--condition align8='ssh torizon@<modulo> "cd /home/torizon/demo &&
    NAV2_PARAMS=/ws/src/demo_navigation/config/params-align8.yaml \
    docker compose -f compose.module.yml up -d --force-recreate nav"'
```

Reaplicar o baseline não é redundante. A ordem é `baseline align8 baseline
align8`; sem comando no segundo baseline, ele herdaria `params-align8.yaml` da
perna anterior e a campanha compararia align8 consigo mesma. O script agora
recusa qualquer comparação com condição sem comando.

com o YAML da condição salvo em `ros2_ws/src/demo_navigation/config/`,
sincronizado por `module.sh sync` e incorporado por `module.sh build` antes. O
caminho é dentro da imagem (`/ws/src/...`), não no diretório remoto `~/demo`:
`compose.module.yml` só monta o XML do CycloneDDS.

## 4. Passo 1 — a hipótese a testar primeiro, e por quê

**`PathAlignCritic.cost_weight: 14.0`.** É o maior peso da lista, e o próprio
YAML já o nomeia duas vezes como o crítico que torna girar melhor que avançar:

- no bloco `POR QUE vx_min E ZERO`: *"num corredor de 1.20 m o PathAlignCritic
  (peso 14, o maior) acha ótimo recuar para realinhar"* — com `vx_min: -0.10`
  isso produzia deadlock contra a parede de trás, medido com ré em 100% das
  amostras;
- no item 13 do cabeçalho: o horizonte curto *"não alcança a referência do
  próprio PathAlignCritic (peso 14, offset 20 pontos ~ 1 m)"*.

`vx_min: 0.0` fechou a saída "recuar" — corretamente, porque ré contra a parede
era travamento. Mas a preferência por **realinhar antes de avançar** continua
intacta, e a única forma de realinhar que resta é **girar**. É consistente com o
sintoma medido: `vx` ≈ 0 em 90,7% e girando em 90,3%.

Contra-pesos que empurram para frente: `PreferForwardCritic` 5,0,
`PathFollowCritic` 5,0, `GoalCritic` 5,0 — cada um com pouco mais de um terço do
peso do alinhamento.

Condições sugeridas para a primeira campanha, uma variável por vez:

| condição | mudança | o que ela testa |
| --- | --- | --- |
| `baseline` | nenhuma | referência, com a dispersão medida sob o protocolo novo |
| `align8` | `PathAlignCritic.cost_weight` 14,0 → 8,0 | alinhamento deixa de dominar o progresso |

Só depois, e uma por campanha: `PathAngleCritic` 2,0 (o portão 3 já registrado),
e `PathFollowCritic` 5,0 → 8,0.

**Critério de parada que não se negocia:** folga de carcaça segue em **+6,5 cm**
e é o teto de qualquer aumento de velocidade. Vem de `robot_radius: 0.38` modelar
o tronco como círculo; a correção de verdade é footprint poligonal com
`consider_footprint: true`, e ela não é pré-requisito desta campanha.

## 5. O que decide

Uma condição vence se, com n ≥ 3 intercalado, a **mediana** da razão de trabalho
de `cmd_vx` subir e a fração de `vx` ≈ 0 cair, **e as faixas não se sobrepuserem**.
Faixas sobrepostas com n = 3 significam "não decidido", não "empate": aumente n
antes de escolher.

Meta de 8 m cumprida continua sendo o portão do F5, mas não é a métrica de
sintonia — é rara demais para discriminar entre condições.

## 6. Pendências fora desta frente

- **F4, controle manual no cockpit.** Bloqueado por infraestrutura, não por
  desenho: `twist_mux` não está em nenhuma imagem e o módulo **não tem rota
  default** (só a `192.0.2.1/24`), então `apt` não resolve nada lá. O gateway da
  LAN `192.0.2.2` responde em 0,337 ms e o módulo já tem DNS corporativo, então
  falta um comando, que precisa de permissão para rodar:

      sudo ip route add default via 192.0.2.2 dev ethernet0 metric 100

  Não é NAT: o módulo está na mesma `/24` do gateway, então não há o que
  masquerar e o firewall do host não precisa ser tocado. Para sobreviver a
  reboot, a rota tem de entrar na configuração de rede do Torizon, não só em
  runtime.
- **Passada visual do cockpit.** Nada nesta sessão foi visto num navegador: não
  há Chrome nesta máquina e o MCP de automação não dirige o Firefox instalado.
  Tudo foi medido no caminho de dados, com o cliente rosbridge do próprio
  cockpit. O gate visual continua pendente.
- **Telemetria do alvo não commitada.** `target_monitor` e as mudanças de
  `hmi/` estão verificadas (`docs/results/cockpit-reset-nao-destrutivo.md` §5)
  mas seguem fora do git, junto de outro trabalho da sessão anterior
  (`module.sh`, `wait_for_tf.py`, `sim.launch.py`).

## 7. Achado do smoke HIL de 26/08/2026: colisão de blackboard na BT, corrigida

O smoke de baseline no HIL real (Aquila) mediu `cmd_vx` ≈ 0 durante os 60 s
inteiros: `controller_server` abortava `[follow_path]` a ~1 Hz, culminando em
`Failed to make progress` aos 40,1 s — exatamente `movement_time_allowance`.
A campanha baseline × align8 foi **pausada** neste ponto: rodar 3 reps × 2
condições × 420 s sobre um Nav2 que nunca sai do lugar não mede critic
nenhum.

**Causa raiz, confirmada contra o upstream.** `nav_to_pose_smoothed.xml` tinha
`SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"` — a MESMA chave
de entrada e saída, que é também a chave que `FollowPath` lê. `RateController`
(que recomputa+suaviza) e `FollowPath` são irmãos do mesmo `PipelineSequence`;
como `FollowPath` retorna RUNNING (ação longa), o próprio `PipelineSequence`
re-tica os irmãos anteriores a cada ciclo permitido pelo `RateController`
(1 Hz), reescrevendo `{path}` — e o `FollowPath` via isso como meta nova e
abortava o handle em andamento. Reproduzido e confirmado por um mantenedor do
Nav2 em [ros-navigation/navigation2#5817](https://github.com/ros-navigation/navigation2/issues/5817):
"we expect users to remap the smoothed path to a different blackboard
variable".

**Correção aplicada:** `smoothed_path="{smooth_path}"` e
`FollowPath path="{smooth_path}"`. Travada por
`tests/test_nav_to_pose_smoothed_bt.py` (RED antes da correção, GREEN depois).

**Validação, mesma meta (0,0)→(0,8), antes × depois no HIL real:**

| métrica | antes (60 s, bug) | depois (35 s, corrigido) |
| --- | --- | --- |
| `Aborting handle` / `Failed to make progress` no log | contínuo, ~1 Hz, 53 s | **zero** |
| `cmd_vx` pico / médio | 0,003 / 0,0000 m/s | 0,087 / 0,0109 m/s |
| `cmd_vx` ≈ 0 | 100,0% das amostras | 66,9% das amostras |
| razão de trabalho `vx` (> 0,05 m/s) | 0,0% | 8,6% |
| caminho percorrido / deslocamento líquido | 0,28 m / 0,07 m | 0,68 m / 0,23 m |
| tombamento / folga mínima | 0,59° / +0,065 m | 0,64° / +0,065 m (inalterado) |

A correção da BT explica a maior parte do sintoma (o abort contínuo some por
completo), mas `cmd_vx` médio continua bem abaixo de `vx_max` (0,15 m/s) e a
meta de 8 m não foi concluída em 35 s — coerente com a hipótese de confiança
média já registrada: pode restar uma contribuição separada do MPPI (mínimo
local perto do custo quase-plano do `PathAlignCritic`, §1). Isso é sinal a
investigar, não motivo para reabrir a campanha ainda: a decisão de retomar
baseline × align8, aprofundar o MPPI (`CriticsStats`, testes de meta angulada)
ou fazer um smoke mais longo primeiro fica para quem opera a próxima sessão.

## 8. Aprofundamento no MPPI de 26/08/2026: `CriticsStats` não existe; `offset_from_furthest: 20` está bem calibrado

Seguindo o Passo 2 da §2 (instrumentar o MPPI antes de tocar pesos), duas
afirmações da investigação compilada foram verificadas contra o pacote
`nav2_mppi_controller` real instalado (1.3.12, mesma versão no host e na
Aquila — `dpkg -l | grep nav2-mppi`).

**`CriticsStats` não existe nesta versão.** Inspecionando
`/opt/ros/jazzy/include/nav2_mppi_controller/*.hpp` diretamente: cada critic
implementa `score(CriticData & data)` (`critic_function.hpp`) e escreve no
MESMO acumulador compartilhado `xt::xtensor<float, 1> & costs`
(`critic_data.hpp`). Não há struct de estatísticas por critic, nem publisher
de diagnóstico, em lugar nenhum da API pública do pacote. Obter contribuição
numérica por critic exigiria fazer patch e rebuild do pacote — um
compromisso bem maior do que qualquer coisa já feita neste projeto (que usa
Nav2 de estoque + BT XML e params customizados, nunca um fork de pacote).

**A visualização de trajetórias (`visualize: true`) não é seguro alternar em
runtime.** `TrajectoryVisualizer` só cria seus `LifecyclePublisher`s dentro de
`on_configure()`, então um `ros2 param set` num nó já ativo não tem efeito.
Tentar forçar isso via `ros2 lifecycle set /controller_server deactivate` +
`cleanup` (para reconfigurar) acionou o `lifecycle_manager_navigation` a
resetar e reconfigurar TODOS os nós geridos de uma vez — e o
`nav2_container` inteiro morreu com SIGSEGV (`exit code -11`) reconfigurando
a operação `AdjustSpeedLimit` do `route_server` (sem grafo carregado). Um
`docker compose restart nav` simples e limpo reconfigurou os mesmos nós sem
crash — o crash parece específico do caminho de reset por perda de bond do
lifecycle manager, não de uma inicialização a frio. **Não cicle o lifecycle
de um nó gerido individualmente neste projeto; reinicie o container `nav`
inteiro se precisar reconfigurar algo.**

**Espaçamento real do caminho suavizado, medido (não presumido).** Chamando
`/compute_path_to_pose` e `/smooth_path` diretamente (mesmas chamadas de ação
que a BT faz, sem tocar no lifecycle dos nós geridos — zero risco), na mesma
meta (0,0)→(0,8) usada nos smokes:

| caminho | pontos | espaçamento médio | mediano | min / max |
| --- | --- | --- | --- | --- |
| bruto (`GridBased`/NavFn) | 260 | 0,0269 m | 0,0250 m | 0,0250 / 0,0707 m |
| suavizado (`simple_smoother`, 4 ms) | 260 | 0,0261 m | 0,0250 m | 0,0205 / 0,0707 m |

A resolução real (~0,026 m) é a metade da resolução assumida pela
investigação compilada (0,05 m, a grade do costmap). Recalculando a fórmula
do próprio README do MPPI com a resolução MEDIDA:

```
offset ≈ horizonte(9,6 s) × vx_max(0,15 m/s) ÷ resolução_medida(0,0261 m) ÷ 3
        ≈ 18,4 pontos
```

`PathAlignCritic.offset_from_furthest: 20` (o valor atual) está a menos de
10% desse recálculo. **A hipótese de que esse parâmetro está mal dimensionado
fica refutada** — veio do perfil padrão, mas por coincidência (ou não: a
grade global usa 0,05 m e o planner/smoother parecem densificar por um fator
~2) está bem calibrado. Não há razão, a partir desta medição, para tocar em
`offset_from_furthest` antes de reabrir a campanha.

Em aberto: a comparação foi feita num único trecho quase-reto da meta padrão;
não foi testado como o `simple_smoother` se comporta num caminho com curvas
reais (Passo 3 da §2, metas a 45°/90°, continua não executado).

## 9. Metas anguladas de 27/08/2026: não é mínimo local de bearing — é `cmd_vx` ~0 geral, já na saída crua do MPPI

Executado o Passo 3 da §2 (metas a 0°/45°/90° em relação ao heading de spawn,
mesmo corredor, ensaios de 20-28 s, abaixo do timeout de progresso de 40 s),
reaproveitando `MAZE11_GOALS` de `nav_trial.py` — `(0.00, 8.00)` cai
exatamente a 0° do heading de spawn (`x=0, y=0, yaw=1.5708`, de
`demo_simulation/scenarios.py`), `(-1.60, 1.60)` a 45°, `(-8.00, 0.00)` a 90°.

| bearing | meta | distância | razão de trabalho | pico/médio `cmd_vx` | deslocamento líquido |
| --- | --- | --- | --- | --- | --- |
| 0° | (0,00, 8,00) | 8,0 m | 0,0% | 0,002 / 0,0000 m/s | 0,07 m |
| 45° | (-1,60, 1,60) | 2,26 m | 0,0% | 0,001 / 0,0000 m/s | 0,00 m |
| 90° | (-8,00, 0,00) | 8,0 m | 0,7% | 0,051 / 0,0047 m/s | 0,10 m |

**Nenhum gradiente por bearing** — os três ficam perto de zero, e o 90° (que
seria o caso "difícil" pela hipótese do `PathAlignCritic`) não é pior que o
0°. Isso já responde ao Passo 3: o sintoma é incapacidade geral de avançar,
não um mínimo local específico de caminho lateralizado.

**Series de controle, todas negativas** (ou seja, nenhuma delas explica o
sintoma):

- Reset com 20 s de espera antes da meta (em vez dos 6 s padrão) — mesmo
  resultado. Descarta costmap ainda não repovoado após "costmaps limpos".
- Reenviar a mesma meta SEM resetar (estado já aquecido) — 0,00 m de
  deslocamento, igual ou pior. Descarta que o próprio reset seja a causa.
- `docker compose restart nav` completo (processo `nav2_container` do zero,
  não só os triggers de reset) — mesmo resultado. Descarta que o segfault do
  `route_server` (§8) tenha deixado algo degradado.
- `/demo/perception/detections` e `/demo/camera/image_raw` verificados
  publicando normalmente a ~10 Hz durante um trial preso — não é o stub de
  percepção parado, nem falta de dado no costmap. (A leitura inicial de
  "parado" foi falso positivo: `ros2 topic hz`/`echo --once` com timeout de
  6-8 s é curto demais para a primeira amostra chegar por este link.)
- Log do container `nav` no período: nenhuma menção a `Spin`, `BackUp`,
  `RecoveryActions` ou custo de obstáculo — a BT nunca cai no `Fallback`, só
  fica dentro do `PipelineSequence` replanejando e suavizando a 1 Hz sem
  nunca produzir velocidade de saída.

**Localizado dentro do próprio `controller_server`, não em filtro
posterior.** Medindo `/cmd_vel_nav` (saída crua do MPPI, ANTES do
`velocity_smoother`/`collision_monitor`) em vez de `/demo/cmd_vel_si`: mesmo
resultado (0,0% de trabalho, pico 0,001 m/s). Isso descarta
`collision_monitor`, `velocity_smoother` e `cmd_vel_si_to_stick` como a
causa — o MPPI já decide por uma saída quase nula, mesmo numa meta reta de
8 m sem obstáculo, com erro de heading zero.

**Hipótese "replanejamento a 1 Hz reseta o otimizador a cada tick" —
refutada por leitura de fonte** (`nav2_mppi_controller/src/controller.cpp` e
`nav2_controller/src/controller_server.cpp`, branch `jazzy`):
`setPlannerPath()` — chamado a cada tick do `RateController`, é a linha
"Passing new path to controller" do log — só atualiza o `goal_checker`,
nunca chama `controller->reset()`. O reset do otimizador roda exatamente uma
vez, no encerramento da meta (sucesso/cancelamento/falha) — bate com os
logs, que só mostram "Optimizer reset" uma vez, no cancelamento final. A
issue [ros-navigation/navigation2#4545](https://github.com/ros-navigation/navigation2/issues/4545)
("[MPPI] Zero velocity used after Optimizer reset") documenta que um reset
real causa ~2 s de saída quase nula até convergir — não os 20-28 s inteiros
observados aqui.

**Não é falta de mapa/localização persistente.** `nav_quadruped.launch.py`
usa custo rolante com `track_unknown_space: true` e `allow_unknown: true`
deliberadamente, para aceitar metas a 8 m sem mapa nenhum — documentado no
cabeçalho do próprio launch file como proposital, não uma lacuna. O projeto
já tem `slam_toolbox` funcionando (`slam.launch.py` + `slam_params.yaml`),
mas só no caminho diffdrive/TurtleBot4 (`base_frame: base_link`, que o Go2
não tem — mesmo motivo pelo qual o quadrúpede não usa AMCL). Adicionar mapa
persistente para o quadrúpede reverteria essa decisão documentada e não
tocaria no otimizador/critics do MPPI, que é onde o sintoma mora. **Não é a
frente certa para este bug.**

**Em aberto quando esta seção foi escrita.** Eliminado: bearing/critic
específico, transiente de costmap pós-reset, estado aquecido de
cancelamentos repetidos, resíduo do crash do `route_server`, percepção
parada, `collision_monitor`/`velocity_smoother`/conversor SI. Continuação
abaixo, no mesmo dia.

## 10. Continuação de 27/08/2026: o sintoma real não é `cmd_vx`≈0, é `cmd_wz` sustentado sem correção

Monitorando `/demo/odom` e TF `map`→`base` a 1 Hz durante um trial preso
(meta (0,0)→(0,8), 0° de bearing, o caso supostamente mais fácil) — TF e
odometria concordam entre si o tempo todo, sem salto nem divergência: não é
localização quebrada. Mas o `yaw` sobe monotonicamente e sem parar,
89,9° → 92,6° → 99,1° → 106,8° → 114,6° → 121,9° → 126,4° em 10 s, enquanto
`x`/`y` quase não mudam. O robô não está "parado" — está girando no
próprio eixo, ininterruptamente, para o mesmo lado.

Extraindo `cmd_wz` do CSV (que `nav_trial.py` grava mas o resumo impresso
nunca mostra): `cmd_wz` positivo em 65% das amostras, pico 0,1235 rad/s,
**nunca negativo em nenhuma amostra** — não há correção, só giro acumulando
para o mesmo lado, numa meta cujo erro de rumo inicial é ZERO (o robô nasce
de cara para a meta). `cmd_vx` continua perto de zero (pico 0,036 m/s).

Duas hipóteses concretas de causa foram checadas e refutadas:

- **Meta com orientação de chegada incompatível com o rumo do caminho?**
  Não. `nav_trial.py:_send` calcula `heading = atan2(target.y - previous.y,
  target.x - previous.x)` a partir da posição REAL do robô no início do
  ensaio (`previous = self.pose` em `run()`), não um valor fixo — para
  `(0,0)→(0,8)` isso dá exatamente 90°, igual ao yaw de spawn. A orientação
  de chegada enviada já é a mesma do caminho; não há conflito
  `GoalAngleCritic` vs `PathAngleCritic` por essa via.
- **`GoalAngleCritic` com tolerância assimétrica de yaw, favorecendo um
  lado?** Não. `symmetric_yaw_tolerance_` é `false` por DEFAULT no header
  (`goal_angle_critic.hpp:46`) e não está definido em
  `nav2_params_go2.yaml` — é comportamento padrão de estoque, não uma
  configuração deste projeto.

**Reformulação do sintoma para a próxima sessão:** não é "MPPI decide ficar
parado". É "MPPI comanda um giro sustentado, sem nunca inverter o sinal,
numa meta de erro de rumo zero e caminho reto." Isso é mais específico e
mais estranho do que a suspeita original do `PathAlignCritic` — um giro que
nunca se corrige não é o comportamento esperado de nenhum critic isolado
tentando alinhar com um caminho reto.

**Diagnóstico seguro e barato ainda não executado, e recomendado como
próximo passo:** `enabled` de cada critic é lido uma vez em `on_configure()`
(`critic_function.hpp`), então **não é seguro** testar "desligar o
`PathAngleCritic`/`GoalAngleCritic` e ver se o giro para" com
`ros2 param set` num nó já ativo — o padrão já mordeu uma vez nesta sessão
(§8, o crash do `route_server` ao ciclar o lifecycle de um nó gerido para
forçar reconfiguração). O caminho seguro é um `docker compose ... up -d
--force-recreate nav` com um arquivo de override de compose que monta uma
YAML de parâmetros adicional (`enabled: false` num critic por vez) SEM
tocar na imagem — evita tanto o rebuild caro (§ receita do cache de layers)
quanto o crash do lifecycle cycle. Ainda não tentado.

Também não comparado: as condições exatas do smoke pós-fix de 26/08 (razão
de trabalho 8,6% na mesma meta) contra os ensaios de 27/08 — algo relevante
pode diferir além do que já foi descartado aqui (candidato mais provável,
ainda não checado: se aquele smoke recebeu a MESMA orientação de chegada
calculada por `nav_trial.py`, ou se rodou antes de alguma mudança de estado
desta sessão).

## 11. Achado de 27/08/2026 (offline, custo zero de bancada): as quatro metas do ensaio estão atrás de parede, e o plano global atravessa parede

Nenhuma das dez seções acima verificou a premissa mais barata de todas: **a meta
que o ensaio manda é alcançável pelo caminho que o planejador desenha?**
Ferramenta nova, versionada, sem ROS e sem Gazebo — só leitura do STL:

```bash
python3 scripts/maze_geodesic.py            # maze11, MAZE11_GOALS, escala do SDF
```

Ela mede a geodésica pelo espaço **navegável** (o livre erodido por
`robot_radius` 0,383, que é o que o costmap enxerga) e compara com a linha reta:

| meta (map) | reta | geodésica | razão | 1ª parede na reta | rota visível do spawn |
| --- | ---: | ---: | ---: | ---: | ---: |
| (0,00; 8,00) — 0° | 8,00 m | **12,23 m** | 1,53× | **3,88 m** | 30,1% |
| (−8,00; 0,00) — 90° | 8,00 m | **33,76 m** | 4,22× | **3,90 m** | 10,6% |
| (−1,60; 1,60) — 45° | 2,26 m | **5,54 m** | 2,45× | **0,96 m** | 63,5% |
| (−5,83; 4,91) | 7,62 m | **24,02 m** | 3,15× | 1,05 m | 15,3% |

**As quatro têm parede na reta.** E do spawn o robô enxerga, com oclusão,
**19,6%** do espaço livre que está dentro dos 8 m de `obstacle_max_range`. Ele
está 80% cego no instante em que a meta chega.

`MAZE11_GOALS` não está errada — `maze_fit.py` a gerou para ser **patrulha**
(centros de corredor espalhados dentro de um raio), e patrulha num labirinto é
exatamente isto. O que estava errado era ler "meta a 8 m, rumo zero, caminho
reto" como se descrevesse uma pista livre. Não descreve nenhuma.

### O que isso faz com o Nav2, e por que nada acusa

Três parâmetros já documentados, cada um defensável sozinho, se somam:

- `global_costmap`: `rolling_window: true`, **sem `static_layer`**, sem mapa,
  sem SLAM, sem AMCL;
- `planner_server/GridBased`: **`allow_unknown: true`**;
- `local_costmap`: `track_unknown_space` **não declarado**, ou seja `false` —
  célula nunca observada vale **custo 0**, e não desconhecida.

Resultado: o NavFn traça a reta por cima da parede que ainda não foi observada,
`compute_path_to_pose` devolve `SUCCEEDED`, e `/plan` aparece bonito no RViz e
no cockpit. **Não há erro, aviso ou log.** É o mesmo modo de falha do reset que
apagava o robô: verde em toda a tela, apontando para uma coisa que não existe.

Corroboração em dado que já estava no repositório: a §8 mediu o caminho do NavFn
para (0,8) em **260 pontos × 0,0269 m ≈ 7,0 m**. A geodésica real é **12,23 m**,
e a própria linha reta é 8,00 m. Um caminho de 7 m entre dois pontos a 8 m de
distância não conecta os dois pontos por rota nenhuma — aquele plano já era um
plano por dentro de parede, e foi lido na época como "quase-reto".

### A ordenação que a §9 procurou por *bearing* e não achou

A §9 concluiu "nenhum gradiente por bearing" e encerrou a linha. A variável que
ordena não é o rumo, é **a distância até a primeira parede na reta**:

| meta | 1ª parede | deslocamento líquido medido (§9) |
| --- | ---: | ---: |
| (−1,60; 1,60) | 0,96 m | **0,00 m** |
| (0,00; 8,00) | 3,88 m | 0,07 m |
| (−8,00; 0,00) | 3,90 m | 0,10 m |

Ordenação perfeita, com n = 3 — sugestivo, não conclusivo. Mas repare no corte:
o horizonte do MPPI é 96 × 0,1 × 0,15 = **1,44 m**. A parede da meta de 45° cai
**dentro** do horizonte e o robô não sai do lugar; as duas de ~3,9 m caem
**fora** e ele arranha alguns centímetros antes de travar. O smoke pós-correção
de BT de 26/08, que fez 0,68 m de caminho na meta de 0°, cabe no mesmo padrão —
é o trecho de corredor livre antes da parede, não uma corrida melhor.

### Hipótese de mecanismo para o giro unidirecional da §10

Marcada como **hipótese**; a geometria acima é medição, isto não é.

Dentro do corredor de 1,20 m com `inflation_radius` 0,55, a faixa de custo zero
tem 0,10 m de largura — todo o resto do corredor **conhecido** custa caro. Fora
do corredor, a célula nunca observada custa **zero** no `local_costmap`. Ou
seja: o campo de custo torna o desconhecido atrás da parede mais barato que o
corredor real. O `PathAlignCritic` (peso 14) segura o robô contra um caminho que
aponta para lá, o `CostCritic` proíbe o que ele de fato vê, `vx_min: 0.0`
fechou a ré — e o único grau de liberdade que sobra é a guinada. Como a
geometria que produz o gradiente é **estática**, o sinal do giro nunca inverte.
Isso é exatamente a assinatura da §10: `cmd_wz` positivo em 65% das amostras,
**nunca negativo**, `cmd_vx` ≈ 0.

### O caminho do diff-drive já faz o certo; só o quadrúpede navega às cegas

| | `nav2_params.yaml` (TB4) | `nav2_params_go2.yaml` (Go2) |
| --- | --- | --- |
| `global_costmap` plugins | `static_layer` + obstacle + perception + inflation | obstacle + perception + inflation |
| `rolling_window` global | `false` | `true` |
| mapa persistido | `demo_navigation/maps/warehouse.{pgm,yaml}` | **nenhum** |

O `static_layer` do Go2 **já está definido e inerte**, com o procedimento de
religar escrito ao lado dele. A infraestrutura de SLAM também já existe e já
pagou o preço das armadilhas silenciosas (`slam.launch.py` +
`slam_params.yaml`): transição de lifecycle encadeada, remap de `/scan` em vez
de parâmetro, params por arquivo. **A sugestão do operador de persistir o mapa
não é frente nova: é ligar o que já está na árvore.**

### Os dois bloqueios reais para o SLAM do quadrúpede, e o tamanho de cada um

1. `slam_params.yaml` tem `base_frame: base_link`, e o Go2 tem `base`. É um
   parâmetro, não um impedimento de arquitetura — a §9 tratou como se fosse.
2. **Este é o de verdade:** `slam_toolbox` consome `sensor_msgs/LaserScan`, e o
   `/demo/scan` do Go2 é o anel degenerado de 1 dos 16 do L1 — o mesmo que o
   delta 3 do `nav2_params_go2.yaml` já mediu como **"46 de 640 feixes válidos,
   ZERO obstáculos, numericamente idêntico ao mundo vazio"**. Alimentar o SLAM
   com ele produz mapa vazio, sem erro. O dado bom está em `/demo/scan_cloud`
   (PointCloud2, 16 anéis), e achatá-lo é `ros-jazzy-pointcloud-to-laserscan`,
   pacote de estoque disponível no apt do Jazzy (2.0.2), não instalado ainda em
   nenhuma imagem. Mesmo formato de solução do delta 3: usar a nuvem.

### Passo 1 — EXECUTADO no mesmo dia, no HIL real, e a §11 está PROVADA

Evidência completa: **`docs/results/ml35-f5-rota-conectada.md`** (CSVs ao lado).
A/B com minutos de intervalo, mesma bancada, mesmas imagens, mesmos parâmetros,
Nav2 no Aquila AM69. **Única variável: a geometria da meta.**

| métrica | rota conectada | controle — meta de patrulha (0; 8) |
| --- | ---: | ---: |
| **razão de trabalho `vx`** | **37,5%** | **0,0%** |
| `cmd_vx` ≈ 0 | 12,9% | 98,6% |
| deslocamento líquido | **7,11 m** | 0,19 m |
| eficiência de trajeto | 57,2% | 16,2% |
| **metas cumpridas** | **8 de 8** | 0, nunca encerrou em 120 s |
| `cmd_wz` > 0 / < 0 | 45,2% / 51,3% | 7,8% / 64,8% |
| **deriva líquida de yaw** | **+1,1°** em 240 s | **−186,8°** em 120 s |

A razão de trabalho nunca passou de 8,6% em condição alguma já testada neste
projeto. Foi a **37,5%** sem tocar em nada além da meta.

**O giro unidirecional da §10 está explicado e não é defeito de controlador.**
Com plano válido o `cmd_wz` alterna quase igualmente e a deriva líquida é
+1,1° em quatro minutos — o MPPI corrige. Com meta atrás de parede o giro volta
a ser unidirecional. **E o sinal inverteu** em relação à §10 (lá sempre
positivo, aqui predominantemente negativo, na mesma meta): isso mata a família
"assimetria de critic" / "erro de sinal na guinada" — erro de sinal não troca
de sinal.

O passo 1 fica registrado como feito. Quem retomar começa no passo 2.

## 12. Próximos passos — estado em 27/08/2026, fim da sessão

Esta seção substitui todas as listas de "próximo passo" anteriores. Ela é o
ponto de partida de quem abrir a próxima sessão.

### O que está FECHADO e não deve ser reaberto

| item | como morreu |
| --- | --- |
| as cinco hipóteses da §1 (CPU, `/clock`, amostragem MPPI, `inflation_radius`, rede) | medição, §1 |
| `offset_from_furthest: 20` mal dimensionado | recalculado com resolução medida, §8 |
| mínimo local por bearing | sem gradiente por rumo, §9 |
| `collision_monitor` / `velocity_smoother` / conversor SI | medido em `/cmd_vel_nav`, §9 |
| meta com orientação de chegada incompatível | leitura de `nav_trial.py:_send`, §10 |
| `GoalAngleCritic` com tolerância assimétrica | default de estoque, §10 |
| **"assimetria de critic" / erro de sinal na guinada** | **o sinal do giro INVERTEU entre corridas** — erro de sinal não troca de sinal (parte 2) |
| **teste de desligar critic um a um** | **cancelado**: mediria a reação do MPPI a uma entrada que se sabe inválida |
| **`clearing: false` no `obstacle_layer` global** | **medido e reprovado** — é o raytrace, e o raytrace é o que cria espaço LIVRE. Ver `docs/results/ml35-f5-memoria-costmap.md`. Travado por teste. |

### O que está PROVADO

A navegação nunca esteve quebrada. **O plano global alterna entre duas rotas
incompatíveis a 1 Hz** — 11,5 m (a verdadeira) e 8,6 m (atravessando parede não
observada, barata porque `allow_unknown: true`) — e o MPPI recebe um caminho que
inverte 90–180° a cada segundo. Dê a ele uma meta sem parede na reta e o mesmo
robô, no mesmo módulo, com os mesmos parâmetros, cumpre **8 metas de 8** com
razão de trabalho de **37,5%**.

Evidência: `docs/results/ml35-f5-rota-conectada.md` e
`docs/results/ml35-f5-memoria-costmap.md`.

### Ordem para a próxima sessão

**1. Medir a anomalia dos 0,55 m. Antes de qualquer outra coisa.**

O costmap global marca a primeira célula ≥ 253 (faixa inscrita) a **0,55 m em
+y**, enquanto `maze_fit.py` mede **3,47 m de pista livre** nessa direção a
partir do spawn. As duas leituras não se conciliam, e a folga de nascimento é
justamente 0,55 m.

Isto é pré-requisito do item 2, não paralelo a ele: um mapa persistente
**herdaria o erro em definitivo**. Hoje o raytrace apaga a marca no ciclo
seguinte; com SLAM ela vira parede permanente e o robô fica cercado por uma
parede que não existe.

Candidatos não testados, em ordem de suspeita:
- inflação de 0,85 m do costmap global vinda das paredes laterais do corredor de
  1,20 m — o mais provável, mas explica custo alto, **não** um valor ≥ 253, que
  é a faixa inscrita e vem de `robot_radius`;
- marca da própria perna em trote — `selfhit.py` foi rodado com o robô **parado**;
- nuvem marcada em frame errado.

Ferramentas prontas: `scripts/costmap_probe.py` (costmap local),
`scripts/selfhit.py`, e a sonda de custo global usada nesta sessão está descrita
em `ml35-f5-memoria-costmap.md` §2.

**2. Persistir o mapa com `slam_toolbox` — o pedido do operador, na única camada
que o sustenta.**

Precisa persistir **ocupado _e_ livre**; a camada de obstáculo tem um botão só
para os dois, e é por isso que o item reprovado acima não funcionou.

Receita, com o que já existe e o que falta:

| passo | estado |
| --- | --- |
| `slam.launch.py` (lifecycle encadeado, remap de `/scan`) | **já existe e funciona** |
| `slam_params.yaml` | existe, mas com `base_frame: base_link`; o Go2 tem `base` |
| `sensor_msgs/LaserScan` utilizável | **falta** — o `/demo/scan` do Go2 é o anel degenerado com **zero obstáculos** (delta 3 do `nav2_params_go2.yaml`) |
| `ros-jazzy-pointcloud-to-laserscan` | estoque, 2.0.2 no apt do Jazzy, **em imagem nenhuma** |
| `static_layer` no global do Go2 | **já definida e inerte**, com o procedimento de religar ao lado |
| AMCL | **não é necessário** nesta topologia: odom do Gazebo é verdade de terreno e `map`→`odom` já é a identidade do `odom_tf`. Em hardware real ele volta. |

Manter `allow_unknown: true`: no instante 0 o mapa está vazio e o plano vai reto
pelo desconhecido, o que é **correto** — é o que permite aceitar meta a 8 m sem
mapa. O que muda é que a parede, uma vez vista, **nunca mais sai**, o atalho
deixa de existir e o plano converge.

**Custo honesto:** o `pointcloud_to_laserscan` exige **rebuild arm64 nativo no
módulo**. É a parte cara desta frente. O bind mount de config entregue nesta
sessão **não** ajuda aqui — ele cobre YAML, não pacote apt.

**3. Reescrever o portão do F5.** "Goal Nav2 `SUCCEEDED` com o robô de pernas,
Nav2 no módulo" foi cumprido **oito vezes numa corrida**. O que continuava
reprovando era o protocolo de 8 m sobre metas de patrulha, que pede ao
planejador uma coisa que a geometria do maze11 não oferece
(`scripts/maze_geodesic.py`: 4 de 4 metas com parede na reta). O portão tem de
ser reescrito sobre rota conectada ou sobre mapa persistido antes de voltar a
ser cobrado.

**4. Só então sintonia de critic.** `PathAlignCritic` 14 × 8 (§4), com entrada
válida e sob o protocolo intercalado da §2. Resta déficit real: 37,5% de razão
de trabalho e 0,0454 m/s médio ainda estão abaixo de `vx_max` 0,15 m/s. A
condição `params-align8.yaml` já foi regerada a partir do baseline atual.

### Pendências fora desta frente, inalteradas

- **Cockpit F4** (controle manual atrás do `twist_mux`): bloqueado por
  infraestrutura — `twist_mux` não está em nenhuma imagem e o módulo não tem
  rota default, então `apt` não resolve nada lá. Falta
  `sudo ip route add default via 192.0.2.2 dev ethernet0 metric 100`, com
  persistência na configuração de rede do Torizon.
- **Gate visual do cockpit:** nada foi visto num navegador em nenhuma sessão
  desta série. Não há Chrome nesta máquina e o MCP de automação não dirige o
  Firefox instalado.
- **Trabalho não commitado de sessões anteriores** ainda na árvore:
  `nav_campaign.py`, `nav_select.launch.py`, `nav_to_pose_smoothed.xml`,
  `compose.host.yml`, `test_nav_campaign.py`, `test_nav_params_override.py`,
  `test_nav_to_pose_smoothed_bt.py`, `params-align8.yaml`,
  `docs/guia-hil-go2-labirinto.md`.

### Como retomar a bancada

```bash
# host: simulador + cockpit
cd docker && docker compose -f compose.host.yml up -d sim cockpit hmi

# módulo (192.0.2.5): nav + percepção
./scripts/module.sh sync           # YAML de parâmetros chega por bind mount
ssh torizon@192.0.2.5 'cd /home/torizon/demo && \
  docker compose -f compose.module.yml up -d'
./scripts/module.sh verify         # tem de dar 3/3

# ensaio (host), SEMPRE com --goals=
. /opt/ros/jazzy/setup.bash && . ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/docker/cyclonedds/host.rendered.xml
ros2 service call /demo/sim/reset std_srvs/srv/Trigger      # reposiciona o robô
python3 scripts/nav_trial.py saida.csv --seconds 240 --goals="..."
```

**Uma armadilha de ferramenta, encontrada rodando o passo 1.** Toda meta do
maze11 tem `x` negativo, e `--goals -1.50,...` é lido pelo argparse como uma
flag: o script imprime `usage` e sai **0**. Num pipe com `2>/dev/null` isso vira
uma corrida silenciosa que não faz nada. **Use sempre `--goals=`**, com o sinal
de igual.

**O que esta seção NÃO afirma.** Não afirma que o MPPI está perfeitamente
sintonizado — resta o déficit do item 5. Afirma que as §§7–10 mediram o
controlador com uma entrada inválida, e que nenhuma conclusão sobre critics
sobrevive a isso. As cinco hipóteses refutadas da §1 continuam refutadas —
nada aqui as reabre.
