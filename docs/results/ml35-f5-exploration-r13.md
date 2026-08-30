# R13 — diagnostico de movimento (zigue-zague) e de parada

Rodada diagnostica, HIL real (Aquila AM69 + host), **nao conta para aceitacao
da F5**. Configuracao identica a R12 (`frontier_score`, `clearance_m`,
`goal_timeout_s`, `total_timeout_s`, `frontier_endpoint_setback_m`) — nenhuma
mudanca de comportamento nesta rodada, so instrumentacao nova em
`scripts/exploration_trial.py`. Evidencia: `ml35-f5-exploration-r13.csv`,
`ml35-f5-exploration-r13-goals.csv`, este relatorio.

## Precondicoes (confirmadas, nao inferidas)

1. `/demo/sim/reset` chamado uma vez -> resposta `success=True`, robo em
   `x=0.0000196 y=-0.00827` (spawn).
2. `scripts/module.sh up` recriou `nav`/`perception` com o robo ja no spawn
   (sem o refuso de `check_robot_near_spawn_before_nav_restart`).
3. Apos estabilizar: `/demo/odom` em `(0.00019, -0.00578)`, `/demo/exploration/status`
   `state=idle, elapsed_s=0.0, frontier_count=0`, `/map` 88x87 celulas com
   origem `(-3.858, -0.481)` — mapa novo, centrado no spawn.
4. `/demo/exploration/start` chamado **exatamente uma vez**. Nenhuma meta
   manual, nenhum teleop.

Um bug real foi encontrado e corrigido **antes** da rodada valida: a
subscricao de `/plan` usava o mesmo perfil QoS `TRANSIENT_LOCAL` de `/map`,
mas `nav2_planner` publica `/plan` como `RELIABLE`/`VOLATILE` — a
incompatibilidade de durabilidade fazia a rodada nunca receber nenhuma
mensagem em `/plan` (confirmado ao vivo pelo warning `incompatible QoS ...
DURABILITY`, com o resto do gravador saudavel). Corrigido para o mesmo perfil
de profundidade simples usado em `/demo/odom`/`/demo/cmd_vel_si`, e a rodada
foi reiniciada do zero (o robo nunca tinha saido do spawn na tentativa
descartada, entao o reset nao precisou ser refeito).

## Metricas da rodada

| metrica | valor |
| --- | --- |
| amostras / duracao sim / duracao parede | 1320 / 642.2 s / 659.5 s |
| `escaped` | false (esperado — R13 nao busca a saida) |
| estado final / mensagem | `failed` / "prazo total de exploracao excedido" |
| `path_m` | 34.55 m |
| `map_known_cells` inicio -> fim | 1699 -> 19021 |
| metas despachadas | 23 (16 ok / 7 falhas, todas por timeout de meta) |
| `refused=1 timed_out=1 blacklisted=0 provisional_recoveries=1 barren_cycles=0` | |
| `tilt_deg` maximo | 1.32° — **zero quedas** |
| `active_vx_work_ratio` | 75.3% |

`stop_reason` calculado automaticamente: **`total_timeout`** — a exploracao
nao ficou sem fronteiras (`barren_cycles_final=0`); ela usou o orcamento
inteiro de 600 s ainda encontrando fronteiras novas. Mesmo padrao saudavel de
R12.

## 1. O zigue-zague: separando as tres possibilidades

### Possibilidade 1 — o plano global ja nasce em zigue-zague

`plan_straightness` (distancia reta ponta-a-ponta / comprimento real do
plano; 1.0 = reto): mediana **0.96**, p10 **0.90**, so 12.4% das amostras
abaixo de 0.9 e apenas 0.7% abaixo de 0.7.

**Fraca.** O plano global nao nasce torto na maioria do tempo; os poucos
trechos com `straightness` baixo sao esperados perto de curvas/cruzamentos
(comprimento de plano tipico ali e maior — mediana de `plan_length_m` 0.65 m,
maximo 6.98 m nos trechos mais longos).

### Possibilidade 2 — plano centralizado, MPPI oscila ao segui-lo

Isolando apenas amostras "corredor reto genuino" (`plan_straightness > 0.97`
E ambas as paredes dentro do alcance de sondagem de 1.5 m E `state ==
navigating`, 389 de 1320 amostras):

- `|cmd_wz|` mediana **0.052 rad/s**, p90 **0.122 rad/s** — **82.0%** das
  amostras desse subconjunto "reto" tem comando de guinada acima de
  0.02 rad/s. O MPPI esta ativamente comandando correcoes de rumo mesmo
  quando o proprio plano que ele segue e essencialmente reto.
- A assimetria parede-esquerda vs parede-direita nesse mesmo subconjunto:
  mediana de `|esquerda - direita|` = 0.25 m, mas a **direcao** da assimetria
  se divide quase 50/50 (40.1% mais folga a esquerda, 36.0% mais folga a
  direita, 23.9% centrado em +-10 cm) — nao e um viés fixo para um lado, e
  sim uma oscilacao entre os dois lados.

**Forte.** O padrao de comando de guinada nao-nulo sobre um plano reto,
combinado com a assimetria de parede oscilando de lado em vez de fixa, e a
assinatura que a possibilidade 2 preve.

### Possibilidade 3 — plano e comando retos, a marcha desvia fisicamente

Comparacao amostra-a-amostra da taxa de guinada REAL do corpo (derivada de
`yaw_deg`/`sim_s` entre amostras consecutivas) contra o `cmd_wz` comandado na
amostra anterior, restrita a `state == navigating` e `|cmd_wz| > 0.03 rad/s`
(756 amostras):

- razao `taxa_real / cmd_wz`: p10 **0.35**, mediana **0.80**, p90 **1.15**.
- mesma direcao (a marcha gira para o lado pedido): **97.2%**.
- razao > 1.15 (sobre-executa o comando em mais de 15%): **9.8%** das
  amostras.

**Nao confirmada por esta medicao.** A hipotese registrada no plano de R14
("0.35 executa ~137% do que o MPPI acredita comandar") previa uma
sobre-execucao sistematica; esta medicao de campo mostra o oposto na
mediana (sub-execucao, razao 0.80), com sobre-execucao > 15% em menos de 10%
das amostras. **Isto NAO invalida a hipotese do plano** — a medicao aqui e
grosseira (amostragem a 2 Hz, sem compensacao de atraso entre o instante do
comando e sua realizacao no corpo, sem filtro) e serve como evidencia
exploratoria, nao como substituto do A/B controlado que R14 ja prescreve.
Mas ela tambem nao a corrobora, e isso muda a prioridade recomendada abaixo.

**Leitura conjunta:** a evidencia desta rodada aponta a possibilidade 2
(MPPI oscilando) como a explicacao mais bem sustentada, com a possibilidade 3
nao corroborada (mas tambem nao definitivamente descartada) por esta mesma
rodada. A possibilidade 1 e fraca.

**Recomendacao para R14:** dado que esta rodada nao corrobora a
sobre-execucao de 137% do `foot_placement.k_yaw`, o primeiro A/B de R14
poderia inverter a ordem sugerida no plano e comecar pelo lado do MPPI
(peso do `PathAlignCritic`, `offset_from_furthest`, peso do
`PathAngleCritic`, ou frequencia de substituicao do plano ativo) antes do
A/B de `k_yaw`. Isto e uma recomendacao baseada em evidencia de campo, nao
uma imposicao — o A/B de `k_yaw` continua sendo o mais barato e menos
arriscado de executar primeiro se a preferencia operacional for comecar pelo
que ja tem numeros medidos em outra sessao.

## 2. A parada: classificacao pelas sete categorias

`stop_reason` (automatico) = **`total_timeout`** para o final da rodada —
a exploracao nao morreu por falta de fronteiras, morreu por orcamento de
tempo enquanto ainda avancava.

Dentro da rodada, os sete tipos de parada do protocolo:

| categoria | ocorreu? | evidencia |
| --- | --- | --- |
| `navigating` com comando mas sem movimento | **sim, 5 janelas** | ver tabela abaixo |
| `selecting` sem candidato | quase nao | `selecting_no_candidate_s = 1.425 s` no total de 642 s |
| fronteiras existentes mas filtradas | **sim, toda vez** | `frontier_clusters_raw` sempre > `frontier_clusters` (ex.: 16 brutos -> 12 filtrados, 12 -> 8, 6 -> 5) |
| nenhuma fronteira bruta | nao | `frontier_clusters_raw` nunca chegou a 0 |
| timeout de meta | **sim, 7 de 23 metas** | ver `-goals.csv`, mensagem `meta de fronteira expirou` |
| timeout total de 600 s | **sim — foi o motivo final** | `stop_reason=total_timeout` |
| `failed` por ciclos esteris | nao | `barren_cycles_final = 0` |

### As 5 janelas de imobilidade com comando (watchdog de R15, validado com dado real)

`find_stalled_navigating_windows()` (novo, `min_stall_s=10.0`,
`move_threshold_m=0.05`) encontrou:

| janela (sim_s) | duracao | posicao | parede esq/dir | meta associada |
| --- | ---: | --- | --- | --- |
| 2602.5-2617.6 | 15.1 s | (-0.12, 1.58) -> (-0.05, 1.54) | 0.35/0.45 -> 0.95/1.50 | meta 2 (timeout) |
| 2718.6-2730.3 | 11.7 s | (-3.48, 0.47) -> (-3.53, 0.51) | 0.20/0.25 -> 0.20/0.75 | meta 6 (timeout) |
| 2774.1-2798.3 | 24.2 s | (-3.30, 1.66) -> (-3.33, 1.69) | 1.00/0.90 -> 1.10/0.75 | meta 8 (timeout) |
| 2900.4-2911.1 | 10.7 s | (-1.32, 2.60) -> (-1.32, 2.70) | 0.75/**0.05** (fixo) | meta 11 (timeout) |
| 2911.6-2921.8 | 10.2 s | (-1.31, 2.68) -> (-1.32, 2.69) | 0.75/**0.05** -> 0.80/1.50 | meta 11 (timeout) |

**4 das 7 metas com timeout (2, 6, 8, 11) tem uma janela de imobilidade
comandada dentro delas.** As duas ultimas linhas sao a mesma meta 11: quase
20 s combinados com a parede direita a apenas 5 cm — o robo ficou preso
contra uma parede, nao apenas parado por uma decisao de replanejamento. As
metas 9 e 21, apesar de tambem terminarem em timeout, nao produziram uma
janela detectada — ou tiveram progresso real intermitente, ou o padrao de
imobilidade nao satisfez os limiares desta deteccao; nao investigado mais a
fundo aqui.

Isto e evidencia direta e medida de que o watchdog de movimento proposto em
R15 (cancelar a meta, registrar falha provisoria e tentar outro ponto/outra
fronteira **antes** de esgotar os 45 s completos) teria algo real para
capturar: mais da metade dos timeouts desta rodada (4 de 7) tinham o robo
efetivamente parado, nao apenas progredindo devagar.

### Fronteiras brutas vs filtradas

`frontier_clusters_raw` excedeu `frontier_clusters` em toda amostra de
`selecting` com dado (56 amostras): razoes tipicas de ~25-40% de perda por
filtro (ex. 16->12, 12->8, 9->4, 6->5). O filtro de clearance/standoff esta
ativo e removendo uma fracao real dos clusters brutos a cada ciclo, mas nunca
zerou o total disponivel nesta rodada — nao foi a causa de nenhuma parada
aqui (nao houve `barren_*`). Este numero fica registrado como linha de base
para a proxima rodada que terminar em `nenhuma fronteira segura alcancavel`,
onde a nova classificacao `barren_no_raw_frontiers` vs
`barren_frontiers_filtered` sera o dado que falta.

## O que esta rodada fecha e o que nao fecha

- **Fecha**: uma leitura instrumentada e automatica do zigue-zague (as tres
  possibilidades, separadas por dado, nao por suposicao) e da parada (sete
  categorias, com contagem real por categoria).
- **Fecha**: confirmacao de campo de que o watchdog de movimento de R15 teria
  trabalho real a fazer (4 janelas de imobilidade comandada, nao hipoteticas).
- **Nao fecha**: qual variavel exata de R14 corrigir primeiro — a evidencia
  desta rodada pesa para o lado do MPPI (possibilidade 2), mas nao e um A/B
  controlado e nao substitui o que R14 ja prescreve.
- **Nao fecha**: a causa das metas 9/21 (timeout sem janela de imobilidade
  detectada) — aberto, nao investigado nesta rodada.
- **Nao muda**: `_map_seq` continua contando mensagens, nao conteudo; esta
  rodada nao precisou dessa correcao porque nunca dependeu de reextrair o
  mapa sobre uma mensagem republicada identica.

## Limitacoes

HIL real (Aquila AM69 + host), nenhuma conclusao aqui vale para performance,
CPU, termico ou timing sem validacao explicita nesse sentido (`CLAUDE.md`
regra 5). A razao taxa-real/comando da secao 3 e uma medida exploratoria de
2 Hz sem compensacao de atraso — util para orientar prioridade, nao para
decidir sozinha. Uma unica rodada; nenhuma das leituras acima foi repetida
para descartar variancia de execucao para execucao.
