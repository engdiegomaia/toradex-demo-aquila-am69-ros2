# R14 — primeiro A/B do lado do MPPI, resultado negativo/inconclusivo

Rodada HIL real (Aquila AM69 + host). Unica variavel: `PathAlignCritic.
offset_from_furthest` 20 -> 10 em `nav2_params_go2.yaml` (~1 m -> ~0.5 m de
referencia a frente). Nenhuma outra config mudou em relacao a R13 -- mesmo
`maze_explorer.py` de R13 (pre-R15; o vigia de movimento, a recuperacao por
varredura e a classificacao honesta ainda nao existiam nesta rodada).
Evidencia: `ml35-f5-exploration-r14.csv`, `ml35-f5-exploration-r14-goals.csv`.

## Precondicoes

1. `/demo/sim/reset` -> `success=True`, robo em `(0.00016, -0.00542)`.
2. Config sincronizada (`module.sh sync`) e `nav`/`perception` recriados
   (`module.sh up`) com o robo ja no spawn -- sem o refuso de
   `check_robot_near_spawn_before_nav_restart`.
3. Confirmado antes de iniciar: lifecycle ativo, `/demo/exploration/status`
   `state=idle, frontier_count=0`, `/map` 88x87 com origem
   `(-3.858, -0.477)` -- mapa novo, identico em forma ao de R13.
4. `/demo/exploration/start` chamado exatamente uma vez.

## O que a rodada deu

| metrica | R13 (offset 20) | R14 (offset 10) |
| --- | --- | --- |
| duracao ativa | 600.5 s | 500.4 s |
| motivo da parada | `total_timeout` | `barren_other` (10 ciclos baldios) |
| metas despachadas | 23 | 16 |
| metas ok / falhas | 16 / 7 (70%) | 9 / 7 (56%) |
| `tilt_deg` maximo | 1.32° | 1.22° -- **zero quedas nas duas** |
| folga minima de parede | 0.05 m | 0.10 m |
| janelas de imobilidade | 5 (10,2-24,2 s) | 4 (11,0-23,8 s) |

A rodada **nao chegou ao fim do orcamento de 600 s** -- parou aos 500 s por
esgotar `barren_selections_limit` (10 ciclos consecutivos sem candidato),
classificado `barren_other` (fronteiras brutas existiam, filtro/supressao as
comeu todas). Isto por si so ja e um resultado pior que o baseline de R13 em
completude de exploracao, ainda que sem nenhuma queda.

## O criterio que a mudanca deveria mover: nao moveu, ou piorou

Isolando o mesmo subconjunto "corredor reto genuino" que R13 definiu
(`plan_straightness > 0.97` E as duas paredes dentro de 1.5 m E
`state == navigating`):

| metrica no subconjunto reto | R13 (offset 20) | R14 (offset 10) |
| --- | --- | --- |
| amostras no subconjunto | 389 / 1320 | 131 / 1320 |
| `|cmd_wz|` mediana | 0.052 rad/s | **0.079 rad/s** |
| `|cmd_wz|` p90 | 0.122 rad/s | 0.123 rad/s |
| fracao com `|cmd_wz|` > 0.02 | 82,0% | 76,3% |
| assimetria parede-esq/dir, mediana `|E-D|` | 0.25 m | 0.25 m |
| direcao da assimetria | ~40% E / 36% D / 24% centro (oscila) | **63% E / 15% D / 21% centro (vies fixo)** |
| `plan_straightness` global, mediana | 0.96 | 0.93 |
| `plan_straightness` global, fracao < 0.9 | 12,4% | **39,1%** |
| amplitude pico-a-pico de yaw real por janela de ~5 s, mediana | 10,7° | **16,9°** |

Nenhum destes numeros aponta para menos oscilacao. A mediana de `|cmd_wz|`
subiu, a fracao de amostras "retas" no subconjunto caiu (fronteiras
diferentes exploradas, caminho mais curto), `plan_straightness` piorou, e a
amplitude de guinada real por janela piorou. O unico numero estavel foi o
p90 de `|cmd_wz|` e a mediana de assimetria de parede em valor absoluto --
mas a **direcao** da assimetria mudou de "oscila entre os dois lados"
(R13, a assinatura que motivou este A/B) para "vies fixo para a esquerda"
(R14). Isso nao e o padrao que a hipotese de R13 previa se o encurtamento da
referencia tivesse funcionado.

## Por que isto nao e uma conclusao limpa

- **n=1 por lado**, a mesma limitacao que R13 ja declarava. As duas rodadas
  exploraram geometrias DIFERENTES do labirinto (R14 parou mais cedo, com 16
  metas contra 23, e o subconjunto reto tem 131 amostras contra 389) --
  parte da diferenca pode vir do trecho de labirinto percorrido, nao so do
  parametro mudado.
- A parada por `barren_other` aos 500 s tambem pode ser efeito de qual regiao
  o robo alcancou antes de ficar sem candidato, nao necessariamente uma
  consequencia direta do `offset_from_furthest` menor -- mas nao ha como
  descartar essa hipotese com uma unica rodada de cada lado.
- Criterio de aceitacao "razao deslocamento-liquido/caminho > 90%" **nao foi
  medido**: a instrumentacao atual (`exploration_trial.py`) nao registra
  comprimento de caminho por meta, so `elapsed_s`; calcular isso exigiria uma
  nova coluna, nao um numero que ja exista para reaproveitar sem risco de
  inventar a definicao.
- Colisao lateral: nenhuma amostra abaixo de 0.05 m nesta rodada (minimo
  0.10 m) -- **melhor** que R13 (minimo 0.05 m), mas de novo confundido com
  geometria diferente percorrida.

## Leitura contra os criterios de aceitacao de R14

| criterio | resultado | veredito |
| --- | --- | --- |
| zero quedas | tilt max 1.22° | **passa** |
| sem colisao lateral | folga minima 0.10 m | **passa** (mas ver limitacao acima) |
| desvio maximo do centro < 15 cm | nao medido diretamente (sem coluna de centro-de-corredor) | **nao avaliado** |
| diferenca de folga E/D em retas < 10 cm | mediana 0.25 m nas duas rodadas | **reprova**, sem mudanca |
| oscilacao de yaw < 5° | mediana 10,7-16,9° nas duas rodadas | **reprova**, sem mudanca (R14 pior na mediana) |
| razao liquido/caminho > 90% | nao medido | **nao avaliado** |
| sem regressao de conclusao de metas | 70% -> 56% | **reprova** |

Dos dois criterios efetivamente mensuraveis com a instrumentacao atual
(clearance E/D e oscilacao de yaw), nenhum melhorou; um piorou na mediana. A
taxa de metas concluidas caiu. Isto e um resultado **negativo ou, no minimo,
nao demonstrado** para este A/B especifico -- nao uma confirmacao da
hipotese de R13.

## Recomendacao

`offset_from_furthest` volta a 20 (revertido nesta mesma rodada de commits)
para nao empilhar uma mudanca nao demonstrada sob a proxima tentativa -- a
mesma disciplina de uma variavel por vez que motivou testar isto primeiro
exige tambem desfazer o que nao funcionou antes de tentar a proxima. As
opcoes que o plano ja previa para o proximo A/B do lado do MPPI continuam
abertas: peso do `PathAlignCritic` (14, o maior), peso do `PathAngleCritic`
(2.0), ou frequencia de substituicao do plano ativo -- nenhuma tentada
ainda. Dado que este primeiro A/B nao fechou nada, a escolha entre elas (ou
a alternativa de retomar o A/B de `k_yaw` do plano original) fica para
decisao explicita antes de gastar mais tempo de bancada real.

## Limitacoes

HIL real, nenhuma conclusao aqui vale para performance/CPU/termico sem
validacao propria (`CLAUDE.md` regra 5). n=1 por configuracao, confundido
com geometria de exploracao diferente entre as duas rodadas -- ver secao
acima. `plan_straightness`, `cmd_wz` e amplitude de yaw sao medidas ja
usadas em R13; a amplitude pico-a-pico por janela de 5 s e nova nesta rodada
e ainda nao teve seu limiar de alarme (5°) validado como o numero certo a
perseguir, so citado por ser o da acceptance criteria original.
