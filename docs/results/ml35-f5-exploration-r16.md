# R16 — validação integrada (R15a + `cost_weight=14.0`/`offset_from_furthest=20`)

Rodada HIL real (Aquila AM69 + host), executada pela arquitetura oficial
Docker + cockpit web (`docs/guia-hil-go2-labirinto.md`), nao por chamada
direta de servico como as rodadas anteriores: exploracao iniciada clicando
"iniciar busca" no cockpit em `http://localhost:8081`. Configuracao: codigo
de R15a (commit `26aba8f`, HEAD no momento da rodada) + `PathAlignCritic`
no baseline de R13 (`cost_weight=14.0`, `offset_from_furthest=20`) --
nenhuma outra mudanca de MPPI. Evidencia:
`ml35-f5-exploration-r16.csv`, `ml35-f5-exploration-r16-goals.csv`.

## Precondicoes (fluxo oficial completo)

1. `git status` limpo, HEAD em `26aba8f` (R15a).
2. `./scripts/module.sh sync` (arvore ja sincronizada; sem mudanca desde
   R15a, mas rodado por disciplina de processo apos o erro de R14b).
3. `/demo/sim/reset` -> `success=True`, robo em `(0.000, 0.000)`.
4. `./scripts/module.sh up` recriou `nav`/`perception` com o robo ja no
   spawn (guarda de "robo perto do spawn antes de reiniciar nav").
5. `./scripts/module.sh verify` -- 4/4 estagios OK (UDP, modulo ve host,
   host recebe do modulo, `bt_navigator: active`).
6. Confirmado antes de iniciar: `state=idle`, `frontier_count=0`, mapa
   fresco 88x85.
7. Exploracao iniciada **uma unica vez**, pelo botao "iniciar busca" do
   cockpit (nao por `ros2 service call` direto).
8. Um vigia de seguranca em processo separado, fora do executivo,
   monitorou `/demo/exploration/status` a cada ~5 s durante toda a rodada:
   se o estado chegasse a `homing_exit`, cancelaria a busca na hora --
   o risco de queda no homing cego (`docs/results/
   ml35-f5-homing-fall-analise.md`) continua sem mitigacao proprio, entao
   esta rodada nao deveria cruzar para o homing. O marcador nunca foi
   visto (`marker_observations=0`), `homing_entries=0` -- o vigia nunca
   precisou agir.

## Resultado bruto

| campo | valor |
| --- | --- |
| `stop_reason` | `barren_other` (`nenhuma fronteira segura alcancavel`) |
| duracao ativa | 431,4 s (contra ate 600 s de `total_timeout`) |
| metas ok / total | **9/15 (60%)** |
| recusadas / expiradas (finais) | 7 / 5 |
| ciclos esteris (`barren_cycles_final`) | 10 (bateu o limite `barren_selections_limit`) |
| recuperacoes provisorias / por varredura | 0 / 0 |
| marcador visto / homing iniciado | 0 / 0 |
| tilt maximo | 2,37° (o maior de todas as rodadas, ainda sem queda) |
| `z_min_m` | 0,304 (altura de apoio normal, sem colapso) |
| janelas de imobilidade (offline, `min_stall_s=10.0`) | 3 (14,6 / 12,1 / 14,5 s) |
| disparos do vigia de movimento (live, `stall_window_s=15.0`) | **2** |

## Checklist pedido pela revisao de codigo

**Nenhum giro legitimo cancelado pelo vigia.** Confirmado nos dois
disparos (abaixo) -- em ambos a guinada fica CONGELADA (23,4° e 41,13°,
sem variar um decimo de grau) durante toda a janela, nao girando. Esta
rodada nao produziu nenhum caso real de rotacao-em-pe-sem-translacao
para testar o lado positivo da correcao (nao cancelar uma rotacao
legitima) -- so o lado negativo (nao deixar passar uma imobilidade real)
foi exercitado de fato.

**Cada acionamento do vigia corresponde a imobilidade real.** Os dois
disparos, inspecionados na telemetria bruta:

| meta | inicio (sim_s) | pose (x,y) | guinada | `cmd_wz` tipico | `cmd_vx` tipico |
| --- | --- | --- | --- | --- | --- |
| 11 | 20454.16 | congelada | 23,40° (fixa) | -0.02 a -0.03 rad/s | ~0.0000-0.0019 |
| 13 | 20518.44 | congelada | 41,13° (fixa) | 0.02 a 0.03 rad/s | ~0.0000-0.0010 |

Em ambos os casos o MPPI continuou emitindo um `cmd_wz` pequeno e nao-nulo
sem nenhum efeito real no robo -- exatamente o cenario que motivou nao
verificar o comando de velocidade (achado 1 da revisao): um comando
"ativo" nao significa progresso real, e o vigia (que olha pose, nao
comando) classificou os dois corretamente como parado. As duas metas
cortaram aos ~15 s, bem antes do teto de 45 s.

**A cobertura parcial prevista no comentario corrigido de
`nav2_params_go2.yaml` se confirma, com uma ressalva de metodologia.**
CORRECAO (revisao pos-R16): a versao original deste paragrafo dizia que as
janelas offline de 14,6 e 14,5 s ficavam "acima" do limiar `stall_window_s
= 15,0` -- errado, 14,6 e 14,5 sao MENORES que 15,0. O vigia disparou
mesmo assim porque as duas medidas nao sao a mesma grandeza: a janela
offline (`find_stalled_navigating_windows`, amostras a ~0,4-0,5 s de
espacamento) mede a distancia entre a PRIMEIRA e a ULTIMA amostra que o
proprio criterio do script classifica como parada, enquanto o relogio do
vigia ao vivo conta tempo continuo (`_now_s()`) desde a ultima pose com
progresso real, checado a cada tick -- pontos de partida e granularidade
diferentes. Uma defasagem da ordem de meio segundo entre as duas contagens
(o proprio espacamento entre amostras do script) basta para explicar por
que o vigia disparou (>= 15,0 s no seu relogio) enquanto o script mede uma
janela um pouco mais curta -- o disparo real provavelmente caiu ENTRE duas
amostras do script, nao dentro de uma janela que ele delimitou com
precisao de decimo de segundo. Isto e uma hipotese plausivel pela ordem de
grandeza, nao uma causa verificada amostra a amostra.
A terceira janela offline (12,1 s, durante a meta 12) fica abaixo do
limiar por uma margem bem maior que meio segundo, e essa meta de fato NAO
foi cortada pelo vigia -- expirou pelos 45 s normais, o comportamento
esperado.

**Recuperacao por Spin:** nao exercitada nesta rodada
(`provisional_recoveries_final=0`) -- os 10 ciclos esteris vieram todos
de fronteiras que existiam mas foram filtradas (`barren_cycles`), nunca
de zero clusters brutos, que e a unica condicao que dispara a varredura.
Item do checklist nao testado por falta de ocasiao, nao por falha.

**Exploracao parou antes do prazo total, mas nao "prematuramente" no
sentido de um bug:** apos 10 ciclos consecutivos de selecao esteril
(fronteiras existentes mas todas filtradas ou dentro da tolerancia de
chegada), `_note_barren_selection` bateu `barren_selections_limit` e
`_fail('nenhuma fronteira segura alcancavel')` disparou como projetado --
uma classificacao terminal honesta (o proprio objetivo de R15), nao um
travamento silencioso. Ainda assim, esta rodada fechou aos 431 s contra os
600 s de R13, o que junto com os 60% de metas concluidas (ver abaixo) sao
o lado fraco do resultado.

**Taxa de metas concluidas vs os 70% de R13:** 60% (9/15) -- abaixo do
baseline, acima de R14 (56%), R14b (47%) e R14c (52%). Segundo melhor
resultado de completude entre as cinco rodadas medidas ate agora.

**Chegada a regiao sudoeste:** nao alcancada de forma clara. A pose real
do robo (nao so as metas tentadas) ficou em `x` entre -3,52 e 0,15 e `y`
entre -0,03 e 5,38 -- as duas metas que mirariam mais a oeste/sudoeste
(metas 11 e 14, `x` = -4,978 e -4,128) **falharam** (vigia e timeout,
respectivamente) antes de o robo chegar la. Marcador da saida nunca visto
(`marker_observations=0`) -- ao contrario de R14b, que o avistou 24 vezes.

## Zigzag: comparativo (mesmo script, mesma metodologia das rodadas anteriores)

| metrica (subconjunto "corredor reto genuino") | R13 (baseline) | R14 (offset10) | R14c (w7 isolado) | R16 (R15a+baseline) |
| --- | --- | --- | --- | --- |
| amostras no subconjunto | 445 / 1320 | 131 / 1320 | 320 / 1320 | 177 / 1320 |
| `\|cmd_wz\|` mediana | 0.056 | 0.079 | 0.048 | **0.031** |
| `\|cmd_wz\|` p90 | 0.124 | 0.123 | 0.126 | **0.101** |
| fracao `\|cmd_wz\|` > 0.02 | 82,7% | 76,3% | 77,5% | **74,0%** |
| assimetria E/D, mediana `\|E-D\|` | 0.25 m | 0.25 m | 0.25 m | **0.50 m** |
| direcao da assimetria | 41% E/33% D/26% C | 63% E/15% D/21% C | 45% E/31% D/25% C | **60% E/31% D/9% C** |
| `plan_straightness` global, mediana | 0.96 | 0.933 | 0.942 | 0.932 |
| `plan_straightness`, fracao < 0.9 | 12,4% | 39,1% | 31,7% | **40,9%** |
| amplitude pico-a-pico de yaw/~5s, mediana | 10,7° | 16,9° | 10,2° | **8,8°** |
| folga minima de parede | 0.05 m | 0.10 m | 0.05 m | 0.05 m |
| metas ok / total | 16/23 (70%) | 9/16 (56%) | 11/21 (52%) | **9/15 (60%)** |
| motivo da parada | `total_timeout` | `barren_other` | `total_timeout` | `barren_other` |
| duracao ativa | 600,5 s | 500,4 s | 591,2 s | 431,4 s |
| janelas de imobilidade (offline) | 5 | 4 | 7 | 3 |
| disparos reais do vigia (live) | -- (nao existia) | -- (nao existia) | -- (codigo pre-R15 nesta rodada) | **2** |
| tilt maximo | -- | -- | 1,73° | **2,37°** |

## Leitura

Quadro misto, no mesmo padrao de R14c: `cmd_wz` mediano e amplitude de
guinada sao os **melhores de todas as cinco rodadas medidas** (0.031 rad/s,
8.8°), sugerindo que cortar cedo os trechos de imobilidade real (via o
vigia) reduz o tempo gasto oscilando contra uma parede em vez de progredir.
Mas a assimetria de parede piorou (0.5 m, vies de 60% para a esquerda --
direcao parecida com o vies de R14, nao com o de R14b) e
`plan_straightness` tambem, e a rodada fechou mais cedo (431 s,
`barren_other`) com uma taxa de metas abaixo do baseline (60% contra 70%).

Nao da para separar, com n=1 e geometria de labirinto diferente a cada
rodada, quanto disso e efeito real do software de R15a (por exemplo:
cortar metas cedo pode ter deixado o robo mal-posicionado para a proxima
selecao, empurrando-o para um canto com mais fronteiras filtraveis) e
quanto e apenas a variacao natural de qual regiao do labirinto cada
execucao explora -- a mesma ressalva metodologica ja registrada em
R14b/R14c. O que esta rodada estabelece com confianca, porque e uma
verificacao direta de codigo contra dado real, nao uma comparacao entre
rodadas: **o vigia de movimento de R15a funcionou como projetado** -- dois
disparos, os dois sobre imobilidade real confirmada na telemetria bruta,
nenhum falso positivo sobre rotacao (embora nenhuma rotacao legitima
tenha ocorrido para testar esse lado), e a cobertura parcial (2 de 3
janelas, a terceira abaixo do limiar de 15 s) e exatamente a que o
comentario corrigido em `nav2_params_go2.yaml` descreve.

## Recomendacao

Nao ha motivo para reverter nada de R15a -- o codigo fez o que foi
desenhado para fazer, com evidencia real. CORRECAO (revisao pos-R16): a
versao original desta frase dizia "nenhuma metrica de suavidade piorou",
o que contradiz a propria secao "Leitura" acima -- `plan_straightness` e a
assimetria de parede pioraram nesta rodada. O que de fato nao piorou (na
verdade, foi o melhor das cinco rodadas medidas) e `cmd_wz` e a amplitude
de guinada; isso nao cobre plan_straightness nem a assimetria, que sao
metricas separadas e pioraram. Nenhuma delas, porem, e evidencia de um
DEFEITO introduzido por R15a -- ver a ressalva de n=1/geometria variavel
logo abaixo. A questao em aberto e a mesma de R14b/R14c: se vale investir mais
tempo real de bancada em n>1 por configuracao antes de tirar qualquer
conclusao sobre o efeito do software de R15/R15a na taxa de conclusao de
metas, ou se e hora de aceitar a variancia entre rodadas como o limite
pratico deste metodo de medicao e seguir para os proximos itens do plano
(a recuperacao por varredura ainda sem exercicio real, o homing cego ainda
sem mitigacao, os 3 cold starts, a aceitacao final).

## Limitacoes

HIL real, nenhuma conclusao aqui vale para performance/CPU/termico sem
validacao propria (`CLAUDE.md` regra 5). n=1 por configuracao em todas as
cinco rodadas comparadas, geometria de exploracao diferente em cada uma --
o proprio metodo pode nao ter poder para separar efeito de software de
ruido de execucao, como ja discutido em R14b/R14c. A recuperacao por
varredura e o comportamento do vigia sob rotacao legitima permanecem sem
exercicio real em HIL; esta rodada so validou o lado "imobilidade real"
do vigia. O risco de queda no homing cego continua documentado e sem
mitigacao -- esta rodada nao o testou porque o marcador nunca foi visto,
nao porque o risco foi resolvido.
