# R14b — segundo A/B do lado do MPPI, tambem negativo, e confundido por processo

Rodada HIL real (Aquila AM69 + host). Variavel pretendida: `PathAlignCritic.
cost_weight` 14.0 -> 7.0 em `nav2_params_go2.yaml` (offset_from_furthest de
volta a 20, o baseline de R13). Evidencia: `ml35-f5-exploration-r14b.csv`,
`ml35-f5-exploration-r14b-goals.csv`.

## Erro de processo descoberto durante a analise (declarado antes dos numeros)

`./scripts/module.sh sync` antes desta rodada sincronizou a arvore
`ros2_ws/src` inteira, o que **tambem** levou ao modulo as mudancas de R15
(vigia de movimento, recuperacao por varredura, `_map_seq` por conteudo) —
ja commitadas (`04b8764`) antes deste HIL comecar. Confirmado nos dados:
3 das 15 metas desta rodada terminam com a mensagem `vigia de movimento:
comando sem deslocamento real`, que nao existe no codigo de R13/R14.

**Consequencia:** esta rodada testou `cost_weight=7.0` **junto com** as
quatro mudancas de software de R15, nao `cost_weight` isolado. Comparar
com R13/R14 (que rodaram sem R15) e valido para uma leitura de tendencia
geral, mas nao e um A/B de variavel unica no sentido que o plano pede.
Registrado aqui como o que de fato aconteceu, nao escondido.

## Precondicoes

1. `/demo/sim/reset` -> `success=True`, robo em `(0.00011, -0.00609)`.
2. Config + arvore sincronizadas (`module.sh sync`), `nav`/`perception`
   recriados (`module.sh up`) com o robo ja no spawn.
3. Confirmado antes de iniciar: `state=idle, frontier_count=0`, `/map`
   88x87 com origem `(-3.858, -0.478)`.
4. `/demo/exploration/start` chamado exatamente uma vez.

## Comparativo (script unico, mesma metodologia nas tres rodadas)

| metrica (subconjunto "corredor reto genuino") | R13 (offset20/w14) | R14 (offset10/w14) | R14b (offset20/w7, + R15) |
| --- | --- | --- | --- |
| amostras no subconjunto | 445 / 1320 | 131 / 1320 | 261 / 1320 |
| `|cmd_wz|` mediana | 0.056 | 0.079 | 0.078 |
| `|cmd_wz|` p90 | 0.124 | 0.123 | 0.136 |
| fracao `|cmd_wz|` > 0.02 | 82,7% | 76,3% | 81,2% |
| assimetria E/D, mediana `|E-D|` | 0.25 m | 0.25 m | **0.65 m** |
| direcao da assimetria | ~41% E / 33% D / 26% centro | 63% E / 15% D / 21% centro | **57% D / 30% E / 13% centro** |
| `plan_straightness` global, mediana | 0.96 | 0.933 | **0.872** |
| `plan_straightness`, fracao < 0.9 | 12,4% | 39,1% | **57,2%** |
| amplitude pico-a-pico de yaw/~5s, mediana | 10,7° | 16,9° | 14,2° |
| folga minima de parede | 0.05 m | 0.10 m | 0.05 m |
| metas ok / total | 16/23 (70%) | 9/16 (56%) | **7/15 (47%)** |
| motivo da parada | `total_timeout` | `barren_other` | `barren_other` |
| duracao ativa | 600,5 s | 500,4 s | **402,5 s** |
| janelas de imobilidade | 5 | 4 | **6** |

(Os numeros de R13/R14 aqui foram recalculados com o mesmo script desta
rodada para garantir metodologia identica; o subconjunto de R13 sai 445
amostras aqui contra 389 no relatorio original de R13 — pequena diferenca
de metodologia entre o script ad-hoc daquela rodada e este, nao investigada,
mas nao muda a leitura qualitativa.)

Nenhuma metrica melhorou. Varias pioraram na direcao errada, e por uma
margem maior que R14: a assimetria de parede triplicou em magnitude (0.65 m
contra 0.25 m nas duas rodadas anteriores) e trocou de vies esquerdo (R14)
para vies direito -- ainda um vies fixo, nao a oscilacao 50/50 que a
hipotese original de R13 descrevia. `plan_straightness` teve a pior leitura
das tres rodadas. A taxa de metas concluidas caiu mais uma vez (70% -> 56%
-> 47%), e a rodada parou ainda mais cedo (600 -> 500 -> 402 s), sempre por
`barren_other`.

Zero quedas nesta rodada tambem (tilt max 1.19°). Marcador da saida foi
visto 24 vezes (`marker_observations=24`), sempre alem de
`homing_max_distance_m` (4.88-7.74 m, todas ignoradas como "longe demais")
-- o robo chegou perto o suficiente para AVISTAR a saida nesta rodada, o
que nao tinha acontecido em R13/R14.

## Leitura

Duas tentativas consecutivas no lado do MPPI (`offset_from_furthest` e
`cost_weight`, ambas no mesmo critico dominante) mediram **negativo ou
neutro-para-pior** em quase todo criterio disponivel, nunca uma melhora
clara. A segunda, alem disso, saiu confundida por um erro de processo (R15
junto). Isso levanta uma possibilidade que nenhuma das duas rodadas
isoladas apontava sozinha: **o proprio metodo de comparacao n=1, com
geometria de labirinto diferente a cada rodada, pode nao ter poder
suficiente para detectar o efeito real destas mudancas** -- ou a hipotese
de R13 (oscilacao dominada por um unico critico) pode estar incompleta.
Nenhuma das duas alternativas foi testada aqui.

## Recomendacao

`cost_weight` revertido a 14.0 na mesma rodada de commits, pelo mesmo
motivo de R14: nao empilhar mudanca nao demonstrada. Antes de gastar mais
tempo real de bancada num terceiro A/B do lado do MPPI (peso do
PathAngleCritic ou frequencia de replanejamento, as duas opcoes que
restam), duas coisas parecem mais uteis que adivinhar a proxima variavel:

1. Corrigir o processo -- sincronizar so o arquivo de config (ou uma
   arvore congelada) num HIL que se pretende de variavel unica, para nao
   repetir a confusao desta rodada.
2. Decidir explicitamente se vale a pena continuar neste ramo (MPPI) com
   n=1 por tentativa, ou se e hora de tentar `k_yaw` (a alternativa mais
   barata do plano original, ainda nao tentada) ou repetir uma das duas
   configuracoes ja tentadas para separar efeito real de ruido de
   execucao.

## Limitacoes

HIL real, nenhuma conclusao aqui vale para performance/CPU/termico sem
validacao propria (`CLAUDE.md` regra 5). n=1 por configuracao nas tres
rodadas, geometria de exploracao diferente em cada uma. Esta rodada
especificamente NAO isola `cost_weight` (ver secao de erro de processo
acima) -- trate como evidencia de tendencia, nao como A/B controlado.
