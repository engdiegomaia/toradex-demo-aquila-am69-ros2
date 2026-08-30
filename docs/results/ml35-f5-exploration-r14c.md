# R14c — retest isolado de `cost_weight`, resultado misto

Rodada HIL real (Aquila AM69 + host). Variavel pretendida: `PathAlignCritic.
cost_weight` 14.0 -> 7.0 em `nav2_params_go2.yaml` (`offset_from_furthest` de
volta a 20, o baseline de R13), desta vez testada em isolamento verdadeiro.
Evidencia: `ml35-f5-exploration-r14c.csv`, `ml35-f5-exploration-r14c-goals.csv`.

## Correcao do processo (por que esta rodada e diferente de R14b)

R14b mediu `cost_weight=7.0` **junto com** as quatro mudancas de software de
R15 (vigia de movimento, recuperacao por varredura, `_map_seq` por conteudo),
porque `module.sh sync` levou a arvore `ros2_ws/src` inteira, nao so o
arquivo de config. Para isolar de verdade a variavel desta rodada,
`demo_navigation/maze_explorer.py` foi **temporariamente substituido** no
modulo pelo conteudo pre-R15 (`git show 11aba10:...`), mantendo apenas a
mudanca de `cost_weight` no config. Depois desta analise, o arquivo foi
restaurado ao conteudo real de R15 (`git checkout HEAD -- ...`) -- a rodada
em si nao teve nenhum efeito permanente sobre o codigo de R15.

## Precondicoes

1. `/demo/sim/reset` -> robo confirmado no spawn.
2. Config com `cost_weight=7.0` (unica mudanca real) + `maze_explorer.py`
   pre-R15 sincronizados; `nav`/`perception` recriados com o robo ja no
   spawn.
3. Confirmado antes de iniciar: `state=idle`, mapa fresco.
4. `/demo/exploration/start` chamado exatamente uma vez.

## Comparativo (script unico, mesma metodologia nas quatro rodadas)

| metrica (subconjunto "corredor reto genuino") | R13 (offset20/w14) | R14 (offset10/w14) | R14b (offset20/w7, +R15) | R14c (offset20/w7, isolado) |
| --- | --- | --- | --- | --- |
| amostras no subconjunto | 445 / 1320 | 131 / 1320 | 261 / 1320 | 320 / 1320 |
| `\|cmd_wz\|` mediana | 0.056 | 0.079 | 0.078 | **0.048** |
| `\|cmd_wz\|` p90 | 0.124 | 0.123 | 0.136 | 0.126 |
| fracao `\|cmd_wz\|` > 0.02 | 82,7% | 76,3% | 81,2% | 77,5% |
| assimetria E/D, mediana `\|E-D\|` | 0.25 m | 0.25 m | 0.65 m | **0.25 m** |
| direcao da assimetria | 41% E / 33% D / 26% centro | 63% E / 15% D / 21% centro | 57% D / 30% E / 13% centro | **45% E / 31% D / 25% centro** |
| `plan_straightness` global, mediana | 0.96 | 0.933 | 0.872 | 0.942 |
| `plan_straightness`, fracao < 0.9 | 12,4% | 39,1% | 57,2% | **31,7%** |
| amplitude pico-a-pico de yaw/~5s, mediana | 10,7° | 16,9° | 14,2° | **10,2°** |
| folga minima de parede | 0.05 m | 0.10 m | 0.05 m | 0.05 m |
| metas ok / total | 16/23 (70%) | 9/16 (56%) | 7/15 (47%) | **11/21 (52%)** |
| motivo da parada | `total_timeout` | `barren_other` | `barren_other` | **`total_timeout`** |
| duracao ativa | 600,5 s | 500,4 s | 402,5 s | 591,2 s |
| janelas de imobilidade | 5 | 4 | 6 | **7** |
| tilt maximo | -- | -- | 1,19° | 1,73° |

(R13/R14/R14b recalculados com o mesmo script desta rodada; ver nota de
metodologia em `ml35-f5-exploration-r14b.md`.)

## Leitura

Isolado de verdade do codigo de R15, `cost_weight=7.0` produz um quadro
**misto**, bem diferente do negativo uniforme de R14b:

- **Melhor ou igual ao baseline** nas metricas mais diretamente ligadas ao
  zigzag: `cmd_wz` mediano caiu abaixo do proprio baseline R13 (0.048 contra
  0.056 rad/s), a amplitude de guinada por janela de 5s tambem caiu levemente
  (10,2° contra 10,7°), e a assimetria de parede voltou ao patamar e a
  distribuicao de R13 (mediana 0.25 m, ~45/31/25 contra ~41/33/26) -- isto
  **indica que o vies fixo severo de R14b nao era efeito isolado de
  `cost_weight`**; nao confirma que a causa era especificamente o codigo de
  R15, ja que R13/R14b/R14c cada uma explorou uma geometria de labirinto
  diferente com n=1 por configuracao. A contribuicao real de R15 (positiva,
  negativa ou nula sobre este eixo) permanece em aberto ate a validacao
  integrada em R16.
- **Pior que o baseline** em `plan_straightness` (fracao abaixo de 0.9 subiu
  de 12,4% para 31,7%, embora bem melhor que os 57,2% de R14b) e sobretudo
  em indicadores de missao: menos janelas de imobilidade nunca -- pelo
  contrario, 7 janelas, a pior contagem das quatro rodadas -- e taxa de metas
  concluidas mais baixa que R13 (52% contra 70%).
- A rodada terminou por `total_timeout` (o padrao saudavel, igual a R13, ao
  contrario de R14/R14b que pararam cedo por `barren_other`), e sem quedas
  (tilt maximo 1,73°, ainda bem abaixo de qualquer limiar de risco).

Nao ha vitoria liquida clara: as metricas de suavidade de trajetoria melhoram
marginalmente, mas os indicadores de missao (metas concluidas, janelas de
imobilidade) pioram. Com n=1 por configuracao e geometria de exploracao
diferente a cada rodada, nao da para separar com confianca "cost_weight=7.0
ajuda um pouco e atrapalha um pouco" de "ruido de execucao entre rodadas".

## Recomendacao

Revertido a 14.0 (commit desta rodada). Duas tentativas consecutivas no
mesmo critico dominante (`offset_from_furthest`, depois `cost_weight`) nao
produziram uma melhora inequivoca -- a segunda, mesmo isolada corretamente,
saiu mista. Isso reforca a possibilidade ja levantada em R14b: o metodo de
comparacao n=1 pode nao ter poder suficiente para o efeito real destas
mudancas, ou a hipotese de R13 (oscilacao dominada por um unico peso de
critico) esta incompleta.

Proximo passo recomendado: **nao abrir um terceiro A/B de peso de MPPI**
(PathAngleCritic ou frequencia de replanejamento). Em vez disso, tratar o
zigzag como mitigado (nao eliminado) por ajuste de controlador, e contar com
as quatro correcoes de software de R15 (vigia de movimento, recuperacao por
varredura, `_map_seq` por conteudo, classificacao honesta de estado
terminal) -- ja implementadas e testadas em unidade, ainda nao validadas em
HIL -- para lidar com as consequencias praticas (travamentos, mapa sem
fronteiras) em vez de eliminar a causa raiz via peso do MPPI. R16 (validacao
integrada) e o proximo HIL real a rodar.

## Limitacoes

HIL real, nenhuma conclusao aqui vale para performance/CPU/termico sem
validacao propria (`CLAUDE.md` regra 5). n=1 por configuracao nas quatro
rodadas, geometria de exploracao diferente em cada uma. Esta rodada isola
`cost_weight` do codigo de R15 corretamente, mas por isso tambem NAO reflete
o comportamento que vai efetivamente rodar em R16 (que inclui R15) -- serve
para julgar a variavel de MPPI, nao para prever R16.
