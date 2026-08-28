# ML3.5 F5 — rodada C (`map_update_interval` 1.0) e o portão de metas

**Estado:** portão de TF **APROVADO**; portão curto de estabilidade
**APROVADO**; portão de desempenho de travessia **NÃO EXECUTADO**.
**Data:** 28/08/2026. **Commit:** `d7efd30`.
Configuração: `joint_state_broadcaster` a 50 Hz, `map_update_interval` 1.0,
costmap global 40 m / 0,10 m, `restamp_tf: true`. Mundo `quadruped_maze11`,
HIL real sobre Ethernet. Nada aqui valida hardware Go2, térmica ou consumo.

## Rodada C — a reversão do mapa não custou nada

Rodada independente, uma variável contra o braço B: `map_update_interval`
5.0 -> 1.0. Mesmo protocolo (recria sim, reinicia módulo, espera SLAM, 300 s).

| | B (map 5.0) | C (map 1.0) |
| --- | ---: | ---: |
| `/map` | 0,2 Hz | **1,000 Hz** |
| `odom <- lidar` disponível | 99,94% | **99,94%** |
| `odom <- base` disponível | 99,94% | 99,94% |
| `base <- lidar` disponível | 100,00% | 100,00% |
| `async_slam_toolbox` | 18,0% | 19,0% |
| `nav2_container` | 240,0% | 256,0% |
| `demo-nav-1` | 403,07% | 422,91% |
| carga média | 18,52 | 17,86 |
| `odom_tf`, taxa | 49,76 Hz | **49,97 Hz** |
| `odom -> base`, chegada máx | 134,6 ms | **65,2 ms** |
| fator de tempo real | 0,976 | 0,988 |

O custo da rasterização a cada segundo em vez de a cada cinco é **1 ponto
percentual** no `async_slam_toolbox`, simétrico ao 1,1 pp medido na ida
(31,4% -> 30,3%). A simetria é o que dá confiança na medida: é uma **variação
pequena e operacionalmente irrelevante**, não uma economia. O portão de TF não
se moveu. `/map` a 1,000 Hz prova que a config
montada no módulo está viva.

## Portão de metas: 3 corridas de 180 s, `maze11-short`

O conjunto `maze11-short` são três metas de corredor a 0,5 m uma da outra,
recicladas até o corte de 180 s — então cada corrida encerra ~10 metas, não 3.

| | corrida 1 | corrida 2 | corrida 3 |
| --- | ---: | ---: | ---: |
| metas cumpridas / encerradas | 9 / 10 | 8 / 10 | 9 / 10 |
| primeiras TRÊS metas | ok ok ok | ok ok ok | ok ok ok |
| estouros de prazo | 0 | **1** (meta 3) | 0 |
| canceladas no corte | 1 | 1 | 1 |
| razão de trabalho em vx | 17,1% | 15,6% | 22,3% |
| `cmd_vx` negativo | 0% | 0% | 0% |
| tilt de pico | 1,17° | 1,06° | 1,18° |
| folga mínima (lidar) | 0,448 m | 0,448 m | 0,448 m |
| percurso / velocidade média | 6,88 m / **0,0383** | 6,16 m / **0,0342** | 8,04 m / **0,0447** |

Varredura de log limitada, janela de 12 min cobrindo as três corridas,
container `nav`:

```
worldToMap | invalid source | extrapolation into the future  ->  0 ocorrências
```

O único aviso recorrente é `Control loop missed its desired rate of 10.0000 Hz`,
com taxa corrente entre 8,6 e 10,4 Hz.

## Veredito por critério

O contrato do portão foi **dividido** ao interpretar estes números, e a divisão
está justificada na seção seguinte. Não é o limite que baixou; é o critério que
passou a medir o que o protocolo de fato produz.

### Portão curto de ESTABILIDADE — APROVADO

| critério | resultado | |
| --- | --- | --- |
| primeiras três metas `SUCCEEDED` | 3/3 nas três corridas | ok |
| cada meta dentro de 45 s | máximo **27,9 s** (corrida 3, meta 0) | ok |
| zero erros do Nav2 | nenhum | ok |
| zero extrapolações de TF | 0 | ok |
| zero `worldToMap failed` | 0 | ok |
| zero `invalid source` | 0 | ok |
| zero quedas | tilt máx 1,18°, folga 0,448 m | ok |
| zero trocas grandes de rota injustificadas | 0 por meta nas nove | ok |

Duração por meta medida de ENVIO a ENVIO (`sent_sim_s` consecutivos), que é o
que o CSV sustenta: a coluna `elapsed_s` é acumulada desde o início do ensaio e
não é duração de meta. Nas três corridas, as três primeiras metas:

    corrida 1   12,4   4,8   5,1 s
    corrida 2   16,5  15,8   5,8 s
    corrida 3   27,9  21,3   5,3 s

A primeira meta de cada corrida é sempre a mais cara — carrega a aquisição
inicial — e mesmo ela fica a 17 s do limite no pior caso.

### Portão de DESEMPENHO de travessia — NÃO EXECUTADO

Percurso representativo, com metas separadas por pelo menos o horizonte do
MPPI — ou, de preferência, a própria saída autônoma em até 600 s. Nenhum dos
dois foi rodado. `maze11-short` **não serve** para este portão.

## Por que o contrato foi dividido, e por que isso não é baixar a régua

O limite de 0,05 m/s foi importado de um ensaio de TRAVESSIA e aplicado a metas
separadas por 0,5 m. Neste protocolo a média de percurso inclui aceitação e
preparação de cada meta, aceleração, desaceleração pelo goal checker, nova
aquisição e a rota de retorno quando a sequência é reciclada. Isso mede
**estabilidade e latência de metas**, não velocidade de travessia.

A correção errada seria baixar o limite até os números passarem. A correta é
separar os dois portões, que medem coisas diferentes e devem ter protocolos
diferentes. O limite de travessia continua **0,05 m/s e não foi tocado** — ele
só deixou de ser cobrado de um ensaio que não é de travessia.

Para o registro, os números de percurso do `maze11-short`, que **não são
critério de nada**:

| | corrida 1 | corrida 2 | corrida 3 |
| --- | ---: | ---: | ---: |
| percurso | 6,88 m | 6,16 m | 8,04 m |
| velocidade média | 0,0383 | 0,0342 | 0,0447 m/s |

E o que eles não dizem: a linha de base do maze11 registrada em
`config/gait_go2.yaml` (V0, média de n=3) é **0,0399 m/s**, e estas três dão
média 0,0391. Decimar o broadcaster e reverter o mapa **não mexeram na
velocidade**. O que mexeu foi a razão de trabalho em vx, de 6,2% para
15,6–22,3%: o robô passa duas a três vezes mais tempo com avanço efetivo, e isso
não virou velocidade média. É exatamente a distância entre o limite de MÁQUINA,
que esta sessão atacou e fechou, e o limite de DECISÃO DE TRAJETO, isolado em
`ml35-f5-clock-fanout.md` e ainda de pé — que é o que o portão de desempenho
existe para medir.

## O estouro de prazo da corrida 2

Ocorreu na **quarta** meta, já na sequência reciclada, fora do contrato de três
metas. Fica registrado como sinal de **variabilidade** — uma em trinta metas
encerradas — e não invalida o portão curto. Se reaparecer no smoke da
exploração, vira sintoma; isolado, é ruído amostral.

## `Control loop missed`: métrica, não bloqueio

O único aviso recorrente é `Control loop missed its desired rate of 10.0000 Hz`,
com taxa corrente entre 8,6 e 10,4 Hz. Uma faixa isolada não informa frequência
nem gravidade, então **não** bloqueia nada aqui. O que decide são números que
esta corrida não coletou e o smoke da exploração deve coletar: total de avisos,
avisos por minuto, maior sequência consecutiva, e correlação com meta parada ou
comando zero. Só há motivo para perfilar `nav2_container` e `maze_explorer` se
houver sequência sustentada abaixo da frequência desejada COM paradas
correlacionadas.
