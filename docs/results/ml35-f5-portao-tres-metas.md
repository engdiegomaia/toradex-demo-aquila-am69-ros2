# ML3.5 F5 — rodada C (`map_update_interval` 1.0) e o portão de metas

**Estado:** portão de TF **APROVADO**; portão de metas **PARCIAL** — todos os
critérios passam menos a velocidade média.
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
percentual** no `async_slam_toolbox` — simétrico ao 1,1 pp medido na ida
(31,4% -> 30,3%), o que é a confirmação cruzada de que o número é real e é
ruído. O portão de TF não se moveu. `/map` a 1,000 Hz prova que a config
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

| critério | resultado | |
| --- | --- | --- |
| 3/3 metas cumpridas | 3/3 nas três corridas | ok |
| zero quedas | tilt máx 1,18°, folga 0,448 m | ok |
| zero `worldToMap failed` | 0 | ok |
| zero `invalid source` | 0 | ok |
| zero extrapolações de TF | 0 | ok |
| velocidade média ≥ 0,05 m/s | 0,0342 – 0,0447 | **REPROVA** |

## O que a reprovação de velocidade significa, e o que não significa

**Não é regressão.** A linha de base registrada em `config/gait_go2.yaml` para o
maze11 (V0, média de n=3) é **0,0399 m/s**. As três corridas dão média 0,0391.
Decimar o broadcaster e reverter o mapa deixaram a velocidade onde ela estava.

**O que melhorou é outra coisa.** A razão de trabalho em vx foi de 6,2% (a
melhor corrida anterior, com `restamp_tf`) para **15,6–22,3%**: o robô passa
duas a três vezes mais tempo com comando de avanço efetivo. Isso não virou
velocidade média, e essa é exatamente a distância entre o limite de MÁQUINA,
que estas mudanças atacaram, e o limite de DECISÃO DE TRAJETO, que
`ml35-f5-clock-fanout.md` já havia isolado e que continua de pé.

**Cuidado com a métrica.** `maze11-short` são metas a 0,5 m; boa parte de cada
ciclo é reaquisição de meta próxima, não travessia. Velocidade de percurso neste
conjunto não é comparável com travessia de 8 m, e o critério de 0,05 m/s foi
escrito sem separar os dois casos. Isso é motivo para rever o critério, não para
declarar o portão aprovado.

**Nada aqui foi interrompido pelas condições de parada da diretriz 8:** não
houve extrapolação de TF, `worldToMap`, robô fora do costmap nem meta recusada.
O estouro de prazo da corrida 2 é prazo, não recusa.
