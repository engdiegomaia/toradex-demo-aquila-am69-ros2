# ML3.5 F5 — A/B do `joint_state_broadcaster`: 1000 Hz contra 50 Hz

**Estado:** EXECUTADO. Portão de TF ≥ 99,5% **APROVADO** (99,94%).
Um critério do plano **REPROVOU**: a CPU do `maze_explorer` não caiu.
**Data:** 28/08/2026. **Commit:** `235ac1f` (código), imagem `sim` reconstruída
a partir de `a7dc097`.

HIL real: Gazebo Harmonic + cockpit na estação x86, `demo-nav-1` /
`demo-perception-1` / `demo-tools-1` no Aquila AM69 sobre Ethernet,
`ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`. Mundo `quadruped_maze11`.
**Robô parado** nos dois braços — nenhuma meta ativa. Nada aqui valida marcha,
Go2 físico, térmica ou consumo (regras 5 e 7 do `CLAUDE.md`).

Uma variável entre os braços: o conteúdo de
`demo_simulation/config/joint_state_broadcaster.yaml`.

Evidência bruta ao lado deste arquivo:

- `ml35-f5-ab-A-jsb1000.csv` / `-odomtf.csv` — braço A, 180 s
- `ml35-f5-ab-B-jsb50.csv` / `-odomtf.csv` — braço B, 180 s

## Protocolo, e por que ele é o que é

Cada braço: **recria o `sim` → reinicia a pilha do módulo → espera o SLAM →
estabiliza → mede.** Idêntico nos dois.

A simetria não é zelo: a primeira tentativa de braço B foi medida contra um
`sim` recém-reiniciado enquanto o módulo seguia de pé havia duas horas, com um
mapa maduro. Como a disponibilidade de TF depende da carga do módulo e a carga
do `nav2_container` depende do tamanho do costmap, essa corrida não podia ser
comparada com o braço A. Foi descartada e refeita.

O braço A **desta** tabela não é a medição de 1090 Hz da manhã: é uma corrida
nova, com o mesmo protocolo do braço B, obtida montando por cima do arquivo
instalado um YAML com `update_rate: 1000`. Mesma imagem, mesmo launch, mesmo
número de reinícios.

## Resultado

| | A — 1000 Hz | B — 50 Hz | critério | |
| --- | ---: | ---: | --- | --- |
| `/joint_states` | 986,069 Hz | **45,133 Hz** | 45–55 | ok |
| `/tf` | 1054,503 Hz | **144,599 Hz** | queda > 80% | ok (**−86,3%**) |
| `/demo/odom` (sonda, tempo de sim) | 50,00 Hz | 50,00 Hz | 49–50 | ok |
| `/clock` | 973,693 Hz | 940,282 Hz | inalterado | ok |
| **`odom <- lidar` disponível** | **94,75%** | **99,94%** | ≥ 99,5% | **ok** |
| `odom <- base` disponível | 94,75% | 99,94% | — | ok |
| `base <- lidar` disponível | 100,00% | 100,00% | — | ok |
| idade mediana da nuvem | 65,0 ms | 27,0 ms | < 150 | ok |
| TF atrás da nuvem, p99 | 60,0 ms | **0,0 ms** | — | ok |
| carimbos no futuro | 0,00% | 0,00% | 0 | ok |
| fator de tempo real | 0,983 | 0,976 | — | ok |
| `demo-nav-1` | 484,93% | **403,07%** | — | −81,9 pp |
| `demo-perception-1` | 281,81% | 242,13% | — | −39,7 pp |
| carga média do módulo | 26,90 | **18,52** | — | −31% |
| `nav2_container` | 298,0% | 240,0% | — | −58 pp |
| `pointcloud_to_laserscan` | 36,3% | 23,0% | — | −13,3 |
| `async_slam_toolbox` | 31,4% | 18,0% | — | −13,4 |
| `odom_tf` | 19,6% | 19,0% | — | ~igual |
| **`maze_explorer`** | **67,6%** | **76,0%** | queda material | **REPROVA** |

`/tf` a 144,6 Hz e não a 50 porque três publicadores restam ali — o
`robot_state_publisher` agora decimado, o `odom_tf` e o `map -> odom` do SLAM,
cada um a ~50 Hz. Os 1054 Hz do braço A eram as doze juntas das pernas.

`/demo/odom` medido em tempo de PAREDE dá 48,9 Hz, abaixo da faixa; medido em
tempo de simulação pela sonda dá 50,00 Hz. A diferença é o fator de tempo real
de 0,976. A faixa é sobre a taxa da odometria, não sobre a lentidão do
simulador.

## Regularidade de `odom -> base`

Amostrador de 200 Hz, duas séries: o intervalo entre carimbos DISTINTOS (o que
o `odom_tf` diz) e o intervalo entre as PRIMEIRAS observações (quando chegou).

| | A — 1000 Hz | B — 50 Hz |
| --- | ---: | ---: |
| amostras / taxa | 8758 / 49,47 Hz | 8735 / 49,76 Hz |
| carimbo, p99 | 20,0 ms | 20,0 ms |
| **carimbo, máximo** | **120,0 ms** | **40,0 ms** |
| chegada, p99 | 50,0 ms | 45,0 ms |
| chegada, máximo | 108,8 ms | 134,6 ms |

A leitura importa: no braço A o `odom_tf` **pulava ciclos de carimbo** — um
máximo de 120 ms num publicador de 20 ms são cinco ciclos perdidos de uma vez.
No braço B o pior caso é 40 ms, um ciclo. Não era rajada de entrega; era o nó
não sendo escalonado a tempo de carimbar.

## O que isto fecha, e o que não fecha

**Fecha a atribuição da aresta.** `odom <- base` e `odom <- lidar` deram
**exatamente o mesmo número** nos dois braços — 94,75% e 94,75%, depois 99,94% e
99,94%. A cadeia composta não perde nada além do que a aresta dinâmica perde, e
a estática está em 100% nos dois. Toda a indisponibilidade era do `odom_tf`.

Isto também corrige a atribuição de 28/08 pela manhã, que comparava três
execuções separadas de durações diferentes (100,00% / 95,30% / 78,11%). A
conclusão qualitativa estava certa; os números não eram comparáveis entre si.

**Fecha a causalidade pedida.** A queda de `/tf` melhorou as três coisas ao
mesmo tempo: disponibilidade temporal (94,75 → 99,94), CPU dos consumidores
(`nav2_container` −58 pp, `pointcloud_to_laserscan` −13,3, `async_slam` −13,4,
carga do módulo −31%) e regularidade de `odom -> base` (máximo de 120 → 40 ms).

**NÃO fecha o `maze_explorer`, e essa era a minha hipótese.** Eu havia dito que
os ~68% de um explorador OCIOSO eram o `TransformListener` dele deserializando
1090 mensagens por segundo. Com o fluxo reduzido em 86%, ele **subiu**, de 67,6%
para 76,0%. A hipótese está errada como enunciada.

A explicação plausível é que ele estava sendo ESTRANGULADO: com a carga do
módulo caindo de 26,9 para 18,5, um nó que antes disputava escalonamento passa a
rodar à vontade e a consumir mais tempo de CPU por segundo de parede. Isso é
consistente com o resto da tabela, mas **é hipótese, não medição** — a prova
seria perfilar as threads do processo, como foi feito em
`ml35-f5-clock-fanout.md`.

**Nada aqui diz nada sobre a marcha.** O robô ficou parado nos dois braços. A
verificação de que decimar o broadcaster não degradou o andar é o próximo passo
e ainda não foi feita.

## Limitações

- O `maze_explorer` aparece nos dois containers do `top`: em `nav` é o
  explorador; em `perception` a linha truncada é o `maze_exit_detector`. Os
  números acima são o de `nav`.
- Uma corrida de braço B foi **descartada** e não está aqui: a sonda subiu na
  janela logo após o restart do `sim`, quando `/clock` ainda não chegava a
  assinante novo (RTF 0,000, 100% de carimbos "no futuro"). Disponibilidade e
  intervalos daquela corrida eram válidos — não usam o relógio do nó — mas idade
  e RTF não eram, e uma tabela com metade das colunas inválidas não é evidência.
  A fome de `/clock` era transitória: 968 Hz para um assinante novo minutos
  depois.
- `map_update_interval` ficou em 5,0 nos DOIS braços, como o plano exige. A
  reversão para 1,0 é rodada própria.
