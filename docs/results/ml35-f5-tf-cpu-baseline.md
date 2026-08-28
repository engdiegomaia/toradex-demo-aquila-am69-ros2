# ML3.5 F5 — sonda temporal do lidar e orçamento de CPU do módulo

Data: 28/08/2026. Executado no HIL real: Gazebo Harmonic e cockpit na estação
x86, `demo-nav-1` / `demo-perception-1` / `demo-tools-1` no Aquila AM69 sobre
Ethernet, `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, ambos os lados na mesma /24.

Módulo: Torizon OS 7.7.0+build.40 (scarthgap), 8 núcleos, 31 GiB, docker 25.0.9
arm64. Robô **parado** durante todas as medições — nenhuma meta ativa, nenhum
controller em malha. Isso é uma limitação importante e está anotada no fim.

Evidência bruta, ao lado deste arquivo (`artifacts/` é rascunho e não entra em
git — a convenção deste projeto é a evidência viver em `docs/results/`):

- `ml35-f5-tf-lidar-baseline.csv` — 180 s, configuração anterior
- `ml35-f5-tf-lidar-rodadaB.csv` — 180 s, Etapas 2 e 3 aplicadas
- `ml35-f5-tf-pair-odom-base.csv` — aresta dinâmica isolada
- `ml35-f5-tf-pair-base-lidar.csv` — aresta estática isolada

---

## 1. Veredito da Etapa 1: REPROVA, num critério só

| Critério | Alvo | Baseline | Rodada B | |
| --- | --- | --- | --- | --- |
| taxa de `/demo/scan_cloud` | ~9–10 Hz | 10,00 Hz | 10,00 Hz | ok |
| taxa de `/demo/odom` | ~49–50 Hz | 50,00 Hz | 50,00 Hz | ok |
| idade mediana da nuvem | < 150 ms | 30,0 ms | 30,0 ms | ok |
| idade p95 / p99 / máx | — | 41 / 45 / 50 ms | 40 / 44 / 47 ms | ok |
| maior lacuna sem nuvem | sem lacuna de segundos | 100 ms | 100 ms | ok |
| carimbos no futuro | 0 | 0,00 % | 0,00 % | ok |
| fator de tempo real | — | 0,992 | 0,993 | ok |
| **TF disponível** | **≥ 99,5 %** | **78,11 %** | **79,13 %** | **REPROVA** |

1786 e 1787 amostras, 180 s cada. Tudo o que é transporte passa com folga: a
nuvem chega 30 ms depois do próprio carimbo, a odometria corre a 50 Hz, o
relógio não salta e não há carimbo no futuro. O problema é exclusivamente a
disponibilidade da transformação.

## 2. O que exatamente falha na TF

`transform_latency_ms` é `carimbo_da_nuvem − carimbo_da_TF_mais_recente`.
Positivo significa que a árvore está ATRÁS da nuvem e o consumidor teria de
extrapolar para frente.

| | mediana | p95 | máx |
| --- | --- | --- | --- |
| amostras que PASSAM | −20 ms | 0 ms | 0 ms |
| amostras que FALHAM | +60 ms | +100 ms | +160 ms |

A distribuição é **bimodal**, não uma cauda: ou a TF está à frente da nuvem e a
interpolação funciona, ou está 60–160 ms atrás e `can_transform` recusa —
corretamente. Não é ruído de medição e não é aquecimento do buffer: a primeira
TF ficou disponível na amostra 0, e depois do aquecimento 37,4 % das amostras
continuam falhando.

Isolando as duas arestas com `--target-frame` / `--source-frame`:

| par | disponível |
| --- | --- |
| `base ← lidar` (estática, `/tf_static`) | 100,00 % |
| `odom ← base` (dinâmica, nó `odom_tf`) | 95,30 % |
| `odom ← lidar` (a cadeia composta, que a `ObstacleLayer` usa) | 78–79 % |

A aresta que atrasa é a dinâmica. `base ← lidar` não expira, e o número de
atraso que a sonda imprime para ela (~4,8×10⁶ ms) é o carimbo de publicação do
`/tf_static` — a métrica de atraso não tem significado para par estático, e isso
é limitação da sonda, não achado.

## 3. A causa: o módulo está saturado

`docker stats --no-stream`, três amostras, e `top -b -n2` dentro dos containers:

```
demo-nav-1          481–494 %      de 800 % disponíveis
demo-perception-1   263–277 %
                    ---------
                    ~750–770 %     load average 24–26 em 8 núcleos
```

Atribuição por processo (medida instantânea, segunda iteração do `top`):

| processo | % de um núcleo | container |
| --- | --- | --- |
| `nav2_container` (`component_container_isolated`) | 298,4 | nav |
| `maze_explorer` | 71,7 | nav |
| `pointcloud_to_laserscan` | 35,5 | nav |
| `async_slam_toolbox_node` | 30,3 | nav |
| `target_monitor` | 20,1 | nav |
| **`odom_tf`** | **19,4** | nav |
| `maze_exit_detector` | 81,4 | perception |
| `detection_stub` | 76,7 | perception |
| `detections_to_cloud` | 69,4 | perception |
| `camera_decompressor` (`republish`) | 37,2 | perception |

`odom_tf` — que é **o publicador da única aresta de TF que falha** — é um nó
Python disputando 19 % de núcleo numa máquina com fila de execução três vezes
maior que a contagem de núcleos. A hipótese que amarra tudo é que ele não é
escalonado a tempo, publica em rajadas, e a árvore fica 60–160 ms atrás da
nuvem. É consistente com a bimodalidade e com o par isolado, mas **não foi
provada**: a prova seria medir o intervalo de publicação do próprio `odom_tf`
sob carga e sem carga.

## 4. O que a rodada B mediu, e o que ela não mudou

Rodada B = Etapas 2 e 3 aplicadas juntas (as duas são YAML, e só
`demo_navigation/config` é montado no módulo — Python exige rebuild).

**Etapa 3 — costmap global: confirmada em runtime.**

```
resolution: 0.10000000149011612    width: 400    height: 400
origem (-19.90, -19.90)  →  40,0 x 40,0 m centrados no robô
```

A grade mestre manteve 400 células por eixo e a cobertura dobrou para ±20 m,
que é o que o labirinto de diagonal ~18,4 m exigia. O risco que estava em aberto
— a `static_layer` reprojetar entre o mapa de 5 cm do SLAM e a mestre de 10 cm —
**não se materializou**: o costmap publicado contém 128 células letais e 795
inscritas. Ele reprojeta.

**Etapa 2 — `map_update_interval` 1.0 → 5.0: NÃO reduziu CPU.**

```
async_slam_toolbox_node    31,4 %  →  30,3 %
demo-nav-1 (total)        493,6 %  → 481–494 %
TF disponível              78,11 % →  79,13 %
```

1,1 ponto percentual, dentro do ruído. A leitura correta é que a rasterização da
grade não era onde a CPU do `slam_toolbox` estava — o casamento de scans é, e ele
roda a cada scan independentemente deste parâmetro. O comentário no
`slam_params.yaml` já dizia que o parâmetro só controla a rasterização; a medição
confirma que ele está certo e que a economia esperada não existia.

**Pelo critério do próprio plano** — "manter 5.0 somente se reduzir CPU sem
piorar navegação ou atualização do mapa" — 5.0 não se justifica pela CPU. O que
resta a favor dele é ser o default do upstream; contra, é deixar a
`static_layer` até 5 s atrás da parede que o SLAM já conhece. Decisão em aberto.

## 5. O que isto significa para as queixas de origem

- **"o consumo de CPU aumentou"** — confirmado e quantificado: ~770 % de 800 %,
  fila de execução 24–26. Não é percepção do operador.
- **"a navegação piorou"** — 21 % das nuvens não são transformáveis no carimbo
  delas. Toda nuvem nessa fatia é observação que a `ObstacleLayer` descarta.
- **"perde dados do mapa"** — a causa da janela de 20 m está corrigida e
  verificada (seção 4). A fatia de 21 % descartada é uma SEGUNDA causa, distinta,
  e continua aberta.

Os dois maiores consumidores — `nav2_container` a 298 % e o trio de percepção a
~227 % — não são tocados por nada que foi aplicado até aqui. Chamar a atenção
para `detection_stub` a 76,7 %: ele é, por contrato, um gerador determinístico de
detecções sintéticas.

## 6. O achado que reenquadra tudo: `/tf` a ~1090 Hz

Medido no host, com a pilha completa no ar:

```
ros2 topic hz /tf            average rate: 1090.898   (window 10000)
ros2 topic hz /joint_states  min 0.000s  max 0.004s   (mesma ordem)
ros2 topic info /tf          Publishers: 4  |  Subscribers: 11
                             Node name: robot_state_publisher
```

O controlador de marcha publica estado de junta no passo de física de 1 ms, e o
`robot_state_publisher` converte fielmente cada um num `TFMessage`. O resultado é
**~1090 mensagens de TF por segundo**, e os ONZE assinantes as deserializam.

Foi isto que o teste de deploy da Etapa 4 revelou, por acidente. Com a
otimização de fronteira JÁ no módulo (verificado: `_frontier_cells` e
`_selection_key` presentes em `/ws/src`, e o status publicando
`frontier_extract_ms`), o `maze_explorer` OCIOSO caiu de 71,7 % para 68,5 %.
Praticamente nada — porque com o robô parado `_begin_selection` nunca roda e não
há extração a otimizar. **Os ~68 % de um explorador ocioso não são fronteira: são
o `TransformListener` dele deserializando 1090 mensagens por segundo em Python.**

O `/map` não é o culpado: mede 88 x 85 células (7480), desprezível.

Duas consequências que valem mais que qualquer coisa aplicada até aqui:

1. **`robot_state_publisher` NÃO roda no módulo** — está no host. Verificado:
   zero processos `robot_state` nos dois containers do módulo. Logo esse fluxo
   de 1090 Hz **atravessa a Ethernet** antes de ser deserializado onze vezes do
   lado do Aquila. É carga de rede e de CPU, causada por configuração do host.
2. **Nav2 não precisa de nada disso.** As arestas que a navegação usa —
   `base -> lidar`, `base -> trunk` — são juntas FIXAS, e o
   `robot_state_publisher` já as publica em `/tf_static`. O tráfego de 1090 Hz em
   `/tf` é composto das juntas das PERNAS, que só interessam a visualização.

E fecha a hipótese da seção 3 com mecanismo: `odom_tf` é um nó Python que precisa
publicar `odom -> base` a 50 Hz no meio dessa enxurrada, competindo por
escalonamento numa máquina com fila 24–26. Quando perde, a aresta fica 60–160 ms
atrás e as nuvens daquele intervalo viram inobserváveis.

**Direção de correção proposta, NÃO aplicada:** estrangular a taxa do
`joint_state_broadcaster` (ou do `robot_state_publisher`) de ~1 kHz para algo da
ordem de 50 Hz. Isso mexe na configuração do controlador de marcha, cujo
comportamento é medido, e por isso não foi feito sem decisão explícita.

## 7. Limitações desta medição

- **O robô estava parado.** Sem meta ativa não há controller em malha, e
  `nav2_container` a 298 % é o custo em REPOUSO. Sob navegação ele sobe. Pela
  mesma razão, o filtro de log (`invalid source`, `extrapolation`,
  `Failed to make progress`, `worldToMap`, `Control loop missed`) voltou vazio:
  não há progresso a falhar quando ninguém pediu meta.
- A Etapa 4 foi construída e deployada no módulo depois das rodadas acima
  (`module.sh build` + `up`, imagens verificadas sem stack de renderização), mas
  **não pôde ser medida**: a extração de fronteira só roda no estado `selecting`,
  e o robô ficou parado. O número que ela melhora — 158,6 ms -> 18,1 ms medidos
  em x86 — só aparece com a busca autônoma em curso.
- A causalidade CPU → TF é hipótese consistente com três medições
  independentes, não fato provado.
- O relógio do módulo salta; nenhuma conclusão aqui depende de `Up <tempo>` nem
  de duração vista pelo lado do módulo.
