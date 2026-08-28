# ML3.5 F5 — a rota de exploração de 19 metas: o MPPI manda girar, a planta não gira

Data: 27/08/2026. Topologia: **HIL real** — Gazebo Harmonic no host x86,
Nav2 + percepção no Aquila AM69 (`192.0.2.5`, `enp0s31f6`, RTT 0,170 ms),
`bt_navigator` em `active`, `slam_toolbox` como único autor de `map -> odom`.

Amostras cruas: `ml35-f5-slam-exploration.csv` (2400 amostras, 10 Hz).
Referência fria da mesma bancada: `ml35-f5-slam-cold.csv`.

**Veredito: 0 metas cumpridas de 7 encerradas. O robô andou 6 cm em 240 s.**

> **CORREÇÃO (27/08/2026, mesma data).** A §4 deste documento era hipótese e
> foi **confirmada** por `ml35-f5-slam-giro.md`: o bloqueio era o amostrador do
> MPPI (`wz_std` / `iteration_count`). Já a atribuição da §3 à **zona morta do
> supervisor de marcha está errada em causa** — medido com `--sim-log` na
> corrida seguinte, `yawSat` fica em 11% de média e 64% de pico, com
> `RECOVER=0`. O eixo de guinada sempre teve folga; ele nunca recebeu ordem
> grande o bastante. Leia a §3 abaixo como "o comando era pequeno demais para
> ser executado", não como "a planta tem uma zona morta que o rejeita".

---

## 1. O que a rota mediu

Rota conectada de 19 metas, saltos de ~1,4 m, `--seconds 240`. Reset de
simulação (`success=True`, robô reposto em x=0, y=0, yaw=90°) e reset de
navegação (`success=True`, costmaps limpos, servidores reativados)
imediatamente antes. Nada foi reconstruído entre a referência fria e esta.

| métrica | **exploração 19 metas** | **referência fria** (meta única 0,00; 8,00) |
| --- | ---: | ---: |
| duração (tempo de simulação) | 239,9 s | 119,9 s |
| fator de tempo real | 0,997 | 0,957 |
| caminho percorrido | **0,06 m** | 0,82 m |
| deslocamento líquido | **0,01 m** | 0,17 m |
| **razão de trabalho `vx`** (> 0,05 m/s) | **0,0%** | 0,0% |
| `cmd_vx` ≈ 0 (≤ 0,005 m/s) | **100,0%** | 97,2% |
| `cmd_vx` pico | **0,0008 m/s** | 0,019 m/s |
| `cmd_wz` pico | 0,054 rad/s | 0,111 rad/s |
| metas | **0 de 7** (falhou=6, prazo=1) | 0 de 5 (falhou=5) |
| tilt de pico / quedas | 0,35° / 0 | 1,22° / 0 |
| folga de carcaça | +0,153 m | +0,065 m |

Trocar meta única distante por 19 metas conectadas **não melhorou nada** — piorou.
A hipótese que motivou a rota (metas curtas encadeadas destravam o deslocamento)
está refutada para este estado da pilha: o bloqueio é anterior à geometria da meta.

## 2. O bloqueio não é o planner, e não é o costmap

Ambos estão saudáveis, e isso foi medido, não suposto:

- **Plano existe em 99,0% das amostras** (2377 de 2400), comprimento 1,77–8,56 m,
  idade mediana 0,50 s, máxima 1,97 s. O `planner_server` nunca ficou sem
  resposta.
- **A geometria do costmap global está correta**, confirmando a correção desta
  sessão: `400x400 @ 0,050 m` = **20,0 x 20,0 m**, origem `(-9,75; -10,00)`
  acompanhando o robô, `rolling_window: true`. **Zero ocorrências de
  `worldToMap failed`.**
- **O costmap local está limpo**: 6,0 x 6,0 m, 74,0% livre, 18,0% letal,
  8,1% inflado, **0,0% desconhecido**.

O costmap global está 95,9% desconhecido e 0,0% livre, mas isso é *consequência*
do robô não ter saído do lugar, não causa: o `NavFn` roda com
`allow_unknown: true` e produziu plano o tempo todo.

## 3. A causa: o comando de guinada integra +184,7°, e o robô gira −2,4°

Este é o número que fecha o caso.

| | valor |
| --- | ---: |
| `cmd_wz` médio assinado | **+0,01344 rad/s** |
| `cmd_wz` > 0 | **91,1%** das amostras |
| `cmd_wz` < 0 | 4,4% das amostras |
| **giro integrado do comando** (240 s) | **+184,7°** |
| **giro real do robô** | **−2,4°** (yaw 90,0° → 87,6°) |

O MPPI **não está inerte e não está confuso de direção**. Ele comanda guinada
positiva de forma sustentada e consistente — e positiva é a direção *certa*: o
robô nasce em yaw 90° (apontando +Y) e a primeira meta (−1,50; 0,05) está a
oeste, com o plano apontando ~178°. Girar +88° é exatamente a manobra correta.

O que falha é a **magnitude**: 0,0134 rad/s médio contra um `wz_max` de
0,20 rad/s é **6,7% do teto**. Esse comando atravessa
`cmd_vel_si_to_stick`, que divide pelo ganho (`WZ_PER_STICK = 0.5`), e chega ao
supervisor de marcha como **0,027 unidades de manche** — 5,4% do `STICK_CLAMP`
de 0,5. O robô recebe a ordem de girar e não gira.

A partir daí o laço trava sozinho:

```
erro de rumo ~88°  ->  MPPI comanda +0,013 rad/s  ->  abaixo da zona morta
        ^                                                     |
        |                                                     v
   erro nunca cai  <-  robô não gira  <-  planta ignora o comando
```

E com o rumo travado a ~88°, `vx` **tem** de ficar em zero: os critics de
alinhamento de caminho não premiam avançar perpendicular ao plano, e
`vx_min: 0.0` proíbe ré. O congelamento total é o comportamento correto do
MPPI dado um giro que não acontece.

**O erro de rumo confirma isso: mediana 88,2°, p90 92,2°, e `|erro| < 30°` em
0,0% das 2377 amostras com plano.** Em 240 s ele nunca chegou perto de resolver.

A referência fria mostra o mesmo defeito, com outra meta: erro de rumo mediano
92,4°, `|wz|` médio 0,021 rad/s. **Não é específico da rota de exploração.**

## 4. Por que a magnitude é pequena: `wz_std` contra `iteration_count: 1`

Hipótese, com o parâmetro identificado mas **ainda não testada por variação
controlada**:

`nav2_params_go2.yaml` traz `wz_std: 0.08`, `wz_max: 0.20` e
`iteration_count: 1`. Com uma única iteração de otimização por ciclo, a
sequência de controle parte do resultado anterior (~zero) e só pode se deslocar
uma fração de um desvio-padrão por ciclo. O comentário do próprio arquivo já
antecipa o mecanismo para o valor antigo:

> `0.08, subido de 0.05 junto com wz_max abaixo. Rotacao so e alternativa real a
> re se o amostrador de fato a explorar: com wz_std 0.05 contra um wz_max de
> 0.20, a nuvem de amostras quase nao alcanca o teto novo.`

O aumento de 0,05 para 0,08 foi na direção certa e **não foi suficiente**: o
medido agora é 6,7% do teto. As duas alavancas óbvias — subir `iteration_count`
e subir `wz_std` — não foram exercitadas nesta corrida.

## 5. Achado secundário: `map -> odom` chega adiantado no módulo

Nos logs do `demo-nav-1`, 5 ocorrências de:

```
Exception in transformPose: Lookup would require extrapolation into the future.
Requested time 10094.920000 but the latest data is at time 10094.900000,
when looking up transform from frame [map] to frame [odom]
```

Deltas medidos: 0,020 s (3x), 0,060 s, 0,080 s. Mais 1x `Failed to make
progress` e 8x `Optimizer reset`. O `controller_server` também perdeu taxa
repetidamente (10 Hz nominal, 6,25–9,35 Hz medido).

Isso aborta metas mas **não explica o congelamento** — só 5 eventos em 240 s, e
a meta 2 correu 90 s ininterruptos com `cmd_vx` em zero o tempo inteiro. É um
defeito real e separado, do `slam_toolbox` publicando `map -> odom` no host
enquanto o `controller_server` consome no módulo.

## 6. O que isto NÃO estabelece

Conforme as regras 5 e 7 do `CLAUDE.md`:

- Os números descrevem a pilha distribuída host+módulo. **Não** validam
  localização por pernas, um Go2 físico, térmica, nem desempenho isolado do
  AM69.
- A zona morta do supervisor de marcha é **inferida** da razão comando-integrado
  vs. giro-real. Não foi medida diretamente: esta corrida rodou sem `--sim-log`,
  então as estatísticas do supervisor (`mode`, tilt, `yawSat`) não existem.
- A relação `wz_std`/`iteration_count` da §4 é hipótese com parâmetro
  identificado, não resultado de A/B.

## 7. Próximo passo mais barato

Uma corrida de 120 s com `iteration_count: 2` e `wz_std: 0.15`, mesma rota,
mesma bancada, com `--sim-log` ligado para capturar `yawSat` e confirmar ou
derrubar a zona morta. Se o giro integrado passar a se realizar no yaw real, a
§3 vira causa confirmada e a §4 vira correção.

---

Comando desta corrida:

```
python3 scripts/nav_trial.py docs/results/ml35-f5-slam-exploration.csv \
  --seconds 240 \
  --goals='-1.50,0.05;-2.90,0.10;-3.30,1.40;-1.90,1.75;-1.70,2.95;-1.65,4.50;
           -1.70,5.75;-1.70,7.15;-2.60,8.05;-3.30,7.05;-3.30,5.65;-3.30,4.25;
           -3.75,3.30;-5.35,3.25;-6.45,2.90;-6.50,1.35;-6.25,0.15;-4.95,-0.20;
           -4.90,-0.90'
```
