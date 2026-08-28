# ML3.5 F5 — o bloqueio era o amostrador do MPPI, não a planta

Data: 27/08/2026. Topologia: **HIL real** — Gazebo Harmonic no host x86,
Nav2 + percepção no Aquila AM69 (`192.0.2.5`, `enp0s31f6`), `slam_toolbox`
como único autor de `map -> odom`.

Amostras cruas: `ml35-f5-slam-giro.csv` (1200 amostras, 10 Hz).
Corrida que motivou esta: `ml35-f5-slam-exploration.md`.

**A única variável entre as duas corridas são dois parâmetros do MPPI.** Mesma
rota de 19 metas, mesma bancada, mesma imagem, sem rebuild — o
`compose.module.yml` faz bind mount de `demo_navigation/config`, então bastou
`module.sh sync` + restart do container `nav`.

| | antes | depois |
| --- | ---: | ---: |
| `wz_std` | 0,08 | **0,15** |
| `iteration_count` | 1 | **2** |

---

## 1. O robô saiu do lugar

| métrica | **antes** (240 s) | **depois** (117 s) |
| --- | ---: | ---: |
| caminho percorrido | 0,06 m | **2,35 m** |
| deslocamento líquido | 0,01 m | **1,09 m** |
| velocidade média | 0,0002 m/s | **0,0201 m/s** |
| `cmd_vx` pico | 0,0008 m/s | **0,123 m/s** |
| `cmd_vx` ≈ 0 | 100,0% | **65,1%** |
| **razão de trabalho `vx`** | 0,0% | **7,3%** |
| `cmd_vx` negativo | 0% | 0% |
| tilt de pico / quedas | 0,35° / 0 | 0,91° / 0 |

Normalizando pela duração, o deslocamento líquido vai de 0,004 m/min para
0,56 m/min — **fator ~134**.

## 2. O teste decisivo: o giro comandado agora se realiza

Era este o critério declarado antes da corrida.

| | antes | depois |
| --- | ---: | ---: |
| `\|cmd_wz\|` de pico | 0,0541 rad/s | **0,2000 rad/s** |
| ... como fração de `wz_max` (0,20) | 27% | **100%** |
| giro real do robô (desenrolado) | **−2,3°** | **+56,0°** |
| erro de rumo, mínimo | 86,1° | **7,4°** |
| erro de rumo, p10 | 86,6° | **23,0°** |
| `\|erro de rumo\| < 30°` | **0,0%** das amostras | **12,4%** das amostras |

O pico de `|cmd_wz|` bate **exatamente em `wz_max`**. Antes o amostrador não
chegava nem a um terço do teto; agora encosta nele. O erro de rumo, que em
240 s nunca desceu de 86°, agora chega a 7,4°.

**A hipótese da §4 da corrida anterior está confirmada: o bloqueio era o
amostrador.** Com `wz_std: 0.08` e uma única iteração por ciclo, a média da
sequência de controle partia de ~zero e não conseguia se deslocar o suficiente
para produzir um giro executável.

## 3. CORREÇÃO à evidência anterior: não era a zona morta da planta

A §3 de `ml35-f5-slam-exploration.md` atribuiu o congelamento a o comando
"morrer na zona morta do supervisor de marcha". **Essa atribuição estava
errada em causa, ainda que a aritmética estivesse certa.**

Esta corrida rodou com `--sim-log`, que era exatamente o que faltava para medir
o supervisor em vez de inferi-lo:

```
supervisor de marcha   413 linhas, RECOVER=0, tilt_max=0.8 deg,
                       yawSat média=11% pico=64%
```

**`yawSat` médio de 11% e pico de 64% é um eixo de guinada com folga de sobra,
não um eixo em batente.** `RECOVER=0`: a marcha nunca entrou em recuperação. A
planta sempre foi capaz de girar; ela nunca recebeu uma ordem grande o
suficiente para tal.

A distinção importa porque as duas leituras levam a correções opostas: a
atribuição errada aponta para `cmd_vel_si_to_stick` e para os ganhos do
supervisor — nenhum dos quais precisava de mudança.

## 4. A previsão de custo NÃO se confirmou

Ao subir `iteration_count` de 1 para 2, registrei no YAML que isso dobra o
custo do otimizador e que abaixo de ~5 Hz o parâmetro deveria voltar para 1.
Medido nos logs do `controller_server`, a taxa do laço de controle:

| | antes (`iteration_count: 1`) | depois (`iteration_count: 2`) |
| --- | ---: | ---: |
| amostras de "loop missed" | 55 | 90 |
| taxa mínima | 4,65 Hz | 4,00 Hz |
| **taxa mediana** | **7,58 Hz** | **8,13 Hz** |
| taxa máxima | 12,05 Hz | 11,49 Hz |

**A mediana subiu.** Dobrar a iteração não degradou a taxa de forma mensurável
— o `controller_server` já perdia os 10 Hz nominais antes da mudança, e a perda
não vem do custo do otimizador. A previsão de custo foi conservadora e o dado
não a sustenta. O gatilho de reverter para 1 **não** deve ser acionado.

Isso reabre uma pergunta separada e ainda não respondida: **por que o laço de
controle não fecha em 10 Hz no AM69**, já que não é o otimizador.

## 5. O que continua falhando

**0 metas cumpridas de 4 encerradas** (falhou em t=17 s, 25 s, 66 s, 68 s).
Movimento existe, conclusão de meta não.

Nos logs do `demo-nav-1` durante a corrida: 11 `Aborting handle`, 4
`extrapolation into the future` no `map -> odom`, 1 `Failed to make progress`,
6 `Optimizer reset`. As causas nomeadas cobrem 5 dos 11 abortos; o restante não
tem causa nomeada no log.

Também: **8 trocas grandes de rota** (>1 m ou >45°) em 117 s, contra 5 em 240 s
antes — o plano está instável. Com o costmap global 95,9% desconhecido e o
`NavFn` rodando com `allow_unknown: true`, o plano atravessa espaço não
observado e se reescreve a cada varredura nova. Essa é a suspeita mais forte
para o próximo passo, e **é suspeita, não resultado**.

## 6. O que isto NÃO estabelece

Regras 5 e 7 do `CLAUDE.md`:

- Os números descrevem a pilha distribuída host+módulo. **Não** validam
  localização por pernas, um Go2 físico, térmica, nem desempenho isolado do
  AM69.
- A razão "giro realizado / giro comandado" **não** foi usada como métrica: a
  média assinada de `cmd_wz` sobre uma corrida com reversões não integra para o
  giro líquido, e a razão de 167% que ela produz não tem significado físico. O
  que sustenta a §2 é o pico de `|cmd_wz|` batendo em `wz_max` e o erro de rumo
  caindo de 86° para 7,4°.
- A instabilidade de plano da §5 não foi testada por variação controlada.

## 7. Próximo passo

O giro está resolvido; o alvo agora é a conclusão de meta. O candidato mais
barato é atacar a instabilidade do plano — o mapa vivo ainda não cobre o
espaço que o `NavFn` atravessa como desconhecido. Uma corrida com o robô
explorando primeiro (deixando o SLAM preencher) antes de perseguir metas
distantes separaria "plano instável porque o mapa está vazio" de "plano
instável por outra razão".

---

Comando desta corrida:

```
python3 scripts/nav_trial.py docs/results/ml35-f5-slam-giro.csv \
  --seconds 120 --sim-log /tmp/aquila-fix-simlog.txt \
  --goals='<as mesmas 19 metas de ml35-f5-slam-exploration.md>'
```
