# ML3.5 F5 — o piso ocioso de CPU do modulo, e o que ele custava

**Estado:** EXECUTADO — **portao de 8 m ainda REPROVADO**
**Data:** 25/08/2026 (madrugada de 26/08 no relogio do modulo)
**Baseline do modulo:** Aquila AM69 V1.0A, Torizon OS 7.7.0, 8 x Cortex-A72

Evidencia colhida no AM69 real. O robo e os sensores seguem simulados no host,
entao **nada aqui valida marcha real, um Go2 fisico, termica ou consumo**
(regras 5 e 7 do `CLAUDE.md`). Nao houve medicao de temperatura nem de potencia.

## Resultado executivo

Duas coisas, e elas nao vao na mesma direcao.

**O que foi resolvido:** o mecanismo que custava 2,8x de velocidade media desde
24/08 — saturacao de CPU, defasagem de sensor, `collision_monitor` recusando a
fonte e parando o robo — **desapareceu**. Recusas cairam de 16 (e de 5 na melhor
corrida posterior) para **zero**, descartes de costmap para zero, e a maquina
passou a navegar com **307% de 800% ociosos**.

**O que nao foi resolvido:** o robo nao anda melhor. Com CPU sobrando e sem uma
unica recusa de sensor, ele continua com `vx` em zero em **90,7%** das amostras e
girando em **90,3%** delas, e as duas metas de 8 m estouraram o prazo de novo.

O valor disto nao e o ganho de velocidade, que nao houve. E que **os dois limites
que estavam confundidos agora estao separados**: o de CPU esta fechado e medido, e
o de decisao de trajeto ficou sozinho em cena, sem desculpa de maquina cheia.

## Como o custo foi encontrado

O A/B de amostragem do MPPI (`docs/results/ml35-f5-mppi-amostragem.md`) tinha
refutado a hipotese de que a CPU do Nav2 estava no laco do controlador, e
terminava mandando **perfilar antes de sintonizar**. Foi o que se fez.

Amostragem de `/proc/<tid>/stat` por thread, sobre todos os processos do cgroup
do container `nav`, janela de ~20 s, **pilha de pe e SEM META ATIVA**:

```text
piso ocioso total          367% de 800%
  component_container      215%   (Nav2 composto, ~30 threads a ~7% cada)
  odom_tf                   40%   republicador em Python
  recvUC                    38%   thread de recepcao do CycloneDDS
  cmd_vel_si_to_stick       36%   republicador em Python
  nav_control_relay         35%   relay de servicos
```

**367% de 800% com o robo parado.** E 111% disso eram tres republicadores em
Python cujo trabalho util cabe em ~1%.

O `component_container` estar espalhado em ~30 threads a ~7% cada, em vez de ter
uma thread quente, ja dizia que o custo nao era algoritmico: e entrega de
mensagem.

## A causa, que ja estava documentada em outro lugar

O cabecalho de `demo_simulation/clock_throttle.py` mediu isto em 21/08:
`use_sim_time: true` faz o **rclpy** criar uma assinatura de `/clock` por no,
independentemente de o no chamar o relogio. O Gazebo publica `/clock` a ~870 Hz
porque o passo de fisica da marcha e 1 ms.

O que faltava era perguntar quais nos **precisavam** dessa assinatura. Por
inspecao de codigo (`ast`, nao leitura casual):

| no | chamadas a `get_clock()` | veredito |
| --- | --- | --- |
| `cmd_vel_si_to_stick` | nenhuma | converte Twist em Twist. Nao tem header nem timer. |
| `nav_control_relay` | nenhuma | os timeouts dele usam `time.monotonic()` de proposito — ver o docstring de `_wait`. |
| `odom_tf` | uma, em `_identity()` | carimba a aresta ESTATICA `map -> odom`. O buffer estatico do tf2 devolve transformada estatica para qualquer instante: esse stamp nao entra em lookup. O caminho quente (`_on_odom`) copia o stamp da mensagem. |

Os tres assinavam `/clock` a ~870 Hz para nao usar nenhuma mensagem.

**Isto NAO e o estrangulamento de `/clock` reprovado em 21/08.** La a taxa caia
para todo mundo, o MPPI incluso, e a navegacao morreu (0,0039 m/s contra
0,0251 m/s). Aqui a taxa nao muda para ninguem: muda quem assina.

## A mudanca

`use_sim_time: False` nos tres, em `nav_quadruped.launch.py` e
`nav_control.launch.py`. Vale para os dois robos — `nav_control.launch.py` e
compartilhado — e nao ha caminho por modo.

Junto saiu o argumento `use_sim_time` de `nav_control.launch.py`, e as duas
chamadas que o passavam. Argumento declarado que ninguem consome e falha
silenciosa: o valor seria aceito e ignorado sem uma linha de log. Removido, a
mesma chamada falha alto.

Quatro testes estruturais em `demo_bringup/test/test_sim_time_scope.py`, todos
verificados por mutacao. O que importa e o segundo, que trava o sentido inverso:
**um no que sobe sem `use_sim_time` nao pode chamar `get_clock()`** — senao
alguem acrescenta um timer, o no passa a ler tempo de PAREDE achando que le tempo
simulado, e os stamps saem anos no futuro sem que nada acuse.

## Medicao depois

**Os tres nos, mesmo metodo, mesma janela:**

| no | antes | depois |
| --- | ---: | ---: |
| `cmd_vel_si_to_stick` | 36% | **0%** |
| `nav_control_relay` | 35% | **0%** |
| `odom_tf` | 40% | **17,7%** |
| **soma** | **111%** | **17,7%** |

Os 17,7% que sobram no `odom_tf` sao o trabalho real dele: TF a partir de
`/demo/odom` a ~50 Hz.

Confirmado tambem de fora, sem inferencia: `ros2 topic info /clock -v` lista 37
assinantes e **nenhum dos tres esta entre eles**.

**Sob navegacao, cgroup (`cpu.stat usage_usec`), janela de 62 s:**

```text
maquina OCUPADA   493% de 800%   -> 307% OCIOSOS
  demo-nav-1        336%
  demo-perception-1 149%
```

Antes desta mudanca, `nav` sozinho media 600-727% **com a percepcao parada**, e
com a percepcao no ar a soma passava da capacidade da maquina.

### O que esta medicao NAO prova

O total ocioso do container `nav` **nao caiu**: 367% antes, 377% depois, pelo
mesmo metodo. O `component_container` cresceu de 215% para ~317%, ou seja
absorveu quase exatamente o que os tres nos devolveram.

A leitura honesta e que **a capacidade liberada foi para o Nav2**, nao que o
consumo total tenha diminuido. Em maquina saturada, porcentagem por processo
mede o que o processo **conseguiu**, nao o que ele queria — e o Nav2 estava
faminto. Quem quiser afirmar reducao liquida de consumo precisa de outra
medicao.

Duas ressalvas a mais, para nao inflar o resultado:

- as duas amostras ociosas nao estao na mesma condicao de simulacao (RTF 0,944
  contra 0,757; `/clock` no modulo a ~616 Hz contra 580-620 Hz), entao a
  comparacao de piso ocioso carrega esse confundidor;
- o numero de 600-727% de 24/08 veio de outro instrumento. A comparacao
  defensavel e a dos tres nos, que usa o mesmo metodo dos dois lados.

## A corrida de confirmacao

Protocolo padrao: `nav_trial.py --seconds 420 --goal-timeout 200`, metas do
`maze11`, camera comprimida, host com `sim`/`viz`/`cockpit`/`hmi`, modulo com
`nav`/`perception`.

| medida | valor |
| --- | ---: |
| fator de tempo real | 0,757 |
| caminho percorrido | 10,31 m |
| deslocamento liquido | 1,11 m |
| **velocidade media** | **0,0246 m/s** |
| `cmd_vx` pico / medio | 0,125 / 0,0020 m/s |
| tilt pico | 20,21 deg |
| quedas | **0** |
| folga minima de carcaca | +0,065 m |
| **metas de 8 m** | **0 de 2 — prazo em 202 s e 402 s** |

Amostras em `ml35-f5-simtime-run1.csv`.

**Mecanismo, que e onde a mudanca aparece:**

| ocorrencia no log do `nav` | ethernet0 (24/08) | esta corrida |
| --- | ---: | ---: |
| `Ignoring the source` | 16 | **0** |
| `Robot to stop due to invalid source` | 4 | **0** |
| `message filter dropping message` | — | **0** |
| `missed its desired rate` | — | 1 |

E a TF ficou integra apesar de o `odom_tf` nao seguir mais `/clock`:
`tf2_echo map base` resolve em tempo simulado, e zero descartes de costmap.

**Movimento, com limiar unico aplicado aos tres CSV para serem comparaveis:**

| corrida | `vx` ~ 0 | razao de trabalho | girando |
| --- | ---: | ---: | ---: |
| ethernet0 (RAW, 24/08) | 83,8% | 5,9% | 93,9% |
| comprimido run2 | 90,3% | 0,6% | 72,2% |
| **esta (sem `/clock` nos tres)** | **90,7%** | **0,6%** | **90,3%** |

Nao ha ganho de movimento. Esta corrida esta dentro da faixa de ruido ja
estabelecida (0,0109 a 0,0353 m/s em configuracoes iguais ou proximas), e a
razao de trabalho empata com a corrida anterior.

## Uma hipotese testada e descartada no caminho

Corredor de 1,20 m com `inflation_radius` 0,55 deixaria so 10 cm de faixa
central sem custo de inflacao, o que tornaria girar no lugar mais barato que
avancar. Medido no `local_costmap` durante a navegacao: **64,1% das celulas com
custo 0**, e o que estava a frente do robo era parede real (custo 99, inscrito)
de 0,4 m a 1,4 m, nao penalidade de inflacao. **Hipotese descartada** — o robo
girava porque estava de frente para um obstaculo naquele instante.

Uma amostra nao descreve a distribuicao. O que ela faz e impedir que a inflacao
vire a proxima sintonia sem evidencia.

## Veredito do portao

**F5 permanece aberto.** O portao exige n=3 com ao menos uma meta de 8 m
`SUCCEEDED` por corrida; esta corrida cumpriu zero de duas.

O que **passou a estar comprovado** e nao estava:

- o piso ocioso de CPU do modulo era 367% de 800%, e 111% dele era desperdicio
  puro — entrega de `/clock` a nos que nao leem relogio;
- os tres republicadores devolveram 93% de tempo de nucleo, verificado por dois
  metodos independentes (perfil por thread e lista de assinantes de `/clock`);
- a defasagem de sensor e as recusas do `collision_monitor` **acabaram**;
- a pilha navega com 307% de 800% ociosos, com percepcao no ar;
- e, o mais importante para o que vem depois: **com CPU sobrando e zero recusas,
  o robo continua girando em vez de transladar**. O limite de trajeto nao era
  efeito colateral da CPU.

## Proximo portao

O que mudou de ordem: sintonizar controlador agora e ensaio limpo, porque o
confundidor de CPU saiu. O que **nao** mudou:

1. **Consertar o protocolo antes de sintonizar.** n >= 3 por condicao,
   intercalado, mediana e faixa. A dispersao medida de 2,4x em configuracao
   identica continua valendo, e nenhuma corrida unica decide nada — esta
   incluida.
2. **Usar razao de trabalho e `vx` ~ 0 como metrica primaria**, nao velocidade
   media. Sao muito menos ruidosas (0,6% / 0,6% em duas corridas diferentes,
   contra 2,4x de dispersao na velocidade) e medem diretamente o sintoma
   "gira em vez de transladar".
3. **So entao MPPI** (`PathAlignCritic` 14,0 contra `PathAngleCritic` 2,0), com
   o costmap medido a cada condicao e nao suposto.
