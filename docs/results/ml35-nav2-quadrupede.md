# ML3.5 — Nav2 sobre o quadrúpede: desvio reativo

Estação x86, Gazebo Harmonic, `quadruped_objects.sdf`. Data: 20/08/2026.

**NÃO VALIDADO EM HARDWARE.** Todo número aqui é da estação x86. Emulação arm64
não mede desempenho (regra 5 do `CLAUDE.md`), e nada aqui foi executado no Aquila
AM69 real.

## Onde o Nav2 roda, e por quê

Medido: `ls /opt/ros/jazzy/lib | grep -c nav2` dentro de `demo-sim:spike-go2`
devolve **0**. A imagem do spike não tem Nav2. Enquanto ela for a imagem do
simulador, o Nav2 sobe **nativo no host x86** e conversa com o container pelo DDS
(`--network=host`, domínio 69, `rmw_cyclonedds_cpp`).

O host tem tudo o que a pilha precisa, incluindo o MPPI — que não aparece como
diretório em `/opt/ros/jazzy/lib` porque é biblioteca de plugin
(`libmppi_controller.so`), e não executável.

## Duas correções que precederam qualquer medição

### `use_composition` carregando nós em container inexistente

`navigation_launch.py` com composição ligada usa `LoadComposableNodes` para pôr os
servidores em `/nav2_container`, mas **não cria** esse container — quem cria é o
`bringup_launch.py`. Incluindo apenas o primeiro, os nós vão para um container que
não existe: **nada sobe e nada imprime erro**. O log do launch termina no
`odom_tf` e `navigate_to_pose` nunca aparece.

Corrigido em `nav_quadruped.launch.py` com `use_composition: 'False'`.

### `ros2 action list` como portão de prontidão

Com o grafo incompleto, `ros2 action list` **bloqueia indefinidamente** — não
devolve vazio, não expira. Um runner ficou 10 min preso. O portão correto é a
linha `Managed nodes are active` do log do `lifecycle_manager`.

## Corrida 1 — parâmetros do TB4 no verificador de progresso

Percurso: (4, 1,5) → (4, −1,5) → (0,0), ciclo, `goal_timeout_s: 120`.
Duração: 300 s de tempo de simulação, 15 030 amostras de odometria.

| grandeza | valor |
| --- | --- |
| caminho percorrido | 4,16 m |
| deslocamento líquido | 0,73 m |
| extremos | x −0,65 … +0,04 · y −0,25 … +0,28 |
| metas concluídas | **0 de 3** (todas por prazo de 120 s) |
| `Failed to make progress` | **22** |
| `mode=RECOVER` (quedas) | **0** |
| `cmd_vx` | min −0,100 · max **+0,040** (de 0,15 permitidos) |
| `cmd_wz` | min −0,118 · max +0,120 · **10 % no teto** |
| folga mínima a obstáculo | 0,862 m (nunca chegou perto) |

### O que isso diz, e o que não diz

O robô **nunca saiu da origem** e foi para trás. 39 % das amostras de `cmd_vx`
são negativas, e o máximo para frente é 0,040 m/s — um quarto do permitido. A
guinada satura no teto em 10 % do tempo e o robô gira **para longe** da meta:
rumo pedido 20,6°, guinada final 97°.

Zero quedas. O robô estava mecanicamente bem o tempo todo. O que falhou foi o
comando.

### Diagnóstico do `Failed to make progress`: sintoma, não causa

O verificador do TB4 exige 0,5 m em 10 s. O Go2 gira a 0,12 rad/s (saturação do
momento de guinada na QP de equilíbrio), então reorientar 90° consome 13 s com
avanço quase nulo. Exigir 0,5 m em 10 s reprova manobra que a física do robô
proíbe.

Corrigido para 0,20 m em 40 s. **Mas isso não explica `cmd_vx` ≈ 0**: o aborto
de progresso interrompe a meta, não o comando de velocidade. A causa do comando
nulo é outra e está registrada abaixo.

### Diagnóstico do comando nulo: horizonte medido em tempo, não em distância

Antes de culpar o costmap, ele foi medido. Com o robô parado no mundo dos
objetos, `/local_costmap/costmap` (120 × 120, 6 × 6 m):

| grandeza | valor |
| --- | --- |
| custo na célula do robô | **0** |
| custo máximo dentro de 0,6 m | **0** |
| células letais dentro de 0,6 m | **0** |
| células letais no mapa | 42 (os obstáculos) |
| células desconhecidas | 0 |
| nuvem: pontos acima de 0,12 m | 249, o mais próximo a **1,31 m** |

**O costmap está correto.** A hipótese de "robô dentro de região inflada" está
refutada por medição. Também vale registrar o corte de altura funcionando: dos
2097 pontos da nuvem, 1848 estão abaixo de 0,12 m (chão) e são descartados.

O que sobra é o horizonte. `time_steps: 56 × model_dt: 0.05` = 2,8 s. No TB4, a
0,5 m/s, isso cobre 1,4 m; no Go2, a 0,15 m/s, cobre **0,42 m**. E 0,42 m não
alcança a referência dos próprios críticos: o `PathAlignCritic` tem peso 14 — o
maior de todos — e busca referência `offset_from_furthest: 20` pontos de caminho
à frente, cerca de 1 m. O crítico dominante não chega na própria referência.

**Horizonte se mede em distância.** Copiá-lo em tempo para um robô 3× mais lento
dá 30 % da antecipação.

## Corrida 2 — horizonte 96 × 0,1 s (1,44 m), laço 10 Hz, progresso 0,20 m/40 s

| grandeza | corrida 1 | corrida 2 |
| --- | --- | --- |
| caminho percorrido | 4,16 m | 5,21 m |
| deslocamento líquido | 0,73 m | 0,74 m |
| extremos em x | −0,65 … +0,04 (**para trás**) | −0,04 … **+0,98** |
| `cmd_vx` máximo | +0,040 | +0,058 |
| amostras de `cmd_vx` negativas | 39 % | **16 %** |
| `Failed to make progress` | 22 | **3** |
| `missed its desired rate` | 13 | **1** |
| quedas | 0 | 0 |
| folga mínima a obstáculo | 0,862 m | **0,275 m** |

O sinal trocou. O robô avança monotonicamente rumo à meta — x de 0,04 a 0,98, y de
0,00 a 0,70, com o rumo convergindo para os 20,6° do caminho — passa a 0,275 m da
caixa vermelha **sem colidir**, e depois executa a meta (0,0) andando **de ré**:
1 m de ré é mais barato que 180° de giro à taxa de guinada disponível.

A malha fecha: nuvem → costmap → planejador → MPPI → marcha → odometria → TF →
costmap.

E ainda assim **nenhuma meta foi concluída**, todas por prazo. Motivo abaixo.

## A causa de fundo: `/demo/cmd_vel` não está em SI

`/demo/cmd_vel` é um `geometry_msgs/Twist` que carrega **posição normalizada de
manche**, não m/s. A prova é aritmética, na fonte vendorizada:

```
StateTrotting.cpp:192   v_cmd = invNormalize(ly, -0.4, +0.4)
mathTools.h:10          invNormalize(v, min, max) = 0.4 * v    (minLim=-1, maxLim=1)
twist_to_inputs.py:283  ly = linear.x                          (ganho UNITÁRIO)
```

`linear.x` chega ao robô multiplicado por **0,4**; `angular.z`, por **0,5**.

O projeto sempre soube disso e trabalha assim: `demo_routine.to_twist` divide pelo
ganho antes de publicar, e `ml35-f4-parcial.md` registra "comando
`linear.x = 0.25` (→ `v_cmd = 0,1 m/s`)".

**O Nav2 não pode trabalhar assim**, e essa é a diferença que importa. Ele não é
só um publicador de velocidade: o MPPI **integra** `vx` como m/s para prever onde
o robô estará. Publicando manche, todo rollout erra a distância por 2,5×, e o
horizonte calibrado em 1,44 m vale 0,58 m na planta.

Em números da corrida 2: `cmd_vx` médio 0,0141 de manche = **0,0056 m/s reais**.
Isso é 18× menor que o ponto validado de 0,10 m/s e está **abaixo** do mínimo de
~0,05 m/s em que a marcha se sustenta. A 0,0056 m/s, 4,27 m levam 12,7 min e
nenhum prazo de 120 s pode ser cumprido.

### Correção: um nó de fronteira, não uma mudança de planta

```
Nav2 (collision_monitor) --/demo/cmd_vel_si--> cmd_vel_si_to_stick --/demo/cmd_vel--> planta
                                SI                                      manche
```

`demo_bringup/cmd_vel_si_to_stick` divide pelos ganhos do controlador. O contrato
de `/demo/cmd_vel` **não muda**, a planta **não é tocada**, `demo_routine` e
`gait_trial.sh` seguem iguais, e todo número já gravado em `docs/results/`
continua significando o que significava.

A alternativa — fazer `twist_to_inputs` aceitar SI — deixaria `/demo/cmd_vel`
honesto e foi rejeitada por escopo: mudaria a planta, os dois comandantes
existentes e o significado de cada `--v-cmd` já registrado.

Armadilha que a correção cria: passam a existir dois tópicos `Twist` com unidades
diferentes. Ligar o Nav2 de volta em `/demo/cmd_vel` faz o robô andar a 40 % do
pedido **sem erro nenhum em log** — o sintoma é a corrida 2.

## Corrida 3 — fronteira de unidades corrigida

| grandeza | corrida 1 | corrida 2 | corrida 3 |
| --- | --- | --- | --- |
| caminho percorrido | 4,16 m | 5,21 m | **8,36 m** |
| deslocamento líquido | 0,73 m | 0,74 m | **3,51 m** |
| extremos em x | −0,65 … +0,04 | −0,04 … +0,98 | 0,03 … **+4,24** |
| `cmd_vx` máximo, em SI | 0,016 m/s | 0,023 m/s | **0,119 m/s** de 0,15 |
| `cmd_vx` médio, em SI | 0,006 m/s | 0,006 m/s | **0,021 m/s** |
| amostras negativas | 39 % | 16 % | **9 %** |
| `Failed to make progress` | 22 | 3 | **2** |
| comprimento do plano final | 0,69 m | 0,75 m | **4,96 m** |
| quedas | 0 | 0 | 0 |

Folga a cada obstáculo, mínimo ao longo de toda a trajetória:

| obstáculo | folga | veredito |
| --- | --- | --- |
| caixa vermelha | 0,192 m | passa |
| cilindro verde | **0,014 m** | raspou, sem colidir |
| caixa azul | 0,237 m | passa |
| cilindro amarelo | 0,203 m | passa |

O robô chegou a **(3,976 · 1,470)**, ou seja **3,8 cm** da meta 1 em (4,0 · 1,5),
depois de passar pelos quatro obstáculos com folga positiva. A malha está fechada
e funciona.

### Uma hipótese minha refutada de caminho

Antes desta corrida eu suspeitava que o limitador da magnitude fosse a dispersão
de amostragem do MPPI, que eu havia reduzido (`vx_std` 0,2 → 0,06,
`wz_std` 0,4 → 0,05) escalando pela velocidade máxima do robô, sem evidência. O
mecanismo era plausível: amostras que terminam no mesmo lugar dão custos iguais,
pesos softmax uniformes, e média ponderada de ruído de média zero é zero.

**A medição refutou isso.** Só a correção de unidades levou `cmd_vx` de 0,006 para
0,119 m/s — 79 % do teto — com os mesmos desvios de amostragem. Os desvios ficaram
como estavam. O limitador era a planta entregando 0,4× do pedido: a velocidade
alcançada nunca confirmava a comandada, e a sequência de controle do MPPI, que é
reaproveitada de um ciclo para o outro, era corrigida para baixo a cada passo.

## O defeito seguinte: orientação final custa mais que a aproximação inteira

Nenhuma meta fechou nem na corrida 3, e não por falta de chegar. O `yaw` de cada
meta apontava para a meta **seguinte**, então na chegada o Nav2 pedia giro parado:

| meta | chega a | yaw pedido | giro parado | custo a 0,12 rad/s |
| --- | --- | --- | --- | --- |
| (4,0 · 1,5) | +20,6° | −90,0° | 110,6° | 16,1 s |
| (4,0 · −1,5) | −90,0° | +159,4° | 110,6° | 16,1 s |
| (0,0 · 0,0) | +159,4° | +20,6° | 138,9° | 20,2 s |

E **girar parado não fica parado**: entre t = 120 s e t = 216 s o robô girou de
−12,8° para −114° e no processo derivou de (3,976 · 1,470) para (4,167 · 0,693) —
**0,78 m em y**. Saiu da tolerância de posição que já havia satisfeito. A meta
reprovou por prazo depois de ter chegado.

Duas correções, o mesmo defeito:

1. O `yaw` de cada meta passa a ser o rumo de **chegada** — a direção em que o
   robô já vem andando. A orientação fica satisfeita quando a posição fica, e o
   giro para a meta seguinte acontece **andando**, que é onde o Go2 gira melhor.
2. `yaw_goal_tolerance` de 0,25 para 0,5 rad. Numa patrulha a orientação final não
   tem propósito; 28,6° ainda é restrição de rumo, mas não paga giro parado.

## Corrida 4 — metas com rumo de chegada, 420 s

**A primeira meta concluída.** `meta 3 cumprida (1 de 3)`, e o ciclo seguiu para a
meta 4. O caminho de conclusão de meta funciona de ponta a ponta.

| grandeza | corrida 3 (300 s) | corrida 4 (420 s) |
| --- | --- | --- |
| metas concluídas | 0 | **1** |
| caminho percorrido | 8,36 m | 10,87 m |
| deslocamento líquido | 3,51 m | **0,60 m** |
| extremo em x | +4,24 | +2,05 |
| `cmd_vx` máx / médio, SI | 0,119 / 0,021 | 0,119 / **0,0098** |
| amostras negativas | 9 % | 24 % |
| folga mínima | 0,014 m | **0,159 m** |
| quedas | 0 | 0 |

### O que esta corrida NÃO estabelece

**Não estabelece que a mudança melhorou nada.** As linhas de deslocamento e
velocidade média pioraram, e a diferença **não é atribuível**: o MPPI é um
amostrador estocástico com `regenerate_noises: true`, há **n = 1 por condição**, e
o traçado mostra o robô oscilando entre x = 0,33 e 0,65 durante os primeiros 200 s
com o rumo virado para o lado errado (−13° quando a meta está a +20,6°). Isso é
variação entre execuções, não efeito medido.

Para atribuir, seria preciso repetição por condição — o mesmo critério que a Fase B
aplicou ao Defeito 2 e que este resultado ainda não cumpre.

O que a corrida 4 estabelece, e é o que importa para o pedido:

- **movimento contínuo por 420 s**, atravessando duas falhas de meta e uma
  conclusão sem parar o ciclo;
- **zero colisões** nos quatro obstáculos, folga mínima 0,159 m;
- **zero quedas**.

### Prazo por meta, recalculado por aritmética

Os 120 s usados nas corridas 1–4 são curtos por construção. À velocidade média
medida de 0,021 m/s, a perna mais longa (4,27 m) leva **203 s** em linha reta, e
264 s com fator 1,3 de desvio. `DEFAULT_GOAL_TIMEOUT_S` passou para **300 s**.

A média (0,021 m/s) é muito menor que o pico (0,119 m/s) porque o MPPI passa boa
parte do tempo corrigindo rumo, e a 0,12 rad/s de teto de guinada corrigir rumo
custa tempo em que quase não se avança.

**O efeito desse prazo na taxa de conclusão de metas não foi medido.** É aritmética
a partir de dado medido, não um resultado.

## O que estas corridas NÃO estabelecem

- Não estabelecem que o desvio funciona. O robô nunca chegou perto de obstáculo.
- Não estabelecem nada sobre localização: `/demo/odom` é ground truth do Gazebo.
- Não estabelecem nada sobre percepção: `demo_perception` continua stub.
- Não estabelecem nada sobre o Aquila AM69.
