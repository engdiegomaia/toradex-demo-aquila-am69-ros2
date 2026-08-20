# ML3.5 F4 — ajuste de postura na parada (Defeito 2)

**Data:** 20/08/2026 · **Máquina:** host x86 (Gazebo Harmonic + `gz_quadruped_hardware`)
**Escopo:** simulação. Nada aqui foi medido no Aquila AM69 real.

## O sintoma

Depois de qualquer trecho de caminhada, uma parada longa terminava com o robô
dobrando as pernas e caindo. O tempo até o colapso escalava com o resíduo de
velocidade no instante da parada (`velErrXY`), o que já estava registrado como
**Defeito 2** em `docs/guides/go2-testes.md` §7.4:

| `velErrXY` na entrada do HOLD | tempo até cair |
| --- | --- |
| 0,010 | não caiu em 90 s |
| 0,020 | 46,1 s |
| 0,034 | 17,8 s |

Depois que `foot_placement.k_yaw` endireitou a caminhada
(`docs/results/ml35-caminhada-reta.md`), o resíduo típico caiu e o gatilho de
90 s deixou de ser confiável. Os ensaios abaixo usam um gatilho mais duro:
15 s de caminhada a 0,15 m/s com 0,10 rad/s, e então **180 s de parada**.

## Causa

`StateTrotting::captureBodyReference()` congela `pcd_` — a referência de posição
do corpo que a QP de equilíbrio persegue — na posição do corpo do instante em
que a parada começa. Os pés continuam escorregando por algumas centenas de ms
depois disso.

A referência congelada fica então deslocada do centroide de apoio pelo resíduo
que a parada carregava. A QP passa a receber um pedido de distribuição de força
que nenhum conjunto de contatos consegue entregar; o resíduo de momento de
guinada satura; a distribuição de força degrada; o robô dobra.

A saturação é observável: `yawSat=100%` em toda a janela de parada dos ensaios
que caíram.

## Duas intervenções, medidas separadamente

Ambas entraram no código com o **default compilado igual ao comportamento
antigo**, para que um ensaio sem YAML reproduza a linha de base gravada:

- `hold.weight_moment_yaw` (`StateTrotting::applyHoldYawWeight`) — agenda o peso
  `S_(5,5)` da QP por modo, via o novo `BalanceCtrl::setYawMomentWeight`.
  Default 450 = o mesmo de `balance.weight_moment[2]`, ou seja, no-op.
- `hold.settle_rate` (`StateTrotting::settleHoldPosture`) — caminha `pcd_` (x,y)
  para a média das posições dos pés, com limite de taxa. Default 0 = no-op.

O limite de taxa não é enfeite: um salto em `pcd_` pede um degrau de força à QP
através de `Kpp = 70` contra o clamp de aceleração de 3 m/s².

`settleHoldPosture()` roda **só em HOLD**, não em RECOVER, que reancora `pcd_`
todo tick por projeto. E ela move apenas `pcd_`, nunca reancora a posição do
corpo — senão o robô caminha sozinho.

## Evidência

Gatilho: `--v-cmd 0.15 --w-cmd 0.10 --cycles 1 --walk 15 --hold 0 --final-hold 180`.
Uma corrida por condição, uma variável por corrida.

| cond | `Syaw` no HOLD | `settle_rate` | parada de 180 s | RECOVER | `yawSat` (últimas janelas) | tilt de pico na caminhada |
| --- | --- | --- | --- | --- | --- | --- |
| `p0_hold_base` | 450 | 0 | **colapso em 161,7 s** | 74 | 100% | 1,19° |
| `p1_syaw100` | 100 | 0 | **colapso em 91,1 s** | 357 | 100% | 2,24° |
| `p2_settle` | 450 | 0,020 | **180 s de pé** | 0 | 10–66% | 1,38° |
| `p3_ambos` | 100 | 0,020 | 180 s de pé | 0 | 54–90% | 0,81° |

### O resultado negativo importa mais que o positivo

Baixar o peso de momento de guinada na parada é a correção óbvia, e é a errada.
`p1` antecipou o colapso em 70 s e multiplicou o RECOVER por quase cinco.
Amolecer o resíduo não remove o pedido de momento — remove o motivo de a QP
distribuir força contra ele. **Não repetir isso sozinho.**

### Por que `p2` e não `p3`

`p3` também sobrevive, mas com a saturação de guinada de volta em 54–90% contra
10–66% de `p2`: é estritamente pior no mecanismo, e muda duas coisas de uma vez.
Promovido: `hold.settle_rate = 0.02`, com `weight_moment_yaw` no default
compilado.

Que o settle sozinho tenha derrubado a saturação de 100% para 10–66% **sem
tocar no peso** é o que sustenta a causa acima: a saturação era consequência do
erro posicional acumulado, não da folga de momento.

## Portão

Com `hold.settle_rate = 0.02` promovido no `gait_go2.yaml`, os três critérios
de F4 correm de novo para provar que a parada estável não custou a caminhada:

| critério | aceitação | medido | |
| --- | --- | --- | --- |
| **G1** — 5 ciclos de andar/parar (0,10 m/s, 8 s / 8 s) | deriva de rumo < 5° | **0,2°**, 89,2% reto, tilt de pico 1,21°, **0 RECOVER** | passa |
| **G2** — rastreamento de guinada comandada (86° pedidos em 15 s) | > 85% | **82,7° = 96,2%** | passa |
| **G3** — gatilho do Defeito 2 (parada de 90 s após giro) | não cair | **90 s de pé** (linha de base: colapso em 39,4 s) | passa |

G1 é também a réplica do settle: cinco paradas de 8 s, nenhum RECOVER. G2 caiu
de 99% (medido com `k_yaw` sozinho) para 96,2% — o settle atua na parada, e a
diferença cabe na dispersão entre corridas de uma amostra só; segue bem acima
dos 85% da linha de base.


## Achado colateral: o desvio lateral visível é o Defeito 1, não a marcha

Observação de operador em 20/08/2026: "o robô caminha em curva, e parado ele
treme e faz pequenos movimentos em yaw". As duas coisas são reais e as métricas
resumidas deste relatório as escondiam. Nenhuma das duas é causada pelo settle.

### O tremor parado é anterior, e já melhorou 2,7×

Amplitude de guinada pico a pico durante as paradas, mesma medida nas três
configurações:

| cond | settle | amp. média | amp. pico | \|dyaw\| pico | duração da parada |
| --- | --- | --- | --- | --- | --- |
| `c0_base` (antes do `k_yaw`) | não | 6,14° | 8,04° | 21,6 °/s | 4,9 s |
| `c3_kyaw025` | não | 2,27° | 3,08° | 12,5 °/s | 4,9 s |
| `final_g1_ciclos` | 0,02 | 2,63° | 3,74° | 13,8 °/s | 7,9 s |

O settle não piora o tremor e sustenta paradas 60% mais longas. O resíduo de
~2,6° pico a pico permanece aberto.

### O desvio andando não é curva de rumo — é caranguejo, e é o estimador

Deriva separada por fase, 5 ciclos de 8 s andando / 8 s parado:

| fase | tempo | Δx | Δy | taxa |
| --- | --- | --- | --- | --- |
| caminhada | 39,6 s | 4,190 m | 0,1124 m | 2,84 mm/s = **2,68% de Δx** |
| parada | 39,6 s | 0,163 m | 0,0024 m | 0,06 mm/s |

O rumo fecha em 0,2°: o robô mantém a proa e escorrega de lado. Só andando.
(Cuidado com a inferência fácil: "só andando" **não** exclui o estimador — ele
integra cinemática de pé e por isso só acumula erro quando os pés se movem.)

`k_y` foi varrido e **refutado** como causa. Protocolo idêntico, uma variável:

| cond | `k_y` | Δx caminhada | Δy caminhada | caranguejo | tilt pico |
| --- | --- | --- | --- | --- | --- |
| `y0_base` | 0,005 | 4,228 m | 0,0831 m | 1,97% | 1,13° |
| `y1_k005` | 0,05 | 4,187 m | 0,0829 m | 1,98% | 1,18° |
| `y2_k015` | 0,15 | 4,085 m | 0,0748 m | 1,83% | 1,45° |
| `y3_k025` | 0,25 | 4,029 m | 0,1696 m | 4,21% | 1,25° |

Sem tendência — e o mesmo baseline deu 2,68% na corrida do portão, então a
dispersão dessa métrica é ~0,7 ponto percentual. `k_y` fica em 0,005.

A causa aparece ao comparar `estPos` da linha de supervisor com o `/demo/odom`
de verdade, que é o que o comentário acima daquele log já mandava fazer:

| cond | Δ estimado (x, y) | Δ real (x, y) | erro em y |
| --- | --- | --- | --- |
| `y0_base` | 3,896 · −0,035 | 4,443 · +0,104 | −0,139 m |
| `y1_k005` | 3,892 · −0,072 | 4,385 · +0,043 | −0,116 m |
| `y2_k015` | 3,872 · −0,008 | 4,268 · +0,142 | −0,150 m |
| `y3_k025` | 3,787 · −0,008 | 4,263 · +0,173 | −0,181 m |

O estimador acredita que andou reto (`y` estimado ≈ 0) enquanto o robô escorregou
0,10 a 0,17 m de lado, e subestima o avanço em ~12%. O robô rastreia fielmente
uma crença errada; os erros de rastreamento ficam pequenos porque são calculados
contra a mesma crença que deriva.

**Consequência de projeto:** nenhum ganho de marcha corrige isso. O controlador
não tem referência absoluta, e fechar essa malha é trabalho da navegação por
contrato (`/demo/odom` → Nav2 → `/demo/cmd_vel`). Um paliativo no nó de rotina,
corrigindo pelo `/demo/odom`, funcionaria **só em simulação**: no hardware
`/demo/odom` vem do próprio estimador, então a correção se fecharia sobre a mesma
crença e não faria nada. Não implementado por isso.

## Rotina de exposição: o que a primeira coreografia errou

Ensaio ao vivo de 240 s (2,8 passadas) da primeira versão, contra `/demo/odom`
real. O que funcionou:

- **Nunca caiu.** `z` mínimo 0,343 m em 240 s de movimento contínuo.
- **As paradas são paradas de verdade**: 1–21 mm de deslocamento durante cada
  ajuste de postura, contra ~800 mm num segmento de caminhada. É o `settle_rate`
  fazendo o que foi promovido para fazer, agora visto ao vivo e não só no
  gatilho de 180 s.
- **Repetível entre passadas**: `frente` 0,802 / 0,839 / 0,833 m; `ré` −0,820 /
  −0,853 / −0,862 m; arcos +46,5 / +46,7 / +46,4°.
- **Rastreamento**: avanço 98–102% do comandado, giro no lugar 97%, arco 99%.

O que estava errado: **o padrão não fechava.** Envelope x [0,04; 5,25],
y [−0,01; 3,04], distância máxima da origem 5,85 m e crescendo — **+1,32 m em x
e +0,89 m em y por passada**. Em poucos minutos o robô sai da área de exposição.

A causa é geométrica. `arco esquerda` (+0,782, +0,373) e `arco direita`
(+0,717, **+0,438**) têm os dois Δy positivo, porque o arco à direita começa já a
+46° de proa. **Inverter o sinal de `wz` não é o espelho de um arco** — o espelho
é a reversão temporal, `(v, ω) → (−v, −ω)`. O par de sinal invertido traça um S
e translada.

Duas assimetrias medidas de passagem, que qualquer par espelhado precisa
respeitar:

| eixo | ida | volta | razão |
| --- | --- | --- | --- |
| avanço | 0,098–0,102 m/s | 0,132–0,139 m/s | **1,36** |
| lateral | 0,090–0,098 m/s (esq.) | 0,085–0,086 m/s (dir.) | 0,91 |

A marcha anda para trás 36% mais rápido com o mesmo comando. O fator 0,75 na
duração da `ré` da primeira coreografia compensava isso por acidente, não por
projeto.

### A coreografia refeita fecha por geometria

Caixa de quatro lados de 0,8 m com giro de 90° no lugar entre eles, círculo de
raio 1,0 m em quatro arcos de 90° no mesmo sentido, e o par lateral
esquerda/direita de duração igual. Cada elemento volta sozinho ao ponto de
partida, sem depender de erros se cancelarem.

A caixa também **cancela o caranguejo por simetria**: o desvio de ~2% aponta
sempre para a esquerda do corpo, e quatro lados a 90° apontam esse desvio em
quatro direções opostas.

Os 1–3% de sub-rastreamento de guinada são compensados comandando o ângulo
dividido pelos ganhos medidos (`YAW_TRACKING_SPOT = 0.973`,
`YAW_TRACKING_ARC = 0.990`). O resíduo faz a figura **precessar** em torno do
próprio centro em vez de transladar — precessão mantém o robô no palco,
translação não.

Verificado ao vivo, 300 s, uma passada completa de 233 s:

| | primeira coreografia | refeita |
| --- | --- | --- |
| fechamento de posição por passada | **1,59 m** | **0,183 m** (8,7× melhor) |
| raio máximo em torno do início | 5,85 m, crescendo | **2,18 m, limitado** |
| proa por passada | não fechava | **+23,2° de precessão** |
| quedas em 300 s | 0 | 0 (`z` mín. 0,351 m) |
| deslocamento durante as paradas | 1–21 mm | 0–25 mm |

A precessão de 23°/passada dá uma volta completa em ~15 passadas (≈1 h), com o
robô sempre dentro de ~2,2 m. Para exposição contínua isso basta.

**Próxima melhoria disponível, medida mas não aplicada:** o ganho de guinada
depende do comprimento do segmento. Os 0,973 vieram de giros de 4,2 s; nos giros
de 16,3 s desta corrida o rastreamento medido foi 92,7° de 93,4° comandados, ou
seja 0,992 — o transitório de partida se amortiza num segmento longo. Recalibrar
para os valores de segmento longo deve derrubar a precessão de 23° para ~2° por
passada. Não foi promovido porque exigiria outra corrida de verificação, e a
regra do projeto é que nada vira default sem passar pelo portão.

## O que isto NÃO estabelece

- Nada no hardware. Números de simulação sob Gazebo Harmonic no host x86.
- Não fecha o **Defeito 1** (deriva XY do estimador, 0,18 m em 90 s).
- A árvore TF continua sem fechar (sem frame `odom`) — bloqueio de F5 confirmado.
- `hold.settle_rate` foi medido em um único valor (0,02 m/s). Não há varredura;
  0,01 e 0,04 não foram tentados.
- Uma corrida por condição. A separação é grande (cair em 91–162 s contra ficar
  de pé 180 s, 0 contra 74–357 RECOVER), mas não é n>1.
- O ratchet de rumo descrito em `ml35-caminhada-reta.md` persiste:
  `captureBodyReference()` continua redefinindo `yaw_cmd_` a cada parada.
