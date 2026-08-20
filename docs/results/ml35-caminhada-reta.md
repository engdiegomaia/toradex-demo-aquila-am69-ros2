# ML3.5 — caminhada reta do Go2: causa raiz e correção

**Data:** 20/08/2026. **Onde rodou:** workstation x86, container `aquila-go2`,
Gazebo Harmonic. **Nada disto foi medido no Aquila AM69 real** — regra 5 e 7 de
`CLAUDE.md` valem integralmente: estes números não autorizam nenhuma afirmação
sobre CPU, latência ou térmica no módulo.

Sintoma relatado: o robô cambaleia e não anda reto ao receber comando de avanço.

## Causa raiz

Realimentação positiva na **colocação de pé**, em `FeetEndCalc::calcFootPos`
(`unitree_guide_controller/src/gait/FeetEndCalc.cpp:68`):

```cpp
const double d_yaw = estimator_->getDYaw();     // taxa de guinada MEDIDA
next_yaw = d_yaw*(1-phase)*t_swing + d_yaw*t_stance/2
         + k_yaw_*(d_yaw_global - d_yaw);
```

Os dois primeiros termos rotacionam o ponto de pouso **junto com** a rotação que
o corpo já tem; só `k_yaw` se opõe. Com `t_swing = t_stance = 0,225`:

| fase do balanço | coeficiente que segue | opõe (`k_yaw = 0,15`) | saldo |
|---|---|---|---|
| 0,0 | 0,3375 | 0,15 | **+0,1875 segue** |
| 0,5 | 0,2250 | 0,15 | **+0,0750 segue** |
| 1,0 (toque) | 0,1125 | 0,15 | −0,0375 opõe |

`k_yaw = 0,15` **não foi mal escolhido**: o comentário do próprio arquivo o
dimensiona contra o coeficiente **no toque do pé** (0,1125), e ali está correto.
A lacuna é o resto do balanço, onde o saldo é positivo — o corpo gira, o padrão
de apoio gira mais, o corpo gira mais.

### Por que o QP nunca corrigia

O mesmo comentário registra que a colocação de pé é *"the only actuator that can
hold heading on this robot"*, porque o QP de balanço satura em ~5,3 N·m de
momento de guinada — 0,13 rad/s de autoridade. Medido na base, em WALK:

| | base (`k_yaw` 0,15) | `k_yaw` 0,25 |
|---|---|---|
| \|Mz\| médio pedido | 5,23 N·m | 4,31 N·m |
| ticks no batente (≥5,2 N·m) | **98,6%** | 66,2% |
| \|yawErr\| médio | **9,97°** | **0,41°** |
| \|yawErr\| pico | **45,39°** | **2,12°** |

Na excursão do ciclo 2 da base o robô girou 38° em ~2,5 s = **0,27 rad/s**, com
`Mz` cravado no teto todo o tempo. A colocação de pé dirigia a rotação a **2× a
autoridade que o QP tem para contrariá-la**. O QP não estava mal sintonizado;
estava perdendo uma disputa que não podia ganhar.

### Catraca de rumo (segundo achado, independente do ganho)

`captureBodyReference()` reancora `yaw_cmd_ = getYaw()` na saída de WALK. Medido:
`yawErr` chega a −38,5° andando e cai para −1,2° no HOLD seguinte. O erro **não é
corrigido, é apagado por redefinição**, e o ciclo seguinte trata 42° como "reto".
Fechar a malha absoluta de rumo é papel do Nav2 (contrato do projeto); o que esta
correção trata é a retidão **dentro** de cada movimento.

## Correção

Uma linha de YAML: `foot_placement.k_yaw` de **0,15 → 0,25**. Sem rebuild.

## Evidência

Cenário idêntico em todas as corridas, RTF 1,00, container reiniciado limpo entre
elas: `--v-cmd 0.10 --w-cmd 0.0 --cycles 3 --walk 12 --hold 5`. Valores vivos
provados pela linha `gait params:` em cada corrida.

`lat_m` = maior desvio perpendicular ao rumo inicial; `serp°` = pico-a-pico médio
de yaw por ciclo; `reto%` = deslocamento líquido / trajetória.

| corrida | `k_yaw` | reto% | lat_m | serp° | drift° | tilt_pk | yawSat% | dWzMed | REC |
|---|---|---|---|---|---|---|---|---|---|
| c0_base | 0,15 | 82,6 | 1,140 | 27,9 | 39,1 | 2,13 | 96,4 | 432,6 | 0 |
| c0r2 | 0,15 | 83,5 | 1,560 | 25,4 | 35,2 | 2,72 | 96,7 | 429,7 | 0 |
| c3 | **0,25** | 95,3 | 0,080 | 3,1 | −3,3 | 0,95 | 62,2 | 18,8 | 0 |
| c3r2 | **0,25** | 88,6 | 0,095 | 2,1 | −1,6 | 1,05 | 59,0 | 15,1 | 0 |
| c3r3 | **0,25** | 87,4 | 0,109 | 4,2 | 0,1 | 2,08 | 81,0 | 29,8 | 0 |
| c4 | 0,35 | 87,0 | 0,145 | 0,8 | 0,1 | 0,87 | 11,5 | 4,5 | 0 |

**As faixas não se sobrepõem** nas métricas que importam: desvio lateral
1,14–1,56 m contra 0,08–0,11 m (12–19×), serpenteio 25–28° contra 2–4° (7–12×),
deriva 35–39° contra −3,3 a +0,1°. Zero `RECOVER` em todas as corridas, nas duas
condições — a correção não compra retidão com estabilidade.

## Portão

| critério | resultado | veredito |
|---|---|---|
| 5 ciclos andar/parar (o critério F4 que os 4 experimentos rejeitados quebraram) | deriva de rumo **−0,3°** em 5 ciclos, tilt de pico 1,11° andando / 1,03° parado, zero `RECOVER`, 4,268 m líquidos | **passa**, com ressalva: 1,03° parado contra o critério de < 1° é marginal (base de 18/08: 0,9°) |
| rastreamento de guinada comandada | **85,3° de 86° pedidos = 99%** (base documentada: 85%) | **melhora** 14 pontos |
| rastreamento de velocidade | 0,107 m/s líquidos para 0,10 comandado = 107% | dentro da faixa medida de 97–111% |
| Defeito 2 (parada longa de 90 s, gatilho `--w-cmd 0.10 --walk 15 --final-hold 90`) | **sobreviveu os 90 s de pé, zero `RECOVER`** — a base sob o mesmo gatilho cai aos 39,4 s | **não piora; melhora**, mas ver ressalva |

Ressalva sobre o Defeito 2, para não ser lida como conclusão: `yawSat` continua
em **100%** nas últimas linhas de HOLD, ou seja o eixo de guinada segue no batente
parado. O portão da Fase B exige `yawSat < 50%` **e** sobrevivência; só a segunda
metade foi atendida. E é **n = 1** num defeito estocástico. Portanto a Fase B
continua aberta: `k_yaw = 0,25` não é o tratamento do Defeito 2, apenas não o
agrava — plausivelmente porque uma caminhada mais reta entrega menos resíduo à
parada, que é o mecanismo já registrado em `ml35-f4-parcial.md`.

`k_yaw = 0,35` foi medido e mantém o rumo ainda melhor (deriva 0,1°, yawSat 12%),
mas leva o avanço líquido a **137%** do comandado — fora do envelope contra o
qual `nav2_params.yaml` é dimensionado. Por isso a escolha é 0,25, e 0,35 fica
para reavaliação junto com a Fase E.

## Resultado negativo, registrado para não ser repetido

`trot.kd_w` na entrada de guinada, 70 → 20 (experimento **B1c** do plano da Fase
B): **o robô tombou** — tilt de pico 158°, 68 `RECOVER`, `z` variando 0,291 m. O
eixo saiu do batente como a caracterização previa (`yawSat` 96,4% → 72,9%,
`dWzMed` 432 → 64), e ainda assim caiu.

Conclusão que contraria a hipótese de partida desta sessão: **o termo derivativo
em 70 está fazendo amortecimento necessário durante a caminhada**, não apenas
amplificando ruído de vibração de tronco. B1c foi proposto contra o defeito de
*parado*; medido sobre a *caminhada*, ele quebra a marcha.

## O que este resultado NÃO estabelece

- **Nada em hardware.** Simulação apenas.
- **O Defeito 2 não está resolvido.** Uma corrida sobreviveu 90 s, mas com o
  eixo de guinada ainda 100% no batente e com n = 1. A Fase B segue aberta.
- **`k_x`/`k_y` seguem em 0,005.** A Fase C (varredura contra o coeficiente de
  capture point ≈ 0,18) permanece aberta e não foi tocada aqui.
- **A catraca de rumo entre movimentos continua.** Corrigida a retidão dentro de
  cada movimento, o rumo absoluto ainda não é recuperado entre paradas, por
  projeto.
