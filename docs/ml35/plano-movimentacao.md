# ML3.5 — plano de movimentação do Go2

Escrito em 19/08/2026, depois de pesquisa da documentação de controle de
quadrúpedes em ROS 2 e de reler a evidência de 18/08. Substitui o antigo plano
das Fases 1–3, já executadas; os resultados permanecem em
`docs/results/ml35-f4-parcial.md` §"Fases 1-3".

Cada fase para no portão e espera, conforme `estado-fases.md`.

---

## Correção de duas conclusões da primeira versão deste plano

Registradas em vez de apagadas, porque as duas eram plausíveis lendo só o
código, e as duas são desmentidas por medição que já existia no repositório.

**1. "A marcha não tem autoridade sobre velocidade" — falso.** A Fase 1 de
18/08 mediu rastreamento de 97%, 105% e 111% em `v_cmd` = 0,05 / 0,10 / 0,20
m/s, com zero `RECOVER`. O robô alcança a velocidade comandada. O que continua
verdade é *como*: o laço de velocidade fecha pelo QP empurrando contra
`reference_band` saturado, não pela colocação de pé — o termo neutro de Raibert
apenas **preserva** a velocidade que o corpo já tem. Ver Achado 1.

**2. "A árvore TF fecha em simulação" — falso, e o oposto é o fato.** O plugin
de odometria do Gazebo publica em `/go2/ground_truth_tf`, que
`bridge_quadruped.yaml` **deliberadamente não faz bridge**. Medido em 18/08: não
existe frame `odom`, a árvore está ancorada em `trunk` e flutua. É bloqueador
confirmado de F5 — o item **maior** que resta, não o menor.

---

## Achados da pesquisa

### 1. `k_x`/`k_y = 0.005`: a lacuna é real, o sintoma não é o que eu disse

O termo de realimentação de `FeetEndCalc::calcFootPos` é o ganho da **heurística
de Raibert**, e a escala com fundamento é o coeficiente de *capture point*
√(h/g) ≈ **0,18 s** para h ≈ 0,33 m:

```
next_step(0) = v_body*(1-phase)*t_swing + v_body*t_stance/2 + k_x*(v_body - v_goal)
                └────────── termo neutro, preserva a velocidade ──────────┘   └── realimentação ──┘
```

Conferido verbatim no upstream (`unitree_guide`, `Gait/FeetEndCal.cpp`):
`_kx = _ky = _kyaw = 0.005`, os três iguais, valores do A1. `k_yaw` já foi
corrigido aqui para 0,15 pelo mesmo argumento, e foi o que fez o robô andar.

O que a lacuna **não** causa: falha de rastreamento de velocidade (medido, ~100%).
O que ela plausivelmente causa, e é o que a varredura tem de medir:

- `posErrXY` cravando em `reference_band` em toda corrida gravada — a velocidade
  é imposta por um laço saturado em vez de regulada;
- passeio de rumo em malha aberta (12° a 43° entre execuções);
- fragilidade de partida e parada, onde um termo de frenagem explícito é
  justamente o que falta.

É hipótese com fundamento teórico e sem medição. Uma varredura decide.

### 2. Não há resgate do upstream

Abertos no `legubiao/quadruped_ros2_control`: **#61** "trote tomba quando a
velocidade é alta" (jan/2026) e **#66** "Go2 não anda normalmente no Gazebo"
(jul/2026). Esta árvore está à frente do upstream nessa classe de defeito.

### 3. Dois outros controladores, na mesma interface

`go2_description/config/gazebo.yaml` já declara `ocs2_quadruped_controller`
(MPC convexo via OCS2, **com estimador Kalman linear**) e
`rl_quadruped_controller` (política TorchScript). Ambos são controladores
`ros2_control` atrás da mesma interface de hardware e do mesmo
`/control_input`: trocar é troca de container, não de contrato.

Isto deixou de ser curiosidade e passou a ser candidato direto: o estimador do
OCS2 é exatamente a peça que falta na Fase D. Custo honesto: Pinocchio +
`ocs2_ros2` + geração de código CppAD dentro da imagem **arm64 que roda no
módulo**, e pela regra 5 o custo de CPU só é mensurável no Aquila real.

### 4. O envelope real do robô já está medido

Da Fase 2 de 18/08: **0,20 m/s linear, 0,13 rad/s angular**. A guinada satura em
~0,13 rad/s com `Mz` travado em ±5,3 N·m — o teto de momento, agora expresso em
taxa. `nav2_params.yaml` pede `max_vel_theta ≈ 0,12`, não os 0,25 que a ponte
aceita.

Consequência para a Fase B: se o QP já está no teto de autoridade de guinada,
pesar o resíduo de momento em 450 não compra nada e custa distribuição de força.

### 5. Dois números incoerentes, registrados

| Onde | Valor | Contra | Decisão |
|---|---|---|---|
| `BalanceCtrl` `friction_ratio` | 0,4 | `mu1/mu2 = 0.6` nos pés (`leg.xacro:188`) | **não mexer**: 0,4 é o número que tem de valer em chão real. Virou parâmetro e ficou documentado |
| `WaveGenerator(0.45, 0.5, …)` | literal em código | convenção "parâmetros em YAML" | corrigido na Fase A |

---

## Fases

Todas rodam **no host x86**, no container `sim`. Nada aqui vai para o módulo.

### Fase A — Parametrizar a marcha e versionar o banco de ensaio ✅

**Concluída em 19/08/2026.** Evidência em `ml35-f4-parcial.md` §"Fase A".

### Fase B — Defeito 2: eixo de guinada parado

**Complexidade: média. É o único defeito aberto.** Três experimentos separáveis,
em ordem de valor esperado:

| # | Experimento | Por que agora |
|---|---|---|
| B1a | baixar **só a entrada de guinada** de `balance.weight_moment` (450 → 100) | ataca o mecanismo medido: o QP troca distribuição de força para perseguir um `Mz` que a Fase 2 provou inalcançável acima de 5,3 N·m. **Ensaiado 19/08, n = 1, sinal bom:** sobreviveu 90 s onde a base caiu aos 39,4 s, zero `RECOVER`, eixo fora do batente (54–84% contra 100% cravado), caminhada dentro de 3%. Custo previsto apareceu: rastreamento de guinada 76% contra 85%, deriva de rumo +13,5° em 90 s parado |
| B1b | o mesmo com 450 → 50 | não ensaiado. Só vale se B1a repetir e o custo de rumo incomodar menos que a queda |
| B1c | `trot.kd_w` por eixo: `[70, 70, X]` com X de 70 → 20 → 10 | o termo derivativo lê ~50× a rotação real do corpo (medido: média igual ao pico, assinatura de vibração de tronco). Escalar o termo é knob diferente de filtrá-lo, e mais barato |
| B2 | `trot.kp_w` casado com a autoridade (40–60), mantendo o termo proporcional | com 780 e clamp 10 a banda proporcional é 0,73° — relé. 40–60 dá 10–14° |
| B3 | passa-baixa na taxa de guinada **sem** zerar o proporcional | a única combinação ainda não ensaiada |

**B1a/b e B1c são edições de YAML; B2 e B3 são código.** Isso saiu de uma
assimetria que a Fase A expôs: `Kd_w_` já é `Mat3` construída de um `Vec3`, então
ganho derivativo por eixo é de graça — mas `kp_w_` é um `double` que multiplica os
três eixos, então ganho proporcional por eixo exige mudar o tipo. Ordem de
execução segue o custo: B1a, B1b, B1c, e só então B2/B3.

#### O gatilho, e por que ele precisou ser fixado antes de medir qualquer correção

Primeira tentativa de linha de base **não reproduziu o defeito**: 3 ciclos a
`v_cmd = 0,10` e depois 90 s de HOLD ficaram de pé, com tilt de pico de 0,23° e
deriva de rumo de +0,8°. Isso é coerente com o mecanismo já registrado — o tempo
até a queda escala com o resíduo deixado quando a marcha para (resíduo 0,010 não
cai em 90 s; 0,020 cai aos 46,1 s; 0,034 cai aos 17,8 s) — e é fatal para a
comparação: com n = 1 por condição e um defeito estocástico, qualquer "melhora"
de B1 seria indistinguível de sorte.

Gatilho que fira sempre, retirado do pior caso já documentado (caminhada com
giro), e medido:

```bash
./scripts/gait_trial.sh <csv> --v-cmd 0.10 --w-cmd 0.10   --cycles 1 --walk 15 --hold 0 --final-hold 90
```

Linha de base com ele: **caiu aos 39,4 s**, dentro da faixa registrada de 18 a
46 s, com `yawSat = 100%` nas últimas linhas de HOLD. É este o gatilho de todas
as condições de B, e **cada condição precisa de repetição** — um único número
não separa efeito de dispersão.

**Portão:** HOLD de 90 s após caminhada com giro, zero `RECOVER`, `yawSat < 50%`,
**e** os critérios F4 já verdes preservados (sem rotação livre parado, ≤ 5° em
paradas de 8 s; rastreamento de velocidade dentro do medido na Fase 1). O que
quebrar critério verde é revertido e registrado, como os quatro anteriores.

### Fase C — Varredura de `k_x`/`k_y`

**Complexidade: média. Hipótese, não correção.**

`k_x = k_y` em 0,005 (base) → 0,05 → 0,10 → 0,18, cruzada com `v_cmd` = 0,05 /
0,10 / 0,20 m/s. Uma variável por vez.

Medir, e o critério não é velocidade média (já está em ~100%): fração do tempo
com `posErrXY` no `reference_band`, deriva de rumo por ciclo, tilt de pico,
`RECOVER`, e comportamento de partida/parada. Se a saturação da banda cair,
**reabrir `reference_band`** — ele foi dimensionado como estrangulador de uma
fuga que esta correção removeria.

Riscos: passo mais longo pode sair do espaço de trabalho da perna a 0,20 m/s, e
o termo multiplica ruído de velocidade estimada. Varredura, não salto.

**Portão:** melhora medida em pelo menos duas das quatro métricas sem regressão
nas outras, ou registro explícito de que a hipótese não se sustentou.

### Fase D — Odometria e TF: o bloqueador de F5

**Complexidade: alta. É o item maior que resta, e define o tamanho de F5.**

Fato medido em 18/08: não existe frame `odom`; a árvore TF flutua ancorada em
`trunk`. O Nav2 não localiza sem `odom → base_link`.

Três caminhos, a decidir com medição e ADR:

1. fazer bridge de `/go2/ground_truth_tf` — resolve simulação em minutos e
   **não** resolve hardware; só aceitável como degrau explicitamente temporário;
2. publicar TF a partir do estimador do `unitree_guide_controller` — honesto em
   hardware, mas o estimador não tem medida absoluta de XY e derivou 0,18 m em
   90 s (Defeito 1);
3. trocar para `ocs2_quadruped_controller` e usar o KF dele — resolve odometria e
   o teto de velocidade do trote heurístico de uma vez, ao custo de Pinocchio +
   CppAD em arm64, só mensurável no hardware.

**Portão:** `ros2 run tf2_tools view_frames` com a árvore fechada de `odom` até
os pés, e uma decisão registrada sobre quem produz odometria em hardware.

### Fase E — Reconciliar `nav2_params.yaml` com o envelope medido

**Baixa.** `max_vel_theta ≈ 0,12 rad/s` e `max_vel_x ≈ 0,20 m/s`. Sem isso o DWB
pede o que o robô não entrega e conclui que está preso.

### Fase F — Warehouse · Fase G — Regressão do diff-drive

Inalteradas. Independentes de A–E, esperam sem custo.

---

## Decisão pendente: trocar de controlador

**Reavaliar na Fase D, não antes.** O que mudou desde a primeira versão deste
plano: a Fase D precisa de um estimador de qualquer forma, e o OCS2 traz um. A
comparação deixa de ser "MPC contra heurístico" e passa a ser "escrever
estimador + publicador de TF" contra "adotar um controlador que já tem os dois",
com o custo de build arm64 no outro lado da balança.

Registrar como ADR, com um orçamento de CPU medido no Aquila como condição.

---

## Riscos

| Risco | Prob. | Mitigação |
|---|---|---|
| B1 conserta o parado e degrada a **caminhada** — os quatro experimentos rejeitados foram medidos andando, este defeito é de parado | Média → **baixa para B1a** | ambos os cenários no portão de B. B1a (n = 1) deixou trajetória, tilt e `z` dentro de 3%; o custo caiu na guinada, não na caminhada |
| `k_x` alto amplifica ruído de velocidade estimada e alonga o passo além do espaço de trabalho a 0,20 m/s | Média | varredura em 4 pontos; tilt e `RECOVER` como parada |
| Fase D escolhe o caminho 1 (bridge do ground truth) e o débito é esquecido | **Alta** | só com ADR e prazo; F5 não pode ser declarada sobre ele |
| Estimador próprio (caminho 2) herda a deriva de XY do Defeito 1 | **Alta** | medir deriva contra ground truth antes de ligar no Nav2 |
| OCS2 (caminho 3) não fecha o orçamento de CPU no AM69 | Média | medir no hardware antes de commitar a decisão; regra 5 |
| Warehouse derruba o RTF e muda a dinâmica de contato | Média | RTF medido antes e depois; RTF ≠ 1 invalida comparação de marcha |

---

## Ordem

**A ✅ → B → C, parar e reavaliar.** D é a maior e depende de decisão de
arquitetura; E, F e G esperam sem custo.

## Fontes

- <https://github.com/legubiao/quadruped_ros2_control> e
  <https://deepwiki.com/legubiao/quadruped_ros2_control> (arquitetura, os três
  controladores, estimadores do OCS2)
- <https://github.com/unitreerobotics/unitree_guide> —
  `unitree_guide/src/Gait/FeetEndCal.cpp`, ganhos upstream
- <https://github.com/legubiao/quadruped_ros2_control/issues> — #61, #66
- Heurística de Raibert / capture point:
  <https://arxiv.org/pdf/2212.05154>, <https://arxiv.org/pdf/2307.04030>
- QP de distribuição de força: MIT Cheetah 3,
  <https://dspace.mit.edu/bitstream/handle/1721.1/126619/IROS.pdf>
- <https://leggedrobotics.github.io/ocs2/overview.html>
