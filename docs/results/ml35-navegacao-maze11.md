# ML3.5 F2 — velocidade e qualidade de decisão do Nav2 no maze11

**Data:** 21/08/2026
**Máquina:** host x86 (amd64), Gazebo em container + Nav2 nativo
**Mundo:** `quadruped_maze11.sdf`, nascimento no canto inferior direito, `yaw 1.5708`
**Ferramenta:** `scripts/nav_trial.py` (dirige por metas, nunca publica `/demo/cmd_vel`)

> **Nada aqui vale para o Aquila AM69.** Tudo foi medido na simulação no host
> x86. Regras 5 e 7 do `CLAUDE.md`: emulação e simulação não medem desempenho de
> hardware, e validação de hardware só se declara com evidência do módulo real.

## O que se pediu

1. Rodar o cenário S6 no maze11 e deixar o robô um pouco mais rápido.
2. Depois: *"o robô está muito travado nas decisões de navegação, andando muito
   de ré, ajuste as decisões de movimento para utilizar mais o movimento comum e
   rotações, verifique a integração com lidar e odometria."*
3. Depois: *"identificar rotas sem colisão dando prioridade para a rota onde o
   robô caminha para frente com maior espaço sem objetos ou paredes, senão o robô
   tem que fazer curvas."*

## Resultado, em uma linha

Velocidade média **0,0399 → 0,0650 m/s (+63%)**, ré **11–62% → 0%** das
amostras, eficiência de trajeto **13% → 57%**, com a folga de carcaça inalterada
e zero quedas. O ganho **não** veio de subir teto de velocidade: veio de tirar a
ré do espaço de amostragem e de fazer o planejador preferir espaço livre.

## Condições medidas

Protocolo idêntico em todas: 180 s de tempo de simulação, `--goal-timeout 90`,
as 4 metas de patrulha do maze11, n = 3, pilha reiniciada do zero entre
condições (`reset_stack.sh`).

| | V0 linha de base | V1 só marcha | V2 sem ré | V3 final |
| --- | --- | --- | --- | --- |
| `k_yaw` (marcha) | 0,25 | 0,35 | 0,35 | 0,35 |
| `vx_min` (MPPI) | −0,10 | −0,10 | 0,0 | 0,0 |
| `wz_max` (MPPI) | 0,12 | 0,12 | 0,20 | 0,20 |
| inflação global | 0,55 / 5,0 | 0,55 / 5,0 | 0,55 / 5,0 | **0,85 / 2,0** |
| `SmoothPath` no BT | não | não | não | **sim** |
| n | 3 | 1 | 3 (parcial) | 3 |

### Números

| Métrica | V0 (n=3) | V3 (n=3) |
| --- | --- | --- |
| velocidade média | 0,0525 / 0,0428 / 0,0244 → **0,0399** | 0,0693 / 0,0614 / 0,0644 → **0,0650** |
| dispersão (pior→melhor) | 2,2× | **1,13×** |
| `cmd_vx` pico | 0,070 / 0,087 / 0,108 | **0,143 / 0,130 / 0,135** |
| `cmd_vx` médio | −0,008 / −0,002 / +0,017 | **+0,055 / +0,051 / +0,052** |
| `cmd_vx` negativo | 54% / 37% / 11% | **0% / 0% / 0%** |
| caminho percorrido | 9,45 / 7,69 / 4,40 m | 12,47 / 11,04 / 11,58 m |
| `yawSat` média / pico | 38/92, 25/94, 20/90 % | **3/74, 3/64, 1/70 %** |
| tilt pico | 1,5 / 3,3 / 0,7 ° | 0,9 / 0,5 / 0,7 ° |
| `RECOVER` | 0 / 0 / 0 | 0 / 0 / 0 |
| folga mínima (lidar) | 0,448 m → carcaça **+0,065 m** | 0,448 m → carcaça **+0,065 m** |

`vx_max` ficou em **0,15 nas quatro condições**, de propósito — ver abaixo.

## Por que subir `vx_max` não era o caminho

O plano original previa varrer `vx_max` 0,15 → 0,18 → 0,20. A medição da V0
matou essa ideia antes de gastar corridas nela:

- `cmd_vx` pico ficou entre **0,070 e 0,108** contra um teto de **0,15 que nunca
  foi alcançado**. Teto que não é atingido não é o limitante.
- `cmd_vx` **médio** era ≈ 0 (−0,008 a +0,017 m/s), com 11–62% das amostras
  negativas. O comando era ruído em torno de zero. Multiplicar o teto de uma
  distribuição centrada em zero não move a média.

O que decide o tempo de travessia é a **média**, não o pico. Na V3 o pico subiu
para 0,130–0,143 **sem tocar em `vx_max`**: o teto sempre esteve lá, o
controlador é que não o usava.

## Os três defeitos encontrados

### 1. Deadlock por ré (`vx_min: -0.10`)

Medido com `sensor_check.py`, janela de 20 s: **`/demo/cmd_vel_si` com `vx < 0`
em 100% das amostras**, e o robô andou **0,00 m em 25 s**.

A cadeia: num corredor de 1,20 m o `PathAlignCritic` (peso 14, o maior) acha
ótimo recuar para realinhar; o robô recua; encosta na parede de trás; e ré
**continua** ótima, porque a geometria que a tornou ótima não mudou. O robô fica
comandando ré contra a parede indefinidamente.

`PreferForwardCritic` já estava ligado, com peso 5,0, e não segurou — peso não
compete com uma opção que não deveria estar no espaço de amostragem.

**Correção:** `vx_min: 0.0` no MPPI e `min_velocity[0]: 0.0` no
`velocity_smoother`. O MPPI passa a não conseguir amostrar ré: para corrigir rumo
tem de **girar**. Recuo de verdade continua possível, porque o `backup` do
`behavior_server` publica `cmd_vel` direto e não passa por esses limites.

Efeito imediato: ré de **100% → 0%**, frente 99%.

### 2. `smoother_server` configurado e nunca invocado

**Nenhum behavior tree que acompanha o Nav2 Jazzy chama `SmoothPath`.**
Verificado com `grep -l SmoothPath` em
`/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/*.xml`: zero arquivos.

Então o `simple_smoother` deste projeto subia, ficava `active`, e **nada nunca
lhe mandava trabalho**. Sem erro, sem aviso. Mesma classe de defeito do
`perception_layer` sem produtor.

Por que importa: o NavFn é Dijkstra sobre grade de 0,05 m 8-conexa, então o
caminho global sai em **escada**. O `PathAlignCritic`, peso 14, cola o robô nessa
escada. O robô persegue cada degrau, o que aparece como giro contínuo sem motivo
visível — num corredor reto.

**Correção:** `demo_navigation/behavior_trees/nav_to_pose_smoothed.xml`, cópia da
árvore default com `ComputePathToPose` e `SmoothPath` numa `Sequence`. O caminho
absoluto é injetado por `RewrittenYaml` em `nav_quadruped.launch.py` — não fica
cravado no YAML, e o `navigation_launch.py` vendorizado não é tocado.

### 3. NavFn escolhia rota por comprimento, não por folga

NavFn é Dijkstra puro sobre o custo do costmap e não tem termo próprio de folga.
Com `inflation_radius: 0.55` num corredor de 1,20 m, a faixa de custo **zero** no
meio tem 0,10 m (1,20 − 2×0,55). Fora dessa fita o custo é plano, e o Dijkstra
escolhe pelo comprimento — ignorando espaço livre.

**Correção, só no costmap global:** `inflation_radius: 0.85`,
`cost_scaling_factor: 2.0`. Com 0,85 > 0,60 (meia-largura do corredor) não existe
mais célula de custo zero no corredor: o gradiente é monotônico até o centro, e
rota apertada passa a custar mais que rota aberta.

Duas coisas que essa mudança **não** faz:

- **Não fecha corredor.** A faixa *inscrita*, que o planejador trata como
  obstáculo, vem de `robot_radius` (0,38), não de `inflation_radius`. Subir a
  inflação só reordena preferência entre rotas que continuam todas viáveis.
- **Não recria o defeito de "plano global que o local recusa".** O comentário
  original pedia inflação casada entre os dois costmaps, e isso vale numa direção
  só: global **mais** conservador que o local é seguro (o caminho global cai
  sempre em terreno que o local aceita); global **menos** conservador é que
  produz o defeito. O local ficou em 0,55 de propósito — subir a inflação local
  aproximaria o MPPI do campo de custo saturado que trava o otimizador.

## Integração de lidar e odometria: verificada, e está sã

Pedido explícito. Medido com `sensor_check.py` e `selfhit.py`, janelas de 20–25 s:

| Item | Medido | Veredito |
| --- | --- | --- |
| `/demo/odom` vs TF `odom -> base` | erro **0,0000 m** | casado |
| `/demo/odom` | 49,8 Hz | ok |
| `/demo/scan_cloud` | 9,9 Hz, 10240 pontos/nuvem | sustenta o laço de 10 Hz do MPPI |
| `/demo/cmd_vel_si` | 20,0 Hz | ok |
| TF `map -> odom`, `odom -> base` | presentes | árvore fecha |
| auto-colisão do lidar | nenhuma | ver abaixo |
| `/demo/scan` sem retorno | 43% | esperado: parede de 0,60 m, raio passa por cima |

**Deriva de odometria está descartada por construção**, não por medida de erro:
`/demo/odom` é ground truth do Gazebo (`/go2/odom` via `bridge_quadruped.yaml`).
Odometria de perna real é trabalho de F5, e é lá que deriva passa a ser risco.

**Sobre auto-colisão do lidar.** Uma primeira janela mostrou 3086 pontos a menos
de 0,383 m com rumo médio +82,7° e desvio de só 2,2°, o que parecia peça do robô.
**Não é.** Numa segunda janela o agrupamento havia mudado de lugar, e os raios
dos setores próximos ajustam `d/cos(θ−θ₀)` de uma **parede reta** a 0,372 m com
normal em ~+140°, cobrindo 145° contínuos. O robô estava encostado na parede
(defeito 1), e parede vista de perto por um robô parado dá exatamente a
assinatura de rumo fixo.

Fica registrado que `selfhit.py` **não conclui com o robô parado** — ele avisa
isso na saída. Com robô parado, parede também dá desvio zero.

## Costmap: o que o `CostCritic` realmente vê

Medido com `costmap_probe.py`, corredor do maze11:

| | valor |
| --- | --- |
| células conhecidas | 14400 (120×120 @ 0,05 m) |
| custo 0 (livre) | 63,6% |
| ≥ 253 (colisão para o `CostCritic`) | 23,3% |
| = 254 (letal) | 178 |
| custo na célula do robô | 0 |
| faixa transversal de custo zero | 0,80 m de 1,60 m sondados |

Isso **refuta** a hipótese de que o corredor de 1,20 m saturava o costmap e por
isso o MPPI travava. Não satura: há faixa livre de sobra. O defeito era de
decisão, não de percepção.

Armadilha encontrada no caminho, que vale para qualquer sonda futura:
`/local_costmap/costmap` é `OccupancyGrid` e é uma **reescala** — o
`Costmap2DPublisher` mapeia 254 → 100, 253 → **99**, 255 → −1, resto para 1..98.
Comparar com 253 ali nunca casa, e o resultado sai como "nenhuma célula de
colisão" num corredor cercado de parede. Use `costmap_raw` (`nav2_msgs/Costmap`),
que traz 0..254.

## O portão, e o que ele não cobre

Critério de F2: zero quedas, zero colisão nos corredores de 1,20 m, tilt dentro
da banda medida, e média melhor que a V0 com n = 3.

| Critério | V3 | |
| --- | --- | --- |
| quedas | 0 em 3 corridas | ✅ |
| `RECOVER` do supervisor | 0 em 3 corridas | ✅ |
| tilt pico | 0,5–0,9° (V0: 0,7–3,3°) | ✅ |
| folga de carcaça | +0,065 m, igual à V0 | ✅ não piorou |
| média > V0 | 0,0650 vs 0,0399 (+63%) | ✅ |

**O que este documento NÃO estabelece:**

- **Nada sobre o Aquila AM69.** Nem CPU, nem latência, nem se o MPPI mantém
  10 Hz no módulo. O MPPI é o nó mais caro da pilha e isso só se mede no
  hardware.
- **Taxa de metas em regime.** A corrida de confirmação abaixo fechou 1 de 2.
  Uma amostra não é uma taxa.
- **Folga de +6,5 cm é pouco.** É o portão de qualquer aumento futuro de
  velocidade, e não folgou com a V3. Vem de `robot_radius: 0.38`, que modela o
  tronco de 0,70 × 0,31 m como **círculo**; lateralmente o robô precisa de
  0,155 m, não 0,383 m. A correção estrutural é declarar footprint poligonal e
  `consider_footprint: true`, já apontada no próprio `nav2_params_go2.yaml`, e
  está fora do escopo de F2.
- **Deslocamento líquido não é métrica de travessia.** Nas corridas 2 e 3 da V3
  ele deu 0,62 e 0,52 m com 11 m de caminho, porque o ciclo de 4 metas volta para
  perto da partida. Caminho e velocidade média são as métricas.

## Corrida de confirmação: meta cumprida

Nas corridas de 180 s com `--goal-timeout 90` nenhuma meta fechou, e isso é
aritmética do protocolo, não defeito: a 0,065 m/s, 90 s cobrem 5,9 m de caminho
e a primeira meta está a 8 m. Nenhuma meta **podia** fechar.

Repetido com prazo compatível com a velocidade real
(`--seconds 420 --goal-timeout 200`), mesma configuração V3:

| Métrica | valor |
| --- | --- |
| metas | **1 cumprida de 2 encerradas** (`meta 0: ok em t=96s`) |
| caminho percorrido | 22,03 m em 419,9 s |
| velocidade média | 0,0525 m/s |
| `cmd_vx` pico / médio | 0,136 / 0,0441 m/s |
| `cmd_vx` negativo | **0%** |
| tilt pico | 0,63° |
| `RECOVER` | 0 em 1674 linhas de supervisor |
| `yawSat` média / pico | 2% / 76% |
| folga mínima | 0,448 m → carcaça +0,065 m |

A meta 0 está a 8 m e foi alcançada em 96 s. A meta 1 (a 8 m no outro eixo,
atravessando o labirinto) expirou em 200 s. Sob V0 nenhuma meta havia sido
cumprida em nenhuma corrida.

A média de 0,0525 m/s nesta corrida é menor que os 0,0650 m/s das corridas de
180 s. Não é regressão: a corrida é 2,3× mais longa e inclui o trecho em que o
robô insiste na meta 1 até o prazo, que é justamente o pedaço lento. Compare
condições no mesmo protocolo, nunca entre protocolos.

## Como reproduzir

```bash
export MAZE_MODELS=~/ros_maze_worlds/models
./scripts/run_quadruped_sim.sh quadruped_maze11.sdf     # terminal 1
ros2 launch demo_bringup nav_quadruped.launch.py        # terminal 2
python3 scripts/nav_trial.py out.csv --seconds 180 \
    --goal-timeout 90 --sim-log <log do terminal 1>     # terminal 3
```

Confira que os parâmetros entraram — os quatro foram lidos do nó em execução,
não do YAML:

```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /controller_server FollowPath.vx_min
ros2 param get /controller_server FollowPath.wz_max
```

Antes de reiniciar o Nav2, limpe órfãos da execução anterior — matar o
`ros2 launch` deixa os nós filhos vivos segurando índice de participante do
CycloneDDS, e a subida seguinte morre acusando o DDS. Ver
`docs/guia-operacao.md` §9.12.
