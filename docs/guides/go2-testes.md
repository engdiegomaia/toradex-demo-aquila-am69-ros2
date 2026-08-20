# Go2 — guia de comandos para testar a aplicação

Roteiro de operação do spike do quadrúpede em simulação. Tudo aqui roda na
**workstation x86**, nada no módulo Aquila — Gazebo é OGRE 2 e não pode subir no
AM69 (ver `CLAUDE.md`, regra 1).

Estado da aplicação em 20/08/2026: o robô **caminha, para e se mantém parado
de forma repetível**, e roda uma **rotina de exposição em loop** por tempo
indefinido (§7.3) — 300 s de movimento contínuo sem uma queda. O que era o
bloqueio de 19/08, a queda em `HOLD` entre ~18 s e ~46 s (Defeito 2, §7.5),
está fechado.

Em 20/08/2026 a **caminhada reta** foi corrigida: `foot_placement.k_yaw` 0,15 →
0,25 derrubou o desvio lateral de 1,14–1,56 m para 0,08–0,11 m e a deriva de rumo
de 35–39° para menos de 1°, com zero quedas e rastreamento de guinada comandada
subindo de 85% para 99% (`../results/ml35-caminhada-reta.md`).

Ainda em 20/08/2026 a **parada longa foi fechada**: `hold.settle_rate = 0.02`
reancora a referência de corpo no centroide de apoio durante o `HOLD` e o robô
passou de colapso em 161,7 s para 180 s de pé com zero `RECOVER`, mantendo os
três critérios de F4 (`../results/ml35-postura-parada.md`). Baixar o peso de
momento de guinada na parada foi medido e **rejeitado** — piora.

Dois comportamentos visíveis seguem **abertos**, ambos medidos e nenhum causado
pelas correções acima:

- **Tremor parado**: ~2,6° de guinada pico a pico, picos de 14 °/s. É anterior;
  o `k_yaw` já o cortou de 6,1° para 2,3°.
- **Caranguejo andando**: o rumo fecha em 0,2°, mas o robô escorrega de lado a
  ~2% da distância de avanço. A causa é o **Defeito 1**: o estimador acredita que
  andou reto (`estPos` em y ≈ 0) enquanto o `/demo/odom` real mostra 0,10–0,17 m
  de desvio. Nenhum ganho de marcha corrige — `k_y` foi varrido e refutado.
  Fechar essa malha é trabalho da navegação por contrato.

Evidência em `docs/results/ml35-f4-parcial.md`; plano e tabela de experimentos em
`../ml35/plano-movimentacao.md`; contexto de implementação em
`go2-proximos-passos.md`.

---

## 1. Pré-requisitos

Uma vez por máquina:

```bash
# a imagem do spike precisa existir
docker images | grep demo-sim
# esperado: demo-sim   spike-go2   ...
```

O script já cuida do `xhost` e do `/dev/dri`. Se o Gazebo abrir preto, o
problema é aceleração gráfica do host, não a aplicação.

---

## 2. Subir a simulação

```bash
cd ~/toradex/demo/aquila-am69-ros2
./scripts/run_quadruped_sim.sh
```

Mantenha o terminal aberto — é ele que segura o container `aquila-go2`. O script
copia `ros2_ws/src` para dentro, roda `colcon build` e lança
`demo_simulation quadruped.launch.py`. **A primeira subida leva ~2 min** por
causa do build.

Para um mundo diferente:

```bash
./scripts/run_quadruped_sim.sh /caminho/para/mundo.sdf
```

### Aguardar a máquina de estados

Não mande comando antes disto aparecer:

```bash
docker logs -f aquila-go2 2>&1 | grep --line-buffered "gait FSM"
```

Sequência esperada: `passive` → `fixed down` → `fixed stand` → a linha
`gait FSM: fixed stand. Waiting`. Só a partir daí o robô aceita velocidade.

Bloquear até estar pronto, em script:

```bash
until docker logs aquila-go2 2>&1 | grep -q "gait FSM: fixed stand. Waiting"; do sleep 5; done
```

---

## 3. Abrir um shell com o ambiente ROS

Todo comando `ros2` das seções seguintes assume este ambiente:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh && exec bash'
```

Do **host** também funciona, desde que o ambiente bata — `ROS_DOMAIN_ID=69` e
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Trocar o RMW quebra a descoberta entre
containers (`CLAUDE.md`, regra 2):

```bash
export ROS_DOMAIN_ID=69 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 4. Verificar o contrato de tópicos

```bash
ros2 topic list | grep demo
ros2 topic hz /demo/odom              # esperado ~100 Hz
ros2 topic hz /demo/scan
ros2 topic hz /demo/camera/image_raw
ros2 node list
```

Tipos, para conferir que a ponte está no ar:

```bash
ros2 topic info /demo/cmd_vel         # geometry_msgs/msg/Twist
ros2 topic info /control_input        # control_input_msgs/msg/Inputs
```

Não encadeie `| head` nesses comandos: o `ros2 topic info` fecha com
`BrokenPipeError` e polui a saída sem motivo.

`/demo/cmd_vel` é a interface pública. `/control_input` é interna do
controlador, emulando gamepad — não publique nela diretamente.

---

## 5. Fazer o robô andar

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

`Ctrl-C` para parar. **Publique a 20 Hz ou mais**: a ponte tem watchdog de
0,3 s e silêncio significa parar.

### Conversão de comando

O `Twist` passa por uma ponte que emula gamepad, então o número que você publica
não é a velocidade final:

| `Twist` | limite | velocidade resultante | máximo |
|---|---|---|---|
| `linear.x` | stick ±0,5 | `0,4 × x` m/s | 0,20 m/s |
| `linear.y` | stick ±0,5 | `0,3 × y` m/s | 0,15 m/s |
| `angular.z` | stick ±0,5 | `0,5 × z` rad/s | 0,25 rad/s |

Então `linear.x = 0.25` → **0,1 m/s**, que é o ponto de operação validado.

> Não comande velocidades muito baixas achando que é mais seguro. O trote tem um
> ponto de projeto: abaixo de ~0,05 m/s o comprimento de passo fica milimétrico
> sob uma elevação de pé de 8 cm, e o robô marcha no lugar e desestabiliza. Isso
> está medido em `ml35-f4-parcial.md`.

Andar para trás, de lado e girar:

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.25}}"
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.25}}"
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```

Guinada comandada **funciona, com folga medida**: em 15 s a `w_cmd = 0,10`
rad/s (`angular.z = 0.2`) o robô entrega 72,7° dos 86° comandados — 85%. Mas o
eixo de guinada opera cravado no batente do clamp (`yawSat = 100%`, §6): é
liga-desliga, não regulador. E caminhada **com giro** é o pior caso do Defeito 2
— é justamente o gatilho usado para reproduzi-lo (§11).

---

## 6. Ler a linha de diagnóstico

O supervisor do trote imprime a 4 Hz. É o instrumento principal:

```bash
docker logs -f aquila-go2 2>&1 | grep --line-buffered "trot supervisor"
```

```
trot supervisor: mode=WALK cmd=(0.1000,0.0000,0.0000) tilt=0.4deg
  posErrXY=0.0071 velErrXY=0.0564 contact=[1 0 0 1]
  estPos=(0.041,0.072,0.334) estVel=(0.071,-0.029) estYaw=-0.0deg
  Mz=5.3/5.2Nm Fz=142/142N footErr=[0.010 0.008 0.008 0.010]
  yawErr=.../pk...deg dWzPk=... dWzMed=... yawSat=...%
```

A última linha é a instrumentação do eixo de guinada, elidida aqui porque não há
leitura medida dela em caminhada reta — a assinatura que existe é de `HOLD` e
está mais abaixo.

| campo | o que significa | valor saudável |
|---|---|---|
| `mode` | `HOLD` parado, `WALK` andando, `RECOVER` perdendo atitude | — |
| `cmd` | velocidade que chegou ao controlador (m/s, m/s, rad/s) | bate com a tabela §5 |
| `tilt` | ângulo do corpo contra a vertical | < 2° |
| `posErrXY` | erro da referência de posição do corpo | < 0,015 m |
| `contact` | pés no chão, ordem FR FL RR RL | alterna `[1 0 0 1]` ↔ `[0 1 1 0]` |
| `estPos` / `estYaw` | crença do estimador | ver nota abaixo |
| `Mz` | momento de guinada **pedido/realizado** | os dois próximos |
| `Fz` | força vertical **pedida/realizada** | ~142/142 N |
| `footErr` | distância de cada pé ao alvo | < 0,02 m e **simétrico** |
| `yawErr` | erro de guinada **atual/pico da janela** | oscilando em torno de zero |
| `dWzPk` / `dWzMed` | demanda de aceleração de guinada **antes do clamp**, pico e média da janela | ≪ `yaw_clamp` (10 rad/s²) |
| `yawSat` | % dos ticks da janela com o eixo no batente | 0 em regime |

Nos campos de guinada, pico e média são **acumulados na janela e zerados a cada
impressão**. Isso não é detalhe: amostrar um relé de 500 Hz a 4 Hz sem acumular
é aliasing, e foi o que manteve o defeito invisível. Assinatura medida em
`HOLD`, momentos antes de uma queda:

```
yawErr=4.86/pk5.31deg  dWzPk=366.2  dWzMed=287.6  yawSat=100%
```

Leia isso como três fatos: o erro fica **cravado de um lado** (3,5–5,3°, sem
cruzar zero) porque `kp_w = 780` contra clamp de 10 rad/s² dá banda proporcional
de 0,73°; a demanda é 33× o clamp; e o eixo está no batente em 100% dos ticks.
Não é uma leitura saudável — é o Defeito 2 se formando.

Notas de leitura que custaram caro para descobrir:

- `estPos(2)` fica ~24 mm **abaixo** do real. É viés conhecido e constante
  (`foot_radius = 0.02` contra `feet_h_ = 0` no estimador), não deriva.
- `Mz` travado em ±5,3 com sinal alternando é o **batente**, não controle. O
  teto de 5,3 N·m é o do QP neste robô e não é falha em si — mas ficar nele
  **continuamente**, sem alternar, é: veja `yawSat` acima e §7.5. Levantar o
  clamp foi medido e é pior (13,2 pedidos, 5,8 entregues, queda em 4 s).
- `footErr` **assimétrico e constante** é a assinatura do bug de alvo obsoleto
  já corrigido. Se voltar, é regressão.

---

## 7. Roteiros de teste

### 7.1 Caminhada contínua

```bash
ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}" &
sleep 30 && kill %1
```

Aceitação: nenhuma linha `mode=RECOVER`, `tilt` sempre < 2°, ~3 m percorridos.

### 7.2 Andar e parar repetidamente — o teste que pega regressão

```bash
docker exec aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  for c in 1 2 3 4 5; do
    echo "ciclo $c antes:"; timeout 12s ros2 topic echo /demo/odom --field pose.pose --once
    timeout 8s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}" >/dev/null 2>&1 || true
    sleep 8
    echo "ciclo $c depois:"; timeout 12s ros2 topic echo /demo/odom --field pose.pose --once
  done'
```

Aceitação medida em 18/08/2026: **zero quedas**, `tilt` máximo 1,0° andando e
0,9° parado, `footErr` de 0,4 a 1,3 cm e simétrico, 3,85 m em 5 ciclos.

O rumo **passou a ser critério** em 20/08/2026. Com `k_yaw = 0,25` a deriva
medida em 5 ciclos é **−0,3°**; a redação anterior desta seção ("passeio aleatório
por projeto, 12° a 43°") descrevia `k_yaw = 0,15` e não vale mais — o passeio era
um laço de realimentação positiva na colocação de pé, não uma escolha de projeto
(`../results/ml35-caminhada-reta.md`). Aceitação: deriva < 5° em 5 ciclos.

O que continua sendo papel do Nav2 é o rumo **absoluto** entre movimentos:
`captureBodyReference()` reancora a referência em cada parada, então um desvio já
acumulado não é recuperado.

### 7.3 Rotina de exposição contínua

O nó `demo_routine` conduz a simulação sozinho, em loop, com ajuste de postura
entre cada movimento. Roda no host ou no módulo — fala só o contrato público
`/demo/cmd_vel` e lê `/demo/odom` apenas para saber se o robô está de pé.

```bash
ros2 launch demo_bringup routine.launch.py
```

**Nunca rode junto com `nav.launch.py`**: os dois publicam em `/demo/cmd_vel` e
os comandos se intercalam.

A passada dura 233 s e é fechada por geometria: caixa de quatro lados de 0,8 m
com giro de 90° no lugar, círculo de raio 1,0 m em quatro arcos de 90°, e o par
lateral esquerda/direita. Medido: 0,183 m de erro de fechamento por passada,
+23,2° de precessão, raio máximo de 2,18 m em torno do início, zero quedas em
300 s (`../results/ml35-postura-parada.md`).

O ajuste de postura entre movimentos é **silêncio**, não `Twist` de zeros:
publicar zeros mantém o comando fresco e o robô nunca chega a `HOLD`. O watchdog
de 0,3 s (§7.4 abaixo) é o que converte silêncio em parada.

Parâmetros úteis:

```bash
ros2 launch demo_bringup routine.launch.py settle_s:=8.0 move_s:=6.0 loop:=false
```

Se a figura transladar em vez de precessar, alguém trocou a coreografia por
pares de arco com sinal de `wz` invertido — isso não fecha, e o custo medido foi
1,59 m de deriva por passada.

---

### 7.4 Watchdog de comando

```bash
timeout 5s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

Aceitação: em até 0,3 s após o `timeout`, o supervisor mostra `mode=HOLD` com
`cmd=(0.0000,0.0000,0.0000)`. O robô **não** pode continuar andando.

### 7.5 Postura parada prolongada — falha conhecida (Defeito 2)

**Corrigido em 20/08/2026 por `hold.settle_rate = 0.02`.** O texto abaixo
descreve a falha porque ela volta assim que esse valor sai do YAML, e porque a
correção óbvia é a errada.

`captureBodyReference()` congela `pcd_` na posição do corpo do instante da
parada; os pés seguem escorregando depois disso. A referência congelada fica
deslocada do centroide de apoio pelo resíduo que a parada carregava, a QP recebe
um pedido de distribuição de força impossível, o resíduo de momento de guinada
satura em 100% e o robô dobra. `settleHoldPosture()` caminha `pcd_` de volta ao
centroide a 0,02 m/s, com limite de taxa para não pedir degrau de força.

| condição | `Syaw` no `HOLD` | `settle_rate` | parada de 180 s | `RECOVER` | `yawSat` |
|---|---|---|---|---|---|
| default antigo | 450 | 0 | colapso em 161,7 s | 74 | 100% |
| só baixar o peso | 100 | 0 | **colapso em 91,1 s** | **357** | 100% |
| **promovido** | 450 | 0,02 | **180 s de pé** | **0** | 10–66% |
| ambos | 100 | 0,02 | 180 s de pé | 0 | 54–90% |

Baixar `balance.weight_moment[2]` na parada parece a correção e **não é**:
antecipou o colapso em 70 s e multiplicou o `RECOVER` por cinco. Amolecer o
resíduo não remove o pedido de momento, remove o motivo de a QP distribuir força
contra ele. Não repita isso sozinho.

Sem `settle_rate`, o tempo até a queda escala com o resíduo com que a marcha
entrou na parada — leia `velErrXY` na primeira linha de `HOLD`:

| `velErrXY` ao entrar em `HOLD` | como se chegou lá | tempo até cair |
|---|---|---|
| 0,010 | reto, partida limpa | não caiu em 90 s |
| 0,020 | reto, após 5 ciclos | 46,1 s |
| 0,034 | após giro | 17,8 s |

Repetível, não determinístico — por isso uma condição só conta com o gatilho
fixo de §11, e nunca com n = 1.

Isso também explica por que o critério F4 verde usa paradas de **8 s**: em 8 s só
se vê o começo do transitório. Inspeção de uma parada curta, que é o critério de
hoje:

```bash
docker logs aquila-go2 2>&1 | grep "mode=HOLD" \
  | sed -E 's/.*tilt=([0-9.]+)deg.*footErr=\[(.*)\]/tilt=\1 err=[\2]/' | tail -20
```

Aceitação em parada de 8 s: `tilt` < 1°, `footErr` simétrico e < 2 cm, nenhum
`RECOVER`.

Para exercitar a parada longa de propósito, use `--final-hold` (§11): o harness
registra a queda em vez de abortar o ensaio.

---

## 8. Medir pose e deriva

Pose única, legível:

```bash
ros2 topic echo /demo/odom --field pose.pose --once
```

Pose com yaw em graus:

```bash
ros2 topic echo /demo/odom --field pose.pose --once | python3 -c '
import sys, math, re
t = sys.stdin.read()
g = lambda k: float(re.search(k + r": (-?[0-9.e-]+)", t).group(1))
x, y, z = g("x"), g("y"), g("z")
o = re.search(r"orientation:(.*)", t, re.S).group(1)
q = lambda k: float(re.search(k + r": (-?[0-9.e-]+)", o).group(1))
yaw = math.degrees(math.atan2(2 * (q("w") * q("z") + q("x") * q("y")),
                              1 - 2 * (q("y") ** 2 + q("z") ** 2)))
print(f"pos=({x:.3f},{y:.3f},{z:.3f}) yaw={yaw:.2f}deg")'
```

Os nomes `x`, `y`, `z` são extraídos antes da f-string de propósito: aspas
escapadas dentro de uma f-string não sobrevivem ao aninhamento de aspas do
shell.

`/demo/odom` é ground truth do Gazebo. O estimador interno do controlador
aparece como `estPos`/`estYaw` na linha de diagnóstico — comparar os dois é o
que separa "o robô caiu" de "o controlador acha que caiu".

Resumos rápidos de uma execução:

```bash
docker logs aquila-go2 2>&1 | grep -c "mode=RECOVER"                     # quedas
docker logs aquila-go2 2>&1 | grep "mode=WALK" \
  | sed -E 's/.*tilt=([0-9.]+)deg.*/\1/' | sort -g | tail -1             # tilt máx
```

---

## 9. Testes unitários

```bash
docker exec aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && cd /test
  colcon test --packages-select demo_simulation
  colcon test-result --verbose'
```

Esperado: **20 testes, 0 falhas**. Cobrem o mapeamento `Twist` → `Inputs`, o
watchdog e o latch de partida do trote.

---

## 10. Assinaturas de falha

Cada linha abaixo foi observada e diagnosticada nesta aplicação.

| Sintoma | Causa provável | Onde olhar |
|---|---|---|
| `state=fixed stand` para sempre, ponte publicando | handshake de partida perdido — o controlador lê struct, não stream | `_StartLatch` em `twist_to_inputs.py` |
| marcha no lugar, não translada | comando abaixo do ponto de projeto da marcha | tabela §5; use `linear.x ≥ 0.15` |
| pernas cruzam para o centro do corpo | padrão de apoio girando com a taxa de guinada | `k_yaw_` em `FeetEndCalc.cpp` |
| gira parado, arrasta os pés, tomba de pé | alvo de apoio obsoleto num par diagonal | `footErr` assimétrico; `GaitGenerator::generate` |
| anda para trás com comando positivo | QP perseguindo momento que não consegue entregar | `Mz` pedido ≫ realizado |
| cai no primeiro passo, `footErr` > 30 cm | PD de junta dominando o controlador de força | `calcGain`, deve ser 3,0/2,0 |
| `tilt` > 12° | supervisor entra em `RECOVER` e cancela o comando | esperado; investigue o que veio antes |
| tomba parado, 18–46 s depois de a marcha parar | **Defeito 2**: eixo de guinada no batente bombeando resíduo, e o QP trocando distribuição de força para perseguir um `Mz` inalcançável | `yawSat` e `velErrXY` na entrada do `HOLD`; §7.5 |
| `yawErr` cravado de um lado, sem cruzar zero | banda proporcional de 0,73° — o eixo é relé, não regulador | `trot.kp_w` e `ang_acc_limit_yaw` em `gait_go2.yaml` |
| `dWzMed` ≈ `dWzPk`, na casa das centenas | termo derivativo lendo vibração de tronco, ~50× a rotação real do corpo | `trot.kd_w`; é o experimento B1c |
| **anda curvando**, `yawErr` cresce monotonicamente e `Mz` fica cravado no teto | realimentação positiva na colocação de pé: o termo neutro segue a rotação medida e `k_yaw` não a cancela fora do toque do pé | `foot_placement.k_yaw` (≥ 0,25); `../results/ml35-caminhada-reta.md` |
| tombou com `kd_w` de guinada reduzido | o termo derivativo em 70 é amortecimento necessário na caminhada, não só ruído | resultado negativo já medido; não repetir B1c isolado |
| tomba parado depois de ~90–160 s, `yawSat` = 100% em toda a janela | referência de corpo congelada fora do centroide de apoio pelo resíduo da parada | `hold.settle_rate`; §7.5 |
| tomba parado **mais cedo** após baixar o peso de momento de guinada | amolecer o resíduo remove o motivo de a QP distribuir força, não o pedido de momento | resultado negativo já medido; não repetir isolado |
| **treme parado**, ~2,6° de guinada pico a pico | resíduo do eixo de guinada; anterior ao settle, já reduzido 2,7× pelo `k_yaw` | aberto; amplitude por parada em `../results/ml35-postura-parada.md` |
| **sai da linha andando com o rumo estável** (caranguejo, ~2% da distância) | **Defeito 1**: o robô rastreia fielmente um estimador que deriva; `k_y` foi varrido e refutado | compare `estPos` do supervisor com o `/demo/odom` real; §8 |

---

## 11. Sintonia da marcha e ensaio instrumentado

A sintonia do trote não está mais compilada em literais. Ela vive em
`ros2_ws/src/demo_simulation/config/gait_go2.yaml` e é injetada pelo spawner:

```bash
# usar outro arquivo, sem rebuild
ros2 launch demo_simulation quadruped.launch.py \
  gait_params:=/test/src/demo_simulation/config/minha_varredura.yaml
```

O caminho é o de **dentro** do container, e o arquivo tem de existir em
`ros2_ws/src/demo_simulation/config/` antes de a sim subir — `/proj/src` é
montado read-only e copiado na partida. Como fazer isso na prática está no fim
desta seção.

O arquivo **não** vai em `go2_description/config/gazebo.yaml`: aquele pacote é
vendorizado e o README dele garante que os configs estão intactos byte a byte —
essa garantia é o que sustenta o argumento de licença. O `<parameters>` do
plugin de hardware também está num xacro vendorizado, então o ponto de injeção
que é nosso é o spawner.

### Provar quais valores estão valendo

Duas linhas, e as duas importam. A primeira prova que o arquivo foi aplicado:

```bash
docker logs aquila-go2 2>&1 | grep "node arguments"
# ... --params-file .../go2_description/config/gazebo.yaml
#     --params-file .../demo_simulation/config/gait_go2.yaml
```

O nosso arquivo tem de aparecer **por último** — é isso que o faz sobrepor. A
segunda mostra o que o controlador realmente carregou:

```bash
docker logs aquila-go2 2>&1 | grep "gait params:"
# gait params: period=0.450 st_ratio=0.500 height=0.080 k=(0.0050 0.0050 0.1500)
#              kp_w=780.0 yaw_clamp=10.0 band=0.0100 S_moment=(450 450 450) mu=0.40
```

Sem a primeira linha, valores iguais aos defaults compilados são
indistinguíveis de "o arquivo não foi lido". Leia as duas antes de acreditar
numa varredura.

### Ensaio instrumentado

`scripts/gait_trial.sh` dirige o ensaio e grava a evidência, substituindo o
`for` de `ros2 topic pub` do §7.2:

```bash
./scripts/gait_trial.sh /tmp/run.csv --v-cmd 0.10 --cycles 5 --walk 8 --hold 8
```

O que ele faz que a versão manual não fazia:

| | Por quê |
|---|---|
| aborta se `z <= --min-z` (0,30 m), antes do ensaio e antes de cada ciclo | uma varredura inteira já foi perdida medindo um robô tombado sendo arrastado, com números plausíveis |
| paginação de fases pelo **tempo de simulação**, e as duas bases de tempo na mesma linha do CSV | 8 s de parede só são 8 s de simulação em RTF 1 |
| para de publicar em vez de publicar zeros | o watchdog da ponte é o mecanismo de parada; publicar zero testa um caminho que o robô nunca percorre |
| `--v-cmd`/`--w-cmd` em SI, com a conversão de stick aplicada e o excesso recusado | `v_cmd = 0,4 × linear.x`; pedir além do envelope seria clampado em silêncio |
| `--final-hold N` fica parado N s depois do último ciclo e **registra** a queda em vez de abortar | nesse trecho cair é a medição, não violação de precondição |

Resumo no stderr (comprimento de trajetória, deslocamento líquido, deriva de
rumo, tilt de pico por fase, RTF); CSV no arquivo.

As linhas de cabeçalho do resumo cobrem **só os ciclos de andar/parar**. O
`--final-hold` é outro experimento — lá o robô pode cair — e um corpo
escorregando de costas soma metros de "trajetória" e arrasta a velocidade média:
com as duas janelas somadas, uma corrida reportou 0,0222 m/s onde o valor correto
era 0,0629. A parada final aparece em linha própria, numa das duas formas:

```
final hold     survived 89.9 s standing
final hold     COLLAPSED at 39.4 s (z <= 0.30 m)
```

### Contar `RECOVER` do ensaio, e não da sessão

`grep -c mode=RECOVER` sobre o log inteiro conta tudo, inclusive o que aconteceu
antes e depois do ensaio — e o que acontece **depois** é o Defeito 2: deixado em
HOLD sem comando, o robô tomba entre 18 s e 46 s. Uma corrida da Fase A devolveu
23 `RECOVER` com tilt de pico de 2,58° na janela do ensaio; os 23 eram de 31 s
depois do último ciclo.

Recorte pela janela de parede que o próprio CSV registra:

```bash
ini=$(awk -F, 'NR==2{print int($4)}' /tmp/run.csv)
fim=$(awk -F, 'END{print int($4)+1}' /tmp/run.csv)
docker logs aquila-go2 2>&1 | grep mode=RECOVER \
  | awk -v a="$ini" -v b="$fim" -F'[][]' '{t=int($6)} t>=a && t<=b' | wc -l
```

### Gatilho do Defeito 2 — fixe-o antes de comparar qualquer condição

A primeira linha de base **não reproduziu o defeito**: 3 ciclos a
`v_cmd = 0,10` seguidos de 90 s de `HOLD` ficaram de pé, com tilt de pico de
0,23°. Coerente com o mecanismo (o tempo até a queda escala com o resíduo) e
fatal para a comparação — com n = 1 por condição e um defeito estocástico,
qualquer "melhora" contra essa base seria indistinguível de sorte.

O gatilho saiu do pior caso documentado, caminhada **com giro**:

```bash
./scripts/gait_trial.sh /tmp/b0.csv --v-cmd 0.10 --w-cmd 0.10 \
  --cycles 1 --walk 15 --hold 0 --final-hold 90
```

Com ele a base cai aos **39,4 s**, dentro da faixa registrada, com
`yawSat = 100%` constante nas últimas linhas de `HOLD`. Mexer em `--walk`,
`--w-cmd` ou `--final-hold` muda a probabilidade de queda e invalida a
comparação: use exatamente este gatilho entre condições.

E ele **não** substitui §7.2. O gatilho mede a parada longa; o critério F4 que os
quatro experimentos rejeitados quebraram é o roteiro de andar/parar. Uma condição
só vira default depois de passar nos dois.

### Varredura de uma linha de YAML

O caminho mais curto, que funciona com `run_quadruped_sim.sh` sem alteração:
edite o valor em `ros2_ws/src/demo_simulation/config/gait_go2.yaml` **no host,
antes de subir a sim** — o script copia `ros2_ws/src` para dentro do container na
partida. Confirme com as duas linhas de log acima, rode o gatilho, e desfaça com
`git checkout` no fim.

> `run_quadruped_sim.sh` aceita **só um caminho de mundo** como `$1`; ele não
> repassa `gait_params`. `./scripts/run_quadruped_sim.sh gait_params:=/tmp/x.yaml`
> vira `world:=gait_params:=/tmp/x.yaml` e o Gazebo falha ao carregar o mundo.
> Para usar um arquivo alternativo por nome, é o `ros2 launch ... gait_params:=`
> acima, executado dentro do container.

Exemplo com o experimento B1a, baixar só a entrada de guinada do peso de momento
do QP (`balance.weight_moment: [450, 450, 100]`) — uma linha, sem rebuild.
Resultado medido, **n = 1**: sobreviveu 89,9 s onde a base caiu aos 39,4 s, zero
`RECOVER`, `yawSat` de 54–84% em vez de 100% cravado, e caminhada dentro de 3%
em trajetória, tilt e `z`. O custo caiu na guinada, como previsto: rastreamento
de 76% contra 85%, e +13,5° de deriva de rumo em 90 s parado.

É **sinal, não conclusão** — n = 1, e o roteiro de §7.2 não foi rodado sob B1a.
Por isso `gait_go2.yaml` continua em 450. A tabela de experimentos da Fase B
(B1a, B1b, B1c, B2, B3) está em `../ml35/plano-movimentacao.md`.

---

## 12. Encerrar

```bash
docker stop aquila-go2
```

O container é `--rm`; nada persiste. Para reconstruir depois de mexer em
`ros2_ws/src`, basta subir de novo — o script sempre roda `colcon build`.
