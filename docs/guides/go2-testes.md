# Go2 — guia de comandos para testar a aplicação

Roteiro de operação do spike do quadrúpede em simulação. Tudo aqui roda na
**workstation x86**, nada no módulo Aquila — Gazebo é OGRE 2 e não pode subir no
AM69 (ver `CLAUDE.md`, regra 1).

Estado da aplicação em 18/08/2026: o robô caminha e para de forma repetível.
Evidência em `docs/results/ml35-f4-parcial.md`; contexto de implementação em
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

Guinada comandada **ainda não foi validada** — é item aberto de F4.

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
```

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

Notas de leitura que custaram caro para descobrir:

- `estPos(2)` fica ~24 mm **abaixo** do real. É viés conhecido e constante
  (`foot_radius = 0.02` contra `feet_h_ = 0` no estimador), não deriva.
- `Mz` travado em ±5,3 com sinal alternando é o **batente**, não controle. É o
  teto de guinada do QP neste robô — esperado, não é falha.
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

O rumo **não** é critério: sem comando de guinada o trote faz passeio aleatório
por projeto (rumo final de 12° a 43° entre execuções). Fechar essa malha é papel
do Nav2.

### 7.3 Watchdog de comando

```bash
timeout 5s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

Aceitação: em até 0,3 s após o `timeout`, o supervisor mostra `mode=HOLD` com
`cmd=(0.0000,0.0000,0.0000)`. O robô **não** pode continuar andando.

### 7.4 Postura parada prolongada

Com o robô já de pé e sem comando, deixe 60 s e verifique:

```bash
docker logs aquila-go2 2>&1 | grep "mode=HOLD" \
  | sed -E 's/.*tilt=([0-9.]+)deg.*footErr=\[(.*)\]/tilt=\1 err=[\2]/' | tail -20
```

Aceitação: `tilt` < 1°, `footErr` simétrico e < 2 cm, nenhum `RECOVER`.

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

---

## 11. Sintonia da marcha e ensaio instrumentado

A sintonia do trote não está mais compilada em literais. Ela vive em
`ros2_ws/src/demo_simulation/config/gait_go2.yaml` e é injetada pelo spawner:

```bash
# usar outro arquivo, sem rebuild
ros2 launch demo_simulation quadruped.launch.py \
  gait_params:=/test/src/demo_simulation/config/minha_varredura.yaml
```

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

Resumo no stderr (comprimento de trajetória, deslocamento líquido, deriva de
rumo, tilt de pico por fase, RTF); CSV no arquivo.

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

Para exercitar o Defeito 2 de propósito, é o contrário: rode o ensaio e **deixe
o HOLD correr** por 90 s depois do último ciclo, sem publicar nada.

---

## 12. Encerrar

```bash
docker stop aquila-go2
```

O container é `--rm`; nada persiste. Para reconstruir depois de mexer em
`ros2_ws/src`, basta subir de novo — o script sempre roda `colcon build`.
