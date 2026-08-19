# ML3.5 F4 — checkpoint parcial

Data: 17/08/2026. Host x86 (`diegom-nb`). **F4 em andamento; portão ainda não
batido.** Este arquivo registra o ponto exato de parada para a próxima sessão.

## Implementado neste checkpoint

- `demo_simulation/urdf/go2_sim.urdf.xacro`: wrapper nosso sobre o Go2
  vendorizado. Anexa câmera e lidar aos links que já existiam no modelo, sem
  trazer `gz_quadruped_playground`.
- O mesmo wrapper adiciona `OdometryPublisher` de ground truth do Gazebo em
  `/go2/odom`. Esta é uma fonte **temporária de F4**; seu TF não é bridged.
  Odometria por pernas e `odom -> base_link` continuam sendo F5.
- `config/bridge_quadruped.yaml`: `/clock`, `/demo/odom`, `/demo/scan`,
  `/demo/camera/image_raw`, `/demo/camera/camera_info` e `/demo/imu`.
- `worlds/quadruped_empty.sdf`: mundo sem assets externos, mas com o sistema
  `gz-sim-sensors-system`. O `empty.sdf` embutido não tem esse sistema e deixa
  câmera/lidar mudos sem erro.
- `twist_to_inputs`: sinais lateral e yaw corrigidos para compensar as
  inversões de `StateTrotting`; comandos saturados no envelope de stick
  comprovado em F3, `[-0.03, 0.03]`.
- Testes unitários do mapeamento e testes estáticos dos nomes/tipos do contrato.
- `demo_perception` permaneceu intocado.

## Validação executada

### Build, xacro e testes

- Os seis pacotes do quadrúpede compilaram na imagem
  `local/demo-aquila-sim:f3`.
- `go2_sim.urdf.xacro GAZEBO:=true` expandiu sem erro.
- `gz sdf -k worlds/quadruped_empty.sdf`: `Valid.`
- A rodada anterior à última edição passou **6 testes, 0 falhas** em
  `demo_simulation` (2 linters + 4 testes de mapeamento). Depois disso foram
  adicionados 2 testes estáticos do contrato e o clamp seguro foi alterado;
  portanto a suíte final de 8 testes **ainda precisa ser repetida**.

Os avisos de API deprecated em `gz_quadruped_hardware` permanecem os mesmos de
F3 e não são falha de build.

### Contrato atravessando dois containers

Simulador no domínio 171 e `local/demo-aquila-tools:dev` como consumidor, ambos
em `network=host` e CycloneDDS:

| Tópico | Tipo observado no segundo container | Mensagem real |
|---|---|---|
| `/demo/cmd_vel` | `geometry_msgs/msg/Twist` | publicação atravessou DDS e moveu a planta |
| `/demo/odom` | `nav_msgs/msg/Odometry` | sim, `frame_id=odom` |
| `/demo/scan` | `sensor_msgs/msg/LaserScan` | sim, `frame_id=lidar` |
| `/demo/camera/image_raw` | `sensor_msgs/msg/Image` | sim, `frame_id=front_camera` |
| `/demo/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | tópico e tipo observados |
| `/demo/imu` | `sensor_msgs/msg/Imu` | tópico e tipo observados |

Isso prova troca real para odom, scan e imagem; não foi inferido apenas de
`ros2 topic list`.

## Premissa que caiu: escalar até o limite matemático do controlador

O comentário de F3 dizia que F4 deveria dividir Twist pelos limites internos de
`StateTrotting` (0,4 m/s, 0,3 m/s e 0,5 rad/s). Foi implementado e executado. Um
comando aparentemente baixo, `linear.x=0.03`, virou `ly=0.075` e derrubou o
Go2 em dois segundos:

| | Antes | Depois |
|---|---:|---:|
| x | 0,060 m | -0,612 m |
| y | -0,014 m | -0,486 m |
| z | **0,355 m** | **0,073 m** |
| orientação | praticamente identidade | tombada (`qx=0,915`) |

O processo e os controladores não reportaram erro. O deslocamento grande era
queda/deslizamento, não marcha rápida.

A última edição voltou ao ganho unitário comprovado em F3 e acrescentou
saturação em `|stick| <= 0.03`. **Essa edição ainda não foi reexecutada na
planta.** Não aumentar o limite baseado apenas nos valores matemáticos do
controller.

## Renderização dos sensores nesta máquina

`gui:=false` sem `DISPLAY` falhou no Sensors/Ogre2: a GPU é NVIDIA
(`10de:28b9`) e mapear somente `/dev/dri` não fornece EGL headless dentro da
imagem. Gazebo terminou com `eglInitialize failed` / OpenGL 3.3 e segfault do
thread de sensores.

`gui:=true`, `DISPLAY=:1`, socket X11, `/dev/dri` e grupo 992 funcionou: câmera,
lidar e odometria publicaram. O próximo teste seria `gui:=false` **mantendo**
`DISPLAY=:1` e o socket X11, para o servidor renderizar sensores via X/GLX sem
abrir janela. O comando foi abortado antes de criar o container; não ficou
nenhum `aquila-f4-contract` ativo.

## Próximos passos, em ordem

1. Repetir build e os 8 testes de `demo_simulation` no container.
2. Subir `quadruped_empty.sdf` com `gui:=false`, mas `DISPLAY=:1` e X11
   montado. Confirmar que odom, scan e imagem continuam publicando.
3. Repetir `linear.x=0.03` por 10–12 s com o clamp atual. Medir pose antes/depois;
   critério: `z ~= 0.35 m`, orientação nivelada e x monotônico. Se cair, parar
   e reduzir o envelope — não mascarar pela ausência de erro no log.
4. Subir o container `perception` no mesmo domínio. Confirmar mensagens reais
   em `/demo/perception/detections`; o código de `demo_perception` não deve ser
   alterado.
5. Quando a rede permitir, reconstruir `base` e `sim` pelo compose oficial e
   executar `robot_type:=quadruped` no `warehouse.sdf`. A imagem do spike não
   contém `nav2_minimal_tb4_sim`.
6. Repetir `--profile learn` com o diff-drive e exigir goal Nav2 `SUCCEEDED`.
7. Só então atualizar F4 para concluída e iniciar F5 (`odom -> base_link`,
   odometria por pernas e Nav2).

## Ainda não validado

- clamp `0.03` atual em runtime;
- perception consumindo a nova câmera;
- `warehouse.sdf` com o Go2;
- imagem `sim` oficial reconstruída;
- regressão diff-drive;
- RViz2 do Go2;
- qualquer execução arm64 ou no Aquila AM69.

## Execução adicional — 18/08/2026

O ensaio com `world:=empty.sdf` confirmou dois problemas distintos:

- sem `/demo/cmd_vel`, o robô entrava automaticamente em `TROTTING` após
  15 s e podia cair por deriva de velocidade/yaw;
- ao aplicar `linear.x=0.03`, a passada dinâmica ainda tombava em cerca de 20 s,
  mesmo após sintonia conservadora temporária.

Foi corrigido no workspace o primeiro problema: `twist_to_inputs` agora para em
`FIXEDSTAND` e só envia `command=4` quando recebe um `Twist` não nulo. A
execução runtime ficou em `fixed stand` até t≈61 s, com `z=0,347 m`, orientação
nivelada e velocidade zero.

Também foi corrigida a conversão de posição/velocidade dos pés no estimador
vendorizado para o frame global, incluindo a velocidade angular do corpo. Build
limpo e suíte passaram: **10 testes, 0 falhas**.

O trote permanece bloqueado: a transformação do estimador melhora o repouso,
mas a passada ainda cai. Não marcar F4 como concluída nem aumentar o limite de
`_SAFE_STICK_LIMIT` até haver retuning/teste específico do controlador dinâmico.

### Reensaio de movimentação — 18/08/2026

Com o container identificado (`lucid_bohr`) e o comando
`linear.x=0.03` publicado a 10 Hz, o Go2 avançou sem alternância de pernas
observável e caiu. Isso confirma que o tópico DDS e o caminho
`/demo/cmd_vel -> /control_input` estão funcionando; a falha está na marcha
dinâmica (gait/estimador/ganhos), não no throttle nem na inicialização.

Próximas etapas são deliberadamente separadas:

1. Medir cada ciclo de trote com `/demo/odom`, IMU e pose do modelo, começando
   por um único comando curto e sem yaw; não aumentar o limite seguro de 0,03.
2. Instrumentar as fases de swing/stance e revisar o sinal dos pés e o ganho
   de yaw no `unitree_guide_controller`; validar primeiro trote parado, depois
   avanço de 0,01 m/s.
3. Repetir o teste no `warehouse.sdf` somente após o trote ficar estável no
   `quadruped_empty.sdf`. O script `scripts/run_quadruped_sim.sh` aceita o
   caminho absoluto do cenário e mantém a execução reproduzível.
4. Com a rede disponível, instalar/vendoriar o mundo oficial no compose,
   executar `robot_type:=quadruped` e então confirmar RViz2, TF e sensores.
5. Rodar `--profile learn` e exigir goal Nav2 `SUCCEEDED` para fechar a
   regressão do diff-drive antes de declarar F4 concluída.

### Step seguinte — ganho de stance no trote

O teste de parada confirmou `state=trotting`, `ly=0` e
`contact=[1 1 1 1]` durante toda a janela. Assim, a queda não depende da
alternância de pés: ela começa quando `FIXEDSTAND` entrega o controle ao
`TROTTING`. A próxima hipótese isolada é a descontinuidade de ganhos
`80/3.5 -> 0.8/0.8`. O workspace passa a usar `Kp=3.0`, `Kd=2.0` também para
stance, igual ao swing. Período, estimador e QP permanecem inalterados; o
resultado deve ser medido novamente antes de qualquer ajuste adicional.

## Separação WALK / HOLD / RECOVER — 18/08/2026

Todos os ensaios abaixo em `quadruped_empty.sdf`, host x86, container
`aquila-go2` subido por `./scripts/run_quadruped_sim.sh`, com o código de
`StateTrotting` e `twist_to_inputs` desta data.

### O que estava errado, medido e não inferido

O `checkStepOrNot()` do upstream só pede passada acima de `|v| > 0.03 m/s`. O
caminho de comando inteiro não alcança esse valor:

```text
Twist.linear.x = 0.03      (máximo do plano de teste)
  -> stick ly = 0.03       (clamp de twist_to_inputs, ganho unitário)
  -> invNormalize(-0.4, 0.4) = 0.4 * 0.03 = 0.012 m/s
```

0,012 m/s é 2,5× menor que o gatilho. Os critérios de erro também não podiam
disparar: o gatilho pede `|posError| > 0.08 m`, mas `pcd_` é saturado em
`posBody ± 0.05 m` uma linha antes. Por isso o registro anterior deste arquivo
mostra `state=trotting`, `ly=0.03` e `contact=[1 1 1 1]` durante toda a janela:
**nenhuma passada foi pedida**. Sintonizar ganhos contra aquele log não mediria
nada.

### Ensaio A — `linear.x=0.01` por 4 s, depois silêncio

```bash
timeout 4s ros2 topic pub -r 20 /demo/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.01}}"
```

| | Antes | Depois (6 s de silêncio) |
|---|---:|---:|
| x | 0,0424 m | 0,0088 m |
| y | 0,0018 m | 0,0411 m |
| z | 0,3469 m | 0,3600 m |
| orientação | nivelada | yaw −2,1°, nivelada |

Log:

```text
mode=WALK cmd=(0.0040,-0.0000,0.0000) tilt=2.0deg contact=[0 1 1 0]
mode=WALK cmd=(0.0040,-0.0000,0.0000) tilt=0.6deg contact=[0 1 1 0]
cmd_vel watchdog: stale, holding position
mode=HOLD cmd=(0.0000,0.0000,0.0000) tilt=0.3deg posErrXY=0.0018 contact=[1 1 1 1]
```

Resultado: passada real, watchdog disparando 0,3 s após o publisher morrer,
`HOLD` com quatro pés no chão e `posErrXY` entre 0,0018 e 0,0077 m estável por
mais de 35 s. **Sem movimento residual** — era o sintoma de `pcd_` integrado.

O deslocamento líquido não prova avanço: 4 s × 0,004 m/s = 16 mm comandados,
menor que a deriva do próprio trote (34 mm em x, no sentido contrário).

### Ensaio B — `linear.x=0.03` por 20 s

| | Antes | Depois |
|---|---:|---:|
| x | 0,0056 m | −0,359 m |
| y | 0,0393 m | 0,760 m |
| z | 0,3598 m | **0,152 m** |
| orientação | nivelada | tombada (`qx=0,784`) |

```text
mode=WALK  tilt=0.7deg  posErrXY=0.0067 contact=[1 0 0 1]
mode=WALK  tilt=1.5deg  posErrXY=0.0160 contact=[1 0 0 1]
mode=WALK  tilt=4.6deg  posErrXY=0.0301 contact=[0 1 1 0]
mode=RECOVER tilt=19.5deg  -> 102deg -> 128deg
```

Duas leituras: a alternância `[1 0 0 1]` ↔ `[0 1 1 0]` prova swing físico dos
pares diagonais, e a queda leva cerca de 3 s de trote. O `RECOVER` detectou e
zerou o comando (`posErrXY=0.0000` confirma a recaptura contínua da
referência), mas quatro pés em stance não seguram uma queda que já estava
balística: entre 4,6° e 19,5° passou menos de 1 s.

### Ensaio C — `linear.x=0.01` por 30 s, container reiniciado

| | Antes | Depois |
|---|---:|---:|
| x | 0,0462 m | −0,0149 m |
| y | 0,0020 m | 0,266 m |
| z | 0,3468 m | **0,226 m** |
| orientação | nivelada | tombada de lado |

Trote sustentado por cerca de 8 s, com `tilt` oscilando 1,1 → 6,0 → 3,0 → 1,1
→ 3,0 → 3,0 → 3,2 → 9,0° e então queda. Durante o trote, `velErrXY` ficou entre
0,03 e 0,09 m/s **com comando de 0,004 m/s**: o corpo é arremessado pela própria
passada, não pelo comando.

### O que ficou provado e o que não ficou

| Item | Estado |
|---|---|
| gait ativa no comando mais lento do plano | ✅ `mode=WALK` em `Twist 0.01` |
| pernas levantam de fato, em pares diagonais | ✅ `[1 0 0 1]` ↔ `[0 1 1 0]` |
| parada zera o comando sem depender do publisher | ✅ watchdog em 0,3 s |
| `HOLD` sem movimento residual | ✅ `posErrXY ≈ 0,005 m` por >35 s |
| supervisor de atitude detecta perda de postura | ✅ `RECOVER` em 12° |
| `RECOVER` **recupera** a postura | ❌ queda continua |
| anda em `0.01` sem cair | ❌ cai em ~8 s |
| anda em `0.03` sem tombar | ❌ cai em ~3 s |
| `/demo/odom` monotônico em x | ❌ deriva domina |

F4 continua aberta. O trote dinâmico é agora o único bloqueador isolado — antes
havia três sobrepostos.

### Próximo experimento isolado, em ordem

Um variável por ensaio, mesma janela de 30 s em `linear.x=0.01`, mesmo mundo.

1. **Ganho de stance de volta ao upstream (`Kp=0.8`, `Kd=0.8`).** O valor atual
   `3.0/2.0` foi escolhido contra o log em que **nenhuma passada era pedida** —
   aquela evidência não vale mais. Em trote, a perna de apoio é controlada por
   força pelo QP; um PD de junta rígido perseguindo um alvo de pé congelado
   briga com ele.
2. **`BalanceCtrl::Ib_` é do A1, fixo no código.** `Vec3(0.0792, 0.2085, 0.2265)`
   com `pcb_ = (0,0,0)`. O tronco do Go2 é `(0.02448, 0.098077, 0.107)` com CoM
   em `(0.0211, 0, −0.0054)` (`go2_description/xacro/const.xacro`), e a massa
   total vem do URDF, ou seja está correta e a inércia não. O QP pesa erro
   angular em 450 contra 20 do linear: essa constante domina o controle de
   atitude. Atenção ao comparar: o valor do A1 é inércia de corpo inteiro, não
   só do tronco, então não basta copiar o tronco do Go2 no lugar.
3. **`stance ratio` 0,50 → 0,60** (`WaveGenerator(0.4, 0.6, ...)`), só depois
   dos dois acima.

Registrado para ninguém procurar no lugar errado: `feet_pos_normal_stand_`
também tem números do A1, mas **não está no caminho** — `FeetEndCalc::init()`
usa `estimator_->getFeetPos2Body()` e a linha da constante está comentada.

## Experimentos isolados sobre a marcha — 18/08/2026

Mesma janela em todos: `linear.x=0.01` a 20 Hz por 30 s, `quadruped_empty.sdf`,
container reiniciado antes de cada ensaio. Linha de base a bater: queda em ~8 s,
`tilt` oscilando até 9°.

### Falha silenciosa encontrada primeiro: handshake de partida perdido

O primeiro ensaio não andou, e não por dinâmica: o robô ficou em `fixed stand`
com `sticks=(ly=0.0100)` fluindo e `command=0`. O watchdog introduzido nesta
data publica a 20 Hz; a ponte enviava `command=4` **uma vez**, e a mensagem
seguinte do tick sobrescrevia o campo antes de qualquer iteração do
`update()` do controlador ler a struct `control_inputs_`.

O controlador não consome mensagens, consome o **último valor** da struct. Um
handshake de uma mensagem só funciona enquanto ninguém mais publica no tópico.
E a falha é muda: a ponte loga "fixed stand -> trotting", os sticks continuam
chegando, e o robô fica de pé parado.

Corrigido com `_StartLatch`: o comando de partida é repetido por 0,5 s (10
ticks). Repetir é idempotente porque `StateTrotting::checkChange` trata
qualquer valor que não seja 1 nem 2 como "continua trotando". 4 testes novos;
suíte do `demo_simulation` em **20 testes, 0 falhas**.

### Ensaio 1 — ganho de stance do upstream (`Kp=0.8`, `Kd=0.8`): **pior**

| | Antes | Depois |
|---|---:|---:|
| x | 0,0778 m | −0,247 m |
| z | 0,3468 m | tombado |

```text
Switched from fixed stand to trotting     t=383.59
mode=RECOVER tilt=3.2deg                  t=384.60
mode=RECOVER tilt=157.3deg                t=385.61
```

Não sustentou `WALK` por sequer um ciclo de diagnóstico. Contra a linha de base
de ~8 s com `3.0/2.0`, a hipótese de que o PD rígido de stance brigava com o QP
**cai**: nesta planta o stance depende do PD de junta mais do que o upstream
supõe. Revertido para `3.0/2.0`.

Registro do raciocínio que não sobreviveu ao ensaio: o argumento era correto
sobre o upstream (lá o stance é dominado por força), mas a base vendorizada usa
`gz_quadruped_hardware`, cujo PD entra somado ao torque comandado — o balanço
entre os dois canais não é o mesmo do robô real.

### Ensaio 2 — inércia do corpo do Go2 em `BalanceCtrl`: **melhora, não resolve**

`calVectorBd` usa `Ib_` como `R·Ib_·Rᵀ·dWbd`, ou seja é a inércia do robô
inteiro em torno do CoM, no frame do corpo. O valor que sobreviveu ao port
A1 → Go2 é do A1 e **subestima este robô em 2,3× em todos os eixos**: o QP pede
43% do momento necessário para frear uma inclinação.

Calculado do `go2_description` na pose de stand (hip 0, thigh 0,8, calf −1,5),
somando os 18 links com o teorema dos eixos paralelos: 15,098 kg, CoM
`(−0,0016, 0, −0,0231)` m, diagonal `(0,1817, 0,4899, 0,5262)`. Termos fora da
diagonal ficam abaixo de 4% e são descartados, como no upstream.

| | `tilt` por segundo | queda |
|---|---|---|
| base | 2,1 4,2 3,3 0,3 0,8 7,4 | 7 s |
| com `Ib_` do Go2 | 0,6 0,4 0,4 1,0 1,5 1,9 6,0 | 8 s |

Mantido: atitude mais calma e 1 s a mais. Não é a causa da queda.

### Ensaio 3 — banda da referência `pcd_` de 0,05 → 0,01 m: **elimina o runaway**

`pcd_` é referência de posição integrada, saturada em `posBody ± 0,05 m`. Com
`Kpp = 70`, essa saturação autoriza `3,5 m/s²` de aceleração horizontal — um
terço da gravidade — num robô comandado a 4 mm/s. Todos os ensaios anteriores
mostram `posErrXY` subindo até exatamente 0,05 e **grudando lá**: a passada não
propulsiona o corpo, a referência continua integrando, satura, e o QP passa a
empurrar no máximo.

| | `posErrXY` | `tilt` por segundo | queda |
|---|---|---|---|
| banda 0,05 | 0,017 → 0,056 (grudado) | 0,6 0,4 0,4 1,0 1,5 1,9 6,0 | 8 s |
| banda 0,01 | 0,005 → 0,013 | 0,2 0,3 0,2 0,2 0,3 1,6 | ~7 s |

Atitude mais plana de todos os ensaios, e o runaway desapareceu. **Não compra
tempo de sobrevivência**, mas muda a forma da falha: a queda agora acontece de
forma abrupta a partir de um estado nivelado e bem rastreado (1,6° → 18,1°
dentro de um segundo), em vez de no fim de uma divergência lenta.

### Onde a investigação parou, e por quê

Três hipóteses foram testadas e nenhuma é a causa: ganho de stance (pior),
inércia (melhora marginal), banda de referência (limpa, sem ganho de tempo).
O que restou é uma falha **discreta**, não um desequilíbrio acumulado: o robô
está nivelado a 0,3°, com erro de posição de 1 cm, e perde tudo em menos de um
segundo.

Candidatos, em ordem, e o que instrumentar antes de mexer em código:

1. **Pé tropeçando.** `FeetEndCalc::calcFootPos` força `foot_pos(2) = 0.0`, ou
   seja, o alvo de apoio é o solo em `z` **global**, assumindo solo em zero e
   estimador de `z` correto. Se o `z` estimado derivar, o alvo entra no chão.
   Instrumentar: `z` de cada pé alvo contra `z` medido, por ciclo.
2. **Deriva do estimador.** `/demo/odom` é ground truth do Gazebo, mas o
   controlador usa o estimador interno. Comparar os dois durante o trote
   separa "o robô caiu" de "o controlador acha que caiu".
3. **Limite de junta / singularidade de IK** na perna em extensão.

Não sintonizar mais nada antes de (1) e (2). Os três ensaios acima mostram que
tuning cego já esgotou o que tinha a dar.

## O robô anda — 18/08/2026

Fecha a investigação aberta na seção anterior. As duas hipóteses que ficaram
como "instrumentar antes de mexer" foram medidas, ambas **refutadas** como
causa, e a medição levou à causa real.

### Medição 1 — estimador contra ground truth: o estimador está certo

`/demo/odom` (Gazebo) contra o estado interno do estimador, durante um trote,
amostrados a 4 Hz:

| t (s) | yaw ground truth | yaw estimador |
|---|---|---|
| 5,5 | +3,9° | +3,4° |
| 6,7 | −5,2° | −6,0° |
| 8,0 | +1,8° | +0,3° |
| 8,5 | +10,2° | +9,4° |
| 10,0 | −16,4° | −23,7° |
| 10,5 | −35,7° | −35,5° |

O estimador rastreia o ground truth. **Não há deriva.** O que existe é uma
oscilação real de guinada, divergente, dobrando de amplitude a cada meio-ciclo
(±0,5° → ±4° → ±10° → −36° → queda), com **comando de yaw igual a zero**. Roll
e pitch ficam em ±1° o tempo todo — a falha é de rumo, não de atitude.

Achado secundário: o `z` estimado fica ~24 mm abaixo do real, **constante**, não
derivando. A causa é `foot_radius = 0.02` no `go2_description`: o frame do pé é
o centro da esfera de colisão, mas `Estimator::update()` mede `feet_h_ = 0` para
o pé em contato. Consequência: o corpo é estimado 2 cm mais baixo, e todo alvo
de balanço em `calcFootPos` (`foot_pos(2) = 0.0`) fica 2 cm abaixo do solo real.
Não é regressão do Go2 — o A1 também tem `foot_radius = 0.02` — mas é um erro
real. Não foi corrigido nesta rodada: não é a causa da queda, e mexer nele junto
com a correção de rumo criaria confundimento.

### Medição 2 — o QP: o "sinal de controle" de guinada era o batente

Instrumentado `bd_` contra `A_ * F_` em `BalanceCtrl` (momento pedido contra
realizado). Durante toda a caminhada saudável:

```
WALK tilt=0.5 yaw=-0.4 Mz_pedido=-5.3 Mz_real=-5.2
WALK tilt=0.4 yaw=-3.6 Mz_pedido= 5.3 Mz_real= 5.2
WALK tilt=0.2 yaw= 6.0 Mz_pedido=-5.3 Mz_real=-5.2
WALK tilt=1.0 yaw= 8.4 Mz_pedido=-5.2 Mz_real=-5.1
```

`Mz_pedido` travado em ±5,3 N·m alternando de sinal, com o erro de guinada
crescendo. 5,3 N·m = 10 rad/s² × `Izz = 0,5262` — é exatamente o batente de
`StateTrotting::calcTau`:

```cpp
d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));
```

Com `kp_w_ = 780`, o termo proporcional satura esse batente com `10/780` =
0,0128 rad = **0,73° de erro de guinada**. Acima disso o eixo é bang-bang, e
quem escolhe o sinal é o termo derivativo reagindo à ondulação do giroscópio na
frequência da marcha — por isso o sinal alterna enquanto o erro permanece em
+6°, +8°, +9°. O termo proporcional nunca corrige o rumo.

### Ensaio 4 — alargar o batente de yaw para ±25 rad/s²: **pior**

Motivado por uma estimativa de cone de atrito (2 pés × 0,4 × 74 N × 0,235 m ≈
14 N·m ⇒ 26 rad/s²). Resultado: WALK caiu de 7 s para 4 s e o robô andou **para
trás** (x final = −0,85 m).

A medição mostra por quê: o pedido gruda no novo batente (±13,1 N·m) e o
realizado despenca atrás dele — 13,2→7,3, 13,5→5,8, −13,3→−3,6. **O QP não
entrega 13 N·m.** A estimativa era otimista: supôs que toda a força tangencial
vai para guinada, quando ela também precisa propelir e equilibrar. Como o
resíduo de momento pesa 450 contra 20/20/50 nas forças lineares, perseguir o
momento impossível destrói a distribuição de força linear.

**O batente de ±10 rad/s² do upstream está bem dimensionado.** Revertido.

### Ensaio 5 — ganhos de atitude por eixo (`Kp_yaw=100`, `Kd_yaw=20`): **não incluído**

Com o batente correto, desmembrar `kp_w_` por eixo tira o yaw da saturação e o
rumo passa a se recuperar no meio do trajeto (−19° → −4°) em vez de divergir
monotonicamente. Mas **não faz o robô andar mais tempo** (5,5 s contra 7 s), e a
razão está na mesma medição: o QP topa em ~5,3 N·m de momento de guinada. Nenhum
ganho de atitude regula rumo aqui. Revertido para o upstream; a análise ficou
registrada em comentário no código para não ser refeita.

### Ensaio 6 — `k_yaw_` de 0,005 → 0,15 em `FeetEndCalc`: **resolve**

Rumo em robô com pernas se controla com onde o pé pousa, não com torque de
atitude. Em `calcFootPos`:

```
next_yaw = d_yaw·(1−phase)·t_swing + d_yaw·t_stance/2 + k_yaw_·(0 − d_yaw)
next_step += raio · [cos, sin](yaw + ângulo_inicial + next_yaw)
```

Os dois primeiros termos giram o ponto de pouso de cada pé em torno do corpo
pela taxa de guinada que o corpo **já tem** — colocação neutra, que preserva o
giro. No instante do pouso (`phase → 1`) o coeficiente é `t_stance/2 = 0,1125`,
então a 1 rad/s o padrão de apoio inteiro é assentado 6,4° rodado e o par
diagonal em balanço varre para dentro: **as pernas cruzam em direção ao centro**
(sintoma observado visualmente), o corpo guina mais, e a próxima colocação gira
mais. Realimentação positiva.

`k_yaw_ = 0.005` é o único termo que se opõe a isso, valendo 4% do que precisa
cancelar. `0.15` cancela os 0,1125 do pouso e deixa margem corretiva.

Resultado, comando `linear.x = 0.25` (→ `v_cmd = 0,1 m/s`) por 30 s:

| | antes | depois |
|---|---|---|
| duração em WALK | 7 s | **30 s, todo o teste** |
| entradas em RECOVER durante a marcha | 1 (queda) | **nenhuma** |
| tilt máx. em WALK | 88° | **2,3°** |
| yaw | −36° divergindo | ±23,7° máx, limitado e auto-corretivo |
| deslocamento | 0,5 m e caiu | **3,00 m** |
| velocidade média | — | **0,106 m/s** contra 0,1 comandado |

A transição para HOLD pelo watchdog, ao fim do comando, também é limpa: sem
RECOVER, tilt ≤ 0,7°.

### Premissa que caiu: o envelope de 0,03 do `_SAFE_STICK_LIMIT`

`_SAFE_STICK_LIMIT = 0.03` estava documentado como "envelope provado estável em
F3". O número é nulo: foi medido enquanto a marcha **nunca ativava**, então
descreve com que força o controlador de equilíbrio conseguia empurrar um robô de
quatro pés plantados, não com que velocidade ele anda.

O comprimento de passo é `v · (t_swing·(1−phase) + t_stance/2) ≈ 0,34·v`, e o
comando só chega à lei de colocação por `k_x · (v_body − v_goal)` com
`k_x = 0.005`. Em stick 0,03 → 0,012 m/s isso é um passo de 4 mm pedido por um
deslocamento de 20 µm do alvo do pé, sob uma elevação de pé de 8 cm: toda a
perturbação de dar o passo e nenhum benefício de momento. Daí "marcha mas não
sai do lugar". Elevado para `0.5` → 0,2 m/s.

**Consequência para o gate de F4:** os critérios de `linear.x = 0.01` e `0.03`
foram escritos sob essa premissa e estão 25× abaixo do ponto de projeto da
marcha. Precisam ser reescritos em termos de `v_cmd` real antes de serem usados
como aceitação.

### Insights do modelo do robô — A1 contra Go2

A documentação do A1 em `docs.quadruped.de` é operacional (instalar, calibrar,
lançar Gazebo/Webots) e não traz ganhos, velocidades de marcha nem parâmetros de
gait; registra que o A1 chega a 3,3 m/s com 33,5 N·m e que o FSM se dirige por
`4` = trote, `wasd` = translação, `jl` = rotação — isso confirma a semântica de
gamepad do `Inputs` que a ponte emula. Os números de movimento estão no modelo:

| | A1 | Go2 | |
|---|---|---|---|
| massa total | ~12,5 kg | 15,10 kg | +21% |
| torque quadril/coxa | 33,5 N·m | **23,7 N·m** | **−29%** |
| torque canela | 33,5 N·m | 35,55 N·m | +6% |
| coxa / canela | 0,200 m | 0,213 m | +6,5% |
| offset lateral do quadril | 0,0838 m | 0,0955 m | +14% |
| curso do quadril | ±46° | ±60° | mais folgado |
| `foot_radius` | 0,02 m | 0,02 m | igual |

1. O Go2 é 21% mais pesado com quadril e coxa 29% mais fracos — torque/peso no
   quadril caiu ~41%. Alvo de força herdado da sintonia do A1 pede torque que o
   Go2 pode não ter.
2. Pernas mais longas + quadris mais afastados + mais massa dão exatamente o
   salto de inércia de 2,3× medido, com o mesmo fator nos três eixos
   (2,29 / 2,35 / 2,32) — confirma a inércia corrigida em `BalanceCtrl`.
3. O curso do quadril do Go2 é mais largo que o do A1, o que enfraquece a
   hipótese de limite articular como mecanismo da queda. Coerente com a causa
   medida ter sido de controle de rumo, não de cinemática.

### Item aberto — deriva de guinada em HOLD

Exposto pela primeira vez porque antes o robô nunca chegava a parar de pé. Já
em HOLD, comando zero, quatro pés em contato:

```
HOLD tilt=0.3 posErr=0.0055 c=[1 1 1 1] z=0.328 yaw=19.6
...
HOLD tilt=1.2 posErr=0.0112 c=[1 1 1 1] z=0.328 yaw=44.5
```

A guinada escorrega monotonicamente ~3°/s enquanto tilt, `z` e `posErrXY`
permanecem pequenos: o corpo gira arrastando os pés. Mesmo teto de 5,3 N·m —
`captureBodyReference()` fixa a referência de yaw uma vez na borda WALK→HOLD, o
erro cresce até ~37° e o controlador saturado nunca o vence. Não corrigido:
fora do escopo de "fazer o robô andar" e não deve ser adivinhado.

## Estabilização após o movimento terminar — 18/08/2026

Sintoma relatado: os movimentos melhoraram, mas repetindo andar/parar o robô
"acaba se perdendo". Reproduzido com 3 ciclos de (andar 8 s, parar 6 s):

| ciclo | yaw após andar | yaw após parado | acréscimo do HOLD |
|---|---|---|---|
| 1 | +2,43° | +1,97° | −0,5° |
| 2 | **−11,62°** | −10,68° | +0,9° |
| 3 | −1,27° | **−9,90°** | −8,6° |

Posição estava boa (0,78 / 0,77 / 0,65 m por ciclo, `y` perto de zero): o robô
não se perde em translação, se perde em **rumo**. E num ciclo posterior ele
**caiu parado**, sem comando: tilt subindo 0,2 → 1,8° ao longo de 8 s e então
`RECOVER tilt=31.4`.

### Medição — alvo de apoio obsoleto

Instrumentado o erro de posição por pé (alvo do gait contra posição medida).
Em 40 s de HOLD:

```
HOLD tilt=0.1 yaw=6.5 Fz=142>142 err=[0.010 0.080 0.075 0.009]
```

Constante, e assimétrico: **8,0 cm e 7,5 cm em um par diagonal**, 1,0 cm nos
outros dois, para sempre. Causa em `GaitGenerator::generate()`:

```cpp
if (contact_(i) == 1) {
    if (phase_(i) < 0.5) start_p_.col(i) = estimator_->getFootPos(i);
    feet_pos.col(i) = start_p_.col(i);
```

`WaveGenerator` fixa a fase em **exatamente 0,5** em `STANCE_ALL`, então
`0.5 < 0.5` é falso e `start_p_` nunca é re-travado depois que a marcha para.
As duas pernas que estavam em balanço no instante da parada ficam com o alvo
congelado no ponto de **decolagem**, não onde pousaram. O PD de junta puxa esse
erro de 8 cm indefinidamente — perturbação constante e assimétrica que gira o
robô parado e, duas vezes, o derrubou.

### Correção — travar o alvo de apoio no toque do pé

`GaitGenerator` passa a travar `start_p_` também na transição para contato, não
só enquanto `phase < 0.5`. Durante a marcha o comportamento é o mesmo; parado,
cada perna trava uma vez ao pousar.

Validação, 5 ciclos de (andar 8 s, parar 8 s):

| | antes | depois |
|---|---|---|
| quedas | 1 (ciclo 3, parado) | **0** |
| tilt máx. parado | explodia a 31° | **0,9°** |
| tilt máx. andando | — | 1,0° |
| `footErr` parado | 8 cm fixo, um par diagonal | **0,4–1,3 cm, simétrico** |
| efeito do HOLD no rumo | +5,6 / +13,3 / +41,2 / +43,9° | **−3,1 / −1,7 / −0,8 / +0,9 / −1,4°** |
| deslocamento | — | 3,85 m em 5 ciclos |

O HOLD deixou de ser fonte de erro de rumo e passou a corrigi-lo em 4 dos 5
ciclos. Em três execuções independentes desta configuração: **zero quedas** nas
três; rumo final após 4–5 ciclos de 43°, 29° e 12°. Limitado, sem acumulação
monotônica, mas com variância grande entre execuções — não há malha fechada de
rumo, e não deve haver neste nível (ver abaixo).

### Ensaio 7 — rumo persistente em HOLD: **pior**

Em vez de recapturar `yaw_cmd_` a cada parada, mantê-lo como setpoint movido só
por `d_yaw_cmd_`. Motivação: recapturar faz o controlador adotar como correto
qualquer rumo para onde derivou, então o erro nunca é corrigido.

Resultado: o robô tombou no ciclo 3 (z 0,354 → 0,217 → 0,086, yaw 164,7°), 83
amostras em RECOVER, tilt máximo em HOLD 8,1°, e já no ciclo 1 o HOLD
*acrescentou* +11,6° de guinada. Com o momento de guinada limitado a ~5,3 N·m o
controlador não gira um robô parado: ele só arrasta os pés tentando, até
desestabilizar. Revertido.

**Consequência de projeto:** este controlador não mantém rumo em malha aberta,
e isso é correto. Um trote cego com essa autoridade de guinada faz passeio
aleatório — a própria Unitree embarca um trim de operador para deriva de
guinada pelo mesmo motivo, a ser aplicado durante o trote
(`docs.quadruped.de`, operação do Go1 e quick-start do A1). Fechar essa malha é
papel da navegação, que enxerga pose e comanda `angular.z` por `/demo/cmd_vel`.

### Ensaio 8 — ganho de apoio mais rígido quando parado: **muito pior**

Motivado pela análise de `unitreerobotics/unitree_ros`: os ganhos de referência
da Unitree para ficar em pé (`unitree_controller/src/body.cpp`, `paramInit()`)
são `Kp` 70 / 180 / 300 em quadril / coxa / canela, e o `StateFixedStand` deste
repositório usa 80/3,5, contra os 3,0/2,0 do trote. Testado 80/3,5 quando
`mode_ != WALK`.

Resultado: caiu no primeiro ciclo e não levantou — 300 amostras em RECOVER,
tilt 52°, `footErr` de 35 a 46 cm.

A analogia estava errada. Aqueles ganhos são de um PD **puro**, único atuador da
junta. Aqui `gz_quadruped_hardware` **soma** o PD ao torque que o `BalanceCtrl`
já calculou: a 80 os dois disputam a perna e o PD vence o controlador de força
que sustenta o robô. **3,0/2,0 é um teto para um PD auxiliar, não um alvo de
rigidez.** Revertido.

### Análise de `unitreerobotics/unitree_ros`

Repositório oficial (ROS 1 Melodic/Kinetic, Gazebo 8): descrições de 19 robôs,
`unitree_gazebo`, `unitree_legged_control` e `unitree_controller` — este último
com controle de junta em torque, posição e velocidade, mais exemplos `servo` e
`move_kinetic`. O README declara que **"Gazebo simulation cannot do high-level
control, namely walking"**: a pilha oficial não caminha em simulação, o que é
exatamente por que o `unitree_guide` da comunidade existe e por que não há
referência de sintonia de marcha ali. Pose de apoio oficial `{0.0, 0.67, −1.3}`
por perna, contra `(0.0, 0.8, −1.5)` no nosso `gazebo.yaml` — o oficial é menos
agachado. Os ganhos citados acima são o único dado de sintonia aproveitável, e
com a ressalva do Ensaio 8.
