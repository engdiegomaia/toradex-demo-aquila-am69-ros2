# Go2 — guia de implementação dos próximos passos

Este documento é o roteiro operacional para transformar o spike do Go2 em uma
simulação reproduzível, melhorar a marcha de forma incremental e integrar o
cenário warehouse sem perder a regressão do diff-drive.

## Estado atual

Já validado:

- os seis pacotes do quadrúpede compilam na imagem `demo-sim:spike-go2`;
- Gazebo abre com o Go2 visível e os sensores publicam;
- o robô passa por `PASSIVE -> FIXEDDOWN -> FIXEDSTAND`;
- `/demo/cmd_vel` chega ao bridge e é convertido para `/control_input`;
- a FSM muda para `TROTTING`;
- o `WaveGenerator` alterna os pares de contato;
- o diff-drive não foi substituído pelo quadrúpede por padrão;
- a conversão de posição/velocidade dos pés para o frame global foi corrigida;
- a suíte do `demo_simulation` passou com 10 testes.

Validado em 18/08/2026, depois da separação WALK/HOLD/RECOVER (evidência
completa em `docs/results/ml35-f4-parcial.md`):

- a marcha ativa no comando mais lento do plano, `Twist linear.x=0.01`;
- as pernas levantam de fato, em pares diagonais (`[1 0 0 1]` ↔ `[0 1 1 0]`);
- parar de publicar zera o comando em 0,3 s pelo watchdog do bridge;
- `HOLD` mantém o corpo sem movimento residual (`posErrXY ≈ 0,005 m` por >35 s);
- o supervisor de atitude entra em `RECOVER` a 12° de inclinação.

Validado em 18/08/2026, depois da correção de rumo por colocação de pé
(`k_yaw_` de 0,005 → 0,15 em `FeetEndCalc`):

- **o robô anda**: 30 s de trote contínuo a `v_cmd = 0,1 m/s`, 3,00 m
  percorridos, nenhuma entrada em `RECOVER`, tilt máximo 2,3°;
- velocidade média medida de 0,106 m/s contra 0,1 m/s comandado;
- a guinada passou de divergente (−36° e queda) para oscilação limitada e
  auto-corretiva (±23,7°), com comando de yaw zero;
- a parada pelo watchdog leva a `HOLD` de pé, sem `RECOVER`, tilt ≤ 0,7°;
- a suíte do `demo_simulation` passou com 20 testes.

Validado em 18/08/2026, depois do travamento do alvo de apoio no toque do pé
(`GaitGenerator::generate`):

- **andar e parar repetidamente é estável**: 5 ciclos de (andar 8 s, parar 8 s),
  3,85 m percorridos, **zero quedas**, tilt máximo 1,0° andando e 0,9° parado;
- o erro de posição por pé parado caiu de 8 cm fixo num par diagonal para
  0,4–1,3 cm simétrico;
- parar passou a **corrigir** o rumo em 4 dos 5 ciclos, em vez de acrescentar
  até +44° por parada.

Ainda em aberto:

- **rumo em malha aberta não é mantido**, por projeto: sem comando de guinada o
  trote faz passeio aleatório (rumo final de 43°, 29° e 12° em três execuções de
  5 ciclos). O teto de ~5,3 N·m de momento de guinada do QP não permite corrigir
  rumo parado — tentar derruba o robô (ensaio 7). Fechar a malha é papel do
  Nav2, via `angular.z` em `/demo/cmd_vel`; falta ensaiar;
- o `z` estimado fica 24 mm abaixo do real, constante, por `foot_radius = 0.02`
  contra `feet_h_ = 0` no estimador: todo alvo de pé em balanço mira 2 cm abaixo
  do solo;
- os gates de `linear.x = 0.01` e `0.03` foram escritos sob a premissa nula do
  `_SAFE_STICK_LIMIT = 0.03` e estão 25× abaixo do ponto de projeto da marcha —
  precisam ser reescritos em termos de `v_cmd` antes de servirem como aceitação;
- o warehouse com Go2, RViz2 e TF completo ainda precisam de validação;
- F4 não deve ser marcada como concluída antes desses gates.

## 1. Preparar o host

Execute no workstation x86 com Docker, X11 e acesso à GPU:

```bash
cd ~/toradex/demo/aquila-am69-ros2
xhost +local:docker
```

Não execute `ros2 launch` diretamente no host se o ROS 2 não estiver instalado.
Todos os comandos ROS deste guia, exceto o script de inicialização, rodam
dentro do container.

## 2. Rodar a aplicação do spike

O script compila os seis pacotes dentro do container, usa o domínio DDS 69 e
mantém o nome `aquila-go2`:

```bash
./scripts/run_quadruped_sim.sh
```

O cenário padrão é o fixture:

```text
quadruped_empty.sdf
```

Esse mundo é preferível ao `empty.sdf` do Gazebo porque contém o sistema de
Sensors necessário para câmera e lidar.

Mantenha o primeiro terminal aberto. O container é removido pelo `--rm` quando
esse terminal for encerrado.

### Verificar a inicialização

Em outro terminal:

```bash
docker ps --filter name=aquila-go2 \
  --format 'table {{.Names}}\t{{.Status}}'

docker logs -f aquila-go2 2>&1 | grep -E \
  'gait FSM|gait diagnostics|trot supervisor|watchdog|Switched|controller'
```

Aguarde:

```text
gait FSM: fixed stand. Waiting for a non-zero /demo/cmd_vel
```

Nesse ponto o robô deve estar em pé, aproximadamente a `z=0.35 m`, sem
comando de movimento.

## 3. Diagnóstico mínimo antes de alterar código

Duas linhas por segundo, de fontes diferentes. A do controlador mostra o que
chegou pelo tópico:

```text
gait diagnostics: state=... command=... sticks=(...) contact=[...]
```

A do supervisor de trote mostra o que o controlador decidiu com isso:

```text
trot supervisor: mode=WALK cmd=(0.0040,-0.0000,0.0000) tilt=2.0deg \
  posErrXY=0.0147 velErrXY=0.0337 contact=[0 1 1 0]
```

`cmd` já está em unidades SI, depois da conversão de stick. A conversão custa
um fator 0,4: `Twist 0.01 -> ly 0.01 -> 0.004 m/s`. Não compare `cmd` com o
valor do `Twist` sem lembrar disso.

Interpretação:

| Observação | Conclusão |
|---|---|
| `ly` não muda após publicar Twist | problema no DDS ou no bridge |
| `state` não muda para `trotting` | comando chegou antes da FSM estar pronta |
| `mode=HOLD` com `Twist` não nulo em curso | comando abaixo de `V_START`, ou watchdog considerando o stream morto |
| `mode=WALK` e `contact` nunca alterna | problema no `WaveGenerator` |
| `contact` alterna e o robô cai | dinâmica, ganhos, estimador ou QP |
| `mode=HOLD` e `posErrXY` cresce | o corpo está sendo empurrado; não é comando |
| `mode=RECOVER` seguido de `tilt > 90°` | o robô caiu; o supervisor só registrou |
| `tilt` cresce mas `cmd` já é zero | postura, não comando |

Nunca conclua sucesso apenas pelo log `controller ... active`; isso não prova
que as juntas estão mantendo o corpo. E `mode=WALK` sozinho também não prova
marcha: só `contact` alternando prova.

## 4. Teste de postura sem andar

Este teste entra em trote com o comando mais lento e depois **para de publicar**.
Não publique zeros: o objetivo é justamente exercitar o watchdog, que é quem
garante que silêncio significa parar.

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh
  . /test/install/setup.sh

  timeout 3s ros2 topic pub -r 20 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.01}}" || true

  timeout 20s ros2 topic echo /demo/odom --field pose.pose >/dev/null || true
'
```

Critério, no log do supervisor: `mode=WALK` durante a publicação, `cmd_vel
watchdog: stale` até 0,3 s depois dela terminar, e então `mode=HOLD`,
`contact=[1 1 1 1]`, `posErrXY` abaixo de 0,01 m e `tilt` abaixo de 2° por 20 s.

Executado em 18/08/2026: passou, com `posErrXY` entre 0,0018 e 0,0077 m estável
por mais de 35 s. Se falhar, não testar avanço ainda.

## 5. Melhorar o movimento em steps pequenos

Altere uma variável por vez e mantenha um registro de cada ensaio.

### Step 5.1 — confirmar rastreamento das juntas

Durante o teste, observe:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh
  . /test/install/setup.sh
  ros2 topic hz /joint_states
'
```

Confirme que existem 12 juntas e que a frequência não cai. Para uma amostra:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic echo /joint_states --once
'
```

Se as juntas publicam mas não acompanham as referências, investigar primeiro o
PD no hardware Gazebo (`joint_effort + kp*(q_cmd-q) + kd*(qd_cmd-qd)`).

### Step 5.2 — ganhos de stance: **próximo experimento isolado**

O valor atual, `Kp=3.0` / `Kd=2.0` igual ao swing, foi escolhido contra um log
em que **nenhuma passada era pedida** (`contact=[1 1 1 1]` o tempo inteiro).
Essa evidência não vale mais: com a marcha ativa, a perna de apoio passa a ser
controlada por força pelo QP, e um PD de junta rígido perseguindo um alvo de pé
congelado briga com ele. O upstream usa `Kp=0.8` / `Kd=0.8` no stance.

Ensaio: voltar ao valor do upstream, recompilar, repetir **exatamente** a mesma
janela de 30 s em `linear.x=0.01`, e comparar tempo até queda e envelope de
`tilt` contra a linha de base já registrada (queda em ~8 s, `tilt` oscilando
até 9°).

```bash
./scripts/run_quadruped_sim.sh
```

Se não melhorar, o próximo suspeito é `BalanceCtrl::Ib_`, que tem a inércia do
A1 fixa no código — ver `docs/results/ml35-f4-parcial.md`. Nunca alterar
simultaneamente ganhos, `gait_height`, inércia e período.

### Step 5.3 — testar avanço muito lento

Somente depois de passar no teste de postura:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  timeout 10s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.01}}"
'
```

Critérios:

- o corpo continua nivelado;
- `contact` alterna em pares;
- `/demo/odom` cresce de forma monotônica em x;
- não há queda ou salto abrupto de yaw.

Só depois testar `0.02` e, por último, `0.03`. O valor `0.03` é o limite
empírico atual, não uma garantia de estabilidade.

### Step 5.4 — testar rotação

Não combine yaw e avanço no primeiro ensaio:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  timeout 8s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{angular: {z: 0.02}}"
'
```

Se houver rotação no sentido errado, revisar o sinal de `rx` em
`twist_to_inputs.py` e em `StateTrotting::getUserCmd()`. Se houver tombamento,
reduzir yaw antes de mexer na translação.

### Step 5.5 — parada segura: **implementado em 18/08/2026**

O bridge tem watchdog: `twist_to_inputs` publica a 20 Hz e zera os sticks
quando o último `Twist` passa de 0,3 s. Silêncio agora significa parar, e o
controlador entra em `HOLD` sozinho.

O que **não** foi implementado, de propósito: voltar a `FIXEDSTAND` por
`command=2` depois de N segundos parado. `HOLD` é postura em trote com quatro
pés no chão, medida estável por mais de 35 s; trocar de estado da FSM
esconderia uma eventual falha do controlador de postura atrás do controlador
de `FIXEDSTAND`, que é justamente o que não se quer medir agora.

## 6. Medir odometria e pose

Durante cada ensaio, salve uma amostra:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic echo /demo/odom --once
'
```

Para uma medição repetida, grave o log do simulador:

```bash
docker logs -f aquila-go2 > /tmp/aquila-go2.log
```

O critério de estabilidade é pose nivelada, `z` próximo de `0.35 m`, sem
crescimento contínuo de roll/pitch e sem deslocamento que seja apenas queda ou
deslizamento.

## 7. Integrar o cenário warehouse

O spike não contém `nav2_minimal_tb4_sim`; por isso o mundo oficial não deve
ser referenciado como substituição implícita no launch. Há duas opções.

### Opção A — passar um SDF existente

Quando houver um `warehouse.sdf` válido no host:

```bash
./scripts/run_quadruped_sim.sh \
  /caminho/absoluto/warehouse.sdf
```

O SDF deve incluir os sistemas de Physics, SceneBroadcaster e Sensors. Depois
de iniciar, confirme no log o nome do mundo e teste:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh && . /test/install/setup.sh
  ros2 topic hz /demo/scan
  ros2 topic hz /demo/camera/image_raw
'
```

### Opção B — instalar o pacote oficial

Com a rede disponível, no ambiente que constrói a imagem:

```bash
sudo apt install -y ros-jazzy-nav2-minimal-tb4-sim
ros2 pkg prefix nav2_minimal_tb4_sim
```

Localize o SDF instalado e passe seu caminho absoluto ao script ou ao launch.
Não copie um mundo Gazebo Classic arquivado; use um SDF compatível com Gazebo
Harmonic.

O cenário deve ser validado primeiro sem Nav2: robô visível, sensores ativos,
`/demo/odom` publicando e nenhum link solto no TF.

## 8. RViz2 e TF

RViz2 deve rodar fora do container Gazebo, conforme a separação do projeto.
Quando a imagem oficial estiver disponível:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml up viz
```

No RViz2, confirmar:

- `RobotModel` mostra corpo e 12 juntas;
- `joint_states` anima as pernas;
- câmera, lidar e IMU têm tópicos ativos;
- não existem links soltos;
- `odom -> base_link` tem um único proprietário.

No spike, a odometria Gazebo é uma fonte temporária de F4. A substituição por
odometria legged e a propriedade definitiva de `odom -> base_link` pertencem ao
F5.

## 9. Compose oficial

Quando a rede voltar:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml build base sim viz
docker compose -f compose.host.yml run --rm sim \
  ros2 launch demo_bringup sim.launch.py robot_type:=quadruped
docker compose -f compose.host.yml up viz
```

O default continua sendo `diffdrive`; `robot_type:=quadruped` é obrigatório.
`docker compose up sim` sozinho não inicia o Go2.

## 10. Regressão do diff-drive

Depois de qualquer vendorização ou mudança de launch:

```bash
cd ~/toradex/demo/aquila-am69-ros2/docker
docker compose -f compose.host.yml --profile learn up --build
```

O gate é um goal Nav2 terminando em `SUCCEEDED`. Se falhar, separar a
regressão do diff-drive da investigação do Go2; não corrigir os dois caminhos
no mesmo commit.

## 11. Critérios para fechar F4

Marcar F4 somente quando todos forem verdadeiros. Estado em 18/08/2026:

| Critério | Estado |
|---|---|
| Go2 aparece com malhas e proporções corretas | ✅ |
| estável em `FIXEDSTAND` por pelo menos 60 s | ✅ |
| há swing físico das pernas em pares diagonais | ✅ `[1 0 0 1]` ↔ `[0 1 1 0]` |
| comando zerado leva a `HOLD` sem movimento residual | ✅ >35 s |
| estável em trote parado por pelo menos 20 s | ✅ |
| anda a `v_cmd = 0,05 m/s` sem cair | ✅ 97% de rastreamento |
| anda a `v_cmd = 0,10 m/s` sem cair | ✅ 105% |
| anda a `v_cmd = 0,20 m/s` sem cair | ✅ 111%, teto da ponte |
| guinada limitada com comando de yaw zero | ✅ ±23,7°, auto-corretiva |
| não deriva em guinada parado em `HOLD` | ✅ corrigido pelo latch de toque |
| yaw comandado não produz rotação explosiva | ✅ zero `RECOVER`, tilt ≤ 1,3° |
| **guinada comandada rastreável** | ⚠️ satura em ~0,13 rad/s, ver abaixo |
| warehouse carrega com câmera e lidar ativos | ⬜ |
| RViz2 mostra as 12 juntas e TF consistente | ❌ falta `odom → base_link` |
| diff-drive mantém goal Nav2 `SUCCEEDED` | ⬜ |

**Envelope real do robô, medido em 18/08: 0,20 m/s linear e 0,13 rad/s
angular.** A guinada satura em ~0,13 rad/s porque o momento do QP topa em
5,3 N·m — acima disso o comando não tem efeito. `nav2_params.yaml` precisa de
`max_vel_theta ≈ 0,12`, não os 0,25 que a ponte aceita.

**A árvore TF não tem `odom → base_link`** e nem o frame `odom`: está toda
ancorada em `trunk`. O Nav2 não localiza assim. Publicar a odometria do
quadrúpede é trabalho obrigatório de F5.

Os dois critérios anteriores de `linear.x` foram escritos quando `_SAFE_STICK_LIMIT = 0.03` era
tido como "envelope estável de F3". Esse envelope foi medido com a marcha nunca
ativando, então descrevia o empurrão máximo sobre um robô de pés plantados, não
a velocidade de caminhada. `linear.x = 0.01` dá `v_cmd = 0,004 m/s`, 25× abaixo
do ponto de projeto do trote: passo de 4 mm sob elevação de pé de 8 cm. Reescrever
em termos de `v_cmd` antes de usar como aceitação.

A ordem importa: não ensaiar yaw nem warehouse enquanto o avanço reto cair, ou
os dois ensaios medem a mesma queda com nomes diferentes.

Cada ajuste deve ser um commit separado, com comando, duração, pose inicial e
pose final registrados neste documento ou em
`docs/results/ml35-f4-parcial.md`.
