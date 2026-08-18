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

Ainda em aberto:

- o trote dinâmico ainda pode fazer o corpo afundar e cair;
- a estabilidade após a troca para `TROTTING` precisa ser medida novamente
  após o ajuste de ganho de stance `Kp=3.0`, `Kd=2.0`;
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
  'gait FSM|gait diagnostics|Switched|controller'
```

Aguarde:

```text
gait FSM: fixed stand. Waiting for a non-zero /demo/cmd_vel
```

Nesse ponto o robô deve estar em pé, aproximadamente a `z=0.35 m`, sem
comando de movimento.

## 3. Diagnóstico mínimo antes de alterar código

O controlador imprime uma linha por segundo:

```text
gait diagnostics: state=... command=... sticks=(...) contact=[...]
```

Interpretação:

| Observação | Conclusão |
|---|---|
| `ly` não muda após publicar Twist | problema no DDS ou no bridge |
| `state` não muda para `trotting` | comando chegou antes da FSM estar pronta |
| `ly` muda e `contact` nunca alterna | problema no `WaveGenerator` |
| `contact` alterna e o robô cai | dinâmica, ganhos, estimador ou QP |
| `ly=0` e `contact=[1 1 1 1]`, mas cai | controle de postura do trote está fraco |

Nunca conclua sucesso apenas pelo log `controller ... active`; isso não prova
que as juntas estão mantendo o corpo.

## 4. Teste de postura sem andar

Este teste inicia o trote e em seguida mantém velocidade zero:

```bash
docker exec -it aquila-go2 bash -lc '
  . /opt/ros/jazzy/setup.sh
  . /test/install/setup.sh

  timeout 5s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.03}}"

  timeout 20s ros2 topic pub -r 10 /demo/cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}}"
'
```

Critério: `state=trotting`, `ly=0.0000`, `contact=[1 1 1 1]` e corpo estável por
20 s. Se falhar, não testar avanço ainda.

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

### Step 5.2 — ajustar apenas os ganhos de stance

O primeiro ajuste aplicado foi igualar stance e swing em `Kp=3.0`, `Kd=2.0`.
Recompile e repita o teste de postura antes de qualquer mudança no período:

```bash
./scripts/run_quadruped_sim.sh
```

Se ainda houver queda, testar somente uma alternativa conservadora, por
exemplo `Kp=5.0`, `Kd=2.5`, e repetir exatamente a mesma janela de 20 s. Nunca
alterar simultaneamente ganhos, `gait_height` e período.

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

### Step 5.5 — adicionar parada segura

O estado atual aceita `ly=0` e permanece em `TROTTING` com todos os pés em
stance. Uma melhoria posterior é implementar no bridge um watchdog que, após
um período configurável de Twist zero, envie `command=2` e retorne a
`FIXEDSTAND`. Essa alteração deve ser feita somente depois de medir que a
postura em trote parado está estável, para não esconder uma falha de controle.

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

Marcar F4 somente quando todos forem verdadeiros:

- Go2 aparece com malhas e proporções corretas;
- fica estável em `FIXEDSTAND` por pelo menos 60 s;
- fica estável em trote parado por pelo menos 20 s;
- anda em `linear.x=0.01` sem cair;
- anda em `linear.x=0.03` sem tombar;
- yaw controlado não produz rotação explosiva;
- warehouse carrega com câmera e lidar ativos;
- RViz2 mostra as 12 juntas e TF consistente;
- diff-drive mantém goal Nav2 `SUCCEEDED`.

Cada ajuste deve ser um commit separado, com comando, duração, pose inicial e
pose final registrados neste documento ou em
`docs/results/ml35-f4-parcial.md`.
