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
