# S5 — Nav2 desviando de obstáculos

Mundo: `quadruped_objects.sdf` · Estação x86 · ~10 min

## Para que serve

É o primeiro cenário em que o robô decide para onde ir. Nos cenários S0–S4 quem
comandava era `gait_trial.sh` ou `demo_routine`, publicando velocidade direto.
Aqui o comando é uma **meta**, o Nav2 planeja, e o desvio é consequência do
costmap — não de um roteiro escrito à mão.

Isso exercita, de ponta a ponta: a nuvem do lidar → costmap → planejador → MPPI →
`/demo/cmd_vel` → `twist_to_inputs` → controlador de marcha → Gazebo → odometria
→ TF → costmap. Uma volta fechada. Quase todo defeito desta pilha aparece aqui.

## Onde cada coisa roda

| componente | máquina | por quê |
| --- | --- | --- |
| Gazebo + robô + bridge | container `aquila-go2`, host x86 | OGRE 2, regra 1 do `CLAUDE.md` |
| Nav2 + `odom_tf` + patrulha | host x86, **nativo** | a imagem `demo-sim:spike-go2` **não tem Nav2** |

O segundo ponto é medido, não suposto: `ls /opt/ros/jazzy/lib | grep -c nav2`
dentro daquela imagem devolve **0**. Enquanto ela for a imagem do spike, o Nav2
sobe nativo no host e conversa com o container pelo DDS — mesma rede
(`--network=host`), domínio 69, `rmw_cyclonedds_cpp`. No modo hil o Nav2 vai para
o módulo, e aí a imagem do módulo é que precisa tê-lo dentro.

## Geometria e por que as metas são estas

Obstáculos do mundo (do próprio SDF):

| objeto | posição (x, y) | meia-extensão no plano |
| --- | --- | --- |
| caixa vermelha | 1,5 · 0,0 | 0,21 (meia-diagonal) |
| cilindro verde | 3,0 · +0,45 | 0,18 |
| caixa azul | 3,0 · −0,55 | 0,28 (meia-diagonal) |
| cilindro amarelo | 4,5 · 0,0 | 0,12 |

Raio circunscrito do tronco do Go2: **0,383 m**.

O percurso default do `patrol_commander` é um triângulo de três metas escolhido
para que **cada meta seja alcançável** e **cada trecho tenha a reta bloqueada**:

Folga calculada por **distância ponto-segmento**, não pela distância vertical
num `x` escolhido — aquela superestima a folga e foi o erro da primeira versão
desta tabela.

| trecho | folga da reta | consequência |
| --- | --- | --- |
| (0,0) → (4, 1,5) | **−0,068 m** da caixa vermelha | reta bloqueada |
| (4, 1,5) → (4, −1,5) | **−0,003 m** do cilindro amarelo | reta bloqueada |
| (4, −1,5) → (0,0) | **−0,127 m** da caixa azul | reta bloqueada |

As três retas estão bloqueadas, então o desvio é obrigatório. E as três **metas**
são folgadas — +0,887, +0,713 e +0,905 m — porque meta apertada faz o Nav2 falhar
por chegada impossível, o que se confunde com falha de desvio. Se o robô andar em linha reta, ou o costmap está vazio ou ele passou
por dentro do obstáculo; as duas coisas são falha.

### O quadrado de 3 m que parece óbvio não serve

A meta (3,0) cai no vão entre o cilindro verde e a caixa azul. Esse vão tem
0,45 − 0,18 = 0,27 de um lado e −0,55 + 0,28 = −0,27 do outro: **0,54 m de
largura livre**, contra **0,77 m** que o robô precisa. A meta é impossível.

E o Nav2 **aceita** essa meta. Ele só reprova depois de esgotar as recuperações,
e o que aparece no log é aborto de navegação — que se lê como "o desvio não
funciona" e é uma meta que nunca teve solução. Confira a aritmética antes de
culpar o planejador.

## Rodar

Terminal 1 — simulador:

```bash
./scripts/run_quadruped_sim.sh quadruped_objects.sdf
```

Espere `state=fixed stand` no log. **Não siga antes disso.**

Terminal 2 — Nav2 e TF:

```bash
cd ros2_ws && source install/setup.bash
ros2 launch demo_bringup nav_quadruped.launch.py
```

O portão de prontidão é a linha `Managed nodes are active` do
`lifecycle_manager_navigation`. Espere por ela.

Terminal 3 — patrulha:

```bash
source ros2_ws/install/setup.bash
ros2 run demo_bringup patrol_commander --ros-args -p use_sim_time:=true
```

## Armadilhas, todas medidas

### `ros2 action list` travando para sempre

Não use `ros2 action list` como portão de prontidão em script. Com o grafo
incompleto ele **bloqueia indefinidamente** — não devolve vazio, não expira.
Um runner meu ficou 10 min preso nisso. Use o log do `lifecycle_manager`.

### Nada sobe e nada dá erro

Sintoma: o log do launch termina em `odom_tf` e mais nada; `navigate_to_pose`
nunca aparece.

Causa: `use_composition` ligado ao incluir só o `navigation_launch.py`. Aquele
arquivo usa `LoadComposableNodes` para carregar os servidores dentro de
`/nav2_container`, mas **não cria** esse container — quem o cria é o
`bringup_launch.py`, que é justamente o que não estamos incluindo. Os nós vão
para um container inexistente, em silêncio.

`nav_quadruped.launch.py` fixa `use_composition: 'False'` por isso.

### O robô anda em espasmos

`demo_routine` está rodando junto. Os dois publicam `/demo/cmd_vel` — o Nav2 pelo
`collision_monitor`, a rotina direto. `twist_to_inputs` obedece a última mensagem
que chegou e alterna entre as duas a 20 Hz. Nenhum log menciona isso.

```bash
ros2 topic info /demo/cmd_vel --verbose | grep -c "Node name"   # tem de ser 1
```

Um `patrol_commander` **de uma execução anterior** dá o mesmo sintoma, e é fácil
deixar um para trás: ele não morre quando o simulador cai, só fica esperando a
ação voltar. Aconteceu quatro vezes numa bateria de medição, e os órfãos seguiam
publicando no domínio 69. Antes de subir, confira:

```bash
pgrep -af "patrol_commander|demo_routine"   # tem de estar vazio
```

### Dois publicadores na TF

`odom_tf` publica `map -> odom` como identidade. AMCL e SLAM também publicam essa
aresta. Rodar os dois não dá erro: o consumidor alterna entre duas crenças de
onde o robô está. Se subir AMCL ou SLAM, passe
`publish_map_identity:=false`.

### `Failed to make progress` a cada 10 s

Verificador de progresso com os valores do TurtleBot 4: 0,5 m em 10 s. O Go2 gira
a 0,12 rad/s, então reorientar 90° consome 13 s com avanço quase nulo — o aborto é
garantido antes de o robô começar a andar. `nav2_params_go2.yaml` usa 0,20 m em
40 s.

Sintoma na primeira medição: **22 abortos em 5 min, zero quedas.** Quando o
verificador reprova e o robô não cai, o suspeito é o verificador.

### O robô anda a 40 % do pedido e nada acusa

`/demo/cmd_vel` **não está em SI.** É um `Twist` que carrega posição normalizada
de manche, e o controlador multiplica `linear.x` por 0,4 e `angular.z` por 0,5 ao
receber (`StateTrotting.cpp:192`, `twist_to_inputs.py:283`).

O Nav2 não pode publicar ali direto, porque o MPPI **integra** `vx` como m/s para
prever posição. Por isso existe `cmd_vel_si_to_stick`: o Nav2 publica SI em
`/demo/cmd_vel_si` e o nó converte. Se você ligar o Nav2 de volta em
`/demo/cmd_vel`, o robô anda a 40 % do pedido sem erro nenhum em log.

```bash
ros2 topic info /demo/cmd_vel_si --verbose | grep -c "Node name"   # 1: o Nav2
ros2 topic info /demo/cmd_vel    --verbose | grep -c "Node name"   # 1: o conversor
```

### O robô chega na meta e a meta reprova

Orientação final. Se o `yaw` da meta não for o rumo de **chegada**, o Nav2 pede
giro parado ao chegar — 110 a 139° no percurso default, 16 a 20 s ao teto de
guinada. E girar parado não fica parado: medido, o robô chegou a 3,8 cm da meta e
derivou 0,78 m em y girando, saindo da tolerância de posição.

`patrol_commander` usa o rumo de chegada por isso, e
`yaw_goal_tolerance` é 0,5 rad.

## Resultados medidos (20/08/2026)

Percurso (4 · 1,5) → (4 · −1,5) → (0 · 0), ciclo, 300 s de tempo de simulação.
Detalhe e evidência em [`../../results/ml35-nav2-quadrupede.md`](../../results/ml35-nav2-quadrupede.md).

| corrida | mudança | caminho | deslocamento | `cmd_vx` máx SI | abortos | quedas |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | parâmetros do TB4 | 4,16 m | 0,73 m (**para trás**) | 0,016 m/s | 22 | 0 |
| 2 | horizonte 1,44 m, laço 10 Hz | 5,21 m | 0,74 m | 0,023 m/s | 3 | 0 |
| 3 | fronteira de unidades | **8,36 m** | **3,51 m** | **0,119 m/s** | 2 | 0 |
| 4 | metas com rumo de chegada, 420 s | 10,87 m | 0,60 m | 0,119 m/s | 4 | 0 |

Na corrida 3 o robô chegou a 3,8 cm da primeira meta, passando pelos quatro
obstáculos com folga positiva — mínima de 0,014 m no cilindro verde.

Na corrida 4 **a primeira meta foi concluída** e o ciclo seguiu, com movimento
contínuo por 420 s, zero colisões (folga mínima 0,159 m) e zero quedas.

**Cuidado ao comparar 3 com 4.** O MPPI é um amostrador estocástico
(`regenerate_noises: true`) e há **n = 1 por condição**: a piora no deslocamento
não é atribuível à mudança. Para atribuir, é preciso repetição por condição.

O costmap foi medido separadamente, com o robô parado: custo **0** na célula do
robô, **0** dentro de 0,6 m, 42 células letais nos obstáculos, zero desconhecidas.
Dos 2097 pontos da nuvem, 1848 são chão e são descartados pelo corte de 0,12 m.

## O que este cenário NÃO valida

- **Localização.** `/demo/odom` é ground truth do Gazebo, e `odom_tf` só a
  republica como TF. O robô sabe exatamente onde está porque o simulador contou.
  Estimativa de estado com perna é F5.
- **Percepção.** `demo_perception` continua stub sintético. O que desvia aqui é o
  lidar, pela `obstacle_layer`; a `perception_layer` está no costmap por contrato,
  alimentada por um stub.
- **Desempenho.** Números de CPU, taxa de laço e FPS aqui são da estação x86.
  Nada disso transfere para o Aquila AM69, e emulação arm64 não mede desempenho
  (regra 5 do `CLAUDE.md`).

## Aceitação

- [ ] `Managed nodes are active` aparece no log do `lifecycle_manager`
- [ ] `ros2 topic info /demo/cmd_vel --verbose` mostra **um** publicador
- [ ] a árvore TF tem raiz `map` (`ros2 run tf2_tools view_frames`)
- [ ] `/demo/scan_cloud` publica e o costmap local mostra células ocupadas
- [ ] a trajetória gravada tem folga positiva para todos os quatro obstáculos
- [ ] zero `mode=RECOVER` no log do simulador (o robô não caiu)
