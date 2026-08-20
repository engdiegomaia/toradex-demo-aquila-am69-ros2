# Cenários de simulação

Cinco mundos, cada um exercitando uma parte diferente da pilha. Um guia por
cenário, com o que ele mede, como rodar, e o que aceitar como resultado.

**Todos rodam na estação x86.** Gazebo Harmonic é OGRE 2 e não roda no Aquila
AM69 — regra 1 do `CLAUDE.md`. Nenhum número medido aqui vale como validação de
hardware.

Resultado da bateria de 20/08/2026, RTF 1,00 em todas:

| # | resultado | `RECOVER` | tilt pico | caranguejo | erro do estimador em y |
| --- | --- | --- | --- | --- | --- |
| S0 plano | passa | 0 | 1,08° | 3,20% | −0,145 m |
| S1 rampa 6° | passa | 0 | **0,96°** | 2,38% | −0,111 m |
| S2 irregular | passa | 0 | 2,81° | **6,31%** | **−0,244 m** |
| S3 corredor | passa | 0 | 1,38° | 0,33% | −0,141 m |
| S4 objetos | passa (câmera) | 0 | — | — | — |

O caranguejo acompanha o erro do estimador em todos os cenários, e não a
dificuldade da marcha: é **maior em terreno irregular** e **menor em rampa** do
que no plano. Confirma que a deriva lateral é o Defeito 1 e não sintonia de
marcha. O S2 é o cenário de referência para trabalhar o estimador.

| # | Cenário | Mundo | Exercita | Guia |
| --- | --- | --- | --- | --- |
| S0 | Plano vazio | `quadruped_empty.sdf` | marcha, referência de tudo | [s0-plano-vazio.md](s0-plano-vazio.md) |
| S1 | Rampa 6° | `quadruped_ramp.sdf` | equilíbrio em inclinação, limite de tilt | [s1-rampa.md](s1-rampa.md) |
| S2 | Terreno irregular | `quadruped_rough.sdf` | colocação de pé, estimador fora do plano | [s2-terreno-irregular.md](s2-terreno-irregular.md) |
| S3 | Corredor | `quadruped_corridor.sdf` | `/demo/scan`, caminho do costmap (F5) | [s3-corredor.md](s3-corredor.md) |
| S4 | Objetos | `quadruped_objects.sdf` | `/demo/camera/*`, contrato de percepção | [s4-objetos.md](s4-objetos.md) |
| S5 | Nav2 desviando | `quadruped_objects.sdf` | malha fechada: nuvem → costmap → MPPI → marcha | [s5-nav2-desvio.md](s5-nav2-desvio.md) |

## Como rodar qualquer um

```bash
# na estação x86, terminal 1 -- sobe o mundo
./scripts/run_quadruped_sim.sh quadruped_ramp.sdf

# terminal 2 -- verifica o contrato de tópicos
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69
python3 scripts/scenario_check.py --seconds 20
```

O argumento de `run_quadruped_sim.sh` **sem barra** é resolvido no `share` de
`demo_simulation`. Com barra, é tratado como caminho absoluto. Passar um nome
que não existe faz o Gazebo subir com um mundo vazio e sem erro visível — se o
robô aparecer num plano infinito quando você pediu uma rampa, foi isso.

## O verificador

`scripts/scenario_check.py` é só leitura: não publica em `/demo/cmd_vel`, então
pode rodar em paralelo com qualquer roteiro de movimento. Ele mede taxa por
tópico contra um piso, e o piso não é alvo de projeto — é o valor abaixo do qual
o consumidor a jusante quebra de forma observável. Um tópico a 0 Hz com o nome
aparecendo no `ros2 topic list` é o modo de falha mais silencioso desta
simulação, e é a razão de o script existir.

## O que é conhecido e esperado em TODOS os cenários

Não são defeitos deste ou daquele mundo; são o estado da aplicação hoje.

- **A árvore TF não fecha no topo.** Medido com QoS correto em `/tf_static`:
  **20 arestas, 8 estáticas, raiz `base`** — `base` → `trunk` → `lidar`,
  `imu_link`, `front_camera` e as quatro pernas até os pés. A árvore do robô é
  completa e o frame `lidar` existe. Faltam **duas arestas no topo**:
  `odom → base` e `map → odom`. O `/demo/odom` já declara
  `header.frame_id: "odom"` numa mensagem cujo frame ninguém publica.
  `base_link` não existe (o URDF usa `base` e `trunk`), e o `nav2_params.yaml`
  pede `base_link` — isso é configuração de arquivo do projeto, não mudança em
  pacote vendorizado.
- **O robô não desvia de obstáculo, e há três razões independentes.** (1) Nada
  consome o scan: `gait_trial.sh` e `demo_routine` são malha aberta, publicam
  velocidade fixa e nunca leem `/demo/scan`. (2) O Nav2 é o consumidor e não
  pode subir — o `nav2_params.yaml` já está configurado para
  `observation_sources: scan` / `/demo/scan` / `/demo/odom`, mas pede os frames
  `map`, `odom` e `base_link`, e **nenhum dos três existe** (o URDF tem `base` e
  `trunk`; `grep base_link` em `go2_description` não retorna nada). (3) Mesmo com
  o Nav2 de pé, o `/demo/scan` é **um anel de um lidar de 16**, e esse anel não
  vê objeto isolado à frente — medido em [s4-objetos.md](s4-objetos.md).
- **`/demo/perception/detections` só existe se `demo_perception` estiver
  rodando**, e quando existe é um stub **determinístico e sintético**: ele não
  olha a imagem. Nenhum cenário aqui valida detecção.
- **Caranguejo de ~2% da distância de avanço.** O estimador acredita que andou
  reto enquanto o robô escorrega de lado. Causa medida em
  `../../results/ml35-postura-parada.md`; é o Defeito 1, e não há ganho de
  marcha que corrija.
- **Tremor de ~2,6° de guinada pico a pico quando parado.**

## Armadilha: "a simulação travou, o robô não levantou"

Sintoma: o Gazebo abre, o robô aparece deitado ou de pé rígido, e nada acontece.
O log repete:

```
[controller_manager]: No clock received, using time argument instead!
```

**Não é o robô.** É o `/clock` não chegando ao lado ROS. Sem ele o
`controller_manager` não avança, a FSM nunca sai do estado inicial, e o robô
nunca fica de pé. Medido em 20/08/2026: o Gazebo estava rodando normalmente —
sim_time 524,9 s, RTF 0,9997, 524.932 iterações — e a `ros_gz_bridge` estava viva
e inscrita no tópico Gazebo certo. Ela simplesmente não repassou.

### Como distinguir em 30 segundos

```bash
# 1. O Gazebo esta avancando? (dentro do container)
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  gz topic -e -t /world/<nome_do_mundo>/stats -n 1'
# -> se real_time_factor ~1.0 e iterations sobe, a fisica esta OK

# 2. O gz /clock publica?
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  gz topic -e -t /clock -n 2'

# 3. O ROS /clock recebe?
docker exec aquila-go2 bash -c '. /opt/ros/jazzy/setup.sh; \
  ros2 topic hz /clock'
```

Gazebo avançando + `gz /clock` publicando + `ros2 topic hz /clock` **sem saída** =
esta falha. Note que `ros2 topic info /clock` mostra contagens de publisher e
subscriber corretas mesmo assim — a contagem não prova entrega, e é por isso que
o `scenario_check.py` conta mensagens numa janela em vez de listar tópicos.

### Causa e conduta

A falha apareceu no quarto container consecutivo de uma bateria, com troca rápida
de containers em `--network=host` e o mesmo `ROS_DOMAIN_ID`. Reproduzindo o mesmo
cenário **sozinho, do zero**, ele sobe normalmente: 0 ocorrências de `No clock`.
Portanto não é o mundo, e não é o cenário — é estado de descoberta DDS
atravessado entre containers.

Conduta:

1. **Nunca rode duas simulações ao mesmo tempo.** Elas disputam o nome
   `aquila-go2` e o domínio DDS. Foi assim que esta falha foi provocada.
2. Derrube o container, espere ele **sair de `docker ps -a`** (com `--rm` a
   remoção é assíncrona e o nome fica reservado), e suba de novo.
3. Entre cenários de uma bateria, dê alguns segundos de folga em vez de
   reiniciar imediatamente.

## Estado do caminho de navegação (medido em 20/08/2026)

Os três bloqueios que esta seção listava como abertos **estão fechados**. O Nav2
sobe, ativa e planeja sobre o quadrúpede. O histórico está preservado abaixo
porque cada bloqueio foi fechado de um jeito que tem consequência.

| peça | estado | evidência |
| --- | --- | --- |
| lidar como sensor real | **sim** | `L1_lidar`, `gpu_lidar`, 640 × 16, 0,05–10 m, 10 Hz |
| `/demo/scan` (LaserScan) | sim, 10 Hz | **2D: um anel dos 16.** Inútil para costmap |
| `/demo/scan_cloud` (PointCloud2) | **sim** | os 16 anéis. 2097 pontos, 249 acima do chão a 1,31–4,43 m |
| odometria `/demo/odom` | sim, 50 Hz | **ground truth do Gazebo**, não estimativa com perna |
| `odom` → `base` na TF | **sim** | `demo_bringup/odom_tf` |
| `map` → `odom` na TF | **sim** | identidade, mesmo nó. Não é localização |
| árvore TF completa | **sim** | 22 arestas, 9 estáticas, raiz `map` |
| frame base | resolvido | é `base`; quem cedeu foi o Nav2, não o URDF |
| Nav2 configurado para o Go2 | **sim** | `nav2_params_go2.yaml`, 13 deltas sobre o do TB4 |
| Nav2 ativando e planejando | **sim** | `Managed nodes are active`; ver [s5-nav2-desvio.md](s5-nav2-desvio.md) |

### Como cada bloqueio foi fechado, e o que isso custou

1. **Frames.** `demo_bringup/odom_tf` republica `/demo/odom` como `odom → base` e
   publica `map → odom` como identidade. Isso **não é estimativa de estado** — é
   ground truth do simulador virando TF. Serve para exercitar percepção e
   planejamento; é inútil como validação de localização, e sai quando a F5
   entregar o estimador com perna.

   Armadilha: dois publicadores na mesma aresta da TF **não dão erro**. O
   consumidor recebe as duas e usa a última que chegou. Se subir SLAM ou AMCL,
   passe `publish_map_identity:=false`.

2. **Nome do frame.** Quem cedeu foi o arquivo de parâmetros, não o URDF:
   `nav2_params_go2.yaml` usa `base`. `go2_description` é vendorizado com garantia
   byte-a-byte que sustenta o argumento de licença, então o nome do frame é dado.

3. **Representação do lidar.** Nem reapontar o sensor nem editar o URDF: a ponte
   passou a expor `/scan/points` como `PointCloud2` em `/demo/scan_cloud`, e o
   costmap consome isso. A ponte é o ponto de injeção que o projeto possui.

   O número que justifica: no mundo `quadruped_objects.sdf`, com quatro objetos a
   1,5–4,5 m, `/demo/scan` dá **zero** obstáculos — numericamente idêntico ao
   mundo vazio — e `/demo/scan_cloud` dá **249** pontos de obstáculo. No corredor,
   2422 a partir de 1,06 m.
