# S4 — Objetos no campo de visão

Mundo: `quadruped_objects.sdf` · Estação x86 · ~4 min

## Para que serve

Exercita `/demo/camera/image_raw` e `/demo/camera/camera_info`, e o consumidor
desse contrato, `demo_perception`.

## ATENÇÃO antes de ler qualquer resultado

`demo_perception` hoje é um **stub de detecção sintética e determinística**. Ele
**não olha a imagem** — é o contrato de tópicos do `CLAUDE.md` sendo respeitado
para que a inferência TIDL possa entrar depois sem refatoração.

Portanto este cenário **não valida detecção**. Ele valida que a imagem chega, com
a geometria e a taxa certas, e que `/demo/perception/detections` continua sendo
publicado e consumido. Confundir as duas coisas é exatamente como o stub viraria
"visão funcionando" num relatório.

## Geometria

Quatro objetos em cores saturadas, distintas do chão cinza e do fundo azul:

| objeto | posição (x, y) | forma |
| --- | --- | --- |
| caixa vermelha | 1,5 · 0,0 | 0,30 m cúbica |
| cilindro verde | 3,0 · +0,45 | r 0,18, h 0,50 |
| caixa azul | 3,0 · −0,55 | 0,40 × 0,40 × 0,60 |
| cilindro amarelo | 4,5 · 0,0 | r 0,12, h 0,80 |

Três distâncias (1,5 / 3,0 / 4,5 m) para que uma futura inferência tenha alvos
em escalas diferentes sem precisar de outro mundo.

## Rodar

```bash
./scripts/run_quadruped_sim.sh quadruped_objects.sdf
python3 scripts/scenario_check.py --seconds 20
```

Andar pouco — a caixa vermelha está a 1,5 m à frente:

```bash
./scripts/gait_trial.sh /tmp/s4.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 8 --hold 5
```

Para ver a imagem, com `demo_perception` rodando e o contrato fechado:

```bash
ros2 launch demo_perception perception.launch.py   # ou o launch equivalente
ros2 topic hz /demo/perception/detections
ros2 run rqt_image_view rqt_image_view /demo/camera/image_raw
```

`rqt_image_view` e RViz2 rodam **só na estação x86** (regra 1 do `CLAUDE.md`).

## Aceitação

Medido em 20/08/2026, robô parado, janela de 15 s.

| medida | valor | aceitação |
| --- | --- | --- |
| `/demo/camera/image_raw` | 10 Hz, 640×480 `rgb8`, 921600 bytes | > 5 Hz |
| geometria contra `camera_info` | confere | igual |
| intensidade média | 179,4 (contra 180,9 no mundo vazio) | — |
| `RECOVER` | 0 | 0 |

A câmera passa. **O lidar não vê os objetos** — ver abaixo, e é o achado
principal deste cenário.

### O lidar 2D não vê obstáculo isolado à frente

| mundo | feixes válidos | alcance mínimo |
| --- | --- | --- |
| S0 vazio | 46/640 (7%) | 4,80 m (só chão) |
| **S4 objetos** | **46/640 (7%)** | **4,82 m** |
| S3 corredor | 358/640 (56%) | 0,72 m |

O S4 é **numericamente idêntico ao mundo vazio**: os quatro objetos, de 0,30 a
0,80 m de altura, a 1,5–4,5 m à frente, contribuem **zero** retornos.

Causa. O `L1_lidar` do Go2 (`go2_description/xacro/gazebo.xacro:288`) é um
`gpu_lidar` **3D**: 640 amostras horizontais em 200° por **16 anéis verticais**
em ±15°, montado em `trunk` com `rpy="0 2.8782 0"` — 164,9° de pitch, que é o
domo do L1 real olhando para baixo e para frente.

A bridge mapeia para `sensor_msgs/LaserScan`, que é **2D**. O `/demo/scan`
carrega 640 ranges, ou seja **um anel dos 16** — os outros quinze são descartados
na travessia, sem aviso. E o anel exposto aponta de tal forma que vê parede rente
ao corpo (S3, mínimo 0,72 m) e chão a 5–10 m, mas não objeto isolado à frente.

**Consequência de projeto:** um costmap do Nav2 alimentado por `/demo/scan` não
veria justamente os obstáculos que o robô precisa desviar. Resolver isso é
escolher entre expor o lidar como `PointCloud2` (os 16 anéis) em vez de
`LaserScan`, ou remontar/reapontar o sensor. Nenhuma das duas foi feita.

## Armadilha específica deste cenário

**Câmera a 0 Hz com o tópico listado** é o modo de falha clássico, e a causa
quase sempre é o mundo não carregar o sistema `Sensors` do Gazebo. Todos os
mundos deste diretório carregam `gz-sim-sensors-system` com `ogre2` de propósito;
se você criar um mundo novo copiando de `empty.sdf` do Gazebo, a câmera existe no
modelo e não publica nada, **sem nenhum erro nomeando o plugin que falta**.
