# ML3.5 F5 — saída autônoma do labirinto: implementação fechada, aceitação PENDENTE

**Estado:** `PENDING EXECUTION` — implementação e suítes de host concluídas;
**nenhuma corrida de aceitação foi executada, no host ou no Aquila.**
**Data do registro:** 28/08/2026
**Regra 7:** nada aqui reivindica validação de hardware. Toda linha da seção 4
está vazia de propósito e só pode ser preenchida com evidência real capturada
sob `docs/results/`.

---

## 1. O que a entrega faz

O robô parte **sem mapa prévio**, extrai fronteiras do mapa vivo do SLAM,
navega até elas e, ao reconhecer visualmente o painel magenta da saída,
aproxima-se dele em passos curtos e atravessa a abertura. Nenhuma meta manual,
nenhum waypoint, nenhuma coordenada do labirinto no processo de decisão.

| Peça | Onde roda | Arquivo |
| --- | --- | --- |
| Extração de fronteiras (sem ROS) | módulo | `demo_navigation/frontier.py` |
| Executivo de exploração | módulo | `demo_navigation/maze_explorer.py` |
| Planner que não cruza desconhecido | módulo | `ExplorationGrid`, `nav2_params_go2.yaml` |
| Árvore Nav2 de exploração | módulo | `behavior_trees/nav_to_pose_exploration.xml` |
| Detector do marcador | módulo | `demo_perception/maze_exit_detector.py` |
| Painel magenta | host (mundo) | `worlds/quadruped_maze11.sdf` |
| Validador de ground truth | **host, só simulação** | `demo_simulation/maze_escape_validator.py` |
| Botões e HUD de busca | cockpit | `hmi/js/panels/exploration.js` |

A separação da última linha é o que torna a aceitação uma medida: o validador
observa a odometria ground truth e publica `/demo/maze/escaped`, e **o
explorador não assina esse tópico**. Há teste estrutural afirmando que ele não
sobe nos launches do lado do robô.

## 2. Interfaces públicas

| Interface | Tipo | Uso |
| --- | --- | --- |
| `/demo/exploration/start` | `std_srvs/Trigger` | arma a busca; recusa a segunda |
| `/demo/exploration/cancel` | `std_srvs/Trigger` | idempotente; o reset do Nav2 chama antes |
| `/demo/exploration/status` | `std_msgs/String`, JSON, transient-local | estado, tempo, fronteiras, marcador, falha |
| `/demo/perception/maze_exit/detections` | `vision_msgs/Detection2DArray` | caixa no quadro |
| `/demo/perception/maze_exit/pose` | `geometry_msgs/PoseStamped` | frame da câmera |
| `/demo/maze/escaped` | `std_msgs/Bool`, transient-local | **oráculo de aceitação** |

Estados publicados: `idle`, `waiting_map`, `selecting`, `navigating`,
`homing_exit`, `completed`, `failed`, `cancelled`.

## 3. O que já foi executado (host, 28/08/2026)

| Suíte | Resultado |
| --- | --- |
| `pytest tests/` (contrato) | **150 passam** |
| `demo_navigation` (funcional + lint) | **25 passam** |
| `demo_perception` (funcional + lint) | **33 passam** |
| `npm test` no `hmi/` | **169 passam** |

Lint **pré-existente e não introduzido aqui**, vermelho em HEAD antes desta
entrega: `demo_simulation` (`quadruped.launch.py:123` E501,
`test_scene_cameras_launch.py` D205/D400) e `demo_bringup`
(`nav_quadruped.launch.py:57` I100, `test_camera_transport.py` D213).

Nada disso mede navegação. São testes de host: contrato, máquina de estados,
fiação de launch e dependências de manifesto.

## 4. Aceitação — NÃO EXECUTADA

### 4.1 Portão de estabilidade (bloqueante, precede tudo)

Três metas curtas conectadas, `--goals=maze11-short`, com o MPPI atual.

| Critério | Exigido | Medido |
| --- | --- | --- |
| metas `SUCCEEDED` | 3 de 3 | — |
| `worldToMap failed` | 0 | — |
| quedas | 0 | — |
| velocidade média | ≥ 0,05 m/s | — |
| trocas de plano > 1 m ou > 45° sem mudança de mapa | 0 | — |

Última corrida conhecida (`artifacts/maze11-short-gate.csv`, 28/08):
**REPROVADA** — 37,1 s de simulação, 0,69 m percorridos, **0,0185 m/s**,
`cmd_vx ≠ 0` em 18,7% das amostras. A corrida anterior, antes de
`restamp_tf: true`, tinha o robô **congelado**: `cmd_vx` zero em 150 s.
O parâmetro destravou o comando e **não fechou o portão**.

Ordem de investigação, uma variável por corrida: resultado da ação →
instabilidade do plano → desvio temporal do TF → frequência do controller. O
desfecho e o `error_code` de cada meta agora ficam em `<csv>-metas.csv`.

### 4.2 Aceitação HIL final

| Critério | Corrida 1 | Corrida 2 | Corrida 3 |
| --- | --- | --- | --- |
| partida fria, sem mapa salvo | — | — | — |
| `/demo/maze/escaped=true` em ≤ 600 s | — | — | — |
| tempo até a saída | — | — | — |
| percurso | — | — | — |
| fronteiras avaliadas | — | — | — |
| falhas de fronteira | — | — | — |
| instante da primeira detecção | — | — | — |
| razão de trabalho | — | — | — |
| cobertura do mapa | — | — | — |
| quedas | — | — | — |
| `worldToMap failed` | — | — | — |

## 5. Riscos abertos, a verificar na bancada antes de culpar o algoritmo

1. **`camera_info` no módulo.** Desde `ml35-f5-camera-comprimida.md` o RAW não
   atravessa mais o fio: a imagem chega comprimida e é religada por `SetRemap`
   em `demo_bringup/launch/perception.launch.py`. Esse remap cobre a imagem do
   detector automaticamente, **mas `/demo/camera/camera_info` não é
   remapeado**. Se ele não atravessar, o detector publica detecção e **nunca**
   publica pose — falha silenciosa. Primeira coisa a checar:
   `ros2 topic hz /demo/camera/camera_info` **no módulo**.
   Há teste nomeando esse modo de falha
   (`test_no_pose_without_camera_info`).
2. **CPU do detector.** Laço Python puro, ~19 200 iterações por quadro a 10 Hz,
   num módulo já medido a ~600% de 800%. Compete exatamente com o recurso que o
   portão está falhando. Mitigação sem recompilar: subir `sample_stride`.
   **Só mede no Aquila** (regra 5).
3. **Visibilidade do painel.** O marcador está 1,7 m além da abertura e só é
   visível através dela. Ângulo de aproximação estreito; se o robô cruzar a
   região sem apontar a câmera para a abertura, a detecção não acontece.
4. **Oscilação de plano.** A entrega evita reavaliar fronteira a cada `/map`
   justamente por causa da instabilidade já medida, mas isso é uma decisão de
   projeto, não uma medida. Só a corrida diz.

## 6. Como reproduzir quando a bancada estiver livre

```bash
# host — portão primeiro, e só ele
python3 scripts/nav_trial.py artifacts/gate.csv --goals=maze11-short --seconds 180
#   -> artifacts/gate.csv         telemetria por amostra, com goal_index
#   -> artifacts/gate-metas.csv   uma linha por meta: desfecho e error_code

# cadeia oficial para o HIL
scripts/module.sh sync && scripts/module.sh build
scripts/module.sh up && scripts/module.sh verify

# no módulo, ANTES de acusar a percepção
ros2 topic hz /demo/camera/camera_info
ros2 topic hz /demo/perception/maze_exit/detections

# a busca, pelo cockpit ou pela linha de comando
ros2 service call /demo/exploration/start std_srvs/srv/Trigger
ros2 topic echo /demo/exploration/status
ros2 topic echo /demo/maze/escaped
```
