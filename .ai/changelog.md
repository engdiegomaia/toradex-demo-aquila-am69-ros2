# Changelog

Registro de fases e decisões do projeto. Uma entrada por fase concluída.
Formato: mais recente primeiro.

---

## 2026-08-07 — ML2: modelo de robô e TF

**Entregue:** pacote `demo_description` — xacro diff-drive parametrizado,
árvore TF, config RViz de desenvolvimento (x86), testes de URDF.

Arquivos: `urdf/{demo_robot.urdf.xacro,_materials,_inertia,_wheel,_sensors}.xacro`,
`launch/view_robot.launch.py`, `rviz/demo_description.rviz`,
`test/test_urdf_parses.py`, `README.md`.

**Aceitação ML2 (`.ai/AGENTS.md` §9):**

| Critério | Estado |
| --- | --- |
| xacro expande sem erro | OK |
| frames obrigatórios existem | OK — 7 links, 6 joints, raiz única `base_footprint` |
| sem publicador TF duplicado | OK — tabela de posse no README |
| RViz funciona no x86 | Config válida e launch importável; **confirmação visual pendente do operador** |
| `colcon test` | 13 testes no workspace, 0 falhas |

### Decisão: diff-drive agora, quadrúpede em ML3.5

O robô-alvo da demo é um quadrúpede. ML2/ML3 entregam base diff-drive de
propósito. Em agosto/2026 nenhum projeto mantido entrega
quadrúpede + Jazzy + Gazebo Harmonic + Nav2 funcionando:

- `chvmp/champ` upstream é ROS 1 apenas (último commit jul/2024, `move_base`/`amcl`).
- Melhor base Jazzy+Harmonic (`khaledgabr77/unitree_go2_ros2`) lista Nav2 como
  "coming soon" desde mai/2025 — não entregue em ~15 meses.
- Único CHAMP+Nav2 demonstrado (`arjun-sadananda/go2_nav2_ros2`) é Humble +
  Gazebo **Classic**, e compensa erro de odometria dobrando a velocidade linear
  no estimador de estado — contorno, não calibração.

Odometria por dead-reckoning de cinemática de marcha deriva muito mais que
odometria de rodas, e a localização do Nav2 depende disso. Assumir esse risco
primeiro bloquearia containers, emulação arm64 e bring-up de hardware — o
objetivo real da demo.

O contrato de tópicos torna a decisão reversível: tudo a jusante fala
`/demo/cmd_vel` (`geometry_msgs/Twist`), então trocar por quadrúpede depois fica
confinado a `demo_description` + `demo_simulation`. Nenhuma mudança em Nav2,
percepção, HMI ou containers. Rastreado como ML3.5.

### Decisão: plugin nativo `gz-sim-diff-drive-system`, não `gz_ros2_control`

O `.so` já vem com o Gazebo Harmonic do host (verificado:
`libgz-sim8-diff-drive-system.so.8.14.0`), custo zero de pacotes. `gz_ros2_control`
arrastaria `ros2_control` + `controller_manager` + `diff_drive_controller` —
superfície muito maior para um milestone cuja aceitação é "teleop, tópicos
trocam mensagens, Nav2 ativo, goal completa". É o padrão de `nav2_minimal_tb*_sim`
e do `diff_drive.sdf` do próprio Gazebo. Migrar depois não altera nada a jusante.

### Correções de infraestrutura (pré-existentes)

- **Symlinks quebrados** em `ros2_ws/install/`: 4 apontavam para
  `/home/diego-maia/toradex/custom-cases/2608-demo-aquila-ros2/`, caminho onde o
  repo vivia antes. Quebravam `colcon build` do `demo_tutorials`. Removidos.
- **`.gitignore` ausente**: `build/`, `install/` e `log/` estavam a caminho de
  serem commitados. Com `--symlink-install` esses diretórios contêm caminhos
  absolutos — commitá-los reproduz exatamente o bug acima em outra máquina.
  Criado.

### Pendente para o operador

```bash
sudo apt install -y liburdfdom-tools    # check_urdf (sudo precisa de senha)
ros2 launch demo_description view_robot.launch.py   # confirmação visual no RViz
```

---

## 2026-07-31 — L1: fundamentos ROS 2 (ML1)

Pacote `demo_tutorials`: heartbeat publisher/subscriber em
`/demo/system/heartbeat` com `rate_hz` retunável em runtime, serviço
`AddTwoInts`, `heartbeat.launch.py` e `turtlesim_demo.launch.py`, testes
unitários + flake8 + pep257.

Fases L0a (ROS 2 Jazzy + turtlesim), L0b (Gazebo Harmonic standalone) e
L0c (`ros_gz_bridge`) concluídas antes, documentadas em `docs/development.md`.
