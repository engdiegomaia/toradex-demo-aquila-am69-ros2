# Procedência — camada de controle vendorizada

Vale para os quatro pacotes vendorizados juntos em F3:

- `control_input_msgs`
- `controller_common`
- `unitree_guide_controller`
- `gz_quadruped_hardware`

**Nenhum deles é nosso.** Mesmo padrão de `go2_description/README.md` e de
`demo_navigation/launch/nav2_vendored/`: mudar o mínimo, provar a procedência,
deixar o diff visível.

---

## Origem

- **Repositório:** <https://github.com/legubiao/quadruped_ros2_control>
- **Commit:** `5434c5810d1a7fe223bcfd04550e9d3bfdd4b458` ("x30 repaint", 24/06/2025)
- **Caminhos:** `commands/control_input_msgs`, `libraries/controller_common`,
  `controllers/unitree_guide_controller`, `hardwares/gz_quadruped_hardware`

Extraídos da imagem `demo-sim:spike-go2`, construída em F2 a partir de clone
raso desse commit. Nomes upstream preservados: os `$(find <pacote>)` dos xacro e
dos launch resolvem sem edição, e o diff contra o upstream fica nulo.

---

## Licença — o que a auditoria encontrou

Os `package.xml` declaram `Apache-2.0` (ou `Apache 2`), mas a cobertura real é
mista. Medido, não presumido:

| Pacote | `package.xml` declara | Cobertura real | Titular |
|---|---|---|---|
| `gz_quadruped_hardware` | `Apache 2` | `LICENSE` próprio, headers Apache em **5/5** fontes | Open Source Robotics Foundation |
| `unitree_guide_controller` | `Apache-2.0` | `LICENSES/unitree_guide/LICENSE.txt` da raiz upstream — **BSD 3-Clause** | Unitree Robotics |
| `controller_common` | `Apache-2.0` | idem (código extraído do `unitree_guide_controller`) | Unitree Robotics |
| `control_input_msgs` | `Apache-2.0` | idem; só definições de mensagem, 20 KB, sem fonte C++ | Unitree Robotics |

### O que isso significa

`gz_quadruped_hardware` é um **fork do `gz_ros2_control` da OSRF**, e é o caso
limpo: `LICENSE` próprio no pacote e header Apache completo em todos os cinco
fontes. Nada a resolver.

Os outros três derivam do **`unitree_guide` da Unitree**, portado para ROS 2 pelo
`legubiao`. Os fontes em sua maioria não têm header de copyright — carregam só
`// Created by tlab-uav on 24-9-6.` — o que na auditoria crua parece licença
ausente. Não é: o repositório upstream mantém `LICENSES/unitree_guide/LICENSE.txt`
na raiz **exatamente** para cobrir esse código, e o texto é BSD 3-Clause,
copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics").

Confirmação independente: `include/unitree_guide_controller/common/mathTypes.h`
é o único fonte com header, e o header diz
`Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.` —
mesmo titular.

É **o mesmo titular e a mesma licença** das malhas do `go2_description`, cuja
identidade com `unitreerobotics/unitree_ros` está provada por hash. As duas
camadas da demo (descrição e controle) convergem para a mesma origem e a mesma
licença BSD-3.

### Diferença em relação ao caso do a1_description

Aqui existe texto de licença e titular identificado — o que faltava no
`a1_description` (`<license>TODO</license>`, sem texto, sem copyright) e que fez
o alvo do ML3.5 mudar de A1 para Go2 em F2. A cláusula 1 do BSD-3 (reter o aviso
de copyright na redistribuição de fonte) é cumprível: o `LICENSE` está em cada
pacote.

---

## Edições feitas sobre o upstream

### 1. `LICENSE` adicionado a três pacotes

`control_input_msgs`, `controller_common` e `unitree_guide_controller` receberam
cópia de `LICENSES/unitree_guide/LICENSE.txt` do repo upstream, íntegra. Upstream
mantém esse texto só na raiz; ao extrair os pacotes individualmente ele
precisa viajar junto, ou a redistribuição não cumpre a cláusula 1.

`gz_quadruped_hardware` já trazia o seu e não foi tocado.

### 2. `package.xml` — licença precisada

Nos três pacotes derivados do `unitree_guide`, `<license>Apache-2.0</license>`
foi corrigido para `<license>BSD-3-Clause</license>`, que é o que o texto que os
cobre de fato diz. Declarar Apache sobre código BSD-3 estaria errado nas duas
direções.

`gz_quadruped_hardware` teve `Apache 2` normalizado para `Apache-2.0` (grafia
SPDX). Titular e conteúdo intactos.

### 3. Código de `unitree_guide_controller` editado a partir de F4

Isto **mudou** depois da vendorização, e a versão anterior deste arquivo dizia o
contrário. A frase "nenhum arquivo de código foi editado" era verdade em
17/08/2026 e deixou de ser no dia seguinte, quando F4 começou a mexer na marcha.
Registrado aqui em vez de corrigido em silêncio.

Editados em `unitree_guide_controller`, todos com o número medido e o motivo em
comentário no ponto de uso:

| Arquivo | O que mudou |
|---|---|
| `src/FSM/StateTrotting.cpp` + `.h` | reescrito: modos WALK/HOLD/RECOVER, supervisor de atitude, banda de referência dimensionada ao comando, diagnóstico e instrumentação do eixo de guinada |
| `src/control/BalanceCtrl.cpp` + `.h` | inércia do Go2 no lugar da do A1; pesos do QP e cone de atrito vindos de parâmetro |
| `src/gait/FeetEndCalc.cpp` | ganho de rumo `k_yaw` 0,005 → 0,15; os três ganhos de Raibert vindos de parâmetro |
| `src/gait/GaitGenerator.cpp` + `.h` | alvo de apoio reancorado no toque e enquanto a marcha está parada |
| `src/control/Estimator.cpp` + `.h` | acesso a estado usado pelo diagnóstico |
| `src/UnitreeGuideController.cpp` + `.h` | declaração e validação dos parâmetros de marcha |
| `include/.../control/GaitParams.h` | **arquivo novo, nosso**: a superfície de sintonia |

Histórico completo em `git log ae3d9a1..HEAD --
ros2_ws/src/unitree_guide_controller/`; a evidência que motivou cada mudança
está em `docs/results/ml35-f4-parcial.md`.

**O que continua intacto, e por quê importa:** `src/quadProgpp/` (solver de
terceiro), `CMakeLists.txt`, `package.xml` além da licença, e o plugin XML. E,
fora deste pacote, `go2_description/` inteiro — nenhum arquivo em `meshes/`,
`xacro/`, `urdf/` ou `config/` foi tocado. Foi por isso que a sintonia da marcha
foi para `demo_simulation/config/gait_go2.yaml`, injetada pelo spawner, em vez de
para `go2_description/config/gazebo.yaml`: o argumento de licença daquele pacote
depende de ele continuar byte a byte igual ao upstream.

Nenhum arquivo CMake ou xacro foi editado em nenhum dos quatro pacotes.

---

## Por que estes quatro, e não o repo inteiro

O repo upstream traz 24 pacotes. Entram só os que a demo usa:

- `unitree_guide_controller` — controlador de marcha PD clássico, sem política
  de RL. É o que fez o portão de F2 bater.
- `controller_common` — biblioteca de que o controlador depende.
- `control_input_msgs` — o tipo `Inputs` que o controlador consome.
- `gz_quadruped_hardware` — o plugin `ros2_control` que roda dentro do processo
  do `gz sim`. **Achado de F2:** é o plugin *deste repo*, versão 2.0.6, não o
  `gz_ros2_control` 1.2.19 do apt. O plano original supunha o do apt; instalar o
  do apt e esperar que a base o use é suposição não verificada.

Ficam de fora, deliberadamente: `ocs2_quadruped_controller` e
`rl_quadruped_controller` (controladores que a demo não usa),
`hardware_unitree_sdk2` (SDK do robô físico — e é onde mora a colisão
CycloneDDS × `unitree_sdk2` registrada em `docs/ml35/estado-fases.md`),
`unitree_joystick_input`, e todas as descrições de outros robôs.

---

## Ao atualizar estes pacotes

1. Reconferir se `LICENSES/` da raiz upstream ainda cobre o que se traz.
2. Se um fonte ganhar header de copyright upstream, ele passa a valer sobre esta
   tabela — atualizar aqui.
3. Não trazer `hardware_unitree_sdk2` sem antes resolver a colisão de RMW: a
   regra 2 do projeto é `rmw_cyclonedds_cpp` sempre, e o SDK pede FastDDS.
