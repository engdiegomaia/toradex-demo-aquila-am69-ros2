# ML3.5 — labirintos: o que serve para o Go2, e onde nascer

Estação x86. Data: 21/08/2026. Mede **geometria de mundo**, offline, a partir do
STL — não roda ROS, não roda Gazebo, não move o robô.

**NÃO VALIDADO EM HARDWARE.** Nenhum número aqui é do Aquila AM69, e nenhum
número aqui é de desempenho. É a resposta a uma pergunta anterior a qualquer
corrida: *este labirinto cabe o robô, e onde ele deve nascer?*

Este arquivo fecha a referência que o cabeçalho de
`demo_simulation/worlds/quadruped_maze.sdf` já fazia a `docs/results/ml35-labirinto.md`
e que até 21/08/2026 **não existia**.

---

## Reprodutível: `scripts/maze_fit.py`

```bash
git clone --depth 1 https://github.com/cafemesa/ros_maze_worlds.git ~/ros_maze_worlds
python3 scripts/maze_fit.py --models ~/ros_maze_worlds/models maze10 maze11
```

Sai `SERVE` / `NÃO SERVE` por labirinto, mais a `<pose>` recomendada pronta para
colar no SDF. Código de saída ≠ 0 se algum labirinto não servir.

### O método, e por que ele é assim

1. **Pegada das paredes.** Os labirintos são caixas extrudadas; as **faces
   horizontais** do STL são o topo e a base dessas caixas, logo a projeção delas
   em XY é exatamente a pegada. Rasterizar só as horizontais evita ter de fechar
   sólido, e é o que dispensa biblioteca de malha.
2. **Transformada de distância** do espaço livre → folga até a parede mais
   próxima em cada célula.
3. **Cabe o robô** = folga ≥ 0,383 m, o raio circunscrito do tronco do Go2
   (0,70 × 0,31 m). O robô só passa por vão maior que **0,77 m**.
4. **Componentes conectados** sobre essas células. O número que decide se existe
   patrulha é a área do **maior componente**, não a área total: dois bolsões
   separados por um corredor estreito somam área e não servem para nada.
5. **Pose de nascimento**, em dois modos (`--start`):
   - `run` — célula com a maior corrida livre em `+x`. Serve para ensaio de
     marcha reta.
   - `se` / `ne` / `nw` / `sw` — célula mais próxima daquele canto. É o que uma
     demonstração quer: começar numa ponta e atravessar o labirinto inteiro em
     vez de nascer no meio dele.

   Em qualquer dos dois a célula sai do conjunto com folga ≥ 0,55 m, ou seja
   centro de corredor. Sem essa condição o robô nasce encostado numa parede e o
   primeiro passo já raspa.
6. **Yaw de nascimento.** A pose sozinha não basta. O robô nasce olhando para
   `+x` e a primeira coisa que faz é andar para frente; num canto, `+x` costuma
   ser parede. O script mede a pista livre nas quatro direções na célula
   escolhida e recomenda o yaw da maior.

A pose do modelo é o **negativo** do ponto escolhido: o robô nasce em (0, 0),
então é o labirinto que se move até o robô cair no lugar certo. O `map` fica
alinhado com o mundo do Gazebo (`odom_tf` publica `map → odom` identidade), logo
as metas em coordenadas do robô valem com qualquer yaw de nascimento.

---

## Resultado: `maze10` e `maze11` são geometricamente gêmeos

Escala 0,002 (o dobro do upstream), resolução de rasterização 0,01 m.

| | `maze10` | `maze11` |
| --- | --- | --- |
| triângulos no STL | 156 | 284 |
| pegada bruta do STL | 4200 × 4200 × 300 | 5800 × 5800 × 300 |
| pegada a 0,002 | 8,40 × 8,40 m | **11,60 × 11,60 m** |
| altura da parede | 0,60 m | **0,60 m** |
| corredor (mediana) | 1,20 m | **1,20 m** |
| margem por lado | 21,7 cm | **21,7 cm** |
| área navegável (R = 0,383) | 18,4 m² | **35,4 m²** |
| componentes conectados | **1** | **1** |
| veredito | SERVE | **SERVE** |

**O `maze11` tem o mesmo passo de célula do `maze10`** — 600 unidades brutas de
STL. O que muda é o tamanho da grade, 5800 contra 4200. Por isso os dois
critérios que dimensionaram `<scale>0.002</scale>` no `maze10` valem verbatim:

1. corredor de **1,20 m** contra os 0,77 m que o tronco precisa;
2. parede de **0,60 m** contra o lidar L1 assentado a ~0,306 m do chão — parede
   na altura exata do plano de varredura entra e sai do scan conforme o tronco
   oscila, e isso parece defeito de bridge, não geometria.

**Consequência prática, e é a que economiza o dia:** trocar `maze10` por
`maze11` **não pede reajuste** de escala, `robot_radius`, `inflation_radius`,
resolução de costmap nem sintonia de marcha. A geometria que esses parâmetros
enxergam é idêntica. O que muda é a área: quase o dobro, num único componente.

### Escalas alternativas do `maze11`, medidas e descartadas

| escala | pegada | parede | corredor | margem/lado | área navegável |
| --- | --- | --- | --- | --- | --- |
| 0,001 (upstream) | 5,80 m | **0,30 m** | 0,60 m | **−8,3 cm** | **0 m², 0 componentes** |
| **0,002** | 11,60 m | 0,60 m | 1,20 m | 21,7 cm | **35,4 m², 1 componente** |
| 0,0025 | 14,50 m | 0,75 m | 1,50 m | 36,7 cm | 73,3 m², 1 componente |
| 0,003 | 17,40 m | 0,90 m | 1,80 m | 51,7 cm | 123,0 m², 1 componente |

A escala do upstream reprova nos **dois** critérios ao mesmo tempo, e é por isso
que ela é inutilizável e não apenas apertada: corredor de 0,60 m contra os
0,77 m necessários (margem **negativa**, zero células e zero componentes), e
parede de **0,30 m** contra o lidar a ~0,306 m — a parede fica *dentro* do plano
de varredura. `maze_fit.py --scale 0.001` devolve código de saída 1.

0,0025 e 0,003 dariam mais margem e são **rejeitadas por outro motivo**: a
0,002 o `maze11` já reproduz exatamente o corredor validado no `maze10`, e
mudar escala junto com labirinto trocaria duas variáveis de uma vez. Se a
margem de 21,7 cm se mostrar insuficiente contra a deriva lateral (Defeito 1),
0,0025 é o próximo passo — e aí é **uma** variável.

---

## Poses medidas, por labirinto

Escala 0,002. Colar em `<model name="labirinto">` com `rpy = 0 0 0`.

| labirinto | `--start` | `<pose>` em uso | yaw | folga | extensão navegável (coords do robô) |
| --- | --- | --- | --- | --- | --- |
| `maze10` | `run` (à mão) | `-2.670 3.061 0 0 0 0` | 0 | — | — |
| **`maze11`** | **`se`** | **`-11.672 11.649 0 0 0 0`** | **1,5708** | **0,55 m** | x[−9,87 … +0,16] y[−0,94 … +9,87] |

### Por que o `maze11` exige yaw 1,5708 e não é opcional

Pista livre medida na célula de nascimento (canto inferior direito):

| direção | pista livre |
| --- | --- |
| `+y` | **3,47 m** |
| `−x` | 3,47 m |
| `+x` | **0,16 m** ← é para cá que o robô olha com yaw 0 |
| `−y` | 0,16 m |

Com yaw 0 o robô nasce a 16 cm de uma parede. Sair dali exige girar parado, que
é o que este robô faz pior: teto de guinada de ~0,13 rad/s com `Mz` no batente
de 5,3 N·m. Por isso o mundo **declara o próprio yaw**, numa linha
`<!-- go2_spawn_yaw: 1.5708 -->`, e `run_quadruped_sim.sh` a lê e passa como
`yaw:=`. Nada a lembrar na linha de comando, e o número fica ao lado da
geometria que o justifica. `GO2_SPAWN_YAW=<rad>` sobrepõe.

Medido na simulação depois da mudança: `pose: x=-0.002 y=0.042 z=0.347
yaw=90.0 deg`, lidar `min 0.55 m` — igual à folga que a rasterização previu.

### Metas de patrulha do `maze11`, medidas

`maze_fit.py --start se --goals 4`, em coordenadas do robô, todas em centro de
corredor e dentro dos 8 m de `MAX_GOAL_RADIUS_M`:

```
(0.00, 8.00)   (-8.00, 0.00)   (-1.60, 1.60)   (-5.83, 4.91)
```

Metas a menos de 2 m da partida são descartadas pelo script
(`--min-goal-radius`): não são travessia, e numa patrulha viram uma parada que
não mede nada.

### A pose do `maze10` em uso não é a que o script recomenda, e está certo assim

`maze_fit.py` recomenda `-2.440 5.051` para o `maze10` (corrida 6,68 m). A que
está commitada é `-2.670 3.061` (corrida 6,45 m, medida à mão em 20/08). **As
duas são centros de corredor válidos** — o script apenas desempata diferente.

Não "corrija" o `maze10` para casar com o script: aquela pose é a que produziu
toda a evidência de S6 até aqui, e trocá-la invalidaria a comparação sem
comprar nada.

### Duas armadilhas que a extensão do `maze11` revela

1. **`MAX_GOAL_RADIUS_M = 8.0`** é constante em `patrol_commander.py:58`. A
   ponta `y = −10,38 m` do componente navegável **é rejeitada na entrada**. Ou
   as metas de patrulha ficam dentro de 8 m, ou a constante vira parâmetro com o
   motivo escrito. Rejeitar na entrada é o comportamento correto — o costmap
   global é janela rolante e uma meta fora dela é *aceita* e depois falha perto
   da borda, onde o erro já não tem nome.
2. **A pegada de 11,60 m cabe** na janela rolante do costmap global (20 m), mas
   com menos folga que os 8,40 m do `maze10`. Não é bloqueio; é a razão de a
   extensão estar tabelada aqui.

---

## Verificação em runtime — 21/08/2026, `quadruped_maze11.sdf`

A rasterização é geometria de arquivo. Isto é o mundo carregado de verdade, com
o robô nele. Estação x86, container `demo-sim:spike-go2`.

| critério | medido | veredito |
| --- | --- | --- |
| mundo carrega | `World [quadruped_maze11] initialized with [1ms] physics profile` | passa |
| malha resolve | **0** ocorrências de `Unable to find file` / `Failed to load mesh` | passa |
| robô de pé | `state=fixed stand`, `z = 0,347 m` | passa |
| yaw de nascimento | **90,0°** — o mundo declarou, o script leu, o launch aplicou | passa |
| folga prevista × medida | previsto 0,55 m · lidar `min = 0,55 m` | **casa** |
| lidar com retorno real | **475 de 640 feixes válidos (74%)**, 0,55–3,91 m | passa |
| nuvem de obstáculo | **2581 pontos** em 0,12 < z < 1,0 m, faixa 0,55–3,88 m | passa |
| contrato de tópicos | `scenario_check.py --seconds 12` → `PASSOU: 0 problema(s)` | passa |
| RTF | **1,000** (sim 25,0 s / wall 25,0 s) | passa |

**Os 2581 pontos são o número que prova que a geometria resolveu**, e não o log.
Referência: no mundo vazio `/demo/scan` dá zero obstáculos; no de objetos, 249;
no corredor, 2422. Um labirinto tem parede em toda volta, e 2581 é coerente com
isso. Malha que não resolve dá uma linha de aviso e uma nuvem vazia — as duas
coisas fáceis de não ver.

**O RTF de 1,000 fecha o risco que estava tabelado**: 284 triângulos de colisão
contra os 156 do `maze10` não custaram tempo real, logo comparações de marcha
feitas aqui seguem válidas. RTF ≠ 1 invalidaria todas elas.

`/demo/cmd_vel` e `/demo/perception/detections` aparecem como AUSENTE e é o
esperado: nada comanda o robô nessa verificação, e `demo_perception` não sobe com
o `nav_quadruped.launch.py`.

## O que este resultado NÃO estabelece

- **Nada em hardware.** Nem AM69, nem robô físico.
- **Nada de desempenho.** Não mede RTF, CPU, latência nem FPS. O RTF do
  `maze11` tem de ser medido em runtime: 284 triângulos contra 156 é mais malha
  de colisão, e RTF ≠ 1 invalida qualquer comparação de marcha.
- **Não estabelece que o robô atravessa o labirinto.** Estabelece que existe
  espaço geométrico contínuo onde ele *caberia* parado. Marcha, costmap,
  planejamento e a deriva lateral do estimador são outra medição — a de S6.
- **Não estabelece que a margem de 21,7 cm é suficiente.** A deriva lateral
  medida nos cenários é de 3,2–6,3% do percurso; sobre 5 m isso é 0,16–0,32 m,
  ou seja **da ordem da margem**. Quem fecha essa malha é o Nav2, e provar isso
  é corrida, não rasterização.
- **A licença do `ros_maze_worlds` continua pendente** (`<license>TODO</license>`,
  sem arquivo LICENSE). A malha segue dependência externa do operador, montada
  em runtime, **nada versionado neste repositório** — a mesma conduta que
  reprovou o CHAMP na F2.
