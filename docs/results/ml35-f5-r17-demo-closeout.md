# R17 — revisão e fechamento da demo supervisionada

Data: 30/08/2026. Cenário de apresentação: `quadruped_maze11.sdf`.

## Resultado

O estado R17 está fechado como **demo supervisionada de navegação e exploração
no labirinto**, não como demonstração autônoma de fuga. O último estado com
evidência HIL integrada continua sendo R16 (`ml35-f5-r16-demo-stable`). R17 é a
revisão de código a ser implantada e validada na bancada antes da apresentação.

## Correções da revisão

- Os breadcrumbs passaram a guardar a pose de partida do segmento concluído.
  Antes, a primeira meta de retorno apontava para a pose atual e consumia uma
  tentativa sem recuar.
- `params-align8.yaml` e `nav2_params_go2_footprint.yaml` agora acompanham
  `consider_footprint: true` do perfil padrão. Os testes de igualdade detectaram
  a divergência antes do fechamento.
- O guia rápido ganhou Markdown executável, links nos índices, configuração
  local explícita, cenário HIL correto e remoção de endereço IP da bancada.
- Três pendências pequenas de lint nos pacotes próprios foram corrigidas.

## Validação executada

| verificação | resultado |
| --- | --- |
| contratos da raiz (`pytest -q tests`) | 292 aprovados |
| pacotes ROS próprios via `colcon test` | 316 aprovados |
| HMI (`npm test`) | 12 aprovados |
| Compose host e módulo (`config -q`) | aprovados |
| `compileall` e `git diff --check` | aprovados |
| build da imagem `local/demo-aquila-nav:dev` no host amd64 | aprovado |

Os 316 testes ROS são: `demo_bringup` 64, `demo_description` 13,
`demo_navigation` 114, `demo_perception` 52, `demo_simulation` 70 e
`demo_tutorials` 3. Os avisos de `fork()` emitidos pelo linter são do executor
multithread do teste e não representam falha.

## Cenário e odometria observados

O smoke de host foi iniciado com `ROBOT_TYPE=quadruped`, `SIM_GUI=false` e sem
`SIM_ARGS`. Pela tabela autoritativa em `robot_selection.py`, essa combinação
resolve obrigatoriamente para `quadruped_maze11.sdf`; `warehouse.sdf` é somente
o fallback `diffdrive`.

Uma leitura pontual no spawn mostrou um único publicador de `/demo/odom`, o
`ros_gz_bridge`, com `frame_id=odom`, `child_frame_id=base`, posição aproximada
`(-0,003; 0,042; 0,347)` m e orientação próxima de 90 graus. Isso é compatível
com a pose inicial declarada do labirinto. O contrato estrutural também garante
que `odom_tf` copia o timestamp da própria odometria para `odom → base`, em vez
de recarimbar a aresta com outro relógio.

Não foi iniciada uma trajetória R17. A leitura do explorador na Aquila revelou
estado antigo após reinício do relógio do simulador (tempo decorrido negativo),
logo cruzar esse estado com uma nova corrida produziria evidência inválida. A
sincronização remota usa `rsync --delete` e foi corretamente bloqueada sem
autorização explícita; Nav2/percepção também não foram recriados por atalho.

## Preflight obrigatório antes da apresentação

Com o robô no spawn e autorização para substituir a árvore remota:

```bash
scripts/module.sh sync
scripts/module.sh build
scripts/module.sh up
scripts/module.sh verify
```

Depois confirme produtor único e coerência inicial:

```bash
source scripts/env.sh
ros2 topic info /demo/cmd_vel -v
ros2 topic info /demo/odom -v
ros2 topic echo /demo/odom --once
```

Só aceite a rodada se houver um publicador de comando, um publicador de
odometria, o robô estiver próximo de `(0, 0)` e o explorador estiver em `idle`
com tempo não negativo. Durante a apresentação, monitore a telemetria do
cockpit e cancele ao entrar em `homing_exit` ou diante de risco de queda.

## Limitações que permanecem

- `/demo/maze/escaped` nunca atingiu `true` em uma rodada aceita.
- O homing cego mantém risco de queda documentado e não mitigado.
- R17 e `consider_footprint: true` ainda não têm rodada HIL na Aquila.
- Não há três partidas frias bem-sucedidas.
