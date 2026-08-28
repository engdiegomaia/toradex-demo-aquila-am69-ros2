# Guia de implementação — fechamento do ML3.5 F5

**Objetivo:** chegar a uma demonstração HIL repetível em que o Go2 parte sem
mapa salvo, explora o labirinto, reconhece a saída e cruza a abertura em até
600 segundos.

**Estado (28/08/2026, tarde) — o bloqueio de partida foi RESOLVIDO.**

Este guia foi escrito com o portão de navegação reprovado: a corrida de então
percorreu 0,13 m em 90 s, não concluiu metas e terminou com `Failed to make
progress`, porque o `collision_monitor` interrompia comandos ao não conseguir
transformar `/demo/scan_cloud` de `lidar` para `base` no timestamp da amostra.

**Essa causa foi encontrada e corrigida.** Era `/tf` a 1090 Hz: o
`joint_state_broadcaster` herdava os 1000 Hz do `controller_manager` e as doze
juntas das pernas atravessavam a Ethernet para onze assinantes do lado do
Aquila, deixando o `odom_tf` sem escalonamento para carimbar `odom -> base` a
tempo. Decimado para 50 Hz, a disponibilidade de `odom <- lidar` foi de 94,75%
para **99,94%** (`docs/results/ml35-f5-ab-joint-states.md`).

Portões já **APROVADOS**, com evidência versionada em `docs/results/`:

| portão | resultado | evidência |
| --- | --- | --- |
| TF ≥ 99,5% | 99,94% | `ml35-f5-ab-joint-states.md` |
| janela do costmap global | 40 m a 0,10 m, 400 células/eixo | `ml35-f5-tf-cpu-baseline.md` |
| atualização do mapa | `/map` a 1,000 Hz | `ml35-f5-portao-tres-metas.md` |
| **curto de estabilidade** | 3/3 metas nas três corridas, pior 27,9 s de 45 | `ml35-f5-portao-tres-metas.md` |
| marcha | tilt máx 1,18°, folga 0,448 m, zero quedas | idem |

**O que resta**, e é a partir daqui que o guia vale:

1. validar a percepção no Aquila (próximo bloqueio real);
2. smoke da exploração — que é a medição correta da Etapa 4, e a única que diz
   algo sobre o custo de extração de fronteiras;
3. diagnóstico **só** se o smoke falhar;
4. três partidas frias de até 600 s;
5. relatório final e limpeza.

Este guia é sequencial. Não avance para uma etapa enquanto o critério de saída
da etapa anterior estiver vermelho. A regra "não ajuste fronteiras, detector ou
marcador enquanto três metas curtas não funcionarem" está **satisfeita** — as
três metas curtas funcionam.

## 1. Regras da campanha

1. Alterar uma variável por corrida.
2. Reiniciar a pilha entre condições que mudem TF, QoS ou lifecycle.
3. Guardar o CSV principal e o `<csv>-metas.csv` de toda corrida usada numa
   decisão.
4. Não acompanhar logs continuamente. Capturar apenas janelas limitadas após a
   corrida, com `--since`, `--tail` e filtro explícito.
5. Não considerar pico de comando como movimento. O portão usa metas
   concluídas, velocidade média e razão de trabalho.
6. Não aumentar `source_timeout` para aceitar nuvem de vários segundos: isso
   transforma dado velho em falsa segurança. Primeiro corrigir transporte,
   timestamp e TF.

## 2. Etapa A — fechar o caminho temporal do lidar

### 2.1 Sintoma que precisa desaparecer

```text
collision_monitor: Robot to stop due to invalid source
getTransform: Failed to get "lidar"->"base" frame transform:
Lookup would require extrapolation into the future
controller_server: Failed to make progress
```

O `restamp_tf: true` de `slam_toolbox` corrige `map -> odom`, mas não corrige o
timestamp de `/demo/scan_cloud` nem a disponibilidade de `odom -> base` para
esse timestamp.

### 2.2 Instrumentação mínima

Adicionar um script ROS somente de diagnóstico, por exemplo
`scripts/tf_lidar_probe.py`, que amostre por no máximo 60 segundos:

- timestamp atual de `/clock`;
- timestamp de cada `/demo/scan_cloud`;
- idade da nuvem: `clock - cloud.header.stamp`;
- resultado de `can_transform(base, lidar, cloud_stamp)`;
- timestamp mais recente disponível para `odom -> base`;
- taxa e maior intervalo entre nuvens e entre transforms.

O script deve produzir CSV, não uma linha de log por amostra. Saída sugerida:

```text
artifacts/tf-lidar-baseline.csv
```

Critérios de sanidade antes de alterar parâmetros:

- `/demo/scan_cloud` próximo de 9–10 Hz;
- `/demo/odom` próximo de 49–50 Hz;
- idade mediana da nuvem menor que 150 ms;
- percentil 99 menor que o `source_timeout`;
- `can_transform` verdadeiro em pelo menos 99,5% das amostras.

Também medir os publicadores para detectar duplicidade:

```bash
ros2 topic info /demo/scan_cloud --verbose
ros2 topic info /tf --verbose
ros2 topic hz /demo/scan_cloud --window 100
ros2 topic hz /demo/odom --window 200
```

### 2.3 Ordem das correções

Aplicar somente a primeira correção compatível com a evidência e repetir a
sonda antes de tentar a próxima.

#### A. Timestamp antigo já na origem

Se a nuvem chega regularmente, mas já nasce atrasada, corrigir o produtor ou a
bridge em:

- `ros2_ws/src/demo_simulation/config/bridge_quadruped.yaml`;
- launch do simulador/bridge;
- configuração do sensor no xacro vendorizado, sem modificar o pacote
  vendorizado diretamente.

Preservar `qos_profile: SENSOR_DATA`. Não republicar a nuvem inteira por um nó
Python apenas para trocar o header.

#### B. Perdas ou rajadas no transporte HIL

Se a taxa no host estiver saudável e no Aquila houver lacunas:

- confirmar que existe um único assinante remoto por consumidor necessário;
- confirmar MTU, perdas e tráfego da interface `ethernet0`;
- medir a nuvem com Nav2 sozinho e depois com `perception` ativo;
- comparar CPU e rede com o detector habilitado e desabilitado;
- reduzir taxa/resolução do PointCloud na origem somente se a medida provar
  saturação.

Não trocar `SENSOR_DATA` para reliable sem uma corrida A/B: uma nuvem grande e
reliable pode criar backlog, que é pior que descartar uma amostra antiga.

#### C. `odom -> base` chega depois da nuvem

Se a nuvem é nova, mas o transform correspondente chega tarde:

- revisar `demo_bringup/odom_tf` e a QoS de `/demo/odom`;
- confirmar que há um único publicador de `odom -> base`;
- manter `publish_map_identity=false`;
- aumentar `collision_monitor.transform_tolerance` apenas até cobrir o
  percentil 99 medido, com margem pequena e documentada.

`source_timeout` deve continuar representando falha real do sensor. Não usar
quatro ou cinco segundos apenas porque uma captura mostrou backlog desse
tamanho.

### 2.4 Testes exigidos para a correção

- teste estrutural garantindo o QoS e o tópico de `/demo/scan_cloud`;
- teste unitário da sonda para cálculo de idade e percentis;
- `pytest tests/`;
- testes de `demo_navigation` e `demo_simulation` afetados;
- `git diff --check`.

### 2.5 Critério de saída da Etapa A

Durante 180 segundos, sem meta:

- nenhuma parada por `invalid source`;
- nenhuma extrapolação `lidar -> base`;
- taxa e idade dentro dos limites medidos;
- nenhum publicador duplicado de TF.

## 3. Etapa B — tornar o cockpit fail-safe

### 3.1 Status inválido não pode liberar meta manual

Hoje `parseExplorationStatus()` converte mensagem inválida em
`state: 'failed'`, mas `failed` é terminal e libera cliques. Corrigir
`hmi/js/panels/exploration.js` com uma destas estratégias:

1. recomendada: preservar o último status válido; se ele era ocupado, continuar
   ocupado e acrescentar erro de comunicação;
2. alternativamente, criar estado local `status_error`, considerado ocupado
   até chegar um status válido ou o operador cancelar explicitamente.

Adicionar testes para a sequência, não apenas para funções isoladas:

```text
navigating -> JSON inválido -> clique continua bloqueado
navigating -> desconexão -> clique continua bloqueado
cancelled -> JSON inválido -> operador continua com controle
```

### 3.2 Bloquear durante a chamada de start

Ao clicar em “iniciar busca”, aplicar imediatamente um estado local `starting`
antes de chamar `/demo/exploration/start`. `starting` deve:

- esconder/desabilitar o botão de início;
- exibir o botão de cancelamento;
- bloquear `sendGoal()` e o clique no canvas;
- ser substituído pelo primeiro status ROS válido;
- voltar a um estado terminal visível se o serviço falhar.

Adicionar testes com uma Promise de serviço pendente para provar que o clique
manual permanece bloqueado durante toda a chamada.

### 3.3 Critério de saída da Etapa B

```bash
cd hmi
npm test
```

E teste manual no navegador:

- duplo clique em “iniciar busca” envia uma chamada;
- clique no mapa durante `starting` não envia NavigateToPose;
- status inválido mantém o mapa bloqueado se a busca estava ativa;
- cancelar devolve o controle manual.

## 4. Etapa C — validar o novo portão por meta

### 4.1 Preparação

Executar a cadeia oficial depois das correções:

```bash
scripts/module.sh sync
scripts/module.sh build
scripts/module.sh up
scripts/module.sh verify
```

O rebuild pode ser omitido apenas para YAML sob o mount documentado de
`demo_navigation/config`; mudanças em Python, launch, package ou Dockerfile
exigem rebuild.

Antes do gate, confirmar uma vez:

```bash
ros2 param get /slam_toolbox base_frame
ros2 param get /slam_toolbox transform_timeout
ros2 param get /slam_toolbox restamp_tf
ros2 topic echo /map --once --field info
```

Esperado: `base`, `0.2`, `True` e mapa com origem que inclua o robô.

### 4.2 Execução

Rodar com ambiente ROS 2 carregado e DDS do HIL configurado:

```bash
python3 scripts/nav_trial.py artifacts/gate-f5.csv \
  --goals=maze11-short \
  --seconds 180 \
  --goal-timeout 45
```

O comando deve gerar:

```text
artifacts/gate-f5.csv
artifacts/gate-f5-metas.csv
```

Verificar que o sidecar contém exatamente uma linha útil por meta encerrada,
com `outcome`, `status`, `error_code`, `error_msg` e `plan_switches`.

### 4.3 Portão de aprovação

Todos os itens são obrigatórios:

| Critério | Limite |
| --- | --- |
| Metas curtas | 3/3 `SUCCEEDED` |
| Queda | zero |
| `worldToMap failed` | zero |
| `invalid source` no collision monitor | zero |
| Extrapolação TF | zero |
| Velocidade média | pelo menos 0,05 m/s |
| Grandes trocas de plano sem mudança de mapa | zero |
| `error_code` das metas | sucesso/NONE |

Se reprovar, a próxima alteração deve ser escolhida pelo `error_code` e pelas
amostras daquela meta. Não iniciar o explorador para “ver se funciona”.

## 5. Etapa D — validar percepção antes da busca

Com o robô parado e depois apontado para o painel:

```bash
ros2 topic hz /demo/camera/camera_info --window 50
ros2 topic hz /demo/perception/maze_exit/detections --window 50
ros2 topic echo /demo/perception/maze_exit/pose --once
```

Medir no Aquila:

- CPU do `maze_exit_detector`;
- taxa de imagem efetivamente processada;
- impacto na taxa do controller e na idade de `/demo/scan_cloud`.

Se o detector competir com navegação, primeiro aumentar `sample_stride` por
parâmetro. Só substituir a implementação após uma comparação com a mesma cena.

Critério de saída:

- `camera_info` presente;
- detecção confirmada em três de cinco quadros;
- pose publicada no frame da câmera;
- nenhum impacto que faça a Etapa C reprovar novamente.

## 6. Etapa E — corrida autônoma controlada

### 6.1 Smoke de uma partida

Começar com mapa limpo e robô na pose inicial. Iniciar pelo cockpit ou:

```bash
ros2 service call /demo/exploration/start std_srvs/srv/Trigger
```

Não acompanhar tópicos indefinidamente. Usar a HMI e, ao fim, capturar uma vez:

```bash
ros2 topic echo /demo/exploration/status --once
ros2 topic echo /demo/maze/escaped --once
```

Abortar com `/demo/exploration/cancel` se houver risco de queda, colisão ou
saída dos limites físicos.

### 6.2 Três partidas frias

Somente após o smoke completar:

1. reiniciar simulação, Nav2, SLAM e percepção;
2. garantir ausência de mapa/pose-graph salvo;
3. iniciar busca uma única vez;
4. encerrar ao receber `/demo/maze/escaped=true` ou aos 600 s;
5. salvar evidências antes da próxima partida.

Registrar por corrida:

- tempo até a saída;
- percurso total;
- fronteiras avaliadas e colocadas em blacklist;
- primeira detecção do marcador;
- metas Nav2 por desfecho e `error_code`;
- cobertura final do mapa;
- razão de trabalho de `cmd_vx`;
- quedas, recuperações e grandes trocas de plano;
- contagem dos erros proibidos.

Aceitação final: três de três partidas com `/demo/maze/escaped=true` em até 600
segundos, sem queda, sem intervenção manual e sem conhecimento de coordenadas
do labirinto no explorador.

## 7. Atualização obrigatória da documentação

Após cada gate usado para decisão:

1. mover os CSVs aceitos para `docs/results/` ou documentar por que permanecem
   em `artifacts/`;
2. atualizar `docs/results/ml35-f5-busca-autonoma.md`;
3. substituir “Aceitação — NÃO EXECUTADA” pelo estado realmente medido;
4. registrar comando, commit, configuração, duração e resultado;
5. não declarar hardware validado com base em testes de host.

## 8. Checklist de conclusão

- [ ] caminho temporal do lidar saudável por 180 s;
- [ ] HMI bloqueia metas em `starting` e em falha de status durante busca;
- [ ] `gate-f5-metas.csv` contém três sucessos;
- [ ] portão curto atinge velocidade média mínima;
- [ ] detector vê painel e publica pose no Aquila;
- [ ] busca autônoma completa uma partida smoke;
- [ ] três partidas frias completam em até 600 s;
- [ ] documentação e evidências versionadas;
- [ ] `git status` limpo, exceto artefatos deliberadamente não versionados.
