# ML3.5 — plano dos próximos passos

Escrito em 18/08/2026, depois de o Go2 passar a caminhar e parar de forma
repetível. Evidência do estado atual em `docs/results/ml35-f4-parcial.md`;
comandos de operação em `docs/guides/go2-testes.md`.

Este documento cobre mais de uma fase de propósito, para dar visão do caminho.
A execução continua respeitando `estado-fases.md`: **cada fase para no portão e
espera**, e `CLAUDE.md` manda implementar um milestone por sessão.

---

## Onde o portão F4 realmente está

A tabela de critérios em `go2-proximos-passos.md` §11 ficou defasada em duas
linhas com o trabalho de 18/08:

| Critério | Tabela dizia | Realidade medida |
|---|---|---|
| não deriva em guinada parado em `HOLD` | ❌ ~3°/s arrastando os pés | **corrigido** pelo travamento do alvo de apoio no toque; HOLD corrige rumo em 4 de 5 ciclos |
| anda em `linear.x=0.01` / `0.03` | ⚠️ inválido | precisa ser **reescrito** em termos de `v_cmd`, não apenas marcado como inválido |

Restam quatro critérios nunca ensaiados: guinada comandada, warehouse,
RViz2/TF e regressão do diff-drive.

---

## Padrões a espelhar

Levantados do próprio repositório. Onde não há precedente, está dito.

| Categoria | Fonte | Padrão |
|---|---|---|
| Launch | `demo_bringup/launch/{sim,nav,perception,viz}.launch.py` | um launch explícito por papel de container, sem condicionais |
| Parâmetros | `demo_navigation/config/nav2_params.yaml` | YAML, nunca embutido em código |
| Testes | `demo_simulation/test/test_twist_to_inputs.py` | lógica pura extraída sem ROS (`_CommandGate` recebe `now`), pytest + ament flake8/pep257 |
| Diagnóstico | `StateTrotting::logDiagnostics()` | uma linha periódica com todos os campos que separam os modos de falha |
| Evidência | `docs/results/ml35-f4-parcial.md` | comando, duração, pose inicial, pose final, e os ensaios **rejeitados** com o porquê |

**Sem precedente no repositório:** mundo warehouse e publicação de TF pelo
quadrúpede. São caminhos novos, não há padrão a copiar.

---

## Fases

### Fase 1 — Sincronizar o portão F4 com a realidade

**Complexidade: baixa.**

O latch de toque mudou o comportamento de parada, então os critérios já verdes
precisam ser reexecutados com o código atual, não herdados. Reescrever os dois
critérios inválidos em termos de `v_cmd`:

| Novo critério | Origem do número |
|---|---|
| anda a `v_cmd = 0,05 m/s` | mínimo utilizável; abaixo disso o passo fica milimétrico sob elevação de pé de 8 cm |
| anda a `v_cmd = 0,10 m/s` | ponto de operação validado em 18/08 |
| anda a `v_cmd = 0,20 m/s` | teto da ponte (`_SAFE_STICK_LIMIT = 0.5`) |

**Valida:** roteiro §7.2 de `go2-testes.md`, 5 ciclos, zero quedas, mais uma
varredura das três velocidades.

### Fase 2 — Guinada comandada

**Complexidade: média.**

Último critério de comportamento de movimento e **pré-requisito do Nav2**: o DWB
comanda `angular.z` continuamente, então uma guinada comandada que não rastreia
inviabiliza F5 antes de começar.

Ensaiar `angular.z` a 0,1 / 0,3 / 0,5 (→ 0,05 / 0,15 / 0,25 rad/s), parado e
combinado com `linear.x`. Riscos já medidos: o QP satura em ~5,3 N·m de momento
de guinada, e `d_yaw_cmd_` passa por filtro de 0,9. Se não rastrear, a correção
é em `FeetEndCalc` — **não** em ganho de atitude, que já foi medido como
incapaz de regular rumo neste robô.

**Valida:** `Mz` pedido ≈ realizado, sem `RECOVER`, taxa de guinada medida em
`/demo/odom` contra a comandada.

### Fase 3 — RViz2 e TF

**Complexidade: média, com incerteza alta.**

As 12 juntas visíveis e a árvore TF consistente. **Aqui pode estar o trabalho
escondido do plano:** é preciso confirmar se o `unitree_guide_controller`
publica `odom → base_link`. Se não publicar, o Nav2 não fecha e este vira o item
mais pesado.

RViz2 roda **só no host** (`CLAUDE.md`, regra 1).

**Valida:** `ros2 run tf2_tools view_frames`, árvore sem galho solto.

### Fase 4 — Cenário warehouse

**Complexidade: média.**

Hoje só existe `quadruped_empty.sdf`. Adicionar o warehouse com câmera e lidar
ativos, preservando o contrato de tópicos.

**Valida:** `ros2 topic hz` em `/demo/scan` e `/demo/camera/image_raw` no mundo
novo, `demo_perception` publicando `/demo/perception/detections` sem alteração,
e RTF medido antes e depois.

### Fase 5 — Regressão do diff-drive

**Complexidade: baixa.**

Garantir que o trabalho no quadrúpede não quebrou a demo que já funcionava: goal
Nav2 `SUCCEEDED` com diff-drive.

### Fase 6 — F5: Nav2 com o quadrúpede

**Complexidade: alta.** Depende de todas as anteriores, e o tamanho real é
definido pelo que a Fase 3 revelar.

---

## Riscos

| Risco | Probabilidade | Mitigação |
|---|---|---|
| **`/demo/odom` é ground truth do Gazebo.** Para o Nav2 isso é trapaça: em hardware real a odometria sai do estimador via TF. F5 pode passar em simulação e falhar no módulo | **Alta** | tratar na Fase 3; decidir se `/demo/odom` passa a vir do controlador **antes** de declarar F5 |
| **Limites de velocidade do Nav2 não batem com o envelope do robô** (0,2 m/s, 0,25 rad/s). O DWB comanda mais rápido, a ponte satura, o Nav2 conclui que está preso | **Alta** | reconciliar `nav2_params.yaml` com a tabela de conversão de `go2-testes.md` §5 antes do primeiro goal |
| **Rumo em malha aberta faz passeio aleatório** (12° a 43° entre execuções). Se o ganho de guinada do Nav2 for insuficiente, o robô erra o goal | Média | a Fase 2 mede a autoridade real de guinada e alimenta a sintonia do DWB |
| Viés de 24 mm no `z` estimado (`foot_radius = 0.02` contra `feet_h_ = 0` no estimador) | Baixa em simulação, **alta em hardware** | rastreado; não bloqueia F4/F5 em simulação |
| Warehouse derruba o FPS e muda a dinâmica de contato | Média | medir RTF antes e depois; RTF ≠ 1 invalida qualquer comparação de marcha |

---

## Ordem recomendada

**Fases 1 → 2 → 3, depois parar e reavaliar.** A Fase 3 decide se F5 é um dia
de trabalho ou uma semana, e as Fases 4 e 5 são independentes dela — esperam sem
custo.
