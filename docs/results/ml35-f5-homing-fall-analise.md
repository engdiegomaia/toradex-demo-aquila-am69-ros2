# Investigação da queda em homing cego perto da abertura de saída

Investigação de código apenas (nenhuma rodada nova em hardware) sobre a queda
registrada em 30/08 durante a validação posicionada da AprilTag
(`docs/results/ml35-f5-apriltag-positioned.md`, memória
`homing-cego-derruba-quadrupede.md`): o robô tombou em `(-5.057, -1.255)`
enquanto `_send_homing_step` operava em `blind=True`, perto do limite
`BOUNDARY_Y - ROBOT_RADIUS = -1.28`. Nunca investigada até agora. Requisito
do plano: investigar antes de qualquer corrida completa que cruze a saída.

## O que o código mostra (verificado, não hipótese)

`_send_navigation` (`maze_explorer.py`) só define
`goal.behavior_tree` quando `exploration=True`:

```python
if exploration:
    goal.behavior_tree = str(self.get_parameter('exploration_bt_xml').value)
```

Para homing (`exploration=False`, todo `_send_homing_step`), o campo fica
vazio e o `bt_navigator` cai no seu `default_nav_to_pose_bt_xml` --
`nav_to_pose_smoothed.xml` neste projeto. Essa árvore usa
`PlannerSelector default_planner="GridBased"`, e `GridBased`
(`planner_server` em `nav2_params_go2.yaml`) roda com **`allow_unknown: true`**.

A exploração usa a outra árvore, `nav_to_pose_exploration.xml`, cujo
`ComputePathToPose` fixa `planner_id="ExplorationGrid"` -- e
`ExplorationGrid` roda com **`allow_unknown: false`**, deliberadamente (o
comentário no YAML: "fronteiras... sempre... espaço observado... previne
uma meta valida de tomar atalho por celulas que a SLAM ao vivo ainda nao
observou").

**Ou seja: a mesma classe de atalho por espaço desconhecido que a
exploração recusa de propósito é exatamente o que o homing cego permite.**
Isso não é um detalhe qualquer para este caso: a abertura de saída, por
definição, fica na borda do que o SLAM já mapeou -- é geometria de fronteira
por natureza, o tipo de lugar mais provável de ter célula `UNKNOWN` por
perto.

Segundo ponto, também verificado no código: `_send_homing_step` com
`blind=True` NÃO limita o passo a `homing_step_m` (0.5 m) como faz o passo
com marcador fresco:

```python
step = distance - stop if blind else min(homing_step_m, distance - stop)
```

Isso é deliberado (comentário: uma sequência de passos curtos as cegas só
dá ao planejador paredes para recusar) -- mas o efeito colateral é que o
único NavigateToPose despachado as cegas pode cobrir uma distância bem
maior que qualquer meta de exploração comum, sobre um planejador que aceita
atravessar `UNKNOWN`, na região exata onde o mapa é mais incompleto.

## Hipótese (NÃO confirmada em hardware)

A combinação -- planejador que aceita `UNKNOWN` + passo as cegas sem teto de
distância + geometria de fronteira real (a abertura) -- é consistente com o
que foi observado: o robô pode ter recebido um caminho que atravessa uma
região sem obstáculo conhecido no costmap (porque nunca foi observada),
tentando manter a marcha reta a `vx_max` por um trecho mais longo que o
costmap local realmente valida, e encontrado geometria física real (a borda
da abertura) que o costmap não continha -- ou simplesmente perdido
estabilidade numa curva mais brusca que o planejador computou sobre espaço
desconhecido.

Isto explicaria por que este é um modo de queda DIFERENTE do já documentado
(quedas em `/demo/sim/reset` sem parar a marcha primeiro): aqui a marcha
nunca parou, o controlador estava ativo o tempo todo, e a queda aconteceu
durante navegação autônoma normal.

**O que isto NÃO é:** uma conclusão sobre controlador, terreno ou timing —
nenhuma dessas três foi isolada nem medida. É uma hipótese de configuração,
com uma causa concreta e testável (o planejador do homing cego difere do da
exploração exatamente no parâmetro que decide se atravessar espaço não
mapeado). Confirmar exigiria reproduzir a queda com telemetria de
`/plan`/`/local_costmap/costmap` no momento exato, o que este round de
investigação (sem HIL novo) não fez.

## Mitigação candidata (não aplicada nesta rodada)

Fazer o homing cego usar a MESMA árvore/planejador que a exploração
(`ExplorationGrid`, `allow_unknown: false`) removeria a diferença
identificada acima. Isso é uma mudança de comportamento real e deveria
seguir a mesma disciplina de uma variável por vez das rodadas R14/R15 --
não aplicada aqui porque este item era "investigar", não "corrigir". Se
confirmada como a causa, a mudança mínima seria:

```python
goal.behavior_tree = str(self.get_parameter('exploration_bt_xml').value)
```
movida para fora do `if exploration:` em `_send_navigation`, aplicando-se
também ao homing -- ou, alternativa mais conservadora, um
`homing_bt_xml` próprio que reuse `ExplorationGrid` sem herdar nenhum outro
comportamento da árvore de exploração.

## Como isto muda o plano de aceitação

Nenhuma corrida de aceitação (R16 em diante) deve deixar o robô entrar em
`homing_exit` sem supervisão até esta hipótese ser testada ou a mitigação
aplicada e validada -- especialmente em `blind=True`, que é exatamente o
modo em que a queda registrada aconteceu. R16, como planejado, para de
explorar antes da região do marcador (captura o mapa em `x<-4.5, y<2.0`
antes disso); nenhuma mudança de plano é necessária para R16 especificamente
por causa deste achado, mas o cruzamento efetivo da saída (item já listado
como pendente) não deve ser tentado sem decidir sobre esta mitigação
primeiro.
