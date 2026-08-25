# Cockpit web — ajustes de UI de bancada (25/08/2026)

**Onde foi medido:** estação x86 (amd64), modo `learn`, tudo no host.
**Onde NÃO foi medido:** Aquila AM69. Nada neste documento é validação de
hardware (regra 7 do `CLAUDE.md`). Os tempos e as poses abaixo valem para o host
e só para ele.

Três pedidos do operador, fora da numeração de fases do
`plano-cockpit-web.md` — não abrem fase nem fecham portão:

1. marca Toradex ao menos ao dobro;
2. reiniciar a navegação a partir do cockpit;
3. vista de cena acompanhando o robô, nas duas câmeras.

---

## 1. Marca ao dobro

| | antes | depois |
|---|---|---|
| marca Toradex | 34 px | **68 px** |
| marca ROS | 22 px | **44 px** |
| altura mínima da faixa | 48 px | **80 px** |

As três medidas saem de tokens em `hmi/css/tokens.css` (`--logo-toradex`,
`--logo-ros`, `--bar-min-height`), e a última é consumida **também** por
`layout.css`, na linha da grade que reserva a barra. Isso não é organização: a
`.bar` tem `overflow-x`, não `-y`, então uma marca maior que a faixa é cortada
**sem sintoma nenhum** — nada no console, nada no layout, só a marca com o topo
faltando. Acoplar as duas ao mesmo número torna essa divergência impossível.

Verificado no que o nginx entrega, não no arquivo fonte:

```
$ curl -s localhost:8081/css/tokens.css | grep logo-toradex
  --logo-toradex: 68px;
$ curl -s -o /dev/null -w '%{http_code}\n' localhost:8081/
200
```

**Não há captura de tela.** O Chrome não está instalado nesta máquina (o MCP de
devtools não encontrou binário) e o Firefox snap em headless não respondeu — nem
com o perfil existente ("already running, but is not responding") nem com perfil
novo. O que existe é a verificação do CSS servido, acima. A conferência visual
fica pendente do operador, junto da do kiosk Chromium no F2.

---

## 2. Reiniciar a navegação

`/demo/nav/reset` (`std_srvs/Trigger`), servido pelo `nav_control_relay`, no
container do Nav2.

```
$ ros2 service call /demo/nav/reset std_srvs/srv/Trigger
response:
  success: True
  message: 'navegacao reiniciada: meta descartada, costmaps limpos, servidores reativados'
```

- **duração:** 6,638 s
- **meta em andamento:** terminou `CANCELED` (não `ABORTED`, não pendurada)
- **servidores depois:** todos `active [3]`
- **meta nova em seguida:** aceita imediatamente

### O achado que mudou o desenho: `RESET`+`STARTUP` mata o container

A primeira implementação usava as transições que descrevem exatamente o pedido —
`RESET` (desativa e desconfigura tudo) e `STARTUP` (configura e ativa tudo) no
`lifecycle_manager_navigation`. Resultado medido:

```
[component_container_isolated-4] [INFO] [route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [exit code -11]
```

`-11` é `SIGSEGV`. **Reproduzido duas vezes**: uma com meta ativa, uma sem. Este
guia registrava antes um segfault "que aconteceu uma vez" no `route_server` — não
é intermitente, é determinístico, e está no caminho do `CONFIGURE`.

O `route_server` está na lista de `lifecycle_nodes` do `navigation_launch.py`
vendorizado, que tem de seguir idêntico ao upstream (base do argumento de
proveniência), então retirá-lo não é opção. A demo não usa roteamento e ele não
tem seção em `nav2_params_go2.yaml`. Hipótese **não confirmada** no fonte do
Nav2: reconfiguração sobre estado que não sobrevive ao `CLEANUP`. Candidato a
issue upstream.

Sequência adotada, que nunca passa por `CONFIGURE`:

1. `CancelGoal` em `/navigate_to_pose` com `goal_info` zerado — cancela todas as
   metas, inclusive as que o cockpit não sabe que existem;
2. `ClearEntireCostmap` no global e no local, **com os servidores ainda ativos**
   — nó desativado não responde serviço, e limpar depois de pausar não limparia
   nada nem daria erro;
3. `PAUSE`;
4. `RESUME` — tentado mesmo se o `PAUSE` falhar, porque a alternativa é deixar a
   pilha desativada.

O relógio do `_wait()` é `time.monotonic()`, e não o do nó: um Nav2 desativado
coexiste com um `/clock` parado, e um timeout medido em tempo simulado nessa
janela nunca expira.

**A localização não é tocada.** `lifecycle_manager_localization` fica fora de
propósito: no caminho de mapa estático, reciclar o AMCL joga a pose fora, e o
robô "se perde" num reset que era só para descartar a meta.

---

## 3. Seguir o robô

10 Hz, `/demo/odom`, as duas câmeras. O que se move é **só o alvo** da órbita —
azimute, elevação e distância ficam como estavam, então o enquadramento medido do
armazém continua valendo com o robô andando.

Medido com o robô em `(-1,598 ; 0,130)`:

| câmera | pose | esperado |
|---|---|---|
| `scene_top` | `(-1,552 ; 0,151 ; 6,0)` | sobre o robô, `z` preservado |
| `scene_iso` | `(-4,551 ; 3,150 ; 2,4)` | robô + `(-3, +3, 2,4)`, o deslocamento medido |

Erro de acompanhamento na topo: ~5 cm — a banda morta de 2 cm mais o que o robô
andou entre a amostra e a escrita. O `z` das duas está intacto, que era o risco
(uma vista de topo que sobe entra nas vigas do telhado do armazém; ver a
armadilha 7 do guia).

- desligar pelo botão: pose **congelada** ao longo de 6 s de caminhada;
- religar: recentrou.

### O seed `odom -> mundo`

O `set_pose` do Gazebo só entende o referencial do **mundo**, e as duas plantas
divergem no que publicam em `/demo/odom`:

| planta | origem da odometria | seed |
|---|---|---|
| quadrúpede | ground truth do `gz-sim-odometry-publisher-system` | **0** — já é pose no mundo |
| diff-drive | integrada dos encoders pelo plugin `DiffDrive`, origem na pose de spawn | **pose de spawn** |

Por isso `simulation.launch.py` passa `follow_offset_{x,y,yaw}` explicitamente e
`quadruped.launch.py` não passa nada. Os três **não** são lidos de `x`/`y`/`yaw`
dentro de `scene_cameras.launch.py`: as duas plantas declaram esses nomes, o
include os herdaria das duas, e a câmera do quadrúpede passaria a seguir um
fantasma deslocado pela pose de spawn — no `maze11`, deslocado **e** girado 90°.

### Pan com seguimento ligado

Os botões de mover continuam válidos: com seguimento, o pan é um deslocamento
**relativo ao robô**, saturado em 15 m. Serve para olhar de lado ou um pouco à
frente sem perder o acompanhamento; `recentrar` zera esse deslocamento.

### Estado publicado, não ecoado

`/demo/cockpit/scene/following` (`std_msgs/Bool`, latched), publicado pelo nó:

```
$ ros2 topic echo /demo/cockpit/scene/following --once
data: true
```

O botão pinta o que o tópico diz, nunca o que o clique pediu. Mesmo raciocínio do
rótulo de simulação, e pelos mesmos casos em que o clique local não sabe a
resposta: F5, segundo cockpit aberto, e alguém que desligou por linha de comando.

---

## Testes

```
tests/                              36 passed   (era 29)
hmi/  node --test                  138 passed   (era 131)
em container: demo_simulation       32 passed
              demo_navigation        2 passed
              demo_bringup          46 passed
```

Novos: `demo_simulation/test/test_scene_view_controller.py` (9 casos sobre a
órbita — ida e volta da pose de launch, alvo degenerado da vista de topo, pan
sobrevivendo ao tique seguinte, saturação do deslocamento, limite de elevação) e
seis guardas estruturais em `tests/test_cockpit_web_contract.py`, um deles
justamente para impedir que alguém "simplifique" o reset de volta para
`RESET`+`STARTUP`.

Ao escrever esses guardas, um deles casou com a **própria documentação** do
relay: `'lifecycle_manager_localization' not in code` batia no docstring que
explica por que a localização fica de fora. Os guardas agora leem o código com
`ast`, com comentários e docstrings removidos. Regex não serviria: uma tripla
quota dentro de f-string quebra a varredura em silêncio.

Aproveitado o passo: as três falhas de lint pré-existentes em `demo_simulation`
(I100, E731, E501) foram corrigidas, para que a suíte onde entraram testes novos
esteja verde em vez de já vermelha.

---

## O que fica pendente

- **conferência visual em navegador** — nenhuma das três mudanças foi vista numa
  tela. Vale para a marca em especial, que é puramente visual.
- **Aquila AM69** — nada rodou lá. O `nav_control_relay` é o único dos três que
  vai executar **no módulo** no modo `hil`, e o tempo de 6,6 s é do host; num
  Cortex-A72 com `/clock` a ~880 Hz não há razão para supor o mesmo número.
- **issue upstream** do segfault do `route_server`, se alguém quiser levá-la.
