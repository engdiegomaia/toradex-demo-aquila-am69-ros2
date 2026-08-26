# Guia do cockpit web

Como rodar, o que cada região da tela mostra, o que os botões fazem, e o que
não é possível fazer por eles.

Este documento é **operacional**. A evidência de bancada (medições, capturas,
falhas encontradas) está em [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md)
e [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md); as decisões de
projeto e as fases em [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md).

> **Nada aqui foi executado no Aquila AM69.** Tudo abaixo é o modo `learn`, com
> todos os containers na workstation x86. O kiosk no módulo é o F2 do plano e
> continua em aberto.

---

## Sumário

- [1. O que é o cockpit](#1-o-que-é-o-cockpit)
- [2. Rodar](#2-rodar)
- [3. A tela](#3-a-tela)
- [4. Os controles](#4-os-controles)
- [5. O que NÃO dá para fazer](#5-o-que-não-dá-para-fazer)
- [6. Trocar de cenário](#6-trocar-de-cenário)
- [7. Ajustar qualidade de imagem](#7-ajustar-qualidade-de-imagem)
- [8. Diagnóstico](#8-diagnóstico)
- [9. As armadilhas que já custaram tempo](#9-as-armadilhas-que-já-custaram-tempo)
- [10. Como o cockpit é feito](#10-como-o-cockpit-é-feito)
- [11. O que está feito e o que falta](#11-o-que-está-feito-e-o-que-falta)

---

## 1. O que é o cockpit

Uma página web que mostra o robô e deixa comandá-lo, sem RViz e sem a janela do
Gazebo. Ela substituiu quatro tentativas de **incorporar janelas X11** numa
aplicação única — todas falharam, e o checkpoint delas
([`results/cockpit-standalone-parcial.md`](results/cockpit-standalone-parcial.md))
está marcado como superado. **Não retome aquele caminho.**

O eixo agora é: renderizar **a partir de tópicos ROS 2**. Isso muda o que é
possível. RViz e Gazebo são OGRE 2 e precisam de OpenGL de desktop, então nunca
poderiam ir para o Aquila (regra 1 do `CLAUDE.md` — o AM69 só expõe OpenGL ES
3.2 e Vulkan 1.2). Um navegador desenhando `nav_msgs/OccupancyGrid` num
`<canvas>`, não. É por isso que **este cockpit vira o HMI do módulo no M3**, e a
tentativa anterior não viraria.

Duas peças:

| Serviço | O que é | Onde roda hoje | Onde roda no M3 |
| --- | --- | --- | --- |
| `cockpit` | `rosbridge_server` + `web_video_server` | host | Aquila |
| `hmi` | nginx servindo o bundle de `hmi/` | host | Aquila |

O bundle é HTML/CSS/ES modules **sem etapa de build** — sem npm, sem bundler,
sem `node_modules`. Editar um arquivo e recarregar a página é o ciclo inteiro.
Isso é decisão de projeto (Decisão 5 do plano): a cadeia npm é justamente o que
não deve viajar para o módulo.

---

## 2. Rodar

### Pré-requisitos

Uma vez por máquina:

```bash
cd docker
cp .env.example .env      # e edite MAZE_MODELS, se for usar o labirinto
```

### Subir

```bash
cd docker
docker compose -f compose.host.yml --profile learn up -d --build
```

Isso sobe cinco serviços: `sim` (Gazebo + robô + pontes), `nav` (Nav2),
`perception` (stub de detecção), `cockpit` (rosbridge + vídeo) e `hmi` (nginx).

**Abra <http://localhost:8081>.**

O Nav2 leva ~30 s para ficar pronto. A página pode ser aberta antes: os painéis
sobem vazios e se preenchem sozinhos, e o ponto de frescor de cada um diz se
aquele tópico já chegou. É de propósito — um cockpit que trava até tudo estar
pronto não mostra o que está faltando.

### Portas

Todos os containers usam `network_mode: host`, então **nada aqui é mapeamento de
porta**: os números abaixo abrem direto na workstation. Se colidirem com algo
que você já roda (8080 é popular), troque em `docker/.env`.

| Variável | Padrão | Serve |
| --- | --- | --- |
| `COCKPIT_HMI_PORT` | 8081 | a página — **é esta que você abre** |
| `COCKPIT_ROSBRIDGE_PORT` | 9090 | WebSocket que o navegador consome |
| `COCKPIT_VIDEO_PORT` | 8080 | MJPEG das câmeras |

### Derrubar

```bash
docker compose -f compose.host.yml --profile learn down
```

### Editar o bundle

Mudança em `hmi/` (HTML, CSS, JS) não precisa de `colcon`, mas precisa de
rebuild da imagem, porque o nginx serve uma cópia:

```bash
cd <raiz do repo>
DOCKER_BUILDKIT=0 docker build -f docker/hmi/Dockerfile -t local/demo-aquila-hmi:dev .
cd docker && docker compose -f compose.host.yml up -d --no-build --force-recreate hmi
```

Antes de recarregar, rode os testes do bundle — eles não precisam de ROS nem de
container:

```bash
cd hmi && node --test "test/**/*.test.js"
```

As **aspas no glob são obrigatórias**: sem elas o shell expande, o `node --test`
recebe um diretório, resolve para nada e sai com código 0 — verde sem ter
rodado teste nenhum.

---

## 3. A tela

Cinco regiões, na disposição de [`ml35/cockpit-division-view.png`](ml35/cockpit-division-view.png):

```
+---------------------------+--------+
|          CENA             |  NAV   |
+---------------------------+--------+
|          LOGS             | CÂMERA |
+---------------------------+--------+
|          BARRA DE CONTROLE         |
+------------------------------------+
```

| Região | Fonte | Responde a que pergunta |
| --- | --- | --- |
| **Cena** | `/demo/cockpit/scene_{iso,top}/image_raw` | o robô se mexeu? |
| **Navegação** | `/global_costmap/costmap`, `/plan`, `/demo/scan`, TF | ele sabe para onde ir? |
| **Logs** | `/rosout` + telemetria de `/demo/cmd_vel` e `/demo/odom` | o que ele está tentando fazer? |
| **Câmera** | `/demo/camera/image_raw` | o que ele está vendo? |
| **Barra** | estado do WebSocket | o cockpit ainda está falando com o robô? |

### O painel de Cena é o mais importante e é o menos óbvio

São **duas câmeras estáticas do mundo** — não do robô — alternadas pelos botões
`iso` / `topo` no cabeçalho. Elas existem porque respondem "o robô se mexeu?"
sem depender de nenhum nó do stack: se o Nav2, a TF e a odometria mentirem
juntos, esta imagem continua honesta. É a testemunha independente da tela.

### O ponto de frescor

Cada painel tem uma bolinha no canto do cabeçalho, com três estados que **não
são a mesma coisa**:

| Cor | Estado | Significa |
| --- | --- | --- |
| cinza | `never` | esse tópico nunca produziu nada desde que a página abriu |
| verde | `live` | chegou amostra dentro da janela daquele painel |
| laranja | `stale` | chegou antes e **parou** |

`stale` é o que pega um simulador morto, e por isso não pode parecer `never`.

### Cores

A paleta é a identidade Toradex: fundo branco, azul `#00508c`, verde `#96c837`,
laranja `#ff5a00`. O **vermelho de falha (`#c0261b`) não é da marca, de
propósito**: "parado há tempo demais" e "morto" precisam parecer coisas
diferentes através da sala, e usar laranja nos dois faria o pior caso
desaparecer dentro do caso comum.

No mapa, a mesma lógica: azul é o plano global, verde é o robô, laranja é
obstáculo, e a meta é violeta — uma cor que **não** é da marca, justamente para
não poder ser confundida com um estado.

Cor de canvas não existe em JavaScript. `hmi/js/panels/palette.js` lê os tokens
`--map-*` de `hmi/css/tokens.css` uma vez, na montagem. Mudar tema é mexer num
arquivo.

---

## 4. Os controles

### Clicar no mapa manda meta

Clique no painel verde. Vai um `NavigateToPose` para o Nav2, e o HUD no rodapé
do painel passa a mostrar o estado (`navegando · 5,04 m restantes`), incluindo a
contagem de recuperações. Um botão **cancelar meta** aparece no cabeçalho
enquanto há meta ativa.

Clique perto demais do robô (< 0,25 m) é tratado como engano e ignorado.

### Simulação: ▶ ⏸ ⟲

Na barra. Chamam `/demo/sim/{play,pause,reset}` (`std_srvs/Trigger`).

O **reset exige dois cliques**: o primeiro arma o botão (ele vira `confirmar` em
laranja por 4 s), o segundo executa. Ele devolve o robô à pose inicial. **Não**
apaga o costmap — quem limpa o costmap é [reiniciar a navegação](#reiniciar-a-navegação),
botão separado, de propósito: repor o robô sem derrubar o que o Nav2 já sabe do
mundo é o que permite os dois em sequência sem perder trabalho.

No cenário quadrúpede o clique leva alguns segundos a mais do que no
diff-drive, e isso é esperado: o robô é **parado** antes de ser reposicionado e
só volta a andar depois — ver a [armadilha 9](#9-o-reset-derrubava-o-quadrúpede-em-marcha).

O rótulo ao lado (`rodando` / `pausado` / `sem simulador`) **não é o eco do
último clique**: ele vem do `/clock`. Um eco mentiria em todos os casos que
importam — container `sim` morto, chamada expirada, pausa feita pela GUI do
Gazebo, mundo resetado por outra pessoa. Se o botão diz uma coisa e o rótulo
diz outra, **o rótulo está certo**.

A diferença entre `pausado` e `sem simulador` é a que custa: amostras chegando
com o mesmo tempo simulado é pausa; amostras **parando** é ausência.

### Câmera de cena: o pad sobre a imagem

Canto inferior direito do painel azul. Três fileiras:

```
 ↺  ▲  ▼  ↻      girar e inclinar
 ◀  △  ▽  ▶      mover
 +  −  recentrar  aproximar / afastar / voltar ao enquadramento inicial
 seguir robô      liga/desliga o acompanhamento (vale para as DUAS vistas)
```

Segurar o botão repete. Os comandos agem sobre **a câmera que está na tela** —
trocar entre `iso` e `topo` troca o alvo junto. O `seguir robô` é a exceção: ele
vale para as duas de uma vez, porque o que ele muda é o **alvo da órbita**, e não
há versão disso que faça sentido para uma câmera só.

O navegador publica **deltas** em `/demo/cockpit/scene/cmd_view`
(`geometry_msgs/TwistStamped`, com o `header.frame_id` escolhendo a câmera). Quem
guarda a órbita, satura os limites e escreve a pose no Gazebo é o nó
`scene_view_controller`. Por isso recarregar a página **não** mexe no
enquadramento, e dois cockpits abertos não brigam pela pose.

### Seguir o robô

Ligado por default. As duas câmeras acompanham `/demo/odom` a 10 Hz, e o que se
move é só o **alvo** da órbita — azimute, elevação e distância ficam onde
estavam. Consequência prática: o enquadramento medido do armazém (a diagonal de
`(-3, +3, 2,4)` na iso, os 6 m de altura na topo) continua valendo enquanto o
robô caminha, em vez de o robô sair do quadro em quinze segundos.

Os botões de mover (`◀ △ ▽ ▶`) continuam úteis com o seguimento ligado: com ele
ligado o pan vira um **deslocamento relativo ao robô**, saturado em 15 m, e não
um ponto fixo do mundo. Serve para olhar o robô de lado, ou um pouco à frente
dele, sem perder o acompanhamento. `recentrar` zera esse deslocamento junto com
o resto.

O estado do botão vem de `/demo/cockpit/scene/following`
(`std_msgs/Bool`, latched), publicado pelo nó, e **não do próprio clique** — o
mesmo raciocínio do rótulo de simulação, e pelos mesmos três casos: F5, segundo
cockpit aberto, e alguém que desligou o seguimento por `ros2 service call`.

Desligar (`follow:=false` no launch, ou o botão) devolve a vista larga do
cenário, que é o que se quer para conferir o mundo inteiro ou para comparar com
um enquadramento anterior.

### Reiniciar a navegação

**reiniciar nav**, no cabeçalho do painel verde. Chama `/demo/nav/reset`
(`std_srvs/Trigger`), servido pelo `nav_control_relay` **dentro do container que
roda o Nav2** — no `hil` isso é o Aquila.

Como o reset da simulação, **exige dois cliques** (o primeiro arma por 4 s). Não
por simetria: ele descarta a meta em andamento, e um clique por engano no meio de
uma demo custa a demo.

A sequência, na ordem, e cada passo existe:

1. `CancelGoal` em `/navigate_to_pose` com `goal_info` zerado — cancela **todas**
   as metas, inclusive a que o cockpit não sabe que existe (mandada pelo
   `patrol_commander`, ou por outra aba);
2. `ClearEntireCostmap` no global e no local, **com os servidores ainda ativos**
   — um nó desativado não responde serviço, então limpar depois de pausar não
   limparia nada e não daria erro;
3. `PAUSE` no `lifecycle_manager_navigation`;
4. `RESUME`.

Medido em 6,6 s no host. A meta interrompida termina `CANCELED`, os servidores
voltam `active [3]`, e uma meta nova é aceita em seguida.

O HUD do painel mostra `reiniciando…` durante a sequência e `reiniciado` no fim.
Sem timeout do lado do navegador: quem tem o timeout é o nó (60 s por transição),
e fechar a aba no meio **não** interrompe o reset.

**Por que não `RESET` + `STARTUP`**, que é o caminho óbvio: ele derruba o
container. Ver a armadilha 8.

A localização **não** é tocada. `lifecycle_manager_localization` fica fora da
sequência de propósito: no caminho de mapa estático, reciclar o AMCL joga a pose
fora e o robô "se perde" num reset que era só para descartar a meta.

### Controle manual: desligado de propósito

As setas e o E-STOP da barra estão desenhados, alcançáveis por teclado, e
**inertes**, com o motivo no `title`. Hoje o Nav2 é o único publicador em
`/demo/cmd_vel`; um botão de teleop publicando ali daria dois escritores não
arbitrados no mesmo tópico — o último a escrever ganha, e nenhum dos dois sabe
que perdeu. Fecha no **F4**, com `twist_mux`, não com um mux escrito à mão no
navegador.

---

## 5. O que NÃO dá para fazer

### Rodar a simulação no Aquila

Não é limitação de implementação, é a regra 1 do `CLAUDE.md`: o Gazebo é OGRE 2
e precisa de OpenGL de desktop; o AM69 expõe apenas OpenGL ES 3.2 e Vulkan 1.2.
**Nenhuma quantidade de código na UI muda isso.**

O que existe é **controlar a simulação a partir do cockpit**. No M3, com o
cockpit servido pelo Aquila, o clique sai do módulo e a chamada de serviço
atravessa o grafo ROS — exatamente como a meta do Nav2 já atravessa hoje. O
processo do simulador continua na workstation x86.

Se a distinção parecer sutil na hora da demo: o que está na tela do módulo é
uma **imagem** vinda do host, e um **botão** que fala com o host.

### Chamar serviço com tipo do Gazebo pelo navegador

Também não é limitação temporária. O `rosbridge` monta o pedido importando o
pacote de interfaces **dentro do próprio container**, e o container `cockpit`
não tem `ros_gz_interfaces` — nem deve ter, porque no M3 ele roda no Aquila e no
modo `deploy` não existe Gazebo nenhum. Ver a armadilha 2 na
[seção 9](#9-as-armadilhas-que-já-custaram-tempo).

---

## 6. Trocar de cenário

O mundo padrão é o armazém do `nav2_minimal_tb4_sim`. Para o labirinto de 11,6 ×
11,6 m, duas variáveis de ambiente (em `docker/.env` ou exportadas antes do
`up`):

```bash
export MAZE_MODELS=/caminho/para/ros_maze_worlds/models

export SIM_ARGS="world:=/ws/install/demo_simulation/share/demo_simulation/worlds/quadruped_maze11.sdf \
  yaw:=1.5708 \
  scene_top_x:=-4.855 scene_top_y:=4.855 scene_top_z:=13.0 \
  scene_iso_x:=-13.0 scene_iso_y:=-3.0 scene_iso_z:=9.0 \
  scene_iso_pitch:=0.6717 scene_iso_yaw:=0.7676"

docker compose -f compose.host.yml up -d --force-recreate sim
```

O que cada grupo faz:

- **`MAZE_MODELS`** — o `sim` monta esse diretório em `/maze/models` e aponta
  `GZ_SIM_RESOURCE_PATH` para lá. **Sem ele o mundo carrega e o labirinto
  simplesmente não está lá** — sem erro nenhum. Os modelos vêm de
  `github.com/cafemesa/ros_maze_worlds` e nenhum asset externo é copiado para o
  repositório.
- **`yaw:=1.5708`** — o canto de partida do labirinto tem parede no `+x`
  default; sem girar, o robô nasce de cara na parede.
- **`scene_*`** — enquadramento das duas câmeras de cena. Os valores acima foram
  **medidos**, não escolhidos: são os que pegam o labirinto inteiro
  (`scene_top`) e uma diagonal legível (`scene_iso`).

`SIM_ARGS` é livre e passa direto para `sim.launch.py`. É onde mora tudo que é
do **cenário** e não do **modo**. Vazio = armazém.

### Por que as câmeras de cena são modelos e não estão nos mundos

Duas razões que só aparecem depois:

1. o mundo default é de terceiros e **não é editável** — e é ele que sobe quando
   ninguém passa `world:=`;
2. pendurar a câmera no `go2_description` vendorizado quebraria a **garantia
   byte a byte** que sustenta o argumento de licença.

Por isso elas são modelos spawnáveis
(`demo_simulation/models/cockpit_scene_{iso,top}.sdf`), plantados em qualquer
mundo por `scene_cameras.launch.py`.

---

## 7. Ajustar qualidade de imagem

Padrão atual: **1600 × 1200 a 10 Hz**, anti-aliasing 8, JPEG de qualidade 95.

**A taxa é 10 e não 15 por medição.** As duas câmeras renderizam no mesmo
processo do Gazebo, e nesta resolução a workstation não entrega 15 Hz de
qualquer forma. Medido na bancada, mesmo mundo (`maze11`), janela de 10 s:

| `update_rate` | entregue no tópico | fator de tempo real |
| --- | --- | --- |
| 15 | 9,43 Hz | **0,59** |
| 10 | 9,77 Hz | **0,97** |

Pedir 15 não rendia um quadro a mais e custava 40% da velocidade da simulação —
o que estica cada meta do Nav2 na mesma proporção.

**Se precisar economizar, mexa nesta ordem:**

1. **qualidade JPEG**, em `hmi/js/config.js` (`STREAM_QUALITY`). Degrada
   suavemente e não muda nada do lado do ROS. É o primeiro lugar a mexer se o
   gargalo for **banda** — modo `hil`, stream atravessando a Ethernet até o
   Aquila.
2. **`update_rate`** nos dois SDF, se o gargalo for **render**.
3. **resolução**, por último. Ela desloca o enquadramento em pixels; e a
   **proporção 4:3 não pode mudar** — o `horizontal_fov` e as poses das duas
   câmeras foram medidos nela, e ir para 16:9 mantendo o hfov corta vertical e
   desenquadra as duas cenas de uma vez, sem erro nenhum.

Qualquer mudança nos SDF exige rebuild de `base` **e** de `sim` (o workspace é
compilado em `base`), e o cache do Docker mente aqui — ver armadilha 4.

### Para medir o fator de tempo real

```bash
docker compose -f compose.host.yml exec -T sim bash -lc 'source /ws/install/setup.bash
  ros2 topic echo --once /clock; sleep 10; ros2 topic echo --once /clock'
```

Divida o avanço do tempo simulado pelos 10 s de parede.

---

## 8. Diagnóstico

O cockpit foi feito para responder sozinho "esse tópico está chegando?" — é para
isso que o nome do tópico está impresso em cada cabeçalho e que existe o ponto de
frescor. Quando não bastar:

```bash
cd docker

# A página existe?
curl -o /dev/null -w '%{http_code}\n' localhost:8081/

# O rosbridge subiu?
docker compose -f compose.host.yml logs cockpit | grep -i rosbridge

# As câmeras de cena estão publicando?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 topic hz /demo/cockpit/scene_iso/image_raw'

# O MJPEG responde?
curl -o /dev/null -w '%{http_code}\n' \
  'localhost:8080/snapshot?topic=/demo/cockpit/scene_iso/image_raw'

# Os serviços de simulação existem?
docker compose -f compose.host.yml exec -T sim bash -lc \
  'source /ws/install/setup.bash; ros2 service list | grep /demo/sim'
```

**O console do navegador é fonte de primeira classe aqui.** Falhas de chamada de
serviço aparecem lá (`[cockpit] falha ao pausar a simulação: ...`) e, com mais
detalhe, no log do container `cockpit`.

### Sintomas comuns

| Sintoma | Causa provável |
| --- | --- |
| Painel verde vazio, HUD diz `sem TF map→base` | armadilha 1 |
| Botão de simulação não faz nada | armadilha 2 — olhe o log do `cockpit` |
| Painel azul preto, ponto cinza | mundo sem as câmeras: `sim` recriado sem `SIM_ARGS`? |
| Labirinto não aparece, chão vazio | `MAZE_MODELS` não exportado |
| Editei o bundle e nada mudou | a imagem `hmi` não foi reconstruída |
| Editei um SDF e nada mudou | armadilha 4 |
| Tudo cinza, badge `desconectado` | `cockpit` caiu, ou a porta 9090 colidiu |

---

## 9. As armadilhas que já custaram tempo

### 1. O `/tf_static` chega **uma vez só**, e qual mensagem é sorte

Sintoma: o painel verde ficava em `sem TF map→base` em cerca de metade dos
carregamentos.

O `rosbridge` entrega **uma** mensagem latched por inscrição. Medido em três
inscrições novas e consecutivas: a primeira trouxe as arestas do robô, a segunda
`map→odom`, a terceira `map→odom`. `queue_length: 16` não muda nada — a perda é
**acima** da fila do cliente.

Correção, já no código: `nav-panel.js` reinscreve em `/tf_static` a cada 1,5 s,
no máximo 8 vezes, e para em definitivo assim que `lookup('map','base')`
resolve. Converge em 2 a 4 rodadas.

### 2. O navegador não pode chamar serviço com tipo do Gazebo

Sintoma: os botões de simulação não faziam absolutamente nada. A UI não
mostrava erro; a causa só aparecia no log do container `cockpit`:

```
call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
from package ros_gz_interfaces. Caused by: No module named 'ros_gz_interfaces'
```

O `rosbridge` monta o pedido importando o pacote de interfaces **dentro do
próprio container**, e o `cockpit` não tem `ros_gz_interfaces`. E não deve ter:
no M3 ele roda no Aquila, e no modo `deploy` não há Gazebo nenhum.

Correção: o nó **`sim_control_relay`** (lado do simulador) expõe
`/demo/sim/{play,pause,reset}` como `std_srvs/Trigger` e traduz para
`ControlWorld`. A fronteira do navegador só fala tipos de núcleo do ROS.
Guardado por `tests/test_cockpit_web_contract.py::test_browser_never_speaks_gazebo_interfaces`.

### 3. O nome do mundo não é o nome do arquivo

Os serviços do Gazebo moram em `/world/<nome>/...`, e `<nome>` é o atributo do
elemento `<world>`. `quadruped_maze11.sdf` declara `<world name="quadruped_maze11">`,
mas o armazém do `nav2_minimal_tb4_sim` declara `<world name='warehouse'>`.
Adivinhar pelo nome do arquivo acerta num caso e erra no outro, **em silêncio**:
a ponte sobe, anuncia os serviços ROS, e cada chamada expira num serviço gz que
não existe.

Por isso `sim_control.launch.py` faz parse do SDF, e um arquivo sem `<world>`
derruba o launch dizendo qual arquivo era.

### 4. O cache do Docker mente sobre `COPY ros2_ws/src`

Sintoma: você edita um SDF ou um `.py` do workspace, reconstrói, e a mudança não
está no container. O build reporta `CACHED` para a camada de `COPY`.

Aconteceu três vezes em uma sessão. **Contorno: rodar `docker compose build base`
duas vezes** — a segunda pega. E o workspace é compilado em `base`, então
qualquer mudança em `ros2_ws/` exige `base` **e depois** `sim`:

```bash
export DOCKER_BUILDKIT=0
docker compose -f compose.host.yml build base
docker compose -f compose.host.yml build base   # sim, de novo
docker compose -f compose.host.yml build sim
```

Sempre confirme antes de concluir que a mudança não funcionou:

```bash
docker compose -f compose.host.yml exec -T sim \
  grep update_rate /ws/install/demo_simulation/share/demo_simulation/models/cockpit_scene_iso.sdf
```

**A variante pior é rodar a suíte de testes de um pacote dentro do container.**
`/ws/src` é uma cópia da imagem, não um bind mount do seu diretório de trabalho:
uma imagem de antes da sua edição roda os testes **antigos** e passa. Um teste que
você acabou de escrever simplesmente não é coletado, e a saída é verde. Aconteceu
em 25/08: `32 passed` na imagem reconstruída contra `22 passed` na anterior, com o
arquivo novo ausente da lista de coleta. Confira a contagem, ou confira a
coleta:

```bash
docker compose -f compose.host.yml run --rm -T tools \
  bash -lc 'ls /ws/src/demo_simulation/test/'
```

### 5. Buildx não enxerga imagens locais

`docker compose build` com o builder `armbuilder` ativo falha com
`pull access denied ... local/demo-aquila-base:dev`. O builder multi-arquitetura
não vê o daemon local. Para builds de bancada:

```bash
export DOCKER_BUILDKIT=0 BUILDX_BUILDER=default
```

### 6. `node --test` com glob sem aspas passa sem rodar nada

```bash
node --test "test/**/*.test.js"    # certo
node --test test/**/*.test.js      # verde sem ter rodado teste
```

Sem aspas o shell expande, o Node recebe um diretório, resolve para nada e sai
com código 0.

### 7. Enquadramento de câmera se mede, não se chuta

A primeira tentativa (iso em `-7,-7,5`) caía **dentro** dos corredores de
prateleira do armazém — o robô virava um ponto branco atrás de uma prateleira. A
de topo a 12 m batia numa viga do telhado exatamente sobre o robô. Os dois
enquadramentos atuais saíram de tentativas medidas contra a pegada real do
cenário.

Ao testar comando de órbita pela linha de comando, use `ros2 topic pub -t 1 -w 1`
e não `-r 3`: seis segundos a 3 Hz aplicam ~18 passos de 0,35 rad ≈ 2π, a câmera
volta ao ponto de partida, e parece que nada aconteceu.

### 8. `RESET` + `STARTUP` no Nav2 mata o container (segfault no `route_server`)

Este era o desenho natural do "reiniciar nav": o `lifecycle_manager` do Nav2 tem
`RESET` (desativa e desconfigura tudo) e `STARTUP` (configura e ativa tudo), e
nenhuma outra dupla de transições descreve tão bem "reinicie a pilha".

Medido em 24/08/2026, no host, modo `learn`, o `nav2_container` morre:

```
[component_container_isolated-4] [INFO] [route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [exit code -11]
```

`-11` é `SIGSEGV`. Reproduzido **duas** vezes — com meta ativa e sem meta ativa.
Não é o "aconteceu uma vez" que este guia registrava antes: é determinístico, e
está no caminho do `CONFIGURE`, que é justamente o que `STARTUP` faz.

O `route_server` está na lista de `lifecycle_nodes` do `navigation_launch.py`
vendorizado — que tem de seguir **idêntico ao upstream** (ver
`launch/nav2_vendored/README.md`), então tirá-lo da lista não é opção. Esta demo
não usa roteamento, e ele não tem seção em `nav2_params_go2.yaml`; a hipótese é
que ele reconfigure sobre estado que não sobrevive ao `CLEANUP`, mas isso não foi
confirmado no fonte do Nav2. **Candidato a issue upstream.**

O que o `nav_control_relay` faz em vez disso — cancelar, limpar costmaps,
`PAUSE`, `RESUME` — nunca passa por `CONFIGURE`, e por isso nunca chega perto
disso. Se algum dia alguém "simplificar" a sequência para `RESET`+`STARTUP`, o
sintoma será o container `nav` reiniciando e o cockpit perdendo o link no meio da
demo. Existe um guarda estrutural em `tests/` exatamente para isso.

#### Não é só no reset: acontece no boot normal (25/08/2026)

A frase acima, "está no caminho do `CONFIGURE`", estava certa e era estreita
demais. O `CONFIGURE` do `route_server` também acontece na **subida normal** do
`nav`, e lá o mesmo segfault aparece — de forma **intermitente**: o mesmo
`docker compose up nav` subiu numa vez e derrubou a pilha na seguinte.

O estado que ele deixa é o que faz isso caro:

```
$ docker compose ps
nav   running                       <- e mentira

$ docker exec docker-nav-1 ps -eo comm
ros2                                <- so o pai
odom_tf
nav_control_rel
cmd_vel_si_to_s                     <- nenhum servidor do Nav2
```

Morrem **todos** os servidores de uma vez, o container segue `running`, e de
dentro dele `get_node_names()` lista só os nós do `sim`. Quem olha o Compose
conclui "o Nav2 está de pé"; quem olha o robô conclui "a navegação piorou". Foi
metade da regressão investigada em `docs/results/ml35-regressao-navegacao.md`.

A correção vive em `nav2_params_go2.yaml`: `route_server.operations` lista só
`AdjustSpeedLimit`, então o plugin que estoura nunca é construído. Sobrepor essa
lista obriga a declarar também o **tipo** de cada plugin dela
(`AdjustSpeedLimit.plugin`), senão a subida é reprovada com
`Can not get 'plugin' param value` — falha alta, e nisso melhor que o segfault.

### 9. O reset derrubava o quadrúpede em marcha

O reset da simulação teleporta o robô de volta à pose inicial via
`SetEntityPose` (ver armadilha 2 sobre por que não é uma chamada direta do
Gazebo pelo navegador). Isso resolve um defeito pior — `reset.all` **apagava**
o robô inteiro, ver `docs/results/cockpit-reset-nao-destrutivo.md` §1-2 — mas
sozinho não bastava para o quadrúpede, e por dois motivos diferentes,
descobertos em sequência:

1. **teleportar sem reancorar o controlador de marcha.** O `StateTrotting`
   (controlador C++ do gait) captura sua referência de postura uma única vez, e
   um teleporte muda a pose sem passar por essa captura. Medido sem nenhum
   comando de velocidade publicado por 26 s: mesmo assim o robô se arrastou
   0,87 m e girou 135° sozinho, perseguindo a pose de ANTES do reset;
2. **teleportar sem parar.** `SetEntityPose` reposiciona o corpo e **preserva a
   velocidade**. Um robô em marcha, teleportado, é solto ainda viajando com as
   pernas em balanço — e cai. Medido com o Nav2 conduzindo de verdade durante o
   reset: a altura do robô caiu de 0,337 m para 0,162 m em 1 segundo.

A correção para o robô ANTES de teleportar e o reancora DEPOIS — dois serviços
internos (`/demo/gait/hold`, `/demo/gait/resume`), servidos pelo mesmo nó que já
traduzia `/demo/cmd_vel` para os eixos do gait. Nenhum dos dois é exposto ao
cockpit; o operador só vê o efeito, que é o clique de reset levar ~2-7 s a mais
no quadrúpede do que no diff-drive. Verificado inclusive com o robô **caído**
(tombado, preso no modo de recuperação do controlador — que não sai sozinho de
cabeça para baixo): o reset o devolve de pé.

Evidência completa: `docs/results/cockpit-reset-nao-destrutivo.md` §3.1.

---

## 10. Como o cockpit é feito

### Árvore

```
hmi/
├── index.html            as cinco regiões
├── css/
│   ├── tokens.css        paleta, tipografia, movimento — fonte ÚNICA de cor
│   ├── layout.css        a grade de cinco regiões
│   └── panels.css        cromo dos painéis, barra, pad de câmera
├── img/                  marcas Toradex e ROS (PNG branco com alfa)
├── js/
│   ├── main.js           só fiação: resolve config, monta painéis, um timer
│   ├── config.js         endpoints, tópicos, URL do MJPEG
│   ├── ros/              transporte
│   │   ├── rosbridge-client.js   WebSocket, reconexão, ações, serviços
│   │   ├── png-decompress.js     costmap comprimido (33x menor que JSON)
│   │   ├── tf-tree.js            cache de TF
│   │   └── freshness.js          os pontos verde/laranja/cinza
│   ├── panels/           renderização
│   │   ├── stream-panel.js       painéis de imagem (cena e câmera)
│   │   ├── nav-panel.js          canvas do mapa, clique-para-meta
│   │   ├── map-view.js           mundo <-> tela, LUT de custo (puro, testado)
│   │   ├── palette.js            lê os tokens --map-* do CSS
│   │   ├── log-panel.js          /rosout + telemetria
│   │   ├── control-bar.js        estado do link
│   │   ├── sim-controls.js       play/pause/reset
│   │   ├── view-controls.js      pad de câmera
│   │   └── detection-overlay.js  NÃO montado hoje — ver abaixo
│   └── ...
└── test/                 node --test, sem navegador
```

### Do lado do ROS

| Arquivo | Papel | Roda em |
| --- | --- | --- |
| `demo_bringup/launch/cockpit.launch.py` | rosbridge + web_video_server | host hoje, Aquila no M3 |
| `demo_simulation/launch/scene_cameras.launch.py` | planta as duas câmeras de cena | host |
| `demo_simulation/launch/sim_control.launch.py` | ponte de serviços gz + fachada | host |
| `demo_simulation/scene_view_controller.py` | órbita das câmeras de cena | host |
| `demo_simulation/sim_control_relay.py` | fachada `std_srvs` para play/pause/reset | host |
| `demo_navigation/launch/nav_control.launch.py` | sobe a fachada de reset do Nav2 | host ou Aquila |
| `demo_navigation/nav_control_relay.py` | fachada `std_srvs` para reiniciar o Nav2 | onde o Nav2 roda |

As duas últimas linhas são o único par desta tabela que **roda no módulo** no
modo `hil`: elas moram no container `nav`, junto da pilha que reiniciam. A
fachada de simulação fica presa ao host porque o Gazebo fica.

### Um detalhe que parece bug e não é

O painel da câmera **não desenha as caixas de detecção**, e a ausência é
deliberada. O `demo_perception` de hoje é um stub determinístico: varre uma
caixa sintética pela imagem quer haja objeto ali ou não. Sobre o vídeo isso vira
um retângulo passeando de um lado para o outro — pior que nada numa demo, porque
o espectador lê aquilo como detecção de verdade.

**As detecções continuam publicadas e continuam alimentando a
`perception_layer` do costmap.** O contrato de tópicos do `CLAUDE.md` está
intacto; saiu só o desenho. `detection-overlay.js` segue no bundle, testado,
para voltar quando o TIDL substituir o stub.

### Testes

```bash
cd hmi && node --test "test/**/*.test.js"    # 138 — lógica do bundle
cd .. && python3 -m pytest tests/ -q          # 36 — guardas estruturais
```

Os guardas estruturais são checagens estáticas em arquivos commitados, não
testes de runtime. Existem porque cada invariante que eles cobrem é barata de
quebrar numa edição de uma linha e cara de descobrir — as de colocação só falham
no Aquila, semanas depois.

---

## 11. O que está feito e o que falta

### Feito (24/08/2026, só no host)

- **F1** — transporte e esqueleto: serviços `cockpit` e `hmi`, bundle, câmera ao
  vivo, reconexão automática. Evidência: [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md).
- **F3b** — painel de cena (duas câmeras alternáveis) e painel de navegação
  (costmap, plano, laser, pegada, clique-para-meta). Portão cumprido: meta
  clicada aceita e executada pelo Nav2.
- **Controle de simulação** pelo cockpit: play, pause, reset.
- **Controle de câmera**: girar, inclinar, mover, zoom, recentrar.
- **Identidade Toradex** e repaletização do mapa para fundo claro.
- **Qualidade de imagem**: 800×600@5 Hz → 1600×1200@10 Hz, JPEG 70 → 95.

Evidência do conjunto: [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md).

### Ajustes de UI (25/08/2026, só no host)

Três pedidos de bancada, fora da numeração de fases:

- **Marca Toradex ao dobro** (34 → 68 px de altura; a do ROS, 22 → 44). A altura
  mínima da faixa saiu de 48 para 80 px **pelo mesmo token** (`--bar-min-height`
  em `tokens.css`) — a barra tem `overflow-x`, não `-y`, então as duas medidas
  divergirem cortaria a marca sem avisar.
- **Seguir o robô** nas duas vistas de cena. Verificado no host: `scene_top` em
  `(-1,552 ; 0,151 ; 6,0)` contra robô em `(-1,598 ; 0,130)` — acompanhamento
  dentro de ~5 cm com o `z` preservado; `scene_iso` em `(-4,551 ; 3,15 ; 2,4)`,
  isto é, o deslocamento medido `(-3, +3, 2,4)` mantido enquanto desliza com o
  robô. Desligar congelou a pose por 6 s; religar recentrou.
- **Reiniciar a navegação** pelo cockpit, em 6,6 s, com a meta interrompida
  terminando `CANCELED` e os servidores voltando `active [3]`.

Nada disso foi visto num navegador com captura de tela: o Chrome não está
instalado nesta máquina e o Firefox snap em modo headless não respondeu. O que
existe é a página servida com o HTML e o CSS corretos (`curl` 200, tokens e os
dois botões presentes no que o nginx entrega) mais a verificação pelo lado do
ROS. **Nada disso rodou no Aquila AM69** (regra 7 do `CLAUDE.md`); as medidas são
todas do host x86 em `learn`.

### Falta

- **F4 — controle manual.** `twist_mux` arbitrando contra o Nav2. É a única
  região da tela que ainda mente, e por isso os botões estão desligados.
- **F2 — kiosk no módulo.** Chromium no Aquila servindo este mesmo bundle. Três
  coisas que só o módulo responde:
  1. o bundle foi verificado no **Firefox**; o kiosk é Chromium, e a ressalva de
     cache do `<img>` em `config.js` vem da literatura, não de medição;
  2. o MJPEG a 1600×1200 atravessando a Ethernet — nada disso foi medido lá;
  3. no modo `deploy` não existe Gazebo: os botões de simulação precisam sumir
     ou dizer por que não valem. **Ainda não foi tratado.**

### Uma pendência conhecida, sem relação com o cockpit

O segfault do `route_server` ao configurar, agora caracterizado e
determinístico — ver a [armadilha 8](#8-reset--startup-no-nav2-mata-o-container-segfault-no-route_server).
Não é regressão do cockpit; é o motivo pelo qual o reset de navegação usa
`PAUSE`/`RESUME` em vez de `RESET`/`STARTUP`.

---

## Referências

- [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md) — decisões e fases
- [`ml35/estado-fases.md`](ml35/estado-fases.md) — estado do ML3.5, leia primeiro numa sessão nova
- [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md), [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md) — evidência de bancada
- [`guia-operacao.md`](guia-operacao.md) — o resto da demo: robô, Nav2, percepção, módulo
- `CLAUDE.md` na raiz — as regras invioláveis, principalmente a 1
