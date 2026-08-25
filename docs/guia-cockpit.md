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
laranja por 4 s), o segundo executa. Ele devolve o robô à pose inicial **e**
apaga o costmap que o Nav2 acumulou — um clique por engano no meio da demo custa
a demo.

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
```

Segurar o botão repete. Os comandos agem sobre **a câmera que está na tela** —
trocar entre `iso` e `topo` troca o alvo junto.

O navegador publica **deltas** em `/demo/cockpit/scene/cmd_view`
(`geometry_msgs/TwistStamped`, com o `header.frame_id` escolhendo a câmera). Quem
guarda a órbita, satura os limites e escreve a pose no Gazebo é o nó
`scene_view_controller`. Por isso recarregar a página **não** mexe no
enquadramento, e dois cockpits abertos não brigam pela pose.

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
cd hmi && node --test "test/**/*.test.js"    # 131 — lógica do bundle
cd .. && python3 -m pytest tests/ -q          # 29 — guardas estruturais
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

O `nav2_container` deu segfault (`exit code -11`) uma vez ao configurar o
`route_server`. Reiniciar o serviço `nav` resolveu. Se voltar, é candidato a
issue própria — não é regressão do cockpit.

---

## Referências

- [`ml35/plano-cockpit-web.md`](ml35/plano-cockpit-web.md) — decisões e fases
- [`ml35/estado-fases.md`](ml35/estado-fases.md) — estado do ML3.5, leia primeiro numa sessão nova
- [`results/cockpit-web-f1.md`](results/cockpit-web-f1.md), [`results/cockpit-web-f3b.md`](results/cockpit-web-f3b.md) — evidência de bancada
- [`guia-operacao.md`](guia-operacao.md) — o resto da demo: robô, Nav2, percepção, módulo
- `CLAUDE.md` na raiz — as regras invioláveis, principalmente a 1
