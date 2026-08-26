# Cockpit web — plano aprovado e registro de decisões

**Data:** 24/08/2026
**Estado:** plano aprovado. **F1 concluído em 24/08/2026** — evidência em
`docs/results/cockpit-web-f1.md`. Próxima fase: F3b.
**Substitui:** a seção "Próxima metodologia recomendada" de
`docs/results/cockpit-standalone-parcial.md`, que recomendava Qt/`rviz_common`
e foi descartada — ver Decisão 3.

Quem retomar este trabalho lê **este arquivo** e depois vai direto para a
seção "Onde retomar". O levantamento já foi feito; não precisa ser refeito.

---

## 1. O problema, diagnosticado

A falha das quatro tentativas anteriores não foi de framework. Foi de **eixo**.

Reparentar (`XReparentWindow`) janelas de clientes que já pertencem ao Mutter
só funciona quando o cliente implementa XEmbed. O Gazebo entrou porque é uma
janela Qt simples; RViz2 e `rqt_image_view` não entraram e permaneceram sob
frames `mutter-x11-frames` próprios. Isso é comportamento **correto** do
compositor, não defeito.

A única forma de vencer esse eixo era desligar o GNOME, o que já está proibido
no procedimento (a tentativa de `gnome-shell --replace` interrompeu a sessão do
operador).

**Nenhuma tecnologia de UI resolve isso, porque o obstáculo está abaixo da UI.**
Trocar PyQt5 por GTK, Qt6, wxWidgets ou Electron não muda nada.

---

## 2. Decisões tomadas em 24/08/2026

### Decisão 1 — Trocar o eixo de "captura de janela" para "dados"

Existem dois eixos estáveis, ambos evitando o WM:

| Eixo | Como | O que é embutido |
| --- | --- | --- |
| **A — dados** | cada painel renderiza a partir de tópicos ROS 2 | nada; a app desenha |
| **B — pixels** | cada app num servidor X **próprio**, transmitida | um framebuffer |

**Escolhido: eixo A.** No eixo B o GNOME nunca veria as janelas (o Gazebo seria
o único cliente do seu Xvfb, dono absoluto daquela árvore X), o que é técnica
madura — mas foi descartado na Decisão 4.

### Decisão 2 — Cockpit web (`rosbridge` + `web_video_server` + HTML/CSS/JS)

Avaliadas quatro opções:

| Opção | Layout da imagem | Gazebo/RViz reais | Reuso no kiosk AM69 | Escolha |
| --- | --- | --- | --- | --- |
| **1. Cockpit web** | exato (CSS grid) | não (eixo A) | **total** | ✅ **adotada** |
| 2. Lichtblick/Foxglove | aproximado, sem marca | não | não | descartada |
| 3. Qt/C++ + `rviz_common` | exato | RViz sim, Gazebo não | não | descartada |
| 4. Continuar com Xlib | — | — | — | rejeitada |

Motivos concretos da 1, não preferência:

- **É o que o próprio projeto já especificou.** `.ai/AGENTS.md` §5.7 define
  `ROS 2 → rosbridge_server → WebSocket → Chromium kiosk`, e a árvore de §4 já
  reserva `hmi/` e `docker/hmi/` (este último existe e está **vazio**). Não é
  caminho novo — é a spec que ainda não foi executada.
- **Um deliverable, dois destinos.** O mesmo bundle é o cockpit do host hoje e
  o HMI do M3 no AM69 depois. As opções 2 e 3 seriam descartadas no caminho.
- A barra de controles com logo Toradex e o painel de logs **só existem** na 1.
- A opção 3 força C++ (não existe binding Python de `rviz_common`), contraria a
  convenção "Python por padrão" do CLAUDE.md, e ainda deixaria o Gazebo sem
  solução.

Ressalva registrada: o Foxglove Studio open-source foi descontinuado no formato
antigo e o fork ativo é o Lichtblick (BMW). **Isso não foi verificado nesta
sessão** e não sustentou o descarte da opção 2 — ela caiu por layout e por
não servir ao kiosk.

### Decisão 3 — Descartar a recomendação do checkpoint anterior

`cockpit-standalone-parcial.md` recomendava incorporar
`rviz_common::RenderPanel` + `VisualizationManager`. Descartado: é C++ obrigatório,
resolve só um dos cinco painéis, e não serve ao alvo arm64 (RViz2 é OGRE 2 e
não pode rodar no AM69 — regra 1 do CLAUDE.md). Aquele documento continua
válido como registro histórico da falha; a recomendação final dele, não.

### Decisão 4 — Ir direto para o fallback por dados, sem o spike do eixo B

O plano previa uma fase **F2**: spike de 90 min com Gazebo em display X isolado
+ aceleração por `/dev/dri` + noVNC em `<iframe>`, com portão de
`glxinfo` reportando renderizador de hardware e ≥15 FPS interativos.

**O operador decidiu pular a F2 e ir direto ao F3b** (caminho por dados).

Custo aceito explicitamente com essa decisão:

- perde-se a **órbita livre de câmera** do Gazebo;
- perde-se a **árvore de displays** do RViz (ligar/desligar camadas em runtime);
- ganha-se clique-para-meta no mapa, CPU/GPU livres no host (Gazebo passa a
  `gui:=false`), e um único caminho de transporte para todos os painéis.

Consequência: **F2 e F3a não existem mais.** As fases restantes são
F1 → F3b → F4 → F5 → F6, mantidos os nomes originais para casar com este
registro.

### Decisão 5 — Sem etapa de build npm; cliente rosbridge próprio

`.ai/AGENTS.md` §5.7 pede "dependency set mínimo e estável". O bundle será
HTML/CSS/JS em ES modules puro, servido por `nginx:alpine` (multi-arch),
**sem `npm install`, sem bundler, sem step de build**. Isso elimina a cadeia de
suprimentos npm em arm64, que é onde ela dói.

Consequência: `roslibjs` **não** será vendorizado. O protocolo rosbridge v2 é
JSON simples (`subscribe`/`unsubscribe`/`publish`/`advertise`/`call_service`) e
um cliente mínimo com reconexão cabe em ~200 linhas testáveis.

Contexto: a checagem de alcance do registro npm foi **negada pelo sandbox**
nesta sessão, então a disponibilidade de `roslib` no registro **não foi
confirmada**. Isso reforça a decisão, mas não foi a causa dela.

### Decisão 6 — Câmera de cena vai no mundo, nunca no `go2_description`

O painel azul ("GAZEBO SIMU VIEW") passa a ser uma câmera do mundo bridged para
ROS. Ela vai em `ros2_ws/src/demo_simulation/worlds/*.sdf`, que são arquivos do
projeto.

`go2_description` é **vendorizado com garantia byte-a-byte que sustenta o
argumento de licença** (ver o README do pacote e o cabeçalho de
`bridge_quadruped.yaml`). Não pode ser editado — nem para pendurar uma câmera
de perseguição no tronco. Se uma câmera que segue o robô for necessária depois,
o caminho é um modelo SDF separado movido pelo serviço gz `set_pose` a partir
de `/demo/odom`, **não** editar o pacote vendorizado.

Início: duas câmeras estáticas (isométrica e de topo), alternáveis na UI.

### Decisão 7 — `twist_mux` resolve o débito de arbitragem

O checkpoint anterior registrou que teleop e Nav2 podem publicar no mesmo
`/demo/cmd_vel` sem arbitragem. Resolvido com `ros-jazzy-twist-mux` (pacote
padrão, disponível nas duas arquiteturas), prioridade manual > Nav2, com
timeout. Não escrever mux à mão.

---

## 3. Evidências verificadas nesta sessão

Checadas de fato, não supostas:

| Fato | Como foi verificado |
| --- | --- |
| `ros-jazzy-rosbridge-suite` 2.7.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-rosbridge-server` 2.7.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-web-video-server` 3.1.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-twist-mux` 4.5.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-foxglove-bridge` 3.4.1 | `apt-cache policy`, amd64 |
| Os cinco existem em **arm64** | `packages.ros.org/ros2/ubuntu dists/noble main/binary-arm64/Packages.gz` |
| Host tem Node v24.15.0 / npm 11.14.1 | `node --version`, `npm --version` — **não será exigido em runtime** |
| Alcance do registro npm | **NÃO verificado** (comando negado pelo sandbox) |
| Go2 tem câmera frontal | `go2_description/xacro/gazebo.xacro:259`, `<sensor name="front_camera" type="camera">` |
| Câmera já chega ao contrato | `bridge_quadruped.yaml` → `/demo/camera/image_raw`, QoS reliable (deliberado; best-effort perdia os fragmentos 640x480 no enlace) |
| Metas vão por **ação**, não tópico | `scripts/nav_trial.py:131` usa `ActionClient(NavigateToPose, 'navigate_to_pose')` |
| `/demo/navigation/status` | está na convenção do CLAUDE.md e **nenhum nó publica** — precisa ser criado no F4 |
| `docker/hmi/` | existe e está **vazio** |
| `hmi/` na raiz | **não existe** ainda (reservado em `.ai/AGENTS.md` §4) |
| `viz` já monta `../scripts:/cockpit:ro` | `docker/compose.host.yml`, serviço `viz` |
| Todos os serviços usam `network_mode: host` | `x-common` em `compose.host.yml` — portas **não** são mapeadas, bindam direto no host |
| Serviços existentes no host | `base`, `sim`, `nav`(perfil learn), `perception`(perfil learn), `viz`, `tools`(perfil tools) |
| `sim.launch.py` aceita `gui:=` | `sim.launch.py:95`, default `true` |
| Testes do cockpit | `tests/test_cockpit_desktop_safety.py`, 5 testes (nomes na §6, F5) |

---

## 4. Arquitetura aprovada

Tudo abaixo é **host x86**, exceto onde marcado. Nav2 e percepção continuam no
AM69 sem alteração.

```text
  ┌─ host x86 ───────────────────────────────────┐    ┌─ Aquila AM69 ─┐
  │  sim (Gazebo, gui:=false)  ──DDS──┐          │    │  nav (Nav2)   │
  │  viz:                             ├ rosbridge┼DDS─┤  perception   │
  │    web_video_server               │   :9090  │    └───────────────┘
  │    demo_hmi (relay de metas + status)        │
  │  hmi: nginx  ── bundle estático              │
  └──────────────────────────────────────────────┘
                    │
              Chromium (host hoje, kiosk no AM69 depois — M3)
```

Portas sugeridas: rosbridge 9090, `web_video_server` 8080, nginx 8081.
Como `network_mode: host`, elas bindam direto — **devem ser configuráveis por
`.env`** para não colidir com nada do operador. Sem IP hard-coded (CLAUDE.md).

Mapeamento dos cinco painéis da imagem de referência
(`docs/ml35/cockpit-division-view.png`, o contrato visual do layout):

| Painel | Fonte | Transporte |
| --- | --- | --- |
| **GAZEBO SIMU VIEW** (azul) | câmera de cena do mundo SDF → `/demo/cockpit/scene/image_raw` | `<img>` MJPEG |
| **RVIZ** (verde) | mapa, footprint, `/demo/scan`, plano global, meta, detecções | `<canvas>` 2D via rosbridge |
| **RVIZ CAMERA** (rosa claro) | `/demo/camera/image_raw` + `/demo/perception/detections` | `<img>` MJPEG + overlay |
| **LOGS DE MOVIMENTAÇÃO** (rosa) | `/rosout`, `/demo/cmd_vel`, `/demo/odom`, status Nav2 | rosbridge WebSocket |
| **BARRA DE CONTROLES** (cinza) | teleop → `twist_mux` → `/demo/cmd_vel`; metas → ação | rosbridge WebSocket |

---

## 5. O que NÃO muda

Intocados: Nav2, percepção, contrato de tópicos, `sim.launch.py`,
`compose.module.yml`, `go2_description` (vendorizado) e todo o ML3.5 F1–F6.

As adições ficam em lugares que a árvore já reservava: `hmi/`,
`ros2_ws/src/demo_hmi/`, `docker/hmi/Dockerfile` e serviços novos em
`docker/compose.host.yml`.

---

## 6. Fases restantes, com portões

### F1 — Esqueleto do cockpit e transporte (risco baixo)

Serviços `rosbridge` e `web_video_server`; serviço `hmi` (`nginx:alpine`,
multi-arch); bundle `hmi/` em ES modules com cliente rosbridge próprio
(Decisão 5); grid CSS reproduzindo a imagem; estado de conexão, dado obsoleto
e reconexão (§5.7 do AGENTS); host/porta por configuração.

> **Portão:** as cinco regiões aparecem no DP-1 na proporção da imagem; o painel
> de câmera mostra `/demo/camera/image_raw` ao vivo; derrubar o rosbridge muda o
> estado visual para "desconectado" e ele reconecta sozinho.

**CONCLUÍDO em 24/08/2026, modo learn na workstation.** Portão atendido nos três
itens, com capturas de tela e logs em `docs/results/cockpit-web-f1.md`.

O que ficou de pé: serviços `cockpit` (rosbridge 2.7.0 + web_video_server 3.1.0,
imagem própria multi-arch por construção) e `hmi` (`nginx:alpine`) em
`compose.host.yml`; `cockpit.launch.py` em `demo_bringup`; bundle em `hmi/` com
cliente rosbridge próprio, rastreamento de frescor never/live/stale, painel de
câmera e painel de logs funcionais; 52 testes de bundle sob `node --test` e 7
guardas estruturais sob pytest.

Duas coisas custaram tempo e estão travadas por teste — ver o documento de
evidência: `web_video_server` não faz percent-decode de `topic`, e um `<img>`
com MJPEG não dispara `load` no Firefox.

### F3b — Painéis azul e verde por dados

- **Azul:** duas câmeras estáticas (isométrica e topo) nos `worlds/*.sdf`
  (Decisão 6), bridged para `/demo/cockpit/scene/image_raw`; Gazebo com
  `gui:=false`.
- **Verde:** vista de navegação 2D em `<canvas>` — mapa, footprint,
  `/demo/scan`, plano global, meta, detecções. Clique no mapa → meta.

> **Portão:** os dois painéis atualizam ao vivo com o robô andando; o clique no
> painel verde gera uma meta que o Nav2 aceita.

### F4 — Controle manual com árbitro (fecha o débito conhecido)

Pacote `demo_hmi` (`ament_python`): nó de teleop assinando comandos do
rosbridge; `twist_mux` (Decisão 7); relay `/goal_pose` → ação `NavigateToPose`;
publicação de `/demo/navigation/status` (que hoje não existe).

Semântica preservada de `scripts/cockpit_teleop.py` — deadman/watchdog de
400 ms, parada em release/perda de foco/EOF, três zeros no shutdown — e a
tabela de velocidades já definida:

| Ação | linear.x | angular.z |
| --- | ---: | ---: |
| frente | +0,25 | 0 |
| ré | −0,20 | 0 |
| esquerda | 0 | +0,20 |
| direita | 0 | −0,20 |

> **Portão:** o robô **move de fato** pelos botões — este é o débito honesto do
> checkpoint anterior, que nunca foi ensaiado; com Nav2 navegando, tocar no
> teleop assume o controle e soltar devolve; E-STOP para em <200 ms.

### F5 — Retirar o caminho morto

Remover `scripts/cockpit.py` (PyQt5/Xlib, 672 linhas) e
`scripts/cockpit_teleop.py`; retargetar `scripts/run_cockpit.sh` (a lógica de
lifecycle, `stop_all`, `show_status` e `cleanup` é boa e se aproveita).

Destino dos 5 testes de `tests/test_cockpit_desktop_safety.py`:

| Teste | Destino |
| --- | --- |
| `test_launcher_cannot_manage_the_desktop_session` | **mantém** — a proibição de mexer no GNOME continua valendo |
| `test_manual_control_has_deadman_and_zero_paths` | **mantém**, reapontado para `demo_hmi` |
| `test_standalone_window_embeds_all_three_clients` | **substituir** — vira teste de composição de painéis |
| `test_controller_is_a_normal_top_level_window` | **remover** — não há mais janela Qt |
| `test_window_selection_ignores_qt_helpers` | **remover** — não há mais seleção de janela X11 |

### F6 — Evidência e documentação

`docs/results/cockpit-web.md` com FPS medidos, latência de teleop e
screenshots; substitui `cockpit-standalone-parcial.md` como estado corrente.
Atualizar `.ai/CLAUDE.md` "Onde estamos", `.ai/changelog.md` e
`docs/ml35/estado-fases.md`.

**Fora de escopo:** kiosk Chromium acelerado no AM69. Isso é o M3 e só pode ser
validado no hardware real. Não afirmar aceleração de GPU no módulo a partir
deste trabalho (regra 7 do CLAUDE.md).

---

## 7. Riscos registrados

| Risco | Prob. | Mitigação |
| --- | --- | --- |
| `/map` grande satura o rosbridge em JSON | Média | compressão PNG do rosbridge; se falhar, mapa vira stream do `web_video_server` |
| Suporte a ações ROS 2 no rosbridge 2.x não confirmado | Média | o relay em `demo_hmi` (F4) não depende disso — é plano B embutido |
| Câmeras estáticas não enquadram bem o maze11 (11,6 × 11,6 m) | Média | duas poses alternáveis; ajustar contra a pegada medida no cabeçalho do mundo |
| Painéis sem interatividade não convencem na demo | Média | **custo já aceito na Decisão 4** |
| Latência do enlace Ethernet HIL no teleop | Média | medir; a F5 do ML3.5 já mostra 8 m estourando o protocolo — não misturar os dois problemas |
| Portas 8080/9090 colidirem no host | Baixa | configuráveis por `.env`; `network_mode: host` não isola |

---

## 8. Pontos abertos, a confirmar na implementação

1. ~~`rosbridge_suite` 2.x expõe ações ROS 2 (`send_action_goal`)?~~
   **RESPONDIDO no F1: sim.** A versão 2.7.0 registra `SendActionGoal`,
   `ActionFeedback`, `ActionResult` e `AdvertiseAction` no startup. O relay do
   F4 pode encolher — mas confirmar com uma meta longa de verdade antes de
   apagar o plano B: registrar capacidade não é entregar feedback.
2. Compressão PNG do rosbridge é suficiente para o `/map` do maze11?
   (ainda aberto; `png_compression: true` já está ligado em `cockpit.launch.py`)
3. ~~`web_video_server` aceita QoS reliable no `/demo/camera/image_raw`?~~
   **RESPONDIDO no F1: sim, sem reconfiguração.** 6,4 MB em 6 s de MJPEG e
   `/snapshot` devolvendo JPEG 640x480.
4. Enquadramento das duas câmeras estáticas contra a pegada de 11,6 m.
5. **Novo:** o bundle foi verificado no Firefox. O kiosk do M3 é Chromium; a
   ressalva de cache do `<img>` em `config.js` vem da literatura e não foi
   medida.

---

## 9. Onde retomar

F1 e **F3b** estão fechados (24/08/2026). Evidência do F3b, incluindo o controle
de simulação, o controle de câmera, a identidade Toradex e os números de
qualidade de imagem: **`docs/results/cockpit-web-f3b.md`**. Guia operacional
(rodar, painéis, controles, cenários, armadilhas): **`docs/guia-completo.md`**
(Parte II).

O que existe hoje, na tela:

| Região | Fonte | Controles |
| --- | --- | --- |
| azul, cena | duas câmeras estáticas do mundo | iso/topo; girar, inclinar, mover, zoom, recentrar |
| verde, navegação | costmap, plano, laser, pegada, TF | clique manda meta; cancelar meta |
| rosa, logs | `/rosout` + telemetria de `cmd_vel`/odom | — |
| rosa claro, câmera | `/demo/camera/image_raw` | — |
| barra | estado do link | play/pause/reset da simulação |

### Próximo passo: F4 — controle manual

É a única região da tela que ainda mente: os botões de seta e o E-STOP estão
desenhados, alcançáveis por teclado, e **desligados**, com o motivo no `title`.
Hoje o Nav2 é o único publicador em `/demo/cmd_vel`; um botão de teleop que
também publicasse ali daria dois escritores não arbitrados no mesmo tópico, com
o último a escrever ganhando e nenhum dos dois sabendo que perdeu. Fecha com
`twist_mux` (Decisão 7), não com um mux escrito à mão no navegador.

### Depois: F2 — kiosk no módulo

Chromium em modo kiosk no Aquila, com aceleração de GPU, servindo este mesmo
bundle. Três coisas ainda não medidas e que só o módulo responde:

1. o bundle foi verificado no **Firefox**; o kiosk é Chromium, e a ressalva de
   cache do `<img>` em `config.js` vem da literatura, não de medição;
2. o MJPEG das câmeras de cena a 1600x1200 atravessando a Ethernet — o
   parâmetro a baixar primeiro é a qualidade JPEG em `hmi/js/config.js`, que
   degrada suavemente, e não a resolução do sensor, que desloca o
   enquadramento em pixels;
3. os serviços de simulação **não** existem no modo `deploy`: sem Gazebo, os
   botões de play/pause/reset precisam sumir ou dizer por que não valem. Isso
   ainda não foi tratado.

### Restrições que não mudam

- **O simulador nunca vai para o módulo.** O Gazebo é OGRE 2 e o AM69 só expõe
  OpenGL ES 3.2 e Vulkan 1.2 (regra 1). "Controlar a simulação pelo cockpit"
  é suportado; "rodar a simulação no módulo" não é, e nenhuma quantidade de
  código na UI muda isso.
- **O navegador não fala tipos do Gazebo.** O rosbridge monta o pedido
  importando o pacote de interfaces dentro do container do cockpit, que não tem
  `ros_gz_interfaces` — e no modo `deploy` nem faria sentido ter. A fronteira é
  `std_srvs`; a tradução mora no `sim_control_relay`, do lado do simulador.
- **`go2_description` é vendorizado e não se toca**, nem para pendurar uma
  câmera no tronco.

Antes de escrever qualquer coisa, reler `.ai/AGENTS.md` §5.7 (requisitos do HMI)
e §4 (árvore reservada), e `docs/results/cockpit-web-f3b.md` (as armadilhas
medidas).
