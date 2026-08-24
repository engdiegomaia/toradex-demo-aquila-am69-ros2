# Cockpit standalone — checkpoint parcial

**Data:** 24/08/2026
**Host:** Ubuntu 24.04, X11, GNOME/Mutter, dois monitores
**Tela:** DP-1, 2560×1080
**Modo:** HIL; Gazebo/RViz/câmera no host, Nav2 e percepção no Aquila AM69
**Estado:** **NÃO ACEITO — as três visualizações ainda não foram integradas**

## Objetivo correto

Uma única aplicação standalone deve conter Gazebo, RViz, a imagem de
/demo/camera/image_raw, controles manuais do Go2 e lifecycle coordenado com a
stack HIL. Não basta alinhar três janelas independentes.

O GNOME deve permanecer ativo. Reiniciar, substituir ou desabilitar o compositor
está fora do procedimento: a tentativa de gnome-shell --replace interrompeu a
sessão e outros processos do operador.

## Orquestração implementada

scripts/run_cockpit.sh valida X11/EWMH, PyQt5, Docker e modelos; sobe Nav2 e
percepção no Aquila; sobe aquila-go2, RViz e rqt_image_view no host; abre o
cockpit na tela solicitada; e executa cleanup ao fechar. Uma instância residual
do simulador é recusada para evitar duas plantas.

## Metodologia e tentativas

### 1. Barra e posicionamento EWMH — rejeitado por requisito

A primeira interpretação criou uma barra de 48 px e posicionou Gazebo, RViz e
câmera como janelas top-level no DP-1. A disposição funcionou, mas não era um
cockpit standalone.

A barra usou primeiro Qt.Tool. O Mutter a marcou como Withdrawn/UnMapped após
troca de foco. Qt.Window corrigiu somente a barra; não resolveu o requisito.

### 2. QWindow.fromWinId/createWindowContainer — parcial e rejeitado

Foi criada uma janela Qt com Gazebo à esquerda, RViz e câmera à direita e
controles embaixo. Resultado observado:

- Gazebo renderizou dentro da janela;
- RViz e câmera permaneceram independentes;
- seus painéis internos ficaram pretos;
- a UI mostrou falsamente “3/3”, pois contou wrappers Qt, não ReparentNotify.

### 3. Anexação atômica — não resolveu

Anexar Gazebo antes dos demais fazia xwininfo percorrer a árvore OpenGL
aninhada e exceder o timeout. A lógica passou a esperar os três IDs e anexar num
callback. A corrida sumiu, mas RViz e câmera continuaram externos.

### 4. Descoberta por _NET_CLIENT_LIST — diagnóstico melhor, mesma falha

A descoberta foi trocada por:

    xprop -root _NET_CLIENT_LIST
    xprop -id <id> _NET_WM_NAME WM_NAME WM_CLASS

IDs atuais no último ensaio:

| Visualização | ID | WM_CLASS |
| --- | --- | --- |
| Gazebo | 0x4c0001a | gz-sim-gui, Gazebo GUI |
| RViz | 0x4400015 | rviz2, rviz2 |
| câmera | 0x4a00006 | rqt_image_view, rqt_image_view |
| cockpit | 0x4000006 | cockpit.py, Aquila Cockpit |

xwininfo -id 0x4000006 -tree mostrou somente Gazebo como filho. RViz e câmera
continuaram sob frames mutter-x11-frames próprios.

### 5. Reparenting Xlib direto — implementado, ainda não executado

O código mais recente substitui wrappers Qt por X11Embedder, usando
XReparentWindow, XMapWindow, XMoveResizeWindow e XSync em superfícies QWidget
nativas.

Essa versão **não foi executada**: o operador solicitou interromper e registrar
o checkpoint logo após a edição. Não deve ser considerada solução.

## Controle manual implementado

scripts/cockpit_teleop.py roda dentro do container viz. O cockpit mantém um
docker compose exec -T persistente e envia:

    CMD <linear.x> <linear.y> <angular.z>
    STOP
    QUIT

O helper publica geometry_msgs/Twist em /demo/cmd_vel a 10 Hz. Há parada em
release, perda de foco, timeout de 400 ms, EOF, QUIT e fechamento; no shutdown
são publicados três zeros.

| Ação | linear.x | angular.z |
| --- | ---: | ---: |
| frente | +0,25 | 0 |
| ré | −0,20 | 0 |
| esquerda | 0 | +0,20 |
| direita | 0 | −0,20 |

A ponte iniciou no container e o RMW iniciou. Movimento real pelos botões **não
foi ensaiado**.

**Limitação:** controle manual e Nav2 podem publicar no mesmo /demo/cmd_vel.
Ainda não existe mux/arbitragem. Antes da aceitação, inserir prioridade manual e
timeout explícito ou controlar por ações Nav2.

## Validações executadas

    pytest -q tests/test_cockpit_desktop_safety.py  # 5 passed
    python3 scripts/cockpit.py --self-test          # OK
    python3 -m py_compile scripts/cockpit.py scripts/cockpit_teleop.py
    bash -n scripts/run_cockpit.sh
    docker compose -f docker/compose.host.yml config
    git diff --check

Testes ROS direcionados também passaram: demo_bringup 46/46,
demo_simulation 23/23 e demo_perception 18/18. A primeira execução de
demo_perception falhou porque o sandbox não permitiu escrever em ~/.ros/log; a
repetição com ROS_LOG_DIR=/tmp/aquila-ros-test-logs passou integralmente.

flake8 não estava instalado e foi SKIPPED. Importar o helper no Python puro do
host falhou com ModuleNotFoundError: geometry_msgs, esperado fora do ambiente
ROS. Dentro do viz, o helper montou e iniciou.

## Estado exato para continuidade

- Launcher e cleanup são utilizáveis para experimentação.
- A janela e o layout standalone existem.
- Gazebo foi incorporado com a API Qt anterior.
- RViz e câmera **não** foram incorporados.
- O caminho Xlib direto está no código, mas **não foi executado**.
- O controle possui deadman/watchdog, mas não moveu o robô em teste.
- O milestone permanece aberto.

## Próxima metodologia recomendada

### Preferido: UI Qt/ROS nativa, sem capturar janelas externas

1. Incorporar RViz por rviz_common::RenderPanel e VisualizationManager.
2. Assinar sensor_msgs/Image e renderizar diretamente num widget Qt.
3. Avaliar plugin gz-gui próprio ou cena Gazebo Transport/rendering dedicada.
4. Adicionar mux ROS explícito entre Nav2 e teleop.
5. Manter o launcher somente como orquestrador.

Isso remove dependência de frames Mutter, IDs X11 e reparenting de clientes Qt.

### Se continuar com X11

1. Testar X11Embedder com duas aplicações Qt mínimas, fora do ROS.
2. Verificar ReparentNotify, pai real e Map State após cada operação.
3. Só contar um painel depois de confirmar o pai X11.
4. Restaurar clientes para o root antes de fechar.
5. Testar foco, teclado, OpenGL, resize e encerramento.
6. Repetir com Gazebo, RViz e câmera, uma variável por ensaio.
7. Nunca reiniciar ou substituir o GNOME.
