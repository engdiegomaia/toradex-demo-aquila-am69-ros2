## 1. Entrar no repositório

  cd /home/diego-maia/toradex/demo/aquila-am69-ros2

  ## 2. Preparação inicial

  Necessário na primeira execução ou após mudanças relevantes em ros2_ws/src:

  scripts/module.sh sync
  scripts/module.sh build

  Construa as imagens do host separadamente:

  BUILDX_BUILDER=default docker compose \
    -f docker/compose.host.yml build base

  BUILDX_BUILDER=default docker compose \
    -f docker/compose.host.yml build sim cockpit hmi

  Autorize o Gazebo a usar o X11:

  xhost +local:docker

  Esses builds não precisam ser repetidos em toda execução.

  ## 3. Verificar stacks antigas

  docker ps -a --format '{{.Names}}\t{{.Status}}' | grep -v '^docker-'

  Se houver outro projeto Compose executando sim, cockpit ou hmi, encerre-o pelo nome correto antes de prosseguir:

  docker compose -p NOME_DO_PROJETO \
    -f docker/compose.host.yml down

  ## 4. Subir o host

  Para uma partida limpa, suba primeiro Gazebo, cockpit e HMI. Assim o simulador nasce com o robô no spawn antes de o SLAM receber sua primeira varredura:

  docker compose -f docker/compose.host.yml \
    up -d sim cockpit hmi

  O mundo padrão já é quadruped_maze11.sdf; não é necessário definir SIM_ARGS.

  Confira:

  docker compose -f docker/compose.host.yml ps

  Acompanhe a inicialização do simulador, se necessário:

  docker compose -f docker/compose.host.yml \
    logs --tail=100 sim

  ## 5. Subir Nav2 e percepção na Aquila

  scripts/module.sh up
  scripts/module.sh verify

  O verify deve confirmar comunicação DDS e Nav2 ativo. Se o robô estiver longe do spawn, module.sh up recusará corretamente a operação.

  ## 6. Abrir o cockpit web

  No navegador do host:

  http://localhost:8081

  De outra máquina na mesma rede:

  http://192.0.2.6:8081

  O navegador conversa com:

  - HMI/nginx na porta 8081;
  - web_video_server na porta 8080;
  - rosbridge WebSocket na porta 9090.

  ## 7. Verificação rápida

  source scripts/env.sh
  ros2 topic list
  ros2 topic hz /demo/odom

  Confira também:

  scripts/module.sh status

  No cockpit, aguarde:

  - vídeo disponível;
  - robô em pé;
  - mapa aparecendo;
  - Nav2 ativo;
  - explorer em idle.

  ## 8. Iniciar a exploração

  Use o botão de início da exploração no cockpit web.

  Alternativamente:

  source scripts/env.sh
  ros2 service call /demo/exploration/start \
    std_srvs/srv/Trigger '{}'

  Durante a rodada:

  - inicie somente uma vez;
  - não envie metas manuais;
  - não use teleop;
  - interrompa se houver risco de queda.

  Para cancelar:

  ros2 service call /demo/exploration/cancel \
    std_srvs/srv/Trigger '{}'

  ## 9. Reiniciar uma rodada sem derrubar tudo

  Primeiro reposicione o robô:

  source scripts/env.sh
  ros2 service call /demo/sim/reset \
    std_srvs/srv/Trigger '{}'

  Confirme odometria próxima de (0,0):

  ros2 topic echo /demo/odom --once

  Somente depois recrie Nav2 e percepção:

  scripts/module.sh up
  scripts/module.sh verify

  Nunca execute module.sh up com o robô longe do spawn para depois chamar reset; isso corrompe a âncora inicial do mapa.

  ## 10. Encerrar

  docker compose -f docker/compose.host.yml down
  scripts/module.sh down

  O fluxo correto, resumido, é:

  host: sim + cockpit + hmi
            ↓
  Aquila: nav + perception
            ↓
  browser: http://localhost:8081
            ↓
  iniciar exploração pelo cockpit web
