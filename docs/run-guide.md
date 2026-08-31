# Guia rápido de execução da demo

Este checklist cobre a demo HIL: Gazebo, cockpit e HMI no host x86; Nav2 e
percepção na Aquila AM69. Para instalação e diagnóstico detalhados, consulte o
[`guia-completo.md`](guia-completo.md).

## 1. Entrar no repositório

```bash
cd <caminho-do-repositorio>/aquila-am69-ros2
```

## 2. Preparação inicial

Na primeira execução, copie e preencha a configuração local. Endereços de
rede e dados da bancada devem permanecer apenas nesse arquivo ignorado pelo
Git.

```bash
cp docker/.env.example docker/.env
${EDITOR:-vi} docker/.env
```

Sincronize e construa o software do módulo após mudanças em `ros2_ws/src`:

```bash
scripts/module.sh sync
scripts/module.sh build
```

Construa as imagens do host separadamente:

```bash
BUILDX_BUILDER=default docker compose \
  -f docker/compose.host.yml build base

BUILDX_BUILDER=default docker compose \
  -f docker/compose.host.yml build sim cockpit hmi
```

Se a interface gráfica do Gazebo for usada, autorize o X11:

```bash
xhost +local:docker
```

Esses builds não precisam ser repetidos em toda execução.

## 3. Verificar stacks antigas

```bash
docker ps -a --format '{{.Names}}\t{{.Status}}' | grep -v '^docker-'
```

Se houver outro projeto Compose executando `sim`, `cockpit` ou `hmi`, encerre-o
pelo nome correto antes de prosseguir:

```bash
docker compose -p NOME_DO_PROJETO \
  -f docker/compose.host.yml down
```

## 4. Subir o host

Para uma partida limpa, suba primeiro Gazebo, cockpit e HMI. Assim o simulador
nasce com o robô no spawn antes de o SLAM receber sua primeira varredura.

```bash
docker compose -f docker/compose.host.yml up -d sim cockpit hmi
docker compose -f docker/compose.host.yml ps
```

O mundo padrão já é `quadruped_maze11.sdf`; não é necessário definir
`SIM_ARGS`. Para acompanhar a inicialização:

```bash
docker compose -f docker/compose.host.yml logs --tail=100 sim
```

## 5. Subir Nav2 e percepção na Aquila

```bash
scripts/module.sh up
scripts/module.sh verify
```

O `verify` deve confirmar a comunicação DDS e o Nav2 ativo. Se o robô estiver
longe do spawn, `module.sh up` recusará corretamente a operação.

## 6. Abrir o cockpit web

No navegador do host, abra:

```text
http://localhost:8081
```

De outra máquina na mesma rede, use o endereço do host definido localmente:

```text
http://<HOST_IP>:8081
```

O navegador usa HMI/nginx na porta 8081, `web_video_server` na 8080 e o
WebSocket do rosbridge na 9090.

## 7. Verificação rápida

```bash
source scripts/env.sh
ros2 topic list
ros2 topic hz /demo/odom
scripts/module.sh status
```

No cockpit, aguarde vídeo disponível, robô em pé, mapa aparecendo, Nav2 ativo
e explorador em `idle`.

## 8. Iniciar a exploração

Use o botão de início da exploração no cockpit. Como alternativa:

```bash
source scripts/env.sh
ros2 service call /demo/exploration/start std_srvs/srv/Trigger '{}'
```

Durante a rodada, inicie somente uma vez, não envie metas manuais nem use
teleop. Esta entrega é uma demo supervisionada: interrompa imediatamente se
houver risco de queda ou se o estado entrar em `homing_exit`.

```bash
ros2 service call /demo/exploration/cancel std_srvs/srv/Trigger '{}'
```

## 9. Reiniciar uma rodada sem derrubar tudo

Primeiro reposicione o robô:

```bash
source scripts/env.sh
ros2 service call /demo/sim/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /demo/odom --once
```

Confirme a odometria próxima de `(0, 0)` e somente depois recrie Nav2 e
percepção:

```bash
scripts/module.sh up
scripts/module.sh verify
```

Nunca execute `module.sh up` com o robô longe do spawn para depois chamar
`reset`; isso corrompe a âncora inicial do mapa.

## 10. Encerrar

```bash
scripts/module.sh down
docker compose -f docker/compose.host.yml down
```

Fluxo resumido: host (`sim` + `cockpit` + `hmi`) → Aquila (`nav` +
`perception`) → navegador (`http://localhost:8081`) → iniciar exploração pelo
cockpit.
