# Preparação do target — Aquila AM69, primeiro acesso real

Módulo `<MODULE_HOST>`, 2026-08-20. Primeira sessão em que o
hardware esteve acessível. Tudo abaixo foi **medido no módulo**, não inferido.

Contexto: executado em paralelo aos ensaios de marcha do Go2 no host x86. Essa
concorrência determinou três decisões — build nativo no módulo em vez de QEMU, o
guard do `up` contra colisão em `/demo/cmd_vel`, e não tocar em
`scripts/run_quadruped_sim.sh` nem em `scripts/env.sh`, ambos em uso.

---

## Inventário do módulo (medido)

| Item | Valor |
| --- | --- |
| OS | Torizon OS 7.7.0+build.40 (scarthgap) |
| Deploy ostree | `7.7.0+build.40-tcbuilder.20260716014324` |
| Kernel | 6.6.142-7.7.0 `#1-Torizon SMP PREEMPT` |
| Arquitetura | aarch64 |
| CPU | 8 × Cortex-A72 |
| RAM | 31 GiB (690 MiB em uso) |
| Disco | 115 G em `/dev/disk/by-label/otaroot`, **108 G livres** |
| Docker | 25.0.9, engine arm64, overlay2, cgroup v2 (systemd) |
| Compose | v2.26.0 |
| Usuário | `torizon`, uid 1000, **no grupo `docker`** (990) |

Interfaces com endereço:

| Link | Estado | Endereço |
| --- | --- | --- |
| `ethernet0` | UP | `<MODULE_IP>/24` + IPv6 global |
| `br-a00dfb945795` | UP | `172.18.0.1/16` (bridge do compose do easy-pairing) |
| `docker0` | DOWN | `172.17.0.1/16` |
| `wlan0`, `ethernet1`, `can0-3` | DOWN | — |

Host x86: `<HOST_IP>` em `wlp0s20f3` (Wi-Fi). Mesma /24 do módulo, rota
direta (`ip route get <MODULE_IP>` → `src <HOST_IP>`).

Estado inicial dos containers: apenas `torizon-easy-pairing-bash-1` (Torizon
Cloud). Nenhuma imagem `demo-aquila-*`. `~/` vazio.

**Nota sobre a regra 3 do `CLAUDE.md`:** o módulo já está em 7.7.0, acima do
7.4.0 que a regra discute. Nenhum caminho de upgrade foi tocado nesta sessão.

---

## Rede DDS: medida, não assumida

`ufw` está **ativo** no host, o que normalmente é o primeiro suspeito quando a
descoberta falha. Foi testado antes de qualquer configuração de DDS, com
datagramas UDP reais nas portas que o domínio 69 usa
(`7400 + 250×69 + d` → 24650+):

| Direção | Porta UDP | Resultado |
| --- | --- | --- |
| módulo → host | 24660 | **recebido** (de `<MODULE_IP>`) |
| módulo → host | 24678 | **recebido** |
| host → módulo | 24661 | **recebido** (de `<HOST_IP>`) |

Conclusão: **nenhuma mudança de firewall é necessária.** Duas portas distintas
foram testadas de propósito — uma única porta passando não distingue "política
permissiva" de "regra pontual".

Este teste fica como etapa 1 de 3 do `scripts/module.sh verify`, e nessa ordem,
porque depurar descoberta de DDS sem antes provar alcance UDP é como se
diagnostica um firewall como bug de DDS.

---

## Duas correções necessárias antes de qualquer build

### 1. A camada de container é de F1; o quadrúpede chegou em F3

`docker/base/Dockerfile` faz `COPY ros2_ws/src` e um `colcon build` do
workspace inteiro. Quando foi escrito, a árvore era diff-drive. F3 adicionou
`gz_quadruped_hardware`, que declara:

```xml
<depend>gz_sim_vendor</depend>
<depend>gz_plugin_vendor</depend>
```

`<depend>`, não `<exec_depend>`: o `rosdep` resolve isso em tempo de build e
instala a stack Gazebo inteira. O efeito é **OGRE 2 dentro das imagens arm64 que
vão para o AM69** — regra 1 do `CLAUDE.md`. E é silencioso: o build passa, a
imagem cresce, e a violação só aparece em runtime no módulo, como uma biblioteca
querendo um OpenGL de desktop que não existe lá.

Correção, em `base/` e replicada em `nav/`, `perception/`, `tools/` (as três
refazem o `colcon build` do workspace todo, então nenhuma delas passa sem isso):

```dockerfile
ARG SKIP_KEYS_EXTRA=""          # módulo: "gz_sim_vendor gz_plugin_vendor"
ARG COLCON_IGNORE_PACKAGES=""   # módulo: "gz_quadruped_hardware"
```

Os dois são necessários e nenhum basta sozinho: `--skip-keys` impede o `rosdep`
de **instalar** Gazebo, `--packages-ignore` impede o `colcon` de colocar o
pacote no grafo do workspace. Só o primeiro quebra o build; só o segundo ainda
embarca Gazebo. Ambos default vazio, então o lado amd64/host não muda.

`unitree_guide_controller` **é** compilado no módulo (precisa apenas de
`controller_interface` e `kdl_parser`, nada de Gazebo), porque no modo `deploy`
o controlador de pernas roda no módulo.

#### `--packages-skip` foi tentado primeiro e falhou

A primeira versão usava `--packages-skip`, com a expectativa registrada de que
`demo_simulation` compilaria normalmente porque sua dependência de
`gz_quadruped_hardware` é `exec_depend`, não `build_depend`. **A expectativa
estava errada.** O build falhou:

```
ERROR:colcon.colcon_ros.task.ament_python.build:Failed to find the following files:
- /ws/install/gz_quadruped_hardware/share/gz_quadruped_hardware/package.sh
Check that the following packages have been built:
- gz_quadruped_hardware
Failed   <<< demo_simulation [0.01s, exited with code 1]
Summary: 8 packages finished, 1 package failed, 1 package not processed
```

O grafo de dependências do `colcon` **não distingue `exec` de `build`**.
`--packages-skip` mantém o pacote no grafo sem construí-lo, e a geração de
environment hooks do `ament_python` para `demo_simulation` então exige um
artefato em `install/` que nunca foi produzido.

O custo real não é `demo_simulation`: é o "1 package not processed", que era
**`demo_bringup`** — onde vivem `nav.launch.py` e `perception.launch.py`, os
entrypoints dos dois serviços do módulo. Sem `demo_bringup` não há stack no
módulo.

`--packages-ignore` remove o pacote do workspace inteiramente, então nada
depende dele. Essa é a flag correta, e a distinção entre as duas não aparece em
lugar nenhum da mensagem de erro.

#### O que sobrou de `gz-*` na imagem, e por que não é violação

Medido na lista de instalação do `apt`: **42,1 MB** no total, e os únicos
pacotes `gz-*` são

```
ros-jazzy-gz-cmake-vendor  ros-jazzy-gz-math-vendor
ros-jazzy-gz-tools-vendor  ros-jazzy-gz-utils-vendor
```

puxados via `ros-jazzy-sdformat-vendor` / `sdformat-urdf`. São matemática,
headers e build tooling — CPU puro, zero contato com GPU. Nenhum `gz-sim`,
`gz-rendering`, `gz-gui`, `libogre` ou `rviz`. Comparação: o `nav2-bringup`
recusado em F1 levava a imagem a 3,7 GB.

A verificação da regra 1 no `scripts/module.sh build` casa exatamente com o
conjunto de renderização e nada além. Um `grep gz` genérico sinalizaria esses
quatro vendors e mandaria a próxima pessoa caçar uma violação inexistente; um
`grep gazebo` **perderia** `gz-sim`, porque o Harmonic abandonou o nome.

#### O guard de regra 1 acusou uma violação que não existia

Primeira versão do padrão: `grep -iE 'ogre|gz-rendering|gz-sim|gz-gui|rviz'`.
Resultado na `demo-aquila-nav`:

```
REGRA 1 VIOLADA: demo-aquila-nav contem:
    libnav2_progress_checker_selector_bt_node.so
    libpose_progress_checker.so
    libsimple_progress_checker.so
```

`pr`**`ogre`**`ss`. Três progress checkers do Nav2, sem OpenGL em lugar nenhum.

O OGRE real se chama `libOgreMain` / `libOgreOverlay` (O maiúsculo) ou
`libogre-next-*`, então a metade OGRE do padrão é **case-sensitive** e a metade
`ogre-next` é ancorada no hífen. `gz-sim` e `gz-gui` levam dígito
(`libgz-sim8.so`) para não casarem com `gz-math` nem `gz-utils`. Padrão final,
validado contra 8 nomes verdadeiros e 7 benignos:

```
libOgre|ogre-next|gz-rendering|gz-sim[0-9]|gz-gui[0-9]|rviz
```

Um guard que grita errado é pior que nenhum guard: ensina a ignorá-lo.

### 2. `autodetermine` no módulo escolhe a bridge do Docker

`docker/cyclonedds/module.xml` usava `<NetworkInterface autodetermine="true"/>`
com um comentário pedindo verificação por `ip -br link` antes do primeiro run.
A verificação foi feita: o módulo tem `ethernet0` **e** `br-a00dfb945795`
(172.18.0.1) UP ao mesmo tempo, porque a própria stack de easy-pairing da
Toradex roda em compose. `autodetermine` ranqueia interfaces e pode escolher a
bridge, e aí o CycloneDDS transmite num endereço que o host não roteia.

A interface agora é **fixada na renderização**, detectada no módulo a partir de
`MODULE_IP` em vez de escrita à mão — um `ethernet0` hard-coded sobreviveria à
troca de placa ou à mudança para `wlan0` e falharia calado.

---

## Configuração renderizada em tempo de deploy

Nenhum endereço entra no git (convenção do `CLAUDE.md`). `scripts/module.sh
sync` renderiza `docker/cyclonedds/module.xml` e escreve o resultado em
`~/demo/cyclonedds/module.xml` no módulo:

```
<NetworkInterface name="ethernet0" priority="default"/>  <!-- fixado por scripts/module.sh -->
<Peer address="127.0.0.1"/>
<Peer address="<HOST_IP>"/>  <!-- host x86, injetado por scripts/module.sh -->
```

O peer `127.0.0.1` é *load-bearing* e continua lá: com `AllowMulticast=false`,
`nav` e `perception` no módulo não se veem entre si sem ele (evidência em
`ml35-f1-execucao.md`).

`~/demo/.env` é gerado, não sincronizado: `ROS_DOMAIN_ID=69`,
`HOST_IP=<HOST_IP>`, `MODULE_IP=<MODULE_IP>`, `REGISTRY=local`,
`TAG=dev`.

### Uma armadilha encontrada na própria renderização

O primeiro renderizador casava `/<\/Peers>/` sem âncora. O bloco de comentário
do `module.xml` **menciona** o fechamento de `Peers` em prosa, então o peer do
host foi injetado no meio de um comentário, gerando XML malformado. Quem pegou
foi a validação de XML no fim da renderização — que por isso não é opcional.

Corrigido nas duas pontas: o `awk` ancora em linha inteira e injeta uma única
vez, e o comentário do `module.xml` foi reescrito para não conter as tags
literais, com um aviso dizendo por quê.

---

## Build nativo no módulo, não QEMU

`CLAUDE.md` documenta `docker buildx --platform linux/arm64` a partir do host.
`scripts/module.sh build` **não** usa esse caminho, por dois motivos medidos:

1. **QEMU arm64 não estava sequer habilitado neste host** — `binfmt_misc` não
   tem handler aarch64 registrado, e `docker buildx ls` reporta o builder
   `armbuilder` com plataformas `linux/amd64 (+3), linux/386` e nenhum arm64.
2. **O host é onde os ensaios de marcha rodam.** Aqueles ensaios medem *quando*
   o robô cai (o gatilho de B corrente cai aos 39,4 s). Um build QEMU satura a
   CPU do workstation e **corrompe a medição**, não apenas a atrasa. O módulo
   tem 8 A72 e 31 GiB ociosos.

O que se perde: `buildx` produz manifest multi-arch, o build nativo produz
imagem local só-arm64. Correto para bring-up, errado para distribuição — quando
entrar um registry, o caminho multi-arch do `CLAUDE.md` é o que vale. Nenhum dos
dois mede desempenho (regra 5).

---

## Guard do `up`: por que subir `nav` agora seria destrutivo

`compose.module.yml` sobe `nav`, e `nav` é o Nav2, que **publica**
`/demo/cmd_vel`. Com a simulação do Go2 rodando no host no mesmo
`ROS_DOMAIN_ID=69`, isso são dois publishers no tópico que comanda o robô.

O robô simulado passaria a receber comandos que ninguém enviou, e **nada em log
nenhum das duas máquinas diria por quê**: o tópico é válido, os dois publishers
estão saudáveis, e o DDS faz exatamente o que foi mandado. O ensaio de marcha em
curso seria silenciosamente corrompido, não interrompido.

`scripts/module.sh up` agora detecta simulação ativa no host (`docker ps` por
`aquila-go2|demo-aquila-sim|demo-sim`) e **recusa**, oferecendo quatro saídas:
esperar, subir só `perception` (não publica `cmd_vel`), usar domínio separado,
ou `--force`.

---

## `scripts/module.sh`

Subcomandos: `inventory`, `sync`, `build`, `up`, `down`, `status`, `verify`,
`shell`. Configuração por ambiente → `docker/.env` → defaults; nada hard-coded.

`verify` tem três etapas, em ordem, porque falham de formas diferentes:

1. **alcance UDP** na porta de descoberta do domínio — separa firewall de DDS;
2. **contrato de tópicos visto de dentro do módulo** — o módulo enxerga o host;
3. **módulo publica, host recebe** (`/demo/system/heartbeat`) — a direção que a
   demo realmente precisa.

A etapa 3 usa o `heartbeat_publisher` do `demo_tutorials` (ML1) de propósito, e
não `nav` ou `perception`: ele publica apenas `/demo/system/heartbeat`, que
nenhum outro nó do projeto consome. É a prova do caminho módulo→host **sem**
efeito colateral no experimento do host.

Detalhe que custa tempo se esquecido: o nó publica o tópico **relativo**
`system/heartbeat`, e `base/Dockerfile` deliberadamente não define
`ROS_NAMESPACE` (definiria e duplicaria o prefixo dos `/demo/*` absolutos em
todo o resto). A etapa 3 define `ROS_NAMESPACE=/demo` **só nesse comando**; sem
isso o tópico resolve para `/system/heartbeat` e o subscriber do host espera
para sempre por um nome que ninguém publica.

---

## Build: resultado

Quatro imagens `arm64`, construídas nativamente no módulo, `colcon` com **10
pacotes** cada (incluindo `demo_bringup`, que é o que importa):

| Imagem | Tamanho |
| --- | --- |
| `local/demo-aquila-base:dev` | 1,24 GB |
| `local/demo-aquila-perception:dev` | 1,28 GB |
| `local/demo-aquila-tools:dev` | 1,32 GB |
| `local/demo-aquila-nav:dev` | 2,44 GB |

Tempo de `colcon` no módulo: **2 min 34 s** para a `base` (o C++ de
`unitree_guide_controller` e `controller_common` domina), 18-19 s para cada
imagem de papel. Isso **não é uma medição de desempenho** (regra 5) — é o custo
de um build, e está aqui só para justificar por que QEMU foi descartado.

Verificação de regra 1 nas quatro imagens: `ok`, nenhuma stack de renderização.

---

## Três armadilhas encontradas na verificação, todas de falha silenciosa

Nenhuma destas apareceu como erro nomeando a causa. Duas fizeram o `verify`
**relatar falha de DDS que não existia**.

### 1. `docker compose exec` não roda o ENTRYPOINT da imagem

As etapas 2 e 3 do `verify` reportaram "nenhum tópico visível" e "host não
recebeu". Diagnóstico real:

```
$ docker compose exec -T tools bash -lc 'which ros2; echo PATH=$PATH'
RGS2_AUSENTE
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
```

`ros2` **não está no PATH**. O `entrypoint.sh` é que faz `source` do underlay e
do overlay `/ws/install`, e `docker compose exec` não o executa. Um `bash -lc`
não resolve: a imagem `ros` não coloca o setup no `.bashrc`.

O que transformou isso em falha silenciosa foi o meu próprio comando:

```bash
ros2 topic list 2>/dev/null | grep /demo/ || echo "(nenhum topico visivel)"
```

O `2>/dev/null` engoliu `command not found`, e o `||` imprimiu exatamente a
mesma linha que uma falha genuína de descoberta imprimiria. **Duas etapas de
verificação relataram um problema de DDS inexistente.**

→ Todo `exec` agora passa por `/usr/local/bin/entrypoint.sh`, e os `ros2` não
têm mais `2>/dev/null`. `exec -d` também ganhou uma confirmação de que o nó
subiu, porque `-d` esconde qualquer erro, inclusive `command not found`.

### 2. `ROS_NAMESPACE` não funciona no ROS 2 Jazzy

O `heartbeat_publisher` publica o tópico **relativo** `system/heartbeat`. Para
chegar em `/demo/system/heartbeat` foi usado `-e ROS_NAMESPACE=/demo`. O tópico
saiu `/system/heartbeat`.

Não é problema de passagem da variável — isso foi verificado:

```
$ docker compose exec -T -e ROS_NAMESPACE=/demo tools \
    /usr/local/bin/entrypoint.sh printenv ROS_NAMESPACE
/demo
$ ... ros2 run demo_tutorials heartbeat_publisher   → /system/heartbeat
$ ... ros2 run ... --ros-args -r __ns:=/demo        → /demo/system/heartbeat
```

A variável está no ambiente do processo e **o ROS 2 a ignora**.

→ Use `--ros-args -r __ns:=`. **Consequência para o projeto:**
`scripts/env.sh` faz `export ROS_NAMESPACE=/demo` com um comentário que assume
que funciona, e avisa para desligá-la ao rodar nós de terceiros. Esse aviso é
desnecessário e a linha não tem efeito. Não foi alterada nesta sessão — o
arquivo é usado pelos ensaios em curso no host —, fica registrado como achado.

### 3. Metade da config de DDS não liga nada, e falha igual a firewall

Com o módulo corretamente configurado e um publisher **comprovadamente rodando**
nele, o host não via nada. Duas causas simultâneas:

| Direção | Por que falhava |
| --- | --- |
| host → módulo | O default do CycloneDDS anuncia por **multicast**, e o módulo tem `AllowMulticast=false`. Nunca escuta. |
| módulo → host | O módulo manda SPDP unicast para as portas RTPS do host, mas um participante CycloneDDS default **não fixa porta determinística** — usa porta efêmera e conta com multicast para ser achado. Não há porta para mirar. |

O lado host precisa de config **casada**: `AllowMulticast=false`,
`ParticipantIndex=auto` e o módulo como `<Peer>`. `scripts/module.sh` agora
renderiza `docker/cyclonedds/host.rendered.xml` (interface detectada por
`HOST_IP`, peer do módulo injetado), gitignored pelo mesmo motivo do
`module.xml`.

---

## Link DDS host ↔ módulo: as duas direções verificadas

Com os dois configs renderizados, domínio 69:

**módulo → host** (publisher no container `tools` do módulo, subscriber nativo
no host):

```
[module.sh] 3/3 modulo publica, host recebe (/demo/system/heartbeat)
    publisher ativo no modulo
    host recebeu do modulo: data: count=11
```

**host → módulo** (publisher nativo no host, `ros2 node list` / `topic echo`
dentro do container `tools` do módulo):

```
--- nodes ---
/demo/heartbeat_publisher
--- topicos /demo ---
/demo/system/heartbeat
--- echo ---
data: count=14
```

Alcance UDP, etapa 1, confirmado por payload (não por "chegou algum
datagrama"):

```
1/3 alcance UDP no dominio 69, porta 24660
    modulo -> host: OK (de <MODULE_IP>)
```

### Por que a etapa 1 checa o payload

Numa execução com a simulação ativa, a etapa 1 reportou
`modulo -> host: OK (de <HOST_IP>)` — **o endereço do próprio host**, num
teste cujo objetivo era provar que o módulo alcança o host. Aquelas são portas
RTPS vivas: o primeiro datagrama a chegar foi tráfego SPDP de terceiros.

Também apareceu o caso de `bind` falhando com `EADDRINUSE` na porta 24660,
porque a simulação do host já a ocupava, e o probe morria num traceback antes
de testar coisa alguma. Agora a etapa procura o primeiro índice de participante
livre na faixa do domínio, **diz** quando a canônica está ocupada (é sinal de
que há DDS vivo no host) e só aceita o datagrama cujo payload é `dds-probe`.

---

## Estado ao fim da sessão

**Pronto e verificado no hardware real:**

- Módulo inventariado; Docker e Compose operacionais; `torizon` no grupo
  `docker`.
- Quatro imagens `arm64` construídas nativamente no módulo, regra 1 verificada
  nas quatro.
- Fontes e config em `~/demo` no módulo, `.env` gerado, `module.xml` renderizado
  com interface fixada e peer do host.
- Alcance UDP nas duas direções, com verificação de payload.
- **Contrato atravessando a fronteira de máquina nas duas direções**, medido.
- `scripts/module.sh` como interface única, com guard contra colisão de
  `/demo/cmd_vel`.

**Não feito, e por quê:**

- **`nav` e `perception` não foram subidos.** Nav2 publica `/demo/cmd_vel` e a
  simulação do host roda no mesmo domínio 69; dois publishers corromperiam o
  ensaio de marcha em curso em silêncio. `scripts/module.sh up` recusa por
  default enquanto houver simulação ativa no host.
- **O módulo não vê os tópicos da simulação do host.** Não é defeito do módulo:
  `scripts/run_quadruped_sim.sh` sobe a sim sem `CYCLONEDDS_URI`, então ela
  anuncia por multicast e o módulo (multicast off) não pode descobri-la. O
  mecanismo está provado — falta passar o config renderizado ao produtor do
  host. **Não alterado nesta sessão porque esse script está em uso pelos ensaios
  de F4.** A mudança é acrescentar ao `docker run`:

  ```
  -e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml \
  -v "${repo_dir}/docker/cyclonedds/host.rendered.xml:/cfg/cyclonedds.xml:ro"
  ```

- **`compose.host.yml` continua montando `cyclonedds/host.xml`**, o template sem
  o peer do módulo. Para o `hil` containerizado ele precisa apontar para o
  arquivo renderizado.
- **Nada de desempenho foi medido** (regras 5 e 7). Nenhuma afirmação sobre CPU,
  latência, térmica ou FPS do AM69 existe neste documento.

**O que continua bloqueado, e não por infraestrutura:** F5 exige goal Nav2
`SUCCEEDED` com o robô de pernas, e a árvore de TF não fecha — não existe frame
`odom` (`plano-movimentacao.md`). O target estar pronto não move esse portão.
