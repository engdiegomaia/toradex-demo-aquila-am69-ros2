# ML3.5 — aplicação rodando no Aquila AM69 em HIL

**Data:** 21/08/2026
**Hardware:** Aquila AM69, Torizon OS 7.7.0, 8 × Cortex-A72, 31 GiB
**Host:** x86, Wi-Fi `wlp0s20f3` (192.168.15.98) — módulo em 192.168.15.122
**Mundo:** `quadruped_maze11.sdf`, metas `MAZE11_GOALS`, protocolo
`nav_trial.py --seconds 180` (prazo de meta 90 s)

> **Atualização de 24/08/2026:** este documento preserva a medição histórica por
> Wi-Fi. O HIL Ethernet posterior mostrou que a troca do meio físico não elevou
> a velocidade com percepção (0,0197 → 0,0198 m/s) e localizou uma
> incompatibilidade de QoS na câmera. As conclusões “Wi-Fi é o gargalo” e
> “câmera não chega” foram refutadas. Ver `ml35-hil-ethernet.md`.

Executado no hardware real. Nada aqui vem de emulação (regra 5 do CLAUDE.md).

## Resultado

A aplicação roda no Aquila AM69: Nav2 arm64 ativo no módulo, composto num
processo único, com o simulador no host. **O módulo deixou de ser o gargalo** —
a velocidade de navegação em HIL passou a empatar com a do host quando o enlace
não está carregado.

O gargalo que sobrou não é o AM69. É o **stream de câmera de 74,2 Mbit/s
atravessando o Wi-Fi**.

## O número que decide, por condição

| # | Condição | Módulo | Câmera no fio | Vel. média | `cmd_vx` médio |
| --- | --- | --- | --- | --- | --- |
| 1 | host-only, DDS multicast default | parado | não | 0,0720 m/s | 0,0566 |
| 2 | host-only, DDS de HIL (peers explícitos) | parado | não | **0,0725 m/s** | 0,0574 |
| 3 | HIL, Nav2 + perception no módulo | ativo | **sim** | 0,0197 m/s | 0,0064 |
| 4 | HIL, só Nav2 no módulo | ativo | não | **0,0427 m/s** | 0,0282 |

Todas com Nav2 composto, mesmo mundo, mesmas metas, mesmo protocolo.

Leitura em três passos:

- **1 vs 2** — a configuração de CycloneDDS com `AllowMulticast=false` e peers
  explícitos **não custa nada**: 0,0720 contra 0,0725. A hipótese de que ela
  fosse a culpada foi levantada e refutada.
- **2 vs 4** — pôr o Nav2 no módulo custa 1,7× (0,0725 → 0,0427). Esse é o custo
  real do HIL: latência de sensor e comando atravessando a rede.
- **4 vs 3** — ligar a perception, e com ela o stream de câmera, custa outros
  2,2× (0,0427 → 0,0197). Sozinho, é **o maior fator isolado do conjunto**.

## O stream de câmera, medido no fio

```
640x480 rgb8, 921600 bytes/quadro
121 quadros em 12,0 s = 10,1 Hz  ->  74,2 Mbit/s
```

Único consumidor: `demo_perception`, no módulo. O tópico é `Image` cru, sem
compressão, com QoS confiável — cada perda vira retransmissão, e a
retransmissão vira contrapressão no publicador dentro do simulador. É por isso
que o efeito aparece na velocidade do robô e não num erro de rede.

**E o stream não chega.** Sonda dentro do `demo-tools-1`, no módulo: **zero
quadros em 30 s**. O `detection_stub` está inscrito e vivo (log de subida
presente, 88,5% de CPU) e publica **zero** detecções — medido do lado do host,
0,00 Hz em `/demo/perception/detections` durante 15 s. Ou seja: o Wi-Fi não
carrega o stream de forma alguma, e os 2,2× que ele custa são o preço da
*tentativa*, não de um dado que esteja sendo usado.

Os dois nós de perception queimam ~170% de CPU somados sem receber nada. A única
coisa de alta taxa que processam é `/clock` — mesmo padrão já visto em `odom_tf`
e `cmd_vel_si_to_stick`.

### Estado do enlace, medido em 21/08/2026

| | Host x86 | Aquila AM69 |
|---|---|---|
| interface em uso | `wlp0s20f3` (Wi-Fi) | `ethernet0` |
| velocidade | — | **1000 Mbit/s full duplex** |
| Ethernet disponível | `enp0s31f6`, **NO-CARRIER** (sem cabo) | em uso |
| Wi-Fi | em uso | `wlan0` DOWN |

**O módulo já está em Ethernet.** A metade que falta é o cabo no host. Receita em
`docs/guia-operacao.md` §13, incluindo o `module.sh sync` obrigatório depois da
troca — sem ele o `host.rendered.xml` segue fixando o Wi-Fi e a descoberta falha
igual a firewall. O `run_quadruped_sim.sh` passou a avisar quando a interface
fixada no arquivo diverge da que roteia até o módulo.

A resolução vem de `demo_simulation/urdf/go2_sim.urdf.xacro`, arquivo do projeto
(o `go2_description` vendorizado pede 1280×720 @ 15 Hz e é sobreposto por ele —
1280×720 rgb8 a 15 Hz seriam ~332 Mbit/s).

Caminhos possíveis, em ordem de preferência:

1. **Ethernet no lugar do Wi-Fi.** 74 Mbit/s não é nada em gigabit. Não custa
   fidelidade nenhuma à demo e resolve o fator dominante. É mudança de bancada.
2. Reduzir taxa ou resolução da câmera no `go2_sim.urdf.xacro`. Metade da taxa é
   metade da banda. **Muda o que a demo mostra** — decisão de produto.
3. `image_transport` comprimido. Elimina a maior parte da banda sem perder
   resolução, ao custo de um nó a mais no host e CPU de codificação.

Nenhum foi aplicado: o item 1 é de bancada e o item 2 altera a aparência da
demo.

## Composição do Nav2: o que ela entregou

`nav_quadruped.launch.py` passou a criar o `nav2_container`
(`rclcpp_components/component_container_isolated`) e a ligar
`use_composition: 'True'`. Antes, treze processos separados.

| Métrica no módulo | 13 processos | composto |
| --- | --- | --- |
| memória do container `nav` | 6,89 GiB | **307 MiB** |
| CPU do container `nav` | 470% de 800% | 493% |
| load average (1 min) | 21,9 | 9,5 |
| tempo até `Managed nodes are active` | dezenas de s | **~10 s** |

Memória caiu 22×, load caiu 2,3×, ativação ficou previsível. **CPU total não
mudou** — o custo é de processamento, não de multiplicação de processos, e a
composição não o remove. Vale de todo modo: 6,89 GiB de residente por container
é o tipo de número que fecha a porta para qualquer coisa a mais no módulo.

A armadilha que isso desarma está registrada no launch: ligar `use_composition`
sem criar o container é falha silenciosa — os nós são carregados num container
que ninguém criou, nada sobe e nada imprime erro.

## Estrangular o `/clock`: tentado, medido, revertido

O `/clock` sai do Gazebo a ~880 Hz (passo de física de 1 ms) e, com
`use_sim_time`, cada nó do Nav2 assina o tópico. Parecia a explicação da
saturação, e um nó republicador a 100 Hz foi escrito para cortar isso.

| `/clock` | CPU do container `nav` | Vel. média | `cmd_vx` pico |
| --- | --- | --- | --- |
| ~750 Hz (passagem direta) | 470% | 0,0251 m/s | 0,138 m/s |
| 100 Hz (estrangulado) | **324%** | **0,0039 m/s** | **0,003 m/s** |

A CPU caiu de verdade. A navegação morreu junto: 180 s girando no lugar,
`cmd_wz` ativo em 1721 de 1800 amostras e `cmd_vx` em zero. **Economia de CPU
que faz o robô parar de andar não é otimização.**

Revertido. `clock_throttle.py` fica no pacote, com o A/B no cabeçalho, mas fora
do caminho default: o bridge voltou a publicar `/clock` direto. O mecanismo
exato de como 10 ms de granularidade quebram o MPPI **não foi isolado** — o que
está medido é a relação de causa, não o porquê.

## O que travou a navegação nas corridas ruins

Em todas as corridas de velocidade baixa o padrão é o mesmo, e não é "devagar":

- `cmd_vx` **pico normal** (0,10 a 0,14 m/s, o teto é 0,15) e **médio quase
  zero** (0,006 a 0,008). O robô atinge a velocidade cheia em rajadas curtas.
- Na pior corrida, `cmd_wz` ativo em 1721 de 1800 amostras com `cmd_vx` zerado:
  180 s girando no lugar, yaw oscilando entre 53° e 91°.

Sondando o caminho global durante uma dessas corridas, com o robô parado nos
mesmos 18 s:

```
robo (-0.06, -0.17) yaw 84.5 deg, caminho com 475 poses
cmd: vx=0.0000  wz=0.0350
pose  5: (-0.19,-0.15)  d=0.14 m  rumo relativo  +86.9 deg
pose 10: (-0.31,-0.12)  d=0.26 m  rumo relativo  +84.4 deg
destino: (0.00, 8.00)
```

Duas coisas juntas:

1. **A largada exige um giro parado de ~85°.** O robô nasce com yaw 90° (norte)
   e o corredor que o planejador escolhe sai para oeste. Girar parado é o que
   este robô faz pior — o `run_quadruped_sim.sh` já dizia isso no cabeçalho.
2. **O MPPI comanda `wz = 0,035` rad/s, 17% do teto de 0,20.** A 0,035 rad/s o
   giro de 85° levaria 42 s. Nesse ritmo o robô não sai do lugar, o caminho não
   muda, e o MPPI segue comandando pouco. Impasse estável.

Descartados por medição nessa investigação:

- **loop de controle atrasado** — 1 única linha `Control loop missed` em todos os
  logs do container. O MPPI roda no ritmo configurado.
- **TF/costmap quebrados** — zero mensagens de extrapolação, zero exceções de TF.
  O costmap sondado tem 67% de células livres, custo 0 na célula do robô e as
  paredes onde deveriam estar.
- **`collision_monitor` zerando o comando** — nunca registrou `stop`, `slowdown`
  ou `approach`. É transparente nessas corridas.
- **abortos de `follow_path`** — ~1 por segundo, mas é o `RateController` de 1 Hz
  da árvore de comportamento preemptando a meta anterior. Comportamento normal
  do Nav2, não defeito.

Não corrigido nesta sessão. A hipótese com mais apoio é de sintonia:
`PathAlignCritic` está em 14,0, o maior peso da configuração, e
`PathAngleCritic` — o critic que recompensa alinhar-se a um caminho em ângulo
grande — está em 2,0. Girar parado não aproxima o robô do caminho, então o
critic dominante não dá gradiente nenhum para o giro. **É hipótese, não medida.**

## Correção de um resultado anterior desta sessão

Uma medida anterior concluiu que parar a `perception` **piorava** a velocidade
(0,0131 contra 0,0202 m/s), e com ela a perception foi descartada como causa.
Está errada: foi tomada com os treze processos separados e o módulo saturado,
onde o ruído de corrida engolia o efeito. Com o Nav2 composto, parar a
perception dá 0,0427 contra 0,0197 — 2,2× a favor. Vale a medida nova.

## Outras correções de rumo, registradas para não voltarem

- **Hipótese: o hop do `clock_throttle` custava velocidade.** Refutada. Em
  passagem direta, 0,0202 com o hop e 0,0232 sem, dentro do ruído. O nó foi
  retirado do caminho default por ser intermediário sem função, não por custo
  medido.
- **Hipótese: a configuração de DDS de HIL custava 3,6×.** Refutada pela
  condição 2 da tabela. O primeiro teste mudou duas variáveis ao mesmo tempo
  (config de DDS **e** módulo ausente); repetido com uma só, a config não custa
  nada.
- **`default_server_timeout: 20` ms era curto demais** para o A72 com peers
  explícitos: metas falhavam em t=4 s e t=9 s com `Timed out while waiting for
  action server to acknowledge goal request`. Em 200 ms, mais
  `wait_for_service_timeout: 5000`, o sintoma desapareceu.

## O que isto NÃO estabelece

- **Nada sobre a demo com robô real.** Todo o plant é Gazebo no host.
- **Nada sobre térmica ou consumo do AM69.** Não foram instrumentados.
- **Nada sobre Ethernet.** A recomendação de trocar o Wi-Fi é derivada da banda
  medida, não de uma corrida em Ethernet.
- **Nenhuma meta foi cumprida em corrida de 180 s**, em nenhuma condição. Isso é
  aritmética do protocolo além de qualidade de navegação: a 0,072 m/s, 90 s
  cobrem 6,5 m e a primeira meta está a 8 m. Comparar condições dentro do mesmo
  protocolo; para meta fechada é preciso `--seconds 420 --goal-timeout 200`,
  como em `ml35-navegacao-maze11.md`.
- **A causa da baixa razão de trabalho do `cmd_vx` não está corrigida**, só
  localizada.

## Como reproduzir

```bash
# host: simulador
export MAZE_MODELS=~/ros_maze_worlds/models
./scripts/run_quadruped_sim.sh quadruped_maze11.sdf

# módulo: pilha arm64 (build NATIVO no módulo, não QEMU)
./scripts/module.sh sync && ./scripts/module.sh build
ssh torizon@<módulo> 'cd ~/demo && docker compose -f compose.module.yml up -d'

# host: ensaio. O CYCLONEDDS_URI é obrigatório — nó nativo com DDS default
# não acha o simulador com peers explícitos, e falha igual a firewall.
export CYCLONEDDS_URI=file://$PWD/docker/cyclonedds/host.rendered.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69
python3 scripts/nav_trial.py /tmp/hil.csv --seconds 180
```

Para repetir a condição 4 (sem câmera no fio), suba só `nav` no módulo. Para
repetir a condição 2, pare os containers do módulo e mantenha o resto igual.
