# Cockpit — o reset apagava o robô; agora repõe

Data: 26/08/2026
Máquina: workstation x86 (`sim` + `cockpit` + `hmi` no host) e Aquila AM69 (`nav`, `perception`)
Cenário: `quadruped_maze11.sdf`, modo `hil`

> Este arquivo cobre o botão de reset do cockpit e a resiliência dos painéis.
> Os números de recursos e temperatura do alvo saem do AM69 real (ver §4);
> **nenhuma conclusão de desempenho** é tirada de emulação.

---

## 1. O defeito: `reset.all` apaga a planta

`/demo/sim/reset` traduzia para `ControlWorld` com `reset.all = True`. Uma única
chamada, com a pilha de pé:

| tópico | antes | depois |
| --- | ---: | ---: |
| `/joint_states` | 999 Hz | **morto** |
| `/demo/imu` | 996 Hz | **morto** |
| `/demo/odom` | 49,6 Hz | **morto** |
| `/demo/scan` | 10 Hz | 10 Hz |
| `/demo/camera/image_raw` | 10 Hz | 10 Hz |
| `/clock` | 999 Hz | 997 Hz |

```
$ gz model -m demo_robot
No model named <demo_robot> was found

$ gz model --list
    - ground_plane
    - labirinto          <- demo_robot, cockpit_scene_iso e cockpit_scene_top sumiram
```

**Causa.** O robô e as duas câmeras de cena são INSERIDOS depois da carga do
mundo (`ros_gz_sim create`, em `quadruped.launch.py` e `scene_cameras.launch.py`).
`reset.all` devolve o mundo ao SDF de origem, e o SDF de origem não os contém.

**Por que é o pior modo de falha deste projeto.** O relógio continua a 999 Hz e
o Gazebo deixa os sensores órfãos publicando a 10 Hz. O cockpit fica **inteiro
verde** — relógio andando, câmera com imagem, cena com imagem — apontando para
uma planta que não existe mais. Nenhuma linha de log acusa. Recuperar exigia
reiniciar o container `sim`.

## 2. A correção: repor em vez de resetar

`/demo/sim/reset` passa a teleportar o robô para a pose de nascimento do
cenário, pelo mesmo `/demo/sim/set_entity_pose` que as câmeras de cena já usam.
O mundo não é tocado e **o relógio não volta a zero** — um salto de tempo para
trás invalidaria o buffer de TF do Nav2 e o `controller_manager`, e nada no que
o operador quer de um reset ("põe o robô no início") pede isso.

A pose vem da tabela de `scenarios.py`, com a mesma precedência do `create` que
nasceu o robô (argumento explícito vence, vazio pergunta à tabela). Cravar
(0, 0) devolveria o robô à origem em qualquer mundo cuja área útil não está na
origem — o labirinto é esse caso, e o yaw de nascimento dele **não é zero**
(1,5708: nasce olhando para o corredor, não para a parede).

## 3. Verificação medida da correção

Robô deslocado para (2,0; −1,5), depois um único clique de reset:

```
robô deslocado:  x= 1,9893  y=-1,5002  z=0,3506
resposta:        success=True  'demo_robot reposto em x=0.000 y=0.000 yaw=1.5708'
robô depois:     x= 0,00003  y=-0,0101  z=0,3507
```

| tópico | depois do reset novo |
| --- | ---: |
| `/joint_states` | 1000,5 Hz |
| `/demo/imu` | 973,7 Hz |
| `/demo/odom` | 49,9 Hz |
| `/demo/scan` | 9,99 Hz |
| `/demo/camera/image_raw` | 9,98 Hz |
| `/clock` | 984,8 Hz |

```
$ gz model --list
    - ground_plane
    - labirinto
    - cockpit_scene_top
    - demo_robot          <- continua no mundo
    - cockpit_scene_iso
```

A planta sobrevive inteira. O costmap acumulado **não** é limpo aqui — quem
limpa é `/demo/nav/reset`, que tem botão próprio no cockpit. A granularidade
separada é o que permite recolocar o robô sem derrubar o Nav2.

Guardas em `demo_simulation/test/test_sim_reset.py`, sem Gazebo: `_request('reset')`
tem de levantar, `play`/`pause` não podem carregar nenhum campo de `reset`, a
pose de reposição tem de vir da tabela do cenário, e a ordem parar → teleportar
→ retomar (ver §3.1) tem de estar nessa sequência no código-fonte.

## 3.1. O robô não "reassenta sozinho": dois defeitos silenciosos, medidos depois

A primeira versão deste arquivo dizia "o robô reassenta na altura de marcha por
conta própria". **Estava errado** — não havia teste que exercitasse o robô em
movimento no momento do reset, só o caso parado. Corrigido no mesmo dia depois
de o operador reportar guinada pós-reset.

**Defeito A — teleportar sem reancorar o gait.** `StateTrotting` (o controlador
de marcha C++) captura sua referência de postura (`pcd_`, `yaw_cmd_`) uma única
vez, atrás de um trinco que só um novo comando de caminhada limpa
(`captureBodyReference()`, `hold_captured_`). Um teleporte muda a pose real sem
passar por ali. Medido com `/demo/cmd_vel` e `/demo/cmd_vel_si` **zerados** (para
excluir o Nav2 como causa), cancelando a meta e teleportando via
`gz service .../set_pose`:

```
 t[s]  yaw[deg]       x       y
  0.0     36.10  -0.025   0.023
  9.0    167.48  -0.135   0.301
 26.0    -99.14  -0.210   0.866      <- 0,87 m de deslocamento, 135° de giro,
                                        SEM UM COMANDO PUBLICADO em 26 s
```

O eixo de guinada do `StateTrotting` satura perto de 5,3 N·m e "only drags the
feet trying" (comentário do próprio código-fonte) — não tem autoridade para
zerar esse erro, e ainda por cima o robô estava colapsado no chão enquanto
tentava (ver Defeito B).

**Defeito B — teleportar sem parar.** `SetEntityPose` reposiciona o corpo e
**preserva a velocidade**. Um robô em marcha teleportado é solto ainda viajando,
com as pernas em balanço, e cai. Medido com um fluxo de `/demo/cmd_vel` **vivo**
a 10 Hz durante o reset (o caso real, com o Nav2 conduzindo):

```
 t[s]      x        y       z   yaw[deg]
  0,1   0,215    0,031   0,337     84,2     <- reset pedido
  1,1   0,483   -0,024   0,162     90,6     <- COLAPSADO, desliza 0,27 m
 40,0   0,307   -0,084   0,159     82,7     <- ainda colapsado, se contorcendo
```

Ground truth do Gazebo (`/demo/odom`) confirmou o colapso de forma independente
do estimador do controlador: `z=0,131 m` contra 0,353 m de altura de marcha.

**A correção.** O reset agora chama dois serviços novos do `twist_to_inputs`
(único escritor de `/control_input`, por isso a correção mora ali e não no
controlador C++):

```
/demo/gait/hold    trotting -> fixed stand, eixos centrados, robô imóvel
                   (o relay teleporta aqui, com GAIT_STOP_S = 2,0 s de espera)
/demo/gait/resume  fixed stand -> trotting; StateTrotting::enter() reancora
                   pcd_ e yaw_cmd_ na pose NOVA
```

Melhor esforço: numa planta diferencial os dois serviços não existem, e essa
ausência é caminho normal (`GAIT_TIMEOUT_S = 1,0 s`, mensagem "nada a parar" em
vez de erro). Na quadrúpede, a falta de resposta ou uma recusa não fica
silenciosa — entra na própria mensagem do `Trigger` de `/demo/sim/reset`, que o
cockpit mostra.

**Verificado, repetindo exatamente o caso que falhava** (comando vivo a 10 Hz
durante o reset):

```
resposta: 'demo_robot reposto em x=0.000 y=0.000 yaw=1.5708; gait: trotting ->
           fixed stand, robô imóvel; gait: fixed stand -> trotting, pose
           reancorada'

 t[s]      x        y       z  yaw[deg]  nota
  0,1   0,000   -0,010   0,350     90,07   <- pose comandada, robô em pé
  4,5  -0,000   -0,010   0,350     90,11   <- ainda parado, hold segurando
  5,5  -0,022    0,058   0,360     91,25   <- resume: volta a obedecer o cmd_vel
 39,4  -0,344    1,853   0,360   -158,61   <- z nunca sai de 0,35-0,36 m
```

Robô nunca sai da faixa de altura de marcha (0,35-0,36 m), e volta a andar sob o
mesmo comando que estava recebendo antes do reset. Um segundo reset, imediato,
re-arma normalmente.

**Verificado também com o robô CAÍDO** (tombado, não só deslocado). Derrubado
de propósito via `SetEntityPose` com `orientation=(x=1,y=0,z=0,w=0)` (giro de
180° em roll):

```
mode=RECOVER tilt=131.3deg estPos=(2.442,0.922,0.192) ...  <- de cabeça para
                                                                baixo, preso
                                                                em RECOVER
```

`RECOVER` não sai sozinho enquanto `tilt` não cai abaixo de `TILT_OK` por
`RECOVER_SETTLE_S` — um robô de cabeça para baixo fica ali indefinidamente. O
reset foi chamado nesse estado:

```
resposta: 'demo_robot reposto em x=0.000 y=0.000 yaw=1.5708; gait: trotting ->
           fixed stand, robô imóvel; gait: fixed stand -> trotting, pose
           reancorada'

 t[s]      x        y       z  yaw[deg]
  0,1  -0,002   -0,021   0,338    90,94
 10,6  -0,009    0,013   0,349    90,71
 39,2  -0,009    0,013   0,350    91,42   <- em pé o resto do teste
```

Log do controlador depois: `mode=HOLD tilt=0.2deg ... yawSat=0%` — saiu de
`RECOVER` para `HOLD` limpo, sem o eixo de guinada saturado. Andou normalmente
sob um novo comando logo em seguida (0,75 m em 8 s). O reset recupera de um
tombamento completo, não só de um deslocamento em marcha.

Guardas novos em `test_sim_reset.py` e `test_twist_to_inputs.py`: a ordem
parar→teleportar→retomar tem de aparecer nessa sequência no fonte do relay, o
comando de descida (`2`) tem de sair exatamente uma vez (mantido, derruba o
robô por outro caminho — `StateFixedStand::checkChange` case 2 leva a
`FIXEDDOWN`), o hold não pode ter prazo próprio (só `resume` o encerra, senão é
corrida contra a chamada do relay), e todo caminho de saída do handler de reset
depois do hold tem de retomar o gait — inclusive nos ramos de erro, para não
deixar o robô preso em `FIXEDSTAND` sem log nenhum explicando por quê.

## 4. Telas: o que sobrevive a um reinício

Sonda que fala rosbridge com o **cliente do próprio cockpit**
(`hmi/js/ros/rosbridge-client.js`), um assinante por painel, contagem por janela.

Nesta topologia `cockpit` (rosbridge) e `hmi` (nginx) rodam no **host**; o
módulo roda só `nav` e `perception`. O WebSocket portanto não cai em nenhum dos
dois cenários — a página nunca recarrega e nenhum painel é perdido.

| fonte | linha de base | restart do `sim` | restart do alvo |
| --- | ---: | ---: | ---: |
| cena (`scene_iso/camera_info`) | 58 | sobrevive | sobrevive |
| câmera (`camera/camera_info`) | 58 | sobrevive | sobrevive |
| nav TF (`/tf`) | 57 | sobrevive | sobrevive |
| nav scan (`/demo/scan`) | 57 | sobrevive | sobrevive |
| `/demo/target/ops_log` | 20 | sobrevive | sobrevive |
| `/demo/target/status` | 6 | sobrevive | sobrevive |
| `/demo/cmd_vel_si` | 57 | sobrevive | **zero sem meta ativa** |
| `/demo/odom` | 57 | sobrevive | sobrevive |
| `/clock` | 57 | sobrevive | sobrevive |

O zero de `/demo/cmd_vel_si` depois do restart do alvo **não é defeito**: o
Nav2 subiu ativo (`Managed nodes are active` no log do módulo) e o
`velocity_smoother` só publica depois de receber a primeira meta. O canal novo
reporta isso em texto — `sem comando SI recente do Nav2` — em vez de deixar o
painel mudo.

Uma hipótese foi **testada e descartada**: a de que o `<img>` do MJPEG precisaria
ser religado depois que o publicador volta. Medido com `curl` na mesma resposta
HTTP, através de um restart do `sim`:

```
antes do restart:  1.011.712 bytes
t+5s               1.835.008
t+15s              2.551.808
t+40s              4.612.096      <- cresce sem interrupção
```

O `web_video_server` mantém a inscrição ROS e a resposta multipart abertas, e os
quadros voltam sozinhos. Não gaste código religando o `<img>` sem medir de novo.

## 5. Telemetria do alvo no cockpit

`/demo/target/status` e `/demo/target/ops_log`, publicados por `target_monitor`
junto do Nav2 — no modo `hil` os números são do AM69.

Lido do módulo real (`aquila-am69-12593525`):

```json
{"cpu_percent": 54.53, "temp_c": 34.974, "load1": 7.575, "load5": 7.505,
 "load15": 9.158, "mem_total_mb": 31836.8, "mem_used_mb": 1392.3,
 "mem_percent": 4.373}
```

Temperatura conferida contra o sensor, não inferida: `temp_c` é o máximo das
zonas térmicas, e `thermal_zone1`/`thermal_zone6` liam 34498 milésimos de grau
no mesmo instante — 34,498 °C. As sete zonas (`wkup0/1`, `main0..4`) ficaram
entre 32,1 e 34,5 °C.

Custo do nó no módulo, `ps -o pcpu` (média de vida do processo):

| processo | %CPU |
| --- | ---: |
| `target_monitor` | 4,3 |
| `odom_tf` | 3,7 |
| `cmd_vel_si_to_stick` | 0,8 |

4,3% de um núcleo em 800% disponíveis. `use_sim_time: False` é o que mantém isso
barato — o nó não assina `/clock` (ver `test_sim_time_scope.py` e o bloco PISO
OCIOSO DE CPU em `nav_quadruped.launch.py`).

## 6. O que este arquivo NÃO prova

- O cockpit não foi aberto num navegador nesta sessão: **não há Chrome nesta
  máquina** e o MCP de automação não dirige o Firefox instalado. Tudo em §4 foi
  medido no caminho de dados, com o cliente rosbridge do cockpit, não na tela.
  O gate visual continua pendente de uma passada manual.
- Controle manual (F4) segue **não implementado**: os botões de seta continuam
  inertes de propósito.
- Nada aqui melhora a decisão de trajeto do Nav2. O robô continua girando em vez
  de transladar — ver `ml35-f5-clock-fanout.md`.
