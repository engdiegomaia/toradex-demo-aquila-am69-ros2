# Changelog

Registro de fases e decisões do projeto. Uma entrada por fase concluída.
Formato: mais recente primeiro.

---

## 2026-08-26 (tarde) — o botão de reset do cockpit apagava o robô

O reset da simulação usava `ControlWorld.reset.all`. Essa variante devolve o
mundo ao SDF de origem — e o robô, mais as duas câmeras de cena, são INSERIDOS
depois da carga por `ros_gz_sim create`, logo não estão nele. Um clique deletava
a planta: `/joint_states` 999 Hz → morto, `/demo/imu` 996 Hz → morto,
`/demo/odom` 49,6 Hz → morto, `gz model -m demo_robot` → `No model named`.

**O que torna isto grave não é a perda, é a aparência.** O relógio seguia a
999 Hz e o Gazebo deixava os sensores órfãos publicando a 10 Hz, então o cockpit
ficava inteiro verde — relógio andando, câmera com imagem, cena com imagem —
apontando para um robô que não existia mais. Nenhuma linha de log acusava.
Recuperar exigia reiniciar o container `sim`.

**A correção repõe em vez de resetar.** `/demo/sim/reset` teleporta o robô para
a pose de nascimento do cenário, pelo mesmo `/demo/sim/set_entity_pose` que as
câmeras de cena já usavam, e a pose vem da tabela de `scenarios.py` com a mesma
precedência do `create`. O mundo não é tocado e o relógio **não** volta a zero:
um salto de tempo para trás invalidaria o buffer de TF do Nav2 e o
`controller_manager`, e "põe o robô no início" não pede isso. Verificado: robô
volta de (2,0; −1,5) para (0,00003; −0,010), reassenta em 0,3507 m sozinho, e os
cinco modelos seguem no mundo.

**Telemetria do alvo no cockpit.** `target_monitor` publica CPU, memória,
temperatura e load do AM69 em `/demo/target/status`, e os eixos comandados em
`/demo/target/ops_log` — o painel de logs deixou de depender de `/rosout` bruto.
Temperatura conferida contra o sensor (34,974 °C reportado, `thermal_zone1/6` em
34498 milésimos no mesmo instante). Custo: 4,3% de um núcleo em 800%.

**Protocolo antes de sintonia: `scripts/nav_campaign.py`.** Intercala `A B A B`
em vez de blocar e repõe robô e costmap entre pernas — o elo que faltava entre
`nav_trial.py` e `summarize_trials.py`. Só é possível por causa do conserto
acima: com o reset antigo, a perna 2 mediria um mundo sem robô.

**Hipótese descartada por medição:** religar o `<img>` do MJPEG quando o
publicador volta. Através de um restart do `sim`, os bytes da mesma resposta HTTP
crescem sem interrupção (1,01 → 4,61 MB). O `web_video_server` mantém inscrição
e resposta abertas.

Evidência: `docs/results/cockpit-reset-nao-destrutivo.md`.

---

## 2026-08-26 (madrugada) — o piso ocioso de CPU do módulo: limite fechado, movimento não

O gargalo de CPU do AM69, aberto desde 24/08, **está fechado e medido**. O robô
não anda melhor por isso, e essa distinção é o resultado.
Evidência em `docs/results/ml35-f5-clock-fanout.md`. Sem térmica nem consumo
(regra 7).

**Perfilar em vez de supor.** O A/B anterior terminava mandando descobrir onde a
CPU era gasta. Amostragem de `/proc/<tid>/stat` por thread mostrou o container
`nav` em **367% de 800% com o robô PARADO**, e 111% disso em três republicadores
em Python — `odom_tf`, `cmd_vel_si_to_stick`, `nav_control_relay`.

**A causa:** os três **não chamam `get_clock()` uma única vez**, e mesmo assim
assinavam `/clock` a ~870 Hz, porque quem cria a assinatura é o rclpy a partir de
`use_sim_time: true`, não o código do nó. Não é o estrangulamento de `/clock`
reprovado em 21/08: ali a taxa caía para todos, o MPPI incluso, e a navegação
morria. Aqui a taxa não muda para ninguém — muda quem assina.

**A mudança:** `use_sim_time: False` nos três, em `nav_quadruped.launch.py` e
`nav_control.launch.py`. Vale para os dois robôs, sem caminho por modo. Junto
saiu o argumento `use_sim_time` de `nav_control.launch.py` e as duas chamadas que
o passavam — argumento declarado que ninguém consome é falha silenciosa.

**Medido:** os três caíram de 111% para 17,7%, confirmado por dois métodos
independentes (perfil por thread e lista de assinantes de `/clock`). Sob
navegação: recusas do `collision_monitor` de **16 para 0**, descartes de costmap
zero, e **307% de 800% ociosos** com percepção no ar. TF íntegra.

**O que não mudou:** 0,0246 m/s, `vx` em zero em 90,7% das amostras, **0 de 2
metas de 8 m**, zero quedas. Dentro da faixa de ruído. Com CPU sobrando e sem uma
recusa de sensor, o robô continua girando em vez de transladar — **o limite de
trajeto não era efeito colateral da CPU**, e agora está isolado.

**Ressalva honesta:** o consumo total do container `nav` não caiu (367% → 377%);
o Nav2 absorveu a capacidade liberada. Em máquina saturada, porcentagem por
processo mede o que o processo conseguiu, não o que queria.

**Hipótese testada e descartada:** `inflation_radius` 0,55 em corredor de 1,20 m
não é a causa do giro — 64,1% do `local_costmap` com custo 0, e parede real à
frente.

Testes: quatro em `demo_bringup/test/test_sim_time_scope.py`, verificados por
mutação, travando também o sentido inverso — nó sem `use_sim_time` não pode
chamar `get_clock()`.

---

## 2026-08-25 (noite) — HIL cabeado no AM69: enlace resolvido, portão de 8 m reprovado

Primeira execução do protocolo de estabilidade em enlace cabeado gigabit real.
**O enlace deixou de ser o assunto; o portão de 8 m continua reprovado, e a rede
não é a causa.** Evidência em `docs/results/ml35-f5-ethernet0-repeticao.md`.
Nada de térmica ou consumo foi medido (regra 7).

Pré-condição física finalmente satisfeita: `enp0s31f6` a 1000 Mb/s full,
host ↔ Aquila por `ethernet0`, RTT 0,400 ms, rota simétrica, `verify` 3/3.
A assimetria foi eliminada de forma estrutural — Wi-Fi em métrica 600 contra 100
do cabo, então a `/24` inteira prefere o cabo; a rota `/32` é redundância.

**Hipótese refutada por medição:** a divergência `ethernet1`/`ethernet0` não
explicava "meta curta passa, 8 m estoura". Com rota correta e enlace limpo, as
duas metas de 8 m estouraram igual.

**Duas causas medidas, ambas fora da rede:**

1. **CPU do módulo.** Nav2 sozinho a **600–727% de 800%** no AM69; a percepção
   soma ~187% e passa da capacidade. A frescura do sensor colapsa e o
   `collision_monitor` recusa a nuvem do LiDAR 16 vezes com 1,0–1,2 s de
   defasagem — contra 42 ms com o Nav2 ocioso, o que prova enfileiramento por
   contenção e não transporte. Custo: **2,8×** na velocidade média
   (0,0429 → 0,0155 m/s). Vazão medida nas duas pontas bate (9,50 → 9,45 Hz),
   e o tráfego total é ~100 Mbit/s num enlace de 1 Gbit/s.
2. **Decisão de trajeto.** `vx` em zero em **79%** das amostras e giro em
   **93,9%**: o robô passa o ensaio girando em vez de transladar. Sem a câmera o
   padrão alivia e o custo migra para a rota — 18,00 m de caminho para 5,20 m
   líquidos, **28,9% de eficiência** contra 57% no host. Bate com a hipótese já
   registrada de `PathAlignCritic` 14,0 × `PathAngleCritic` 2,0, ainda sem teste
   de correção.

Tirar a câmera do fio **não** faz a meta passar (0,0429 m/s e ainda 0 de 2). São
dois limites independentes.

Uma falha silenciosa a mais, na própria ferramenta de diagnóstico: **`verify`
reprovava por `/clock` ausente contra um módulo que lia `/clock` a 616 Hz**. A
etapa 2 coletava com `ros2 topic list | grep /demo/` e depois exigia `/clock`,
que não está sob `/demo/` — a checagem não podia passar nunca. O teste que
existia passou durante todo o defeito porque só verificava se a string aparecia
no arquivo. Corrigido, com teste que casa cada tópico exigido contra o filtro de
coleta e que falha por mutação.

Também nesta sessão: imagens arm64 reconstruídas nativamente no módulo (o
`route_server` obsoleto que carregava `ReroutingService` saiu; configura e ativa
sem SIGSEGV), regra 1 verificada nas quatro imagens, e `module.sh up` passou a
usar `--force-recreate` porque o bind mount do XML não muda de caminho e o
Compose consideraria o container antigo atualizado.

Corridas 2 e 3 do protocolo n=3 não foram executadas: a 1 reprovou com mecanismo
identificado. Próxima ordem sugerida pelos dados: reduzir CPU do Nav2 → tirar a
imagem RAW do fio → só então MPPI → repetir 420 s / 200 s com n=3.

---

## 2026-08-25 — HIL no AM69: rota do DDS, precedência de `.env`, limite do Wi-Fi

Sessão de transição para o hardware. **Portão funcional passa no Aquila AM69
real**; o portão de estabilidade não foi executado porque no caminho disponível
(Wi-Fi) ele é inexecutável, não apenas inválido. Nada de desempenho, latência,
térmica, FPS ou quedas foi medido ou é reivindicado (regra 7). Evidência em
`docs/results/ml35-hil-rota-ethernet0.md`.

Premissa derrubada: **o módulo estava acessível todo o tempo.** O bloqueio era
resolução de nome — `MODULE_HOST` usa o nome mDNS, mDNS não resolve neste host e
`docker/.env` não define `MODULE_IP`. O dongle ponto a ponto e a varredura da LAN
por um "décimo MAC Toradex" eram caminhos desnecessários; o segundo não podia
funcionar, porque o módulo publica em duas portas na mesma /24 e já estava no
baseline.

Três falhas silenciosas encontradas:

1. **`docker/.env` sobrescrevia o ambiente explícito.** `set -a; source` atribui
   incondicionalmente, então os guardas `${VAR:-}` nunca distinguiam "não
   definido" de ".env venceu" — ao contrário do que o cabeçalho do script
   promete. Um `HOST_IP` obsoleto venceu o da linha de comando e renderizou os
   dois XMLs do CycloneDDS para uma interface **sem portadora**. Corrigido.
2. **`host.rendered.xml` era byte-idêntico ao template**, com `autodetermine` e
   só o peer localhost. `learn` funciona por causa disso; o HIL não teria caminho
   até o módulo, sem erro em lugar nenhum. `module.sh sync` é o passo que
   renderiza e não é opcional.
3. **O DDS do módulo estava fixado na interface errada.** As duas portas estão na
   mesma /24 e o kernel roteia por `ethernet0` (métrica 101 contra 102), mas o
   `module.xml` fixava `ethernet1` desde 24/08 — bind e envio divergentes por
   construção. Com `ethernet0`, o teste de alcance passou a `OK (de 10.22.1.67)`.

Uma hipótese implementada e **revertida**: `MaxAutoParticipantIndex` = 32. O
rastreamento de descoberta no host mostrou que o SPDP do módulo **é recebido**,
refutando a hipótese; e o valor 32 amplificava o anúncio para 33 portas por peer
por período, justamente onde as escritas falhavam
(`ddsi_udp_conn_write ... retcode -3`). Revertida pelo mesmo critério que o
projeto aplicou à resolução das câmeras: evidência contrária, e não desfazer
decisão validada com base em n=1.

O que falhava no `verify` 3/3 era o instrumento: sob `AllowMulticast=false` a
descoberta unicast é periódica, e o teste conclui ausência antes de ela assentar.
O participante mais novo é o menos provável de ter sido descoberto — a ferramenta
de investigação é a que não vê. Corrigido iniciando o assinante do host antes do
publisher remoto, com prazo limitado e código de saída cobrindo as três etapas.
O ensaio de navegação também passou a registrar tempo monotônico de parede e RTF
no mesmo intervalo de cada CSV; o portão cabeado é n=3, sucesso de uma meta de
8 m em cada corrida e zero quedas.

Dois itens abertos fecharam como "não é problema": o colapso de RTF para ~0,5
**não reproduz** (medido RTF ≈ 0,99 com `learn` e RViz2 em pé, `nproc` = 22), e a
`enp0s31f6` não é cabo — `carrier_up_count` = 0 e o PHY anuncia só `10baseT` num
I219, falha de inicialização na retomada de suspend.

Hipótese aberta para o protocolo de 8 m que falha desde 24/08: a divergência
`ethernet1`/`ethernet0` também existia naquele dia, e `/demo/cmd_vel` corre
módulo→host, exatamente a direção degradada. Por descartar, não afirmada.

---

## 2026-08-25 — cockpit web: ajustes de UI de bancada

Três pedidos do operador, fora da numeração de fases do `plano-cockpit-web.md`:
não abrem fase e não fecham portão. Tudo medido no host x86, modo `learn`;
**nada rodou no Aquila AM69**. Evidência em
`docs/results/cockpit-web-ui-ajustes.md`.

### Marca Toradex ao dobro

34 → 68 px (a do ROS, 22 → 44). A altura mínima da faixa foi de 48 para 80 px
**pelo mesmo token** — `--bar-min-height` em `tokens.css`, consumido também por
`layout.css`, onde fica a linha da grade que reserva a barra. A `.bar` tem
`overflow-x` e não `-y`: uma marca maior que a faixa é cortada sem console, sem
layout quebrado, sem sintoma. Acoplar as duas ao mesmo número torna essa
divergência impossível em vez de improvável.

### Câmeras de cena seguindo o robô

As duas vistas acompanham `/demo/odom` a 10 Hz, e o que se move é **só o alvo** da
órbita — azimute, elevação e distância ficam onde estavam, então o enquadramento
medido do armazém continua valendo com o robô andando. Medido: `scene_top` em
`(-1,552 ; 0,151 ; 6,0)` contra robô em `(-1,598 ; 0,130)`, ~5 cm de erro com o
`z` intacto; `scene_iso` em `(-4,551 ; 3,150 ; 2,4)`, mantendo o deslocamento
medido `(-3, +3, 2,4)`. Preservar o `z` era o risco: uma vista de topo que sobe
entra nas vigas do telhado do armazém.

Com seguimento ligado, os botões de mover viram **deslocamento relativo ao robô**
(saturado em 15 m) em vez de ponto fixo do mundo; `recentrar` zera isso junto.

**A divergência entre as duas plantas teve de ser explicitada.** O `set_pose` do
Gazebo só entende o referencial do mundo, e `/demo/odom` do quadrúpede é ground
truth do `gz-sim-odometry-publisher-system` (já é pose no mundo, seed 0), enquanto
o do diff-drive é integrado dos encoders com origem na pose de **spawn** (seed =
essa pose). Daí `follow_offset_{x,y,yaw}` como argumentos que a planta passa, e
não valores lidos de `x`/`y`/`yaw` dentro de `scene_cameras.launch.py`: as duas
plantas declaram esses nomes, o include os herdaria das duas, e a câmera do
quadrúpede seguiria um fantasma deslocado pela pose de spawn — no `maze11`,
deslocado **e** girado 90°.

O estado do botão vem de `/demo/cockpit/scene/following` (`std_msgs/Bool`,
latched), publicado pelo nó, e **não do clique** — mesmo raciocínio do rótulo de
simulação: F5, segundo cockpit aberto, ou alguém que desligou por linha de
comando são três casos em que o clique local não sabe a resposta.

### Reiniciar a navegação pelo cockpit

`/demo/nav/reset` (`std_srvs/Trigger`), novo `demo_navigation/nav_control_relay.py`
subido por `nav_control.launch.py` — fragmento incluído pelos **dois** caminhos de
Nav2 (mapa estático e reativo), não um `Node` duplicado. Roda no container do
Nav2, isto é, **no Aquila** no modo `hil`. Dois cliques, como o reset de
simulação: ele descarta a meta em andamento.

Medido: 6,638 s, meta interrompida terminando `CANCELED`, servidores voltando
`active [3]`, meta nova aceita em seguida.

### O achado que custou o redesenho: `RESET`+`STARTUP` mata o container

A primeira versão usava as transições que descrevem exatamente o pedido. O
`nav2_container` morre:

```
[route_server]: Configuring Rerouting service operation.
[ERROR] [component_container_isolated-4]: process has died [exit code -11]
```

`SIGSEGV`, **reproduzido duas vezes** — com e sem meta ativa. O changelog e o
guia registravam antes um segfault do `route_server` "que aconteceu uma vez";
não é intermitente, é determinístico, e está no caminho do `CONFIGURE` que
`STARTUP` executa. O `route_server` está na lista de `lifecycle_nodes` do
`navigation_launch.py` vendorizado, que precisa seguir idêntico ao upstream, então
retirá-lo não é opção; a demo não usa roteamento e ele não tem seção em
`nav2_params_go2.yaml`. Hipótese **não confirmada** no fonte do Nav2:
reconfiguração sobre estado que não sobrevive ao `CLEANUP`. Candidato a issue
upstream.

Sequência adotada, que nunca passa por `CONFIGURE`: cancelar todas as metas
(`goal_info` zerado, para pegar também as que o cockpit não conhece), limpar os
dois costmaps **enquanto ainda estão ativos** (nó desativado não responde
serviço, e limpar depois de pausar não limparia nada nem daria erro), `PAUSE`,
`RESUME`. O `RESUME` é tentado mesmo se o `PAUSE` falhar — a alternativa é
deixar a pilha desativada. Os timeouts usam `time.monotonic()`: um Nav2
desativado coexiste com um `/clock` parado, e timeout em tempo simulado nessa
janela nunca expira.

A localização fica fora de propósito. Reciclar o AMCL joga a pose fora, e o robô
"se perde" num reset que era só para descartar a meta.

### Testes

`tests/` 29 → **36**; bundle 131 → **138**. Novos: 9 casos de órbita em
`demo_simulation/test/test_scene_view_controller.py`, 7 casos de fiação do botão
de seguimento no bundle (com um duplo de DOM mínimo, primeiro do projeto), e seis
guardas estruturais — um deles existindo só para impedir que o reset volte a
`RESET`+`STARTUP`.

Um dos guardas casou com a **própria documentação** do relay: buscar
`'lifecycle_manager_localization' not in code` batia no docstring que explica por
que a localização fica de fora. Os guardas agora leem o código com `ast`, sem
comentários nem docstrings. Regex não serviria — uma tripla quota dentro de
f-string quebra a varredura em silêncio.

De carona: as três falhas de lint pré-existentes em `demo_simulation` (I100, E731,
E501) foram corrigidas, para que a suíte onde entraram testes novos esteja verde
em vez de já vermelha.

### Não verificado

Nenhuma das três mudanças foi vista num navegador — o Chrome não está instalado
nesta máquina e o Firefox snap headless não respondeu. O que existe é a página
servida com o HTML e o CSS corretos (`curl` 200, tokens e os dois botões presentes
no que o nginx entrega) mais a verificação pelo lado do ROS. A conferência visual
fica pendente do operador.

---

## 2026-08-24 — cockpit web F3b: cena, navegação, controle e identidade

Fecha o F3b do `plano-cockpit-web.md` e absorve quatro pedidos do operador na
mesma sessão. Evidência completa em `docs/results/cockpit-web-f3b.md`.

### Painel azul — vista da cena

Duas câmeras estáticas do MUNDO, alternáveis no cabeçalho do painel. Elas são
**modelos spawnáveis** (`demo_simulation/models/cockpit_scene_{iso,top}.sdf`),
não elementos escritos nos mundos, por duas razões que só aparecem depois:

- o mundo default (`nav2_minimal_tb4_sim/worlds/warehouse.sdf`) é de terceiros e
  não é editável, e é ele que sobe quando o compose não passa `world:=`;
- pendurar a câmera no `go2_description` vendorizado quebraria a garantia
  byte a byte que sustenta o argumento de licença.

Enquadramento medido, não chutado: a primeira tentativa (iso em `-7,-7,5`) caía
dentro dos corredores de prateleira do armazém, e a de topo a 12 m batia numa
viga do telhado exatamente sobre o robô.

### Painel verde — navegação 2D

Canvas único redesenhado por inteiro a cada tique: costmap (PNG, 33x menor que
JSON), plano global, laser, pegada do robô com bigode de proa, e a meta. O
clique manda `NavigateToPose`. Portão do F3b cumprido: meta clicada aceita e
executada, com feedback e contagem de recuperações chegando ao HUD.

**Falha silenciosa encontrada:** o rosbridge entrega **uma** mensagem latched de
`/tf_static` por inscrição, e qual delas é sorte — medido em três inscrições
novas e consecutivas: arestas do robô, `map->odom`, `map->odom`. Em cerca de
metade dos carregamentos faltava `map->base` e o painel ficava vazio.
`queue_length: 16` não muda nada; a perda é acima da fila do cliente. Correção:
reinscrição limitada (1,5 s, no máximo 8 vezes), que para em definitivo assim
que a cadeia resolve.

### Controle da simulação pelo cockpit

Play, pause e reset na barra. O pedido original dizia "iniciar a simulação **no
target**", e essa parte **não é possível**: o Gazebo é OGRE 2 e o AM69 só expõe
OpenGL ES 3.2 e Vulkan 1.2 (regra 1). O que existe é controlar **a partir do**
cockpit o Gazebo que roda no host — no M3, com o cockpit servido pelo Aquila, o
clique sai do módulo e a chamada atravessa o grafo ROS, como a meta do Nav2 já
atravessa hoje.

**Falha que definiu a arquitetura:** chamar `/demo/sim/control`
(`ros_gz_interfaces/srv/ControlWorld`) direto do navegador não funciona —

    call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv

O rosbridge monta o pedido importando o pacote de interfaces dentro do próprio
container, e o container do cockpit não tem `ros_gz_interfaces`. E não deve ter:
no M3 ele roda no Aquila, e no modo `deploy` não há Gazebo nenhum. Nasceu daí o
nó **`sim_control_relay`** (host), que expõe `/demo/sim/{play,pause,reset}` como
`std_srvs/Trigger` e traduz para `ControlWorld`. A fronteira do navegador só fala
tipos de núcleo do ROS.

O rótulo de estado vem do `/clock`, **não** do último clique: um eco mentiria
com o container morto, com a chamada expirada, ou com a pausa feita pela GUI do
Gazebo. Reset exige dois cliques — ele devolve o robô à pose inicial e apaga o
costmap acumulado do Nav2.

### Controle de câmera

Pad sobre a imagem: girar, inclinar, mover, aproximar, recentrar. O navegador
publica **deltas** em `/demo/cockpit/scene/cmd_view`; quem guarda a órbita,
satura os limites e escreve a pose é o `scene_view_controller`, do lado do
simulador. O caminho alternativo (cliente calcula a pose e manda pronta) quebra
com dois cockpits abertos, zera o enquadramento a cada F5, e duplicaria a
matemática de órbita em duas linguagens.

### Identidade Toradex

Fundo branco, `#00508c`, `#96c837`, `#ff5a00`, valores exatos do time, com as
marcas Toradex e ROS na barra. Duas consequências que não são cosméticas:

- **o vermelho de falha não é da marca, de propósito** — "parado há tempo
  demais" e "morto" precisam parecer coisas diferentes através da sala;
- **a barra é a única superfície escura da tela** — as marcas são tinta branca
  com alfa e sobre branco sumiriam. Os tokens são redefinidos dentro de `.bar`,
  em vez de duplicar cada regra de botão numa variante.

O costmap foi repaletizado junto e cor de canvas deixou de existir em
JavaScript: `hmi/js/panels/palette.js` lê os tokens `--map-*` uma vez.

### Qualidade de imagem e o que ela custa

Câmeras de cena de 800x600 a 5 Hz para **1600x1200**, anti-aliasing 8, e
qualidade JPEG de 70 para 95. A proporção **continua 4:3**, e isso é restrição:
o `horizontal_fov` e as poses foram medidos nessa proporção, e ir para 16:9
desenquadra as duas cenas sem erro nenhum.

A taxa subiu de 5 para **10 Hz, e não 15, por medição**. As duas câmeras
renderizam no mesmo processo do Gazebo, e nesta resolução a workstation não
entrega 15 Hz de qualquer forma:

| `update_rate` | entregue | fator de tempo real |
| --- | --- | --- |
| 15 | 9,43 Hz | 0,59 |
| 10 | 9,77 Hz | 0,97 |

Pedir 15 não rendia um quadro a mais e custava 40% da velocidade da simulação —
o que estica cada meta do Nav2 na mesma proporção. No modo `hil` o stream
atravessa a Ethernet até o Aquila, e nada disso foi medido lá; se o gargalo for
banda e não render, o parâmetro a baixar primeiro é a qualidade JPEG em
`hmi/js/config.js`.

### Caixas de detecção fora do vídeo

A pedido do operador. O `demo_perception` de hoje é um stub que varre uma caixa
sintética pela imagem quer haja objeto ali ou não; sobre o vídeo isso vira um
retângulo passeando, que numa demo é lido como detecção de verdade. As detecções
continuam publicadas e continuam alimentando a `perception_layer` do costmap — o
contrato de tópicos não mudou, saiu só o desenho. O módulo de overlay segue no
bundle, testado, para voltar com o TIDL.

### Suítes

131 testes de bundle (`node --test`) e 29 guardas estruturais (`pytest`).

### Documentação

Novo **`docs/guia-cockpit.md`**: como rodar, o que cada painel mostra, o que os
botões fazem, como trocar de cenário, como ajustar qualidade de imagem, e as
sete armadilhas próprias do cockpit. A seção "Cockpit de demonstração no host"
de `guia-operacao.md` — que descrevia o `run_cockpit.sh` e o eixo X11 — foi
substituída por um ponteiro e por um aviso explícito de não retomar aquele
caminho.

---

## 2026-08-24 — cockpit web F1: transporte e esqueleto, no host

Primeira fase do plano aprovado em `docs/ml35/plano-cockpit-web.md`. O eixo
deixou de ser "capturar janelas X11" e passou a ser "renderizar a partir de
tópicos ROS 2"; esta entrada é a parte de transporte desse eixo.

Dois serviços novos em `docker/compose.host.yml`, fora do perfil `learn` porque
são desejados nos dois modos:

- **`cockpit`** — imagem própria (`docker/cockpit/Dockerfile`) com
  `rosbridge_server` 2.7.0 e `web_video_server` 3.1.0, entrando por
  `demo_bringup/launch/cockpit.launch.py`. É o único papel gráfico-adjacente que
  pode ir para o módulo: nada aqui linka OGRE 2 nem OpenGL de desktop, e por
  isso a imagem é multi-arch por construção. Não é o `viz`, que é amd64 para
  sempre.
- **`hmi`** — `nginx:alpine` servindo `hmi/`. Separado do `cockpit` para que uma
  mudança de CSS não reconstrua uma imagem ROS de 1,3 GB.

O bundle em `hmi/` é HTML/CSS/ES modules sem etapa de build, com cliente
rosbridge próprio (~250 linhas, com reconexão e replay de assinaturas),
rastreamento de frescor never/live/stale, painel de câmera e painel de logs.
52 testes sob `node --test` e 7 guardas estruturais sob pytest.

Portão do F1 atendido e fotografado: cinco regiões na proporção da imagem de
referência, `/demo/camera/image_raw` ao vivo a 9,98 Hz, e derrubar o rosbridge
troca o estado visual e reconecta sozinho. Evidência completa em
`docs/results/cockpit-web-f1.md`.

Duas falhas silenciosas medidas e travadas por teste:

1. `web_video_server` **não faz percent-decode do parâmetro `topic`**. As duas
   formas respondem HTTP 200; a codificada devolve 22 bytes e nenhum quadro.
   `URLSearchParams` escapa `/` por padrão, então o painel ficava vazio com
   status saudável e nada em log.
2. No Firefox, um `<img>` ligado a `multipart/x-mixed-replace` **nunca dispara
   `load`** e mantém `complete === false` durante toda a vida de um stream
   saudável. O frescor da câmera passou a vir de `/demo/camera/camera_info` pelo
   rosbridge — poucas centenas de bytes na mesma taxa da imagem.

Dois pontos abertos do plano foram respondidos: rosbridge 2.x **expõe** ações
ROS 2 (`SendActionGoal` registrado no startup), e `web_video_server` **aceita** o
tópico reliable sem reconfiguração.

Os botões de controle manual estão renderizados e **desabilitados de propósito**:
publicar em `/demo/cmd_vel` junto com o Nav2 sem árbitro é o débito que o F4
fecha com `twist_mux`, não com um mux à mão.

Nada foi executado em arm64 nem no Aquila AM69. Próxima fase: F3b, painéis de
cena e de navegação.

---

## 2026-08-24 — cockpit standalone: checkpoint parcial, não aceito

A primeira implementação interpretou "cockpit" como uma barra que organizava
três janelas independentes. Esse não era o requisito: o cockpit deveria ser uma
única janela Qt incorporando Gazebo, RViz e `rqt_image_view`. Em runtime,
somente o Gazebo foi reparentado; RViz e câmera ficaram externos e seus painéis
internos vazios. Portanto o cockpit **não está concluído**.

A faixa de controle publica comandos manuais de manche em `/demo/cmd_vel` por
uma ponte `rclpy` persistente dentro do container `viz`. Botão/tecla precisa
ficar pressionado; release, perda de foco, timeout de 400 ms, EOF e fechamento
publicam zero. Os valores ficam dentro do envelope de manche validado. Controle
manual e meta Nav2 não devem ser usados simultaneamente, pois compartilham o
mesmo tópico.

O launcher exige uma sessão X11/EWMH já disponível e não tenta substituir,
reparar ou reconfigurar o desktop. Reiniciar o GNOME Shell durante a tentativa
anterior interrompeu processos não relacionados e não faz parte do procedimento.
Os testes são estáticos e não substituem o teste visual/X11 que falhou. A última
tentativa, com `XReparentWindow` direto, foi implementada mas não executada.
Metodologia, IDs, limitações e continuidade estão em
`docs/results/cockpit-standalone-parcial.md`.

## 2026-08-24 — HIL Ethernet executado; entrega fragmentada corrigida, F5 ainda aberta

O HIL rodou com `enp0s31f6` no host e `ethernet1` no Aquila AM69, Nav2 e
percepção arm64 no módulo, e Gazebo, RViz, LiDAR e câmera no host. Como as duas
portas do Aquila estavam na mesma sub-rede e anunciavam o mesmo hostname mDNS,
o endereço alternava entre elas; o bench passou a fixar `MODULE_IP` no `.env`
local antes de renderizar os peers CycloneDDS.

Dois falsos positivos de “DDS conectado” foram encontrados. A câmera RAW
`RELIABLE` era lida como `BEST_EFFORT` e perdia todos os frames fragmentados de
921600 bytes; `detection_stub` agora pede `RELIABLE`. A nuvem LiDAR fazia o
inverso do que Nav2 espera: bridge `RELIABLE` para leitores sensor-data
`BEST_EFFORT`; o bridge agora publica somente `/demo/scan_cloud` com
`qos_profile: SENSOR_DATA`. Câmera, detecções e nuvem de detecções passaram a
~10 Hz, e o `collision_monitor` deixou de tratar o sensor como antigo.

Uma meta curta fechou `SUCCEEDED` em 28 s. O portão longo não: em 419,9 s o
robô percorreu 8,31 m a 0,0198 m/s, sem queda e sem `RECOVER`, mas duas metas de
8 m expiraram aos 200 s. A velocidade é praticamente a mesma do HIL por Wi-Fi
com percepção (0,0197 m/s), refutando a hipótese de que o meio físico, sozinho,
era o gargalo. F5 continua aberta. Evidência e limites em
`docs/results/ml35-hil-ethernet.md`.

## 2026-08-24 — F4 e F6 fechadas: contrato revalidado e fallback executado

`ROBOT_TYPE=quadruped|diffdrive` passou a selecionar em conjunto a planta e o
launch Nav2 correspondente no host e no módulo. `quadruped` é o padrão; valores
desconhecidos falham antes de iniciar a pilha. Testes unitários cobrem o
pareamento e a rejeição.

O portão F6 foi executado em duas subidas limpas do perfil `learn`. O Go2 saiu
de x≈−1,59 e concluiu a meta x=−0,8; o diff-drive saiu de x=0 e concluiu x=1,0.
Ambos terminaram `SUCCEEDED`, `error_code: 0`. O segundo ensaio também validou
o novo caminho headless `SIM_GUI=false`.

Na revalidação F4, os cinco tópicos do contrato apareceram com os tipos
esperados e houve mensagem real de odometria, scan, imagem 640 px e detecção no
container de percepção. A base e todas as imagens de função reconstruíram; o
plugin `gz_quadruped_hardware` ficou restrito à imagem `sim`. Um
`COLCON_IGNORE` presente somente nas imagens headless mantém essa fronteira em
builds e testes incrementais; a imagem `sim` remove o marcador antes de compilar
o backend Gazebo.

Isto não fecha F5: o HIL completo ainda precisa ser repetido no Aquila AM69 por
Ethernet, com a câmera preservada em 640×480 a 10 Hz.

## 2026-08-21 — A aplicação roda no Aquila AM69; o gargalo é a câmera no Wi-Fi

Nav2 arm64 ativo no módulo, **composto num processo único**, com o simulador no
host. O módulo deixou de ser o gargalo. O que sobrou é um stream de câmera de
**74,2 Mbit/s** atravessando o Wi-Fi. Evidência, protocolo e limites em
`docs/results/ml35-hil-aquila.md`.

### O número que decide, isolado em três passos

| Condição | Módulo | Câmera no fio | Vel. média |
| --- | --- | --- | --- |
| host-only, DDS multicast default | parado | não | 0,0720 m/s |
| host-only, DDS de HIL | parado | não | **0,0725 m/s** |
| HIL, Nav2 + perception no módulo | ativo | sim | 0,0197 m/s |
| HIL, só Nav2 no módulo | ativo | não | **0,0427 m/s** |

A configuração de CycloneDDS com `AllowMulticast=false` e peers explícitos **não
custa nada** (0,0720 vs 0,0725) — hipótese levantada e refutada. Pôr o Nav2 no
módulo custa 1,7×. Ligar a câmera custa outros **2,2×**, e é o maior fator
isolado do conjunto. Medido no fio: 640×480 rgb8, 921600 B/quadro, 10,1 Hz.

Não corrigido: trocar o Wi-Fi por Ethernet é mudança de bancada, e reduzir taxa
ou resolução da câmera muda o que a demo mostra — decisão de produto.

### Composição do Nav2: o container que faltava

`nav_quadruped.launch.py` passou a criar o `nav2_container`
(`component_container_isolated`) e ligar `use_composition`. Memória do container
`nav` no módulo **6,89 GiB → 307 MiB**, load average **21,9 → 9,5**, ativação em
**~10 s**. CPU total **não mudou** (470% → 493%): o custo é de processamento, não
de multiplicação de processos.

A armadilha fica registrada no próprio launch: ligar `use_composition` sem criar
o container é falha silenciosa — os nós vão para um container que ninguém criou,
nada sobe e nada imprime erro.

### Estrangular o `/clock` foi tentado, medido e revertido

O `/clock` sai a ~880 Hz e cada nó com `use_sim_time` o assina. Estrangulado a
100 Hz, a CPU do container caiu de 470% para **324%** — e a navegação **morreu**:
0,0039 contra 0,0251 m/s, `cmd_vx` de pico caindo de 0,138 para 0,003, com 180 s
girando no lugar. Economia de CPU que faz o robô parar de andar não é
otimização. `clock_throttle.py` fica no pacote com o A/B no cabeçalho, fora do
caminho default.

### O que trava a navegação, localizado e não corrigido

Em todas as corridas ruins o padrão é `cmd_vx` de **pico normal** (0,10–0,14) e
**médio quase zero** (0,006–0,008): rajadas curtas, não lentidão. Sondando o
caminho global com o robô parado nos mesmos 18 s, a largada exige giro parado de
**~85°** e o MPPI comanda `wz = 0,035` rad/s — **17% do teto de 0,20**, o que
levaria 42 s para o giro. Impasse estável: o robô não sai, o caminho não muda, o
comando não cresce.

Descartados por medição: loop de controle atrasado (1 linha `Control loop missed`
em todos os logs), TF/costmap quebrados (zero extrapolações, costmap com 67%
livre e custo 0 na célula do robô), `collision_monitor` zerando o comando (nunca
registrou ação), abortos de `follow_path` (é o `RateController` de 1 Hz da árvore
preemptando — normal).

### Correções de rumo

- **Uma medida anterior desta sessão está errada** e foi substituída: concluía
  que parar a perception *piorava* a velocidade (0,0131 vs 0,0202). Foi tomada
  com treze processos e o módulo saturado. Composto, o efeito inverte e cresce.
- **O hop do `clock_throttle` não custava velocidade** (0,0202 com, 0,0232 sem —
  dentro do ruído). Foi retirado por ser intermediário sem função.
- **`default_server_timeout: 20` ms era curto** para o A72 com peers explícitos:
  metas falhavam em t=4 s com `Timed out while waiting for action server`. Em
  200 ms mais `wait_for_service_timeout: 5000`, resolvido.

---

## 2026-08-21 — O robô não estava lento, estava indo de ré: três defeitos de decisão no Nav2

Velocidade média no maze11 **0,0399 → 0,0650 m/s (+63%)**, ré **11–62% → 0%** das
amostras, eficiência de trajeto **13% → 57%**, e a **primeira meta cumprida** de
todo o esforço (8 m em 96 s). Zero quedas, zero `RECOVER`, folga de carcaça
inalterada. Evidência, protocolo e limites em
`docs/results/ml35-navegacao-maze11.md`.

### O plano previa subir `vx_max`. A medição matou isso antes de gastar corrida

`cmd_vx` pico ficava entre 0,070 e 0,108 contra um teto de **0,15 que nunca era
alcançado**, e o `cmd_vx` **médio** era ≈ 0 (−0,008 a +0,017) com até 62% das
amostras negativas: o comando era ruído em torno de zero. Multiplicar o teto de
uma distribuição centrada em zero não move a média, e a média é o que decide o
tempo de travessia. `vx_max` ficou em 0,15 nas quatro condições, e o pico subiu
para 0,130–0,143 sozinho — o teto sempre esteve lá, o controlador é que não usava.

### Defeito 1 — deadlock por ré

`sensor_check.py`, janela de 20 s: `/demo/cmd_vel_si` com `vx < 0` em **100%** das
amostras e o robô andou **0,00 m em 25 s**. Num corredor de 1,20 m o
`PathAlignCritic` (peso 14) acha ótimo recuar para realinhar; o robô recua;
encosta na parede de trás; e ré continua ótima, porque a geometria que a tornou
ótima não mudou. `PreferForwardCritic` já estava ligado com peso 5,0 e não
segurou — peso não compete com uma opção que não devia estar no espaço de
amostragem.

Correção: `vx_min: 0.0` no MPPI e `min_velocity[0]: 0.0` no `velocity_smoother`
(os dois, senão o smoother corta o teto novo **sem log**). Para corrigir rumo o
MPPI passa a ter de girar; `wz_max` subiu 0,12 → 0,20, o que só é seguro porque
`k_yaw` 0,35 tirou o eixo de guinada do batente (`yawSat` pico 90–94% → 64–76%).
Recuo real continua possível: o `backup` do `behavior_server` publica `cmd_vel`
direto e não passa por esses limites.

### Defeito 2 — `smoother_server` configurado e nunca invocado

**Nenhum behavior tree que acompanha o Nav2 Jazzy chama `SmoothPath`**
(`grep -l` em `nav2_bt_navigator/behavior_trees/*.xml`: zero arquivos). O
`simple_smoother` subia, ficava `active`, e nada nunca lhe mandava trabalho — sem
erro, sem aviso. Mesma classe do `perception_layer` sem produtor.

Consequência: o NavFn é Dijkstra sobre grade de 0,05 m 8-conexa, o caminho global
sai em **escada**, e o `PathAlignCritic` cola o robô nessa escada. O robô
perseguia cada degrau, o que aparece como giro contínuo num corredor reto.

Correção: `demo_navigation/behavior_trees/nav_to_pose_smoothed.xml`. O caminho
absoluto é injetado por `RewrittenYaml` em `nav_quadruped.launch.py` — a chave
existe no YAML como placeholder porque o `RewrittenYaml` substitui chave que
encontra e não acrescenta chave nova, e a reescrita fica no **nosso** launch para
não tocar o `navigation_launch.py` vendorizado.

### Defeito 3 — NavFn escolhia rota por comprimento, não por folga

Com `inflation_radius: 0.55` num corredor de 1,20 m, a faixa de custo **zero** no
meio tem 0,10 m; fora dela o custo é plano e o Dijkstra decide pelo comprimento.
Correção **só no costmap global**: `inflation_radius: 0.85` (> 0,60, a
meia-largura do corredor) e `cost_scaling_factor: 2.0`, o que dá gradiente
monotônico até o centro e faz rota apertada custar mais que rota aberta.

Isso **divergiu de propósito** do costmap local (0,55 / 5,0), contra o comentário
que existia ali pedindo inflação casada. O comentário vale numa direção só:
global mais conservador que o local é seguro, porque o caminho global cai sempre
em terreno que o local aceita; o inverso é que produz "plano global que o local
recusa". O local ficou em 0,55 porque subir a inflação local aproximaria o MPPI
do campo de custo saturado que trava o otimizador. E a faixa *inscrita*, que o
planejador trata como obstáculo, vem de `robot_radius` (0,38) e não de
`inflation_radius`: subir a inflação não fecha corredor nenhum.

### Lidar e odometria: verificados a pedido, e estão sãos

`/demo/odom` vs TF `odom -> base`: erro **0,0000 m**. Taxas: odom 49,8 Hz,
`scan_cloud` 9,9 Hz com 10240 pontos, `cmd_vel_si` 20,0 Hz. TF fecha nas duas
arestas. Deriva de odometria está descartada **por construção**, não por medida:
`/demo/odom` é ground truth do Gazebo. Odometria de perna é F5, e é lá que deriva
volta a ser risco.

Auto-colisão de lidar foi **investigada e descartada**, depois de um diagnóstico
errado no caminho: 3086 pontos a menos de 0,383 m com rumo +82,7° ± 2,2° pareciam
peça do robô, mas o agrupamento mudou de lugar na janela seguinte e os raios
ajustam `d/cos(θ−θ₀)` de uma parede reta a 0,372 m. O robô estava encostado na
parede pelo defeito 1, e parede vista de perto por robô parado dá exatamente a
assinatura de rumo fixo. `scripts/selfhit.py` agora **avisa** quando o robô andou
menos de 0,15 m na janela, porque nesse caso o teste não separa os dois casos.

### Duas armadilhas de instrumentação, ambas de leitura errada

- **`/local_costmap/costmap` é reescala, não custo bruto.** O `Costmap2DPublisher`
  mapeia 254 → 100, 253 → **99**, 255 → −1. Comparar com 253 ali nunca casa, e o
  resultado sai como "nenhuma célula de colisão" num corredor cercado de parede —
  foi o que minha primeira sonda relatou. Use `costmap_raw` (`nav2_msgs/Costmap`).
  Com a leitura certa: 63,6% livre, 23,3% colisão, custo 0 na célula do robô, o
  que **refuta** a hipótese de que o corredor de 1,20 m saturava o costmap.
- **`spin_once` trata UM item por chamada.** Criar um `TransformListener` junto da
  inscrição do costmap faz os callbacks de `/tf`, que é de alta taxa, consumirem
  todas as iterações, e a mensagem latched do costmap nunca é selecionada. O
  sintoma é "tópico não publica" com o tópico publicando.

### Armadilha de operação: matar o `ros2 launch` deixa os nós vivos

`ros2 launch` é só o pai. `kill` nele órfã os filhos, que seguem segurando índice
de participante do CycloneDDS, e a subida seguinte do Nav2 morre com
`rmw_create_node: failed to create domain` em **todos** os nós — mensagem que
acusa o DDS quando o culpado é a execução anterior. Medido: 31 órfãos de 5
gerações, 14 falhas de domínio. Um nó isolado ainda criava domínio sem erro, o
que faz parecer que o DDS está bem; e está. Documentado em
`docs/guia-operacao.md` §9.12, junto com `pkill -f` casando a própria linha de
comando de quem chama (matou meu shell duas vezes) e com o truncamento de `comm`
em 15 caracteres.

### Novas ferramentas

`scripts/sensor_check.py` (taxas, TF vs odom, e a fração de ré no comando),
`scripts/costmap_probe.py` (perfil de custo transversal, no `costmap_raw`) e
`scripts/selfhit.py` (auto-colisão por constância de raio). Todas somente
leitura, todas limpas em `ament_flake8` e `ament_pep257`.

### Não estabelecido

Nada sobre o Aquila AM69. E a folga de carcaça segue em **+6,5 cm**, que não
folgou com a V3 e é o portão de qualquer aumento futuro de velocidade — ela vem
de `robot_radius: 0.38` modelar o tronco de 0,70 × 0,31 m como círculo, quando
lateralmente o robô precisa de 0,155 m. A correção estrutural é footprint
poligonal com `consider_footprint: true`, fora do escopo desta fase.

---

## 2026-08-21 — S6 passa para o `maze11`, e a geometria de mundo vira medição

Cenário S6 trocado de `maze10` para **`maze11`**, com a partida no **canto
inferior direito** para que a demonstração comece numa ponta e atravesse o
labirinto inteiro. Evidência em `docs/results/ml35-labirinto.md`.

**O achado que tornou a troca barata:** `maze11` é geometricamente gêmeo do
`maze10`. Mesmo passo de célula (600 unidades brutas de STL), então na escala
0,002 os dois têm **corredor de 1,20 m e parede de 0,60 m** — os dois números
que dimensionaram a escala do `maze10`. A área navegável quase dobra, de 18,4
para **35,4 m²**, num único componente conectado. Nada de escala,
`robot_radius`, `inflation_radius` ou costmap precisou ser reajustado.

`scripts/maze_fit.py` — a medição virou script, como a Fase A fez com o ensaio
de marcha. Rasteriza as faces horizontais do STL (que são o topo e a base das
caixas de parede, logo a pegada), roda transformada de distância contra o raio
circunscrito do tronco, e devolve veredito, `<pose>`, `yaw:=` e metas em centro
de corredor. Reproduz os números do `maze10` já commitados.

**A pose sozinha não bastava.** No canto inferior direito a pista livre em `+x`
é de **16 cm**, e o robô nasce olhando para `+x`. Nascer de cara na parede
obriga a girar parado, que é o que este robô faz pior (teto de guinada de
~0,13 rad/s, eixo no batente). Então o mundo passou a **declarar o próprio yaw
de nascimento**, numa linha `go2_spawn_yaw`, e `run_quadruped_sim.sh` a lê e
passa como `yaw:=`. O número fica ao lado da geometria que o justifica, e não
há nada a lembrar na linha de comando.

`scripts/nav_trial.py` — ensaio de navegação por metas que **nunca publica
`/demo/cmd_vel`**, então pode medir com o Nav2 no comando. Mede velocidade
**média** (o pico já era conhecido e não é o que se sente), folga mínima pelo
lidar, e dobra as estatísticas do supervisor de marcha lidas do log do
container, casando pela posição no arquivo e não por timestamp.

Portão de F1 batido e medido: mundo carrega, malha resolve (0 falhas), robô de
pé, yaw **90,0°**, folga prevista 0,55 m contra `min` do lidar **0,55 m**,
**2581 pontos de obstáculo** na nuvem, `scenario_check` `PASSOU: 0 problema(s)`,
**RTF 1,000** — os 284 triângulos de colisão contra os 156 do `maze10` não
custaram tempo real, então comparações de marcha feitas aqui seguem válidas.

Três correções de passagem: o guard de `MAZE_MODELS` estava preso ao nome
literal `quadruped_maze.sdf` e deixaria cada labirinto novo reintroduzir a mesma
falha silenciosa (agora casa `quadruped_maze*.sdf`); `maze_nav_rviz.launch.py`
deixava `colcon test` de `demo_bringup` vermelho num `D213`; e o cabeçalho do
`quadruped_maze.sdf` apontava para `docs/results/ml35-labirinto.md` e para uma
tabela de poses no guia S6, **nenhuma das duas existindo** até aqui.

Registrado e não corrigido: `quadruped_maze.sdf` não é XML bem-formado —
comentário XML não aceita hífen duplo, e o cabeçalho dele usa. O TinyXML2 do
Gazebo tolera, e toda a evidência de S6 saiu dele. O arquivo novo é
bem-formado.

---

## 2026-08-19 — ML3.5 F4: sintonia da marcha vira parâmetro, ensaio vira script

Fase A do `docs/ml35/plano-movimentacao.md`. **Portão batido** — evidência em
`docs/results/ml35-f4-parcial.md` §"Fase A do plano de movimentação".

Toda a sintonia do trote saiu de literais em construtor e virou parâmetro do
controlador: período e razão de apoio da onda, altura de passo, os três ganhos de
Raibert, os ganhos de pose e de balanço, o envelope de comando, a banda de
referência, os quatro clamps entregues ao QP, a diagonal de pesos do QP e o cone
de atrito. Defaults iguais aos valores compilados, então o comportamento não
mudou.

O ponto de injeção **não** é `go2_description/config/gazebo.yaml`: aquele pacote
é vendorizado e o README dele garante os configs byte a byte, garantia que
sustenta o argumento de licença. O `<parameters>` do plugin de hardware também é
xacro vendorizado. O ponto que é nosso é o spawner, com `--param-file`, que
aplica os parâmetros **antes** de `load_controller` — que é quando `on_init()` os
lê. Aplicar depois seria aceito e nunca lido.

Provado com valores distinguíveis, não com valores iguais aos defaults:
`height=0.090 k=(0.0770 …) S_moment=(450 450 111)`. A última é uma entrada só do
vetor de momento, que é exatamente o que a Fase B1 precisa.

Junto: `scripts/gait_trial.py` + `scripts/gait_trial.sh`, versionados, que
abortam se `z <= 0,30` antes do ensaio e antes de cada ciclo, paginam as fases
por **tempo de simulação** com as duas bases de tempo no CSV, e param de publicar
em vez de publicar zeros — o watchdog é o mecanismo de parada.

**Duas conclusões da primeira versão do plano foram corrigidas antes de virarem
trabalho**, as duas desmentidas por medição que já estava no repositório:

- "a marcha não tem autoridade sobre velocidade" — falso: a Fase 1 de 18/08 mediu
  rastreamento de 97/105/111% em 0,05/0,10/0,20 m/s. O laço fecha pelo QP
  saturando contra `reference_band`, não pela colocação de pé; `k_x`/`k_y` em
  0,005 continua sendo lacuna real contra a literatura, mas é hipótese de
  qualidade, não correção de falha medida. Rebaixada de Fase B para Fase C.
- "a árvore TF fecha em simulação" — falso, e o oposto é o fato:
  `/go2/ground_truth_tf` não é bridgeado de propósito, não existe frame `odom`, a
  árvore flutua ancorada em `trunk`. É bloqueador confirmado de F5, e o item
  maior que resta — não o menor, como o plano dizia.

**Armadilha de medição encontrada:** `grep -c mode=RECOVER` sobre o log inteiro
não é a contagem do ensaio. Uma corrida devolveu 23 com tilt de pico de 2,58° na
janela; os 23 eram de 31 s depois do último ciclo, quando o robô tombou em HOLD
prolongado — o Defeito 2, dentro da faixa registrada de 18 a 46 s. Guia corrigido
com o recorte por janela, e o harness passa a reproduzir o Defeito 2 de graça.

---

## 2026-08-17 — ML3.5 F3: quadrúpede Go2 na árvore do projeto, em pé e andando

**Portão batido**, medido por `gz topic -e -t .../dynamic_pose/info`, nunca por
log: em pé a `z=0.352m`; andando com ganho baixo (`linear.x=0.03`) sustenta
`z=0.351m` e desloca `x` de 0.041 para 0.216 em 12s; orientação final `-7.2e-05`.
Zero erros no Gazebo, 3 controladores `active`. Melhor que F2, que via a altura
cair durante a marcha.

**F3 não foi retarget de cinemática.** A troca A1→Go2 de F2 eliminou esse
trabalho — o Go2 é o robô nativo da base upstream. F3 virou vendorização
criteriosa mais integração.

**A justificativa de licença de F2 estava incompleta.** F2 trocou o alvo
registrando que `go2_description` "declara BSD — licença válida". Na hora de
vendorizar, medido de novo: o pacote não tem arquivo `LICENSE`, não tem header
de copyright, e tem `<author>TODO</author>` com `TODO@email.com`. É
materialmente o mesmo estado do `a1_description` rejeitado, exceto por uma
string no `package.xml` — e pior num ponto, já que o A1 ao menos apontava um
maintainer rastreável. "BSD" sozinho não identifica a variante, e todas exigem
reproduzir um aviso de copyright que não existia ali.

Resolvido rastreando até a origem real: **`unitreerobotics/unitree_ros`,
BSD 3-Clause com texto completo e titular identificado** (HangZhou YuShu
TECHNOLOGY CO.,LTD., 2016-2022). As 7 malhas são **bit-idênticas** ao upstream,
provado por hash git blob contra a API do GitHub (`base.dae` renomeado para
`trunk.dae` é a única diferença). A camada xacro não bate — é port ROS 1 → ROS 2
do `legubiao` — e foi adotada como obra derivada coberta pelo BSD-3, com o risco
residual registrado, não apagado. Mesma correção na camada de controle: os
`package.xml` declaravam Apache-2.0, mas `LICENSES/unitree_guide/LICENSE.txt` da
raiz upstream é BSD-3 da Unitree, mesmo titular das malhas.

**Reversão da decisão do ML2, registrada como o plano exigia.** O ML2 decidiu
contra `gz_ros2_control` a favor do plugin nativo `gz-sim-diff-drive-system`,
para não arrastar `ros2_control` + `controller_manager`. ML3.5 reverte: um
quadrúpede não tem plugin nativo equivalente, as juntas são acionadas por um
controlador `ros2_control` cujo hardware interface carrega **dentro** do
processo do `gz sim`. É também por isso que `sim` é um container só e não pode
ser dividido. O plugin que roda é o `gz_quadruped_hardware` vendorizado (2.0.6),
**não** o `gz_ros2_control` 1.2.19 do apt — achado de F2, e o apt continua fora
da imagem de propósito, para não deixar um pacote sem uso fingindo ser o que
executa.

**A armadilha silenciosa desta fase.** A `TimerAction` de 12 s copiada do plant
diff-drive produz: spawn em `z=0.49999`, queda para `z=0.0677` em menos de 1s,
controladores ativando ~3s depois — o robô passa a janela toda em queda livre
sem controlador e desaba. Estado final: colapsado, **três controladores
`active`, zero erros no log**, FSM percorrendo `passive → trotting` sobre um
robô já caído. Nada no log denuncia. Só a pose lida direto do `gz`. Corrigido
com spawn imediato encadeado por `OnProcessExit`, sem timer, que é o que o
launch upstream já fazia. É a segunda vez no ML3.5 que um timer mede a coisa
errada — a primeira foi em F1, e gerou o `wait_for_clock`.

**Vendorizados** (5 pacotes, nomes upstream preservados para que os `$(find)`
resolvam sem edição): `go2_description` (25 MB), `control_input_msgs`,
`controller_common`, `unitree_guide_controller`, `gz_quadruped_hardware`.
Procedência em `go2_description/README.md` e
`unitree_guide_controller/PROVENANCE.md`. Lint de estilo desativado nos dois
pacotes C++: rodava sobre código de terceiro e produzia 98 falhas em código que
a política manda não editar.

**Nosso:** `demo_simulation/launch/quadruped.launch.py`, `twist_to_inputs.py`
(promovido do spike de F2), `sim.launch.py` roteando `robot_type` para um launch
por plant, sem conditionals. Default segue `diffdrive`.

**Confirmação visual posterior:** em 17/08/2026 o operador executou o mesmo
launch do projeto com `gui:=true world:=empty.sdf`, ainda na imagem do spike, e
confirmou o modelo Go2 visível no Gazebo. RViz2 não foi validado.

**Não validado:** nada em arm64, nada no módulo, RobotModel/TF no RViz2, imagem
`sim` oficial e o quadrúpede no `warehouse.sdf`. Marcha em ganho alto continua
instável — é sintonia do mapeamento, e é F4.

---

## 2026-08-14 — ML3.5 F2: alvo trocado de A1 para Go2; spike andando, portão batido

**Motivo da troca de alvo:** verificação de F2 na árvore clonada de
`legubiao/quadruped_ros2_control` achou `a1_description` com
`<license>TODO</license>` — sem header de copyright em nenhum arquivo, sem
cobertura da licença Apache-2.0 da raiz (que cobre só o código, não as
descrições de robô). `go2_description` declara BSD. Como F3 vendoriza
justamente a descrição do robô-alvo, seguir com A1 teria sido vendorizar
arquivo sem licença clara numa demo comercial da Toradex — o mesmo critério que
já eliminou o Tugbot no ML3.1 e dois forks Go2 na escolha da base. Decisão:
**demo passa a usar Go2**, não A1. Nada em F2 a F5 tecnicamente exige um
quadrúpede específico; o contrato de tópicos (F4) não distingue os dois.

**Portão de F2 batido:** Go2 upstream, sem modificação, em pé e andando por
`cmd_vel`, medido por pose real (`gz topic -e -t .../dynamic_pose/info`), não
por ausência de erro em log. Em pé sustentado a `z=0.353m`; andando com ganho
baixo (`linear.x=0.03`) sustenta `z=0.343m` por 8s contínuos sem cair. Zero
erros no log da execução.

Spike descartável (`demo-sim:spike-go2`, `/tmp/f2-spike`, **não commitado** —
nada disto entra na árvore do projeto). Clone raso de `quadruped_ros2_control`;
pacotes que o spike não builda (`ocs2_quadruped_controller`,
`rl_quadruped_controller`, `magicdog_description`, etc.) removidos antes do
`rosdep install` — remoção do que não se builda, nada do que o spike usa foi
tocado. Build: `go2_description`, `unitree_guide_controller` (PD clássico, sem
policy RL), `keyboard_input`, `gz_quadruped_playground`.

**Achado que muda o plano de F4:** o controlador não fala `geometry_msgs/Twist`
nativamente. Usa `control_input_msgs/Inputs` (formato joystick: `lx/ly/rx/ry` +
`command` de estado). F4 (contrato de tópicos) precisará de uma ponte
Twist→Inputs real — o spike escreveu uma versão descartável só para testar o
portão, com mapeamento linear ingênuo e sem os limites de velocidade que uma UI
de joystick real respeitaria. É a causa provável de o robô cair em ganho alto
(`linear.x>=0.15`): sintonia de escala, não falha estrutural — coerente com o
próprio guia, que já registrava esse risco como fora do escopo de F2.

**Achado técnico:** `gz_quadruped_hardware` é do próprio repo (2.0.6, fork do
`gz_ros2_control` upstream, Apache-2.0), **não** o `gz_ros2_control` 1.2.19 do
apt que o plano original supunha. Confirmar qual entra na imagem `sim` real
antes de F3.

**Armadilha de metodologia registrada:** testar a FSM manualmente via
`ros2 topic pub .../control_input` enquanto a ponte de spike do launch ainda
publicava em paralelo produziu dois publishers competindo e um retrocesso de
estado que parecia o controlador instável e não era. Diagnosticado lendo
`StateTrotting::checkChange()` — `command==2` força volta a `FIXEDSTAND` mesmo
em trote estável. Corrigido isolando um único publisher por teste.

Detalhe completo, com a tabela pose-por-momento e a colisão CycloneDDS x
`unitree_sdk2` (segue não bloqueando, container `hw` já existe): `docs/ml35/estado-fases.md`.

---

## 2026-08-14 — ML3.5 F1: baseline diff-drive inteira em containers

**Portão batido:** goal Nav2 `SUCCEEDED` (`error_code: 0`) com a demo rodando em
containers, enviado do container `tools`. `colcon build` limpo nos 6 pacotes,
`colcon test` **46 testes / 0 falhas** (eram 39; +7 do `wait_for_clock`).
Evidência em `docs/results/ml35-f1-execucao.md`.

**Criado:** `docker/{base,sim,nav,perception,viz,tools}/Dockerfile`,
`docker/hw/README.md`, `docker/compose.{host,module}.yml`,
`docker/cyclonedds/{host,module}.xml`, `docker/entrypoint.sh`,
`docker/.env.example`, e os quatro launch files de papel em `demo_bringup`
(`sim`, `nav`, `perception`, `viz`). `learn.launch.py` segue como a composição
nativa não-containerizada.

### Timers de wall-clock não sobrevivem à fronteira de container

O risco número um do plano se confirmou. Os delays por timer do `learn.launch.py`
(20 s perception, 25 s nav) medem tempo desde a subida do **próprio** container,
que não tem relação fixa com o instante em que o Gazebo terminou de carregar o
mundo — `docker compose up` sobe tudo junto.

Substituídos pelo nó **`wait_for_clock`**, que espera `/clock` existir **e
avançar**. Duas amostras com timestamp estritamente crescente: uma só passaria com
Gazebo pausado (`gz sim` sem `-r` inicia pausado), trocando uma falha silenciosa
por outra. Timeout de 120 s com saída != 0 — container que espera para sempre
parece travamento, não falha.

Isto **ampliou o escopo** de F1 além do "nenhuma mudança de comportamento" do
portão: é código novo, não só empacotamento. Decisão do operador, com a
alternativa (portar os timers como estavam) na mesa. Os 12 s de spawn e 15 s de
bridge **dentro** de `simulation.launch.py` continuam intocados — são
intra-container, e ali o timer ainda mede o que deve.

### Vendorização de quatro launch files do Nav2 (regra 1)

`ros-jazzy-nav2-bringup` **hard-depends** de `nav2-minimal-tb3-sim`,
`nav2-minimal-tb4-sim`, `ros-gz-sim` e `navigation2`. Medido: colocou
`libogre-1.9`, `gz-ogre-next-vendor`, `gz-rendering`, `gz-gui` e 30+ pacotes na
imagem `nav` — **3,7 GB e OGRE 2 numa imagem que vai para o AM69**, violação
direta da regra 1. `--no-install-recommends` não ajuda: são `Depends`. O
metapacote `ros-jazzy-navigation2` repete o problema um nível abaixo, via
`nav2-rviz-plugins` → `rviz-ogre-vendor`.

Os quatro launch files necessários (`bringup`, `localization`, `navigation`,
`slam`) estão vendorizados em `demo_navigation/launch/nav2_vendored/`,
Apache-2.0, cabeçalhos de copyright intactos, **só os caminhos de raiz de pacote
re-rooteados**. Os servidores Nav2 entram individualmente no Dockerfile.
Resultado: `nav` de 3,7 GB → **2,48 GB**, zero pacotes OGRE/RViz/Gazebo.

**Custo aceito:** a lista de servidores no `nav/Dockerfile` e em
`demo_navigation/package.xml` agora acopla com `nav2_params.yaml`. Plugin novo de
pacote não listado exige crescer as duas listas; está comentado nos dois lugares.

### Três armadilhas silenciosas, todas registradas em estado-fases.md

1. **`${HOST_IP}` em arquivo bind-mounted nunca expande.** Docker não substitui
   variáveis dentro de arquivo montado; o CycloneDDS recebeu a string literal como
   endereço. Com `AllowMulticast=false`, **nenhum mecanismo de descoberta sobrou**.
   Sintoma: `ros2 node list` vazio e o spawner do Gazebo esperando
   `robot_description` para sempre. Correção: `<Peer address="127.0.0.1"/>`, que é
   load-bearing, não redundante.
2. **`GZ_SIM_RESOURCE_PATH` vazio no container.** Robô spawnava com colisão e
   inércia corretas — física e navegação funcionando — e **sem corpo visível**.
3. **Mapear `/dev/dri` não basta.** `renderD128` é do grupo `render` (gid 992
   neste host) e o usuário do container está em `video`: queda silenciosa para
   render em software. Corrigido com `group_add`.

### Pendências abertas

- `docker/.env.example` não documenta `RENDER_GID`; o arquivo está bloqueado por
  regra de permissão do ambiente. Correção manual do operador, texto pronto em
  `docs/ml35/estado-fases.md`.
- **Nada em arm64 foi construído ou executado**, e o módulo não esteve acessível.
  `compose.module.yml` e `cyclonedds/module.xml` são código não executado
  (regras 5 e 7).
- Confirmação visual em GUI segue **não feita por olho humano**, herdada do ML3.1.

---

## 2026-08-14 — ML3.5 F0: ponto de retorno e spec da containerização

**Motivo:** o robô-alvo da demo é quadrúpede. O operador escolheu a **opção C**,
locomoção por pernas real, com o custo declarado de semanas e risco de não
convergir — não o quadrúpede apenas visual sobre diff-drive.

**Entregue nesta sessão:** F0 (commit `3885f2e`, ver entrada do ML3.1 abaixo)
mais a spec e o documento de continuidade do ML3.5.

### A página que originou a task não é o caminho

O pedido veio com `docs.quadruped.de/projects/a1/html/simulation.html`. Lida: é
**ROS 1** (`roslaunch`), Gazebo **Classic** ou Webots, e stack de controle
própria (`state_estimator`, `quadruped_controller`) **sem Nav2**. Ela define o
*objetivo* — A1 quadrúpede — não o *caminho*. ROS 1 → ROS 2 não é porte de launch
file, e Classic → Harmonic troca engine, formato SDF e sistema de plugins.

### Continua não existindo quadrúpede pronto em Jazzy + Harmonic + Nav2

Reverificado em 14/08/2026, e a conclusão do ML2 se mantém. Além do que o ML2 já
registrava, `legubiao/quadruped_ros2_control` (Apache-2.0, `ros2_control`
nativo, branch default Jazzy) foi adotado como base por ser integração e não
reescrita. Os dois forks Go2 em Jazzy seguem com Nav2 "coming soon" **e sem
licença declarada** — bloqueador para demo comercial, o mesmo critério que
eliminou o Tugbot no ML3.1.

Tudo o que se afirma sobre essa base vem do README e **está marcado para
confirmação na árvore em F2**, não como fato. A integração com Nav2 é nossa;
ninguém entrega pronta.

### Decisões de estrutura

- **Compose passa a ser dividido por máquina, não por modo.**
  `docker/compose.{host,module}.yml` no lugar de `compose/{learn,emul,target}.yaml`.
  Os três arquivos antigos estavam vazios e foram removidos; `CLAUDE.md` e
  `.ai/CLAUDE.md` reconciliados. O modo `emul` caiu junto: imagens arm64 seguem
  construídas sob QEMU, sem compose dedicado para rodar a stack emulada.
- **F1 containeriza o diff-drive antes de trocar o robô.** Se o compose quebrar
  depois que o quadrúpede entrar, não se sabe se foi Docker, DDS ou marcha.
  Containerizando o que já funciona, F2 falha por um motivo só — e se F2 falhar,
  o trabalho de F1 continua valendo para a demo diff-drive.
- **F2/F3 revertem a decisão do ML2 contra `gz_ros2_control`.** Ela foi tomada
  porque o plugin nativo `gz-sim-diff-drive-system` bastava para rodas. Quadrúpede
  não tem equivalente nativo. Registrar aqui para que a reversão não pareça
  esquecimento.

### Colisão de invariantes, registrada antes de doer

`quadruped_ros2_control` documenta conflito entre **CycloneDDS e `unitree_sdk2`**
e recomenda FastDDS. A regra inviolável 2 do projeto é `rmw_cyclonedds_cpp`
sempre. Não bloqueia o ML3.5 — o SDK só entra com A1 físico, fora do escopo — mas
o container `hw` existe vazio desde F1 para o problema aparecer no lugar certo em
vez de surgir como surpresa no bring-up de hardware.

### Documentos

- `docs/ml35/estado-fases.md` — continuidade entre sessões: estado de F0 a F6,
  portão de cada fase, decisões, premissas em vigor e o que confirmar em F2.
- `docs/ml35/guia-ml35-docker.md` — a spec de implementação.

---

## 2026-08-10 — ML3.1: aparência do robô e guia de operação

**Motivo:** a demo tem público externo (cliente, feira, vídeo). O robô era
caixa + dois cilindros + esfera — funcionalmente correto, mas não sustenta uma
apresentação.

**Entregue:** `demo_robot.urdf.xacro` virou um wrapper fino sobre o TurtleBot 4
upstream, três correções pendentes do ML3, `docs/guia-operacao.md` e
`docs/analise-sensores-navegacao.md`.

### A causa real do "robô com partes separadas" era o RViz, não o modelo

Registrado primeiro porque foi o que custou mais tempo, e porque a ordem em que
descobrimos foi a errada.

O sintoma era um amontoado de peças flutuando em vez de um robô. **Duas trocas
de modelo e uma correção de física foram gastas perseguindo isso antes de
alguém abrir a configuração do RViz.** O `nav2_default_view.rviz` do
`nav2_bringup`, que o `learn.launch.py` usava, vem com:

| Display | Valor upstream | Efeito na tela |
| --- | --- | --- |
| `RobotModel` | `Enabled: false` | **o corpo do robô não é desenhado** |
| `TF` | `Enabled: true`, Show Axes + Names | **33 triedros rotulados** flutuando |

Ou seja: nenhum corpo, e 33 marcadores de eixo espalhados pelos links. Isso é
indistinguível, a olho, de um robô mal montado.

Daí `demo_bringup/rviz/demo_view.rviz`, com `RobotModel` ligado e `TF` desligado,
e o cabeçalho do arquivo documentando as duas inversões contra o upstream.

**Lição, e é a mesma do ML3:** o sintoma apareceu na camada de visualização e foi
tratado como se fosse da camada de modelo. Antes de mexer no URDF por causa de
algo que se vê na tela, verifique o que a tela foi configurada para desenhar.

### O modelo agora é wrapper do TurtleBot 4 upstream

A versão anterior montava o robô peça por peça a partir dos meshes individuais
do TB4, com offsets derivados de medição de bounding box. Essa abordagem foi
**abandonada**: cada mesh carrega origem e rotação internas próprias, então
reparentar as peças numa árvore de links nossa espalhava o conjunto, e cada nova
medição consertava uma peça movendo outra.

`demo_robot.urdf.xacro` hoje inclui o modelo já montado:

```
nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro
```

Mantido pela equipe Nav2, offsets corretos por construção, e traz DiffDrive +
JointStatePublisher mais um conjunto de sensores mais rico (RPLIDAR A1, OAK-D
RGBD, IMU) do que o que tínhamos. Os quatro sub-xacros da montagem manual
(`_wheel`, `_sensors`, `_inertia`, `_materials`) foram removidos.

Nenhum binário entra no repo: as meshes são resolvidas por `package://`, e
`nav2_minimal_tb4_sim` já era dependência por causa do mundo do armazém.

**Não re-adicione meshes peça a peça aqui.** Se o robô parecer errado, a ordem é:
conferir o RViz primeiro, depois sobrescrever uma junta abaixo do include.

### O que o wrapper sobrescreve, e uma armadilha do xacro

Upstream é um TurtleBot 4, não "o nosso" robô. O ponto que não é óbvio:

**Re-declarar o plugin DiffDrive não o substitui.** O xacro não funde nem
substitui blocos `<gazebo>` — ele **concatena**. Tentar sobrescrever produz uma
URDF expandida com **dois** plugins DiffDrive dirigindo as mesmas duas juntas.
Foi tentado e rejeitado.

Como o `child_frame_id` é hard-coded em `create3.urdf.xacro` sem argumento de
xacro, a reconciliação foi feita do lado do Nav2: `robot_base_frame: base_link`
em `nav2_params.yaml`. Isso é seguro, não concessão — `base_footprint_joint` é
uma transformada identidade, então `odom -> base_link` e `odom -> base_footprint`
são numericamente a mesma aresta.

Teste `test_exactly_one_of_each_gz_system_plugin` conta **elementos**, não
ocorrências de texto: qualquer grep também casa com os comentários que explicam
a armadilha.

### Três correções pendentes do ML3

1. **`worlds/warehouse.sdf` não existia.** Era o default dos dois launch files, e
   `setup.py` instalava `glob('worlds/*.sdf')`, que casava com nada. Funcionava
   nesta máquina só porque o operador copiara o arquivo localmente; um clone
   limpo falhava com erro do Gazebo que não nomeia o arquivo ausente. Default
   agora resolve para `nav2_minimal_tb4_sim`, e a dependência foi declarada.
2. **Três comentários afirmavam o oposto do descoberto no ML3** — que os tópicos
   do DiffDrive são escopados em `/model/<name>/`. Era exatamente a armadilha
   corrigida em `da53167`; quem lesse "consertaria" o YAML de volta. Corrigidos
   em `simulation.launch.py`, `bridge_warehouse.yaml` e no URDF.
3. **`xacro_args` não existia** em `view_robot.launch.py`, e `use_meshes` não era
   propagado por `simulation.launch.py`. Ambos implementados.

### `weld_fixed_joints.py`

Expande o xacro e remove `<preserveFixedJoint>` para que o Gazebo solde as juntas
fixas. Detalhe que custa uma sessão de debug se ignorado: **em sucesso o script é
silencioso**, porque a substituição `Command` do launch aborta o launch inteiro
se o comando escrever qualquer coisa em stderr.

### Avaliado e descartado: modelos prontos do Gazebo Fuel

**Tugbot (MovAi)** era o melhor candidato — AMR de armazém completo, com
DiffDrive, lidars, câmeras RGBD e IMU montados. **Descartado por licença:** o
`model.config` não declara nenhuma, mas a API do Fuel informa **CC BY-NC-ND 4.0**.
Para demo comercial, `NonCommercial` e `NoDerivatives` são ambos bloqueadores.

**MARBLE_HUSKY** é CC BY 4.0, sem impedimento legal, mas é skid-steer de 4 rodas
**sem plugin DiffDrive** e usa `gpu_ray`, nome do Fortress removido no Harmonic.

Fica registrado: a licença de um modelo do Fuel **não está no `model.config`**;
consulte `https://fuel.gazebosim.org/1.0/<owner>/models/<nome>`.

### Verificado

| Verificação | Resultado |
| --- | --- |
| `colcon build` | 6 pacotes, limpo |
| `colcon test` | 39 testes, 0 falhas |
| Launch files geram `LaunchDescription` | OK |
| `check_urdf`, meshes e fallback | ambos os caminhos parseiam |
| Meshes resolvem por `package://` | OK |
| World default resolve | OK |
| Robô carrega no Gazebo headless | OK — tópicos do contrato publicando |
| Locomoção | OK — `/demo/cmd_vel` moveu o robô, `y ≈ 0` |

**NÃO verificado:** confirmação visual em RViz2/Gazebo com GUI, que depende de
sessão gráfica interativa. Pendente do operador — ver `docs/guia-operacao.md`.

### Documentação

`docs/guia-operacao.md` — guia de operação e edição, escrito para quem chega sem
experiência prévia de ROS 2. Cada comando documentado foi executado antes de
entrar no arquivo; foi assim que as três correções acima apareceram.

`docs/analise-sensores-navegacao.md` — o que exatamente sai do Gazebo, por onde
passa até o Nav2, e como o comando volta aos atuadores. Números medidos, não
copiados de documentação.
---

## 2026-08-10 — ML3: simulação e navegação nativas (concluída)

**Entregue:** quatro pacotes novos — `demo_simulation`, `demo_navigation`,
`demo_perception`, `demo_bringup`. Tudo no host x86; nada toca o módulo.

| Pacote | Conteúdo |
| --- | --- |
| `demo_simulation` | `simulation.launch.py` (gz sim + spawn + bridge), `teleop.launch.py`, `config/bridge_warehouse.yaml` (8 mapeamentos) |
| `demo_navigation` | `config/nav2_params.yaml` (13 blocos de servidor), `navigation.launch.py`, `slam.launch.py` |
| `demo_perception` | `detection_stub`, `detections_to_cloud`, `perception.launch.py` |
| `demo_bringup` | `learn.launch.py` com ordenação por timer |

**Verificado nesta máquina:**

| Verificação | Resultado |
| --- | --- |
| `colcon build` | 6 pacotes, limpo |
| `colcon test` | 36 testes, 0 falhas (16 novos em `demo_perception`) |
| Os 6 launch files geram `LaunchDescription` | OK |
| xacro expande com plugins gz | OK, 320 linhas |
| YAML do Nav2 e do bridge parseiam | OK |
| Pipeline de percepção ao vivo | 10 imagens → 10 detecções → 10 nuvens; header preservado; 5 pontos empilhados a 2.0 m |

**Aceitação do ML3 — executada de verdade em 10/08/2026**, depois que o operador
instalou os pacotes do Nav2:

| Critério | Resultado |
| --- | --- |
| Robô teleoperável | OK — `/demo/cmd_vel` levou o robô de (0,0) a (0.82, 2.31) |
| Odom, scan, TF, comandos trocam mensagens | OK — `/demo/odom` 27.6 Hz, `/demo/scan` 9.97 Hz, `/joint_states` ativo, `odom→base_footprint` e `base_link→laser_frame` corretos |
| Nav2 em estado `active` | OK — os 7 servidores (`map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `velocity_smoother`) |
| Goal concluído | OK — `SUCCEEDED`, `error_code: 0`, (0,0) → (2.43, 0.20) para um goal em (2.0, 0.5) |

Extra verificado: a costura percepção→costmap está viva sob o Nav2 —
`/demo/perception/detection_cloud` a 15.15 Hz com **`Subscription count: 2`**,
os dois costmaps consumindo. Mapa do armazém gerado por SLAM (477×475 células,
5 cm) e commitado em `demo_navigation/maps/`.

### Cinco falhas silenciosas encontradas na primeira execução

Todas tinham a mesma assinatura: nenhum erro em lugar nenhum, tudo "parecendo"
funcionar. Registradas porque cada uma custou tempo e todas voltam se alguém
"limpar" o código.

1. **Spawn cedo demais.** `ros_gz_sim create` chama primeiro o serviço de lista
   de mundos; em t=0 ele não existe e o cliente **retenta a cada 5 s para
   sempre** em vez de falhar. O robô ainda aparece no mundo (o create assíncrono
   aceita), então `gz model --list` mostra `demo_robot` e os sensores publicam —
   mas os plugins DiffDrive e JointStatePublisher nunca inicializam. Corrigido
   com `TimerAction` de 12 s (spawn) e 15 s (bridge).

2. **Nomes de tópico gz não são escopados.** Os elementos `<topic>`,
   `<odom_topic>` e `<tf_topic>` do DiffDrive são **literais**: o plugin escuta
   em `/cmd_vel` e publica em `/odom` e `/tf`, sem prefixo de modelo. O Gazebo
   *também* anuncia `/model/demo_robot/{cmd_vel,odom,tf}` como nomes padrão —
   eles aparecem em `gz topic -l` e parecem certos, mas não têm ninguém
   conectado. A `bridge_warehouse.yaml` apontava para os escopados: robô não
   andava, odom não publicava, zero erros. Diagnóstico veio de
   `gz topic -i -t ...` → `No subscribers on topic`. Mesmo problema no
   `joint_state` (que ainda por cima embutia o nome do mundo).

3. **slam_toolbox é lifecycle node.** Sobe em `unconfigured` e fica lá. O
   processo roda, loga "Node using stack size 40000000", e **não cria assinatura
   de scan nem publica mapa ou `map→odom`**. `ros2 node info` mostrava só
   `/clock`. Corrigido com `LifecycleNode` + `EmitEvent`/`OnStateTransition`
   encadeados (activate só depois de configure OK).

4. **`docking_server` sem `dock_plugins` derruba o bringup inteiro.** Faz parte
   da lista padrão do Nav2 Jazzy; sem configuração ele falha no configure e o
   lifecycle manager **aborta tudo** — `map_server` e `amcl` ficavam `active` e
   o resto parado em `inactive`. A demo não tem dock; configurado o mínimo.

5. **Lista YAML vazia quebra o launch.** `docks: []` chega ao launch como tupla
   Python e aborta com `Expected 'value' to be one of [float, int, str, bool,
   bytes], but got '()'`. A chave tem de ser omitida, não esvaziada.

Correção de rumo: o comentário original em `slam.launch.py` dizia que parâmetros
inline bastavam "por ser uma execução descartável". Estava errado — viraram
`config/slam_params.yaml`, e o `scan_topic` acabou resolvido por **remap**, que
o rclcpp aplica antes de o nó criar a assinatura.

### Decisão: adaptador `PointCloud2` em vez de plugin C++ de costmap

O `CLAUDE.md` exige que as detecções alimentem uma camada de costmap do Nav2, não
só a tela da HMI. A `ObstacleLayer` de fábrica lê `LaserScan` ou `PointCloud2` —
não lê `Detection2DArray`. As duas saídas eram um plugin C++ de costmap ou a
conversão para uma mensagem que a camada já aceita.

Escolhido o adaptador em Python (`detections_to_cloud`), porque a convenção do
projeto é Python por padrão e C++ só onde houver desempenho medido no hardware —
e nada foi medido no AM69 ainda. Trocar por um plugin C++ nativo depois não muda
nada dos dois lados: `demo_perception` continua publicando `Detection2DArray` e o
Nav2 continua marcando as mesmas células.

**Limitação assumida:** uma bounding box 2D não carrega profundidade. O adaptador
assume distância fixa (`assumed_range_m`, 2.0 m) e usa o modelo pinhole para
converter a posição horizontal em azimute. É honesto para um stub e suficiente
para provar a costura do costmap; não substitui profundidade. Daí
`clearing: false` (a projeção é grosseira demais para apagar obstáculos reais do
lidar) e `observation_persistence: 1.0` (detecções expiram em vez de deixar
rastro de obstáculos fantasma).

### Nota: mundo de armazém não vendorizado

O `warehouse.sdf` vem do pacote de sistema `nav2_minimal_tb4_sim`, não copiado
para o repo. `demo_simulation/worlds/` tem só um `.gitkeep` explicando como
apontar o launch para a cópia instalada.

### Correção de escopo

A tabela de contrato de tópicos em `.ai/CLAUDE.md` ainda listava
`/camera/image_raw`, `/perception/detections` e `/cmd_vel` sem o prefixo
`/demo`, divergindo da convenção de namespace da §5.2 do `AGENTS.md` e do que o
código implementa. Alinhada.

---

## 2026-08-07 — ML2: modelo de robô e TF

**Entregue:** pacote `demo_description` — xacro diff-drive parametrizado,
árvore TF, config RViz de desenvolvimento (x86), testes de URDF.

Arquivos: `urdf/{demo_robot.urdf.xacro,_materials,_inertia,_wheel,_sensors}.xacro`,
`launch/view_robot.launch.py`, `rviz/demo_description.rviz`,
`test/test_urdf_parses.py`, `README.md`.

**Aceitação ML2 (`.ai/AGENTS.md` §9):**

| Critério | Estado |
| --- | --- |
| xacro expande sem erro | OK |
| frames obrigatórios existem | OK — 7 links, 6 joints, raiz única `base_footprint` |
| sem publicador TF duplicado | OK — tabela de posse no README |
| RViz funciona no x86 | Config válida e launch importável; **confirmação visual pendente do operador** |
| `colcon test` | 13 testes no workspace, 0 falhas |

### Decisão: diff-drive agora, quadrúpede em ML3.5

O robô-alvo da demo é um quadrúpede. ML2/ML3 entregam base diff-drive de
propósito. Em agosto/2026 nenhum projeto mantido entrega
quadrúpede + Jazzy + Gazebo Harmonic + Nav2 funcionando:

- `chvmp/champ` upstream é ROS 1 apenas (último commit jul/2024, `move_base`/`amcl`).
- Melhor base Jazzy+Harmonic (`khaledgabr77/unitree_go2_ros2`) lista Nav2 como
  "coming soon" desde mai/2025 — não entregue em ~15 meses.
- Único CHAMP+Nav2 demonstrado (`arjun-sadananda/go2_nav2_ros2`) é Humble +
  Gazebo **Classic**, e compensa erro de odometria dobrando a velocidade linear
  no estimador de estado — contorno, não calibração.

Odometria por dead-reckoning de cinemática de marcha deriva muito mais que
odometria de rodas, e a localização do Nav2 depende disso. Assumir esse risco
primeiro bloquearia containers, emulação arm64 e bring-up de hardware — o
objetivo real da demo.

O contrato de tópicos torna a decisão reversível: tudo a jusante fala
`/demo/cmd_vel` (`geometry_msgs/Twist`), então trocar por quadrúpede depois fica
confinado a `demo_description` + `demo_simulation`. Nenhuma mudança em Nav2,
percepção, HMI ou containers. Rastreado como ML3.5.

### Decisão: plugin nativo `gz-sim-diff-drive-system`, não `gz_ros2_control`

O `.so` já vem com o Gazebo Harmonic do host (verificado:
`libgz-sim8-diff-drive-system.so.8.14.0`), custo zero de pacotes. `gz_ros2_control`
arrastaria `ros2_control` + `controller_manager` + `diff_drive_controller` —
superfície muito maior para um milestone cuja aceitação é "teleop, tópicos
trocam mensagens, Nav2 ativo, goal completa". É o padrão de `nav2_minimal_tb*_sim`
e do `diff_drive.sdf` do próprio Gazebo. Migrar depois não altera nada a jusante.

### Correções de infraestrutura (pré-existentes)

- **Symlinks quebrados** em `ros2_ws/install/`: 4 apontavam para
  `/home/diego-maia/toradex/custom-cases/2608-demo-aquila-ros2/`, caminho onde o
  repo vivia antes. Quebravam `colcon build` do `demo_tutorials`. Removidos.
- **`.gitignore` ausente**: `build/`, `install/` e `log/` estavam a caminho de
  serem commitados. Com `--symlink-install` esses diretórios contêm caminhos
  absolutos — commitá-los reproduz exatamente o bug acima em outra máquina.
  Criado.

### Pendente para o operador

```bash
sudo apt install -y liburdfdom-tools    # check_urdf (sudo precisa de senha)
ros2 launch demo_description view_robot.launch.py   # confirmação visual no RViz
```

---

## 2026-07-31 — L1: fundamentos ROS 2 (ML1)

Pacote `demo_tutorials`: heartbeat publisher/subscriber em
`/demo/system/heartbeat` com `rate_hz` retunável em runtime, serviço
`AddTwoInts`, `heartbeat.launch.py` e `turtlesim_demo.launch.py`, testes
unitários + flake8 + pep257.

Fases L0a (ROS 2 Jazzy + turtlesim), L0b (Gazebo Harmonic standalone) e
L0c (`ros_gz_bridge`) concluídas antes, documentadas em `docs/development.md`.
