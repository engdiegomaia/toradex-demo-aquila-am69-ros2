# ML3.5 — HIL por Ethernet no Aquila AM69

**Data:** 24/08/2026
**Hardware:** Aquila AM69, Torizon OS 7.7.0+build.40, 8 × Cortex-A72, 31 GiB
**Topologia:** host `enp0s31f6` (`<HOST_IP>`) ↔ switch ↔ Aquila
`ethernet1` (`<MODULE_IP>`), 1 Gbit/s físico
**Plant e telas:** Gazebo, RViz e câmera no host x86
**Aplicação:** Nav2 e `demo_perception` arm64 no Aquila

Esta é evidência de execução no AM69 real. O robô e os sensores continuam sendo
simulados no host; portanto o ensaio não valida localização por pernas nem um
Go2 físico.

## Resultado executivo

O HIL Ethernet está funcional e uma meta Nav2 fechou no módulo, mas o portão
operacional de 8 m ainda não passou.

- imagem RAW preservada em 640×480 rgb8 a aproximadamente 10 Hz;
- Nav2 e percepção ativos simultaneamente no Aquila;
- mapa/navegação e LiDAR abertos no RViz do host; câmera aberta também em
  `rqt_image_view` no host;
- meta curta `(-1,60; 1,60)` concluída em 28 s;
- protocolo longo de 420 s: 8,31 m percorridos, sem queda, mas nenhuma das metas
  de 8 m concluiu dentro do prazo de 200 s.

Assim, Ethernet, DDS, percepção e controle fechado estão comprovados. F5
permanece em andamento porque o roteiro longo, escolhido antes desta medição
como portão final, falhou.

A visualização ainda tem dois defeitos host-side: o `robot_description` contém
caminhos absolutos `/test/install/...` produzidos dentro do container de
simulação, inexistentes no container `viz`, e o driver registrou falha de link
no shader `indexed_8bit_image` do mapa. O RViz abriu, carregou os plugins Nav2 e
assinou o LiDAR; a câmera foi mostrada numa janela dedicada de `rqt_image_view`.
Esses defeitos não mudam as medições HIL, mas a tela integrada ainda precisa de
correção visual.

## Configuração de rede que faltava

As duas portas do Aquila estavam ligadas ao mesmo switch e à mesma sub-rede. As
duas anunciam `<MODULE_HOST>`; durante o bring-up, a resolução mDNS
alternou entre `<MODULE_ALT_IP>` e `<MODULE_IP>`. Isso também produziu ARP flux: o
host chegou a aprender os dois endereços no MAC de `ethernet1`.

O ensaio fixou explicitamente, no `docker/.env` local e ignorado pelo Git:

- `MODULE_IP=<MODULE_IP>`;
- `HOST_IP=<HOST_IP>`.

Depois de `./scripts/module.sh sync`, os XMLs renderizados ficaram coerentes:

- host: interface `enp0s31f6`, peer `<MODULE_IP>`;
- módulo: interface `ethernet1`, peer `<HOST_IP>`;
- `ROS_DOMAIN_ID=69`, `rmw_cyclonedds_cpp`, multicast desativado e localhost
  preservado para descoberta entre containers da mesma máquina.

Não foi necessário overlay, device tree ou jumper. Com duas NICs na mesma
sub-rede, a seleção explícita da porta é necessária; em instalação definitiva,
usar uma só porta ou sub-redes distintas evita a ambiguidade.

## Dois defeitos de QoS encontrados no HIL

O enlace físico e a descoberta DDS estavam corretos, mas descoberta não garante
entrega de uma amostra fragmentada.

### Câmera

O `ros_gz_bridge` publica `/demo/camera/image_raw` como `RELIABLE`. O
`detection_stub` pedia `BEST_EFFORT`; os endpoints apareciam no grafo, mas o
assinante perdia todos os frames de 921600 bytes no HIL. Uma sonda confiável
recebia o fluxo completo.

Correção: a assinatura do `detection_stub` passou a `RELIABLE`. Depois do
rebuild arm64, câmera, `/demo/perception/detections` e
`/demo/perception/detection_cloud` convergiram para aproximadamente 10 Hz. A
câmera mediu 0,92 MB por mensagem e 9,32 MB/s, cerca de 74,6 Mbit/s.

### Nuvem LiDAR

O bridge também publicava `/demo/scan_cloud` como `RELIABLE`, enquanto
costmaps e `collision_monitor` do Nav2 usam o perfil sensor-data
`BEST_EFFORT`. No enlace HIL, esses leitores deixaram de atualizar e o
`collision_monitor` rejeitou todos os comandos como fonte antiga.

Correção: somente a nuvem LiDAR no bridge recebeu
`qos_profile: SENSOR_DATA`. O produtor e os três consumidores Nav2 passaram a
`BEST_EFFORT`; uma sonda de 15 s percorreu 0,38 m e, na execução final, houve
zero aviso `Ignoring the source`.

## Medições

### Protocolo longo, preservado

Comando: `nav_trial.py --seconds 420 --goal-timeout 200`, metas padrão do
`maze11`, Nav2 + percepção no Aquila e as telas no host.

| Medida | Resultado |
|---|---:|
| tempo de simulação | 419,9 s |
| amostras | 4200 |
| caminho percorrido | 8,31 m |
| deslocamento líquido | 3,38 m |
| velocidade média | 0,0198 m/s |
| `cmd_vx` pico / médio | 0,140 / 0,0135 m/s |
| ré | 0% |
| inclinação máxima | 0,66° |
| folga mínima de carcaça | +0,065 m |
| metas | 0 concluídas; 2 prazos, 1 falha registrada |
| supervisor de marcha | 1588 linhas, `RECOVER=0`, `yawSat` médio 7%, pico 94% |

CSV bruto: `docs/results/ml35-hil-ethernet.csv`.

O resultado praticamente repete o HIL por Wi-Fi com percepção de 21/08
(0,0197 m/s). Portanto a hipótese anterior “o Wi-Fi é o gargalo” caiu: a câmera
é o maior fator de carga, mas trocar apenas o meio físico não remove o custo de
serialização, fragmentação, cópia e processamento do fluxo.

### Meta curta de fechamento

Para separar falha do roteiro longo de incapacidade geral do Nav2, foi enviada
a meta `(-1,60; 1,60)` a partir de aproximadamente `(-3,31; 0,81)`.

| Medida | Resultado |
|---|---:|
| primeiro `SUCCEEDED` | 28 s |
| caminho / deslocamento | 2,13 / 1,74 m |
| velocidade média | 0,0178 m/s |
| inclinação máxima | 0,70° |
| folga mínima de carcaça | +0,067 m |

O script repete a lista de metas; como ela continha um único ponto já atingido,
registrou depois 45 sucessos adicionais a cada intervalo de assentamento. O
número significativo é o primeiro fechamento em 28 s. CSV bruto:
`docs/results/ml35-hil-ethernet-short-goal.csv`.

### Recursos no Aquila

Valores instantâneos durante os ensaios, não máximos:

| Serviço | CPU observada | Memória observada |
|---|---:|---:|
| Nav2 | 493–530% | 307–341 MiB |
| percepção | 189–205% | 112–273 MiB |

As quatro imagens arm64 foram reconstruídas nativamente no AM69; a inspeção
confirmou ausência de Gazebo, RViz, OGRE e rendering nas imagens do módulo.
Não houve medição de temperatura ou consumo.

## Próximo portão

Manter câmera 640×480 a 10 Hz e corrigir a baixa razão de trabalho do comando no
roteiro longo. A próxima comparação deve isolar custo de transporte/processamento
da câmera (por exemplo, transporte comprimido até a percepção, preservando o
tópico RAW no host) e sintonia do MPPI. Depois, repetir exatamente os 420 s e
200 s por meta. F5 fecha somente quando ao menos uma meta de 8 m terminar
`SUCCEEDED` nesse protocolo.
