# ML3.5 — HIL no Aquila AM69: rota do DDS, precedência de `.env` e o limite do Wi-Fi

**Data:** 25/08/2026
**Hardware:** Aquila AM69, Torizon OS 7.7.0+build.40, 8 × Cortex-A72, 31 GiB, 98 GiB livres
**Topologia:** host `wlp0s20f3` (`192.0.2.4`) ↔ AP corporativo ↔ Aquila
`ethernet0` (`192.0.2.3`) — **Wi-Fi, não cabo**
**Plant e telas:** Gazebo (`quadruped_maze11`), RViz e câmeras de cena no host x86
**Aplicação:** Nav2 e `demo_perception` arm64 no Aquila

Esta é evidência de execução no AM69 real, e **apenas do portão funcional**.
Nada de desempenho, latência, térmica, FPS ou quedas foi medido, e nada disso é
reivindicado (regra 7). O portão de estabilidade não foi executado porque, neste
caminho, ele é inexecutável — ver "Irreprodutibilidade" abaixo.

---

## 1. O bloqueio de acesso era resolução de nome, não enlace

A sessão começou com a premissa de que o módulo estava inacessível e que o HIL
esperava um dongle USB-Ethernet ponto a ponto. **A premissa era falsa.** O
módulo respondeu de imediato pela LAN:

```
hostname: aquila-am69-12593525     Torizon OS 7.7.0+build.40     up 5:06
ethernet0  UP  192.0.2.3/24
ethernet1  UP  192.0.2.5/24
```

O host está em `192.0.2.4/24` pelo Wi-Fi — **a mesma /24**. As chaves de host
SSH são idênticas nos dois IPs, o que prova que são duas interfaces da mesma
máquina.

A causa real da falha de `module.sh inventory`: `MODULE_HOST` tem por padrão
`aquila-am69-12593525.local`, mDNS não resolve neste host, e `docker/.env` não
define `MODULE_IP`. Com `MODULE_IP` explícito, funciona sem mais nada.

**Método que não pode funcionar, e por quê.** Identificar o módulo esperando um
*décimo* MAC Toradex aparecer na varredura da LAN nunca dispara: o módulo publica
em duas portas na mesma /24 e já está no baseline dos nove. Identificação tem de
ser por identidade — hostname, chave de host, ou IP registrado em
`ml35-hil-ethernet.md` — nunca por diferença.

---

## 2. Bug corrigido: `docker/.env` sobrescrevia o ambiente explícito

`scripts/module.sh` documenta no cabeçalho "precedência: environment, então
docker/.env, então os defaults", e o comentário no ponto de leitura afirmava que
os guardas `${VAR:-}` garantiam isso. **Não garantiam.**

```bash
set -a; source "${env_file}"; set +a     # atribui INCONDICIONALMENTE
MODULE_HOST="${MODULE_HOST:-...}"        # guarda contra VAZIO, não contra .env
```

`source` atribui sempre; quando o guarda roda, a variável já carrega o valor do
`.env` e os dois casos são indistinguíveis. O guarda defende contra *não
definido*, nunca contra o `.env` vencer.

**Como isso se manifestou.** Um `HOST_IP=192.0.2.6` obsoleto no `.env`, de uma
sessão cabeada anterior, venceu um `HOST_IP=192.0.2.4` explícito na linha de
comando. Os dois XMLs do CycloneDDS foram renderizados para um host inexistente
e para `enp0s31f6`, uma interface **sem portadora**:

```
[module.sh] modulo 192.0.2.5 | host 192.0.2.6 | dominio 69
[module.sh] renderizando cyclonedds/host.rendered.xml (iface enp0s31f6, ...)
```

Isso falha como não-descoberta silenciosa: exatamente a falha que o script
existe para prevenir. Corrigido lendo o `.env` linha a linha e ignorando toda
chave já presente no ambiente.

---

## 3. `host.rendered.xml` estava byte-idêntico ao template

O arquivo montado nos containers do host **nunca havia sido renderizado**:

```xml
<NetworkInterface autodetermine="true" priority="default"/>
<AllowMulticast>false</AllowMulticast>
<Peer address="127.0.0.1"/>            <!-- único peer -->
```

O modo `learn` funciona **por causa** disso: multicast desligado com peer só
localhost confina a descoberta ao host, que é o que `learn` precisa. Mas o HIL
subindo assim não teria caminho algum até o módulo, sem erro em lugar nenhum.
`module.sh sync` é o passo que renderiza; ele não é opcional.

---

## 4. Achado principal: o DDS estava fixado na interface errada

O módulo tem duas interfaces na **mesma /24**, e o kernel tem duas rotas:

```
192.0.2.1/24 dev ethernet0 src 192.0.2.3  metric 101   <- sempre escolhida
192.0.2.1/24 dev ethernet1 src 192.0.2.5 metric 102
```

`ip route get 192.0.2.4` → `dev ethernet0 src 192.0.2.3`.

O `module.xml` estava fixado em **`ethernet1`** desde o HIL de 24/08. Para
qualquer destino nessa sub-rede, o caminho de bind e o de envio divergem por
construção. Depois de re-renderizar sobre `ethernet0`, o teste de alcance passou:

```
1/3 alcance UDP no dominio 69, porta 24666
    modulo -> host: OK (de 192.0.2.3)      <- endereço de origem correto
```

**Hipótese não verificada, registrada para teste.** Essa divergência existia
também em 24/08: o host estava em `192.0.2.6`, que é igualmente
`192.0.2.1/24`, logo a rota do módulo também preferia `ethernet0` enquanto o DDS
estava em `ethernet1`. `/demo/cmd_vel` corre **módulo→host**, exatamente a
direção degradada. Entrega intermitente explicaria "meta curta passou, os dois
alvos de 8 m estouraram" sem invocar a marcha. **Não** se afirma que esta é a
causa das quedas; afirma-se que não foi descartada e que agora é testável:
mesmo protocolo, `ethernet0` fixado, cabo no lugar.

---

## 5. Hipótese levantada e REFUTADA: `MaxAutoParticipantIndex`

O host mostrou 26 portas UDP ligadas no range do domínio 69, ou seja treze
participantes chegando ao índice 13 (`24660 + 2*índice`). Com
`AllowMulticast=false`, o padrão 9 do CycloneDDS sugeria que tudo acima do índice
9 fosse invisível. A hipótese foi implementada (`<MaxAutoParticipantIndex>32`
nos dois templates) e **está revertida**, porque o rastreamento de descoberta no
host a refutou diretamente:

```
dq.builtin: SPDP ST0 ... NEW (... aquila-am69-12593525/0.10.5/Linux/Linux)
            (data udp/192.0.2.3:24673  meta udp/192.0.2.3:24672)
            property_list={"__Hostname":"aquila-am69-12593525","__Pid":"43"}
```

O host **recebe** o SPDP do módulo. A descoberta módulo→host funciona; o que
falhava era o instrumento — `ros2 topic list` e o `echo` do teste 3/3 medem antes
de a descoberta unicast periódica assentar. Pior, o valor 32 fazia cada
participante anunciar para 33 portas por peer por período, e as escritas falhavam
justamente nas portas altas:

```
ddsi_udp_conn_write to udp/192.0.2.3:24708 failed with retcode -3
... 24710, 24712, ... 24722   (índices 24..31)
```

O mesmo `ddsi_udp_conn_write ... failed` consta em
`ml35-regressao-navegacao.md`. Revertido pelo critério que o projeto já aplica:
evidência contrária e amplificação de tráfego num enlace conhecidamente
marginal.

**Lição de método:** o participante mais novo é o menos provável de ter sido
descoberto, então a própria ferramenta usada para investigar é a que não vê. Sob
`AllowMulticast=false`, um diagnóstico precisa de tempo de assentamento antes de
concluir ausência.

---

## 6. Irreprodutibilidade: por que o portão de estabilidade não roda no Wi-Fi

O gargalo da câmera de 74 Mbit/s já estava medido em 21/08 (`.ai/CLAUDE.md`); o
que esta sessão acrescenta é a **variância**. Três medições da mesma câmera, na
mesma configuração, com minutos de intervalo:

| Tópico | Na origem (host) | No módulo, 3 corridas |
| --- | --- | --- |
| `/demo/camera/image_raw` | 9,933 Hz | **1,716 / 9,994 / 2,692 Hz** |
| `/demo/scan` | 9,805 Hz | 8,309 – 10,057 Hz |
| `/clock` | 975 Hz | 594 – 975 Hz |

Não é bloqueio sistemático, é irreprodutibilidade. Uma amostra de 900 kB
(640×480×3) fragmenta em ~640 datagramas UDP e perder **um** descarta a amostra
inteira; o publisher da câmera é RELIABLE (`detection_stub.py`), então o
CycloneDDS retransmite, o que adiciona tempestade de retransmissão e bloqueio de
cabeça de fila. Jitter medido no enlace **vazio**: `mdev 11,3 ms`, pico
`64,1 ms`, contra um controlador Nav2 de período 50 ms.

Comparar corridas neste caminho fabrica regressão. O portão de estabilidade não
é apenas inválido aqui — é inexecutável.

---

## 7. Bancada: o que estava de pé, e o que não é problema

- **`nav` rodava no host** junto com o resto da pilha `learn`, por 39 minutos.
  Com Nav2 subindo no módulo no mesmo `ROS_DOMAIN_ID=69`, seriam dois — dois
  publicadores em `/demo/cmd_vel`, dois costmaps, TF duplicada. Derrubado.
- **RTF ≈ 0,99** (câmera 9,933 Hz contra sensor de 10 Hz em tempo de simulação),
  com `learn` e RViz2 em pé, `nproc` = 22, load ~12. **O colapso de RTF para
  ~0,5 do `ml35-regressao-navegacao.md` não reproduz neste estado.** Logo o
  desligamento pendente é necessário para o PHY da NIC, não para o estado da
  bancada.
- **`enp0s31f6` não é cabo.** `carrier_up_count` = 0 (nunca teve portadora neste
  boot) e o `ethtool` anuncia só `10baseT/Half 10baseT/Full`, quando um I219
  deveria anunciar `1000baseT/Full`. O journal mostra `e1000e: EEE TX LPI TIMER`
  + `NIC Link is Down` a cada retomada (é um notebook, ~105 linhas de
  suspend/resume no boot). A mesma retomada derruba o dongle ASIX, que é por que
  ele aparece de pé e depois caído sem ninguém tocar no cabo.

---

## 8. Estado ao fim da sessão

Portão funcional, no AM69 real:

| Verificação | Resultado |
| --- | --- |
| Nav2 arm64 sobe sem segfault (`route_server` íntegro) | PASSA |
| `demo_perception` arm64 produz `/demo/perception/detections` | PASSA |
| Contrato de tópicos atravessa a fronteira | PASSA |
| Descoberta bidirecional (SPDP do módulo recebido no host) | PASSA |
| `verify` 1/3 alcance UDP | PASSA |
| `verify` 3/3 heartbeat | FALHA — mede antes do assentamento |
| Reprodutibilidade de qualquer taxa | FALHA — ver §6 |

## 9. Pendente, e é físico

1. `sudo modprobe -r e1000e && sudo modprobe e1000e`; critério objetivo:
   `1000baseT/Full` aparecer em "Supported link modes". Se não aparecer,
   **desligar de verdade** (power off, não reboot morno — o estado do PHY
   sobrevive a reboot morno).
2. Cabo no **switch** onde está o módulo, não ponto a ponto: assim o host pega
   endereço cabeado na mesma /24 e a `ethernet2` sem pilha IP deixa de importar.
3. Refazer `sync` / `up` / `verify` com o `HOST_IP` cabeado e **`ethernet0`**.
4. Só então o protocolo n≥3, registrando RTF junto de cada corrida e sem alterar
   carga entre corridas.
5. Corrigir o teste 3/3 do `verify` para dar tempo de assentamento à descoberta
   unicast antes de concluir ausência.
