# ML3.5 F5 — repetição HIL por `ethernet0`, enlace cabeado gigabit

**Estado:** EXECUTADO — **portão de 8 m REPROVADO**
**Data:** 25/08/2026
**Revisão de partida:** `6f036c0` mais as alterações locais desta sessão
**Baseline do módulo:** Aquila AM69 V1.0A, Torizon OS 7.7.0, 8 × Cortex-A72, 31 GiB

Evidência de execução no AM69 real, com build arm64 **nativo no módulo**. O robô
e os sensores seguem simulados no host, portanto **nada aqui valida localização
por pernas, um Go2 físico, térmica ou consumo** (regras 5 e 7 do `CLAUDE.md`).
Não houve medição de temperatura nem de potência.

## Resultado executivo

O enlace foi corrigido e comprovado nos dois sentidos. **O portão de 8 m
continua reprovado**, e a causa **não é a rede**.

A hipótese que estava aberta em `estado-fases.md` — de que a divergência
`ethernet1`/`ethernet0` degradava `/demo/cmd_vel` na direção módulo→host e
explicava "meta curta passou, 8 m estourou" — está **refutada por medição**. Com
rota simétrica, enlace gigabit e `verify` 3/3, as duas metas de 8 m continuaram
estourando o prazo.

O que a medição encontrou no lugar dela: **o Aquila satura de CPU**. O Nav2
sozinho consome 600–727% dos 800% disponíveis. A percepção soma outros ~187%, o
que excede a capacidade da máquina, e aí a frescura dos sensores colapsa.

## Pré-condições físicas, agora satisfeitas

O bloqueio registrado em 24/08 e 25/08 (PHY sem gigabit, sem portadora) deixou
de existir:

```text
enp0s31f6   UP   192.0.2.14/24   Speed: 1000Mb/s   Duplex: Full   Link: yes
host -> modulo:  192.0.2.16 dev enp0s31f6 src 192.0.2.14
modulo -> host:  192.0.2.14  dev ethernet0  src 192.0.2.16
RTT ICMP: 0,400 ms
```

A assimetria de rota foi eliminada de forma **estrutural**, não pontual: o
Wi-Fi ficou com métrica 600 contra 100 do cabo, então a `/24` inteira prefere o
cabo. A rota `/32` para o módulo, persistida no perfil NetworkManager, é
redundância — não é o que segura o caminho.

DDS renderizado coerente: host em `enp0s31f6` com peer `192.0.2.16`; módulo
em `ethernet0` com peer `192.0.2.14`.

## `verify` 3/3, e um falso negativo corrigido no caminho

`scripts/module.sh verify` retorna **0** com as três etapas passando: alcance UDP
módulo→host, contrato de tópicos visto de dentro do módulo, e heartbeat
publicado no módulo e recebido no host.

Na primeira execução ele **reprovou por `/clock` ausente, contra um módulo que
lia `/clock` a 616 Hz**. A etapa 2 coletava com `ros2 topic list | grep /demo/` e
em seguida exigia `/clock`, que não está sob `/demo/`: a checagem não podia
passar nunca. Corrigido para `grep -e /clock -e /demo/`.

O teste que existia (`test_verify_requires_the_host_to_module_topic_contract`)
passou durante todo o defeito, porque só verifica se a string `/clock` aparece no
arquivo — e ela aparecia, na lista de exigências. Presença de uma string não é
evidência de que o pipeline consegue produzi-la. Foi adicionado
`test_every_required_topic_survives_the_collection_filter`, que casa cada tópico
exigido contra o filtro de coleta e **falha** quando o defeito é reintroduzido
(verificado por mutação).

## Corridas executadas

Protocolo: `nav_trial.py --seconds 420 --goal-timeout 200`, metas padrão do
`maze11`, `ROS_DOMAIN_ID=69`, câmera RAW 640×480@10 Hz. Host: `sim`, `viz`,
`cockpit`, `hmi`. Módulo: `nav`, `perception`.

| Medida | HIL completo | HIL só Nav2 (diagnóstico) |
| --- | ---: | ---: |
| fator de tempo real | 0,944 | 0,841 |
| tempo de simulação | 416,5 s | 419,9 s |
| caminho percorrido | 6,45 m | 18,00 m |
| deslocamento líquido | 3,35 m | 5,20 m |
| **velocidade média** | **0,0155 m/s** | **0,0429 m/s** |
| `cmd_vx` pico / médio | 0,124 / 0,0073 m/s | 0,150 / 0,0252 m/s |
| razão de trabalho de `cmd_vx` | 5,8% | 16,8% |
| amostras com `vx` ≈ 0 | 79,0% | 57,7% |
| amostras com giro (`wz` acima de 0,01 rad/s) | 93,9% | 77,3% |
| tilt pico | 0,94° | 15,66° |
| ré | 0% | 0% |
| quedas | 0 | 0 |
| folga mínima de carcaça | +0,065 m | +0,065 m |
| **metas de 8 m** | **0 de 2 — prazo em 203 s e 403 s** | **0 de 2 — prazo em 206 s e 406 s** |

CSV: `ml35-f5-ethernet0-run1.csv` e `ml35-f5-ethernet0-diag-nav-only.csv`.

Os dois RTF ficam acima de 0,84, então as duas corridas são medidas válidas e
comparáveis entre si.

A corrida "só Nav2" **não é o portão** — ela para a percepção no módulo e altera
a topologia declarada. É diagnóstico, e está aqui por separar duas causas que
estavam confundidas. A percepção foi religada ao fim dela.

## O que os números dizem

**1. A rede não é a causa.** Vazão e latência foram medidas nas duas pontas:

| | origem (host) | chegada (módulo) |
| --- | ---: | ---: |
| `/demo/scan_cloud` | 9,50 Hz, 3,10 MB/s | 9,45 Hz, 2,90 MB/s |

Somando a câmera (~9,3 MB/s, ~75 Mbit/s), o tráfego total fica em ~100 Mbit/s
num enlace de 1 Gbit/s. Não há perda: a taxa que chega é a taxa que sai.

**2. A defasagem de sensor é contenção de CPU, não transporte.** O
`collision_monitor` recusou a nuvem do LiDAR 16 vezes, com carimbos divergindo
**1,0–1,2 s** — o mesmo defeito que 24/08 declarou corrigido. Mas com o Nav2
ocioso, o atraso de chegada medido no módulo é pequeno:

```text
scan         n=299  mediana 0,019 s  p95 0,030 s  max 0,049 s
scan_cloud   n=179  mediana 0,042 s  p95 0,049 s  max 0,053 s
camera       n=298  mediana 0,009 s  p95 0,103 s  max 0,209 s
```

O 1,1 s só aparece **sob navegação ativa**. Vazão bate e latência não: é
enfileiramento no consumidor, não descarte no enlace.

**3. O Nav2 sozinho já satura o AM69.** Amostrado durante a corrida diagnóstica,
com a percepção parada:

```text
demo-nav-1: 429% .. 727%   (mediana ~640%)   de 800% disponiveis
load average: 9,3 .. 22,4
```

Com a percepção no ar (~187%), a soma passa da capacidade da máquina. É essa
saturação que produz a defasagem de 1,1 s, que faz o `collision_monitor` recusar
a fonte, que derruba a velocidade média em **2,8×** (0,0429 → 0,0155 m/s).

**4. Mas remover a câmera não faz a meta passar.** Mesmo a 0,0429 m/s, as duas
metas de 8 m estouraram. Existem **dois limites independentes**, e só um deles é
CPU.

**5. O segundo limite é decisão de trajeto, e a assinatura é clara.** No HIL
completo o robô está com `vx` em zero em **79%** das amostras e girando em
**93,9%** delas. Ele passa o ensaio **girando no lugar em vez de transladar**.
Na corrida sem câmera o padrão alivia mas não some (57,7% / 77,3%), e o custo
aparece na rota: **18,00 m de caminho para 5,20 m de deslocamento líquido**, ou
**28,9% de eficiência de trajeto** — contra 57% medidos no host em 21/08.

Isso é assinatura de sintonia do controlador, não de rede, e corresponde à
hipótese sem medida que já estava registrada: `PathAlignCritic` em 14,0 contra
`PathAngleCritic` em 2,0. **Segue como hipótese** — este ensaio mostra o sintoma
com números, não testou a correção.

**6. Estabilidade.** Zero quedas nas duas corridas. Mas o tilt de pico salta de
0,94° para **15,66°** exatamente na corrida em que o robô de fato anda — o valor
baixo do HIL completo descreve um robô quase parado, não um robô estável. A
folga de carcaça segue em **+6,5 cm** nas duas, e continua sendo o portão de
qualquer aumento de velocidade.

## Veredito do portão

**F5 permanece aberto.** O portão adotado exigia n=3 com ao menos uma meta de 8 m
`SUCCEEDED` por corrida. A corrida 1 reprovou, e a corrida diagnóstica mostrou
que a causa não é removível por rede nem por desligar a câmera. As corridas 2 e 3
não foram executadas: repetir uma condição já reprovada, com mecanismo
identificado, gastaria bancada sem produzir informação nova.

O que **passou a estar comprovado** e não estava:

- enlace cabeado gigabit simétrico, com `verify` 3/3;
- imagens arm64 reconstruídas nativamente no módulo, regra 1 verificada nas
  quatro;
- `route_server` configura e ativa sem `ReroutingService` e sem SIGSEGV;
- a hipótese `ethernet1`/`ethernet0` está **refutada**;
- o gargalo é **CPU do módulo**, quantificado, e **decisão de trajeto**,
  quantificada.

## Próximo portão, na ordem que os dados sugerem

1. **Reduzir a CPU do Nav2 no módulo.** É o limite maior e o mais bem medido:
   600–727% de 800% antes de qualquer percepção. Sem folga aqui, nenhuma outra
   correção tem espaço para aparecer.
2. **Tirar a imagem RAW do fio até a percepção** (transporte comprimido,
   preservando o tópico RAW no host), que é o próximo portão já registrado em
   24/08. Vale ~2,8× de velocidade média pela medição desta sessão.
3. **Só então mexer no MPPI** (`PathAlignCritic` × `PathAngleCritic`), com o
   sintoma de "gira em vez de transladar" como critério de aceitação, medido
   pela razão de trabalho de `cmd_vx` e pela eficiência de trajeto.
4. Repetir o protocolo 420 s / 200 s, n=3, sem alterar mais nada.
