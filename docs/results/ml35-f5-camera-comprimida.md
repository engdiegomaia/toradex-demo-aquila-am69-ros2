# ML3.5 F5 — transporte comprimido da câmera: o que ganhou e o que não moveu

**Estado:** EXECUTADO — **engenharia entregue, portão de 8 m ainda REPROVADO**
**Data:** 25/08/2026 (noite)
**Baseline do módulo:** Aquila AM69 V1.0A, Torizon OS 7.7.0, 8 × Cortex-A72
**Topologia:** host `sim`+`viz`+`cockpit`+`hmi`; módulo `nav`+`perception`

Build arm64 nativo no módulo, regra 1 verificada nas quatro imagens. Robô e
sensores seguem simulados no host: **nada aqui valida localização por pernas, um
Go2 físico, térmica ou consumo** (regras 5 e 7). Sem medição de temperatura ou
potência.

## Resultado executivo

O transporte comprimido **funciona e entrega o que prometia em engenharia**, e
**não move o portão de navegação**. As duas coisas são verdade ao mesmo tempo, e
a segunda é a que importa para o F5.

| O que mudou | Medido |
| --- | --- |
| Fio ocupado pela câmera | 9.300 KB/s → **113,5 KB/s** (~82×) |
| CPU total no módulo | ~711% → **~600%** de 800% |
| `load average` no módulo | ~20 → **~15** |
| RAW atravessando o fio | **deixou de atravessar** — os únicos assinantes são `rviz2` e `camera_compressor`, ambos no host |

E, ainda assim:

| Condição | Vel. média | `vx` = 0 | razão de trabalho | metas de 8 m |
| --- | ---: | ---: | ---: | --- |
| RAW no fio (baseline) | 0,0155 m/s | 79,0% | 5,8% | 0 de 2 |
| **Comprimido, corrida 1** | 0,0109 m/s | 91,7% | 2,7% | 0 de 1 — **caiu** |
| **Comprimido, corrida 2** | 0,0265 m/s | 85,1% | 2,1% | 0 de 2 |
| RAW, **sem percepção no módulo** | 0,0429 m/s | 57,7% | **16,8%** | 0 de 2 |

CSV: `ml35-f5-comprimido-run1.csv`, `ml35-f5-comprimido-run2.csv`.

## Leitura honesta dos números

**A queda da corrida 1 não reproduz.** Corrida 1 abortou com o robô no chão
(z = 0,307 m, inclinação 30,9°) aos ~200 s — a primeira queda da campanha.
Corrida 2, com mundo limpo, foi aos 420 s sem queda (tilt de pico 11,17°). Uma
queda em duas corridas, sem mecanismo identificado, é **variância**, não
regressão atribuível à mudança. Não a trate como defeito do transporte
comprimido, e não a trate como resolvida.

**A velocidade não melhorou de forma confiável.** 0,0109 e 0,0265 m/s **cercam**
os 0,0155 m/s da RAW. Com n=2 e essa dispersão, a afirmação defensável é "não
melhorou", não "melhorou 70%".

**A razão de trabalho PIOROU, e é o achado que mais informa.** Comprimido dá
2,1–2,7% contra 5,8% da RAW, e `vx` fica em zero em 85–92% das amostras contra
79%. Cortar CPU **não** devolveu comando ao robô.

**O gargalo remanescente foi isolado.** Mesmo com a câmera comprimida, o
`collision_monitor` continua recusando a nuvem do LiDAR com ~1,0 s de defasagem
(11 ocorrências) e emite explicitamente:

```text
[collision_monitor]: Robot to stop due to invalid source.
```

Ou seja: **ele está zerando o comando ativamente.** A defasagem da nuvem
acompanha a CPU do Nav2 — que segue em 366–531% — e não o tráfego da câmera.

**A variável que domina não é o formato do transporte, é a presença da
percepção.** Em toda condição com percepção no módulo, seja RAW ou comprimida, a
razão de trabalho fica entre 2% e 6%. Sem percepção nenhuma, sobe para 16,8%. O
AM69 não comporta este Nav2 e uma percepção ao mesmo tempo, e comprimir a imagem
alivia ~15% de CPU sem tirar o sistema desse regime.

## O que foi entregue e fica

- Compressão no host (`sim.launch.py`) e descompressão do lado de quem consome
  (`perception.launch.py`), com `image_transport republish` upstream.
- **Contrato intacto:** `demo_perception` não foi tocado. Segue consumindo
  `sensor_msgs/Image` e segue sem saber a origem do quadro (regra 6) — a
  religação é `SetRemap` no launch. A troca pelo TIDL continua sendo troca de
  container.
- **Sem código por modo:** `learn` e `hil` usam o mesmo
  `ros2 launch demo_bringup perception.launch.py`.
- Oito testes estruturais, todos verificados por mutação.

### Duas armadilhas do mesmo nó, ambas silenciosas

1. **`in_transport`/`out_transport` são parâmetros, não posicionais.** Como
   `arguments=['raw','compressed']`, o Jazzy lê o primeiro e deixa o segundo
   vazio, logando `The 'out_transport' parameter is set to:` em branco. No host
   isso virou `republish` raw→raw sobre o **mesmo** tópico: realimentação, com a
   câmera do contrato indo de 10 Hz para **118 Hz** e `Publisher count: 2`.
2. **O remap precisa carregar o sufixo do transporte.** O `image_transport` cria
   `out/compressed`; remapear `out` não casa e é ignorado. O nó publicava em
   `/out/compressed` na raiz. `ros2 topic list | grep camera` não mostrava nada
   de errado — só `ros2 node info` denunciava.

Custaram dois ciclos de build. Por isso a validação da cadeia passou a ser feita
**inteira no host** (~6 min) antes de assar no módulo (~20 min).

## Próximo portão

A ordem do plano estava certa e agora está medida: **reduzir a CPU do Nav2 é a
alavanca, e a fase 2 sozinha não bastava.**

1. **Cortar CPU do Nav2.** Os dois atenuantes documentados já estão gastos
   (`batch_size` 1000, `controller_frequency` 10). A alavanca que sobra sem
   encurtar o horizonte em distância é `time_steps` × `model_dt` a horizonte
   constante — 64 × 0,15 mantém 9,6 s com 33% menos amostragem. É A/B, e o
   critério é `collision_monitor` parar de recusar a nuvem.
2. **Só então o MPPI** (`PathAlignCritic` 14,0 × `PathAngleCritic` 2,0), com
   razão de trabalho de `cmd_vx` acima de 30% como aceitação.
3. Repetir 420 s / 200 s com n=3.

Se (1) não abrir folga suficiente, a decisão que sobra é de produto, não de
sintonia: **trocar o MPPI por DWB no perfil do módulo**, com a perda de
qualidade declarada, ou aceitar que o AM69 roda Nav2 **ou** percepção, e não os
dois com esta configuração.
