# ML3.5 F5 — a navegação não estava quebrada: as metas do ensaio estavam atrás de parede

Data: 27/08/2026. Topologia: **HIL real** — Gazebo Harmonic no host x86,
Nav2 + percepção no Aquila AM69 (`10.22.1.130`, `ethernet0`, RTT 0,156 ms),
`bt_navigator` em `active`. Nada foi reconstruído, nenhum parâmetro mudou,
nenhuma imagem foi tocada entre as duas corridas abaixo.

**A única variável entre elas é a geometria da meta.**

Antecedente teórico e método: `docs/ml35/proximos-passos-navegacao.md` §11.
Amostras cruas: `ml35-f5-rota-conectada.csv`, `ml35-f5-controle-patrulha.csv`.

---

## 1. O A/B, corrido com minutos de intervalo na mesma bancada

| métrica | **rota conectada** (`maze_route.py`) | **controle** — meta de patrulha (0,00; 8,00) |
| --- | ---: | ---: |
| duração (tempo de simulação) | 239,9 s | 119,9 s |
| **razão de trabalho `vx`** (> 0,05 m/s) | **37,5%** | **0,0%** |
| `cmd_vx` ≈ 0 (≤ 0,005 m/s) | **12,9%** | **98,6%** |
| `cmd_vx` pico / médio | 0,140 / **0,0454** m/s | 0,014 / **0,0003** m/s |
| velocidade média | 0,0518 m/s | 0,0097 m/s |
| caminho percorrido | 12,42 m | 1,17 m |
| **deslocamento líquido** | **7,11 m** | **0,19 m** |
| eficiência de trajeto | **57,2%** | 16,2% |
| **metas cumpridas** | **8 de 8 (`ok`)** | **0**, meta nunca encerrou em 120 s |
| `cmd_vx` negativo | 0% | 0% |
| tilt de pico / quedas | 0,82° / 0 | 0,71° / 0 |
| folga de carcaça | +0,065 m | +0,065 m (inalterada) |

A razão de trabalho — a métrica primária escolhida na madrugada de 26/08
justamente por ser estável entre corridas — vai de **0,0% para 37,5%**. As
sessões anteriores mediram 0,0% a 6,7% em **toda** condição já testada
(CPU, `/clock`, amostragem do MPPI, câmera comprimida, Ethernet, correção
da BT). Nenhuma delas passou de 8,6%.

Os 57,2% de eficiência de trajeto reproduzem exatamente os **57% medidos no
host** em 21/08, contra 28,9% no HIL "quebrado". O robô no módulo sempre foi
capaz disso.

## 2. O que isso faz com a §10: o giro unidirecional não é defeito de controlador

A §10 mediu, numa meta de patrulha, `cmd_wz` **nunca negativo** e o yaw subindo
monotonicamente, e concluiu — corretamente, para o dado que tinha — que "um giro
que nunca se corrige não é o comportamento esperado de nenhum critic isolado".

Extraindo `cmd_wz` dos dois CSVs desta corrida:

| | rota conectada | controle patrulha |
| --- | ---: | ---: |
| `cmd_wz` > 0 | **45,2%** | 7,8% |
| `cmd_wz` < 0 | **51,3%** | 64,8% |
| `\|cmd_wz\|` pico | 0,1729 rad/s | 0,1681 rad/s |
| **deriva líquida de yaw** | **+1,1°** em 240 s | **−186,8°** em 120 s |

Com plano válido o MPPI **corrige**: os dois sinais se alternam quase
igualmente e o robô termina 7,11 m adiante apontando para onde começou
(+1,1° de deriva em 4 minutos). Com meta atrás de parede o giro volta a ser
unidirecional e o robô dá **meia volta sobre o próprio eixo** sem sair do lugar.

**E o sinal inverteu.** A §10 mediu giro sempre POSITIVO; este controle mediu
giro predominantemente NEGATIVO, na mesma meta. Isso mata de vez a família de
hipóteses "assimetria de critic" / "erro de sinal na cadeia de guinada": um erro
de sinal não troca de sinal.

O giro unidirecional é a resposta **correta** do MPPI a um plano global que
aponta para dentro de uma parede: `PathAlignCritic` (peso 14) puxa para o
caminho, `CostCritic` proíbe o que o robô de fato vê, `vx_min: 0.0` fechou a ré,
e o único grau de liberdade que resta é a guinada. Como a geometria que produz o
gradiente é estática, o sinal não inverte — mas *qual* sinal depende de que lado
da parede o plano puxa, e por isso ele muda entre corridas.

## 3. O que estava errado, exatamente

Nada no Nav2, nas imagens, no módulo, no enlace ou na sintonia do MPPI.

`MAZE11_GOALS` foi gerada por `maze_fit.py` para ser **patrulha**: centros de
corredor espalhados dentro de um raio de 8 m. Num labirinto isso põe as quatro
metas atrás de parede (`scripts/maze_geodesic.py`):

| meta | reta | geodésica real | razão | 1ª parede na reta |
| --- | ---: | ---: | ---: | ---: |
| (0,00; 8,00) | 8,00 m | 12,23 m | 1,53× | 3,88 m |
| (−8,00; 0,00) | 8,00 m | 33,76 m | 4,22× | 3,90 m |
| (−1,60; 1,60) | 2,26 m | 5,54 m | 2,45× | 0,96 m |
| (−5,83; 4,91) | 7,62 m | 24,02 m | 3,15× | 1,05 m |

Com `global_costmap` rolante **sem `static_layer` e sem mapa** e o NavFn em
`allow_unknown: true`, o plano dessas metas atravessa parede não observada.
`compute_path_to_pose` devolve `SUCCEEDED`, `/plan` aparece no RViz e no
cockpit, e **não há erro, aviso ou log**. A rota conectada do `maze_route.py`,
por construção, tem **0 de 9 pernas com parede na reta** e 100% de visibilidade
da origem de cada perna.

## 4. Consequências

**Para o portão do F5.** "Goal Nav2 `SUCCEEDED` com o robô de pernas, com o Nav2
rodando no módulo" foi cumprido **oito vezes numa corrida**. O que continuava
reprovando era o protocolo de 8 m sobre metas de patrulha — que pedia ao
planejador uma coisa que a geometria do cenário não oferece. O portão precisa
ser reescrito em cima de rota conectada, ou em cima de mapa persistido (§5),
antes de voltar a ser cobrado.

**Para as §§7–10.** Mediram o controlador com uma entrada inválida. As cinco
hipóteses refutadas da §1 seguem refutadas — nada aqui as reabre — mas nenhuma
conclusão sobre *critics* daquelas seções sobrevive. Em particular a campanha
`PathAlignCritic` 14 × 8 mediria ruído.

**Para o teste de desligar critic** (§10, ainda não executado): cancelado como
prioridade. Ele mediria a reação do MPPI a uma entrada que agora sabemos ser
inválida.

## 5. O mapa persistido continua valendo, por outro motivo

O achado **não** dispensa persistir o mapa; muda o argumento. A rota conectada
funciona porque cada perna cabe no que o robô já enxerga — é o operador
resolvendo a geometria fora do Nav2. Um mapa persistido resolve dentro:
qualquer meta, inclusive as de patrulha a 8 m, passa a ter plano válido porque o
NavFn deixa de precisar adivinhar o que há atrás da parede.

Os dois bloqueios continuam os da §11: `slam_params.yaml` com
`base_frame: base_link` (o Go2 tem `base`), e o `slam_toolbox` consumindo
`LaserScan` enquanto o `/demo/scan` do Go2 é o anel degenerado com zero
obstáculos — o dado bom é `/demo/scan_cloud`, achatado por
`ros-jazzy-pointcloud-to-laserscan`. Sem AMCL nesta topologia.

## 6. Limites desta medição

- **n = 1 por condição.** A dispersão de 2,4× medida em configuração idêntica
  continua valendo em princípio, mas o efeito aqui é de outra ordem: 37,5% × 0,0%
  na métrica primária, e 8 metas cumpridas contra zero em qualquer corrida já
  feita neste projeto. Não é um efeito que ruído de 2,4× produza.
- **A rota conectada não é a demo.** É o instrumento que isolou a variável.
  Depender dela seria fixar a trajetória fora do planejador.
- **O déficit residual do MPPI não foi medido.** 37,5% de razão de trabalho e
  0,0454 m/s médio ainda estão abaixo de `vx_max` 0,15 m/s. Se resta margem de
  sintonia, ela só é mensurável agora — com entrada válida, e sob o protocolo
  intercalado da §2.
- **Nada foi visto num navegador.** O gate visual do cockpit segue pendente.
