# ML3.5 F5 — por que o plano global oscila, e por que `clearing: false` NÃO conserta

Data: 27/08/2026, na sequência de `ml35-f5-rota-conectada.md`. Topologia HIL
real: Gazebo no host x86, Nav2 + percepção no Aquila AM69 (`10.22.1.130`).

Este arquivo registra **um achado de causa raiz e um experimento reprovado**.
O experimento está travado por `tests/test_module_params_mount.py` para que
ninguém o repita — é a primeira ideia que ocorre a quem lê o achado.

---

## 1. A causa raiz: o plano global alterna entre duas rotas incompatíveis a 1 Hz

Lendo `/plan` a cada 5 s durante uma meta presa, (0,0) → (0,8), robô no spawn:

| t | pontos | comprimento | rumo dos primeiros 0,5 m |
| ---: | ---: | ---: | ---: |
| +5 s | 458 | **11,49 m** | **173°** — rota verdadeira, para −x |
| +10 s | 458 | 11,50 m | 177° |
| **+15 s** | 291 | **8,59 m** | **89°** — atravessa parede não observada |
| +20 s | 291 | 8,59 m | 89° |
| +25 s | 274 | 8,66 m | 35° |
| +30 s | 271 | 8,81 m | 18° |

Os 11,5 m batem com a geodésica real do maze11 medida offline — 12,23 m por
`scripts/maze_geodesic.py`, sobre grade não inflada. Os 8,6 m são a linha reta
por dentro de parede, que só existe porque `allow_unknown: true` torna o
desconhecido barato para o NavFn (Dijkstra puro sobre custo).

**O MPPI recebe um caminho que inverte de 90° a 180° a cada segundo.** É por
isso que ele gira sem transladar. As §§7–10 de
`docs/ml35/proximos-passos-navegacao.md` perseguiram isso como defeito de
critic; não é. O `PathAlignCritic` está fazendo exatamente o que lhe pedem —
alinhar com um caminho que muda de ideia.

Isto **completa** o achado de `ml35-f5-rota-conectada.md`: lá ficou provado que
o sintoma some com meta sem parede na reta; aqui está o mecanismo pelo qual a
meta com parede o produz.

## 2. O experimento reprovado: `clearing: false` no `obstacle_layer` global

**Hipótese:** dar memória ao mapa global. Parede vista uma vez nunca mais sai,
o atalho de 8,6 m deixa de existir, o plano converge. É literalmente "salvar o
que o robô descobriu".

**Medido, mesma bancada, mesmo dia, meta (0,8), 240 s:**

| métrica | baseline | `clearing: false` |
| --- | ---: | ---: |
| deslocamento líquido | 0,19 m | **1,04 m** |
| razão de trabalho `vx` | 0,0% | **3,9%** |
| caminho percorrido | 1,17 m | 2,07 m |
| `cmd_vx` ≈ 0 | 98,6% | 83,2% |
| **metas cumpridas** | 0 | **0** |
| **plano ainda alterna** | 11,49 ↔ 8,59 m | **11,66 ↔ 8,77 m** |

Melhorou nas margens e **não mexeu no mecanismo**. A oscilação continua
idêntica.

**Por que, medido em `/global_costmap/costmap_raw` na reta até a meta, com
`clearing: false` ligado:**

```
custo em (0, y), passo 0,25 m:
  0.00:199  0.25:180  0.50:180  0.75:255  1.00:255  ... 8.00:255

primeira célula >= 253 na reta:  0,55 m
células desconhecidas (255):     150 de 161
```

**A semântica do parâmetro é contra-intuitiva e é onde a hipótese morre:**
`clearing` **não** é "esquecer obstáculo". É o raytrace — e o raytrace é o
**único** mecanismo que transforma célula desconhecida em **LIVRE** nesta
camada. Desligando-o, nada nunca vira livre: o mapa global fica permanentemente
desconhecido fora das células marcadas.

E como o NavFn roda com `allow_unknown: true`, um mapa mais desconhecido torna o
atalho pelo desconhecido **mais** atraente, não menos. **A correção agrava a
causa que ataca.** Os 1,04 m de melhora são margem, não mecanismo.

## 3. O que de fato resolve

Persistir **as duas** coisas — ocupado *e* livre. A camada de obstáculo do
Nav2 não sabe fazer isso: ela tem um botão só, e ele controla os dois.

O que sabe é um mapa de ocupação de SLAM. `slam_toolbox` acumula livre onde o
raio passou e ocupado onde ele terminou, e **nunca esquece**. Alimentando a
`static_layer` do costmap global (que já está definida e inerte no YAML, com o
procedimento ao lado), com `allow_unknown: true` **mantido** para que meta
distante continue planejável:

- no instante 0 o mapa está vazio e o plano vai reto pelo desconhecido — igual a
  hoje, e correto: é o que permite aceitar uma meta a 8 m sem mapa;
- ao ver a parede a 3,88 m, ela entra no mapa **em definitivo**;
- o atalho de 8,6 m deixa de existir e o plano converge para os 11,5 m;
- a oscilação para porque não há mais nada que desmarque.

Isso é a ideia do operador — salvar o que foi descoberto para não repetir
caminho — na única camada que consegue sustentá-la.

**Os dois bloqueios, inalterados desde a §11:**

1. `slam_params.yaml` tem `base_frame: base_link`; o Go2 tem `base`. Parâmetro.
2. `slam_toolbox` consome `sensor_msgs/LaserScan`, e o `/demo/scan` do Go2 é o
   anel degenerado que o delta 3 do `nav2_params_go2.yaml` mediu como **zero
   obstáculos**. O dado bom é `/demo/scan_cloud`; achatá-lo é
   `ros-jazzy-pointcloud-to-laserscan` (estoque, 2.0.2 no apt do Jazzy), que
   ainda não está em imagem nenhuma — **exige rebuild arm64 nativo no módulo.**

**Sem AMCL** nesta topologia: a odometria do Gazebo é verdade de terreno e
`map`→`odom` já é a identidade do `odom_tf`. Em hardware real ele volta.

## 4. Ganho de infraestrutura entregue no caminho

`compose.module.yml` passou a montar `ros2_ws/src/demo_navigation/config` sobre
`/ws/src/demo_navigation/config` no container `nav`. **Mudar um parâmetro no
módulo passou a custar um `module.sh sync` em vez de um `module.sh build`.**

O alvo da montagem é o **destino final do symlink** que o colcon instala, não o
caminho instalado — montar no caminho instalado substituiria um symlink por um
diretório e o Nav2 continuaria lendo a cópia da imagem, sem erro nenhum: uma
campanha A/B compararia a condição consigo mesma. Verificado no container e
travado por `tests/test_module_params_mount.py`.

Não há risco de a montagem mascarar um rebuild: `module.sh build` constrói a
imagem a partir da mesma árvore sincronizada.

## 5. Uma anomalia aberta, não explicada

O costmap global reporta **primeira célula ≥ 253 (inscrito) a 0,55 m em +y**,
enquanto `maze_fit.py` mede **3,47 m de pista livre** nessa direção a partir do
spawn, e a folga de nascimento é 0,55 m. As duas leituras não se conciliam.

Candidatos não testados: marca da própria perna em trote (o `selfhit.py` foi
rodado com o robô parado), obstáculo marcado em frame errado, ou a inflação de
0,85 m do global vindo das paredes laterais do corredor de 1,20 m — este último
é o mais provável e explicaria custo alto sem parede à frente, mas não um valor
≥ 253, que é a faixa inscrita.

Vale medir antes de rodar o SLAM: se o costmap global marca obstáculo inscrito a
0,55 m à frente num corredor livre por 3,47 m, o mapa de SLAM herdaria o mesmo
erro em definitivo.

## 6. Limites

- n = 1 por condição. O efeito de `clearing: false` (0,19 → 1,04 m) está dentro
  da faixa de ruído já medida neste projeto e **não** é reivindicado como real;
  o que é reivindicado é que a oscilação de plano **não** mudou, e isso é
  qualitativo, não estatístico.
- A leitura de `/plan` é a cada 5 s. A oscilação pode ser mais rápida que isso;
  o `RateController` da BT replaneja a 1 Hz.
- Nada foi visto num navegador. O gate visual do cockpit segue pendente.
