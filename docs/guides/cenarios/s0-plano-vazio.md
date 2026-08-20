# S0 — Plano vazio

Mundo: `quadruped_empty.sdf` · Estação x86 · ~4 min

**É a referência de tudo.** Nenhum outro cenário significa nada sem uma corrida
deste no mesmo dia: os números de marcha só são comparáveis entre corridas no
mesmo RTF, e é aqui que se estabelece o RTF e a linha de base do dia.

## Para que serve

Chão plano infinito, sem nada na altura do lidar. Isola a marcha: qualquer coisa
que apareça nos outros cenários e não apareça aqui é do mundo, não do
controlador.

## Rodar

```bash
# terminal 1
./scripts/run_quadruped_sim.sh quadruped_empty.sdf

# terminal 2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=69
python3 scripts/scenario_check.py --seconds 20

# terminal 2 -- caminhada instrumentada
./scripts/gait_trial.sh /tmp/s0.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 5 --walk 8 --hold 8
```

Espere `state=fixed stand` no terminal 1 antes de comandar qualquer coisa.
Comandar antes disso dá um robô que nunca entra em trote, sem erro que explique.

## Aceitação

Medido em 20/08/2026, RTF 1,00:

| medida | valor | aceitação |
| --- | --- | --- |
| `/clock` | 1000 Hz | > 50 |
| `/demo/imu` | 1000 Hz | > 50 |
| `/demo/odom` | 50 Hz | > 10 |
| `/demo/scan` | 10 Hz | > 5 |
| `/demo/camera/image_raw` | 10 Hz, 640×480 rgb8 | > 5, geometria = `camera_info` |
| `RECOVER` | **0** | 0 |
| tilt de pico andando | 1,08° | < 3° |
| `z` | 0,343–0,359 m | faixa < 3 cm |
| deriva de rumo em 5 ciclos | −0,7° | < 5° |
| velocidade média | 0,1115 m/s | 0,10 comandado, 97–115% |

O relatório inteiro sai `PASSOU: 0 problema(s)`.

## Armadilha específica deste cenário

**O lidar devolve 7% de feixes válidos, e isso está correto.** Medido: 45–46 de
640 feixes, entre 4,66 e 9,78 m. Não é o sensor com defeito e não é "quase nada
funcionando" — são os feixes inferiores acertando o **plano do chão** a alguns
metros de distância. Os outros 93% vão para o horizonte e voltam infinito, porque
não há nada na altura do plano de varredura.

Se você usar este mundo para testar costmap, o costmap enche de um anel de chão a
5–10 m e nenhum obstáculo. Use o S3 para obstáculo de verdade.

**A árvore TF é completa no robô e aberta no topo.** Medido: 20 arestas, 8
estáticas, raiz `base`; `base` → `trunk` → `lidar`, `imu_link`, `front_camera` e
as quatro pernas até os pés. Faltam só `odom → base` e `map → odom`.

Cuidado com a medição: `/tf_static` usa durabilidade `TRANSIENT_LOCAL`. As
juntas fixas são publicadas **uma vez** na subida do `robot_state_publisher` e
retidas para quem assinar depois. Um assinante com QoS padrão (`VOLATILE`) não
recebe nada e conclui que a árvore não tem as juntas fixas. A primeira versão do
`scenario_check.py` cometeu exatamente esse erro e relatou 12 arestas com raiz em
`trunk`, o que estava errado.
