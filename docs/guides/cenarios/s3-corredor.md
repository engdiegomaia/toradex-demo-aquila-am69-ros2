# S3 — Corredor com obstáculos

Mundo: `quadruped_corridor.sdf` · Estação x86 · ~5 min

## Para que serve

Exercita `/demo/scan` e o caminho que a F5 vai usar: scan → camada de custo do
Nav2. **Não exercita marcha:** o chão é plano de propósito, para que qualquer
coisa que apareça nos números de marcha seja atribuível ao S1/S2 e não a este.

## Geometria, e por que ela é assim

Corredor de 1,5 m de largura por 7 m, paredes de 0,10 m de espessura e
**0,60 m de altura**, parede de fundo em x = 7,5, e dois cilindros de 0,15 m de
raio desalinhados em x = 2,5 (y = +0,25) e x = 5,0 (y = −0,25).

A altura de 0,60 m é o número que importa. O lidar do Go2 fica no corpo, a
~0,33 m do chão; uma parede de 0,30 m entra e sai do plano de varredura conforme
o tronco oscila e produz um scan **intermitente**, que parece defeito de bridge e
não é. 0,60 m garante retorno mesmo com o corpo balançando.

1,5 m de largura é folgado o suficiente para o Nav2 planejar e estreito o
suficiente para as duas paredes aparecerem no mesmo scan.

## Rodar

```bash
./scripts/run_quadruped_sim.sh quadruped_corridor.sdf
# verificar o lidar ANTES de andar
python3 scripts/scenario_check.py --seconds 20
# depois andar o corredor
./scripts/gait_trial.sh /tmp/s3.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 50 --hold 5
```

Inspeção direta do scan:

```bash
ros2 topic echo /demo/scan --once | head -20
ros2 topic hz /demo/scan
```

## Aceitação

Medido em 20/08/2026, RTF 1,00, 50 s de caminhada a 0,10 m/s.

| medida | S0 plano | S3 corredor | aceitação |
| --- | --- | --- | --- |
| lidar: feixes válidos | 45/640 (**7%**) | **358/640 (56%)** | > 30% |
| lidar: alcance mínimo | 4,66 m (chão) | **0,72 m** (parede) | < 1,5 m |
| lidar: alcance máximo | 9,76 m | 9,80 m | — |
| `RECOVER` | 0 | **0** | 0 |
| tilt de pico andando | 1,08° | 1,38° | < 3° |
| deriva de rumo | −0,7° | −0,4° | < 5° |
| velocidade média | 0,1115 m/s | 0,0967 m/s | 97–115% |

Os 7% → 56% são o critério deste cenário. Um mínimo de 0,72 m confirma que a
parede lateral está sendo vista a 0,75 m do eixo, que é onde ela está.

## Armadilhas específicas deste cenário

- **O robô vai bater na parede de fundo se você andar demais.** 50 s a 0,10 m/s
  são ~5 m; a parede está em 7,5 m. Com RTF acima do esperado ou comando maior,
  reduza o tempo.
- **O caranguejo de ~2% leva o robô para a parede.** Em 5 m isso são ~10 cm de
  desvio lateral num corredor de 1,5 m — tolerável, mas é a razão pela qual este
  corredor não é mais estreito. Não interprete a aproximação da parede como
  falha de navegação: não há navegação rodando aqui.
- **Costmap vazio não é defeito de bridge se a TF não fecha.** Sem frame `odom` o
  Nav2 não consegue colocar o scan num mapa. Verifique a árvore antes de
  investigar o lidar.
