# S1 — Rampa de 6°

Mundo: `quadruped_ramp.sdf` · Estação x86 · ~5 min

## Para que serve

Mede o equilíbrio da QP em inclinação, o limite de `tilt` do supervisor
(`RECOVER` acima de 12°) e a estimativa de altura do corpo quando o apoio não
está em z = 0.

## Geometria, e por que ela é assim

Caixa de 4 × 2 × 0,2 m inclinada 6°, com a **borda superior próxima exatamente
em z = 0, x = 1,0 m**. Topo da rampa em x = 4,98 m, z = 0,42 m, seguido de um
**platô** plano de x = 4,98 até x = 7,0 na altura do topo.

O cálculo do `<pose>` no SDF existe por um motivo: uma caixa simplesmente
rotacionada em torno do próprio centro deixa um degrau na entrada, e o robô
tropeça nele em vez de subir a rampa — o teste passaria a medir transposição de
degrau, não inclinação. A extremidade próxima afunda abaixo do plano do chão de
propósito, para não haver lábio nenhum.

6° é deliberadamente modesto: é a primeira inclinação a medir, não a última.

**O platô não é decoração.** A primeira versão deste mundo terminava a rampa
numa face vertical. A corrida de 20/08/2026 mostrou o robô subindo os 3,98 m com
tilt abaixo de 0,43° e então **caindo de um penhasco de 0,42 m** na borda de
cima: tilt de pico 155,95°, deriva de rumo −162,7°, 84 `RECOVER`. O resultado
parecia "não sobe rampa" e era "não tem onde chegar" — a falha aconteceu em
x = 4,968, ou seja 3,97 m de uma rampa de 3,98 m.

Vale como lição sobre este harness: um resultado catastrófico com tilt de 155°
merece olhar *onde* no percurso a falha começou antes de concluir qualquer coisa
sobre o controlador. A tabela de `x` contra `tilt` respondeu isso em uma linha.

## Rodar

```bash
./scripts/run_quadruped_sim.sh quadruped_ramp.sdf
# em outro terminal, com o ambiente ROS:
./scripts/gait_trial.sh /tmp/s1.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 60 --hold 5
```

60 s de caminhada: a 0,10 m/s o robô leva ~40 s para vencer os 4 m da rampa a
partir de x = 1,0, e a margem cobre o RTF abaixo de 1.

## Aceitação

Medido em 20/08/2026, RTF 1,00, 70 s de caminhada a 0,10 m/s. **Sobe e
atravessa o platô.**

| medida | S0 plano | S1 rampa 6° | aceitação |
| --- | --- | --- | --- |
| `RECOVER` | 0 | **0** | 0 |
| tilt de pico andando | 1,08° | **0,96°** | < 4° |
| `z` | 0,343–0,359 m | **0,345 → 0,778 m** | sobe 0,42 m com o terreno |
| deriva de rumo | −0,7° | 2,8° | < 5° |
| velocidade média | 0,1115 m/s | 0,1063 m/s | 97–115% |
| deslocamento líquido | 4,33 m | 7,16 m | passa do topo (x = 4,98) |

O tilt de pico **abaixo** do plano vazio é o resultado que importa: a inclinação
de 6° não custa nada ao equilíbrio. A rampa não é o limite; é o piso a partir do
qual medir 10° e 15°.

Note o platô de 7 m. As duas versões anteriores caíram em bordas — primeiro a
próxima (sem platô), depois a distante (platô de 2 m). Se aumentar o tempo de
caminhada, aumente o platô junto.

## Armadilhas específicas deste cenário

- **`z` cresce e isso não é o robô se levantando.** O `/demo/odom` é pose
  absoluta no mundo; subindo a rampa, `z` sobe com o terreno. Qualquer script que
  use `z` como detector de queda — inclusive o `demo_routine` — precisa de um
  limiar relativo ao terreno neste mundo, e o `min_z` default de 0,28 m passa a
  ser inútil depois do primeiro metro de subida.
- **A inclinação melhora o caranguejo, e isso é do estimador.** Medido: 2,38% na
  rampa contra 3,20% no plano e 6,31% em terreno irregular. Não é a marcha ficando
  melhor em subida — é que a integração de cinemática de perna erra menos quando
  o apoio é regular e inclinado do que quando é regular e plano. Compare `estPos`
  com o `/demo/odom` antes de tirar qualquer conclusão de marcha.
- **Subir para 10° e 15° é o passo seguinte, uma variável por corrida.** Não mude
  a inclinação junto com qualquer ganho de marcha; a regra do projeto existe
  porque quatro experimentos já foram revertidos por isso.
