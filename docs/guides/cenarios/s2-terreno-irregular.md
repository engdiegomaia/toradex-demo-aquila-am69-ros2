# S2 — Terreno irregular

Mundo: `quadruped_rough.sdf` · Estação x86 · ~5 min

## Para que serve

Mede a colocação de pé (`FeetEndCalc`) e o estimador de estado quando o apoio
não está todo no mesmo plano. É o cenário que mais estressa o **Defeito 1**: o
estimador integra cinemática de perna, e apoio irregular é exatamente onde essa
integração erra mais.

## Geometria, e por que ela é assim

Sete lajes de 0,40 × 0,30 × **0,03 m**, entre x = 1,2 e x = 5,4, alternando
±0,15 m fora do eixo central.

Os 3 cm não são arbitrários: a elevação de pé da marcha é 8 cm
(`gait.foot_height` em `gait_go2.yaml`), então 3 cm o pé transpõe com folga e o
que se mede é a reação do equilíbrio ao apoio irregular — não a capacidade de
subir degrau. Uma laje de 8 cm ou mais mede outra coisa e provavelmente derruba
o robô.

Ficam fora do eixo para que os pés esquerdo e direito pisem em alturas
diferentes no mesmo instante, que é o caso interessante.

## Rodar

```bash
./scripts/run_quadruped_sim.sh quadruped_rough.sdf
./scripts/gait_trial.sh /tmp/s2.csv --v-cmd 0.10 --w-cmd 0.0 \
  --cycles 1 --walk 60 --hold 5
```

## Aceitação

Medido em 20/08/2026, RTF 1,00, 60 s de caminhada a 0,10 m/s. **Atravessa.**

| medida | S0 plano | S2 irregular | aceitação |
| --- | --- | --- | --- |
| `RECOVER` | 0 | **0** | 0 |
| tilt de pico andando | 1,08° | **2,81°** | < 6° |
| `footErr` máximo | 0,021 m | **0,026 m** | < 0,04 m |
| `footErr` médio | 0,003 m | 0,005 m | < 0,01 m |
| `z` | 0,343–0,359 m | 0,345–0,375 m | faixa < 4 cm |
| deriva de rumo | −0,7° | −0,7° | < 5° |
| velocidade média | 0,1115 m/s | 0,1118 m/s | 97–115% |

As lajes de 3 cm custam ~1,7° de tilt e 5 mm de `footErr`, e nenhuma queda. O
`z` sobe 1,6 cm de faixa, que é o corpo passando por cima das lajes.

### O resultado que interessa: o estimador piora, e mede-se quanto

| cenário | Δ estimado (x, y) | Δ real (x, y) | erro em y | caranguejo |
| --- | --- | --- | --- | --- |
| S0 plano | 3,972 · −0,007 | 4,330 · +0,138 | −0,145 m | 3,20% |
| **S2 irregular** | 5,544 · +0,133 | 5,972 · +0,377 | **−0,244 m** | **6,31%** |

O caranguejo **dobra** em terreno irregular, e o erro do estimador em y dobra
junto. É a confirmação numérica de que os dois são a mesma coisa: o Defeito 1 é
integração de cinemática de perna, e apoio irregular é onde essa integração erra
mais. Este é o cenário de referência para qualquer trabalho futuro no estimador.

## O que olhar na linha de supervisor

`footErr` é a métrica deste cenário. No plano vazio ele é simétrico e abaixo de
2 cm; aqui ele salta quando um pé encontra laje e o diagonal não. Um `footErr`
assimétrico e **persistente** (não um pico) indica alvo de apoio obsoleto, que é
uma assinatura conhecida (§10 do guia de testes).

## Armadilha específica deste cenário

**Não compare a deriva lateral deste cenário com a do S0 sem olhar `estPos`.**
Medido: o caranguejo dobra aqui (3,20% → 6,31%). Isso **não** é a marcha piorando
— é o estimador errando o dobro, e o robô rastreando fielmente a crença errada.
Compare `estPos` da linha de supervisor com o `/demo/odom` real antes de mexer em
qualquer ganho. Um ajuste de `k_y` "para corrigir" isto já foi medido no plano e
refutado (`../../results/ml35-postura-parada.md`).
