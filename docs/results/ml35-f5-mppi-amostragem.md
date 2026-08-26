# ML3.5 F5 — reduzir amostragem do MPPI: A/B reprovado, e o método também

**Estado:** EXECUTADO — **hipótese REFUTADA; alteração NÃO adotada**
**Data:** 25/08/2026 (noite)
**Baseline do módulo:** Aquila AM69 V1.0A, Torizon OS 7.7.0, 8 × Cortex-A72

Execução no AM69 real, com câmera comprimida (fase 2 já em vigor). Robô e
sensores simulados no host: **não valida localização por pernas, Go2 físico,
térmica ou consumo** (regras 5 e 7).

## O que foi testado

`time_steps` 96 → 64 e `model_dt` 0,10 → 0,15, **horizonte constante**: os dois
pares dão 9,6 s. O custo teórico é `batch_size × time_steps`, então 96k → 64k
pontos por iteração, 33% menos amostragem.

A hipótese era a do plano: **cortar CPU do Nav2**, que a sessão anterior mediu
em 366–531% com a percepção junto, saturando o módulo e deixando a nuvem do
LiDAR 1,0 s atrasada.

Os valores em execução foram **provados no runtime**, não presumidos do build:

```text
FollowPath.time_steps        Integer value is: 64
FollowPath.model_dt          Double value is: 0.15
FollowPath.batch_size        Integer value is: 1000
```

## Resultado: a hipótese caiu

| Medida | comprimido 96 × 0,10 | comprimido 64 × 0,15 |
| --- | ---: | ---: |
| **CPU do `nav`** | **~437%** | **~439%** |
| `Ignoring the source` | 11 | 5 |
| `Robot to stop due to invalid source` | 4 | 1 |
| velocidade média | 0,0109 / 0,0265 m/s | 0,0353 m/s |
| razão de trabalho `cmd_vx` | 2,7% / 2,1% | 6,7% |
| `vx` ≈ 0 | 91,7% / 85,1% | 75,7% |
| quedas | 1 de 2 corridas | **caiu aos 276 s** |
| metas de 8 m | 0 | 0 |

CSV: `ml35-f5-mppi64-run1.csv`.

**Trinta e três por cento menos pontos de amostragem não reduziram CPU:
~437% → ~439%.** O custo do MPPI neste alvo **não é dominado por
`batch_size × time_steps`**, ao contrário do que o comentário de CUSTO no
`nav2_params_go2.yaml` assumia desde 20/08. Quem for atacar CPU do Nav2 de novo
deve **começar medindo onde ela é gasta** — perfilando o processo — em vez de
mexer neste par.

## O achado que vale mais que o A/B: o método não decide

As demais métricas vieram todas melhores. Mesmo assim **não sustentam adoção**,
e a razão é aritmética simples:

> Duas corridas na configuração **idêntica** (comprimido 96 × 0,10) deram
> **0,0109 e 0,0265 m/s** — dispersão de **2,4×**.

O ruído entre corridas é maior que o efeito que se quer medir. Portanto
**qualquer A/B de n=1 nesta bancada é ininterpretável**, incluindo este. O
número 0,0353 m/s pode ser efeito do parâmetro ou pode ser a mesma loteria que
produziu 0,0265 contra 0,0109.

Isso invalida o método, não só este resultado. **Antes de qualquer próxima
sintonia, o protocolo precisa mudar:** n ≥ 3 por condição, com mediana e faixa
reportadas, e o par de condições intercalado para não confundir efeito com
deriva da bancada.

## Quedas: já não é variância

| Configuração | Corridas | Quedas |
| --- | ---: | ---: |
| RAW no fio | 2 | 0 |
| comprimido 96 × 0,10 | 2 | 1 |
| comprimido 64 × 0,15 | 1 | 1 |

**Duas quedas em três corridas depois da câmera comprimida, contra zero em duas
antes.** Na sessão anterior a queda isolada foi registrada como variância; com
esta terceira corrida essa leitura não se sustenta mais. Não há mecanismo
identificado, e a correlação com a mudança de transporte **não é causa
demonstrada** — mas é sinal suficiente para virar item de investigação, não
nota de rodapé.

## Decisão tomada

A alteração **não foi adotada**. O `nav2_params_go2.yaml` volta a 96 × 0,10, com
o A/B documentado no cabeçalho do próprio parâmetro e fora do caminho default —
mesma política do nó de estrangulamento do `/clock`. O container do módulo foi
restaurado a partir da imagem e reconfirmado em 96 / 0,10.

## Próximo portão, revisado pelos dados

1. **Consertar o protocolo antes de sintonizar.** n ≥ 3 por condição,
   intercalado, mediana e faixa. Sem isso nenhuma das fases seguintes produz
   conclusão.
2. **Investigar as quedas.** Duas em três corridas é regressão de estabilidade
   até prova em contrário, e estabilidade é pré-requisito de qualquer meta.
3. **Perfilar o Nav2** para descobrir onde os ~440% são gastos, já que não é a
   amostragem do MPPI.
4. Só depois disso, MPPI (`PathAlignCritic` × `PathAngleCritic`) e a repetição do
   portão.

Continua valendo a alternativa de produto, agora com mais peso: **trocar o MPPI
por DWB no perfil do módulo**, ou aceitar que o AM69 roda Nav2 **ou** percepção
com esta configuração.
