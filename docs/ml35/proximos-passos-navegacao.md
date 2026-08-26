# Próximos passos — decisão de trajeto do Nav2 (F5)

Escrito em 26/08/2026, depois de fechar o reset do cockpit.
Estado autoritativo em `docs/ml35/estado-fases.md`; este arquivo é só o roteiro
da próxima frente.

---

## 1. O sintoma, e o que ele já NÃO é

O robô anda — a marcha está sã — mas não cumpre rota. Última medição
(`docs/results/ml35-f5-clock-fanout.md`):

| métrica | valor |
| --- | ---: |
| `vx` ≈ 0 (`\|cmd_vx\| <= 0,005 m/s`) | **90,7%** das amostras |
| girando | **90,3%** das amostras |
| velocidade média | 0,0246 m/s |
| metas de 8 m cumpridas | **0 de 2** |
| quedas | 0 |

**Cinco hipóteses estão refutadas por medição. Não as reabra:**

| hipótese | como morreu |
| --- | --- |
| CPU do módulo | fechada: 307% de folga de 800%, **zero** recusas do `collision_monitor` (`ml35-f5-clock-fanout.md`) |
| taxa do `/clock` | estrangular derrubou a navegação em 21/08 (`demo_simulation/clock_throttle.py`) |
| amostragem do MPPI | reprovada em `ml35-f5-mppi-amostragem.md` |
| `inflation_radius` fechando o corredor | **64,1%** das células do `local_costmap` com custo 0; o que estava à frente era parede real |
| enlace de rede | gigabit full, RTT 0,400 ms, rota simétrica; as duas metas estouraram igual (`ml35-f5-ethernet0-repeticao.md`) |

Lidar e odometria foram auditados e **estão sãos** (odom vs TF com erro
0,0000 m, sem auto-colisão de lidar). O defeito é de **decisão**.

## 2. Protocolo primeiro. Não sintonize antes disso

O número que invalida qualquer corrida única: em configuração **idêntica** a
velocidade média variou **2,4×** entre corridas. Uma perna por condição não
distingue efeito de ruído — nem quando o efeito é real.

`scripts/nav_campaign.py` existe para isso. Ele intercala `A B A B A B` em vez de
blocar `A A A B B B`, e repõe robô e costmap antes de cada perna:

```bash
. /opt/ros/jazzy/setup.bash && . ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/docker/cyclonedds/host.rendered.xml

python3 scripts/nav_campaign.py docs/results/campanha-align \
    --condition baseline \
    --condition align8='<comando que aplica a condição>' \
    --reps 3 --seconds 420
```

`--dry-run` imprime a ordem e o tempo de pista sem gastar bancada. Com 2
condições × 3 replicatas × 420 s são **42 min de pista**, fora bring-up e
reposição.

Só é possível porque o reset deixou de apagar o robô (commit `7a5e539`). Com o
reset antigo, a perna 2 em diante mediria um mundo sem planta — relógio andando,
sensores órfãos publicando, nada acusando. **Se aquele conserto for revertido,
este laço passa a mentir.**

**Métrica primária é razão de trabalho de `cmd_vx` e fração de `vx` ≈ 0**, não
velocidade média: deram 0,6% e 0,6% em duas corridas distintas, contra 2,4× de
dispersão na média. `summarize_trials.py` já reporta mediana e faixa por
condição, sem agrupar amostras.

## 3. Passo 0 — tornar uma condição aplicável de forma confiável

**Este passo vem antes da primeira campanha.** Hoje não há como trocar os
parâmetros do Nav2 de fora, e as duas saídas óbvias falham:

- `ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0`
  é **aceito e lê de volta o valor novo**, mas os pesos dos críticos são lidos no
  `on_configure` do controlador. Uma campanha construída sobre isso compara a
  condição consigo mesma, com evidência de aparência perfeita. É por isso que
  `nav_campaign.py` **não** aplica condição por conta própria: quem aplica é um
  comando fornecido por quem roda.
- `nav_select.launch.py` **não repassa `params_file`** (verificado: só repassa
  `use_sim_time`), e é ele que o `compose.module.yml` invoca. Então passar
  `params_file:=` no `command:` do compose não tem efeito.

O YAML instalado é symlink para dentro da imagem, não para o host:

```
/ws/install/demo_navigation/share/demo_navigation/config/nav2_params_go2.yaml
  -> /ws/build/demo_navigation/config/nav2_params_go2.yaml
  -> /ws/src/demo_navigation/config/nav2_params_go2.yaml     (COPY na imagem base)
```

**A correção recomendada é de três linhas:** repassar `params_file` em
`_launch_navigation` de `nav_select.launch.py`, e declarar o argumento com
default vazio (vazio = "use o default da planta", para não criar um segundo
default capaz de divergir). Aí uma condição passa a ser um ARQUIVO, e o comando
de aplicação fica:

```bash
--condition align8='ssh torizon@<modulo> "cd /home/torizon/demo &&
    docker compose -f compose.module.yml up -d --force-recreate nav"'
```

com o YAML da condição sincronizado por `module.sh sync` antes. Um teste que
trave o repasse evita a variante silenciosa desta armadilha (argumento declarado
mas não repassado, que é exatamente o estado de hoje).

## 4. Passo 1 — a hipótese a testar primeiro, e por quê

**`PathAlignCritic.cost_weight: 14.0`.** É o maior peso da lista, e o próprio
YAML já o nomeia duas vezes como o crítico que torna girar melhor que avançar:

- no bloco `POR QUE vx_min E ZERO`: *"num corredor de 1.20 m o PathAlignCritic
  (peso 14, o maior) acha ótimo recuar para realinhar"* — com `vx_min: -0.10`
  isso produzia deadlock contra a parede de trás, medido com ré em 100% das
  amostras;
- no item 13 do cabeçalho: o horizonte curto *"não alcança a referência do
  próprio PathAlignCritic (peso 14, offset 20 pontos ~ 1 m)"*.

`vx_min: 0.0` fechou a saída "recuar" — corretamente, porque ré contra a parede
era travamento. Mas a preferência por **realinhar antes de avançar** continua
intacta, e a única forma de realinhar que resta é **girar**. É consistente com o
sintoma medido: `vx` ≈ 0 em 90,7% e girando em 90,3%.

Contra-pesos que empurram para frente: `PreferForwardCritic` 5,0,
`PathFollowCritic` 5,0, `GoalCritic` 5,0 — cada um com pouco mais de um terço do
peso do alinhamento.

Condições sugeridas para a primeira campanha, uma variável por vez:

| condição | mudança | o que ela testa |
| --- | --- | --- |
| `baseline` | nenhuma | referência, com a dispersão medida sob o protocolo novo |
| `align8` | `PathAlignCritic.cost_weight` 14,0 → 8,0 | alinhamento deixa de dominar o progresso |

Só depois, e uma por campanha: `PathAngleCritic` 2,0 (o portão 3 já registrado),
e `PathFollowCritic` 5,0 → 8,0.

**Critério de parada que não se negocia:** folga de carcaça segue em **+6,5 cm**
e é o teto de qualquer aumento de velocidade. Vem de `robot_radius: 0.38` modelar
o tronco como círculo; a correção de verdade é footprint poligonal com
`consider_footprint: true`, e ela não é pré-requisito desta campanha.

## 5. O que decide

Uma condição vence se, com n ≥ 3 intercalado, a **mediana** da razão de trabalho
de `cmd_vx` subir e a fração de `vx` ≈ 0 cair, **e as faixas não se sobrepuserem**.
Faixas sobrepostas com n = 3 significam "não decidido", não "empate": aumente n
antes de escolher.

Meta de 8 m cumprida continua sendo o portão do F5, mas não é a métrica de
sintonia — é rara demais para discriminar entre condições.

## 6. Pendências fora desta frente

- **F4, controle manual no cockpit.** Bloqueado por infraestrutura, não por
  desenho: `twist_mux` não está em nenhuma imagem e o módulo **não tem rota
  default** (só a `10.22.1.0/24`), então `apt` não resolve nada lá. O gateway da
  LAN `10.22.1.1` responde em 0,337 ms e o módulo já tem DNS corporativo, então
  falta um comando, que precisa de permissão para rodar:

      sudo ip route add default via 10.22.1.1 dev ethernet0 metric 100

  Não é NAT: o módulo está na mesma `/24` do gateway, então não há o que
  masquerar e o firewall do host não precisa ser tocado. Para sobreviver a
  reboot, a rota tem de entrar na configuração de rede do Torizon, não só em
  runtime.
- **Passada visual do cockpit.** Nada nesta sessão foi visto num navegador: não
  há Chrome nesta máquina e o MCP de automação não dirige o Firefox instalado.
  Tudo foi medido no caminho de dados, com o cliente rosbridge do próprio
  cockpit. O gate visual continua pendente.
- **Telemetria do alvo não commitada.** `target_monitor` e as mudanças de
  `hmi/` estão verificadas (`docs/results/cockpit-reset-nao-destrutivo.md` §5)
  mas seguem fora do git, junto de outro trabalho da sessão anterior
  (`module.sh`, `wait_for_tf.py`, `sim.launch.py`).
