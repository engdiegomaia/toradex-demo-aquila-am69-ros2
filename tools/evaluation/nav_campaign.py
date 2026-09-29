#!/usr/bin/env python3
"""
Conduz uma campanha A/B INTERCALADA de navegação e resume por mediana e faixa.

    python3 tools/evaluation/nav_campaign.py artifacts/campaign-align8 \
        --condition baseline='<comando que reaplica o baseline>' \
        --condition align8='<comando que aplica a condição>' \
        --reps 3 --seconds 420

## Por que este script existe

`nav_trial.py` mede UMA corrida. `summarize_trials.py` resume replicatas que
já existem. O que faltava era o meio: quem decide a ORDEM das corridas e o que
acontece entre elas. Sem isso a comparação continua indefensável, e este projeto
já pagou por isso.

O número que manda: em configuração IDÊNTICA a velocidade média variou **2,4×**
entre corridas (`docs/results/ml35-f5-clock-fanout.md`). Uma corrida por
condição não distingue efeito de ruído — nem quando o efeito é real. Duas
consequências de protocolo, e as duas estão implementadas aqui:

- **Intercalar, não blocar.** A ordem é `A B A B A B`, nunca `A A A B B B`.
  Qualquer deriva ao longo da campanha — térmica, cache do host, memória do
  container, alguém usando a rede — entra igualmente nas duas condições em vez
  de virar diferença entre elas.
- **Mesmo estado inicial em toda perna.** Entre pernas o robô volta à pose de
  nascimento e o costmap é esvaziado. Sem isso a perna 2 começa de onde a perna
  1 parou, e "condição B" passa a significar "condição B partindo de um lugar
  diferente".

## O reset entre pernas só é possível desde 26/08/2026

`/demo/sim/reset` APAGAVA o robô do mundo (`reset.all` devolve o mundo ao SDF
de origem, que não contém um modelo inserido por `create`). Uma campanha que
chamasse reset entre pernas mediria, da perna 2 em diante, um mundo sem robô —
com o relógio andando e os sensores órfãos publicando, ou seja, sem nada
acusando. Ver `docs/results/cockpit-reset-nao-destrutivo.md`.

Hoje o reset teleporta e a planta sobrevive, e é isso que torna este laço
honesto. Se você reverter aquele conserto, ESTE script passa a mentir.

## Como uma condição é aplicada, e por que não é `ros2 param set` embutido

Cada condição carrega um COMANDO DE SHELL, fornecido por quem roda. Deliberado:
os pesos dos críticos do MPPI são lidos no `on_configure` do controlador, então
um `ros2 param set` em `FollowPath.PathAlignCritic.cost_weight` é aceito, lê de
volta o valor novo e **pode não mudar o comportamento**. Um script que aplicasse
`param set` por conta própria produziria uma campanha inteira comparando a
condição consigo mesma, com evidência de aparência perfeita.

Então a decisão fica com o operador, e a forma confiável é recriar a pilha com
o YAML da condição. Exemplo, no módulo:

    --condition align8='ssh torizon@<modulo> "cd /home/torizon/demo &&
        NAV2_PARAMS=/ws/src/demo_navigation/config/params-align8.yaml \
        docker compose -f compose.module.yml up -d --force-recreate nav"'

Em uma comparação, inclusive `baseline` precisa de comando. Sem isso, depois da
primeira perna `align8` as pernas chamadas baseline continuariam usando align8.
O script recusa esse protocolo em vez de produzir um A/B falso.

O baseline deve passar `NAV2_PARAMS=__robot_default__` ao Compose. Valor vazio
produz o argumento inválido `params_override:=` antes que o launch possa escolher o
default do robô.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent

# Serviços de reposição. `sim/reset` põe o robô no nascimento sem apagar nada;
# `nav/reset` esvazia o costmap acumulado. São dois porque vivem em máquinas
# diferentes no modo hil, e a granularidade separada é o que permite recolocar o
# robô sem derrubar o Nav2.
SIM_RESET = '/demo/sim/reset'
NAV_RESET = '/demo/nav/reset'

# Os três managed nodes cujo estado de ciclo de vida decide se uma meta é
# aceita ou recusada com "Action server is inactive". Mesmo conjunto que
# `module.sh verify` checa para bt_navigator (ver aquele script para o
# incidente de 26/08/2026 que motivou a checagem).
READINESS_NODES = ('/bt_navigator', '/controller_server', '/planner_server')

# Depois de teleportar, o quadrúpede reassenta na altura de marcha por conta
# própria (0,5 m de nascimento -> ~0,35 m de marcha, medido). Começar a medir
# durante a queda contaminaria a primeira janela de amostras da perna.
SETTLE_S = 6.0


def _run(command: list[str], timeout: float) -> tuple[int, str]:
    try:
        done = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return 124, f'expirou depois de {timeout:.0f} s'
    return done.returncode, (done.stdout or '') + (done.stderr or '')


def call_trigger(service: str, timeout: float = 30.0) -> tuple[bool, str]:
    """Chama um std_srvs/Trigger e devolve (aceito, texto cru)."""
    code, output = _run(
        ['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger', '{}'],
        timeout,
    )
    # `ros2 service call` sai 0 mesmo quando o serviço respondeu success=False,
    # então o código de saída não basta: quem decide é o campo da resposta.
    accepted = code == 0 and 'success=True' in output
    return accepted, output.strip()


def lifecycle_state(node: str, timeout: float = 10.0) -> str:
    """Primeira palavra de `ros2 lifecycle get <node>` (`'active'`, `'inactive'`,
    `'unconfigured'`...), ou `''` se a chamada falhar ou não responder."""
    code, output = _run(['ros2', 'lifecycle', 'get', node], timeout)
    if code != 0:
        return ''
    line = output.strip().splitlines()[0] if output.strip() else ''
    return line.split()[0] if line else ''


def wait_for_managed_nodes(
    nodes: tuple[str, ...] = READINESS_NODES,
    *,
    timeout: float = 90.0,
    poll_interval: float = 2.0,
    verbose: bool = True,
) -> tuple[bool, float]:
    """
    Espera até todo `node` em `nodes` responder `active`, ou até `timeout`.

    Devolve (pronto, segundos_decorridos). O tempo decorrido é retornado
    mesmo quando pronto=True: bring-up lento é sinal a registrar, não só
    motivo de falha. Existe porque tópico existir não é serviço funcionar —
    depois de um `up`, os três managed nodes podem ficar de pé com o
    lifecycle manager abortado, e toda meta é recusada em silêncio até
    alguém notar (ver `module.sh verify`, etapa 4/4).
    """
    start = time.monotonic()
    deadline = start + timeout
    pending = set(nodes)
    while True:
        for node in list(pending):
            if lifecycle_state(node) == 'active':
                pending.discard(node)
        elapsed = time.monotonic() - start
        if not pending:
            return True, elapsed
        if time.monotonic() >= deadline:
            if verbose:
                print(f'  managed nodes que nao ativaram: {", ".join(sorted(pending))}')
            return False, elapsed
        time.sleep(poll_interval)


def _append_manifest(out_dir: Path, **record) -> None:
    """Acrescenta uma linha ao manifesto da campanha; nunca sobrescreve uma
    perna anterior, mesmo quando a campanha aborta na perna seguinte."""
    with (out_dir / 'manifesto.jsonl').open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')


def reset_between_legs(*, skip_nav: bool, verbose: bool = True) -> bool:
    """
    Devolve robô e costmap ao estado inicial. False se algo recusou.

    Recusa é ABORTO, não aviso: uma perna que começa de um estado diferente das
    outras não é replicata, e incluí-la na mediana estraga exatamente o número
    que a campanha existe para produzir.
    """
    ok, detail = call_trigger(SIM_RESET)
    if verbose:
        print(f'  reset do simulador: {"ok" if ok else "RECUSADO"}')
    if not ok:
        print(f'  {detail}', file=sys.stderr)
        return False

    if not skip_nav:
        ok, detail = call_trigger(NAV_RESET, timeout=60.0)
        if verbose:
            print(f'  reset da navegação: {"ok" if ok else "RECUSADO"}')
        if not ok:
            print(f'  {detail}', file=sys.stderr)
            return False

    time.sleep(SETTLE_S)
    return True


def parse_condition(raw: str) -> tuple[str, str | None]:
    """`nome` ou `nome=comando de shell`."""
    name, sep, command = raw.partition('=')
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError(f'condição sem nome: {raw!r}')
    return name, (command if sep else None)


def validate_conditions(conditions: list[tuple[str, str | None]]) -> None:
    """Exige reaplicacao explicita de cada lado de uma comparacao."""
    if len(conditions) < 2:
        return
    missing = [name for name, command in conditions if not command]
    if missing:
        raise ValueError(
            'comparacao exige comando em toda condicao; sem reaplicar '
            + ', '.join(missing)
            + ', a perna herda a configuracao anterior')


def leg_order(conditions: list[tuple[str, str | None]], reps: int):
    """
    A ordem intercalada, achatada: (rep, nome, comando).

    Exportada para poder ser testada sem simulador: a ordem É o protocolo, e um
    laço trocado transforma a campanha em blocada sem que nada acuse.
    """
    for rep in range(1, reps + 1):
        for name, command in conditions:
            yield rep, name, command


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('out_dir', help='diretório dos CSVs e do resumo')
    parser.add_argument('--condition', action='append', default=[],
                        metavar='NOME[=COMANDO]',
                        help='repita para cada condição; sem =COMANDO não '
                             'aplica nada (referência)')
    parser.add_argument('--reps', type=int, default=3,
                        help='replicatas por condição (protocolo pede n >= 3)')
    parser.add_argument('--seconds', type=float, default=420.0,
                        help='duração de cada perna, passada ao nav_trial')
    parser.add_argument('--goals', default='maze11')
    parser.add_argument('--goal-timeout', type=float, default=90.0,
                        help='prazo por meta repassado ao nav_trial')
    parser.add_argument('--skip-nav-reset', action='store_true',
                        help='não chamar /demo/nav/reset entre pernas')
    parser.add_argument('--apply-timeout', type=float, default=300.0,
                        help='teto para o comando de uma condição')
    parser.add_argument('--readiness-timeout', type=float, default=90.0,
                        help='teto para bt_navigator/controller_server/'
                             'planner_server ficarem active depois de '
                             'aplicar uma condição')
    parser.add_argument('--dry-run', action='store_true',
                        help='imprime a ordem das pernas e sai')
    args = parser.parse_args(argv)

    if not args.condition:
        print('nenhuma condição: use --condition ao menos uma vez',
              file=sys.stderr)
        return 2
    if args.reps < 3 and not args.dry_run:
        # Aviso, não erro: n < 3 é legítimo para depurar o próprio laço. O que
        # não é legítimo é DECIDIR com ele, e o resumo carrega o n.
        print(f'AVISO: reps={args.reps} fica abaixo do protocolo (n >= 3); '
              'a dispersão de 2,4x em configuração idêntica continua valendo.')

    conditions = [parse_condition(raw) for raw in args.condition]
    try:
        validate_conditions(conditions)
    except ValueError as error:
        parser.error(str(error))
    legs = list(leg_order(conditions, args.reps))

    print(f'campanha intercalada: {len(conditions)} condições x {args.reps} '
          f'replicatas = {len(legs)} pernas de {args.seconds:.0f} s')
    print(f'tempo mínimo de pista: {len(legs) * args.seconds / 60:.0f} min '
          f'(sem contar bring-up e reposição)')
    print('ordem: ' + ' '.join(name for _, name, _ in legs))
    if args.dry_run:
        return 0

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    produced: list[tuple[str, Path]] = []

    for index, (rep, name, command) in enumerate(legs, start=1):
        csv_path = out_dir / f'{name}-r{rep}.csv'
        print(f'\n--- perna {index}/{len(legs)}: {name}, replicata {rep} ---')

        if command:
            print(f'  aplicando condição: {command}')
            code, output = _run(shlex.split(command), args.apply_timeout)
            if code != 0:
                print(f'  a condição FALHOU (código {code}); abortando a '
                      f'campanha para não gravar replicata inválida\n{output}',
                      file=sys.stderr)
                return 1

        # `docker compose up -d --force-recreate` (o comando típico de uma
        # condição) retorna antes de bt_navigator/controller_server/
        # planner_server ficarem active. Uma perna que começa a coletar
        # nesse meio-tempo mede "Action server is inactive" em vez do
        # comportamento da condição.
        ready, bringup_s = wait_for_managed_nodes(timeout=args.readiness_timeout)
        print(f'  readiness dos managed nodes: {"ok" if ready else "TIMEOUT"} '
              f'em {bringup_s:.1f}s')
        _append_manifest(
            out_dir, rep=rep, condition=name, command_applied=bool(command),
            readiness_ready=ready, readiness_seconds=round(bringup_s, 1),
        )
        if not ready:
            print(f'  managed nodes nao ficaram active em '
                  f'{args.readiness_timeout:.0f}s; abortando a campanha',
                  file=sys.stderr)
            return 1

        if not reset_between_legs(skip_nav=args.skip_nav_reset):
            print('  reposição recusada; abortando a campanha', file=sys.stderr)
            return 1

        code, output = _run(
            [sys.executable, str(HERE / 'nav_trial.py'), str(csv_path),
             '--seconds', str(args.seconds), f'--goals={args.goals}',
             '--goal-timeout', str(args.goal_timeout)],
            timeout=args.seconds + 300.0,
        )
        print(output.strip()[-2000:])
        if not csv_path.exists():
            print(f'  a perna não gravou {csv_path}; abortando',
                  file=sys.stderr)
            return 1
        produced.append((name, csv_path))

    print('\n===== resumo por condição (mediana e faixa observada) =====')
    code, output = _run(
        [sys.executable, str(HERE / 'summarize_trials.py')]
        + [f'{name}={path}' for name, path in produced],
        timeout=120.0,
    )
    print(output.strip())

    summary = out_dir / 'resumo.txt'
    summary.write_text(output, encoding='utf-8')
    print(f'\nresumo gravado em {summary}')
    return code


if __name__ == '__main__':
    sys.exit(main())
