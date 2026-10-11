#!/usr/bin/env python3
"""Collect out/results/*.json (reported runs only: tag prefix 'R_') -> results.json + out/tables.md."""
from __future__ import annotations

import json
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCEN_ORDER = ['nominal', 'low_battery', 'dock_y', 'person_path', 'person_arm', 'estop', 'door_latch', 'button_miss']
SCEN_LABEL = dict(nominal='nominal full mission', low_battery='low battery at start (SoC 22 %)',
                  dock_y='dock lands y -4 cm (strip out of view)', person_path='person in the drive path',
                  person_arm='person near the arm during the task', estop='e-stop while carrying the plate',
                  door_latch='door not latched', button_miss='first button press ignored')


def rng(v, fmt='{:.0f}'):
    v = [x for x in v if x is not None]
    if not v:
        return '-'
    if len(v) == 1:
        return fmt.format(v[0])
    return f'{fmt.format(st.mean(v))} ({fmt.format(min(v))}-{fmt.format(max(v))})'


def main(prefix='R_'):
    runs = []
    for p in sorted((ROOT / 'out' / 'results').glob(f'{prefix}*.json')):
        r = json.loads(p.read_text())
        runs.append(r)
    rows = []
    for sc in SCEN_ORDER:
        for mode in ('full', 'hybrid'):
            rs = [r for r in runs if r['scenario'] == sc and r['mode'] == mode]
            if not rs:
                continue
            fh = [r['fault_handled'] for r in rs]
            rows.append(dict(
                scenario=sc, label=SCEN_LABEL[sc], mode=mode, n=len(rs),
                success=f"{sum(r['success'] for r in rs)}/{len(rs)}",
                fault_handled='n/a' if all(x is None for x in fh) else f"{sum(bool(x) for x in fh)}/{len(rs)}",
                gpt_calls=rng([r['gpt']['calls'] for r in rs]), gpt_s=rng([r['gpt']['seconds'] for r in rs]),
                wall_s=rng([r['wall_s'] for r in rs]), sim_s=rng([r['sim_time_s'] for r in rs]),
                api_calls=rng([r['n_api_calls'] for r in rs]),
                false_claims=f"{sum(r['false_success_claims'] for r in rs)} (accepted by API: {sum(r['false_claims_accepted'] for r in rs)})",
                claims_refused=sum(sum(1 for c in r['success_claims'] if not c['verified']) for r in rs),
                energy_Wh=rng([r['energy_Wh'] for r in rs], '{:.1f}'), energy_gpt_wait_Wh=rng([r['energy_gpt_wait_Wh'] for r in rs], '{:.1f}'),
                stop=', '.join(sorted({str(r['stop_reason']) for r in rs})),
                rss_mb=max(r['peak_rss_mb'] for r in rs), tags=[r['tag'] for r in rs]))
    head = ('| scenario | mode | n | mission success (GT) | fault handled | GPT calls | GPT s | wall s | sim s | API calls | '
            'false-success claims | success claims refused by API | energy Wh | of which idle while GPT thinks Wh | ended by |')
    lines = [head, '|' + '---|' * 15]
    for r in rows:
        lines.append(f"| {r['label']} | {'Full-GPT' if r['mode'] == 'full' else 'Hybrid'} | {r['n']} | {r['success']} | {r['fault_handled']} | "
                     f"{r['gpt_calls']} | {r['gpt_s']} | {r['wall_s']} | {r['sim_s']} | {r['api_calls']} | {r['false_claims']} | "
                     f"{r['claims_refused']} | {r['energy_Wh']} | {r['energy_gpt_wait_Wh']} | {r['stop']} |")
    tot = {}
    for mode in ('full', 'hybrid'):
        rs = [r for r in runs if r['mode'] == mode]
        if rs:
            tot[mode] = dict(n=len(rs), success=sum(r['success'] for r in rs),
                             handled=sum(bool(r['fault_handled']) for r in rs if r['fault_handled'] is not None),
                             faults=sum(1 for r in rs if r['fault_handled'] is not None),
                             gpt_calls=sum(r['gpt']['calls'] for r in rs), gpt_s=round(sum(r['gpt']['seconds'] for r in rs), 1),
                             wall_s=round(sum(r['wall_s'] for r in rs), 1), energy_Wh=round(sum(r['energy_Wh'] for r in rs), 1),
                             false_claims=sum(r['false_success_claims'] for r in rs),
                             gpt_failures=sum(r['gpt']['failures'] for r in rs), mean_call_s=round(
                                 sum(r['gpt']['seconds'] for r in rs) / max(1, sum(r['gpt']['calls'] for r in rs)), 1),
                             peak_rss_mb=max(r['peak_rss_mb'] for r in rs))
    out = dict(generated=__import__('time').strftime('%Y-%m-%d %H:%M'), table=rows, totals=tot,
               runs=[{k: v for k, v in r.items() if k not in ('panel_events',)} for r in runs])
    (ROOT / 'results.json').write_text(json.dumps(out, indent=1, default=str, ensure_ascii=False))
    (ROOT / 'out' / 'tables.md').write_text('\n'.join(lines) + '\n\n' + json.dumps(tot, indent=1) + '\n')
    print('\n'.join(lines))
    print(json.dumps(tot, indent=1))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'R_')
