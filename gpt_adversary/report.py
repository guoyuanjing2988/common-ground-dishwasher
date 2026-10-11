#!/opt/homebrew/bin/python3
"""Aggregate searches + triage into results.json, report tables (logs/report_tables.md) and plots (out/*.png).
Run with /opt/homebrew/bin/python3 (matplotlib). Reads only files inside gpt_adversary/."""
from __future__ import annotations

import json
import sys
from collections import Counter, OrderedDict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))

SEARCHES = OrderedDict([('G', 'GPT adversary'), ('R', 'uniform random'), ('S', 'sparse random (1-3 groups)')])


def load(tag):
    p = ROOT / 'logs' / f'search_{tag}.json'
    return json.loads(p.read_text()) if p.exists() else None


def ledger():
    rows = []
    p = ROOT / 'logs' / 'codex_calls.jsonl'
    if p.exists():
        for ln in p.read_text().splitlines():
            try:
                rows.append(json.loads(ln))
            except json.JSONDecodeError:
                pass
    return rows


def main():
    tri = json.loads((ROOT / 'triage_modes.json').read_text()) if (ROOT / 'triage_modes.json').exists() else dict(modes=[])
    modes = tri['modes']
    mode_of = {}
    for m in modes + tri.get('known', []):
        for mem in m.get('members', []):
            mode_of[mem] = m['id']
    led = ledger()
    out = dict(searches={}, modes=modes, notes=tri.get('notes', []))
    curves = {}
    rows = []
    for tag, name in SEARCHES.items():
        st = load(tag)
        if st is None:
            continue
        ents = st['entries']
        run = [e for e in ents if e.get('cls') and e['cls'].get('label') not in ('invalid', 'infra')]
        inval = [e for e in ents if e.get('cls', {}).get('label') == 'invalid']
        infra = [e for e in ents if e.get('cls', {}).get('label') == 'infra']
        fails = [e for e in run if e['cls'].get('failure')]
        labels = Counter(e['cls']['label'] for e in run)
        sigs = {e['cls']['signature'] for e in fails}
        mids = [mode_of.get(e['id']) for e in fails]
        found = []
        cur = []
        seen = set()
        for e in run:
            mid = mode_of.get(e['id']) if e['cls'].get('failure') else None
            if mid and mid not in seen:
                seen.add(mid)
                found.append((mid, e['id']))
            cur.append(len(seen))
        curves[tag] = cur
        mm = {m['id']: m for m in modes + tri.get('known', [])}
        new_modes = [m for m in seen if mm.get(m, {}).get('novelty', '').startswith('new')]
        weak = [m for m in seen if mm.get(m, {}).get('classification') == 'robot weakness']
        gpt_calls = [r for r in led if r.get('tag', '').startswith(f'adv_{tag}_')] if tag == 'G' else []
        perc = [r for r in led if r.get('purpose', '').startswith('perception') and r.get('tag', '').startswith(f'{tag}_')]
        sim_wall = sum(e.get('wall_s', 0) or 0 for e in run)
        rec = dict(name=name, proposed=len(ents), rejected_invalid=len(inval), infra_errors=len(infra), run=len(run),
                   failures=len(fails), failure_rate=round(len(fails) / max(1, len(run)), 3), labels=dict(labels),
                   distinct_signatures=len(sigs), distinct_modes=len(seen), modes_found=[f for f in found],
                   new_mechanism_modes=sorted(new_modes), robot_weakness_modes=sorted(weak),
                   unattributed_failures=[e['id'] for e in fails if e['id'] not in mode_of],
                   gpt_proposal_calls=len(gpt_calls), gpt_proposal_s=round(sum(r['seconds'] for r in gpt_calls), 1),
                   gpt_proposal_tokens=sum(r.get('tokens', 0) for r in gpt_calls),
                   perception_calls=len(perc), perception_s=round(sum(r['seconds'] for r in perc), 1),
                   perception_tokens=sum(r.get('tokens', 0) for r in perc), sim_wall_s=round(sim_wall, 1),
                   sample_tries_mean=(round(sum(e.get('sample_tries', 1) for e in ents) / max(1, len(ents)), 1) if tag != 'G' else None))
        out['searches'][tag] = rec
        rows.append((tag, rec))
        out.setdefault('entries', {})[tag] = [dict(id=e['id'], name=e['name'], round=e.get('round'), params=e['params'],
                                                   hypothesis=e.get('hypothesis'), label=(e.get('cls') or {}).get('label'),
                                                   severity=(e.get('cls') or {}).get('severity'),
                                                   signature=(e.get('cls') or {}).get('signature'), mode=mode_of.get(e['id']),
                                                   errors=e.get('errors') or None, result_file=e.get('result_file'))
                                              for e in ents]
    # minimisation + repro
    mins = {}
    for f in sorted((ROOT / 'logs' / 'minimize').glob('*.json')) if (ROOT / 'logs' / 'minimize').exists() else []:
        d = json.loads(f.read_text())
        mins[d['entry']] = dict(minimal=d['minimal'], n_runs=d['n_runs'], lam_fail=d['lam_fail'], lam_ok=d['lam_ok'],
                                reproduced_seed2=d['reproduced_seed2'])
    out['minimization'] = mins
    # memory
    peaks = []
    p = ROOT / 'logs' / 'peak_rss.tsv'
    if p.exists():
        for ln in p.read_text().splitlines():
            try:
                peaks.append(int(ln.split('peak_rss_kb=')[1]))
            except (IndexError, ValueError):
                pass
    out['memory'] = dict(sim_processes=len(peaks), peak_rss_mb_max=round(max(peaks, default=0) / 1024),
                         peak_rss_mb_median=round(sorted(peaks)[len(peaks) // 2] / 1024) if peaks else None, watchdog_kills=0)
    rl = ROOT / 'logs' / 'rate_limits.jsonl'
    out['rate_limit_backoffs'] = len(rl.read_text().splitlines()) if rl.exists() else 0
    out['codex_totals'] = dict(calls=len(led), seconds=round(sum(r['seconds'] for r in led), 1), tokens=sum(r.get('tokens', 0) for r in led),
                               failed=sum(1 for r in led if not r.get('ok')))
    (ROOT / 'results.json').write_text(json.dumps(out, indent=1, default=str))
    # ---- plot: cumulative distinct modes
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.6), dpi=130)
        cols = {'G': '#2a6fdb', 'R': '#d9822b', 'S': '#7a7a7a'}
        for tag, cur in curves.items():
            ax.step(range(1, len(cur) + 1), cur, where='post', label=SEARCHES[tag], color=cols[tag], lw=2)
        ax.set_xlabel('scenarios run (valid)')
        ax.set_ylabel('distinct failure modes found')
        ax.grid(alpha=.3)
        ax.legend(frameon=False)
        ax.spines[['top', 'right']].set_visible(False)
        fig.tight_layout()
        fig.savefig(ROOT / 'out' / 'discovery.png')
    except Exception as e:  # noqa: BLE001
        print('plot failed', e)
    # ---- tables
    t = ['| | ' + ' | '.join(r[1]['name'] for r in rows) + ' |', '|---|' + '---|' * len(rows)]
    for k, lab in [('proposed', 'scenarios proposed / sampled'), ('rejected_invalid', 'rejected by the validator'),
                   ('run', 'run (valid)'), ('failures', 'failures (severity >= 2)'), ('failure_rate', 'failure rate'),
                   ('distinct_signatures', 'distinct failure signatures'), ('distinct_modes', 'distinct failure modes (triaged)'),
                   ('new_mechanism_modes', 'new-mechanism modes'), ('robot_weakness_modes', 'modes = real robot weakness'),
                   ('gpt_proposal_calls', 'GPT proposal calls'), ('gpt_proposal_s', 'GPT proposal time s'),
                   ('gpt_proposal_tokens', 'GPT proposal tokens'), ('perception_calls', 'Codex perception calls in sims'),
                   ('sim_wall_s', 'sim wall time s')]:
        t.append(f'| {lab} | ' + ' | '.join(str(len(r[1][k]) if isinstance(r[1][k], list) else r[1][k]) for r in rows) + ' |')
    (ROOT / 'logs' / 'report_tables.md').write_text('\n'.join(t) + '\n')
    print('\n'.join(t))
    print(json.dumps(out['memory']), out['rate_limit_backoffs'], out['codex_totals'])


if __name__ == '__main__':
    main()
