#!/usr/bin/env python3
"""Collect runs/<problem>/ledger.jsonl into results.json and draw metric_vs_iteration.png (homebrew python3)."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

G = Path(__file__).resolve().parent.parent
TITLES = {'p1_aim': 'P1  insert aim (historical fix replay)', 'p2_lean': 'P2  plate lean (open problem)',
          'p3_ft': 'P3  wrist F/T false triggers (revert)', 'p2b_lean_noaim': 'P2b  lean, aim offset frozen',
          'p4_dock': 'P4  dock y -4 cm sensing', 'p1_aim_rep2': 'P1 replication 2', 'p1_aim_rep3': 'P1 replication 3'}
NCASES = {'p1_aim': 4, 'p2_lean': 5, 'p3_ft': 3, 'p2b_lean_noaim': 5, 'p4_dock': 4, 'p1_aim_rep2': 4, 'p1_aim_rep3': 4}
TARGET = {'p1_aim': 4, 'p2_lean': 4, 'p3_ft': 3, 'p2b_lean_noaim': 4, 'p4_dock': 4, 'p1_aim_rep2': 4, 'p1_aim_rep3': 4}


AIM = {'p1_aim_rep2': '8 mm', 'p1_aim_rep3': '9 mm'}


def load(p):
    f = G / 'runs' / p / 'ledger.jsonl'
    if not f.exists():
        return []
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()]


def main():
    out = dict(generated='harness/aggregate.py', problems={})
    probs = [p for p in TITLES if load(p)]
    for p in probs:
        rows = load(p)
        fin = G / 'runs' / p / 'final.json'
        out['problems'][p] = dict(title=TITLES[p], target=TARGET[p], n_cases=NCASES[p],
                                  final=json.loads(fin.read_text()) if fin.exists() else None,
                                  iterations=[{k: v for k, v in r.items() if k not in ('recs',)} for r in rows])
    (G / 'results.json').write_text(json.dumps(out, indent=1, default=str))

    INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'
    BLUE, ORANGE, RED = '#2a78d6', '#eb6834', '#e34948'
    lean_probs = [p for p in probs if p.startswith('p2')]
    reps = [p for p in probs if '_rep' in p]
    probs = [p for p in probs if p != 'p2b_lean_noaim' and '_rep' not in p]
    fig, axs = plt.subplots(1, len(probs) + 1, figsize=(4.4 * (len(probs) + 1), 3.9), squeeze=False)
    # extra panel: mean final lean per iteration for P2 and P2b (same unit, one axis)
    axl = axs[0][-1]
    for p, col, nm in zip(lean_probs, (BLUE, '#1baf7a'), ('P2 (aim free)', 'P2b (aim frozen)')):
        xs, ys = [], []
        for it in sorted({r['iter'] for r in load(p)}):
            f = G / 'runs' / p / f'it{it}' / ('summary.json' if it == 0 else 'eval')
            vals = []
            if it == 0 and f.exists():
                recs = json.loads(f.read_text())['recs']
                vals = [r['states']['final']['lean_deg'] for r in recs if r.get('states', {}).get('final')]
            elif f.exists():
                for j in sorted(f.glob('seed*.json')):
                    st = json.loads(j.read_text()).get('states', {}).get('final')
                    if st:
                        vals.append(st['lean_deg'])
            if vals:
                xs.append(it)
                ys.append(sum(vals) / len(vals))
        axl.plot(xs, ys, color=col, lw=2, marker='o', ms=7, mec='white', mew=1.5, label=nm, zorder=3)
        if xs:
            axl.text(xs[-1] + .15, ys[-1], nm, color=INK, fontsize=8, va='center')
    axl.annotate('it2: 22 deg off-scale\n(plate dropped one tine over)', (2, 11.8), xytext=(2.4, 10.6), fontsize=7, color=MUTED,
                 arrowprops=dict(arrowstyle='->', color=MUTED, lw=.8))
    axl.axhline(6, color=ORANGE, lw=1.5, ls='--')
    axl.text(0, 6.15, 'target < 6 deg', color=MUTED, fontsize=8)
    axl.set_ylim(0, 12)
    axl.set_xlim(-.5, 9.8)
    axl.set_xticks(range(0, 9))
    axl.set_xlabel('iteration (0 = baseline)', color=MUTED, fontsize=9)
    axl.set_ylabel('mean final lean over 5 seeds (deg)', color=MUTED, fontsize=9)
    axl.set_title('P2 / P2b  mean final plate lean', fontsize=10, color=INK, loc='left')
    axl.grid(axis='y', color=GRID, lw=.8)
    axl.legend(frameon=False, fontsize=8, loc='lower left')
    for sp in ('top', 'right'):
        axl.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'):
        axl.spines[sp].set_color(GRID)
    axl.tick_params(colors=MUTED, labelsize=8)
    for ax, p in zip(axs[0], probs):
        rows = load(p)
        xs, ys, xr, lab = [], [], [], []
        for r in rows:
            it = r['iter']
            if r.get('metric') is None:
                xr.append(it)
                lab.append((it, r.get('status', '')))
            else:
                xs.append(it)
                ys.append(r['metric'])
        ax.axhline(TARGET[p], color=ORANGE, lw=1.5, ls='--', zorder=1)
        ax.text(0, TARGET[p] + .08, f'target {TARGET[p]}/{NCASES[p]}', color=MUTED, fontsize=8, va='bottom')
        ax.plot(xs, ys, color=BLUE, lw=2, marker='o', ms=7, mec='white', mew=1.5, zorder=3, label='run 1 (aim 8 mm)' if p == 'p1_aim' else 'run 1')
        if p == 'p1_aim':
            for j, (rp, col) in enumerate(zip(reps, ('#1baf7a', '#9b59b6'))):
                rr = [r for r in load(rp) if r.get('metric') is not None]
                ax.plot([r['iter'] + .08 * (j + 1) for r in rr], [r['metric'] - .06 * (j + 1) for r in rr], color=col, lw=1.4,
                        ls=':', marker='s', ms=5, zorder=2, label=f'replication {rp[-1]} (aim {AIM.get(rp, "?")})')
            if reps:
                ax.legend(frameon=False, fontsize=7, loc='center right')
        for it, st in lab:
            ax.plot([it], [0], marker='x', color=RED, ms=8, mew=2, zorder=3)
            ax.text(it, .15, st.split('_')[0][:9], color=MUTED, fontsize=7, ha='center', rotation=90, va='bottom')
        acc = (out['problems'][p]['final'] or {}).get('accepted_iter')
        if acc:
            ax.annotate('accepted', (acc, [r['metric'] for r in rows if r['iter'] == acc][0]), xytext=(0, -18),
                        textcoords='offset points', ha='center', fontsize=8, color=INK)
        ax.set_ylim(-.3, NCASES[p] + .5)
        ax.set_xlim(-.5, max(8, max([r['iter'] for r in rows])) + .5)
        ax.set_xticks(range(0, 9))
        ax.set_yticks(range(0, NCASES[p] + 1))
        ax.set_xlabel('iteration (0 = baseline)', color=MUTED, fontsize=9)
        ax.set_ylabel(f'test cases passing (of {NCASES[p]})', color=MUTED, fontsize=9)
        ax.set_title(TITLES[p], fontsize=10, color=INK, loc='left')
        ax.grid(axis='y', color=GRID, lw=.8)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8)
    fig.text(.01, .01, 'x at 0 = iteration not run (patch failed to apply / rejected by the rule check / codex failed)',
             fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, .04, 1, 1))
    fig.savefig(G / 'metric_vs_iteration.png', dpi=150, facecolor='white')
    print('wrote results.json and metric_vs_iteration.png for', probs)


if __name__ == '__main__':
    main()
