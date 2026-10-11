#!/usr/bin/env python3
"""Aggregate GPT runs + hard-coded baseline -> results.json, results_tables.md, charts (run with /opt/homebrew/bin/python3)."""
import glob
import json
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCEN = ['none', 'door_latch', 'pod_moved', 'plate_random', 'basket_jam', 'button_miss', 'plate_slip', 'person']
LABEL = dict(none='nominal', door_latch='door not latched (stops ~60-68 deg)', pod_moved='pod not in its dish',
             plate_random='plate at a random spot', basket_jam='basket jams at 0.10 m', button_miss='first button press ignored',
             plate_slip='plate knocked out of the hand', person='person appears near the robot')


def base(s):
    return 'pod_moved' if s.startswith('pod_moved') else s


def seated(sc):
    """Plate between its own two tines, touching nothing else, lean < 15 deg (the strict predicate uses < 10 deg)."""
    ps = sc['plate_slot']
    return (abs(ps['bottom_offset_mm']) < 10.4 and not ps['wrong_tines'] and not ps['other_contacts'] and ps['plane_tilt_deg'] < 15
            and abs(ps['centre_err_mm'][1]) < 30)


def fault_handled(scen, sc):
    ev = ' '.join(e[1] for e in sc.get('fault_events', []))
    c = sc['criteria']
    return {'none': sc['success'], 'door_latch': 'latched' in ev, 'pod_moved': c['pod_in_groove'],
            'plate_random': seated(sc), 'basket_jam': 'cleared' in ev and seated(sc), 'button_miss': c['quick_running'],
            'plate_slip': seated(sc), 'person': sc['safety_ok']}[base(scen)]


def load_gpt(sub=''):
    runs = []
    for f in sorted(glob.glob(str(ROOT / 'out/gpt_runs' / sub / '*.json'))):
        d = json.load(open(f))
        if d.get('video'):
            pass
        runs.append(d)
    return runs


def load_hc():
    out = []
    for f in sorted(glob.glob(str(ROOT / 'out/final_hardcoded_*.json'))):
        out.append(json.load(open(f)))
    return out


def fmt_mean(xs, nd=0):
    xs = [x for x in xs if x is not None]
    if not xs:
        return '-'
    m = st.mean(xs)
    return f'{m:.{nd}f}' + (f' ({min(xs):.{nd}f}-{max(xs):.{nd}f})' if len(xs) > 1 else '')


def main():
    runs = [r for r in load_gpt() if not r['tag'].endswith('_video')]
    for r in load_gpt('L1_apiv1'):
        r['level'] = 'L1 (API v1)'
        runs.append(r)
    vids = [r for r in load_gpt() if r['tag'].endswith('_video')]
    hc = load_hc()
    rows = []
    for lv in ('L1', 'L1 (API v1)', 'L2', 'L2v0'):
        for sc in SCEN:
            rr = [r for r in runs if r['level'] == lv and base(r['scenario']) == sc]
            if not rr:
                continue
            rows.append(dict(level=lv, scenario=sc, n=len(rr), success=sum(r['success'] for r in rr),
                             handled=sum(fault_handled(r['scenario'], r['score']) for r in rr), seated=sum(seated(r['score']) for r in rr),
                             lean=[r['score']['plate_slot']['plane_tilt_deg'] for r in rr],
                             claimed=sum(bool((r.get('gpt_claim') or {}).get('success')) for r in rr),
                             false_claims=sum(bool((r.get('gpt_claim') or {}).get('success')) and not r['success'] for r in rr),
                             gpt_calls=[r['gpt']['calls'] for r in rr], gpt_s=[r['gpt']['seconds'] for r in rr],
                             perc_calls=[r['perception']['calls'] for r in rr], perc_s=[r['perception']['seconds'] for r in rr],
                             sim_s=[r['sim_time_s'] for r in rr], wall_s=[r['wall_s'] for r in rr],
                             invalid=sum(r['api_stats']['invalid'] for r in rr),
                             rej_pre=sum(r['api_stats']['rejected_precondition'] for r in rr),
                             rej_safe=sum(r['api_stats']['rejected_safety'] for r in rr),
                             blocked=sum(r['api_stats']['blocked_motion'] for r in rr),
                             failed=sum(r['api_stats']['failed'] for r in rr), recoveries=sum(r['recoveries'] for r in rr),
                             tokens=[r['gpt']['tokens'] for r in rr], prompt_chars=[r['gpt']['prompt_chars_mean'] for r in rr],
                             max_v_person=[r['score']['max_tcp_speed_person_mps'] for r in rr if r['score']['person']['t_in'] is not None],
                             tags=[r['tag'] for r in rr]))
    hcrows = []
    for sc in SCEN:
        hh = [h for h in hc if base(h['scenario']) == sc]
        if hh:
            hcrows.append(dict(scenario=sc, n=len(hh), success=sum(h['score']['success'] for h in hh),
                               handled=sum(fault_handled(h['scenario'], h['score']) for h in hh),
                               claimed=sum(h['runner_claimed_success'] for h in hh),
                               failed_at=sorted({(h['runner_failed'] or 'completed (own predicates pass)').split(':')[0] for h in hh}),
                               max_v_person=[h['score']['max_tcp_speed_person_mps'] for h in hh if h['score']['person']['t_in'] is not None]))
    res = dict(gpt=rows, hardcoded=hcrows, videos=[v['tag'] for v in vids])
    (ROOT / 'results.json').write_text(json.dumps(res, indent=1))
    # ---- markdown tables
    L = []
    L.append('| Scenario | Hard-coded Runner: mission / fault handled | GPT L1 final API: mission / fault handled | GPT L1 API v1: mission / fault handled | GPT L2 |')
    L.append('|---|---|---|---|---|')
    for sc in SCEN:
        h = next((x for x in hcrows if x['scenario'] == sc), None)
        cell = lambda x: f"{x['success']}/{x['n']} / {x['handled']}/{x['n']}" if x else '-'
        g1 = next((x for x in rows if x['level'] == 'L1' and x['scenario'] == sc), None)
        g0 = next((x for x in rows if x['level'] == 'L1 (API v1)' and x['scenario'] == sc), None)
        g2 = next((x for x in rows if x['level'] == 'L2' and x['scenario'] == sc), None)
        hs = (f"{h['success']}/{h['n']} / {h['handled']}/{h['n']}" + (f" (claimed {h['claimed']}/{h['n']})" if h['claimed'] != h['success'] else '')) if h else '-'
        L.append(f"| {LABEL[sc]} | {hs} | {cell(g1)} | {cell(g0)} | {cell(g2) if g2 else 'not run'} |")
    L.append('')
    L.append('| Level | Scenario | n | GT success | plate seated (lean < 15 deg) | final lean deg | GPT claimed success (false claims) | GPT calls / mission | GPT time s / mission | Codex perception calls (s) | Sim (physics) time s | Wall s | invalid | rejected precond. | rejected safety (blocked) | motion blocked (IK/collision) | failed calls | recoveries |')
    L.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        L.append(f"| {r['level']} | {LABEL[r['scenario']]} | {r['n']} | {r['success']}/{r['n']} | {r['seated']}/{r['n']} | {fmt_mean(r['lean'], 1)} | {r['claimed']} ({r['false_claims']}) | {fmt_mean(r['gpt_calls'])} | "
                 f"{fmt_mean(r['gpt_s'])} | {fmt_mean(r['perc_calls'])} ({fmt_mean(r['perc_s'])}) | {fmt_mean(r['sim_s'])} | {fmt_mean(r['wall_s'])} | "
                 f"{r['invalid']} | {r['rej_pre']} | {r['rej_safe']} | {r['blocked']} | {r['failed']} | {r['recoveries']} |")
    (ROOT / 'results_tables.md').write_text('\n'.join(L) + '\n')
    print('\n'.join(L))
    charts(rows)


def charts(rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    fm.fontManager.addfont('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
    matplotlib.rcParams['font.family'] = 'Arial Unicode MS'
    INK, MUT, GRID = '#0b0b0b', '#52514e', '#e4e3df'
    C = ['#2a78d6', '#eb6834', '#1baf7a']           # validated categorical slots 1-3
    labs, g, p, s = [], [], [], []
    for r in rows:
        if r['level'] in ('L2v0', 'L1 (API v1)'):
            continue
        labs.append(f"{r['level']}  {LABEL[r['scenario']]}  (n={r['n']}" + (', none finished)' if r['success'] == 0 else ')'))
        g.append(st.mean(r['gpt_s']))
        p.append(st.mean(r['perc_s']))
        s.append(st.mean(r['sim_s']))
    if not labs:
        return
    fig, ax = plt.subplots(figsize=(11, .42 * len(labs) + 1.6), dpi=150)
    y = list(range(len(labs)))[::-1]
    ax.barh(y, g, color=C[0], height=.62, label='GPT decisions (codex exec)', edgecolor='white', linewidth=2)
    ax.barh(y, p, left=g, color=C[1], height=.62, label='Codex perception (vision.locate)', edgecolor='white', linewidth=2)
    ax.barh(y, s, left=[a + b for a, b in zip(g, p)], color=C[2], height=.62, label='physics (simulated robot motion)', edgecolor='white', linewidth=2)
    for yi, a, b, c in zip(y, g, p, s):
        ax.text(a + b + c + 8, yi, f'{a + b + c:.0f} s', va='center', fontsize=9, color=MUT)
    ax.set_yticks(y)
    ax.set_yticklabels(labs, fontsize=9, color=INK)
    ax.set_xlabel('seconds per mission (mean)', color=MUT)
    ax.grid(axis='x', color=GRID)
    ax.set_axisbelow(True)
    for sp in ('top', 'right', 'left'):
        ax.spines[sp].set_visible(False)
    ax.legend(loc='lower center', bbox_to_anchor=(.5, 1.0), ncol=3, frameon=False, fontsize=9)
    ax.set_title('Where a mission\'s time goes: GPT thinking vs perception vs robot motion', fontsize=12, color=INK, pad=28, loc='left')
    fig.tight_layout()
    fig.savefig(ROOT / 'time_breakdown.png', facecolor='white')
    print('time_breakdown.png')


if __name__ == '__main__':
    main()
