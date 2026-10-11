"""Aggregate out/runs/*/result.json -> results.json + results_tables.md (tables for GPT_SORT.md). No simulation here."""
from __future__ import annotations

import collections
import json
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parent
runs = []
for f in sorted(ROOT.glob('out/runs/s*_*/result.json')):
    R = json.loads(f.read_text())
    if R['tag'].startswith(('s', )) and 'score' in R and R['tag'] != 'demo':
        runs.append(R)
runs.sort(key=lambda R: (R['lang'], R['seed']))


def pct(a, b):
    return f'{a}/{b} ({100 * a / b:.0f} %)' if b else '-'


rows, agg = [], collections.defaultdict(lambda: collections.Counter())
fails = []
per_type = collections.defaultdict(lambda: collections.Counter())
plan_s = collections.defaultdict(list)
for R in runs:
    s, lang = R['score'], R['lang']
    A = agg[lang]
    A['runs'] += 1
    A['items'] += s['n_items']
    A['detected'] += s['detected']
    A['safe_ok'] += s['safe_correct']
    A['cat_ok'] += s['category_correct']
    A['n_safe'] += s['n_safe']
    A['soil_ok'] += s['soil_correct']
    A['greasy_ok'] += s['greasy_correct']
    A['prog_ok'] += int(s['program']['ok'])
    A['prog_ok_all'] += int(s['program']['chosen'] == s['program']['gt_on_all_safe_items'])
    import truth as TR
    own = TR.program_rule([dict(soil=g['soil'], greasy=g['greasy']) for g in R['plan']['items'] if g['slot'].startswith(('P', 'B'))])
    A['prog_self'] += int(own == s['program']['chosen'])
    s['program']['rule_on_gpt_own_labels'] = own
    A['feasible'] += int(s['plan_feasible'])
    A['robot_attempted'] += s['n_robot_loaded_claimed'] + len(s['failed'])
    A['robot_claimed'] += s['n_robot_loaded_claimed']
    A['seated15'] += s['n_seated_15']
    A['seated10'] += s['n_seated_10']
    A['human'] += len(s['handed_to_human'])
    A['gpt_calls'] += s['gpt_calls']
    A['gpt_s'] += s['gpt_seconds']
    A['running'] += int(s['program_running'])
    A['clean'] += int(s['max_unplanned_contact_N'] <= 20)
    plan_s[lang].append(s['gpt_plan_seconds'])
    A['backoffs'] += s['gpt_backoffs']
    for x in s['per_item']:
        per_type[x['type']]['n'] += 1
        per_type[x['type']]['safe_ok'] += int(x['safe_ok'])
        per_type[x['type']]['cat_ok'] += int(x['category_ok'])
    deferred = [x['name'] for x in s['per_item'] if x.get('slot') == 'defer']
    A['deferred'] += len(deferred)
    rows.append(dict(seed=R['seed'], lang=lang, n=s['n_items'], det=s['detected'], safe=s['safe_correct'], cat=s['category_correct'],
                     prog=s['program']['chosen'], gt=s['program']['gt_on_planned_load'], own=s['program']['rule_on_gpt_own_labels'], prog_ok=s['program']['ok'],
                     feas=s['plan_feasible'], issues=(s['plan_issues_structural'] or []) + (s['plan_issues_vs_truth'] or []),
                     loaded=s['n_robot_loaded_claimed'], seated15=s['n_seated_15'], seated10=s['n_seated_10'],
                     tilt=[x['plane_tilt_deg'] for x in s['robot_loaded']], human=len(s['handed_to_human']), deferred=len(deferred),
                     calls=s['gpt_calls'], gpt_s=s['gpt_seconds'], plan_s=s['gpt_plan_seconds'], sim_s=s['sim_s'], wall_s=s['wall_s'],
                     running=s['program_running'], max_contact=s['max_unplanned_contact_N'], moved=s['unintended_item_moves']))
    for x in s['per_item']:
        if not x['safe_ok']:
            fails.append(f"s{R['seed']} {lang}: {x['name']} ({x['type']}) safe={x['safe_true']} -> GPT {x.get('category')} safe={x.get('safe_pred')}")
    if not s['program']['ok']:
        bad = [f"{x['type']} soil {x['soil_true']}->{x.get('soil_pred')}" + (f", greasy {x['greasy_true']}->{x.get('greasy_pred')}" if x.get('greasy_pred') != x['greasy_true'] else '')
               for x in s['per_item'] if x['safe_true'] and x.get('slot', '').startswith(('P', 'B')) and (x.get('soil_ok') is False or x.get('greasy_ok') is False)]
        fails.append(f"s{R['seed']} {lang}: program {s['program']['chosen']} vs ground truth {s['program']['gt_on_planned_load']} "
                     f"(rule on GPT's own labels: {s['program']['rule_on_gpt_own_labels']}; label errors in the load: {'; '.join(bad) or 'none'})")
    for i_ in rows[-1]['issues']:
        fails.append(f"s{R['seed']} {lang}: plan issue: {i_}")
    for g, e in s['failed'].items():
        fails.append(f"s{R['seed']} {lang}: robot FAILED {g} at {e.get('stage')}: {str(e.get('reason'))[:120]}")
    for x in s['robot_loaded']:
        if not x['seated_15']:
            fails.append(f"s{R['seed']} {lang}: plate {x['body']} claimed loaded but not seated (lean {x['plane_tilt_deg']} deg, bottom offset {x['bottom_offset_mm']} mm, contacts {x['other_contacts']})")
    if s['max_unplanned_contact_N'] > 20:
        fails.append(f"s{R['seed']} {lang}: unplanned robot contact {s['max_unplanned_contact_N']} N: {s['unplanned_contacts'][:3]}")
    for nm, mv in s['unintended_item_moves'].items():
        fails.append(f"s{R['seed']} {lang}: {nm} ({mv['type']}) moved {mv['moved_mm']} mm unintentionally")
    for g, e in s['handed_to_human'].items():
        if 'bowl' not in str(e.get('reason')) and 'reach' not in str(e.get('reason')):
            fails.append(f"s{R['seed']} {lang}: {g} handed to human: {str(e.get('reason'))[:140]}")

tot = collections.Counter()
for A in agg.values():
    tot.update(A)
summary = {}
for lang, A in list(agg.items()) + [('all', tot)]:
    ps = plan_s[lang] if lang != 'all' else plan_s['en'] + plan_s['zh']
    summary[lang] = dict(runs=A['runs'], items=A['items'], detected=A['detected'], safe_correct=A['safe_ok'], category_correct=A['cat_ok'],
                         dishwasher_safe_items=A['n_safe'], soil_correct=A['soil_ok'], greasy_correct=A['greasy_ok'],
                         program_correct=A['prog_ok'], program_correct_vs_all_safe_rule=A['prog_ok_all'], program_consistent_with_gpt_own_soil_labels=A['prog_self'], plan_feasible=A['feasible'],
                         robot_plates_claimed=A['robot_claimed'], robot_attempts=A['robot_attempted'], seated_15deg=A['seated15'],
                         seated_10deg=A['seated10'], handed_to_human=A['human'], deferred=A['deferred'], program_running=A['running'],
                         runs_clean_20N=A['clean'], gpt_calls=A['gpt_calls'], gpt_seconds=round(A['gpt_s'], 1),
                         plan_seconds_median=round(st.median(ps), 1) if ps else None, plan_seconds_max=round(max(ps), 1) if ps else None,
                         rate_limit_backoffs=A['backoffs'])

peak = []
pf = ROOT / 'logs' / 'peak_rss.tsv'
if pf.exists():
    for ln in pf.read_text().splitlines():
        if 'eval_' in ln or 'demo' in ln:
            peak.append(int(ln.split('peak_rss_kb=')[1]))
res = dict(summary=summary, per_type={k: dict(v) for k, v in per_type.items()}, runs=rows, failures=fails,
           peak_rss_mb=dict(max=round(max(peak) / 1024) if peak else None, n_processes=len(peak)))
(ROOT / 'results.json').write_text(json.dumps(res, indent=1, ensure_ascii=False, default=str))

L = []
L.append('| goal | runs | items found | safe / not-safe correct | category correct | soil level correct (safe items) | greasy correct | program = rule | program = rule on GPT own soil labels | plan feasible | plates loaded by robot, seated (<15 deg) | GPT calls | GPT s (median plan call) |')
L.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for lang in ('en', 'zh', 'all'):
    if lang not in summary:
        continue
    S_ = summary[lang]
    L.append(f"| {lang} | {S_['runs']} | {pct(S_['detected'], S_['items'])} | {pct(S_['safe_correct'], S_['items'])} | {pct(S_['category_correct'], S_['items'])} | "
             f"{pct(S_['soil_correct'], S_['dishwasher_safe_items'])} | {pct(S_['greasy_correct'], S_['dishwasher_safe_items'])} | {pct(S_['program_correct'], S_['runs'])} | {pct(S_['program_consistent_with_gpt_own_soil_labels'], S_['runs'])} | "
             f"{pct(S_['plan_feasible'], S_['runs'])} | {S_['seated_15deg']}/{S_['robot_attempts']} attempted ({S_['seated_10deg']} also < 10 deg) | {S_['gpt_calls']} | {S_['gpt_seconds']} ({S_['plan_seconds_median']}) |")
L.append('')
L.append('| seed | goal | items | found | safe ok | category ok | program (GPT / rule) | plan feasible | robot loaded / seated | plate lean deg | to human | deferred | GPT calls | GPT s | sim s | wall s | max unplanned contact N |')
L.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for r_ in rows:
    L.append(f"| {r_['seed']} | {r_['lang']} | {r_['n']} | {r_['det']} | {r_['safe']} | {r_['cat']} | {r_['prog']} / {r_['gt']} {'✓' if r_['prog_ok'] else '✗'} | "
             f"{'yes' if r_['feas'] else 'NO'} | {r_['loaded']} / {r_['seated15']} | {', '.join(str(t) for t in r_['tilt']) or '-'} | {r_['human']} | {r_['deferred']} | "
             f"{r_['calls']} | {r_['gpt_s']} | {r_['sim_s']} | {r_['wall_s']} | {r_['max_contact']} |")
L.append('')
L.append('| true type | items | safe/not-safe correct | category correct |')
L.append('|---|---|---|---|')
for t, v in sorted(per_type.items()):
    L.append(f"| {t} | {v['n']} | {pct(v['safe_ok'], v['n'])} | {pct(v['cat_ok'], v['n'])} |")
L.append('')
L.append('Failures / deviations:')
for f_ in fails:
    L.append(f'- {f_}')
(ROOT / 'results_tables.md').write_text('\n'.join(L) + '\n')
print('\n'.join(L))
