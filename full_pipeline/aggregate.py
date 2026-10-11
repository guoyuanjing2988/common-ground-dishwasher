#!/usr/bin/env python3
"""Collect trial / single-run results, statics and reach tables into out/results.json and out/tables.md."""
import json
from pathlib import Path

import numpy as np

import pipeline as PL

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'


def load_trials(opt):
    f = OUT / f'{opt}_trials.json'
    if f.exists():
        return json.loads(f.read_text())['trials']
    p = OUT / f'{opt}_trials_partial.jsonl'
    return [json.loads(l) for l in p.read_text().splitlines()] if p.exists() else []


def summarise(rows):
    if not rows:
        return None
    n = len(rows)
    per, att, clean = {}, {}, {}
    for s in PL.STEPS:
        per[s] = sum(any(x['step'] == s and x['ok'] for x in r['steps']) for r in rows)
        att[s] = sum(any(x['step'] == s for x in r['steps']) for r in rows)
        clean[s] = sum(any(x['step'] == s and x.get('clean') for x in r['steps']) for r in rows)
    simt = {s: [x['sim_s'] for r in rows for x in r['steps'] if x['step'] == s and x['ok'] and 'sim_s' in x] for s in PL.STEPS}
    codex = [c for r in rows for c in r.get('codex_s', []) if c]
    succ = [r for r in rows if r['success']]
    fails = {}
    for r in rows:
        if not r['success']:
            fails.setdefault(r['failed'].split(':')[0], []).append(r['failed'][:220])
    perc = [x['metrics'].get('centre_err_mm') for r in rows for x in r['steps'] if x['step'] == 'perceive_plate' and x['metrics'].get('centre_err_mm') is not None]
    perc2 = [x['metrics'].get('perc2_centre_err_mm') for r in rows for x in r['steps'] if x['step'] == 'pick_plate' and x['metrics'].get('perc2_centre_err_mm') is not None]
    def rng(v):
        return [round(float(np.min(v)), 2), round(float(np.mean(v)), 2), round(float(np.max(v)), 2)] if v else None
    def met(step, key):
        return rng([x['metrics'][key] for r in rows for x in r['steps'] if x['step'] == step and x['ok'] and isinstance(x['metrics'].get(key), (int, float))])
    return dict(n=n, success=len(succ), clean_success=sum(r.get('clean_success', False) for r in rows),
                per_step_ok=per, per_step_attempted=att, per_step_clean=clean,
                step_sim_s_mean={s: round(float(np.mean(v)), 1) for s, v in simt.items() if v},
                cycle_sim_s=rng([r['sim_time_s'] for r in succ]), codex_call_s=rng(codex), codex_calls=len(codex),
                cycle_incl_codex_s=rng([r['sim_time_s'] + sum(c for c in r.get('codex_s', []) if c) for r in succ]),
                wall_s=rng([r['wall_s'] for r in rows]),
                perception1_err_mm=rng(perc), perception2_err_mm=rng(perc2),
                overhang_mm=met('pick_plate', 'overhang_true_mm'), push_press_mean_N=met('pick_plate', 'push_press_mean_N'),
                tilt_slip_mm=met('pick_plate', 'tilt_inhand_slip_mm'), hold_slip_mm=met('pick_plate', 'hold_slip_mm'),
                plane_vertical_deg=met('pick_plate', 'plane_from_vertical_deg'), rack_lift_slip_mm=met('pick_plate', 'lift_slip_mm'),
                slot_bottom_offset_mm=met('insert_plate', 'bottom_offset_mm'), slot_tilt_deg=met('insert_plate', 'plane_tilt_deg'),
                touchdown_peak_N=met('insert_plate', 'touchdown_support_peak_N'),
                door_open_deg=met('open_door', 'door_deg'), door_closed_deg=met('close_door', 'door_deg'),
                door_peak_finger_N=met('open_door', 'peak_finger_force_N'),
                pod_basket_clearance_mm=met('pull_basket', 'min_pod_basket_clearance_mm'),
                quick_touch_N=met('press_buttons', 'quick_touch_peak_N'), power_touch_N=met('press_buttons', 'power_touch_peak_N'),
                max_penetration_mm=rng([r['max_penetration_mm'] for r in rows]),
                warnings=sum(sum(r['warnings'].values()) for r in rows),
                unplanned_contacts=[c for r in rows for c in r['unexpected_robot_contacts']][:10],
                failure_modes=fails)


def main():
    res = dict(trials={}, singles={})
    for o in ('M1', 'M2', 'M3'):
        res['trials'][o] = summarise(load_trials(o))
    for o in ('M1', 'M2', 'M3', 'M3a', 'M3flat'):
        f = OUT / f'{o}_single.json'
        if f.exists():
            r = json.loads(f.read_text())
            res['singles'][o] = dict(success=r['success'], clean_success=r.get('clean_success'), failed=r['failed'],
                                     sim_time_s=r['sim_time_s'], codex_s=r['codex_s'], wall_s=r['wall_s'],
                                     steps=[dict(step=s['step'], ok=s['ok'], t=[s.get('t_start'), s.get('t_end')], reason=s.get('reason'),
                                                 unplanned_contact_max_N=s.get('unplanned_contact_max_N'))
                                            for s in r['steps']],
                                     max_penetration_mm=r['max_penetration_mm'], warnings=r['warnings'])
    for k in ('statics', 'mount_table'):
        f = OUT / f'{k}.json'
        if f.exists():
            res[k] = json.loads(f.read_text())
    (OUT / 'results.json').write_text(json.dumps(res, indent=1, default=float))
    lines = ['| step | ' + ' | '.join(o for o in res['trials'] if res['trials'][o]) + ' |',
             '|---|' + '---|' * sum(1 for o in res['trials'] if res['trials'][o])]
    for s in PL.STEPS:
        cells = []
        for o, t in res['trials'].items():
            if t:
                cells.append(f"{t['per_step_ok'][s]}/{t['n']}" + (f" (att. {t['per_step_attempted'][s]})" if t['per_step_attempted'][s] != t['n'] else ''))
        lines.append(f'| {s} | ' + ' | '.join(cells) + ' |')
    (OUT / 'tables.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    for o, t in res['trials'].items():
        if t:
            print(o, {k: v for k, v in t.items() if k not in ('per_step_ok', 'per_step_attempted', 'per_step_clean', 'unplanned_contacts')})


if __name__ == '__main__':
    main()
