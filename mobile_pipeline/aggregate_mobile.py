#!/usr/bin/env python3
"""Aggregate the per-run JSON files in out/ into results_mobile.json and print the markdown tables used in
MOBILE_RESULTS.md. Pure post-processing (no simulation)."""
import glob
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'
ARM = ['TASK_PERCEIVE_PLATE', 'TASK_OPEN_DOOR', 'TASK_LOAD_POD', 'TASK_PULL_BASKET', 'TASK_PICK_PLATE', 'TASK_INSERT_PLATE',
       'TASK_PUSH_BASKET', 'TASK_CLOSE_DOOR', 'PANEL_POWER', 'PANEL_QUICK', 'PANEL_START']
TASK_STATES = ['UNSTOW_ARM'] + ARM + ['STOW_ARM']


def load(pat):
    return [json.loads(Path(p).read_text()) for p in sorted(glob.glob(str(OUT / pat)))]


def job(r, state, last=True):
    js = [j for j in r['jobs'] if j['state'] == state]
    return (js[-1] if last else js[0]) if js else None


def mm(v):
    return 'n/a' if v is None else f'{v:.1f}'


def mission_row(r):
    dk = job(r, 'DOCK') or {}
    m = dk.get('metrics', {}) or {}
    ph = r['base']['phase']
    task = [ph[s] for s in TASK_STATES if s in ph]
    creep = max([p['creep_mm_max'] for p in task] or [0])
    creep_deg = max([p['creep_deg_max'] for p in task] or [0])
    minw = min([min(p['min_load_N'][k] for k in ('wheel_l_tyre', 'wheel_r_tyre')) for p in task] or [0])
    minc = min([min(v for k, v in p['min_load_N'].items() if 'caster' in k) for p in task] or [0])
    cop = min([p['cop_margin_min_m'] for p in task] or [0])
    brk = max([p['brake_torque_max_Nm'] for p in task] or [0])
    dF = [p['dock_F_min_N'] for p in task], [p['dock_F_max_N'] for p in task]
    ch = job(r, 'DOCK_CHARGER') or job(r, 'ABORT_DOCK_CHARGER') or {}
    return dict(run=r['run'], outcome=r['outcome'], all_steps=r['all_9_steps_ok'], sim_s=r['sim_time_s'], wall_s=r['wall_s'],
                codex=[c['seconds'] for c in r['codex']], plate=r.get('plate_offset'), start=r.get('start_pose'),
                dock_err=m.get('dock_err_mm_deg'), est_err=m.get('base_est_err_mm_deg'), dock_F_peak=m.get('dock_force_peak_N'),
                dock_F_after=m.get('dock_force_after_brake_N'), creep_mm=creep, creep_deg=creep_deg, min_wheel_N=minw,
                min_caster_N=minc, cop_m=cop, brake_Nm=brk, dockF_task=[min(dF[0] or [0]), max(dF[1] or [0])],
                charger_err=(ch.get('metrics') or {}).get('charger_pose_err_mm_deg'), charge_F=(ch.get('metrics') or {}).get('charge_force_N'),
                soc_drop=r['battery']['soc_drop_pct'], E_Wh=r['battery']['E_terminal_Wh'], I_peak=r['battery']['I_peak_A'],
                P_peak=r['peak_power_W'], panel=r['panel_final'], rss=r['peak_rss_mb'], energy=r['energy_Wh'],
                by_state=r['energy_by_state_Wh'], t_state=r['time_by_state_s'], warnings=r['warnings'],
                pen=r['max_penetration_mm'], unplanned=[u for u in r['unexpected_robot_contacts'] if u['max_force_N'] > 20])


def phase_energy(r):
    g = dict(drive_out=['BOOT', 'LOCALIZE', 'CHECK_BATTERY', 'UNDOCK_CHARGER', 'NAV_PREDOCK'], dock=['DOCK'],
             task=TASK_STATES + ['ESTOP'], drive_back=['UNDOCK', 'NAV_CHARGER', 'DOCK_CHARGER', 'CHARGE_CHECK', 'DONE'])
    E, T = r['energy_by_state_Wh'], r['time_by_state_s']
    out = {k: (round(sum(E.get(s, 0) for s in v), 3), round(sum(T.get(s, 0) for s in v), 1)) for k, v in g.items()}
    out['codex_wait (inside task)'] = (round(E.get('_codex_wait', 0), 3), round(T.get('_codex_wait', 0), 1))
    return out


def step_table(runs):
    rows = {}
    for st in TASK_STATES:
        vals = [r['arm']['per_state'].get(st) for r in runs if st in r['arm']['per_state']]
        ok = sum(1 for r in runs if any(j['state'] == st and j['ok'] for j in r['jobs']) and
                 (not st.startswith('PANEL') or any(j['state'] == st and j.get('panel_after', {}).get('state') in ('STANDBY', 'SELECTED', 'RUNNING')
                                                    for j in r['jobs'])))
        if not vals:
            continue
        rows[st] = dict(ok=ok, n=len(runs), tcp_max=max(v['tcp_max'] for v in vals), elbow_max=max(v['elbow_max'] for v in vals),
                        q6_low_deg=min(math.degrees(v['q6_low_margin_min']) for v in vals),
                        q4_high_deg=min(math.degrees(v['q4_high_margin_min']) for v in vals),
                        margin_deg=min(math.degrees(v['margin_min']) for v in vals),
                        margin_joint=sorted({v['margin_joint'] for v in vals}),
                        tau=max((r['arm']['ctrl'].get('per_state', {}).get(st, [0, 0])[0] for r in runs), default=0))
        durs = [j['t1'] - j['t0'] for r in runs for j in r['jobs'] if j['state'] == st and j['ok']]
        rows[st]['dur_s'] = round(float(np.mean(durs)), 1) if durs else None
    return rows


def sweep_rows(prefix):
    out = []
    for r in load(f'{prefix}_*.json'):
        failed = [j for j in r['jobs'] if not j['ok']]
        panel_fail = [j['state'] for j in r['jobs'] if j['job'] == 'press' and j.get('panel_after', {}).get('state') not in
                      ('STANDBY', 'SELECTED', 'RUNNING')]
        lj = job(r, 'LOCALIZE_DOCKED') or {}
        steps = {s['step']: s['ok'] for s in r['task_steps']}
        out.append(dict(run=r['run'], offset=r.get('dock_offset'), outcome=r['outcome'], all_steps=r['all_9_steps_ok'],
                        steps_ok=sum(steps.values()), first_fail=(failed[0]['state'] + ': ' + failed[0]['reason'][:110]) if failed else '',
                        panel_retries=len(panel_fail), est_err=(lj.get('metrics') or {}).get('base_est_err_mm_deg'),
                        sim_s=r['sim_time_s'], rss=r['peak_rss_mb']))
    return out


def main():
    res = {}
    missions = load('mission_trial*.json') + load('mission_video.json')
    missions = [r for r in missions if not r.get('debug_truth_perception')]
    res['missions'] = [mission_row(r) for r in missions]
    res['mission_phase_energy'] = {r['run']: phase_energy(r) for r in missions}
    res['steps'] = step_table(missions)
    res['dock_sweep'] = sweep_rows('docksweep')
    res['pose_sweep'] = sweep_rows('posesweep')
    for nm in ('estop', 'nonconductive', 'low_battery', 'panel_door_open_DEBUG', 'mission_tcpcap025_DEBUG', 'mission_nocap_ref_DEBUG'):
        p = OUT / f'{nm}.json'
        if p.exists():
            r = json.loads(p.read_text())
            res[nm] = dict(row=mission_row(r) if r.get('jobs') else None, estop=r.get('estop'), low_battery=r.get('low_battery'),
                           panel_events=r.get('panel_events'), jobs=[dict(state=j['state'], ok=j['ok'], reason=j['reason'][:160],
                                                                        panel=j.get('panel_after'),
                                                                        m={k: v for k, v in (j.get('metrics') or {}).items()
                                                                           if k in ('conductive_peak_N', 'nonconductive_peak_N',
                                                                                    'nonconductive_dwell_s', 'ft_stop', 'tool')})
                                                                   for j in r['jobs']],
                           transitions=r['transitions'], steps=step_table([r]), sim_s=r['sim_time_s'], outcome=r['outcome'])
    if (OUT / 'brake_test.json').exists():
        res['brake_test'] = json.loads((OUT / 'brake_test.json').read_text())
    (ROOT / 'results_mobile.json').write_text(json.dumps(res, indent=1, default=float))
    # ---------------- print tables
    ms = res['missions']
    print('## missions')
    print('| run | outcome | 9 steps | sim s | Codex s | dock err x/y/yaw (mm, mm, deg) | base-est err | creep mm/deg | min wheel / caster N | CoP margin m | brake N m | SoC drop % | E Wh | I peak A | charger err mm | RSS MB |')
    for m in ms:
        de, ee = m['dock_err'] or [None] * 3, m['est_err'] or [None] * 3
        print(f"| {m['run']} | {m['outcome']} | {'yes' if m['all_steps'] else 'no'} | {m['sim_s']} | {', '.join(str(c) for c in m['codex'])} | "
              f"{de[0]}, {de[1]}, {de[2]} | {ee[0]}, {ee[1]}, {ee[2]} | {m['creep_mm']:.2f} / {m['creep_deg']:.3f} | "
              f"{m['min_wheel_N']:.0f} / {m['min_caster_N']:.0f} | {m['cop_m']:.3f} | {m['brake_Nm']:.1f} | {m['soc_drop']:.3f} | {m['E_Wh']:.2f} | "
              f"{m['I_peak']:.1f} | {m['charger_err']} | {m['rss']} |")
    print('\n## steps (FR3 limits)')
    print('| state | ok/n | mean s | peak TCP m/s | peak elbow m/s | min q6-q6min deg | min q4max-q4 deg | min margin deg (joint) | peak tau/limit |')
    for st, v in res['steps'].items():
        print(f"| {st} | {v['ok']}/{v['n']} | {v['dur_s']} | {v['tcp_max']:.2f} | {v['elbow_max']:.2f} | {v['q6_low_deg']:.1f} | "
              f"{v['q4_high_deg']:.1f} | {v['margin_deg']:.1f} (j{v['margin_joint']}) | {v['tau']:.2f} |")
    for k in ('dock_sweep', 'pose_sweep'):
        print(f'\n## {k}')
        print('| offset dx m, dy m, dyaw deg | outcome | 9 steps | steps ok | first failure | base-est err |')
        for r in res[k]:
            print(f"| {r['offset']} | {r['outcome']} | {'PASS' if r['all_steps'] else 'FAIL'} | {r['steps_ok']}/8 | {r['first_fail']} | {r['est_err']} |")
    print('\n## phase energy (Wh, s)')
    for k, v in res['mission_phase_energy'].items():
        print(k, v)
    for nm in ('estop', 'nonconductive', 'low_battery', 'panel_door_open_DEBUG', 'mission_tcpcap025_DEBUG', 'mission_nocap_ref_DEBUG'):
        if nm in res:
            print(f'\n## {nm}: outcome {res[nm]["outcome"]} sim {res[nm]["sim_s"]}')
            print(json.dumps({k: v for k, v in res[nm].items() if k in ('estop', 'low_battery', 'panel_events')}, default=float)[:2500])
            for j in res[nm]['jobs']:
                if j['state'].startswith('PANEL') or not j['ok']:
                    print('  ', j)
    if 'brake_test' in res:
        print('\n## brake test')
        for b in res['brake_test']:
            print(b)


if __name__ == '__main__':
    main()
