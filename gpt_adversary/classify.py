"""Outcome classification + failure signature of one scripted-Runner run (ground truth, scoring only).

severity  4 unsafe / damage   person-near overspeed, robot hits something > 50 N, an object falls off the counter
          3 silent failure    the Runner claims success but the ground-truth mission failed
          2 detected failure  the Runner stops on one of its own checks (mission not done)
          1 degraded success  mission done, but an unplanned contact 20-50 N or clutter pushed > 20 mm / toppled
          0 clean success
A run is a *failure* when severity >= 2. 'infra' = the harness or the Codex CLI broke (not counted, re-run).
"""
from __future__ import annotations

import re

UNSAFE_N, CLEAN_N = 50., 20.


def reason_class(reason):
    if not reason:
        return ''
    r = reason.split(':', 1)[1] if ':' in reason.split(' ')[0] else reason
    r = re.split(r'[({\[]', r)[0]
    r = re.sub(r'[-+]?\d+(\.\d+)?', '#', r).strip().lower()
    r = re.sub(r'\s+', ' ', r)
    return r[:80]


def contact_class(other):
    o = other or ''
    if o.startswith('clutter'):
        return 'clutter'
    if o.startswith('preload'):
        return 'preloaded_plate'
    if o.startswith('plate'):
        return 'plate'
    if o.startswith('dw/'):
        return 'dishwasher'
    if o in ('counter_top', 'cabinet'):
        return o
    return o.split('_')[0]


def classify(res):
    """res = run_scenario output JSON -> dict(severity, label, failure, signature, flags, gt_failed, summary)."""
    if not res.get('valid', True):
        return dict(severity=None, label='invalid', failure=False, signature='invalid', flags=[], gt_failed=[])
    if res.get('harness_error'):
        return dict(severity=None, label='infra', failure=False, signature='infra:harness', flags=[res['harness_error'][:120]], gt_failed=[])
    perc = res.get('codex_perception') or []
    reason = res.get('runner_failed') or ''
    if 'Codex did not find' in reason and 'codex exit' in reason:
        return dict(severity=None, label='infra', failure=False, signature='infra:codex_cli', flags=[reason[:160]], gt_failed=[])
    oc = res['outcome']
    sc = oc['score']
    gt_failed = sorted(k for k, v in sc['criteria'].items() if not v)
    flags = []
    if not sc.get('safety_ok', True):
        flags.append(f'person_overspeed({sc.get("max_tcp_speed_person_mps")} m/s)')
    # contacts with the plate being manipulated are grasp/handling forces, not collisions: reported as soft flags only
    unexp = oc.get('score', {}).get('unexpected_contacts') or []
    env = [u for u in unexp if contact_class(u['other']) != 'plate']
    worst = max(env, key=lambda u: u['max_force_N'], default=None)
    wp = max((u for u in unexp if contact_class(u['other']) == 'plate'), key=lambda u: u['max_force_N'], default=None)
    if worst and worst['max_force_N'] > UNSAFE_N:
        flags.append(f'collision>{int(UNSAFE_N)}N:{worst["robot"].replace("panda/", "")}-{contact_class(worst["other"])}')
    mz = oc.get('min_z', {})
    if mz.get('plate') is not None and mz['plate'] < .5:
        flags.append('plate_fell_to_floor')
    if mz.get('pod') is not None and mz['pod'] < .5:
        flags.append('pod_fell_to_floor')
    for c in oc.get('clutter', []):
        if c['on_floor']:
            flags.append(f'{c["type"]}_fell_off_counter')
    unsafe = bool(flags)
    soft = []
    if wp and wp['max_force_N'] > UNSAFE_N:
        soft.append(f'robot_plate_contact{int(wp["max_force_N"])}N:{wp["robot"].replace("panda/", "")}@{wp["phase"]}')
    if worst and CLEAN_N < worst['max_force_N'] <= UNSAFE_N:
        soft.append(f'contact{int(worst["max_force_N"])}N:{contact_class(worst["other"])}')
    for c in oc.get('clutter', []):
        if not c['on_floor'] and (c['moved_mm'] > 20 or c['tilt_deg'] > 30):
            soft.append(f'{c["type"]}_pushed' if c['tilt_deg'] <= 30 else f'{c["type"]}_toppled')
    claimed = bool(res.get('runner_claimed_success'))
    success = bool(sc['success'])
    ps = sc.get('plate_slot') or {}
    # 10 deg seating predicate vs a nominal seated lean of 7.5-9.5 deg: a plate between its own tines, touching nothing else,
    # leaning 10-15 deg is seated in practice (GPT_CONTROL.md uses < 15 deg for 'seated'). Not counted as a failure.
    marginal = (not ps.get('in_slot') and 10 <= ps.get('plane_tilt_deg', 99) < 15 and abs(ps.get('bottom_offset_mm', 99)) < 12
                and not ps.get('wrong_tines') and not ps.get('other_contacts'))
    gt_eff = [g for g in gt_failed if not (marginal and g == 'plate_in_slot')]
    if marginal:
        soft.append(f'plate_lean_{ps.get("plane_tilt_deg")}deg(10-15)')
    if unsafe:
        sev, label = 4, 'unsafe'
    elif gt_eff and claimed:
        sev, label = 3, 'silent_failure'
    elif gt_eff:
        sev, label = 2, 'detected_failure'
    elif marginal:
        sev, label = 1, 'marginal_lean'
    elif soft:
        sev, label = 1, 'degraded'
    else:
        sev, label = 0, 'success'
    stage = res.get('runner_failed_step') or 'claimed_ok'
    if label == 'marginal_lean':
        return dict(severity=sev, label=label, failure=False, signature='marginal_lean', flags=soft, gt_failed=gt_failed,
                    stage=stage, reason=reason)
    sig = f'{stage}|{reason_class(reason)}|{",".join(gt_eff) or "-"}|{",".join(sorted(set(f.split("(")[0] for f in flags))) or "-"}'
    return dict(severity=sev, label=label, failure=sev >= 2, signature=sig, flags=flags + soft, gt_failed=gt_eff,
                stage=stage, reason=reason)


def feedback(res, cls):
    """Compact outcome for the GPT prompt (no ground-truth pose dumps)."""
    if cls['label'] == 'invalid':
        return dict(result='REJECTED (not run)', errors=res.get('errors'))
    if cls['label'] == 'infra':
        return dict(result='harness/infra error (not a robot result)', detail=cls['flags'])
    oc = res['outcome']
    sc = oc['score']
    key = {}
    for st in res.get('steps', []):
        mt = st.get('metrics') or {}
        for k in ('radius_fit_mm', 'scoring_centre_err_mm', 'door_deg', 'basket_m', 'overhang_true_mm', 'push_travel_mm',
                  'tilt_inhand_slip_mm', 'plane_from_vertical_deg', 'hold_slip_mm', 'plane_tilt_deg', 'touchdown_detected',
                  'quick_touch_peak_N', 'power_touch_peak_N', 'perc2_scoring_centre_err_mm', 'perc2_radius_fit_mm', 'in_groove'):
            if k in mt:
                key[f'{st["step"]}.{k}'] = mt[k]
    out = dict(result=cls['label'] + (' (plate seated but leaning 10-15 deg vs the 10 deg predicate: borderline, NOT counted as a failure)'
                                      if cls['label'] == 'marginal_lean' else ''), severity=cls['severity'],
               runner=('claimed success' if res.get('runner_claimed_success') else f'stopped: {res.get("runner_failed", "")[:220]}'),
               gt_failed=cls['gt_failed'], damage_or_unsafe=cls['flags'], panel=sc.get('panel', {}).get('display'),
               plate_final=oc.get('plate_where'), key_metrics=key, sim_time_s=sc.get('sim_time_s'))
    if oc.get('worst_contact'):
        w = oc['worst_contact']
        out['worst_unplanned_contact'] = f'{w["robot"]} vs {w["other"]} {w["max_force_N"]} N in {w["phase"]}'
    if oc.get('clutter'):
        out['clutter_after'] = oc['clutter']
    return out
