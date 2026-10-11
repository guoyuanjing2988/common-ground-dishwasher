#!/usr/bin/env python3
"""Scripted (no GPT) drivers:
  --mode api      : a fixed L1 call list through the Robot API (sanity check that skills + recoveries work physically)
  --mode hardcoded: the original full_pipeline Runner.run() sequence on the same scene + faults (the baseline GPT is
                    compared against). Its own per-step predicates are kept; the mission score is the same GT score.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mission as MS
import robot_api as RA

ROOT = Path(__file__).resolve().parent

NOMINAL = [('skills.perceive_plate', {}), ('skills.open_door', {}), ('skills.load_pod', {}), ('skills.pull_basket', {}),
           ('skills.pick_plate', {}), ('skills.insert_plate', {}), ('skills.push_basket', {}), ('skills.close_door', {}),
           ('skills.press_button', {'button': 'quick'}), ('skills.press_button', {'button': 'power'})]


def run_api(scen, seed, calls, tag, video=None):
    r, faults, off = MS.make_robot(scen, seed, video=video, run_tag=tag)
    api = RA.RobotAPI(r, 'L2' if any(c[0].startswith('arm.') and c[0] not in ('arm.stow',) for c in calls) else 'L1')
    log = []
    for name, a in calls:
        if name == 'STOP':
            break
        if name == 'DBG_LEAN':                   # scoring-only probe of the seated plate (ground truth)
            print('   LEAN', r.plate_slot_state(), flush=True)
            continue
        if name == 'POD_FROM_LOCATE':            # feed the last pod perception into load_pod (what GPT would do)
            m = log[-1]['res']['measurements']
            name, a = 'skills.load_pod', dict(pick_xy=m['xyz'][:2], yaw_deg=m['yaw_deg'])
        res, sc = api.call(name, a)
        st = res.get('state', {})
        print(f'{name} {a} -> ok={res["ok"]} {res["code"]} {res.get("message","")[:160]} | {json.dumps(res.get("measurements",{}))[:300]} '
              f'| door {st.get("dishwasher_simulated_sensors",{}).get("door_deg")} basket {st.get("dishwasher_simulated_sensors",{}).get("basket_out_m")} '
              f'panel {st.get("dishwasher_simulated_sensors",{}).get("panel")} sim {res["sim_time_s"]} | scoring {sc}', flush=True)
        log.append(dict(name=name, args=a, res={k: v for k, v in res.items() if k != 'state'}, scoring=sc))
    score = r.mission_score()
    r.close()
    print('MISSION', json.dumps({k: score[k] for k in ('success', 'criteria', 'safety_ok', 'max_unplanned_contact_N', 'sim_time_s', 'fault_events')}, default=str))
    return dict(calls=log, score=score, stats=api.stats)


def run_hardcoded(scen, seed, tag, video=None):
    r, faults, off = MS.make_robot(scen, seed, video=video, run_tag=tag)
    t0 = time.monotonic()
    import pipeline as PL
    failed = None

    def perceive_no_gt():
        """Baseline perceive without the pipeline's ground-truth centre-error gate (a real runner has no GT)."""
        out = r.step_perceive()
        if not out.get('_ok') and .095 < r.est['radius_fit'] < .110:
            out.update(_ok=True, _reason='')
        return out
    fns = dict(perceive_plate=perceive_no_gt, open_door=r.step_open_door, load_pod=r.step_load_pod, pull_basket=r.step_pull_basket,
               pick_plate=r.step_pick_plate, insert_plate=r.step_insert, push_basket=r.step_push_basket, close_door=r.step_close_door,
               press_buttons=r.step_buttons)
    for s in PL.STEPS:
        try:
            r.run_step(s, fns[s])
        except PL.StepFail as e:
            failed = str(e)
            break
        except Exception as e:  # noqa: BLE001
            failed = f'{s}: exception {e!r}'
            break
    score = r.mission_score()
    steps = [dict(step=s['step'], ok=s['ok'], reason=s.get('reason', '')) for s in r.steps]
    r.close()
    out = dict(scenario=scen, seed=seed, runner_claimed_success=failed is None, runner_failed=failed, steps=steps, score=score,
               wall_s=round(time.monotonic() - t0, 1), codex_perception_s=r.codex_s)
    print('HARDCODED', scen, 'claimed', failed is None, '| mission success', score['success'], score['criteria'], 'safety', score['safety_ok'],
          'max v person', score['max_tcp_speed_person_mps'], '| failed:', failed, flush=True)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='api', choices=['api', 'hardcoded'])
    ap.add_argument('--scen', default='none')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--calls', default='', help='JSON list of [name, args] (api mode); default = nominal sequence')
    ap.add_argument('--video', default='')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    tag = f'{a.mode}_{a.scen}_s{a.seed}'
    v = Path(a.video) if a.video else None
    if a.mode == 'api':
        calls = json.loads(Path(a.calls).read_text()) if a.calls else NOMINAL
        res = run_api(a.scen, a.seed, calls, tag, v)
    else:
        res = run_hardcoded(a.scen, a.seed, tag, v)
    outp = Path(a.out) if a.out else ROOT / 'out' / f'{tag}.json'
    outp.write_text(json.dumps(res, indent=1, default=str))
