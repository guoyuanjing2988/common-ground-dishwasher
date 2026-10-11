#!/usr/bin/env python3
"""Harness-owned evaluator for the MOBILE pipeline sandbox copies (problems P1 aim, P3 F/T).

  eval_mobile.py --sandbox <copy> --case <name> --out <json> --frames <dir>

Runs ONE case in this process (the harness calls it sequentially under the 3 GB RSS watchdog).
Scoring uses harness/score.py (frozen), hooked around the planner's step methods; the sandbox's own
predicates are reported too but never trusted for the metric.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import resource
import sys
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import score as SC  # noqa: E402

# case definitions are frozen here (not in the sandbox)
POSE = [(.002, 0, 0), (-.002, 0, 0), (.005, 0, 0), (-.005, 0, 0), (0, .002, 0), (0, -.002, 0), (0, .005, 0),
        (0, -.005, 0), (0, 0, .2), (0, 0, -.2), (0, 0, .5), (0, 0, -.5)]
DOCK = [(0, 0, 0), (0, .02, 0), (0, -.02, 0), (0, .04, 0), (0, -.04, 0), (.02, 0, 0), (.04, 0, 0),
        (0, 0, 3.), (0, 0, -3.), (0, 0, 6.), (0, 0, -6.)]


def parse_case(name):
    """pose<i> | dock<i> | mission<seed> (full charger->dock->task->charger mission, random plate/start)"""
    if name.startswith('pose'):
        return 'pose', int(name[4:])
    if name.startswith('dock'):
        return 'dock', int(name[4:])
    if name.startswith('mission'):
        return 'mission', int(name[7:])
    raise ValueError(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sandbox', required=True)
    ap.add_argument('--case', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', default='')
    a = ap.parse_args()
    sb = Path(a.sandbox).resolve()
    a.out = str(Path(a.out).resolve())
    a.frames = str(Path(a.frames).resolve()) if a.frames else ''
    os.chdir(sb)
    sys.path.insert(0, str(sb))
    import numpy as np
    import planner as PLN
    import run_mobile as RM
    import scene_mobile as SM
    RM.OUT.mkdir(exist_ok=True)

    rec = dict(case=a.case, sandbox=str(sb), states={}, step_exc={}, frames=[])
    frames = Path(a.frames) if a.frames else None
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    holder = {}

    def wrap(meth_name, key):
        orig = getattr(PLN.MotionPlanner, meth_name)

        def w(self, *args_, **kw):
            holder['pl'] = self
            try:
                return orig(self, *args_, **kw)
            finally:
                try:
                    st = SC.plate_state(self.model, self.data)
                    st.update(SC.door_basket(self.model, self.data))
                    rec['states'][key] = st
                    if frames and key in ('after_insert', 'after_close'):
                        lab = f'{a.case} {key}: lean {st["lean_deg"]} deg, bottom offset {st["bottom_offset_mm"]} mm, ' \
                              f'in_slot {st["in_slot"]}, wrong tines {st["wrong_tines"]}'
                        rec['frames'].append(SC.snapshot(self.model, self.data, frames / f'{a.case}_{key}.png', lab))
                except Exception as e:  # noqa: BLE001
                    rec['step_exc'][key] = repr(e)
        setattr(PLN.MotionPlanner, meth_name, w)

    wrap('step_insert', 'after_insert')
    wrap('step_push_basket', 'after_push')
    wrap('step_close_door', 'after_close')

    kind, i = parse_case(a.case)
    args = SimpleNamespace(debug_truth_perception=True, tcp_cap=None, t_max=1500., insert_aim=None, noslip=int(os.environ.get('GE_NOSLIP', '3')), soc_min=.30, base_pose='lidar')
    seed0 = 20261011
    t0 = time.time()
    try:
        if kind in ('pose', 'dock'):
            off = (POSE if kind == 'pose' else DOCK)[i]
            st, extra = RM.clear_start(*off)
            bb = None
            if kind == 'pose':
                x0, y0, _ = SM.dock_base_pose()
                bb = (x0, y0, 0.)
            res = RM.run_one(args, f'eval_{a.case}', start=st, plan_kind='docked', offset=off, vblocks=False,
                             believed_base=bb, codex=False, seed=seed0 + i, base_pose_source='lidar')
        else:
            rng = np.random.default_rng(seed0 + 1000 * i)
            po = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)])
            so = rng.uniform([-.05, -.05, -5.], [.05, .05, 5.])
            st = (SM.START_NOMINAL[0] + so[0], SM.START_NOMINAL[1] + so[1], SM.START_NOMINAL[2] + so[2])
            res = RM.run_one(args, f'eval_{a.case}', plate_off=po, start=st, seed=seed0 + i, codex=False, base_pose_source='lidar')
            rec['plate_off'] = [float(v) for v in po]
        rec['outcome'] = res['outcome']
        rec['all_9_steps_ok'] = res['all_9_steps_ok']
        rec['panel_final'] = res.get('panel_final')
        rec['jobs'] = [dict(state=j['state'], ok=j['ok'], reason=str(j.get('reason', ''))[:400]) for j in res['jobs']]
        rec['task_steps'] = [dict(step=s['step'], ok=s['ok'], reason=str(s.get('reason', ''))[:400],
                                  metrics={k: v for k, v in s.get('metrics', {}).items() if k not in ('contacts',)})
                             for s in res['task_steps']]
        rec['sim_time_s'] = res['sim_time_s']
        presses = [j for j in res['jobs'] if j.get('job') == 'press']
        rec['press_attempts'] = len(presses)
        rec['press_detail'] = [dict(state=j['state'], ok=j['ok'], panel_after=j.get('panel_after'),
                                    f_axis_at_stop=j.get('metrics', {}).get('f_axis_at_stop'),
                                    conductive_peak_N=j.get('metrics', {}).get('conductive_peak_N')) for j in presses]
        rec['panel_events'] = res.get('panel_events', [])[:20]
        rec['unexpected_robot_contacts'] = res.get('unexpected_robot_contacts', [])[:10]
        rec['max_penetration_mm'] = res.get('max_penetration_mm')
        pl = holder.get('pl')
        if pl is not None:
            fs = SC.plate_state(pl.model, pl.data)
            fs.update(SC.door_basket(pl.model, pl.data))
            rec['states']['final'] = fs
    except Exception:  # noqa: BLE001
        rec['exception'] = traceback.format_exc()[-3000:]
    rec['wall_s'] = round(time.time() - t0, 1)
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    rec['peak_rss_mb'] = round(r / (1024 * 1024 if sys.platform == 'darwin' else 1024), 1)
    Path(a.out).write_text(json.dumps(rec, indent=1, default=float))
    print('EVAL', json.dumps({k: rec.get(k) for k in ('case', 'outcome', 'all_9_steps_ok', 'wall_s', 'peak_rss_mb')}), flush=True)


if __name__ == '__main__':
    main()
