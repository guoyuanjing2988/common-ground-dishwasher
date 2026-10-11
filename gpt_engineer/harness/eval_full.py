#!/usr/bin/env python3
"""Harness-owned evaluator for the STATIC M1 pipeline sandbox (problem P2, plate lean).

  eval_full.py --sandbox <copy> --case seed<k> --out <json> --frames <dir>

One full 9-step mission (M1 docked cart, Panda), plate pose randomised +-10 mm / +-10 deg from seed k, truth perception
(true pose + 1 mm noise, as in run.py --debug-truth-perception; perception is not the subject here). Scoring with the
frozen harness/score.py after insert, push, close and at the end of the mission.
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import score as SC  # noqa: E402


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
    import pipeline as PL
    import run as RUN
    (sb / 'out').mkdir(exist_ok=True)

    rec = dict(case=a.case, sandbox=str(sb), states={}, step_exc={}, frames=[])
    frames = Path(a.frames) if a.frames else None
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    holder = {}

    def wrap(meth_name, key):
        orig = getattr(PL.Runner, meth_name)

        def w(self, *args_, **kw):
            holder['r'] = self
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
        setattr(PL.Runner, meth_name, w)

    wrap('step_insert', 'after_insert')
    wrap('step_push_basket', 'after_push')
    wrap('step_close_door', 'after_close')
    k = int(a.case.replace('seed', ''))
    rng = np.random.default_rng(20261010 + 1000 * k)
    off = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)])
    rec['plate_off'] = [float(v) for v in off]
    t0 = time.time()
    try:
        res, _W = RUN.run_one(RUN.OPTIONS['M1'], off, None, 20261010 + k, False, run_tag=f'eval_{a.case}')
        rec['success'] = bool(res['success'])
        rec['failed'] = res['failed']
        rec['task_steps'] = [dict(step=s['step'], ok=s['ok'], reason=str(s.get('reason', ''))[:400],
                                  metrics={kk: v for kk, v in s.items() if kk not in ('step', 'ok', 'reason', 'contacts')})
                             for s in res['steps']]
        rec['sim_time_s'] = res['sim_time_s']
        rec['unexpected_robot_contacts'] = res.get('unexpected_robot_contacts', [])[:10]
        rec['max_penetration_mm'] = res.get('max_penetration_mm')
        r = holder.get('r')
        if r is not None:
            fs = SC.plate_state(r.model, r.data)
            fs.update(SC.door_basket(r.model, r.data))
            rec['states']['final'] = fs
    except Exception:  # noqa: BLE001
        rec['exception'] = traceback.format_exc()[-3000:]
    rec['wall_s'] = round(time.time() - t0, 1)
    ru = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    rec['peak_rss_mb'] = round(ru / (1024 * 1024 if sys.platform == 'darwin' else 1024), 1)
    Path(a.out).write_text(json.dumps(rec, indent=1, default=float))
    fin = rec['states'].get('final', {})
    print('EVAL', json.dumps(dict(case=a.case, success=rec.get('success'), final_lean=fin.get('lean_deg'),
                                  after_insert_lean=rec['states'].get('after_insert', {}).get('lean_deg'),
                                  wall_s=rec['wall_s'], peak_rss_mb=rec['peak_rss_mb'])), flush=True)


if __name__ == '__main__':
    main()
