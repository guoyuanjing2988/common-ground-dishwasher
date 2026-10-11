#!/usr/bin/env python3
"""ONE simulation process: run one DSL scenario against the scripted Runner (default) or the GPT-L1 commander.

  python run_scenario.py --scenario scenarios/x.json --out runs/x.json [--seed 1] [--video out/clips/x.mp4 --caption ".." --record-from 40]
  python run_scenario.py --scenario scenarios/x.json --out runs/x_L1.json --commander L1
Always launched through runw.sh (RSS watchdog). Exit code 0 even when the robot fails (the failure is the data).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))

import dsl  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--commander', default='scripted', choices=['scripted', 'L1'])
    ap.add_argument('--video', default='')
    ap.add_argument('--caption', default='')
    ap.add_argument('--record-from', type=float, default=0.)
    ap.add_argument('--speed', type=float, default=4.)
    ap.add_argument('--record-until', type=float, default=1e9)
    ap.add_argument('--keyframes', default='')
    a = ap.parse_args()
    raw = json.loads(Path(a.scenario).read_text())
    spec = raw.get('params', raw)
    ok, errs, s = dsl.validate(spec)
    out = dict(scenario_file=a.scenario, seed=a.seed, commander=a.commander, valid=ok, errors=errs)
    if not ok:
        Path(a.out).write_text(json.dumps(out, indent=1))
        print('INVALID', errs)
        return
    out['diff'] = dsl.diff_from_nominal(s)
    tag = Path(a.out).stem
    t0 = time.monotonic()
    import adv_robot as AR
    try:
        if a.commander == 'scripted':
            kf = None
            if a.keyframes:
                kf = Path(a.keyframes)
                kf.mkdir(parents=True, exist_ok=True)
            res = AR.run_scripted(s, a.seed, tag, video=Path(a.video) if a.video else None, caption=a.caption,
                                  record_from=a.record_from, base_speed=a.speed, keyframes=kf, record_until=a.record_until,
                                  log=lambda x: print(x, flush=True))
        else:
            res = run_l1(AR, s, a.seed, tag)
        out.update(res)
    except Exception as e:  # noqa: BLE001  (harness crash: recorded, not a robot failure)
        out.update(harness_error=f'{e!r}', traceback=traceback.format_exc()[-2000:])
        print('HARNESS ERROR', traceback.format_exc())
    out['wall_total_s'] = round(time.monotonic() - t0, 1)
    Path(a.out).write_text(json.dumps(out, indent=1, default=str))
    sc = out.get('outcome', {}).get('score', {})
    print('RESULT', tag, 'claimed', out.get('runner_claimed_success'), 'failed', out.get('runner_failed'), '| GT', sc.get('success'),
          sc.get('criteria'), 'safety', sc.get('safety_ok'), '| wall', out['wall_total_s'], flush=True)


def run_l1(AR, s, seed, tag):
    """The GPT-L1 commander of ../gpt_control (gpt_agent.run, unchanged) on this scenario's robot. Its logs go to
    gpt_adversary/logs/gpt_runs and out/gpt_runs (gpt_agent.ROOT redirected)."""
    import gpt_agent as GA
    import mission as MS
    GA.ROOT = ROOT

    holder = {}

    def make_robot(name, seed_, video=None, run_tag='', title='', log=print):
        r = AR.AdvRobot(s, seed=seed_, run_tag=run_tag, log=log)
        r.start()
        holder['r'] = r
        return r, ({'person': s['person']} if s['person'] else {}), (0, 0, 0)
    MS.make_robot = make_robot
    out = GA.run('L1', 'adversarial', seed, 'zh', None, f'{tag}', None, 30)
    oc = holder['r'].outcome()
    return dict(gpt_l1=dict(stop_reason=out['stop_reason'], gpt_claim=out['gpt_claim'], turns=out['turns'], gpt=out['gpt'],
                            perception=out['perception'], api_stats=out['api_stats'], actions=out['actions'], recoveries=out['recoveries']),
                runner_claimed_success=bool(out['gpt_claim'] and out['gpt_claim'].get('success')),
                runner_failed=None if (out['gpt_claim'] and out['gpt_claim'].get('success')) else f'GPT: {out["stop_reason"]} '
                + str((out['gpt_claim'] or {}).get('summary', ''))[:300],
                runner_failed_step=None, outcome=oc, wall_s=out['wall_s'])


if __name__ == '__main__':
    main()
