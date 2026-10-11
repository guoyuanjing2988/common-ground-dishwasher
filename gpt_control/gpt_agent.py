#!/usr/bin/env python3
"""GPT orchestrator: GPT (via the codex CLI on the ChatGPT subscription) decides every next action.

Loop: prompt = goal + compact API docs + known model facts + compacted history + fresh state JSON (+ 1-2 images:
the live wall-camera frame and the latest perception overlay) -> codex exec with an output schema
{reasoning_brief, action (enum of the calls enabled at this level), args_json} -> RobotAPI.call -> append -> repeat,
until GPT calls done(success, summary) or a limit (turns, wall time) is hit. The simulation is paused while GPT thinks.
Every turn is logged to logs/gpt_runs/<tag>.jsonl.

  python gpt_agent.py --level L1 --scen none --seed 1 --goal zh
  python gpt_agent.py --level L2 --scen none --seed 1 --goal en --video out/L2_run.mp4
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import codex_client as CC
import mission as MS
import robot_api as RA

ROOT = Path(__file__).resolve().parent
LIMITS = {'L1': dict(turns=40, wall_min=50), 'L2': dict(turns=160, wall_min=170), 'L2v0': dict(turns=120, wall_min=120)}
LEVEL_TEXT = {
    'L1': 'L1 (skills): you sequence robust closed-loop skills (skills.*), perception (vision.*), appliance sensors and state. '
          'Skills run their own force/position control; you choose order, parameters and recovery.',
    'L2': 'L2 (primitives): there are NO task skills. You compose arm motions (arm.move_tcp, arm.move_until_contact, arm.grip/release, '
          'arm.follow_arc, arm.push_along), perception and sensors yourself. You must compute poses from the known model below.',
    'L2v0': 'L2 (primitives, minimal): there are NO task skills. You compose arm motions (arm.move_tcp, arm.move_until_contact, '
            'arm.grip/release), perception and sensors yourself. You must compute poses from the known model below.',
}
RULES = """You are the mission-level controller of a home robot: a Franka arm on a docked cart next to a kitchen counter with a countertop dishwasher.
Each turn you choose exactly ONE API call. The robot executes it while you wait, and you see the result next turn.
The API layer validates every call (argument schema, safety limits, preconditions, for arm motions also reach and collision) and may reject it with an error code; low-level control loops (1 kHz) run on the robot - you never command joints or torques.
Judge outcomes only from the measurements in results and state - do not assume a call worked. When something fails, diagnose from the measurements and suggested causes, then recover: retry with changed parameters, re-perceive, take an alternative, or stop safely and explain.
Safety: whenever state.safety.person_near_robot is true, no motion may run without a TCP speed cap <= 0.25 m/s (robot.set_speed_limit) - or wait (robot.wait) until the person has left.
When the goal is fully achieved - or cannot be achieved safely - call done(success, summary). Be honest in the summary (use the user's language).
Do not run shell commands or read files. Answer only with the JSON object."""


def decision_schema(level):
    names = sorted(n for n, t in RA.T.items() if level in t['levels'])
    return {"type": "object", "additionalProperties": False, "required": ["reasoning_brief", "action", "args_json"],
            "properties": {"reasoning_brief": {"type": "string", "description": "<= 40 words: why this call"},
                           "action": {"type": "string", "enum": names},
                           "args_json": {"type": "string", "description": "the call's arguments as a JSON object string, '{}' if none"}}}


def _short(o, n):
    s = json.dumps(o, separators=(',', ':'), default=str)
    return s if len(s) <= n else s[:n] + '...'


def compact_history(hist, full_last=6):
    lines = []
    for k, h in enumerate(hist):
        res = h['result']
        head = f"#{k + 1} {h['action']}({_short(h['args'], 160)}) -> {res.get('code')}"
        if k >= len(hist) - full_last:
            body = {kk: vv for kk, vv in res.items() if kk not in ('call_wall_s',)}
            lines.append(head + ' ' + _short(body, 900))
        else:
            meas = res.get('measurements') or {}
            lines.append(head + (f": {res.get('message', '')[:120]}" if res.get('message') else '') + (' ' + _short(meas, 140) if meas else ''))
    return '\n'.join(lines) if lines else '(none yet)'


def build_prompt(goal, level, hist, state, extra_note=''):
    hints = level == 'L2'
    return (f"{RULES}\n\nGOAL (from the user): {goal}\n\nCONTROL LEVEL: {LEVEL_TEXT[level]}\n\n"
            f"API (call name(args)):\n{RA.prompt_docs(level)}\n\nKNOWN MODEL AND FACTS:\n{RA.geometry_facts(level, hints=hints)}\n\n"
            f"HISTORY (oldest first, older turns compacted):\n{compact_history(hist)}\n\n"
            f"CURRENT STATE:\n{json.dumps(state, separators=(',', ':'))}\n\n"
            f"{extra_note}"
            "IMAGES: (1) live wall-camera frame now (looks down at the counter right of the dishwasher; the dishwasher door and panel are "
            "outside its view); (2) if attached, the latest perception overlay.\n"
            "Reply with: reasoning_brief, action, args_json.")


def run(level, scen, seed, goal_key, video=None, tag=None, max_turns=None, max_wall_min=None):
    goal = MS.GOALS.get(goal_key, goal_key)
    tag = tag or f'{level}_{scen}_s{seed}_{goal_key}'
    logdir = ROOT / 'logs' / 'gpt_runs'
    logdir.mkdir(parents=True, exist_ok=True)
    jl = logdir / f'{tag}.jsonl'
    jl.unlink(missing_ok=True)
    lim = dict(LIMITS[level])
    lim['turns'] = max_turns or lim['turns']
    lim['wall_min'] = max_wall_min or lim['wall_min']
    title = f'GPT controls the robot  [{level}]  scenario: {scen}'
    r, faults, off = MS.make_robot(scen, seed, video=video, run_tag=tag, title=title, log=lambda s: None)
    api = RA.RobotAPI(r, level)
    schema = decision_schema(level)
    hist, t0 = [], time.monotonic()
    gpt = dict(calls=0, seconds=0., tokens=0, failures=0, parse_errors=0, prompt_chars=[])
    stop_reason = 'max_turns'
    for turn in range(1, lim['turns'] + 1):
        if (time.monotonic() - t0) / 60 > lim['wall_min']:
            stop_reason = 'max_wall_time'
            break
        state = api.state()
        rgb, depth, png = r.capture(f'turn{turn:03d}')
        imgs = [str(png)] + api.pending_images[-1:]
        api.pending_images = []
        prompt = build_prompt(goal, level, hist, state)
        dec, meta = CC.codex_json(prompt, schema, f'{tag}_t{turn:03d}', images=imgs, purpose=f'decision:{level}')
        gpt['calls'] += meta['attempts']
        gpt['seconds'] += meta['seconds']
        gpt['tokens'] += meta.get('tokens', 0)
        gpt['prompt_chars'].append(len(prompt))
        rec = dict(turn=turn, prompt_chars=len(prompt), n_images=len(imgs), gpt_s=meta['seconds'], gpt_tokens=meta.get('tokens'),
                   gpt_attempts=meta['attempts'], sim_time_s=round(float(r.data.time), 2), wall_s=round(time.monotonic() - t0, 1))
        if dec is None:
            gpt['failures'] += 1
            rec.update(error=meta.get('error'))
            with open(jl, 'a') as fh:
                fh.write(json.dumps(rec, default=str) + '\n')
            continue
        action, why = dec['action'], dec.get('reasoning_brief', '')
        try:
            args = json.loads(dec.get('args_json') or '{}')
        except json.JSONDecodeError as e:
            args = None
            gpt['parse_errors'] += 1
            res, sc = dict(ok=False, code='INVALID_ARGS', message=f'args_json is not valid JSON: {e}'), {}
            api.stats['invalid'] += 1
            api.stats['calls'] += 1
            hist.append(dict(action=action, args=dec.get('args_json'), result=res))
        if isinstance(r.rec, type(r.rec)) and hasattr(r.rec, 'gpt'):
            r.rec.gpt.update(turn=turn, action=f'{action}({_short(args, 70) if args is not None else "?"})', reason=why)
            r.rec.think_card(r.data, f'GPT thinking ({meta["seconds"]:.0f} s real time, simulation paused) -> {action}', 1.0)
        if args is not None:
            res, sc = api.call(action, args)
            hist.append(dict(action=action, args=args, result={k: v for k, v in res.items() if k != 'state'}))
        if hasattr(r.rec, 'gpt'):
            r.rec.gpt['result'] = f"{res.get('code')} {res.get('message', '')}"[:140]
        rec.update(action=action, args=args, reasoning=why, result={k: v for k, v in res.items() if k != 'state'}, scoring=sc,
                   sim_time_after_s=round(float(r.data.time), 2))
        with open(jl, 'a') as fh:
            fh.write(json.dumps(rec, default=str) + '\n')
        print(f'[{tag}] t{turn:02d} {meta["seconds"]:5.1f}s {action} {_short(args, 120)} -> {res.get("code")} {res.get("message", "")[:120]}'
              f' | sim {r.data.time:.1f}', flush=True)
        if action == 'done' and args is not None and api.done is not None:
            stop_reason = 'done'
            break
    score = r.mission_score()
    if hasattr(r.rec, 'gpt'):
        r.rec.gpt.update(action='mission end', reason=(api.done or {}).get('summary', stop_reason)[:300],
                         result=f"GT mission success: {score['success']}")
        r.hold('end', 2.0)
    r.close()
    # recoveries: a failed / rejected motion-level call later followed by a success of the same action (skills, arm)
    recov = 0
    for i, h in enumerate(api.history):
        if not h['result'].get('ok') and h['result'].get('code') not in ('INVALID_ACTION', 'INVALID_ARGS') and \
                any(x['action'] == h['action'] and x['result'].get('ok') for x in api.history[i + 1:]):
            recov += 1
    perc = r.codex_meta
    out = dict(tag=tag, level=level, scenario=scen, seed=seed, goal=goal, faults={k: v for k, v in faults.items()}, plate_off=off,
               stop_reason=stop_reason, gpt_claim=api.done, success=score['success'], score=score,
               turns=len(api.history), gpt=dict(calls=gpt['calls'], seconds=round(gpt['seconds'], 1), tokens=gpt['tokens'],
                                               failures=gpt['failures'], parse_errors=gpt['parse_errors'],
                                               mean_s=round(gpt['seconds'] / max(1, gpt['calls']), 1),
                                               prompt_chars_mean=int(sum(gpt['prompt_chars']) / max(1, len(gpt['prompt_chars']))),
                                               prompt_chars_max=max(gpt['prompt_chars'], default=0)),
               perception=dict(calls=len(perc), seconds=round(sum(m['seconds'] for m in perc), 1)),
               sim_time_s=score['sim_time_s'], wall_s=round(time.monotonic() - t0, 1), api_stats=api.stats, recoveries=recov,
               actions=[h['action'] for h in api.history])
    od = ROOT / 'out' / 'gpt_runs'
    od.mkdir(parents=True, exist_ok=True)
    (od / f'{tag}.json').write_text(json.dumps(out, indent=1, default=str))
    print(f'[{tag}] END {stop_reason}: GT success {score["success"]} {score["criteria"]} | GPT claim {api.done} | turns {len(api.history)} '
          f'| gpt {gpt["calls"]} calls {gpt["seconds"]:.0f}s | sim {score["sim_time_s"]}s | wall {out["wall_s"]}s | stats {api.stats}', flush=True)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', default='L1', choices=['L1', 'L2', 'L2v0'])
    ap.add_argument('--scen', default='none')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--goal', default='zh')
    ap.add_argument('--video', default='')
    ap.add_argument('--tag', default='')
    ap.add_argument('--max-turns', type=int, default=0)
    ap.add_argument('--max-wall-min', type=float, default=0)
    a = ap.parse_args()
    run(a.level, a.scen, a.seed, a.goal, Path(a.video) if a.video else None, a.tag or None, a.max_turns or None, a.max_wall_min or None)
