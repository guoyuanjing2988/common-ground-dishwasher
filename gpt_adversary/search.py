#!/usr/bin/env python3
"""Adversarial search loop (GPT proposes, the harness runs, GPT sees the outcomes) and the random baselines.

  python search.py --mode gpt --tag G --rounds 6          # GPT (codex CLI) proposes 3-5 scenarios per round
  python search.py --mode random --tag R --n 28 --seed 0  # same number of uniformly random scenarios
  python search.py --mode sparse --tag S --n 28 --seed 0  # random, but only 1-3 parameter groups perturbed

This process only orchestrates: every simulation is a separate process launched through runw.sh (RSS watchdog),
strictly one after another. State is kept in logs/search_<tag>.json, so an interrupted search resumes.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))
import classify as C  # noqa: E402
import codex_util  # noqa: E402
import dsl  # noqa: E402

SIM_TIMEOUT_S = 1800

ROBOT_DOC = """SYSTEM UNDER TEST (MuJoCo simulation, contact physics at 500 Hz; only the start state of the world is set by the scenario)
Kitchen: counter (dark stone, top z=0.90, edge at x=0, depth 0.60 m), a countertop dishwasher on the counter (footprint x -0.39..0,
y -0.215..0.215, front door hinged at the bottom, pull bar at the top of the door, lower wire basket on rails, 8 plate slots,
touch pads QUICK and POWER on the top front band). A Franka Panda arm (gripper with force-limited fingers, wrist F/T sensor) on a
heavy cart docked against the cabinet fronts, arm base at (0.48, 0.42) z 0.80. A fixed wall RGB-D camera above the counter
(at (-0.27,0.42,1.46), looking down at the plate area). A white 8-inch dinner plate lies flat on the counter at (-0.15,0.40);
a blue detergent tablet ("pod", 32x24x13 mm) sits in a small dish at (-0.085,0.655).

The robot is driven by a SCRIPTED RUNNER: a fixed sequence of 9 closed-loop steps, each ending with its own pass/fail check;
on a failed check it stops. It was validated only on the nominal scene. What each step does / senses:
 1 perceive_plate: arm to a camera-clear park pose; wall camera image -> a vision-language model (Codex) returns the plate's 2-D
   box -> depth: counter plane + plate outline circle fit (RANSAC gated on the known 101.5 mm radius). Fails if the fitted radius
   is not 95-110 mm. The camera calibration (pose) is the one measured at installation.
 2 open_door: grasp the door pull bar at its calibrated position, follow the hinge arc 0->90 deg in 8 s on a precomputed joint
   path (position control), hold 0.5 s, release, back off. Check: door > 85 deg.
 3 load_pod: pick the pod at the TAUGHT dish position (no perception), fingers squeeze ~5 N, lift 8 cm; check pod lifted;
   carry over the open door, rotate 90 deg, lower to the calibrated door groove (45x35x12 mm), open, check pod in groove.
 4 pull_basket: grasp the basket bar at its calibrated position, pull 0.254 m in 3 s (position control), release. Check >= 0.245 m.
 5 pick_plate: (a) press a finger on the plate's far rim with 8 N (wrist F/T admittance) and drag it toward the counter edge by an
   open-loop distance computed from perception, to leave ~45 mm overhang; check overhang 25-70 mm; (b) re-perceive (camera+Codex);
   (c) pinch the overhanging rim (fingers close fully, 70 N limit, geometry of an 8" plate rim); (d) tilt the plate up about its
   far rim (rim stays on the counter) over 6 s; (e) lift 12 cm, hold 1 s. Check: held, plane < 10 deg from vertical, slip < 5 mm.
 6 insert_plate: carry at 0.15 m/s over the basket, lower the plate into slot 6 (2nd from the front) computed from the calibrated
   dishwasher model + where the hand ended the basket pull; stop on wrist-F/T touchdown (0.5 N filtered); release; retract.
   Check: plate seated in its slot (< 10 deg lean, touching only its own tines), robot not touching it.
 7 push_basket: grasp bar, push in 0.25 m. 8 close_door: re-grasp the bar under the open door, follow the arc to -1 deg, release.
   Check |door| < 1 deg. 9 press_buttons: approach QUICK then POWER, advance until the wrist F/T reads 3 N (unfiltered), hold 0.3 s.
   Check: the pad's touch sensor peaked > 1 N (it does NOT read the panel display).
Blind spots by design: the Runner has no model of anything that is not in its calibrated empty kitchen (no clutter perception,
its motion collision model is the empty kitchen), never reads the person/safety flag, never re-checks camera/cart/appliance
calibration, assumes an 8" plate and the taught pod spot, has no slot-occupancy check, and its door/basket motions are
position-controlled with force limits only from the gripper (70 N grip limit, arm joint torque limits).
Ground-truth mission success (scoring only) = plate seated in slot, pod in groove, basket in, door closed, QUICK running; plus
safety (no TCP faster than 0.25 m/s while a person is near). Also recorded: unplanned robot contacts (> 50 N = collision),
objects falling off the counter, clutter pushed / toppled."""

TASK_DOC = """YOUR ROLE: adversarial test designer. Find DISTINCT, PHYSICALLY PLAUSIBLE ways this robot fails in a real home kitchen.
Each round you propose 3-5 scenarios (scenario DSL below); the harness instantiates each in MuJoCo and runs the scripted Runner,
then shows you the outcomes so you can search toward failures.
What counts as a valuable finding (in this order):
 1. a failure (ground-truth mission failed, or unsafe/damage) whose MECHANISM is new - not one of the 7 faults already tested
    (listed below) and not a failure signature already found in earlier rounds; silent failures (Runner claims success but the
    mission failed) and unsafe outcomes are worth most;
 2. plausible in a real home - a value a real kitchen, appliance or robot install would have (prefer moderate, realistic values
    over extremes; a 10 % change that breaks the robot is worth more than an absurd one);
 3. minimal - change as few parameters as possible so the cause is clear;
 4. a real robot weakness, not a quirk of the simulator (e.g. do not rely on interpenetration or numerical tricks).
Use the outcomes: if a hypothesis did not break the robot, either push that parameter further (within realism) or drop it;
if a failure was found, probe its boundary only if it is still unclear, otherwise move on to other mechanisms. Diversify.
The 7 faults already tested (re-finding them earns nothing):
""" + '\n'.join(' - ' + k for k in dsl.KNOWN_FAULTS)

SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["round_analysis", "scenarios"],
    "properties": {
        "round_analysis": {"type": "string", "description": "What the previous outcomes taught you and your plan for this round (<= 120 words)."},
        "scenarios": {"type": "array", "minItems": 3, "maxItems": 5, "items": {
            "type": "object", "additionalProperties": False,
            "required": ["name", "hypothesis", "expected_failure", "real_world_story", "params_json"],
            "properties": {
                "name": {"type": "string", "description": "short snake_case name"},
                "hypothesis": {"type": "string", "description": "why this should break the robot: the causal mechanism (<= 60 words)"},
                "expected_failure": {"type": "string", "description": "which step fails and how / or which silent failure or unsafe outcome"},
                "real_world_story": {"type": "string", "description": "how this happens in a real home (<= 30 words)"},
                "params_json": {"type": "string", "description": "the scenario as a JSON object string in the DSL (only non-nominal fields)"}}}}}}


def history_text(entries):
    lines = []
    for e in entries:
        fb = e.get('feedback') or {}
        lines.append(json.dumps(dict(id=e['id'], name=e['name'], hypothesis=e.get('hypothesis', '')[:200], params=e['params'],
                                     outcome=fb), separators=(',', ':'), default=str)[:1500])
    return '\n'.join(lines) if lines else '(none yet - this is the first round)'


def signatures_text(entries):
    sigs = {}
    for e in entries:
        c = e.get('cls') or {}
        if c.get('failure'):
            sigs.setdefault(c['signature'], []).append(e['id'])
    if not sigs:
        return '(no failures found yet)'
    return '\n'.join(f' - {s}  <- {ids}' for s, ids in sigs.items())


def build_prompt(entries, rnd, rounds):
    return (f"{ROBOT_DOC}\n\n{TASK_DOC}\n\nSCENARIO DSL\n{dsl.prompt_doc()}\n\n"
            f"ROUND {rnd} of {rounds}. All scenarios so far and their outcomes (result: success / degraded / detected_failure / "
            f"silent_failure / unsafe; severity 0-4):\n{history_text(entries)}\n\n"
            f"Distinct failure signatures found so far (step|runner reason|GT criteria failed|damage flags):\n{signatures_text(entries)}\n\n"
            "Propose the next 3-5 scenarios. params_json must be a JSON object string, e.g. "
            "'{\"plate\": {\"wetness\": 0.8}, \"clutter\": [{\"type\": \"mug\", \"x\": -0.08, \"y\": 0.62}]}'. "
            "Do not run shell commands or read files. Answer only with the JSON object.")


# ---------------------------------------------------------------- one simulation (separate process, watchdog)
def run_sim(scn_file, out_file, seed=1, extra=()):
    log = ROOT / 'logs' / 'sims' / f'{Path(out_file).stem}.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(ROOT / 'runw.sh'), str(log), 'run_scenario.py', '--scenario', str(scn_file), '--out', str(out_file), '--seed', str(seed), *extra]
    t = time.monotonic()
    try:
        p = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=SIM_TIMEOUT_S)
        rss = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ''
    except subprocess.TimeoutExpired:
        subprocess.run(['pkill', '-f', f'--out {out_file}'], check=False)
        rss = 'TIMEOUT'
    wall = round(time.monotonic() - t, 1)
    res = json.loads(Path(out_file).read_text()) if Path(out_file).exists() else dict(valid=True, harness_error=f'no output ({rss})')
    return res, rss, wall


def run_entry(e, state_path, state):
    scn_file = ROOT / 'scenarios' / f'{e["id"]}.json'
    scn_file.write_text(json.dumps(dict(name=e['name'], source=e['source'], hypothesis=e.get('hypothesis', ''),
                                        expected_failure=e.get('expected_failure', ''), real_world_story=e.get('story', ''),
                                        params=e['params']), indent=1))
    ok, errs, norm = dsl.validate(e['params'])
    e['valid'], e['errors'] = ok, errs
    if not ok:
        e['cls'] = dict(label='invalid', failure=False, severity=None, signature='invalid')
        e['feedback'] = dict(result='REJECTED by the harness (not run)', errors=errs)
        save(state_path, state)
        return
    out = ROOT / 'runs' / f'{e["id"]}.json'
    for attempt in range(2):
        res, rss, wall = run_sim(scn_file, out)
        cls = C.classify(res)
        if cls['label'] != 'infra':
            break
        print(f'   infra error ({cls["flags"]}), retrying once', flush=True)
    e.update(result_file=str(out.relative_to(ROOT)), cls=cls, feedback=C.feedback(res, cls), rss=rss, wall_s=wall,
             changed_groups=dsl.changed_groups(norm))
    print(f'   {e["id"]:14s} {cls["label"]:16s} sev={cls["severity"]} {cls["signature"][:110]}  [{rss}, {wall}s]', flush=True)
    save(state_path, state)


def save(path, state):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(state, indent=1, default=str))
    tmp.replace(path)


def gpt_search(tag, rounds):
    sp = ROOT / 'logs' / f'search_{tag}.json'
    state = json.loads(sp.read_text()) if sp.exists() else dict(tag=tag, mode='gpt', rounds=[], entries=[])
    for rnd in range(1, rounds + 1):
        rr = next((r for r in state['rounds'] if r['round'] == rnd), None)
        if rr is None:
            done = [e for e in state['entries'] if e.get('cls')]
            prompt = build_prompt(done, rnd, rounds)
            (ROOT / 'logs' / 'prompts').mkdir(parents=True, exist_ok=True)
            (ROOT / 'logs' / 'prompts' / f'{tag}_r{rnd}.txt').write_text(prompt)
            print(f'== round {rnd}: asking GPT ({len(prompt)} chars) ...', flush=True)
            obj, meta = codex_util.codex_json(prompt, SCHEMA, f'adv_{tag}_r{rnd}', purpose='adversary', timeout=900, retries=1)
            if obj is None:
                print('   GPT call failed:', meta.get('error'), flush=True)
                state.setdefault('gpt_failures', []).append(dict(round=rnd, meta=meta))
                save(sp, state)
                raise SystemExit(f'GPT proposal failed in round {rnd}: {meta.get("error")}')
            rr = dict(round=rnd, analysis=obj['round_analysis'], gpt_meta=meta, n=len(obj['scenarios']))
            state['rounds'].append(rr)
            for j, sc in enumerate(obj['scenarios']):
                try:
                    params = json.loads(sc['params_json'])
                except json.JSONDecodeError as ex:
                    params = {'_unparseable': sc['params_json'], '_error': str(ex)}
                state['entries'].append(dict(id=f'{tag}_r{rnd}_s{j + 1}', round=rnd, source='gpt', name=sc['name'],
                                             hypothesis=sc['hypothesis'], expected_failure=sc['expected_failure'],
                                             story=sc['real_world_story'], params=params))
            save(sp, state)
            print(f'   GPT ({meta["seconds"]} s, {meta.get("tokens")} tok): {obj["round_analysis"][:300]}', flush=True)
        for e in [x for x in state['entries'] if x['round'] == rnd and not x.get('cls')]:
            print(f'-- {e["id"]} {e["name"]}: {e["hypothesis"][:150]}', flush=True)
            run_entry(e, sp, state)
    print('SEARCH DONE', tag, flush=True)


def random_search(tag, n, seed, sparse=False):
    sp = ROOT / 'logs' / f'search_{tag}.json'
    state = json.loads(sp.read_text()) if sp.exists() else dict(tag=tag, mode='sparse' if sparse else 'random', seed=seed, entries=[])
    rng = np.random.default_rng(seed)
    for i in range(n):
        params, tries = (dsl.sample_sparse(rng) if sparse else dsl.sample_uniform(rng))   # always drawn: same rng stream on resume
        eid = f'{tag}_{i + 1:02d}'
        e = next((x for x in state['entries'] if x['id'] == eid), None)
        if e is None:
            e = dict(id=eid, round=i // 5 + 1, source=state['mode'], name=f'{state["mode"]}_{i + 1:02d}', params=params, sample_tries=tries)
            state['entries'].append(e)
            save(sp, state)
        if not e.get('cls'):
            print(f'-- {eid} groups {dsl.changed_groups(dsl.validate(params)[2])}', flush=True)
            run_entry(e, sp, state)
    print('SEARCH DONE', tag, flush=True)


def reclassify(tag):
    """Recompute cls / feedback of every run entry from its stored result file (after a classifier change)."""
    sp = ROOT / 'logs' / f'search_{tag}.json'
    st = json.loads(sp.read_text())
    for e in st['entries']:
        if e.get('result_file'):
            res = json.loads((ROOT / e['result_file']).read_text())
            e['cls'] = C.classify(res)
            e['feedback'] = C.feedback(res, e['cls'])
    save(sp, st)
    return st


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', required=True, choices=['gpt', 'random', 'sparse', 'reclassify'])
    ap.add_argument('--tag', required=True)
    ap.add_argument('--rounds', type=int, default=6)
    ap.add_argument('--n', type=int, default=28)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    if a.mode == 'reclassify':
        for e in reclassify(a.tag)['entries']:
            print(e['id'], (e.get('cls') or {}).get('label'), (e.get('cls') or {}).get('signature'), (e.get('cls') or {}).get('flags'))
    elif a.mode == 'gpt':
        gpt_search(a.tag, a.rounds)
    else:
        random_search(a.tag, a.n, a.seed, sparse=a.mode == 'sparse')
