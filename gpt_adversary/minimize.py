#!/usr/bin/env python3
"""Minimal reproducing scenario for a failure (sequential sims, each through runw.sh).

  python minimize.py --entry G_r2_s3 [--mode-key "<stage>|<gt failed>|<flags>"] [--bisect 2] [--repro-seed 2]

1. group-level reduction: revert each changed parameter group to nominal (one at a time, greedily); keep the revert if the
   run still fails in the SAME mode (same stopping step + same ground-truth criteria failed + same damage-flag classes);
2. magnitude bisection on what is left: p = nominal + lam (p - nominal), lam in (0, 1], `--bisect` halvings, to find how
   far from nominal the failure starts (numeric parameters only; clutter / preload / booleans kept as they are);
3. reproduce the minimal scenario with another seed (sensor noise + a fresh Codex perception call).
Result: logs/minimize/<entry>.json and scenarios/min_<entry>.json.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))
import classify as C  # noqa: E402
import dsl  # noqa: E402
import search as SE  # noqa: E402


def mode_key(cls):
    if not cls.get('failure'):
        return None
    flags = sorted({f.split('(')[0].split(':')[0] for f in cls['flags'] if not f.startswith(('contact', 'plate_lean', 'robot_plate_contact')) and
                    not f.endswith(('_pushed', '_toppled'))})
    return f'{cls.get("stage")}|{",".join(cls["gt_failed"])}|{",".join(flags)}'


def find_entry(eid):
    for f in sorted((ROOT / 'logs').glob('search_*.json')):
        st = json.loads(f.read_text())
        for e in st['entries']:
            if e['id'] == eid:
                return e
    raise KeyError(eid)


def scale(s, lam):
    """Numeric parameters (and the plate colour) moved toward nominal: nominal + lam (value - nominal). Clutter, person,
    preloaded slots, booleans and categories are kept as they are."""
    d0 = dsl.defaults()
    s2 = copy.deepcopy(s)
    for g, v in s.items():
        if g in ('clutter', 'person') or not isinstance(v, dict):
            continue
        for f, x in v.items():
            x0 = d0[g][f]
            if dsl._num(x) and dsl._num(x0) and x != x0:
                s2[g][f] = round(x0 + lam * (x - x0), 5)
            elif f == 'rgb' and x != x0:
                s2[g][f] = [round(a0 + lam * (a1 - a0), 4) for a0, a1 in zip(x0, x)]
    return s2


class Runner:
    def __init__(self, eid):
        self.eid = eid
        self.n = 0
        self.log = []
        (ROOT / 'logs' / 'minimize').mkdir(parents=True, exist_ok=True)
        (ROOT / 'runs' / 'min').mkdir(parents=True, exist_ok=True)

    def run(self, s, note, seed=1):
        ok, errs, norm = dsl.validate(dsl.to_input(s))
        if not ok:
            self.log.append(dict(note=note, valid=False, errors=errs))
            print(f'   [{note}] invalid: {errs}', flush=True)
            return None, None
        self.n += 1
        tag = f'min_{self.eid}_{self.n:02d}'
        f = ROOT / 'scenarios' / 'min' / f'{tag}.json'
        f.parent.mkdir(exist_ok=True)
        f.write_text(json.dumps(dict(name=tag, source='minimize', note=note, params=dsl.to_input(norm)), indent=1))
        out = ROOT / 'runs' / 'min' / f'{tag}.json'
        for _ in range(2):
            res, rss, wall = SE.run_sim(f, out, seed=seed)
            cls = C.classify(res)
            if cls['label'] != 'infra':
                break
        k = mode_key(cls)
        self.log.append(dict(note=note, tag=tag, params=dsl.to_input(norm), seed=seed, label=cls['label'], mode_key=k,
                             signature=cls.get('signature'), rss=rss, wall_s=wall, result_file=str(out.relative_to(ROOT))))
        print(f'   [{note}] {cls["label"]:16s} {k}  [{rss}, {wall}s]', flush=True)
        return cls, k


def minimize(eid, key=None, bisect=2, repro_seed=2):
    e = find_entry(eid)
    ok, errs, s = dsl.validate(e['params'])
    assert ok, errs
    key = key or mode_key(e['cls'])
    print(f'== minimize {eid}: target mode {key}; groups {dsl.changed_groups(s)}', flush=True)
    R = Runner(eid)
    cur = s
    for g in dsl.changed_groups(s):
        if len(dsl.changed_groups(cur)) <= 1:
            break
        cand = dsl.revert_groups(cur, [g])
        if not dsl.diff_from_nominal(cand):
            continue
        cls, k = R.run(cand, f'revert {g}')
        if k == key:
            cur = cand
    lam_fail, lam_ok = 1.0, 0.0
    for _ in range(bisect):
        lam = (lam_fail + lam_ok) / 2
        cand = scale(cur, lam)
        if not dsl.diff_from_nominal(cand) or cand == cur:
            break
        cls, k = R.run(cand, f'scale {lam:.3f}')
        if k == key:
            lam_fail = lam
        else:
            lam_ok = lam
    minimal = scale(cur, lam_fail) if lam_fail < 1 else cur
    rep = R.run(minimal, f'reproduce seed {repro_seed}', seed=repro_seed)
    out = dict(entry=eid, target_mode=key, original=dsl.to_input(s), minimal=dsl.to_input(minimal), groups=dsl.changed_groups(minimal),
               lam_fail=lam_fail, lam_ok=lam_ok, reproduced_seed2=rep[1] == key, runs=R.log, n_runs=R.n)
    (ROOT / 'logs' / 'minimize' / f'{eid}.json').write_text(json.dumps(out, indent=1, default=str))
    (ROOT / 'scenarios' / f'min_{eid}.json').write_text(json.dumps(dict(name=f'min_{eid}', source='minimized', from_entry=eid,
                                                                        target_mode=key, params=dsl.to_input(minimal)), indent=1))
    print(f'== minimal {eid}: {json.dumps(dsl.to_input(minimal))}  lam_fail {lam_fail} lam_ok {lam_ok} reproduced(seed {repro_seed}) {rep[1] == key}', flush=True)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--entry', nargs='+', required=True)
    ap.add_argument('--bisect', type=int, default=2)
    ap.add_argument('--repro-seed', type=int, default=2)
    a = ap.parse_args()
    for eid in a.entry:
        minimize(eid, None, a.bisect, a.repro_seed)
