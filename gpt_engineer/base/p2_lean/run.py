#!/usr/bin/env python3
"""End-to-end dishwasher-loading simulation (Franka Panda, MuJoCo).

  python run.py                          # option M1, one run with video -> out/M1_run.mp4 + out/M1_single.json
  python run.py --option M2 --trials 5   # N randomised trials (plate +-10 mm, +-10 deg yaw), seeded
  python run.py --no-video --seed 7
  python run.py --debug-truth-perception # DEVELOPMENT ONLY: skip Codex, use the true pose (+1 mm noise); never reported
Options M1 / M2 / M3 / M4 are defined in OPTIONS below (see MOUNTING.md).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import pipeline as PL
import scene as S

ROOT = Path(__file__).resolve().parent

M3_RACK = S.Layout(plate_mode='rack', rack=(-0.22, 0.80), dish=(-0.47, 0.58), plate0=(-0.15, 0.80),
                   cam_pos=(-0.56, 0.80, 1.36), cam_look=(-0.22, 0.80, 1.00))

OPTIONS = {
    # M1: heavy cart docked against the cabinet fronts, braked; pedestal 0.80 m; arm base 0.48 m in front of the
    # counter edge, 0.42 m right of the dishwasher centre (mount_study grid winner by joint margin).
    'M1': PL.Option('M1', S.Mount('M1', x=.48, y=.42, z=.80, yaw=195., cart_lx=.65, cart_ly=.60, cart_mass=120.)),
    # M2: shorter docked cart (base 0.40 m from the edge) + 0.40 m linear axis along the counter front; one rail
    # setpoint per step (rail_scan.py at x=0.40, z=0.80: best base y per task group). Rail values are relative to y=0.40.
    'M2': PL.Option('M2', S.Mount('M2', x=.40, y=.40, z=.80, yaw=195., rail_travel=.40, cart_lx=.57, cart_ly=.60, cart_mass=120.),
                    rail=dict(start=0., perceive_plate=0., open_door=0., load_pod=0., pull_basket=.08, pick_plate=-.04,
                              insert_plate=-.04, push_basket=.08, close_door=0., press_buttons=.08)),
    # M3: Panda clamped on the worktop right of the dishwasher (m3_scan.py best rack placement, yaw 315). The flat-plate
    # pick is unreachable from every counter placement that also reaches the door, so the plate starts upright in a
    # drying rack (poc_plate). IK uses static-shell collision avoidance (the base is 0.17 m from the machine's side wall).
    'M3': PL.Option('M3', S.Mount('M3', x=-.10, y=.38, z=.90, yaw=315.), layout=M3_RACK,
                    park=(0.15, 0.30, 1.22), hub=(0.20, 0.30, 1.22), ik_collision=True, door=(0., 20., -1), counter_clear_z=1.22,
                    door_pre=.02, door_regrasp_open=PL.grip_ctrl(.0095)),
    # M3a: same clamp, yaw 45 (the only M3 yaw whose door arc passes the pose study, at 2.5 deg margin)
    'M3a': PL.Option('M3a', S.Mount('M3', x=-.10, y=.38, z=.90, yaw=45.), layout=M3_RACK,
                     park=(0.15, 0.30, 1.22), hub=(0.20, 0.30, 1.22), ik_collision=True, door=(1., 40., 1), counter_clear_z=1.22,
                     door_pre=.02),
    # M3flat: M3 attempting the FLAT plate (documents the flat-pick reach failure in simulation)
    'M3flat': PL.Option('M3flat', S.Mount('M3', x=-.10, y=.38, z=.90, yaw=315.),
                        layout=S.Layout(plate_mode='flat', plate0=(-0.15, 0.80), dish=(-0.47, 0.58),
                                        cam_pos=(-0.27, 0.80, 1.46), cam_look=(-0.07, 0.80, 0.90)),
                        park=(0.15, 0.30, 1.22), hub=(0.20, 0.30, 1.22), ik_collision=True, door=(0., 20., -1), counter_clear_z=1.22,
                        door_pre=.02, door_regrasp_open=PL.grip_ctrl(.0095), approach_deg=-60.),
}


def run_one(opt, off, video, seed, codex, keyframes=None, tag='', run_tag=''):
    r = PL.Runner(opt, plate_off=off, video=video, codex=codex, seed=seed, keyframes_dir=keyframes,
                  log=lambda s: print(f'{tag}{s}', flush=True), run_tag=run_tag)
    if not codex:
        r.perceive = _truth_perceive.__get__(r)
    res = r.run()
    W = res.pop('wrench')
    res['mount_wrench'] = wrench_summary(W)
    res['debug_truth_perception'] = not codex
    return res, W


def _truth_perceive(self, tag, mode):
    """DEVELOPMENT ONLY. Truth + 1 mm noise instead of camera/Codex."""
    import mujoco  # noqa: F401
    o = self.data.xpos[self.pid].copy() + np.r_[self.rng.normal(0, .001, 2), 0]
    P = self.data.xmat[self.pid].reshape(3, 3).copy()
    if mode == 'flat':
        P = np.eye(3)
    return dict(origin=o, P=P, radius_fit=S.R_OUT, mode=mode), dict(debug_truth=True, centre_err_mm=1., radius_fit_mm=101.5,
                                                                    normal_err_deg=0., codex_s=0)


def wrench_summary(W):
    """Arm -> cart wrench at the mount over the run (sensor = cart on link0, so arm on cart = -sensor)."""
    if len(W) == 0:
        return {}
    F, M = -W[:, 1:4], -W[:, 4:7]
    return dict(n=len(W), Fz_min_N=float(F[:, 2].min()), Fz_max_N=float(F[:, 2].max()),
                Fh_max_N=float(np.linalg.norm(F[:, :2], axis=1).max()), Fx_min_N=float(F[:, 0].min()), Fx_max_N=float(F[:, 0].max()),
                Fy_abs_max_N=float(np.abs(F[:, 1]).max()), Mh_max_Nm=float(np.linalg.norm(M[:, :2], axis=1).max()))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--option', default='M1', choices=sorted(OPTIONS))
    ap.add_argument('--trials', type=int, default=0)
    ap.add_argument('--seed', type=int, default=20261010)
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--debug-truth-perception', action='store_true')
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    opt = OPTIONS[a.option]
    out = ROOT / 'out'
    out.mkdir(exist_ok=True)
    codex = not a.debug_truth_perception
    sfx = ('_DEBUG' if not codex else '') + (f'_{a.tag}' if a.tag else '')
    if a.trials:
        rng = np.random.default_rng(a.seed)
        offs = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)], size=(a.trials, 3))
        if opt.layout.plate_mode == 'rack':      # plate standing in a 35 mm-pitch rack: poc_plate randomisation
            offs = offs * np.array([.3, .3, .3])  # -> +-3 mm, +-3 deg
        rows, Ws = [], []
        for i, o in enumerate(offs):
            print(f'--- {a.option} trial {i}: offset {o[0]*1000:+.1f} mm {o[1]*1000:+.1f} mm {math.degrees(o[2]):+.1f} deg', flush=True)
            res, W = run_one(opt, o, None, a.seed + i, codex, tag=f'[{i}] ', run_tag=f'{a.option}_trial{i}')
            rows.append(res)
            Ws.append(W)
            with open(out / f'{a.option}_trials{sfx}_partial.jsonl', 'a') as fh:     # survives a crash
                fh.write(json.dumps(res, default=float) + '\n')
            np.save(out / f'{a.option}_trials{sfx}_wrench_t{i}.npy', W)
            print(f'--- trial {i}: {"SUCCESS" if res["success"] else "FAIL: " + str(res["failed"])}  sim {res["sim_time_s"]} s, '
                  f'wall {res["wall_s"]} s, codex {res["codex_s"]}', flush=True)
        per = {s: sum(any(x['step'] == s and x['ok'] for x in r['steps']) for r in rows) for s in PL.STEPS}
        att = {s: sum(any(x['step'] == s for x in r['steps']) for r in rows) for s in PL.STEPS}
        summ = dict(option=a.option, seed=a.seed, n=len(rows), success=sum(r['success'] for r in rows),
                    per_step_success=per, per_step_attempted=att, trials=rows)
        (out / f'{a.option}_trials{sfx}.json').write_text(json.dumps(summ, indent=1, default=float))
        np.save(out / f'{a.option}_trials{sfx}_wrench.npy', np.concatenate(Ws) if Ws else np.zeros((0, 7)))
        print('SUCCESS', summ['success'], '/', len(rows), per)
        return
    video = None if a.no_video else out / f'{a.option}_run{sfx}.mp4'
    kf = out / f'{a.option}_keyframes{sfx}'
    kf.mkdir(exist_ok=True)
    res, W = run_one(opt, np.zeros(3), video, a.seed, codex, keyframes=None if a.no_video else kf, run_tag=f'{a.option}_single')
    (out / f'{a.option}_single{sfx}.json').write_text(json.dumps(res, indent=1, default=float))
    np.save(out / f'{a.option}_single{sfx}_wrench.npy', W)
    print(json.dumps({k: v for k, v in res.items() if k not in ('steps',)}, indent=1, default=float)[:3000])


if __name__ == '__main__':
    main()
