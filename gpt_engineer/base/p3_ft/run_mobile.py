#!/usr/bin/env python3
"""Mobile dishwasher-loading mission (wheeled base + FR3-limited Panda + explicit controller architecture).

  python run_mobile.py --video                       # one full real-Codex mission with video + key frames
  python run_mobile.py --trials 5                    # N full missions, random plate (+-10 mm, +-10 deg) and start (+-5 cm, +-5 deg)
  python run_mobile.py --dock-sweep                  # dock-offset tolerance (+-2/+-4 cm, +3/... ), arm uses the measured base pose
  python run_mobile.py --pose-sweep                  # spec: +-2/+-5 mm, +-0.2/+-0.5 deg, NOT compensated (robot believes nominal)
  python run_mobile.py --estop                       # e-stop while carrying the plate, then resume
  python run_mobile.py --low-battery                 # start with SoC below the threshold -> go charge
  python run_mobile.py --nonconductive               # press the panel with the plain finger pad (no stylus)
  python run_mobile.py --brake-test                  # e-stop braking from 0.5 m/s, arm stowed (v1.0 and provisional specs)
  python run_mobile.py --tcp-cap 0.25 ...            # cap TCP / elbow speed (spec safety design) - cycle-time impact
  python run_mobile.py --debug-truth-perception ...  # true plate pose (+1 mm) after a 15.8 s simulated delay instead of Codex
Every run is ONE process; run them one at a time under ./guard.sh (RSS watchdog).
"""
from __future__ import annotations

import argparse
import json
import math
import resource
import sys
import time
from pathlib import Path

import numpy as np

import pipeline as PL
import scene as S
import scene_mobile as SM

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'
OPTION = PL.Option('mobile', S.Mount('M1', x=.48, y=.42, z=.80, yaw=195., cart_lx=.65, cart_ly=.60, cart_mass=133.))


def clear_start(dx, dy, dyaw):
    """Dock pose + offset, shifted out along x if a rear corner (chassis or dock plate) would start inside the cabinet
    doors (a yawed base touches the doors with one corner first; 0.5 mm clearance). Returns (start, extra_dx)."""
    x, y, hd = SM.dock_base_pose(dx, dy, dyaw)
    th = math.radians(hd)
    pts = [(SM.REAR_X, sy) for sy in (-SM.HY, SM.HY)]        # chassis corners (the dock plate is below the doors)
    xmin = min(x + px * math.cos(th) - py * math.sin(th) for px, py in pts)
    extra = max(0., SM.CAB_X + .0005 - xmin)
    return (x + extra, y, hd), extra


def rss_mb():
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return round(r / (1024 * 1024 if sys.platform == 'darwin' else 1024), 1)


def summarize(sysm, wall, extra=None):
    sup, pl = sysm.supervisor, sysm.planner
    b, pw = sysm.battery, sysm.power
    steps = {s['step']: s for s in pl.steps}
    res = dict(run=sysm.run_tag, outcome=sup.outcome, spec=SM.SPEC_META, sim_time_s=round(sysm.d.time, 2), wall_s=round(wall, 1),
               peak_rss_mb=rss_mb(), transitions=sup.transitions, jobs=sup.results, task_steps=pl.steps,
               all_9_steps_ok=bool(all(steps.get(s, {}).get('ok') for s in PL.STEPS[:-1])) and
               all(r['ok'] for r in sup.results if r['job'] == 'press') and
               any(r.get('panel_after', {}).get('state') == 'RUNNING' for r in sup.results),
               panel_events=sysm.panel.events, panel_final=sysm.panel.label,
               codex=sysm.perception.codex_log, debug_truth_perception=not sysm.perception.codex,
               battery=dict(soc0=round(b.soc0, 4), soc1=round(b.soc, 4), soc_drop_pct=round((b.soc0 - b.soc) * 100, 3),
                            E_terminal_Wh=round(b.E_out_Wh, 2), Ah=round(b.Ah_out, 3), I_peak_A=round(b.I_peak_seen, 1),
                            I_over_cont_s=round(b.I_over_cont_s, 2), V_end=round(b.V, 2)),
               energy_Wh=dict({k: round(v, 3) for k, v in pw.E.items()}),
               energy_by_state_Wh={k: round(v, 3) for k, v in sysm.energy_state.items()},
               time_by_state_s={k: round(v, 2) for k, v in sysm.time_state.items()},
               peak_power_W={k: round(v, 1) for k, v in sysm.peakP.items()},
               base=dict(mcu=sysm.base_mcu.stats, phase=_phase_stats(sysm)),
               arm=dict(ctrl=sysm.arm.stats, per_state={k: {kk: (round(vv, 4) if isinstance(vv, float) else vv) for kk, vv in v.items()}
                                                         for k, v in sysm.step_speed.items()}),
               lidar_fixes=pl.fix_count, messages=sysm.bus.summary(),
               max_penetration_mm=round(pl.metrics['max_penetration_mm'], 3), max_pen_where=pl.metrics['max_pen_where'],
               unexpected_robot_contacts=[dict(step=k[0], phase=k[1], robot=k[2], other=k[3], max_force_N=round(v, 2))
                                          for k, v in pl.contact_log.items()],
               warnings={str(i): int(w.number) for i, w in enumerate(sysm.d.warning) if w.number},
               estop=dict(operator=sysm.operator.log, supervisor=sup.estop_log, probe=sysm.estop_probe), track_err_max_mm=pl.metrics.get('track_err_max_mm'))
    W = np.array(pl.wrench, float)
    if len(W):
        F, M = -W[:, 1:4], -W[:, 4:7]
        res['mount_wrench'] = dict(Fz_min_N=round(float(F[:, 2].min()), 1), Fz_max_N=round(float(F[:, 2].max()), 1),
                                   Fh_max_N=round(float(np.linalg.norm(F[:, :2], axis=1).max()), 1),
                                   Mh_max_Nm=round(float(np.linalg.norm(M[:, :2], axis=1).max()), 1))
    if any(t['to'] == 'CHARGING' for t in sup.transitions):
        ch = [j for j in sup.results if j['state'] == 'CHARGING']
        if ch:
            dt_ = ch[-1]['t1'] - ch[-1]['t0']
            rate = -b.Ah_out / max(dt_, 1e-6)          # Ah/s gained on the charger (net of the awake loads)
            res['low_battery'] = dict(soc_min=sup.soc_min, soc_at_check=round(b.soc0, 4), charged_s=round(dt_, 1),
                                      net_charge_A=round(rate * 3600, 2),
                                      time_to_threshold_min=round((sup.soc_min - b.soc) * b.cap / max(rate * 3600, 1e-6) * 60, 1))
    if extra:
        res.update(extra)
    return res


def _phase_stats(sysm):
    out = {}
    for k, v in sysm.phase_stats.items():
        out[k] = dict(min_load_N={n: round(x, 1) for n, x in v['min_load'].items()},
                      max_load_N={n: round(x, 1) for n, x in v['max_load'].items()},
                      cop_margin_min_m=round(v['cop_margin_min'], 4), brake_torque_max_Nm=round(v['brake_tq_max'], 2),
                      dock_F_max_N=round(v['dock_F_max'], 1), dock_F_min_N=round(v['dock_F_min'], 1),
                      creep_mm_max=round(v['creep_mm_max'], 3), creep_deg_max=round(v['creep_deg_max'], 4),
                      base_speed_max_mm_s=round(v.get('base_speed_max', 0.) * 1000, 2))
    return out


def run_one(args, tag, plate_off=(0, 0, 0), start=SM.START_NOMINAL, plan_kind='full', offset=(0, 0, 0), vblocks=True,
            soc0=.90, estop=None, video=False, base_pose_source='lidar', believed_base=None, tool='stylus', seed=0, codex=None):
    from system import System
    from task_supervisor import door_open_panel_plan, mission_plan
    codex = (not args.debug_truth_perception) if codex is None else codex
    tag = tag + ('_DEBUG' if not codex else '')
    plan = door_open_panel_plan() if plan_kind == 'door_open_panel' else mission_plan(plan_kind, offset, vblocks, tool)
    kf = None
    if video:
        kf = ROOT / 'frames'
        kf.mkdir(exist_ok=True)
    print(f'=== {tag}: plate {np.round(plate_off, 4).tolist()} start {np.round(start, 3).tolist()} plan {plan_kind} '
          f'offset {offset} codex {codex} tool {tool} soc0 {soc0}', flush=True)
    sysm = System(args, plate_off=plate_off, start=start, plan=plan, video=(ROOT / 'mission.mp4') if video else None,
                  run_tag=tag, outdir=OUT, codex=codex, seed=seed, soc0=soc0, vblocks=vblocks, estop=estop, keyframes=kf,
                  base_pose_source=base_pose_source, believed_base=believed_base, tcp_cap=args.tcp_cap, tool=tool,
                  log=lambda s: print(f'[{tag}] {s}', flush=True))
    if getattr(args, 'insert_aim', None) is not None:
        sysm.planner.insert_aim = args.insert_aim
    wall = sysm.run(t_max=args.t_max)
    res = summarize(sysm, wall, dict(plate_offset=[float(v) for v in plate_off], start_pose=[float(v) for v in start],
                                     dock_offset=list(offset), plan=plan_kind, tool=tool, base_pose_source=base_pose_source,
                                     believed_base=believed_base, tcp_cap=args.tcp_cap))
    (OUT / f'{tag}.json').write_text(json.dumps(res, indent=1, default=float))
    jobs = ' '.join(f"{r['state']}:{'ok' if r['ok'] else 'FAIL'}" for r in res['jobs'])
    print(f'=== {tag}: outcome {res["outcome"]}  9 steps ok {res["all_9_steps_ok"]}  sim {res["sim_time_s"]} s wall {res["wall_s"]} s '
          f'SoC drop {res["battery"]["soc_drop_pct"]} %  E {res["battery"]["E_terminal_Wh"]} Wh  rss {res["peak_rss_mb"]} MB', flush=True)
    print('    ', jobs[:1500], flush=True)
    for r in res['jobs']:
        if not r['ok']:
            print('     FAIL', r['state'], r['reason'][:300], flush=True)
    return res


def brake_test(args):
    """E-stop braking from max speed, arm stowed: stopping distance, min wheel / caster normal force (10 ms average),
    max caster lift, CoP margin. Cat 0 (brakes bite, 30 ms engage) and Cat 1 (motors decelerate at 1.5 m/s^2, then brake)."""
    import check_base as CB
    import mujoco
    from system import hull2d, inside_margin
    out = []
    for label, base in (('v1.0', None), ('provisional', SM.PROVISIONAL)):
        for direction in (1., -1.):
            for cat in ('cat0', 'cat1'):
                R = CB.Rig((1.5, -2.5, 90.), vblocks=False, base=base)
                m, d = R.m, R.d
                for _ in range(300):
                    R.step(brake=True)
                vmax, amax = R.bs['max_speed_m_s'], R.bs['max_accel_m_s2']
                v = 0.
                while abs(v) < vmax - 1e-9:
                    v = direction * min(vmax, abs(v) + amax * .002)
                    R.step(v=v)
                for _ in range(500):
                    R.step(v=v)
                p0, v0 = d.xpos[R.bid].copy(), R.speed()
                T = R.bs['brake_holding_torque_Nm_per_wheel']
                buf, lifts, cops, t = [], [], [], 0.
                vc = v
                while R.speed() > 1e-3 or t < .4:
                    if cat == 'cat1' and abs(vc) > 1e-3:
                        vc = math.copysign(max(0., abs(vc) - 1.5 * .002), vc)
                        R.step(v=vc)
                        lvl = 0.
                    else:
                        lvl = min(1., getattr(R, 'lvl', 0.) + .002 / .03)
                        R.lvl = lvl
                        R.brake(lvl)
                        d.ctrl[R.ma] = 0
                        R.g[:] = 0
                        for b in R.pb:
                            mujoco.mj_applyFT(m, d, -m.body_mass[b] * m.opt.gravity, np.zeros(3), d.xipos[b], b, R.g)
                        d.qfrc_applied[:] = 0
                        d.qfrc_applied[R.adof] = R.g[R.adof]
                        mujoco.mj_step(m, d)
                    t += .002
                    L = R.loads()
                    buf.append(list(L.values()))
                    lifts.append([d.xpos[m.body(f'caster{i}').id][2] - R.bs['caster_wheel_radius_m'] for i in range(4)])
                    if len(buf) >= 5:                      # CoP from 10 ms averaged loads at the wheel / caster contacts
                        la = np.mean(buf[-5:], 0)
                        bp = d.body('base')
                        Rm = bp.xmat.reshape(3, 3)
                        sup = [bp.xpos[:2] + (Rm @ np.array([x, y, 0.]))[:2] for x, y in [(0, .25), (0, -.25)] + list(SM.CASTER_XY)]
                        cops.append(inside_margin(hull2d(sup), np.sum(np.array(sup) * la[:, None], 0) / la.sum()))
                Lb = np.array(buf)
                Lf = np.array([Lb[i:i + 5].mean(0) for i in range(len(Lb) - 4)])
                G = np.array(lifts)
                names = list(R.loads().keys())
                r = dict(spec=label, direction='forward' if direction > 0 else 'reverse', stop=cat, v0=round(v0, 3),
                         stop_mm=round(float(np.linalg.norm(d.xpos[R.bid][:2] - p0[:2])) * 1000, 1), stop_s=round(t, 2),
                         min_load_10ms_N={n: round(float(x), 1) for n, x in zip(names, Lf.min(0))},
                         caster_lift_max_mm=round(float(G.max()) * 1000, 3),
                         unloaded_casters_s=round(float(((Lf[:, 2:] < 1).any(1)).sum() * .002), 3),
                         cop_margin_min_m=round(min(cops), 4) if cops else None)
                print(r, flush=True)
                out.append(r)
    (OUT / 'brake_test.json').write_text(json.dumps(out, indent=1))
    print('peak RSS MB', rss_mb())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--trials', type=int, default=0)
    ap.add_argument('--video', action='store_true')
    ap.add_argument('--dock-sweep', action='store_true')
    ap.add_argument('--pose-sweep', action='store_true')
    ap.add_argument('--estop', action='store_true')
    ap.add_argument('--low-battery', action='store_true')
    ap.add_argument('--nonconductive', action='store_true')
    ap.add_argument('--brake-test', action='store_true')
    ap.add_argument('--panel-door-open', action='store_true')
    ap.add_argument('--debug-truth-perception', action='store_true')
    ap.add_argument('--tcp-cap', type=float, default=None)
    ap.add_argument('--soc-min', type=float, default=.30)
    ap.add_argument('--seed', type=int, default=20261011)
    ap.add_argument('--t-max', type=float, default=1500.)
    ap.add_argument('--only', type=str, default=None, help='sweep: comma-separated case indices')
    ap.add_argument('--tag', type=str, default='')
    ap.add_argument('--noslip', type=int, default=3)
    ap.add_argument('--insert-aim', type=float, default=None, help='insert aim offset along -plate normal (m); default 0.003')
    ap.add_argument('--base-pose', choices=['lidar', 'truth'], default='lidar')
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    sfx = f'_{a.tag}' if a.tag else ''
    if a.brake_test:
        return brake_test(a)
    rng = np.random.default_rng(a.seed)
    if a.trials:
        rows = []
        for i in range(a.trials):
            po = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)])
            so = rng.uniform([-.05, -.05, -5.], [.05, .05, 5.])
            st = (SM.START_NOMINAL[0] + so[0], SM.START_NOMINAL[1] + so[1], SM.START_NOMINAL[2] + so[2])
            rows.append(run_one(a, f'mission_trial{i}{sfx}', plate_off=po, start=st, seed=a.seed + i, base_pose_source=a.base_pose))
        return
    if a.dock_sweep or a.pose_sweep:
        if a.dock_sweep:
            cases = [(0, 0, 0), (0, .02, 0), (0, -.02, 0), (0, .04, 0), (0, -.04, 0), (.02, 0, 0), (.04, 0, 0),
                     (0, 0, 3.), (0, 0, -3.), (0, 0, 6.), (0, 0, -6.)]
            kind = 'dock'
        else:
            cases = [(.002, 0, 0), (-.002, 0, 0), (.005, 0, 0), (-.005, 0, 0), (0, .002, 0), (0, -.002, 0), (0, .005, 0),
                     (0, -.005, 0), (0, 0, .2), (0, 0, -.2), (0, 0, .5), (0, 0, -.5)]
            kind = 'pose'
        idx = range(len(cases)) if a.only is None else [int(x) for x in a.only.split(',')]
        for i in idx:
            dx, dy, dyaw = cases[i]
            st, extra = clear_start(dx, dy, dyaw)
            if extra > 0:
                print(f'case {i}: start shifted +{extra * 1000:.1f} mm in x so the yawed base does not start inside the doors', flush=True)
            bb = None
            if kind == 'pose':
                x0, y0, _ = SM.dock_base_pose()
                bb = (x0, y0, 0.)
            run_one(a, f'{kind}sweep_{i}_dx{dx*1000:+.0f}mm_dy{dy*1000:+.0f}mm_dyaw{dyaw:+.1f}{sfx}', start=st, plan_kind='docked',
                    offset=cases[i], vblocks=False, believed_base=bb, codex=False, seed=a.seed + i, base_pose_source=a.base_pose)
        return
    if a.estop:
        return run_one(a, f'estop{sfx}', estop=dict(estop_phase='carry_across', delay=.5, hold=3., ack_after=1.5, action='resume'),
                       seed=a.seed, base_pose_source=a.base_pose)
    if a.panel_door_open:
        return run_one(a, f'panel_door_open{sfx}', start=SM.dock_base_pose(), plan_kind='door_open_panel', vblocks=False,
                       codex=False, seed=a.seed)
    if a.low_battery:
        return run_one(a, f'low_battery{sfx}', soc0=.22, seed=a.seed, base_pose_source=a.base_pose)
    if a.nonconductive:
        return run_one(a, f'nonconductive{sfx}', tool='finger', seed=a.seed, base_pose_source=a.base_pose)
    return run_one(a, f'mission_video{sfx}' if a.video else f'mission{sfx}', video=a.video, seed=a.seed, base_pose_source=a.base_pose)


if __name__ == '__main__':
    main()
