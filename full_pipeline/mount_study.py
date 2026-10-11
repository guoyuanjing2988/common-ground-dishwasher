#!/usr/bin/env python3
"""Mount study: reach + collision check of every pose / path the task needs, per robot mount option.

Method (extends ../dishwasher_model/tools/reach_check.py): mink IK on the FULL scene model
(door / basket / object DOFs frozen with DofFreezingTask, door + basket set to the state the step
needs); the TCP pose must reach < 2 mm / < 1 deg with >= 2 deg joint-limit margin and no
penetration between robot (+ plate in hand, where carried) and anything else, checked with
mj_geomDistance over all pairs (also catches static-static pairs). Paths (door arc, basket pull /
push, flat pick, carry + insert) are solved waypoint by waypoint from the previous solution; a
joint jump > 0.6 rad between waypoints counts as a failure (re-configuration mid-path).

  python mount_study.py --grid          # M1 grid search (parallel), then M2 / M3 comparisons
  python mount_study.py --one M1 0.45 0.30 0.90 195
Writes out/mount_study.json (used for MOUNTING.md).
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import mink
import mujoco
import numpy as np

import dishwasher as dw
import plan
import scene as S
from util import angle_deg, hand_frame, rot_z

ROOT = Path(__file__).resolve().parent
POS_TOL, ROT_TOL, MARGIN_TOL, JUMP_TOL = 2e-3, 1.0, 2.0, 0.6
HOME = np.array([0, -.3, 0, -2.2, 0, 2.0, .785])


class Checker:
    def __init__(self, mount: S.Mount, layout: S.Layout = S.DEFAULT_LAYOUT):
        self.mount = mount
        self.model, _ = S.build(mount, layout=layout)
        m = self.model
        self.data = mujoco.MjData(m)
        self.arm = [m.joint(f'panda/joint{i}') for i in range(1, 8)]
        self.qa = np.array([j.qposadr[0] for j in self.arm])
        self.qf = np.array([m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)])
        self.lo = m.jnt_range[[j.id for j in self.arm], 0]
        self.hi = m.jnt_range[[j.id for j in self.arm], 1]
        dofs = {int(j.dofadr[0]) for j in self.arm}
        self.rail = None
        if mount.kind == 'M2':
            self.rail = m.joint('rail_slide')
            dofs.add(int(self.rail.dofadr[0]))
        self.freeze = mink.DofFreezingTask(m, [i for i in range(m.nv) if i not in dofs])
        robot_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/')}
        coll = lambda g: m.geom_contype[g] or m.geom_conaffinity[g]
        self.pid = m.body('plate').id
        self.g_robot = [g for g in range(m.ngeom) if m.geom_bodyid[g] in robot_bodies and coll(g)]
        self.g_plate = [g for g in range(m.ngeom) if m.geom_bodyid[g] == self.pid and coll(g)]
        self.g_pod = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body('pod').id]
        self.g_env = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in robot_bodies and coll(g)
                      and m.geom_bodyid[g] not in (self.pid, m.body('pod').id)]
        self.g_link0 = {g for g in self.g_robot if m.body(m.geom_bodyid[g]).name == 'panda/link0'}
        self.mount_geoms = {g for g in self.g_env if m.geom(g).name in ('counter_top', 'column', 'column_plate', 'rail')}
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        self.posture = mink.PostureTask(m, cost=1e-3)
        pq = m.joint('plate_free').qposadr[0]
        self.plate_q = slice(pq, pq + 7)

    # -------------------------------------------------------------- state
    def set_env(self, door_deg, basket, finger_q, q_arm=None, plate_pose=None):
        d = self.data
        if q_arm is None:
            mujoco.mj_resetData(self.model, d)
            d.qpos[self.qa] = HOME
        else:
            d.qpos[self.qa] = q_arm
        d.qpos[self.qf] = finger_q
        d.qpos[self.model.joint('dw/door_hinge').qposadr[0]] = math.radians(door_deg)
        d.qpos[self.model.joint('dw/basket_slide').qposadr[0]] = basket
        if plate_pose is not None:
            d.qpos[self.plate_q] = np.r_[plate_pose[1], S.quat(plate_pose[0])]
        else:
            d.qpos[self.plate_q] = np.r_[0, 0, -1, 1, 0, 0, 0]      # park the plate out of the way
        mujoco.mj_forward(self.model, d)

    def seeds(self, p):
        base = self.data.site('robot_mount')
        rel = base.xmat.reshape(3, 3).T @ (p - base.xpos)
        yaw = math.atan2(rel[1], rel[0])
        s = [HOME.copy()]
        for q2, q4, q6 in ((-.3, -2.2, 2.0), (.3, -2.3, 2.2), (.6, -1.6, 2.0), (.2, -1.2, 1.4), (1.0, -1.2, 3.0), (-.6, -2.6, 1.6)):
            q = HOME.copy()
            q[0], q[1], q[3], q[5] = yaw, q2, q4, q6
            s.append(np.clip(q, self.lo + .05, self.hi - .05))
        rng = np.random.default_rng(20261010)
        s += [rng.uniform(self.lo + .1, self.hi - .1) for _ in range(5)]
        return s

    def solve(self, q0, p, R, iters=300, posture_target=None):
        m = self.model
        q = self.data.qpos.copy()
        q[self.qa] = q0
        cfg = mink.Configuration(m, q=q)
        self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
        self.posture.set_target(q if posture_target is None else posture_target)
        limits = [mink.ConfigurationLimit(m, gain=.5)]
        for _ in range(iters):
            v = mink.solve_ik(cfg, [self.task, self.posture], 1.0, solver='daqp', damping=1e-4, limits=limits,
                              constraints=[self.freeze])
            cfg.integrate_inplace(v, 1.0)
            e = self.task.compute_error(cfg)
            if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                break
        return cfg.q.copy()

    def gname(self, g):
        m = self.model
        return m.geom(g).name or m.body(m.geom_bodyid[g]).name

    def clearance(self, with_plate=False, with_pod=False, ignore=(), cap=.05):
        m, d = self.model, self.data
        movers = list(self.g_robot) + (self.g_plate if with_plate else []) + (self.g_pod if with_pod else [])
        best, pair, hits = cap, None, []
        for a in movers:
            for b in self.g_env:
                if a in self.g_link0 and b in self.mount_geoms:
                    continue
                if self.gname(b) in ignore:
                    continue
                if np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b]) > m.geom_rbound[a] + m.geom_rbound[b] + cap:
                    continue
                dist = mujoco.mj_geomDistance(m, d, a, b, cap, None)
                if dist < 0:
                    hits.append((self.gname(a), self.gname(b), round(-dist * 1e3, 2)))
                if dist < best:
                    best, pair = dist, (self.gname(a), self.gname(b))
        # plate vs robot (other than fingers) when carried
        return best, pair, sorted(hits, key=lambda h: -h[2])

    def evaluate(self, q, p, R, wp):
        m, d = self.model, self.data
        d.qpos[:] = q
        if wp.get('plate_rel') is not None:          # carried plate follows the hand
            mujoco.mj_kinematics(m, d)
            Rh, ph = d.site('tcp').xmat.reshape(3, 3), d.site('tcp').xpos
            Rhp, php = wp['plate_rel']                # hand in plate frame
            Rp = Rh @ Rhp.T
            d.qpos[self.plate_q] = np.r_[ph - Rp @ php, S.quat(Rp)]
        mujoco.mj_forward(m, d)
        s = d.site('tcp')
        perr = float(np.linalg.norm(s.xpos - p))
        rerr = angle_deg(s.xmat.reshape(3, 3).T @ R)
        qa = q[self.qa]
        marg = np.minimum(qa - self.lo, self.hi - qa)
        jm = int(np.argmin(marg))
        cl, pair, hits = self.clearance(with_plate=wp.get('plate_rel') is not None, ignore=wp.get('ignore', ()))
        return dict(pos_err_mm=perr * 1e3, rot_err_deg=rerr, margin_deg=math.degrees(marg[jm]), margin_joint=f'joint{jm + 1}',
                    collisions=hits, min_clearance_mm=cl * 1e3, min_clearance_pair=pair)

    @staticmethod
    def ok(ev):
        return (ev['pos_err_mm'] < POS_TOL * 1e3 and ev['rot_err_deg'] < ROT_TOL and ev['margin_deg'] >= MARGIN_TOL
                and not ev['collisions'])

    # -------------------------------------------------------------- paths
    def run_path(self, waypoints, first_seeds=None):
        """waypoints: list of dicts(name, p, R, door, basket, fq, [plate_rel], [ignore]).
        First waypoint: multi-seed; then continuation. Returns (passed, rows, q_list)."""
        rows, qs = [], []
        q_prev = None
        for i, wp in enumerate(waypoints):
            self.set_env(wp['door'], wp['basket'], wp['fq'], q_prev)
            q_env = self.data.qpos.copy()
            if q_prev is None:
                best = None
                for seed in (first_seeds or self.seeds(wp['p'])):
                    self.data.qpos[:] = q_env
                    q = self.solve(seed, wp['p'], wp['R'])
                    ev = self.evaluate(q, wp['p'], wp['R'], wp)
                    sc = (self.ok(ev), -len(ev['collisions']), ev['margin_deg'])
                    if best is None or sc > best[0]:
                        best = (sc, q, ev)
                    if self.ok(ev) and ev['margin_deg'] > 10:
                        break
                _, q, ev = best
            else:
                q = self.solve(q_prev, wp['p'], wp['R'], posture_target=self.data.qpos.copy())
                ev = self.evaluate(q, wp['p'], wp['R'], wp)
            jump = 0. if q_prev is None else float(np.max(np.abs(q[self.qa] - q_prev)))
            ev.update(name=wp['name'], jump_rad=jump, passed=self.ok(ev) and (jump < JUMP_TOL or not wp.get('cont')),
                      q_arm=q[self.qa].tolist())
            if self.rail is not None:
                ev['rail_m'] = float(q[self.rail.qposadr[0]])
            rows.append(ev)
            qs.append(q[self.qa].copy())
            q_prev = q[self.qa].copy()
            if self.rail is not None:
                self.data.qpos[self.rail.qposadr[0]] = q[self.rail.qposadr[0]]
            if not ev['passed']:
                return False, rows, qs
        return True, rows, qs


# ------------------------------------------------------------------ target / path catalogue
def catalogue(layout: S.Layout = S.DEFAULT_LAYOUT):
    """Each entry: (name, list of candidate waypoint lists). The first candidate that passes wins."""
    out = []
    DISH = np.array(layout.dish)
    # door: hook the handle (closed) and follow the arc to 90 deg; same path backwards for closing
    door_c = []
    for kappa, psi, flip in itertools.product((0., .5, 1.), (0., 20., -20., 40., -40.), (1, -1)):
        wps = []
        for th in [0] + list(range(0, 91, 10)):
            t = math.radians(th)
            R = plan.door_bar_frame(t, kappa, psi, flip)
            p = plan.handle_pos(t)
            if not wps:     # pre-grasp: 10 cm back along the approach, door closed
                wps.append(dict(name='door_pregrasp', p=p - .10 * R[:, 2], R=R, door=0, basket=0, fq=.035))
                continue
            wps.append(dict(name=f'door_arc{th}', p=p, R=R, door=th, basket=0, fq=.010, cont=th > 0))
        wps.append(dict(name='door_release', p=plan.handle_pos(math.pi / 2) - .10 * R[:, 2], R=R, door=90, basket=0, fq=.035))
        door_c.append((f'kappa {kappa} psi {psi:+.0f} flip {flip:+d}', wps))
    out.append(('door open/close arc (0-90 deg)', door_c))
    # pod: pick from the dish, place over the door groove (door open)
    pod_c = []
    for tilt in (0., 15., -15.):
        pod_p = np.array([DISH[0], DISH[1], S.COUNTER_Z + .0003 + S.DISH_FLOOR_T + S.POD_SIZE[2] / 2 - .002])
        Rpick = plan.pod_down_frame([1., 0, 0])
        g = plan.groove_center()
        Rplace = plan.pod_down_frame([0, 1., 0])
        from util import rot_axis
        Rplace = rot_axis([0, 1, 0], math.radians(tilt)) @ Rplace
        place_p = np.array([g[0], g[1], plan.groove_face_z() + .003 + .0045])
        pod_c.append((f'place tilt {tilt:+.0f}', [
            dict(name='pod_above', p=pod_p + [0, 0, .08], R=Rpick, door=90, basket=0, fq=.02),
            dict(name='pod_grasp', p=pod_p, R=Rpick, door=90, basket=0, fq=.019, ignore=('dish_floor',), cont=True),
            dict(name='pod_lift', p=pod_p + [0, 0, .10], R=Rpick, door=90, basket=0, fq=.016, cont=True),
            dict(name='groove_above', p=place_p + [0, 0, .08], R=Rplace, door=90, basket=0, fq=.016),
            dict(name='groove_release', p=place_p, R=Rplace, door=90, basket=0, fq=.019, cont=True),
            dict(name='groove_retract', p=place_p + [0, 0, .10], R=Rplace, door=90, basket=0, fq=.02)]))
    out.append(('pod: dish -> door groove', pod_c))
    # basket pull out / push in
    bas_c = []
    for pitch, psi, flip in itertools.product((0., 20., -20., 40.), (0., 20., -20.), (1, -1)):
        R = plan.basket_bar_frame(pitch, psi, flip)
        wps = [dict(name='basket_pregrasp', p=plan.basket_handle(0) - .10 * R[:, 2], R=R, door=90, basket=0, fq=.035)]
        for b in np.linspace(0, dw.BASKET_TRAVEL, 6):
            wps.append(dict(name=f'basket_{b:.2f}', p=plan.basket_handle(b), R=R, door=90, basket=b, fq=.006, cont=b > 0))
        wps.append(dict(name='basket_release', p=plan.basket_handle(dw.BASKET_TRAVEL) - .10 * R[:, 2], R=R, door=90,
                        basket=dw.BASKET_TRAVEL, fq=.035))
        bas_c.append((f'pitch {pitch:+.0f} psi {psi:+.0f} flip {flip:+d}', wps))
    out.append(('basket pull / push (0-0.25 m)', bas_c))
    # plate pick (flat: drag / pinch / tilt / lift; rack: rim pinch + lift) and, separately, carry + insert
    B = dw.BASKET_TRAVEL

    def insert_wps(Rpl, plate_rel):
        slot = plan.plate_slot()
        org = slot - Rpl @ np.array([0, 0, S.MID_Z])
        Rh = Rpl @ plate_rel[0]
        ph = org + Rpl @ plate_rel[1]
        return [dict(name='carry_high', p=ph + [0, 0, .20], R=Rh, door=90, basket=B, fq=.0026, plate_rel=plate_rel),
                dict(name='carry_above', p=ph + [0, 0, .09], R=Rh, door=90, basket=B, fq=.0026, plate_rel=plate_rel, cont=True),
                dict(name='insert', p=ph + [0, 0, .002], R=Rh, door=90, basket=B, fq=.0026, plate_rel=plate_rel, cont=True),
                dict(name='release_retract', p=ph + [0, 0, .12], R=Rh, door=90, basket=B, fq=.02, cont=True)]
    pick_c, ins_c = [], []
    if layout.plate_mode == 'flat':
        for app in (60., 45., -60., -45.):
            wp, info = plan.nominal_flat_sequence(plate_xy=layout.plate0, approach_deg=app)
            plate_rel = (info['Rh_p'], info['ph_p'])
            wps = []
            for n, p, R in wp:
                fq = 0. if n.startswith('push') else (.03 if n in ('pinch_pre', 'pinch_in') else .0026)
                d_ = dict(name=n, p=p, R=R, door=90, basket=B, fq=fq,
                          cont=n.startswith('push_drag') or n.startswith('tilt') or n in ('lift', 'pinch', 'pinch_in', 'push_start'))
                if n == 'lift':
                    d_['plate_rel'] = plate_rel
                wps.append(d_)
            pick_c.append((f'approach {app:+.0f}', wps))
            ins_c.append((f'in-hand from approach {app:+.0f}', insert_wps(info['Rpl_end'], plate_rel)))
        out.append(('flat pick: drag, pinch, tilt, lift', pick_c))
    else:
        org, P = S.rack_plate_pose(layout.rack)
        up = P[:, 1]
        for hysign in (1, -1):
            nrm = P @ np.array([0, -math.sin(S.SLOPE), math.cos(S.SLOPE)])
            hy = hysign * nrm / np.linalg.norm(nrm)
            hz = -up - (-up @ hy) * hy
            hz /= np.linalg.norm(hz)
            Rg = np.column_stack([np.cross(hy, hz), hy, hz])
            r_p = S.R_OUT - .012
            z_rim = .0075 + S.RIM_RISE * (r_p - S.R_DISC) / (S.R_OUT - S.R_DISC)
            grasp = org + P @ np.array([0, r_p, z_rim])
            plate_rel = (P.T @ Rg, P.T @ (grasp - org))
            pick_c.append((f'rack grasp hy {hysign:+d}', [
                dict(name='rack_above', p=grasp - hz * .12, R=Rg, door=90, basket=B, fq=.025),
                dict(name='rack_grasp', p=grasp, R=Rg, door=90, basket=B, fq=.0026, cont=True, ignore=tuple(f'rack_prong{a}_{b}' for a in (0, 2) for b in range(7))),
                dict(name='rack_lift', p=grasp + up * .17, R=Rg, door=90, basket=B, fq=.0026, cont=True)]))
            ins_c.append((f'in-hand from rack hy {hysign:+d}', insert_wps(P, plate_rel)))
        out.append(('rack pick: rim pinch, lift 0.17 m', pick_c))
    out.append(('carry + insert into plate slot', ins_c))
    # buttons
    for b in ('quick', 'power'):
        c = []
        for pitch, yaw, close, flip in itertools.product((0., 20., 40.), (0., -20., 20., -40., 40.), ('vertical', 'horizontal'), (1, -1)):
            R = plan.press_frame(pitch, yaw, close, flip)
            p = plan.button_pos(b) + R[:, 2] * -.0005
            c.append((f'pitch {pitch:+.0f} yaw {yaw:+.0f} {close} {flip:+d}', [
                dict(name=f'btn_{b}_pre', p=p - .05 * R[:, 2], R=R, door=0, basket=0, fq=0.),
                dict(name=f'btn_{b}', p=p, R=R, door=0, basket=0, fq=0., cont=True)]))
        out.append((f'btn_{b} press', c))
    return out


def evaluate_mount(mount: S.Mount, verbose=False, max_cands=None, layout: S.Layout = S.DEFAULT_LAYOUT):
    ch = Checker(mount, layout)
    res = []
    for name, cands in catalogue(layout):
        t0 = time.time()
        best = None
        for ci, (cname, wps) in enumerate(cands[:max_cands]):
            ok, rows, _ = ch.run_path(wps)
            score = (ok, len(rows), min(r['margin_deg'] for r in rows))
            if best is None or score > best[0]:
                best = (score, cname, rows)
            if ok:
                break
        (ok, nrows, _), cname, rows = best
        failed = None if ok else rows[-1]
        r = dict(target=name, passed=ok, choice=cname, n_waypoints=len(rows),
                 worst_pos_err_mm=max(x['pos_err_mm'] for x in rows), worst_rot_err_deg=max(x['rot_err_deg'] for x in rows),
                 min_margin_deg=min(x['margin_deg'] for x in rows), min_clearance_mm=min(x['min_clearance_mm'] for x in rows),
                 min_clearance_at=min(rows, key=lambda x: x['min_clearance_mm'])['name'],
                 max_jump_rad=max(x['jump_rad'] for x in rows),
                 fail_at=None if ok else dict(name=failed['name'], pos_err_mm=failed['pos_err_mm'], rot_err_deg=failed['rot_err_deg'],
                                               margin_deg=failed['margin_deg'], collisions=failed['collisions'][:3],
                                               jump_rad=failed['jump_rad']),
                 rows=rows, seconds=round(time.time() - t0, 1))
        if 'rail_m' in rows[0]:
            r['rail_range_m'] = [min(x['rail_m'] for x in rows), max(x['rail_m'] for x in rows)]
        res.append(r)
        if verbose:
            print(f"  {'PASS' if ok else 'FAIL'} {name:34s} [{cname}] wps {len(rows)} margin {r['min_margin_deg']:.1f} "
                  f"clear {r['min_clearance_mm']:.1f} mm @{r['min_clearance_at']} "
                  f"{'' if ok else 'FAIL@' + str(r['fail_at'])}", flush=True)
    return res


def _job(args):
    args = dict(args)
    lay = S.Layout(**args.pop('layout')) if 'layout' in args else S.DEFAULT_LAYOUT
    mount = S.Mount(**args)
    t0 = time.time()
    try:
        res = evaluate_mount(mount, layout=lay)
    except Exception as e:  # noqa: BLE001
        return dict(mount=args, error=repr(e))
    n = sum(r['passed'] for r in res)
    worst_margin = min(r['min_margin_deg'] for r in res)
    worst_clear = min(r['min_clearance_mm'] for r in res)
    return dict(mount=args, label=mount.label(), n_pass=n, n=len(res), worst_margin_deg=worst_margin,
                worst_clearance_mm=worst_clear, results=[{k: v for k, v in r.items() if k != 'rows'} for r in res],
                seconds=round(time.time() - t0, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--grid', action='store_true')
    ap.add_argument('--one', nargs=5, metavar=('KIND', 'X', 'Y', 'Z', 'YAW'))
    ap.add_argument('--rail', type=float, default=0.)
    ap.add_argument('--out', default='out/mount_study.json')
    a = ap.parse_args()
    if a.one:
        k, x, y, z, yaw = a.one
        m = S.Mount(kind=k, x=float(x), y=float(y), z=float(z), yaw=float(yaw), rail_travel=a.rail)
        print(m.label())
        res = evaluate_mount(m, verbose=True)
        print(sum(r['passed'] for r in res), '/', len(res))
        return
    if a.grid:
        from multiprocessing import Pool
        jobs = [dict(kind='M1', x=x, y=y, z=z, yaw=yaw) for x, y, z, yaw in
                itertools.product((0.40, 0.48, 0.56), (0.22, 0.32, 0.42), (0.80, 0.90, 1.00), (195.,))]
        jobs += [dict(kind='M3', x=-0.08, y=-0.50, z=0.90, yaw=45.), dict(kind='M3', x=-0.10, y=0.62, z=0.90, yaw=-135.),
                 dict(kind='M3', x=-0.10, y=0.62, z=0.90, yaw=180.),
                 dict(kind='M2', x=0.56, y=0.30, z=0.90, yaw=195., rail_travel=0.6),
                 dict(kind='M2', x=0.48, y=0.30, z=0.90, yaw=195., rail_travel=0.6)]
        with Pool(16) as pool:
            res = pool.map(_job, jobs)
        res.sort(key=lambda r: (r.get('n_pass', -1), min(r.get('worst_margin_deg', 0) / 10, r.get('worst_clearance_mm', 0))), reverse=True)
        for r in res:
            if 'error' in r:
                print('ERR', r['mount'], r['error'])
                continue
            print(f"{r['label']:44s} {r['n_pass']}/{r['n']}  margin {r['worst_margin_deg']:5.1f}  clear {r['worst_clearance_mm']:6.1f} "
                  + ' '.join('1' if x['passed'] else '0' for x in r['results']), flush=True)
        Path(a.out).write_text(json.dumps(dict(criteria=dict(pos_mm=POS_TOL * 1e3, rot_deg=ROT_TOL, margin_deg=MARGIN_TOL,
                                                             jump_rad=JUMP_TOL), grid=res), indent=1, default=float))


if __name__ == '__main__':
    main()
