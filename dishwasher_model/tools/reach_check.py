#!/usr/bin/env python3
"""IK reachability check of the dishwasher interaction targets for the Panda in scene_demo.

For every target: the dishwasher is put in the stated state (door angle, basket travel), a set of
'sensible' gripper orientations is generated from the target's geometry (nominal first), and mink
differential IK (full scene model; door/basket/finger DOFs frozen) solves for the TCP pose. The
first candidate that converges with < 2 mm position error, < 1 deg orientation error, joint-limit
margin >= 2 deg and no robot-environment penetration is reported. If a candidate converges but
collides, it is re-solved with mink.CollisionAvoidanceLimit on the nearby geom pairs.

  python tools/reach_check.py                         # check the mount in scene_demo.MOUNT
  python tools/reach_check.py --mount -0.25 -0.5 45   # check another mount (x y yaw_deg)
  python tools/reach_check.py --search                # grid-search the mount, then full check
Writes tools/reach_results.json (per-target numbers + joint solutions, used by render.py).
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import dishwasher as dw  # noqa: E402
import scene_demo as sd  # noqa: E402

P = 'dw/'
HOME = np.array([0, 0, 0, -1.57079, 0, 1.57079, -.7853])
POS_TOL, ROT_TOL, MARGIN_TOL = 2e-3, 1.0, 2.0      # m, deg, deg


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot_about(axis, ang):
    R = np.empty(9)
    q = np.empty(4)
    mujoco.mju_axisAngle2Quat(q, unit(axis), ang)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


def hand_frame(approach, close):
    """Hand rotation: z_h = approach direction (towards the object), y_h = finger closing axis."""
    z = unit(approach)
    y = unit(np.asarray(close, float) - np.dot(close, z) * z)
    return np.column_stack([np.cross(y, z), y, z])


# ------------------------------------------------------------------ targets
def bar_candidates(u, a0, angles=(0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90), psis=(0, 20, -20, 40, -40, 60, -60)):
    """Bar grasp: fingers straddle a bar along u (closing axis perpendicular to the bar).
    phi rotates the approach about the bar axis; psi then tilts it about the closing axis
    (approach partly along the bar, e.g. from below-and-beside). Nominal (phi=psi=0) first."""
    out = []
    for psi in psis:
        for phi in angles:
            a = rot_about(u, math.radians(phi)) @ a0
            c = np.cross(u, a)
            a = rot_about(c, math.radians(psi)) @ a
            for flip in (1, -1):
                out.append((f'bar phi={phi:+d} psi={psi:+d} flip={flip:+d}', hand_frame(a, flip * c)))
    return out


def down_candidates(down, closes, tilt_axes, tilts=(0, 15, -15, 30, -30, 45, -45)):
    out = []
    for t in tilts:
        for ax_name, ax in tilt_axes:
            if t == 0 and ax_name != tilt_axes[0][0]:
                continue
            a = rot_about(ax, math.radians(t)) @ down
            for cname, c in closes:
                for flip in (1, -1):
                    out.append((f'tilt {t:+d} about {ax_name}, close {cname} flip={flip:+d}', hand_frame(a, flip * c)))
    return out


def targets(model, data):
    """Each: (name, door_deg, basket, site, tcp_offset_fn, finger_q, candidates_fn, note)."""
    def frame(name):
        return data.site(P + name).xpos.copy(), data.site(P + name).xmat.reshape(3, 3).copy()

    def base_axes():
        R = data.body(P + 'base').xmat.reshape(3, 3)
        return R[:, 0], R[:, 1], R[:, 2]

    def door_axes():
        R = data.body(P + 'door').xmat.reshape(3, 3)
        return R[:, 0], R[:, 1], R[:, 2]

    def handle():
        p, _ = frame('door_handle')
        dx, dy, dz = door_axes()
        return p, bar_candidates(dy, -dx)

    def groove():
        p, _ = frame('door_groove')
        dx, dy, dz = door_axes()
        n_in = -dx                                   # inner-face normal (up when the door is open)
        bx, by, bz = base_axes()
        return p + n_in * .018, down_candidates(-n_in, [('door-y', dy), ('door-z', dz)],
                                                 [('door-y', dy), ('door-z', dz)])

    def bhandle():
        p, _ = frame('basket_handle')
        bx, by, bz = base_axes()
        return p, bar_candidates(by, -bx)

    def plate():
        p, _ = frame('plate_slot_target')
        bx, by, bz = base_axes()
        return p, down_candidates(-bz, [('x (plate normal)', bx)], [('x', bx)], tilts=(0, 15, -15, 30, -30))

    def bowl():
        p, _ = frame('bowl_slot_target')
        bx, by, bz = base_axes()
        return p, down_candidates(-bz, [('x', bx), ('y', by)], [('x', bx), ('y', by)], tilts=(0, 15, -15, 30, -30))

    def button(nm):
        def f():
            p, R = frame(f'btn_{nm}')
            n = R[:, 0]
            bx, by, bz = base_axes()
            p = p + n * (dw.BTN_HALF[0] + .0095)      # fingertip end (TCP + 8.9 mm) ~0.6 mm off the pad
            out = []
            for beta, gam in [(0, 0)] + [(b_, g_) for b_ in (15, 30, 45, 60) for g_ in (0, -20, 20, -40, 40)] + \
                    [(0, g_) for g_ in (-20, 20, -40, 40)]:
                # beta: approach from above-front (pitch); gam: approach from the side (yaw about z)
                a = rot_about(bz, math.radians(gam)) @ rot_about(by, -math.radians(beta)) @ (-n)
                for cname, c in (('vertical', bz), ('horizontal', by)):
                    for flip in (1, -1):
                        out.append((f'press pitch {beta} yaw {gam} close {cname} flip={flip:+d}', hand_frame(a, flip * c)))
            return p, out
        return f

    return [
        ('door_handle (door closed)', 0, 0, handle, .012, 'bar grasp, fingers straddle the pull bar'),
        ('door_handle (door open 90)', 90, 0, handle, .012, 'bar grasp under the open door'),
        ('door_groove (door open 90)', 90, 0, groove, .0125, 'pod release pose: TCP 18 mm above groove centre (12 mm above face)'),
        ('basket_handle (basket in, door open)', 90, 0, bhandle, .008, 'bar grasp'),
        (f'basket_handle (basket out {dw.BASKET_TRAVEL} m)', 90, dw.BASKET_TRAVEL, bhandle, .008, 'bar grasp'),
        ('plate_slot_target (basket out)', 90, dw.BASKET_TRAVEL, plate, .01, 'TCP at seated-plate centre, from above'),
        ('bowl_slot_target (basket out)', 90, dw.BASKET_TRAVEL, bowl, .027, 'TCP at bowl foot-ring centre, from above'),
        ('btn_quick (door closed)', 0, 0, button('quick'), 0., 'closed fingertips press the pad'),
        ('btn_power (door closed)', 0, 0, button('power'), 0., 'closed fingertips press the pad'),
    ]


# ------------------------------------------------------------------ IK machinery
class Checker:
    def __init__(self, mount):
        self.mount = mount
        self.nseeds = 11
        self.model, _ = sd.build_scene(mount)
        m = self.model
        self.data = mujoco.MjData(m)
        self.arm = [m.joint(f'panda/joint{i}') for i in range(1, 8)]
        self.fing = [m.joint(f'panda/finger_joint{i}') for i in (1, 2)]
        self.qa = np.array([j.qposadr[0] for j in self.arm])
        self.qf = np.array([j.qposadr[0] for j in self.fing])
        self.lo = m.jnt_range[[j.id for j in self.arm], 0]
        self.hi = m.jnt_range[[j.id for j in self.arm], 1]
        robot_dofs = {int(j.dofadr[0]) for j in self.arm}
        self.freeze = mink.DofFreezingTask(m, [i for i in range(m.nv) if i not in robot_dofs])
        self.robot_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/')}
        coll = lambda g: m.geom_contype[g] or m.geom_conaffinity[g]
        self.g_robot = [g for g in range(m.ngeom) if m.geom_bodyid[g] in self.robot_bodies and coll(g)]
        self.g_env = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in self.robot_bodies and coll(g)]
        self.g_link0 = {g for g in self.g_robot if m.body(m.geom_bodyid[g]).name == 'panda/link0'}
        self.g_counter = m.geom('counter_top').id
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        self.posture = mink.PostureTask(m, cost=1e-3)

    def set_env(self, door_deg, basket, fingers):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[self.qa] = HOME
        d.qpos[self.qf] = fingers
        dw.set_state(self.model, d, door_deg, basket, P)

    def seeds(self, p):
        base = self.data.site('robot_mount')
        rel = base.xmat.reshape(3, 3).T @ (p - base.xpos)
        yaw = math.atan2(rel[1], rel[0])
        s = [HOME.copy()]
        for q1, q2, q4, q6 in ((yaw, 0, -1.57, 1.57), (yaw, .3, -2.3, 2.2), (yaw, .6, -0.9, 1.2), (yaw, 1.0, -1.2, 3.0),
                               (yaw - .6, 1.2, -1.0, 3.2), (yaw + .6, 1.2, -1.0, 3.2)):
            q = HOME.copy()
            q[0], q[1], q[3], q[5] = q1, q2, q4, q6
            s.append(np.clip(q, self.lo + .05, self.hi - .05))
        rng = np.random.default_rng(20261010)
        s += [rng.uniform(self.lo + .1, self.hi - .1) for _ in range(4)]
        return s

    def solve(self, q0, p, R, limits_extra=(), iters=300):
        m = self.model
        cfg = mink.Configuration(m, q=self.data.qpos.copy())
        cfg.update(self.data.qpos.copy())
        q = cfg.q.copy()
        q[self.qa] = q0
        cfg.update(q)
        self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
        self.posture.set_target(q)
        limits = [mink.ConfigurationLimit(m, gain=.5)] + list(limits_extra)
        for _ in range(iters):
            v = mink.solve_ik(cfg, [self.task, self.posture], 1.0, solver='daqp', damping=1e-4,
                              limits=limits, constraints=[self.freeze])
            cfg.integrate_inplace(v, 1.0)
            e = self.task.compute_error(cfg)
            if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                break
        return cfg.q.copy()

    def evaluate(self, q, p, R):
        m, d = self.model, self.data
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        s = d.site('tcp')
        perr = float(np.linalg.norm(s.xpos - p))
        Re = s.xmat.reshape(3, 3).T @ R
        rerr = math.degrees(math.acos(np.clip((np.trace(Re) - 1) / 2, -1, 1)))
        qa = q[self.qa]
        marg = np.minimum(qa - self.lo, self.hi - qa)
        jm = int(np.argmin(marg))
        cl, pair, hits = self.clearance()
        return dict(pos_err_mm=perr * 1e3, rot_err_deg=rerr, margin_deg=math.degrees(marg[jm]),
                    margin_joint=f'joint{jm + 1}', collisions=hits, min_clearance_mm=cl * 1e3, min_clearance_pair=pair)

    def gname(self, g):
        m = self.model
        return m.geom(g).name or m.body(m.geom_bodyid[g]).name

    def clearance(self, cap=.05):
        """Signed min distance robot <-> environment (mj_geomDistance; also catches the static
        base link0, whose overlaps produce no contacts). link0 vs counter_top is the mount."""
        m, d = self.model, self.data
        best, pair, hits = cap, None, []
        for a in self.g_robot:
            for b in self.g_env:
                if a in self.g_link0 and b == self.g_counter:
                    continue
                if np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b]) > m.geom_rbound[a] + m.geom_rbound[b] + cap:
                    continue
                dist = mujoco.mj_geomDistance(m, d, a, b, cap, None)
                if dist < 0:
                    hits.append((self.gname(a), self.gname(b), round(-dist * 1e3, 2)))
                if dist < best:
                    best, pair = dist, (self.gname(a), self.gname(b))
        return best, pair, sorted(hits, key=lambda h: -h[2])

    def near_pairs(self, cap=.08):
        m, d = self.model, self.data
        ra, eb = set(), set()
        for a in self.g_robot:
            for b in self.g_env:
                if np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b]) < m.geom_rbound[a] + m.geom_rbound[b] + cap:
                    ra.add(a)
                    eb.add(b)
        return sorted(ra), sorted(eb)

    def ok(self, ev):
        return (ev['pos_err_mm'] < POS_TOL * 1e3 and ev['rot_err_deg'] < ROT_TOL and ev['margin_deg'] >= MARGIN_TOL
                and not ev['collisions'])

    def check_target(self, tgt, quick=False):
        name, door, basket, fn, fq, note = tgt
        self.set_env(door, basket, fq)
        p, cands = fn()
        q_env = self.data.qpos.copy()
        best = None
        if quick:
            cands = cands[:12]
        for cname, R in cands:
            for si, seed in enumerate(self.seeds(p)[: (2 if quick else self.nseeds)]):
                self.data.qpos[:] = q_env
                q = self.solve(seed, p, R)
                ev = self.evaluate(q, p, R)
                if ev['pos_err_mm'] < POS_TOL * 1e3 and ev['rot_err_deg'] < ROT_TOL and ev['collisions'] and not quick:
                    ra, eb = self.near_pairs()
                    lim = mink.CollisionAvoidanceLimit(self.model, [(ra, eb)], minimum_distance_from_collisions=.003,
                                                       collision_detection_distance=.03)
                    self.data.qpos[:] = q_env
                    q2 = self.solve(q[self.qa], p, R, [lim], iters=400)
                    ev2 = self.evaluate(q2, p, R)
                    if self.ok(ev2) or (len(ev2['collisions']) < len(ev['collisions'])):
                        q, ev = q2, ev2
                        ev['note_collision_limit'] = True
                score = (self.ok(ev), ev['pos_err_mm'] < POS_TOL * 1e3, -len(ev['collisions']), ev['margin_deg'], -ev['pos_err_mm'])
                if best is None or score > best[0]:
                    best = (score, cname, si, q.copy(), ev)
                if self.ok(ev):
                    break
            if best and self.ok(best[4]):
                break
        score, cname, si, q, ev = best
        self.data.qpos[:] = q
        mujoco.mj_forward(self.model, self.data)
        ev.update(target=name, door_deg=door, basket_m=basket, orientation=cname, seed=si, note=note,
                  passed=self.ok(ev), q_arm=q[self.qa].tolist(), q_fingers=float(q[self.qf][0]),
                  tcp_target=p.tolist())
        return ev


def run(mount, quick=False, verbose=True):
    ch = Checker(mount)
    rows = []
    for tgt in targets(ch.model, ch.data):
        t0 = time.time()
        ev = ch.check_target(tgt, quick=quick)
        ev['time_s'] = round(time.time() - t0, 2)
        rows.append(ev)
        if verbose:
            col = f"{len(ev['collisions'])} {ev['collisions'][:2]}" if ev['collisions'] else 'none'
            cl = f", clearance {ev['min_clearance_mm']:.1f} mm {ev.get('min_clearance_pair')}" if 'min_clearance_mm' in ev else ''
            print(f"  {'PASS' if ev['passed'] else 'FAIL'} {ev['target']:42s} err {ev['pos_err_mm']:7.3f} mm / {ev['rot_err_deg']:5.2f} deg, "
                  f"margin {ev['margin_deg']:6.1f} deg ({ev['margin_joint']}), collisions {col}{cl}  [{ev['orientation']}]", flush=True)
    return rows


def _score_mount(mount):
    rows = run(mount, verbose=False)
    n = sum(r['passed'] for r in rows)
    worst_margin = min(r['margin_deg'] for r in rows)
    worst_clear = min(r['min_clearance_mm'] for r in rows)
    return (n, round(min(worst_margin / 10, worst_clear), 3)), mount, [r['passed'] for r in rows], worst_margin, worst_clear


def search():
    """Full-mode check on a grid of mounts on the counter beside the dishwasher (robot side -y).
    Score: number of passing targets, then min(worst joint margin [deg]/10, worst clearance [mm])."""
    from multiprocessing import Pool
    grid = list(itertools.product((-0.08, -0.12, -0.16, -0.22), (-0.38, -0.44, -0.50), (0., 45.)))
    with Pool(6) as pool:
        res = pool.map(_score_mount, grid)
    for sc, mount, passed, wm, wc in sorted(res, key=lambda r: r[0], reverse=True):
        print(f'  mount x={mount[0]:+.2f} y={mount[1]:+.2f} yaw={mount[2]:4.0f}: {sc[0]}/9 pass, worst margin {wm:5.1f} deg, '
              f'worst clearance {wc:6.2f} mm  {"".join("1" if p else "0" for p in passed)}', flush=True)
    res.sort(key=lambda r: r[0], reverse=True)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mount', nargs=3, type=float, default=None, metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--search', action='store_true')
    args = ap.parse_args()
    mount = tuple(args.mount) if args.mount else sd.MOUNT
    out = {}
    if args.search:
        res = search()
        out['search_top'] = [dict(score=r[0], mount=r[1], passed=r[2]) for r in res[:10]]
        mount = res[0][1]
        print('best quick mount:', mount)
    print(f'full check, mount x={mount[0]:+.3f} y={mount[1]:+.3f} yaw={mount[2]:.1f} deg (base on counter top z={sd.COUNTER_Z})')
    rows = run(mount)
    n = sum(r['passed'] for r in rows)
    print(f'{n}/{len(rows)} targets pass (pos < {POS_TOL*1e3:.0f} mm, rot < {ROT_TOL} deg, joint margin >= {MARGIN_TOL} deg, no penetration)')
    out.update(mount=mount, criteria=dict(pos_mm=POS_TOL * 1e3, rot_deg=ROT_TOL, margin_deg=MARGIN_TOL), results=rows,
               passed=n, total=len(rows))
    (HERE / 'reach_results.json').write_text(json.dumps(out, indent=1, default=float))


if __name__ == '__main__':
    main()
