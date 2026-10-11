#!/usr/bin/env python3
"""Plain-assert tests for the countertop dishwasher model. Prints numbers; writes test_results.json.

Run: .venv/bin/python tests.py
Physics advances only through mj_step. Joint qpos is written only at test resets.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np

import dishwasher as dw

ROOT = Path(__file__).resolve().parent
P = 'dw/'
RES = {}
OBJ = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])


def build():
    spec = mujoco.MjSpec.from_string(f'''<mujoco model="dw_tests"><compiler angle="radian"/>{dw.SCENE_OPTIONS}
      <worldbody><geom name="floor" type="plane" size="2 2 .1"/></worldbody></mujoco>''')
    info = dw.add_dishwasher(spec, pos=(0, 0, 0), yaw=0.0, prefix=P)
    # test probe: 12 mm cube, 20 g, parked on the floor away from the machine
    tb = spec.worldbody.add_body(name='probe', pos=[1, 1, .007])
    tb.add_freejoint(name='probe_free')
    tb.add_geom(name='probe_geom', type=mujoco.mjtGeom.mjGEOM_BOX, size=[.006] * 3, mass=.02, rgba=[.9, .2, .2, 1], **OBJ)
    return spec.compile(), info


def bodies_of(model, root):
    """Body ids in the subtree of `root` (excluding root itself)."""
    rid = model.body(root).id
    out = []
    for b in range(model.nbody):
        p = b
        while p not in (0, rid):
            p = model.body_parentid[p]
        if p == rid and b != rid:
            out.append(b)
    return out


def geoms_of(model, body_ids, collide_only=True):
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] in body_ids
            and (not collide_only or model.geom_contype[g] or model.geom_conaffinity[g])]


def min_dist(model, data, ga, gb, cap=.05):
    best, pair = cap, None
    for a in ga:
        for b in gb:
            d = mujoco.mj_geomDistance(model, data, a, b, cap, None)
            if d < best:
                best, pair = d, (model.geom(a).name, model.geom(b).name)
    return best, pair


def contacts(model, data):
    pen, rows = 0., []
    for i in range(data.ncon):
        c = data.contact[i]
        pen = max(pen, -c.dist)
        rows.append((model.geom(c.geom1).name, model.geom(c.geom2).name, round(float(c.dist) * 1e3, 3)))
    return pen, rows


def reset(model, data, door_deg=0., basket=0.):
    mujoco.mj_resetData(model, data)
    dw.set_state(model, data, door_deg, basket, P)


def main():
    model, info = build()
    data = mujoco.MjData(model)
    jd, jb = model.joint(P + 'door_hinge'), model.joint(P + 'basket_slide')
    qd, qb = jd.qposadr[0], jb.qposadr[0]
    dd = jd.dofadr[0]
    bid_basket, bid_door, bid_base = model.body(P + 'basket').id, model.body(P + 'door').id, model.body(P + 'base').id

    # ---------------------------------------------------------------- 1. compile
    print(f'[1] compile ok: nbody={model.nbody} ngeom={model.ngeom} njnt={model.njnt} nsensor={model.nsensor}')
    RES['compile'] = dict(nbody=model.nbody, ngeom=model.ngeom, njnt=model.njnt, nsensor=model.nsensor)

    # ---------------------------------------------------------------- 2. mass
    sub = model.body_subtreemass
    br = {n: float(model.body_mass[model.body(P + n).id]) for n in ('door', 'basket', 'upper_tray', 'spray_arm')}
    br['buttons'] = float(sum(model.body_mass[model.body(P + f'btn_{b}_body').id] for b in dw.BUTTONS))
    br['shell+machinery (static base)'] = float(model.body_mass[bid_base])
    br['machinery (hidden)'] = float(model.geom_bodyid.size and model.geom(P + 'machinery').id >= 0 and
                                     dw.MASS_TOTAL - dw.SHELL_MASS - sum(dw.DOOR_MASS.values()) - dw.BASKET_MASS
                                     - dw.TRAY_MASS - .09 - .002 * len(dw.BUTTONS))
    total = float(sub[bid_base])
    print(f'[2] mass total {total:.3f} kg (listing 13 kg); breakdown {json.dumps({k: round(v, 3) for k, v in br.items()})}')
    assert abs(total - dw.MASS_TOTAL) < 1e-6
    RES['mass'] = dict(total=total, breakdown=br, door_com_local=info['door_com'],
                       door_spring_k=info['door_spring_k'], door_spring_ref_deg=math.degrees(info['door_spring_ref']))

    # ---------------------------------------------------------------- 3. static clearances (geometry)
    moving = {'door': [bid_door], 'basket': [bid_basket]}
    others = [b for b in bodies_of(model, P + 'base') if b not in (bid_door, bid_basket)] + [bid_base]
    g_door, g_basket = geoms_of(model, [bid_door]), geoms_of(model, [bid_basket])
    g_fixed = geoms_of(model, others)
    reset(model, data, 0, 0)
    c1 = min_dist(model, data, g_door, g_fixed + g_basket)
    c2 = min_dist(model, data, g_basket, g_fixed)
    print(f'[3] closed+in clearances: door-vs-rest {c1[0]*1e3:.2f} mm {c1[1]}, basket-vs-fixed {c2[0]*1e3:.2f} mm {c2[1]}')
    assert c1[0] > 0 and c2[0] > 0
    sweep_door = []
    for a in np.arange(0, 90.1, 5):
        reset(model, data, a, 0)
        sweep_door.append(min_dist(model, data, g_door, g_fixed + g_basket)[0])
    sweep_basket = []
    for s in np.linspace(0, dw.BASKET_TRAVEL, 11):
        reset(model, data, 90, s)
        sweep_basket.append(min_dist(model, data, g_basket, g_fixed + g_door)[0])
    print(f'[3] door swing 0..90 deg (basket in): min clearance {min(sweep_door)*1e3:.2f} mm; '
          f'basket slide 0..{dw.BASKET_TRAVEL} m (door open): min clearance {min(sweep_basket)*1e3:.2f} mm')
    assert min(sweep_door) > 0 and min(sweep_basket) > 0
    RES['clearance_mm'] = dict(closed_door_vs_rest=c1[0] * 1e3, closed_door_pair=c1[1], basket_in_vs_fixed=c2[0] * 1e3,
                               basket_pair=c2[1], door_swing_min=min(sweep_door) * 1e3, basket_slide_min=min(sweep_basket) * 1e3)

    # ---------------------------------------------------------------- 4. rest: door closed, basket in
    reset(model, data, 0, 0)
    pen, maxd, maxb, ncon = 0., 0., 0., 0
    for _ in range(int(2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        p, rows = contacts(model, data)
        pen = max(pen, p)
        ncon = max(ncon, sum(1 for r in rows if 'probe' not in r[0] + r[1] and 'floor' not in r[0] + r[1]))
        maxd = max(maxd, abs(data.qpos[qd]))
        maxb = max(maxb, abs(data.qpos[qb]))
    closed = dw.door_closed(model, data, P)
    print(f'[4] rest 2 s: dishwasher-internal contacts max {ncon}, max penetration {pen*1e3:.3f} mm, '
          f'|door| max {math.degrees(maxd):.4f} deg (closed={closed}), basket drift {maxb*1e3:.4f} mm')
    assert pen <= 1e-3 and closed and maxb < 1e-3 and ncon == 0
    RES['rest'] = dict(internal_contacts=ncon, max_pen_mm=pen * 1e3, door_max_deg=math.degrees(maxd), basket_drift_mm=maxb * 1e3)

    # ---------------------------------------------------------------- 5. door released at 45 deg
    reset(model, data, 45, 0)
    log, t90, wmax = [], None, 0.
    for k in range(int(8 / model.opt.timestep) + 1):
        if k % 50 == 0:
            log.append((round(data.time, 2), round(math.degrees(data.qpos[qd]), 2)))
        wmax = max(wmax, abs(data.qvel[dd]))
        if t90 is None and math.degrees(data.qpos[qd]) > 89.5:
            t90 = data.time
        mujoco.mj_step(model, data)
    a03 = [a for t, a in log if abs(t - .3) < 1e-6][0]
    print('[5] door released at 45 deg, angle vs time (t s, deg):')
    print('    ' + ', '.join(f'{t:.1f}:{a:.1f}' for t, a in log if abs(t * 10 - round(t * 10)) < 1e-6 and (t <= 1 or abs(t - round(t)) < 1e-6)))
    print(f'    angle at 0.3 s = {a03:.2f} deg; reaches 89.5 deg at t = {t90} s; peak speed {wmax:.3f} rad/s '
          f'({math.degrees(wmax):.1f} deg/s); final {log[-1][1]:.2f} deg')
    assert a03 < 50 and (t90 is None or t90 > 1.0) and wmax < 1.0
    RES['door_release_45'] = dict(angle_at_0p3s=a03, t_reach_89p5=t90, peak_speed_rad_s=wmax, final_deg=log[-1][1], log=log)

    # ---------------------------------------------------------------- 6. door rests on its 90 deg stop
    reset(model, data, 90, 0)
    for _ in range(int(2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    ang = math.degrees(data.qpos[qd])
    lim = [i for i in range(data.nefc) if data.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and data.efc_id[i] == jd.id]
    stop_tau = float(sum(data.efc_force[i] for i in lim))
    pen, rows = contacts(model, data)
    print(f'[6] door at 90 deg after 2 s: {ang:.3f} deg, open-stop limit active={bool(lim)}, stop reaction {stop_tau:.3f} N m, '
          f'door contacts {[r for r in rows if "door" in r[0] + r[1]]}')
    assert 89.5 <= ang <= 90.5 and lim and stop_tau > 0
    RES['door_open_stop'] = dict(angle_deg=ang, limit_active=bool(lim), stop_reaction_Nm=stop_tau)
    # also: from 80 deg it should end on the stop
    reset(model, data, 80, 0)
    for _ in range(int(4 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    a80 = math.degrees(data.qpos[qd])
    print(f'    released at 80 deg -> {a80:.2f} deg after 4 s')
    assert a80 > 89.5
    RES['door_open_stop']['from_80_after_4s'] = a80

    # ---------------------------------------------------------------- 7. basket out / in with a small force
    F = 2.0
    reset(model, data, 90, 0)
    axis = data.xaxis[jb.id].copy()
    out = dict(force_N=F)
    for phase, sgn, done in (('out', 1, lambda q: q >= dw.BASKET_TRAVEL - 1e-3), ('in', -1, lambda q: q <= 1e-3)):
        t0, vmax, pen, tdone, bad = data.time, 0., 0., None, []
        while data.time - t0 < 4:
            data.xfrc_applied[bid_basket, :3] = sgn * F * axis
            mujoco.mj_step(model, data)
            vmax = max(vmax, abs(data.qvel[jb.dofadr[0]]))
            p, rows = contacts(model, data)
            pen = max(pen, p)
            bad += [r for r in rows if 'basket' in r[0] + r[1] or any(n in r[0] + r[1] for n in ('bfloor', 'tine', 'frame', 'roller'))]
            if tdone is None and done(data.qpos[qb]):
                tdone = data.time - t0
            if tdone is not None and data.time - t0 > tdone + .5:
                break
        data.xfrc_applied[bid_basket] = 0
        hold = data.qpos[qb]
        for _ in range(250):
            mujoco.mj_step(model, data)
        out[phase] = dict(t_to_limit_s=tdone, peak_speed_m_s=vmax, final_pos_m=float(data.qpos[qb]),
                          pos_0p5s_after_release=float(hold), basket_contacts=len(bad), max_pen_mm=pen * 1e3)
        print(f'[7] basket {phase} with {F} N: reached limit in {tdone} s, peak {vmax:.3f} m/s, pos {data.qpos[qb]:.4f} m, '
              f'basket contacts {len(bad)}, max pen {pen*1e3:.3f} mm, door {math.degrees(data.qpos[qd]):.2f} deg')
        assert tdone is not None and not bad
    assert abs(data.qpos[qb]) < 2e-3
    RES['basket_slide'] = out
    # stand-still check: with the door open and no force, the basket stays where it is
    reset(model, data, 90, .12)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    print(f'    basket left at 0.120 m with no force -> {data.qpos[qb]:.4f} m after 2 s')
    RES['basket_slide']['standstill_0p12'] = float(data.qpos[qb])

    # ---------------------------------------------------------------- 8. touch sensors
    pj = model.joint('probe_free').qposadr[0]
    m_probe = float(model.body_mass[model.body('probe').id])
    touch = {}
    g0 = model.opt.gravity.copy()
    for nm in dw.BUTTONS:
        reset(model, data, 0, 0)
        site = data.site(P + f'btn_{nm}')
        n = site.xmat.reshape(3, 3)[:, 0]          # outward normal of the band face
        data.qpos[pj:pj + 3] = site.xpos + n * (dw.BTN_HALF[0] + .006 + .004)   # 4 mm in front of the pad
        data.qpos[pj + 3:pj + 7] = [1, 0, 0, 0]
        model.opt.gravity[:] = -9.81 * n             # "drop" the probe onto the vertical pad
        mujoco.mj_forward(model, data)
        before = {b: float(data.sensor(P + f'btn_{b}_touch').data[0]) for b in dw.BUTTONS}
        vals = []
        for k in range(300):
            mujoco.mj_step(model, data)
            if k >= 200:
                vals.append({b: float(data.sensor(P + f'btn_{b}_touch').data[0]) for b in dw.BUTTONS})
        own = float(np.mean([v[nm] for v in vals]))
        cross = max(max(v[b] for v in vals) for b in dw.BUTTONS if b != nm)
        touch[nm] = dict(before=before[nm], reading_N=own, expected_N=m_probe * 9.81, max_other_N=cross)
        print(f'[8] btn_{nm}: before {before[nm]:.3f} N, after drop {own:.4f} N (m g = {m_probe*9.81:.4f} N), other pads max {cross:.4f} N')
        assert before[nm] == 0 and own > 0 and cross == 0
    model.opt.gravity[:] = g0
    RES['touch'] = touch

    (ROOT / 'test_results.json').write_text(json.dumps(RES, indent=1, default=float))
    print('ALL TESTS PASSED -> test_results.json')


if __name__ == '__main__':
    main()
