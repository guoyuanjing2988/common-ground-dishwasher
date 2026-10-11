"""Frozen, harness-owned scoring + key-frame rendering.

This file lives OUTSIDE every sandbox copy, so a GPT patch cannot change how the metric is measured.
It reads only the MuJoCo model/data of a run (ground truth is used for SCORING only, never fed into control).
Constants are copied from the original (unmodified) scene.py / dishwasher.py.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

R_OUT = .1015            # scene.R_OUT (plate outer radius)
LIP_Z = .0173            # rim lip height used by the original plate_slot_state
TINE_PITCH, TINE_R, N_TINE_X = .028, .002, 9      # dishwasher.TINE_PITCH / TINE_R / len(TINE_X)


def touching(m, d, body):
    out = set()
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = int(m.geom_bodyid[c.geom[0]]), int(m.geom_bodyid[c.geom[1]])
        if body == b1:
            out.add(m.geom(c.geom[1]).name or m.body(b2).name)
        elif body == b2:
            out.add(m.geom(c.geom[0]).name or m.body(b1).name)
    return out


def plate_state(m, d):
    """Verbatim logic of the original Runner.plate_slot_state (ground truth), plus the lean (= plane tilt)."""
    pid = m.body('plate').id
    slot = d.site('dw/plate_slot_target').xpos.copy()
    c = d.site('plate_center').xpos.copy()
    err = c - slot
    R = d.xmat[pid].reshape(3, 3).copy()
    n = R[:, 2]
    plane_deg = math.degrees(math.acos(min(1., abs(float(n[0])))))
    th = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    rim = d.xpos[pid] + (np.outer(np.cos(th), R[:, 0]) + np.outer(np.sin(th), R[:, 1])) * R_OUT + R[:, 2] * LIP_Z
    low = rim[np.argmin(rim[:, 2])]
    bx = float(low[0] - slot[0])
    touch = touching(m, d, pid)
    k0, k1 = N_TINE_X - 3, N_TINE_X - 2
    own = {f'dw/plate_tine{r}_{k}' for r in (0, 1) for k in (k0, k1)}
    wrong = sorted(t for t in touch if t.startswith('dw/plate_tine') and t not in own)
    other = sorted(t for t in touch if not (t.startswith('dw/plate_tine') or t.startswith('dw/bfloor') or t.startswith('dw/frame_b')))
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, pid, vel, 0)
    half_gap = TINE_PITCH / 2 - TINE_R
    in_slot = abs(bx) < half_gap and abs(err[1]) < .03 and plane_deg < 10 and not wrong and not other
    seated = abs(bx) < half_gap and abs(err[1]) < .03 and not wrong and not other   # between its own tines, any lean
    return dict(in_slot=bool(in_slot), seated_own_tines=bool(seated), lean_deg=round(plane_deg, 2),
                bottom_offset_mm=round(bx * 1000, 1), centre_err_mm=[round(float(v) * 1000, 1) for v in err],
                wrong_tines=wrong, other_contacts=other, speed_mm_s=round(float(np.linalg.norm(vel[3:])) * 1000, 2),
                touching_robot=any(t.startswith('panda/') for t in touch), sim_t=round(float(d.time), 2))


def door_basket(m, d):
    return dict(door_deg=round(math.degrees(float(d.qpos[m.joint('dw/door_hinge').qposadr[0]])), 2),
                basket_m=round(float(d.qpos[m.joint('dw/basket_slide').qposadr[0]]), 4))


KEEP = ('tine', 'basket', 'bfloor', 'frame_b', 'wire', 'bar')


def snapshot(m, d, path, label=''):
    """Two views of the plate in its slot (edge-on along y, and a 3/4 view), dishwasher shell made see-through.
    Rendering only: geom_rgba is restored afterwards (it has no effect on physics)."""
    from PIL import Image, ImageDraw
    rgba0 = m.geom_rgba.copy()
    try:
        for g in range(m.ngeom):
            nm = m.geom(g).name or ''
            if nm.startswith('dw/') and not any(k in nm for k in KEEP):
                m.geom_rgba[g, 3] = min(m.geom_rgba[g, 3], .08)
            if nm.startswith('panda/') or nm.startswith('base') or nm.startswith('col'):
                m.geom_rgba[g, 3] = min(m.geom_rgba[g, 3], .25)
        slot = d.site('dw/plate_slot_target').xpos.copy()
        r = mujoco.Renderer(m, 360, 480)
        opt = mujoco.MjvOption()
        opt.sitegroup[:] = 0
        imgs = []
        for az, el, dist in ((90., -2., .42), (140., -18., .55)):
            cam = mujoco.MjvCamera()
            cam.lookat[:] = slot + [0, 0, .07]
            cam.distance, cam.azimuth, cam.elevation = dist, az, el
            r.update_scene(d, camera=cam, scene_option=opt)
            imgs.append(Image.fromarray(r.render()))
        r.close()
        W = Image.new('RGB', (960, 384), (20, 20, 20))
        W.paste(imgs[0], (0, 24))
        W.paste(imgs[1], (480, 24))
        dr = ImageDraw.Draw(W)
        dr.text((6, 5), label[:150], fill=(255, 230, 140))
        dr.text((6, 370), 'left: edge-on view along +y (vertical line = 0 deg lean); right: 3/4 view', fill=(200, 200, 200))
        W.save(path)
    finally:
        m.geom_rgba[:] = rgba0
    return str(path)
