#!/usr/bin/env python3
"""multi_dish_b: Franka Panda loads 3 plates + 2 bowls from the counter into the countertop dishwasher basket.

One continuous mj_step simulation. Reuses ../full_pipeline (copied here): docked cart (M2-style, longer linear
axis), mink IK, force-limited gripper, the proven FLAT plate skill (push to edge, pinch, tilt, lift, insert
between tines) and the bowl PoC foot-ring grasp (inverted bowl, placed tilted 15 deg on the leaning bowl row).
Simplifications (DEMO_NOTES.md): ground-truth object poses, door already open + basket already pulled out.

  python multi.py --no-video            # fast check
  python multi.py                       # video -> out/multi_dish_demo.mp4 (+ key frames)
  python multi.py --only plates         # plates only
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

import plan

import dishwasher as dw
import kitchen as K
import pipeline as PL
import scene as S
from util import angle_deg, hand_frame, rot_axis

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'

# ------------------------------------------------------------------ layout (all EST)
RAIL_C, RAIL_TRAVEL = 0.70, 1.10                  # linear axis along the counter front: base y 0.15 ... 1.25
BASE_INSERT = 0.36                                # base y while loading the basket (= proven M2 pick/insert setpoint)
PLATES_XY = [(-0.15, 0.40), (-0.15, 0.63), (-0.15, 0.86)]      # 3 flat plates side by side, 27 mm gaps (depth segmentation)
BOWLS_XY = [(-0.12, 1.04), (-0.12, 1.18)]        # 2 inverted bowls side by side
PLATE_YAW = [0., 0., 0.]
PLATE_SLOTS = [5, 4, 3]                           # slot k = between TINE_X[k] and TINE_X[k+1]; front slot first (seated plates lean toward +x)
BOWL_X_LOCAL = [dw.TINE_X[6], dw.TINE_X[1]]                          # bowl foot-ring x in the basket frame (bowl spacing 0.12 m)
BOWL_TOP_DX = [0.0, 0.12]                         # approach the back spot from the front: hand stays clear of the control band
BOWL_ABOVE = [0.14, 0.035]                        # pre-lower height above the release pose
BOWL_MID = [None, (0.07, 0.085)]                  # back spot: (dx, dz) via point so the hand stays below/ahead of the control band
BOWL_TILT = 15.                                   # deg, bowl PoC
BASKET_OUT = dw.BASKET_TRAVEL                     # 0.33 m
PARK0, HUB0 = np.array([0.30, 0.25, 1.30]), np.array([0.18, 0.36, 1.22])   # M2 park / hub at base y 0.36 (proven)
DOWN = np.diag([1., -1, -1])
PERC_TOL = dict(plate=4.0, bowl=6.0)              # mm: perceived-pose error above which the grasp falls back to GT
SPEED = 3                                         # video speed-up (sim seconds per video second)

# bowl geometry: ../poc_bowl_grasp/poc.py (rim OD 0.115, height 0.060, foot ring OD 0.050, 0.180 kg)
RIM_R, BOWL_HEIGHT, FOOT_Z = .0575, .06, -.004
BOWL_RGBA = [.55, .72, .86, 1]


def add_bowl(spec, name, xy):
    """Inverted bowl (opening down, rim on the counter), primitives only, identical to the bowl PoC."""
    bowl = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], S.COUNTER_Z + BOWL_HEIGHT + .0003], quat=[0, 1, 0, 0])
    bowl.add_freejoint(name=f'{name}_free')
    common = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2],
                  rgba=BOWL_RGBA)
    rim_apothem = RIM_R * math.cos(math.pi / 24)
    slope = math.atan2(rim_apothem - .030, .056)
    for i in range(24):
        a = i * 2 * math.pi / 24
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.sin(slope) * radial + np.array([0, 0, math.cos(slope)])
        normal = math.cos(slope) * radial - np.array([0, 0, math.sin(slope)])
        half_length = (.028 - .002 * math.sin(slope)) / math.cos(slope)
        centre_r = rim_apothem - .002 * math.cos(slope) - half_length * math.sin(slope)
        bowl.add_geom(name=f'{name}_wall_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre_r * radial + [0, 0, .032]),
                      quat=S.quat(np.column_stack([normal, tangent, along])),
                      size=[.002, RIM_R * math.sin(math.pi / 24), half_length], mass=.120 / 24, **common)
    bowl.add_geom(name=f'{name}_bottom', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .002], size=[.030, .002, 0],
                  mass=.045, **common)
    for i in range(16):
        a = i * 2 * math.pi / 16
        r = .025 * math.cos(math.pi / 16) - .002
        bowl.add_geom(name=f'{name}_foot_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[r * math.cos(a), r * math.sin(a), FOOT_Z],
                      quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], size=[.002, .025 * math.sin(math.pi / 16), .004],
                      mass=.015 / 16, **common)
    bowl.add_site(name=f'{name}_foot', pos=[0, 0, FOOT_Z], size=[.002, 0, 0], rgba=[1, .3, .1, 0])


MOUNT = S.Mount('M2', x=.40, y=RAIL_C, z=.80, yaw=195., rail_travel=RAIL_TRAVEL, cart_lx=.57, cart_ly=.60, cart_mass=150.)
LAYOUT = S.Layout(plate0=PLATES_XY[0])


def build():
    spec = mujoco.MjSpec.from_string(S._header())
    S._add_kitchen(spec)
    S._add_camera(spec, LAYOUT)
    dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    site = S._add_robot(spec, MOUNT)
    S._attach_panda(spec, site)
    tints = [[.96, .96, .93, 1], [.97, .93, .86, 1], [.92, .95, .97, 1]]
    for i, xy in enumerate(PLATES_XY):
        S._add_plate(spec, xy, PLATE_YAW[i], name=f'plate{i}', rgba=tints[i])
    for i, xy in enumerate(BOWLS_XY):
        add_bowl(spec, f'bowl{i}', xy)
    K.add_clutter(spec)
    K.add_camera(spec)
    return spec.compile()


# ------------------------------------------------------------------ video HUD: caption bar + final checklist
class Hud:
    caption = ''
    items = []            # [label, status None/True/False, detail]
    final = False
    all_ok = False
    left = []


def hud(self, img, t):
    d = ImageDraw.Draw(img, 'RGBA')
    W, H = img.size
    d.rectangle((0, 0, W, 34), fill=(18, 22, 28, 200))
    d.text((12, 6), 'Franka Panda loads a countertop dishwasher: GPT recognition + depth poses, 3 plates + 2 bowls (MuJoCo, real contact grasps)',
           fill=(225, 230, 236), font=self.font_s)
    d.text((W - 200, 6), f't = {t:6.1f} s   x{SPEED}', fill=(200, 210, 220), font=self.font_s)
    x0, y0 = W - 300, 46
    if Hud.items and not Hud.final:
        d.rectangle((x0 - 10, y0 - 8, W - 8, y0 + 26 * len(Hud.items) + 2), fill=(18, 22, 28, 175))
    for i, (lab, st, det) in enumerate([] if Hud.final else Hud.items):
        mark, col = ('✓', (90, 230, 120)) if st is True else (('✗', (255, 90, 80)) if st is False else ('·', (180, 185, 190)))
        d.text((x0, y0 + 26 * i), f'{mark} {lab}', fill=col, font=self.font_s)
    d.rectangle((0, H - 64, W, H), fill=(18, 22, 28, 235))
    d.text((14, H - 60), Hud.caption, fill=(255, 225, 130), font=self.font_b)
    if self.substep and not Hud.final:
        d.text((16, H - 26), f'step: {self.substep}', fill=(205, 212, 220), font=self.font_s)
    if Hud.final:
        bw, bh = 600, 60 + 34 * len(Hud.items)
        bx, by = W - bw - 12, 44
        d.rectangle((bx, by, bx + bw, by + bh), fill=(14, 18, 24, 225), outline=(90, 230, 120), width=3)
        d.text((bx + 18, by + 12), ('All dishes placed' if Hud.all_ok else 'Not all dishes placed') + ' (measured from physics)',
               fill='white', font=self.font_b)
        for i, (lab, st, det) in enumerate(Hud.items):
            col = (90, 230, 120) if st else (255, 90, 80)
            d.text((bx + 18, by + 54 + 34 * i), f'{"✓" if st else "✗"}  {lab}   {det}', fill=col, font=self.font_s)
        if Hud.left:
            fz = K._font(15)
            ly = by + bh + 8
            d.rectangle((bx, ly, bx + bw, ly + 34 + 22 * len(Hud.left)), fill=(14, 18, 24, 225), outline=(240, 90, 70), width=2)
            d.text((bx + 18, ly + 6), 'Left on the counter (hand-wash / not this load)  留在台面', fill='white', font=fz)
            for k, t in enumerate(Hud.left):
                t = '• ' + t
                if d.textlength(t, font=fz) > bw - 30:
                    while d.textlength(t + '…', font=fz) > bw - 30:
                        t = t[:-1]
                    t += '…'
                d.text((bx + 18, ly + 30 + 22 * k), t, fill=(255, 200, 190), font=fz)
    return img


def capture(self, data, force=False):
    if self.writer is None or (data.time + 1e-9 < self.next_time and not force):
        return
    self.next_time = data.time + SPEED / 30
    self.cam.lookat[:] = self.view[:3]
    self.cam.distance, self.cam.azimuth, self.cam.elevation = self.view[3:]
    self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
    img = Image.fromarray(self.r.render())
    self.writer.append_data(np.asarray(self.hud(img, data.time)))
    self.frames += 1


import os
VIEW = np.array([float(v) for v in os.environ.get('MD_VIEW', '0.05 0.45 1.0 1.9 180 -58').split()], float)


def set_view(self, *a, **k):
    self.view = VIEW.copy()
    self.view_target = VIEW.copy()


PL.Recorder.hud = hud
PL.Recorder.capture = capture
PL.Recorder.set_view = set_view


# ------------------------------------------------------------------ runner
class Multi(PL.Runner):
    def __init__(self, video=None, keyframes_dir=None, log=print):
        opt = PL.Option('multi', MOUNT, layout=LAYOUT, park=tuple(PARK0 + [0, -.04 + .04, 0]), hub=tuple(HUB0),
                        rail={'start': BASE_INSERT - RAIL_C}, builder=build)
        super().__init__(opt, video=video, codex=False, log=log, keyframes_dir=keyframes_dir, run_tag='multi')
        m, d = self.model, self.data
        r0 = BASE_INSERT - RAIL_C
        d.qpos[self.rail_qs] = r0
        d.ctrl[self.rail_act] = r0
        q = self.cfg.q.copy()
        q[self.ik_rail] = r0
        self.cfg.update(q)
        dw.set_state(m, d, door_deg=90., basket=BASKET_OUT)      # start state: door open, basket out (allowed simplification)
        self.basket_est = BASKET_OUT
        self.results = []
        self.perc_log, self.use_perc, self.perc = [], {}, {}
        mujoco.mj_forward(m, d)
        self.clutter0 = {nm: (d.xpos[m.body(nm).id].copy(), d.xmat[m.body(nm).id].reshape(3, 3).copy()) for nm in K.CLUTTER}
        self.rec.set_view()

    def perceive(self, tag, mode):
        """Re-perception of the current plate (at the counter edge): depth only (no new GPT call), blob nearest to the
        predicted position. Ground truth only if this plate was flagged GT-fallback at the recognition step."""
        true = self.data.xpos[self.pid].copy()
        if not self.use_perc.get(self.pname, False):
            return dict(origin=true, P=np.eye(3), radius_fit=S.R_OUT, mode=mode), dict(truth_pose=True)
        rgb, depth = self.cam.capture(self.data, self.rng)
        X, h = K.height_map(self.cam, self.data, depth)
        pred = self.est['origin'] + np.array([plan.push_poses(self.est['origin'])['target_travel'], 0, 0])
        uv = self.cam.project(self.data, pred + [0, 0, .015])
        mask = (h > .003) & (h < .04) & (X[..., 0] < .12) & (np.abs(X[..., 1] - pred[1]) < .2)
        blob = K.component(mask, seed_uv=uv)
        e = K.estimate_plate(X, h, blob)
        err = float(np.linalg.norm(e['origin'][:2] - true[:2]) * 1000)
        self.perc_log.append(dict(item=self.pname, tag=tag, err_mm=round(err, 2), radius_fit_mm=round(e['radius_fit'] * 1000, 1)))
        return dict(origin=e['origin'], P=np.eye(3), radius_fit=e['radius_fit'], mode=mode), dict(depth_err_mm=round(err, 2))

    # -------------------------------------------------------------- recognition (camera + GPT + depth)
    def recognize(self, cache=None):
        m, d = self.model, self.data
        self.cam = K.Cam(m)
        rgb, depth = self.cam.capture(d, self.rng)
        rdir = OUT / 'recog'
        rdir.mkdir(parents=True, exist_ok=True)
        png = rdir / 'counter.png'
        Image.fromarray(rgb).save(png)
        if cache and Path(cache).exists():
            obj, meta = json.loads(Path(cache).read_text()), dict(seconds=None, cached=str(cache))
        else:
            obj, meta = K.ask_gpt(png, rdir)
        self.gpt_meta = meta
        if obj is None:
            obj = dict(items=[])
        X, h = K.height_map(self.cam, d, depth)
        mask = (h > .003) & (h < .09) & (X[..., 0] < .02) & (X[..., 0] > -.6)
        bodies = {f'plate{i}': d.xpos[m.body(f'plate{i}').id].copy() for i in range(3)}
        bodies.update({f'bowl{j}': self.foot(j) for j in range(2)})
        bodies.update({nm: d.xpos[m.body(nm).id].copy() for nm in K.CLUTTER})
        rows, self.perc = [], {}
        for it in obj['items']:
            cat = it['category']
            row = dict(gpt=it)
            x0, y0, x1, y1 = it['box']
            cu, cv = (x0 + x1) / 2, (y0 + y1) / 2
            ray_p, Rc = self.cam.pose(d)
            ray = Rc @ np.array([(cu - self.cam.cx) / self.cam.f, -(cv - self.cam.cy) / self.cam.f, -1.])
            hit = ray_p + ray * (S.COUNTER_Z + .02 - ray_p[2]) / ray[2]            # box centre on the counter (for matching)
            match = min(bodies, key=lambda b: np.linalg.norm(bodies[b][:2] - hit[:2]))
            row['match'] = match
            if cat in ('plate', 'bowl'):
                blob = K.component(mask, box=it['box'])
                if blob is not None:
                    if cat == 'plate':
                        e = K.estimate_plate(X, h, blob)
                        pos = e['origin']
                        nm = min([b for b in bodies if b.startswith('plate')], key=lambda b: np.linalg.norm(bodies[b][:2] - pos[:2]))
                        err = float(np.linalg.norm(pos[:2] - bodies[nm][:2]) * 1000)
                        row.update(match=nm, perceived=pos.tolist(), err_mm=round(err, 2), radius_fit_mm=round(e['radius_fit'] * 1000, 1))
                    else:
                        e = K.estimate_bowl(X, h, blob)
                        pos = e['foot']
                        nm = min([b for b in bodies if b.startswith('bowl')], key=lambda b: np.linalg.norm(bodies[b][:2] - pos[:2]))
                        err = float(np.linalg.norm(pos - bodies[nm]) * 1000)
                        row.update(match=nm, perceived=pos.tolist(), err_mm=round(err, 2))
                    tol = PERC_TOL[cat]
                    row['use_perceived'] = err < tol
                    if nm not in self.perc or err < self.perc[nm]['err_mm']:
                        self.perc[nm] = dict(pos=pos, err_mm=err, use=err < tol)
            rows.append(row)
        self.recog_rows = rows
        self.use_perc = {k: v['use'] for k, v in self.perc.items()}
        # slots from GPT (fallback: fixed rule) -> loading order: front plate slot first, then bowl spots 1, 2
        gslot = {}
        for r in rows:
            t = r['gpt']['target']
            if r['match'].startswith('plate') and t.startswith('plate slot'):
                gslot[r['match']] = int(t.split()[-1]) - 1
            if r['match'].startswith('bowl') and t.startswith('bowl spot'):
                gslot[r['match']] = int(t.split()[-1]) - 1
        ps = [gslot.get(f'plate{i}') for i in range(3)]
        self.slot_from_gpt = sorted([p for p in ps if p is not None]) == [3, 4, 5]
        self.plate_slot = {i: (ps[i] if self.slot_from_gpt else PLATE_SLOTS[i]) for i in range(3)}
        bs = [gslot.get(f'bowl{j}') for j in range(2)]
        self.bowl_from_gpt = sorted([b for b in bs if b is not None]) == [0, 1]
        self.bowl_spot = {j: (bs[j] if self.bowl_from_gpt else j) for j in range(2)}
        self.cards = dict(rgb=rgb, depth=depth, X=X, h=h)
        (rdir / 'recognition.json').write_text(json.dumps(dict(gpt_meta=meta, rows=rows, plate_slot=self.plate_slot,
                                                              bowl_spot=self.bowl_spot, slot_from_gpt=self.slot_from_gpt,
                                                              bowl_from_gpt=self.bowl_from_gpt), indent=1, default=float))
        return rows

    def set_base(self, base_y):
        """Rail setpoint for a base y; park/hub follow the base (they were defined at base y 0.36)."""
        dy = base_y - BASE_INSERT
        self.opt.park = tuple(PARK0 + [0, dy, 0])
        self.opt.hub = tuple(HUB0 + [0, dy, 0])
        return base_y - RAIL_C

    def lift_clear(self, z=1.36):
        p, R = self.ref.get('cmd', self.tcp())
        if p[2] < z - 1e-3:
            self.goto('clear_up', np.r_[p[:2], z], R, float(self.data.ctrl[self.act[7]]), speed=.2)

    # -------------------------------------------------------------- plates
    def do_plate(self, i, n):
        self.pname = f'plate{i}'
        self.pid = self.model.body(self.pname).id
        self.slot_k = self.plate_slot[i]
        src = 'perceived pose (GPT box + depth)' if self.use_perc.get(self.pname) else 'GROUND-TRUTH pose (perception fallback)'
        tag = f'Plate {n}/3 → slot {self.slot_k + 1}   [{src}]'
        self.lift_clear()
        if self.use_perc.get(self.pname):
            self.est = dict(origin=np.asarray(self.perc[self.pname]['pos'], float), P=np.eye(3), mode='flat')
        else:
            self.est = dict(origin=self.data.xpos[self.pid].copy(), P=np.eye(3), mode='flat')
        self.opt.rail['pick_plate'] = self.set_base(float(self.est['origin'][1]) - .04)
        Hud.caption = f'{tag}'
        self.run_step(f'pick_plate{i}', self.pick_flat)
        Hud.caption = f'{tag}  (carry + insert upright between the tines)'
        self.opt.rail['insert_plate'] = self.set_base(BASE_INSERT)
        self.run_step(f'insert_plate{i}', self.step_insert)

    def plate_check(self, i):
        self.pname, self.pid, self.slot_k = f'plate{i}', self.model.body(f'plate{i}').id, self.plate_slot[i]
        st = self.plate_slot_state()
        ok = st['in_slot'] and st['speed_mm_s'] < 5 and not st['touching_robot']
        return ok, st

    # -------------------------------------------------------------- bowls
    def bowl_target(self, j):
        # basket position from the known appliance model + the known pulled-out travel (end stop); no simulator state
        b = np.array(S.DW_POS) + np.array([dw.BASKET_X0 + self.basket_est, 0., dw.BASKET_Z])
        x_loc = BOWL_X_LOCAL[self.bowl_spot[j]]
        t = math.radians(BOWL_TILT)
        # foot-ring centre of the inverted bowl leaning 15 deg on the 35-deg bowl row (dw site bowl_slot_target)
        return b + np.array([x_loc, dw.BOWL_ROW_Y - .0786, .0787]), t

    def foot(self, j):
        return self.data.site(f'bowl{j}_foot').xpos.copy()

    def pick_bowl(self, j):
        bid = self.model.body(f'bowl{j}').id
        self.rail_to('pick_bowl')
        g = np.asarray(self.perc[f'bowl{j}']['pos'], float) if self.use_perc.get(f'bowl{j}') else self.foot(j)
        Rg = hand_frame([0, 0, -1.], getattr(self, 'bowl_close', [1., 0, 0]))   # sink: along y (hand narrow in x near the walls)
        f, qp, _ = self.plan_path([(g + [0, 0, .15], Rg), (g, Rg), (g + [0, 0, .10], Rg)], flips=(1, -1))
        if qp is None:
            self.fail('no joint path for the bowl grasp')
        Rg = Rg if f > 0 else Rg @ self.FLIP
        self.allowed = {bid}
        self.substep('approach the foot ring from above')
        self.transfer('bowl_approach', g + [0, 0, .15], Rg, PL.OPEN, speed=.25, q_hint=qp[0])
        self.move('bowl_descend', 2.0, g, Rg, PL.OPEN, qpath=qp[:2])
        self.move('close', 1.5, g, Rg, PL.grip_ctrl(0.))
        self.hold('close', .3)
        p, R = self.tcp()
        rel = R.T @ (g - p)                                # in-hand offset from the pose used for the grasp
        self.bowl_hold = (rel, None)
        start = self.foot(j)
        mx = dict(slip=0.)
        rel_true = R.T @ (self.foot(j) - p)                # scoring only

        def track():
            p_, R_ = self.tcp()
            mx['slip'] = max(mx['slip'], float(np.linalg.norm(R_.T @ (self.foot(j) - p_) - rel_true)))
        self.ref['cb'] = track
        self.substep('lift 0.10 m')
        self.move('lift', 2.0, g + [0, 0, .10], Rg, PL.grip_ctrl(0.), qpath=qp[1:])
        self.hold('hold', .5)
        self.ref.pop('cb')
        rise = float(self.foot(j)[2] - start[2])
        nf = self.body_contacts(bid, self.finger_bodies)[1]
        ok = rise > .09 and mx['slip'] < .005 and nf > 0
        self.bowl_track = track
        return dict(_ok=ok, _reason='' if ok else f'rise {rise:.3f} slip {mx["slip"]:.4f} nf {nf}', lift_rise_mm=round(rise * 1000, 1),
                    lift_slip_mm=round(mx['slip'] * 1000, 2), finger_contacts=nf)

    def insert_bowl(self, j):
        bid = self.model.body(f'bowl{j}').id
        rel, _ = self.bowl_hold
        # placement frame: fingers along x (off the basket side frame); the bowl is axisymmetric about the approach axis,
        # so turning the hand about it only turns the bowl about its own axis
        Rg = self.ref['cmd'][1] if not hasattr(self, 'bowl_close') else hand_frame([0, 0, -1.], [1., 0, 0])
        self.substep('carry: up, along the counter, over the bowl section')
        self.lift_clear(1.40)
        self.rail_to('insert_bowl')
        tgt, t = self.bowl_target(j)
        best = None
        for fl in (np.eye(3), self.FLIP):      # the bowl is axisymmetric: the hand may approach either way round
            Rgoal_ = rot_axis([1., 0, 0], t) @ Rg @ fl
            low_ = tgt - Rgoal_ @ rel             # measured-at-grasp in-hand offset (bowl turns with the hand)
            above_ = low_ + [0, 0, BOWL_ABOVE[self.bowl_spot[j]]]
            top_ = np.r_[above_[0] + BOWL_TOP_DX[self.bowl_spot[j]], above_[1], 1.32]
            wps_ = [top_] + ([low_ + [BOWL_MID[self.bowl_spot[j]][0], 0, BOWL_MID[self.bowl_spot[j]][1]]] if BOWL_MID[self.bowl_spot[j]] else []) + [above_]
            f_, qs_, wm_ = self.plan_path([(w_, Rgoal_) for w_ in wps_] + [(low_, Rgoal_)])
            self.log_fn(f'    bowl place plan flip={fl[0, 0]:+.0f}: {"ok margin %.1f deg" % math.degrees(wm_) if qs_ is not None else "none"}')
            if qs_ is None:
                for nm_, (pp_, RR_) in zip(['top', 'above', 'low', 'retract'], [(top_, Rgoal_), (above_, Rgoal_), (low_, Rgoal_)]):
                    q_, mg_, er_ = self.solve_q(pp_, RR_)
                    self.log_fn(f'      {nm_} {np.round(pp_, 3)} err {er_ * 1000:.1f} mm margin {math.degrees(mg_):.1f} deg')
            if qs_ is not None and (best is None or wm_ > best[0]):
                best = (wm_, Rgoal_, low_, above_, qs_, top_, wps_)
        if best is None:
            self.fail('no joint path for the bowl placement')
        _, Rgoal, low, above, qs, top, wps = best
        cur = self.ref['cmd'][0]
        self.goto('carry_out', np.r_[max(above[0], .25), cur[1], 1.40], Rg, PL.grip_ctrl(0.), speed=.2)   # out in front of the machine first
        self.goto('carry_over', top, Rgoal, PL.grip_ctrl(0.), speed=.2, wspeed=.5, q_hint=qs[0])
        for k_ in range(1, len(wps)):
            self.move('carry_down', 2.5, wps[k_], Rgoal, PL.grip_ctrl(0.), qpath=qs[k_ - 1:k_ + 1])
        self.hold('carry_settle', .4)
        w0 = float(self.hand_force_world()[2])
        sup = dict(filt=0., cnt=0, at=None, peak=0.)

        def stop():
            s_ = w0 - self.hand_force_world()[2]
            sup['filt'] += .2 * (s_ - sup['filt'])
            sup['peak'] = max(sup['peak'], s_)
            sup['cnt'] = sup['cnt'] + 1 if sup['filt'] > .5 else 0
            if sup['cnt'] >= 5:
                sup['at'] = self.tcp()[0].copy()
                sup['touch'] = sorted(t_ for t_ in self.touching(bid) if not t_.startswith('panda/'))
                return True
            return False
        self.substep('lower onto the leaning bowl row, tilted 15 deg (stop on wrist-F/T touchdown)')
        hit = self.move('lower', 3.0, low - [0, 0, .01], Rgoal, PL.grip_ctrl(0.), stop=stop, qpath=qs[-2:])
        sp = sup['at'] if hit else low - [0, 0, .01]
        self.ref['cmd'] = (sp, Rgoal)
        self.substep('release, retract')
        self.move('release', 1.0, sp, Rgoal, 200.)
        for w_ in reversed(wps):
            self.move('retract', 1.5, w_, Rgoal, 200.)
        self.allowed = set()
        self.hold('settle', 1.0)
        ok, st = self.bowl_check(j)
        return dict(_ok=ok, _reason='' if ok else f'bowl not placed: {st}', touchdown_detected=bool(hit),
                    touchdown_peak_N=round(sup['peak'], 2), touchdown_contacts=sup.get('touch'),
                    touchdown_above_target_mm=round(float(sp[2] - low[2]) * 1000, 1), **st)

    def bowl_check(self, j):
        bid = self.model.body(f'bowl{j}').id
        tgt, _ = self.bowl_target(j)
        e = self.foot(j) - tgt
        R = self.data.xmat[bid].reshape(3, 3)
        up_dot = float(R[2, 2])                                   # bowl opening axis . world z (-1 = fully inverted)
        tilt = math.degrees(math.acos(max(-1., min(1., -up_dot))))          # tilt from fully inverted
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
        touch = self.touching(bid)
        robot = any(t_.startswith('panda/') for t_ in touch)
        tines = sorted(t_ for t_ in touch if t_.startswith('dw/bowl_tine'))
        other = sorted(t_ for t_ in touch if not t_.startswith('dw/'))
        ok = float(np.linalg.norm(e)) < .015 and up_dot < -.9 and float(np.linalg.norm(vel[3:])) < .005 and not robot and tines and not other
        return bool(ok), dict(foot_err_mm=round(float(np.linalg.norm(e)) * 1000, 1), foot_err_xyz_mm=[round(float(v) * 1000, 1) for v in e],
                              tilt_from_inverted_deg=round(tilt, 1), opening_dot_z=round(up_dot, 3),
                              speed_mm_s=round(float(np.linalg.norm(vel[3:])) * 1000, 2), bowl_tines=tines, other_contacts=other,
                              touching_robot=robot)

    def do_bowl(self, j, n):
        src = 'perceived pose (GPT box + depth)' if self.use_perc.get(f'bowl{j}') else 'GROUND-TRUTH pose (perception fallback)'
        tag = f'Bowl {n}/2 → bowl spot {self.bowl_spot[j] + 1}   [{src}]'
        self.lift_clear()
        gy = self.perc[f'bowl{j}']['pos'][1] if self.use_perc.get(f'bowl{j}') else self.foot(j)[1]
        self.opt.rail['pick_bowl'] = self.set_base(float(gy) - .04)
        Hud.caption = f'{tag}'
        self.run_step(f'pick_bowl{j}', lambda: self.pick_bowl(j))
        self.opt.rail['insert_bowl'] = self.set_base(BASE_INSERT)
        Hud.caption = f'{tag}  (place inverted, tilted 15° on the leaning tines)'
        self.run_step(f'place_bowl{j}', lambda: self.insert_bowl(j))

    # -------------------------------------------------------------- whole run
    def card_frames(self, img, seconds):
        if self.rec.writer is None:
            return
        for _ in range(int(30 * seconds)):
            self.rec.writer.append_data(img)
            self.rec.frames += 1

    def make_cards(self):
        rgb = self.cards['rgb']
        W_, H_ = K.CAM_W, K.CAM_H
        gsec = self.gpt_meta.get('seconds')
        # 1) raw camera image + "GPT thinking" banner
        im = Image.fromarray(rgb).convert('RGB')
        dd = ImageDraw.Draw(im, 'RGBA')
        dd.rectangle((0, H_ - 70, W_, H_), fill=(14, 18, 24, 225))
        dd.text((16, H_ - 62), f'GPT (codex exec, ChatGPT subscription) is recognising the counter ...  '
                f'{gsec if gsec is not None else "cached"} s, fast-forwarded', fill=(255, 225, 130), font=K._font(24))
        dd.text((16, H_ - 28), 'Input: this overhead camera image only (no ground truth).  输入：仅此俯视相机图像', fill=(220, 220, 220), font=K._font(18))
        thinking = np.asarray(im)
        # 2) labelled boxes
        items = [r['gpt'] for r in self.recog_rows]
        n_safe = sum(1 for it in items if it['dishwasher_safe'])
        card = K.draw_card(rgb, items, f'GPT recognition: {len(items)} items, {n_safe} dishwasher-safe  |  '
                                       f'green = load  amber = safe, not this load  red = hand-wash   绿=装入 红=手洗')
        # 3) depth poses: perceived vs true
        im = Image.fromarray(card).convert('RGB')
        dd = ImageDraw.Draw(im, 'RGBA')
        for r in self.recog_rows:
            if 'perceived' in r:
                uv = self.cam.project(self.data, np.asarray(r['perceived']) + [0, 0, .015])
                col = (60, 220, 100) if r['use_perceived'] else (255, 160, 40)
                dd.ellipse((uv[0] - 7, uv[1] - 7, uv[0] + 7, uv[1] + 7), outline=col + (255,), width=3)
        lines = ['Depth back-projection (noisy depth) -> metric pose, error vs ground truth:']
        for nm in sorted(self.perc):
            pe = self.perc[nm]
            lines.append(f'  {nm}: {pe["err_mm"]:.1f} mm -> ' + ('use PERCEIVED pose' if pe['use'] else 'GT fallback (error too large)'))
        bh = 26 * len(lines) + 16
        dd.rectangle((W_ - 640, H_ - bh - 10, W_ - 10, H_ - 10), fill=(14, 18, 24, 235))
        for k, t in enumerate(lines):
            dd.text((W_ - 628, H_ - bh + 26 * k), t, fill=(235, 235, 235) if k else (255, 225, 130), font=K._font(17))
        posecard = np.asarray(im)
        return thinking, card, posecard

    def run_all(self, plates=3, bowls=2, cache=None):
        self.ref['cmd'] = self.tcp()
        Hud.items = []
        # opening: the cluttered counter (wide view)
        self.rec.view = np.array([-0.12, 0.0, 0.95, 2.85, 180., -48.], float)
        Hud.caption = 'Kitchen counter: 3 plates, 2 bowls and everyday kitchen items'
        self.substep('')
        self.hold('opening', 6.0, PL.OPEN)
        if self.keyframes_dir:
            self.rec.still(self.data, Path(self.keyframes_dir) / '00_counter.png')
        self.recognize(cache)
        thinking, card, posecard = self.make_cards()
        Image.fromarray(card).save(OUT / 'recog' / 'card.png')
        Image.fromarray(posecard).save(OUT / 'recog' / 'card_poses.png')
        self.card_frames(thinking, 2.5)
        self.card_frames(card, 8.0)
        self.card_frames(posecard, 5.0)
        self.rec.view = VIEW.copy()
        order_p = sorted(range(plates), key=lambda i: -self.plate_slot[i])          # front slot first
        order_b = sorted(range(bowls), key=lambda j: self.bowl_spot[j])
        Hud.items = [[f'Plate {i + 1} → slot {self.plate_slot[i] + 1}', None, ''] for i in order_p] + \
                    [[f'Bowl {j + 1} → bowl spot {self.bowl_spot[j] + 1}', None, ''] for j in order_b]
        Hud.caption = 'Loading: door open, basket out'
        self.hold('settle', 1.0, PL.OPEN)
        fails = []
        for k, i in enumerate(order_p):
            try:
                self.do_plate(i, k + 1)
                Hud.items[k][1] = True
            except PL.StepFail as e:
                fails.append(str(e))
                Hud.items[k][1] = False
                self.log_fn(f'FAIL plate {i}: {e}')
                self.recover()
        for k, j in enumerate(order_b):
            try:
                self.do_bowl(j, k + 1)
                Hud.items[len(order_p) + k][1] = True
            except PL.StepFail as e:
                fails.append(str(e))
                Hud.items[len(order_p) + k][1] = False
                self.log_fn(f'FAIL bowl {j}: {e}')
                self.recover()
        Hud.caption = 'Done: robot returns to park, everything settles (final poses measured)'
        self.lift_clear()
        self.set_base(BASE_INSERT)
        self.transfer('home', PARK0, hand_frame([0, 0, -1.], [0, 1., 0]), PL.OPEN, speed=.25)
        self.hold('final_settle', 2.0)
        table = []
        for k, i in enumerate(order_p):
            ok, st = self.plate_check(i)
            pe = self.perc.get(f'plate{i}', {})
            table.append(dict(item=f'plate{i + 1}', target=f'plate slot {self.plate_slot[i] + 1}', ok=bool(ok),
                              pose_source='perceived' if self.use_perc.get(f'plate{i}') else 'GT fallback',
                              perceived_err_mm=round(pe.get('err_mm', float('nan')), 2),
                              bottom_offset_mm=st['bottom_offset_mm'], centre_err_mm=st['centre_err_mm'],
                              plane_tilt_deg=st['plane_tilt_deg'], speed_mm_s=st['speed_mm_s'],
                              wrong_tines=st['wrong_tines'], other_contacts=st['other_contacts']))
            Hud.items[k][1] = bool(ok)
            Hud.items[k][2] = f'rim offset {st["bottom_offset_mm"]:+.1f} mm, lean {st["plane_tilt_deg"]:.1f}°'
        for k, j in enumerate(order_b):
            ok, st = self.bowl_check(j)
            pe = self.perc.get(f'bowl{j}', {})
            table.append(dict(item=f'bowl{j + 1}', target=f'bowl spot {self.bowl_spot[j] + 1}', ok=bool(ok),
                              pose_source='perceived' if self.use_perc.get(f'bowl{j}') else 'GT fallback',
                              perceived_err_mm=round(pe.get('err_mm', float('nan')), 2), **st))
            Hud.items[len(order_p) + k][1] = bool(ok)
            Hud.items[len(order_p) + k][2] = f'foot err {st["foot_err_mm"]:.1f} mm, tilt {st["tilt_from_inverted_deg"]:.1f}°'
        # clutter: still standing where it was?
        moved = {}
        for nm in K.CLUTTER:
            bid = self.model.body(nm).id
            moved[nm] = dict(moved_mm=round(float(np.linalg.norm(self.data.xpos[bid] - self.clutter0[nm][0])) * 1000, 2),
                             tilt_deg=round(angle_deg(self.data.xmat[bid].reshape(3, 3) @ self.clutter0[nm][1].T), 2))
        Hud.left = [f"{r['gpt']['label_en']} / {r['gpt']['label_zh']}: {r['gpt']['reason_en']}"
                    for r in self.recog_rows if not (r['match'].startswith(('plate', 'bowl')) and r['gpt']['dishwasher_safe'])]
        self.rec.view = np.array([0.12, 0.0, 1.06, 0.95, 180., -55.], float)
        Hud.caption = 'Loaded basket: 3 plates upright between the tines, 2 bowls inverted + tilted on the bowl row'
        self.substep('')
        self.hold('final_view', 9.0)
        if self.keyframes_dir:
            self.rec.still(self.data, Path(self.keyframes_dir) / '98_loaded_basket.png')
        self.rec.view = np.array([0.12, 0.17, 1.06, 0.95, 180., -55.], float)    # basket on the left, cards on the right
        Hud.final = True
        Hud.all_ok = all(r['ok'] for r in table)
        Hud.caption = 'Final check: ' + ', '.join(f'{r["item"]} {"OK" if r["ok"] else "FAIL"}' for r in table)
        self.hold('final', 18.0)
        if self.keyframes_dir:
            self.rec.still(self.data, Path(self.keyframes_dir) / '99_final.png')
        self.rec.close()
        self.cam.close()
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(self.data.warning) if w.number}
        unexpected = [dict(step=k[0], phase=k[1], robot=k[2], other=k[3], max_force_N=round(v, 2)) for k, v in self.contact_log.items()]
        return dict(table=table, fails=fails, steps=self.steps, sim_time_s=round(self.data.time, 2), clutter=moved,
                    recognition=self.recog_rows, gpt_meta=self.gpt_meta, edge_reperception=self.perc_log,
                    slot_from_gpt=self.slot_from_gpt, bowl_from_gpt=self.bowl_from_gpt,
                    wall_s=round(time.monotonic() - self.t_wall0, 1), warnings=warnings,
                    max_penetration_mm=round(self.metrics['max_penetration_mm'], 3), max_pen_where=self.metrics['max_pen_where'],
                    unexpected_robot_contacts=unexpected, video_frames=self.rec.frames)

    def recover(self):
        """After a failed item: open the hand, back up and continue with the next one."""
        self.allowed = set()
        self.ref.pop('cb', None)
        self.ref['cmd'] = self.tcp()
        self.hold('recover', 1.0, PL.OPEN)
        self.lift_clear()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--only', choices=['plates', 'bowls', 'all'], default='all')
    ap.add_argument('--plates', type=int, default=3)
    ap.add_argument('--tag', default='')
    ap.add_argument('--still', action='store_true', help='render the start frame only')
    ap.add_argument('--cache', default=None, help='reuse a saved GPT answer (dev runs only)')
    ap.add_argument('--frames', default='frames')
    ap.add_argument('--seed', type=int, default=None, help='perturb start poses: +-5 mm x/y every item, +-10 deg plate yaw')
    a = ap.parse_args()
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        for L in (PLATES_XY, BOWLS_XY):
            for k in range(len(L)):
                L[k] = tuple(np.array(L[k]) + rng.uniform(-.005, .005, 2))
        for k in range(len(PLATE_YAW)):
            PLATE_YAW[k] = float(rng.uniform(-math.radians(10), math.radians(10)))
        print('perturbed', PLATES_XY, BOWLS_XY, [round(math.degrees(v), 1) for v in PLATE_YAW], flush=True)
    OUT.mkdir(exist_ok=True)
    np_, nb = (a.plates, 0) if a.only == 'plates' else ((0, 2) if a.only == 'bowls' else (a.plates, 2))
    video = None if a.no_video else OUT / f'raw_demo{a.tag}.mp4'
    kf = None if a.no_video else ROOT / a.frames
    if kf:
        kf.mkdir(exist_ok=True)
    if a.still:
        r = Multi(video=OUT / 'still_tmp.mp4', keyframes_dir=ROOT / 'frames')
        r.hold('settle', .3, PL.OPEN)
        cam = K.Cam(r.model)
        rgb, _ = cam.capture(r.data, np.random.default_rng(0))
        Image.fromarray(rgb).save(ROOT / 'frames' / 'countercam_test.png')
        r.rec.view = np.array([-0.12, 0.0, 0.95, 2.85, 180., -48.], float)
        r.rec.still(r.data, ROOT / 'frames' / 'wide_test.png')
        r.rec.view = VIEW.copy()
        Hud.items = [[f'Plate {i + 1} → slot {PLATE_SLOTS[i] + 1}', None, ''] for i in range(3)] + \
                    [[f'Bowl {j + 1} → bowl spot {j + 1}', None, ''] for j in range(2)]
        Hud.caption = 'start'
        r.rec.still(r.data, ROOT / 'frames' / 'view_test.png')
        r.rec.close()
        return
    r = Multi(video=video, keyframes_dir=kf, log=lambda s: print(s, flush=True))
    res = r.run_all(np_, nb, cache=a.cache)
    (OUT / f'result{a.tag}.json').write_text(json.dumps(res, indent=1, default=float))
    for row in res['table']:
        print(json.dumps(row, default=float))
    print('fails', res['fails'], 'sim', res['sim_time_s'], 'wall', res['wall_s'], 'warnings', res['warnings'],
          'pen', res['max_penetration_mm'], res['max_pen_where'])


if __name__ == '__main__':
    main()
