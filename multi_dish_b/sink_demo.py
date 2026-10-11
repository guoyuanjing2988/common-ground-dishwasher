#!/usr/bin/env python3
"""Messy-sink demo: dishes dumped into the sink (mj_step), GPT recognition + pick order (codex exec), depth
re-perception before every pick, rim pinch of plates leaning on the basin wall, foot-ring grasp of upside-down bowls,
placement as in multi.py; what the gripper cannot reach is handed to a person (reported).

  python sink_demo.py [--no-video] [--cache out/sink/recog_gpt.json] [--tag X]
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

import dishwasher as dw
import kitchen as K
import multi as M
import pipeline as PL
import scene as S
import sink_scene as SS
from perception import ransac_plane
from util import hand_frame, unit

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'
SD = OUT / 'sink'
M.build = SS.build
PLATE_SLOTS_SINK = [5, 4, 3, 2]           # slot k (k+1 on screen): front slot first
SIDE = dict(plate=6.0, bowl=6.0)       # GT-fallback thresholds (mm)

SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["items"],
    "properties": {"items": {"type": "array", "items": {
        "type": "object", "additionalProperties": False,
        "required": ["box", "label_en", "label_zh", "category", "in_sink", "dishwasher_safe", "reason_en", "reason_zh",
                     "pick_order", "pose_en"],
        "properties": {
            "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4},
            "label_en": {"type": "string"}, "label_zh": {"type": "string"},
            "category": {"type": "string", "enum": ['plate', 'bowl', 'mug', 'cup', 'cutting_board', 'knife', 'pan', 'flask',
                                                    'soap_bottle', 'sponge', 'cloth', 'oil_bottle', 'faucet', 'other']},
            "in_sink": {"type": "boolean"},
            "dishwasher_safe": {"type": "boolean"},
            "reason_en": {"type": "string", "description": "max 8 words"},
            "reason_zh": {"type": "string", "description": "max 12 characters"},
            "pick_order": {"type": "integer", "description": "1, 2, 3 ... for dishes in the sink to load (top-most / least blocked first); 0 otherwise"},
            "pose_en": {"type": "string", "description": "how it lies, e.g. 'leaning on the front wall', 'upside down', 'on top of a plate', max 6 words"}}}}}}

PROMPT = """You are the vision + planning module of a dish-loading robot arm. The image ({w}x{h} px) is from a camera above a
kitchen counter: a sink (steel basin) full of dirty dishes is right of a small countertop dishwasher (door open, basket out);
other kitchen items stand on the counter.
List EVERY loose item (in the sink and on the counter; not the dishwasher, robot, faucet or the sink itself).
For each: a tight pixel box, English + Chinese label, whether it is in the sink, dishwasher-safe or not with a short reason
(English <= 8 words, Chinese <= 12 characters), how it lies (pose_en), and pick_order: number the dishwasher-safe dishes IN THE
SINK in the order a robot should take them out of the pile: top-most / least-blocked first (an item lying under another item
comes after the item on top of it). Everything else gets pick_order 0. Return JSON only."""


def poly_card(rgb, items, title):
    """Recognition card with pick order."""
    img = Image.fromarray(rgb).convert('RGB')
    d = ImageDraw.Draw(img, 'RGBA')
    f, fs = K._font(16), K._font(15)
    for k, it in enumerate(items):
        x0, y0, x1, y1 = it['box']
        col = (60, 220, 100) if it['pick_order'] > 0 else ((245, 190, 60) if it['dishwasher_safe'] else (240, 70, 60))
        d.rectangle((x0, y0, x1, y1), outline=col + (255,), width=3)
        tag = f"#{it['pick_order']}" if it['pick_order'] > 0 else str(k + 1)
        d.rectangle((x0, y0, x0 + 12 + 10 * len(tag), y0 + 20), fill=col + (235,))
        d.text((x0 + 4, y0 + 1), tag, fill=(0, 0, 0), font=fs)
    picks = sorted([it for it in items if it['pick_order'] > 0], key=lambda it: it['pick_order'])
    rest = [(k, it) for k, it in enumerate(items) if it['pick_order'] <= 0]
    px, py, pw = 700, 48, 575
    d.rectangle((px, py, px + pw, py + 30 + 22 * len(picks)), fill=(14, 18, 24, 228))
    d.text((px + 10, py + 5), 'Pick order (top-most first)  取放顺序（从最上层开始）', fill=(60, 220, 100), font=f)
    for n, it in enumerate(picks):
        t = f"#{it['pick_order']}  {it['label_en']} / {it['label_zh']} – {it['pose_en']}"
        while d.textlength(t, font=fs) > pw - 20:
            t = t[:-1]
        d.text((px + 10, py + 28 + 22 * n), t, fill=(235, 235, 235), font=fs)
    px, pw = 6, 520
    ph = 30 + 22 * len(rest)
    py = K.CAM_H - ph - 6
    d.rectangle((px, py, px + pw, py + ph), fill=(14, 18, 24, 228))
    d.text((px + 10, py + 5), 'Not loaded  不装入', fill=(240, 90, 70), font=f)
    for k, it in rest:
        t = f"{k + 1}  {it['label_en']} / {it['label_zh']}: {it['reason_en']}"
        while d.textlength(t, font=fs) > pw - 20:
            t = t[:-1]
        d.text((px + 10, py + 28 + 22 * rest.index((k, it))), t,
               fill=(245, 190, 60) if it['dishwasher_safe'] else (240, 90, 70), font=fs)
    d.rectangle((0, 0, K.CAM_W, 40), fill=(14, 18, 24, 225))
    d.text((12, 8), title, fill='white', font=K._font(20))
    return np.asarray(img)


class Sink(M.Multi):
    def __init__(self, video=None, keyframes_dir=None, log=print):
        super().__init__(video=video, keyframes_dir=keyframes_dir, log=log)
        self.events = []
        self.handed = []
        self.bowl_close = [0., 1., 0]

    # -------------------------------------------------------------- perception
    def depth(self):
        rgb, depth = self.cam.capture(self.data, self.rng)
        X, h = K.height_map(self.cam, self.data, depth)
        return rgb, X

    def in_basin(self, X):
        x, y, z = X[..., 0], X[..., 1], X[..., 2]
        inner = (x > SS.BX0 + .003) & (x < SS.BX1 - .003) & (z > SS.FLOOR_Z + .004)
        over = (x >= SS.BX1 - .003) & (x < SS.BX1 + .08) & (z > S.COUNTER_Z + .004)       # rims sticking up over the front wall
        return (inner | over) & (y > SS.BY0 + .003) & (y < SS.BY1 - .003) & (z < SS.COUNTER_Z_TOP)

    def box_pts(self, X, box, pad=6):
        x0, y0, x1, y1 = [int(round(v)) for v in box]
        msk = np.zeros(X.shape[:2], bool)
        msk[max(0, y0 - pad):y1 + pad, max(0, x0 - pad):x1 + pad] = True
        return X[msk & self.in_basin(X)]

    def est_plate(self, X, box):
        P = self.box_pts(X, box)
        if len(P) < 200:
            return None
        c, n, inl = ransac_plane(P, thr=.004, iters=400, rng=np.random.default_rng(1))
        if abs(n[2]) > .9:
            return dict(kind='flat', n=n.tolist())
        band = np.abs((P - c) @ n) < .015
        Q = P[band]
        u = unit(np.array([0, 0, 1.]) - n[2] * n)
        v = unit(np.cross(u, n))
        cu, cv = (Q - c) @ u, (Q - c) @ v
        top = np.percentile(cu, 99.5)
        near = cu > top - .03
        ctr = c + (top - S.R_OUT) * u + float(np.mean(cv[near])) * v
        r = np.hypot(cu - (top - S.R_OUT), cv - float(np.mean(cv[near])))
        dd = (Q - ctr) @ n
        s = (dd[r > .088].mean() if (r > .088).any() else 0.) - (dd[r < .06].mean() if (r < .06).any() else 0.)
        if s < 0:
            n = -n
        campos = self.cam.pose(self.data)[0]
        face = .010 if n @ (campos - ctr) > 0 else .005           # visible face: concave disc top / convex disc bottom
        origin = ctr - n * face
        Pm = np.column_stack([np.cross(u, n), u, n])
        return dict(kind='lean', origin=origin, P=Pm, centre=origin + n * S.MID_Z, rim_sign=float(s))

    def est_bowl(self, X, box):
        x0, y0, x1, y1 = box
        cu, cv = (x0 + x1) / 2, (y0 + y1) / 2
        ray_p, Rc = self.cam.pose(self.data)
        ray = Rc @ np.array([(cu - self.cam.cx) / self.cam.f, -(cv - self.cam.cy) / self.cam.f, -1.])
        seed = ray_p + ray * (SS.FLOOR_Z + .05 - ray_p[2]) / ray[2]
        A = X[self.in_basin(X)]
        for _ in range(3):
            sel = A[np.linalg.norm(A[:, :2] - seed[:2], axis=1) < .065]
            if len(sel) < 100:
                return None
            zt = sel[:, 2].max()
            topb = sel[sel[:, 2] > zt - .006]
            seed = np.r_[topb[:, :2].mean(0), seed[2]]
        spread = float(np.mean(np.linalg.norm(topb[:, :2] - seed[:2], axis=1)))
        allc = sel[:, :2].mean(0)
        core = sel[np.linalg.norm(sel[:, :2] - allc, axis=1) < .02]
        dip = float(zt - np.median(core[:, 2])) if len(core) > 20 else 0.
        inverted = spread < .034 and dip < .025          # upright bowl: the middle (cavity) is 40+ mm below the rim; inverted: ~8 mm (foot ring vs bottom disc)
        foot = np.r_[seed[:2], float(np.median(topb[:, 2])) - .004 - .0013]
        return dict(kind='inverted' if inverted else 'upright', foot=foot, spread_mm=spread * 1000, dip_mm=dip * 1000)

    # -------------------------------------------------------------- recognition
    def recognize_sink(self, cache=None):
        m, d = self.model, self.data
        self.cam = K.Cam(m, 'sinkcam', SS.SINKCAM_FOVY)
        rgb, depth = self.cam.capture(d, self.rng)
        SD.mkdir(parents=True, exist_ok=True)
        png = SD / 'sink_counter.png'
        Image.fromarray(rgb).save(png)
        if cache and Path(cache).exists():
            obj, meta = json.loads(Path(cache).read_text()), dict(seconds=None, cached=str(cache))
        else:
            K.SCHEMA_BAK, K.PROMPT_BAK = K.SCHEMA, K.PROMPT
            K.SCHEMA, K.PROMPT = SCHEMA, PROMPT
            obj, meta = K.ask_gpt(png, SD, tag='sink')
            K.SCHEMA, K.PROMPT = K.SCHEMA_BAK, K.PROMPT_BAK
        self.gpt_meta, self.gpt = meta, (obj or dict(items=[]))
        self.rgb0 = rgb
        return self.gpt

    def _has(self, nm):
        try:
            self.model.body(nm)
            return True
        except KeyError:
            return False

    def match(self, est_pos, cat):
        names = [f'{cat}{i}' for i in range(3) if self._has(f'{cat}{i}')]
        pos = {nm: (self.data.site(f'{nm}_center').xpos if cat == 'plate' else self.data.site(f'{nm}_foot').xpos).copy() for nm in names}
        nm = min(names, key=lambda k: np.linalg.norm(pos[k][:2] - est_pos[:2]))
        return nm, float(np.linalg.norm(pos[nm] - est_pos) * 1000)

    # -------------------------------------------------------------- finish the job (full_pipeline skills)
    def pod_state(self):
        return dict(in_groove=True, note='no detergent pod in this scene: pod check not applicable')

    def finish_job(self):
        d = self.data
        self.lift_clear()
        M.Hud.items += [['Push basket in', None, ''], ['Close door', None, ''], ['Press QUICK, then POWER', None, '']]
        n0 = len(M.Hud.items) - 3
        self.opt.rail.update(push_basket=self.set_base(0.48), close_door=self.set_base(0.40), press_buttons=self.set_base(0.48))
        self.set_base(M.BASE_INSERT)
        self.rec.view = np.array([0.20, 0.05, 1.08, 1.5, 150., -25.], float)
        for k, (name, fn, cap) in enumerate([('push_basket', self.step_push_basket, 'Push the loaded basket in (full_pipeline skill)'),
                                              ('close_door', self.step_close_door, 'Close the door along its hinge arc'),
                                              ('press_buttons', self.step_buttons, 'Press QUICK, then POWER (wrist F/T stop at 3 N)')]):
            M.Hud.caption = cap
            if name == 'press_buttons':
                self.rec.view = np.array([0.05, 0.05, 1.30, 0.95, 150., -12.], float)
            try:
                out = self.run_step(name, fn)
                self.finish[name] = dict(ok=True, **{k_: v for k_, v in out.items() if not isinstance(v, (dict, list))})
                M.Hud.items[n0 + k][1] = True
            except PL.StepFail as e:
                self.finish[name] = dict(ok=False, reason=str(e))
                M.Hud.items[n0 + k][1] = False
                self.recover()
                break
        running = all(self.finish.get(n_, {}).get('ok') for n_ in ('push_basket', 'close_door', 'press_buttons'))
        if running:                                        # panel display (visual only): QUICK + POWER lit, RUNNING
            for b in ('quick', 'power'):
                bid = self.model.body(f'dw/btn_{b}_body').id
                for g in range(self.model.body_geomadr[bid], self.model.body_geomadr[bid] + self.model.body_geomnum[bid]):
                    self.model.geom_rgba[g] = [.25, .95, .40, 1]
        self.finish['panel'] = 'RUNNING (QUICK)' if running else 'not started'
        M.Hud.caption = ('Panel: QUICK program RUNNING  面板：快速洗 运行中' if running else 'Program NOT started: ' +
                         '; '.join(f'{k_} failed' for k_, v in self.finish.items() if isinstance(v, dict) and not v.get('ok')))
        self.lift_clear(1.30)
        self.goto('home', np.array([0.45, 0.25, 1.30]), hand_frame([0, 0, -1.], [0, 1., 0]), PL.OPEN, speed=.25)
        self.rec.view = np.array([0.05, 0.05, 1.20, 1.3, 155., -15.], float)
        self.hold('running', 6.0)
        if self.keyframes_dir:
            self.rec.still(d, Path(self.keyframes_dir) / '97_panel_running.png')

    # -------------------------------------------------------------- skills
    def pick_lean(self):
        """pipeline.pick_rack with the perceived (origin, P) of a plate leaning on the basin wall. The plate slides along the
        wall edge during the lift, so the in-hand slip gate is 20 mm (rack PoC: 5 mm); the insert stops on wrist-F/T
        touchdown, which absorbs slip along the plate plane."""
        self.rack_slip_tol = .03
        return self.pick_rack()

    def run_sink(self, cache=None):
        m, d = self.model, self.data
        self.ref['cmd'] = self.tcp()
        M.Hud.items = []
        # dump: dishes fall into the sink (real-time video), wide view
        self.rec.view = np.array([-0.15, 0.62, 0.86, 1.05, 180., -52.], float)
        M.Hud.caption = 'Dirty dishes dumped into the sink (dropped, settling under physics)  脏碗盘被丢进水槽'
        self.substep('')
        M.SPEED = 1
        self.hold('dump', 2.5, PL.OPEN)
        M.SPEED = 3
        self.rec.view = np.array([-0.12, 0.0, 0.95, 2.85, 180., -48.], float)
        M.Hud.caption = ('Messy sink: 3 plates + 2 bowls (this layout: no flat plate, no upright bowl)' if SS.LAYOUT == 'full'
                         else 'Messy sink next to the dishwasher: 3 plates, 3 bowls, kitchen items')
        self.hold('overview', 4.5, PL.OPEN)
        if self.keyframes_dir:
            self.rec.still(d, Path(self.keyframes_dir) / '00_sink.png')
        g = self.recognize_sink(cache)
        items = g['items']
        npick = sum(1 for it in items if it['pick_order'] > 0)
        card = poly_card(self.rgb0, items, f'GPT recognition: {len(items)} items, {npick} dishes to take out of the sink, in this order  '
                                           f'| green = pick  amber = safe, not loaded  red = hand-wash')
        Image.fromarray(card).save(SD / 'card.png')
        im = Image.fromarray(self.rgb0).convert('RGB')
        dd = ImageDraw.Draw(im, 'RGBA')
        dd.rectangle((0, K.CAM_H - 70, K.CAM_W, K.CAM_H), fill=(14, 18, 24, 225))
        gs = self.gpt_meta.get('seconds')
        dd.text((16, K.CAM_H - 62), f'GPT (codex exec, ChatGPT subscription) is recognising the sink ...  {gs if gs else "cached"} s, fast-forwarded',
                fill=(255, 225, 130), font=K._font(24))
        dd.text((16, K.CAM_H - 28), 'Input: this overhead camera image only (no ground truth).  输入：仅此俯视相机图像', fill=(220, 220, 220), font=K._font(18))
        self.card_frames(np.asarray(im), 2.5)
        self.card_frames(card, 8.0)
        self.rec.view = M.VIEW.copy()
        M.Hud.items = []
        rows, picks, n_plate, n_bowl = [], [], 0, 0
        self.plate_slot, self.bowl_spot = {}, {}
        self.use_perc = {}
        self.looks = [dict(look=1, n_items=len(items), n_pick=npick, seconds=self.gpt_meta.get('seconds'))]
        attempted = []
        look = 1
        while len(rows) < 8:
            if look > 1:                                     # LOOK AGAIN: new image, new GPT call on what is left
                rgb, depth = self.cam.capture(d, self.rng)
                X0, _ = K.height_map(self.cam, d, depth)
                left_pts = int(self.in_basin(X0).sum())
                if left_pts < 400:
                    break
                png = SD / f'sink_look{look}.png'
                Image.fromarray(rgb).save(png)
                K.SCHEMA_BAK, K.PROMPT_BAK = K.SCHEMA, K.PROMPT
                K.SCHEMA, K.PROMPT = SCHEMA, PROMPT
                obj, meta = K.ask_gpt(png, SD, tag=f'sink_look{look}')
                K.SCHEMA, K.PROMPT = K.SCHEMA_BAK, K.PROMPT_BAK
                if obj is None:
                    break
                items = obj['items']
                npick = sum(1 for it in items if it['pick_order'] > 0)
                self.looks.append(dict(look=look, n_items=len(items), n_pick=npick, seconds=meta.get('seconds')))
                im = Image.fromarray(rgb).convert('RGB')
                dd = ImageDraw.Draw(im, 'RGBA')
                dd.rectangle((0, K.CAM_H - 50, K.CAM_W, K.CAM_H), fill=(14, 18, 24, 225))
                dd.text((16, K.CAM_H - 42), f'Look #{look}: new image -> GPT again ({meta.get("seconds")} s, fast-forwarded)  再看一次', fill=(255, 225, 130), font=K._font(24))
                self.card_frames(np.asarray(im), 1.2)
                c2 = poly_card(rgb, items, f'Look #{look}: GPT sees {npick} dish(es) still to take out  第{look}次识别：剩余{npick}件')
                Image.fromarray(c2).save(SD / f'card_look{look}.png')
                self.card_frames(c2, 3.5)
            for it in items:                               # GPT sometimes calls an upside-down bowl a 'cup': same skill
                it['skill'] = 'plate' if it['category'] == 'plate' else ('bowl' if it['category'] in ('bowl', 'cup', 'mug') else None)
            def fpt(box):
                x0_, y0_, x1_, y1_ = box
                rp, Rc_ = self.cam.pose(d)
                ray_ = Rc_ @ np.array([((x0_ + x1_) / 2 - self.cam.cx) / self.cam.f, -((y0_ + y1_) / 2 - self.cam.cy) / self.cam.f, -1.])
                return (rp + ray_ * (SS.FLOOR_Z + .05 - rp[2]) / ray_[2])[:2]
            cand = sorted([it for it in items if it['pick_order'] > 0 and it['in_sink'] and it['skill']
                           and all(np.linalg.norm(fpt(it['box']) - a_) > .06 for a_ in attempted)], key=lambda it: it['pick_order'])
            if not cand:
                break
            it = cand[0]
            attempted.append(fpt(it['box']))
            look += 1
            k = len(rows)
            picks.append(it)
            M.Hud.items.append([f"{it['label_en']}", None, ''])
            row = dict(gpt=it, look=look - 1)
            _, X = self.depth()                               # re-perceive (depth) right before the pick
            cat = it['skill']
            tag = f"Pick #{it['pick_order']} {it['label_en']} / {it['label_zh']}"
            try:
                if cat == 'plate':
                    e = self.est_plate(X, it['box'])
                    if e is None or e['kind'] == 'flat':
                        raise HandOver('lies flat on the basin floor: no exposed rim to pinch', e)
                    nm, err = self.match(e['centre'], 'plate')
                    row.update(match=nm, perceived=e['centre'].tolist(), err_mm=round(err, 2))
                    if n_plate >= 3:
                        raise HandOver('no free plate slot left in this load', e)
                    i = int(nm[-1])
                    self.pname, self.pid = nm, m.body(nm).id
                    self.slot_k = PLATE_SLOTS_SINK[n_plate]
                    self.plate_slot[i] = self.slot_k
                    use = err < SIDE['plate']
                    row['pose_source'] = 'perceived' if use else 'GT fallback'
                    if use:
                        self.est = dict(origin=e['origin'], P=e['P'])
                    else:                                          # GT fallback, same frame convention (y = in-plane up)
                        n_ = d.xmat[self.pid].reshape(3, 3)[:, 2].copy()
                        u_ = unit(np.array([0, 0, 1.]) - n_[2] * n_)
                        self.est = dict(origin=d.xpos[self.pid].copy(), P=np.column_stack([np.cross(u_, n_), u_, n_]))
                    M.Hud.caption = f'{tag} → slot {self.slot_k + 1}   [{row["pose_source"]} pose]'
                    self.lift_clear()
                    self.opt.rail['pick_plate'] = self.set_base(float(self.est['origin'][1]) - .04)
                    self.run_step(f'pick_{nm}', self.pick_lean)
                    M.Hud.caption = f'{tag} → slot {self.slot_k + 1}  (carry + insert upright between the tines)'
                    self.opt.rail['insert_plate'] = self.set_base(M.BASE_INSERT)
                    self.run_step(f'insert_{nm}', self.step_insert)
                    n_plate += 1
                    row['loaded'] = f'plate slot {self.slot_k + 1}'
                else:
                    e = self.est_bowl(X, it['box'])
                    if e is None:
                        raise HandOver('not found in the depth image', e)
                    nm, err = self.match(e['foot'], 'bowl')
                    row.update(match=nm, perceived=e['foot'].tolist(), err_mm=round(err, 2), spread_mm=round(e['spread_mm'], 1))
                    if e['kind'] != 'inverted':
                        raise HandOver('opening up (sits on a plate): needs a regrasp/flip this gripper cannot do', e)
                    if n_bowl >= 2:
                        raise HandOver('no free bowl spot left in this load', e)
                    j = int(nm[-1])
                    self.bowl_spot[j] = n_bowl
                    use = err < SIDE['bowl']
                    row['pose_source'] = 'perceived' if use else 'GT fallback'
                    self.perc[nm] = dict(pos=e['foot'], err_mm=err, use=use)
                    self.use_perc[nm] = use
                    M.Hud.caption = f'{tag} → bowl spot {n_bowl + 1}   [{row["pose_source"]} pose]'
                    self.lift_clear()
                    self.do_bowl(j, n_bowl + 1)
                    n_bowl += 1
                    row['loaded'] = f'bowl spot {n_bowl}'
                M.Hud.items[k][1] = True
            except HandOver as h:
                row['handed_to_person'] = h.reason
                M.Hud.items[k][1] = False
                M.Hud.items[k][0] += '  → person'
                M.Hud.caption = f'{tag}: hand to a person – {h.reason}'
                self.hold('handover_note', 6.0)
            except PL.StepFail as e:
                row['failed'] = str(e)
                if row.get('match', '').startswith('plate'):
                    self.plate_slot.pop(int(row['match'][-1]), None)
                elif row.get('match', '').startswith('bowl'):
                    self.bowl_spot.pop(int(row['match'][-1]), None)
                M.Hud.items[k][0] += '  (grasp failed, left in sink)'
                M.Hud.items[k][1] = False
                self.log_fn(f'FAIL {it["label_en"]}: {e}')
                self.recover()
            rows.append(row)
        self.finish = {}
        if getattr(self, 'full', False):
            self.finish_job()
        else:
            M.Hud.caption = 'Done: robot returns to park, everything settles (final poses measured)'
            self.lift_clear()
            self.set_base(M.BASE_INSERT)
            self.transfer('home', M.PARK0, hand_frame([0, 0, -1.], [0, 1., 0]), PL.OPEN, speed=.25)
            self.hold('final_settle', 2.0)
        # final measured state
        table = []
        for i, k_ in self.plate_slot.items():
            self.slot_k = k_
            ok, st = self.plate_check(i)
            table.append(dict(item=f'plate{i}', target=f'plate slot {k_ + 1}', ok=bool(ok), **{k: st[k] for k in
                              ('bottom_offset_mm', 'plane_tilt_deg', 'speed_mm_s', 'wrong_tines', 'other_contacts')}))
        for j, sp in self.bowl_spot.items():
            ok, st = self.bowl_check(j)
            table.append(dict(item=f'bowl{j}', target=f'bowl spot {sp + 1}', ok=bool(ok), **st))
        # anything knocked out of the sink?
        outside = []
        for nm in [x_ for x_ in [f'plate{i}' for i in range(3)] + [f'bowl{j}' for j in range(3)] if self._has(x_)]:
            p = d.xpos[m.body(nm).id]
            in_sink = SS.BX0 - .01 < p[0] < SS.BX1 + .01 and SS.BY0 - .01 < p[1] < SS.BY1 + .01 and p[2] < S.COUNTER_Z + .05
            loaded = any(r_['item'] == nm for r_ in table)
            if not in_sink and not loaded:
                outside.append(dict(item=nm, pos=np.round(p, 3).tolist()))
        moved = {nm: round(float(np.linalg.norm(d.xpos[m.body(nm).id] - self.clutter0[nm][0])) * 1000, 2) for nm in K.CLUTTER}
        for k_, it in enumerate(picks):
            for t_ in table:
                if rows[k_].get('match') == t_['item']:
                    M.Hud.items[k_][1] = t_['ok']
        M.Hud.left = [f"{r_['gpt']['label_en']} / {r_['gpt']['label_zh']}: {r_['handed_to_person']}" for r_ in rows if 'handed_to_person' in r_] + \
                     [f"{r_['gpt']['label_en']} / {r_['gpt']['label_zh']}: grasp failed, left in the sink for a person" for r_ in rows if 'failed' in r_] + \
                     [f"{it['label_en']} / {it['label_zh']}: {it['reason_en']}" for it in items if it['pick_order'] <= 0 and it['in_sink']]
        if not getattr(self, 'full', False):
            self.rec.view = np.array([0.12, 0.17, 1.06, 0.95, 180., -55.], float)
        n_ok = sum(t_['ok'] for t_ in table)
        M.Hud.caption = f'Loaded basket: {n_ok} of {len(picks)} dishes from the sink; the rest is handed to a person'
        self.substep('')
        self.hold('final_view', 9.0)
        if self.keyframes_dir:
            self.rec.still(d, Path(self.keyframes_dir) / '98_loaded_basket.png')
        M.Hud.final = True
        M.Hud.all_ok = n_ok == len(picks)
        M.Hud.items = [x for x in M.Hud.items]
        M.Hud.caption = f'{n_ok} of {len(picks)} loaded (measured from physics); {len(picks) - n_ok} handed to a person'
        self.hold('final', 18.0)
        if self.keyframes_dir:
            self.rec.still(d, Path(self.keyframes_dir) / '99_final.png')
        self.rec.close()
        self.cam.close()
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(d.warning) if w.number}
        unexpected = [dict(step=k[0], phase=k[1], robot=k[2], other=k[3], max_force_N=round(v, 2)) for k, v in self.contact_log.items()]
        return dict(table=table, rows=rows, looks=self.looks, finish=self.finish, layout=SS.LAYOUT, outside_sink=outside, clutter_moved_mm=moved, gpt_meta=self.gpt_meta, gpt=self.gpt,
                    steps=self.steps, sim_time_s=round(d.time, 2), wall_s=round(time.monotonic() - self.t_wall0, 1), warnings=warnings,
                    max_penetration_mm=round(self.metrics['max_penetration_mm'], 3), max_pen_where=self.metrics['max_pen_where'],
                    unexpected_robot_contacts=unexpected, video_frames=self.rec.frames)


class HandOver(Exception):
    def __init__(self, reason, est=None):
        super().__init__(reason)
        self.reason = reason


SS.COUNTER_Z_TOP = S.COUNTER_Z + .08
SS.__dict__.setdefault('COUNTER_Z_TOP', S.COUNTER_Z + .08)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--cache', default=None)
    ap.add_argument('--tag', default='')
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--full', action='store_true', help='all-graspable layout + push basket, close door, QUICK + POWER')
    a = ap.parse_args()
    SS.SEED = a.seed
    if a.full:
        SS.LAYOUT = 'full'
    SD.mkdir(parents=True, exist_ok=True)
    video = None if a.no_video else OUT / f'sink{"_full" if a.full else ""}_demo{a.tag}.tmp.mp4'
    kf = None if a.no_video else ROOT / ('frames_sink_full' if a.full else 'frames_sink')
    if kf:
        kf.mkdir(exist_ok=True)
    r = Sink(video=video, keyframes_dir=kf, log=lambda s: print(s, flush=True))
    r.full = a.full
    res = r.run_sink(a.cache)
    (OUT / f'result_sink{"_full" if a.full else ""}{a.tag}.json').write_text(json.dumps(res, indent=1, default=float))
    for row in res['rows']:
        print('ROW', json.dumps({k: v for k, v in row.items() if k != 'gpt'}, default=float), row['gpt']['label_en'])
    for t in res['table']:
        print('TABLE', json.dumps(t, default=float))
    print('outside', res['outside_sink'], 'clutter', res['clutter_moved_mm'])
    print('sim', res['sim_time_s'], 'wall', res['wall_s'], 'warn', res['warnings'], 'pen', res['max_penetration_mm'], res['max_pen_where'])


if __name__ == '__main__':
    main()
