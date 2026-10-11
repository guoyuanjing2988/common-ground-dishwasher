#!/usr/bin/env python3
"""Cluttered-counter suction demo: GPT recognition (codex exec) + depth perception of the piles + suction loading.

  python kitchen_demo.py capture     # sim: settle the cluttered scene, save wall RGB + depth + camera pose + GT (scoring)
  python kitchen_demo.py plan        # codex exec on the wall image -> out/kitchen/plan.json (no sim)
  python kitchen_demo.py run         # sim + video: opening, recognition card, perception, loading, end card
"""
from __future__ import annotations

import json
import math
import sys
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import kitchen as K
import suction_demo as SD

SD.EXTRA_BUILD[:] = [K.add_clutter, K.add_wallcam]
KOUT = K.KOUT
FONT = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
W, H = SD.W, SD.H
OPEN_VIEW = '-0.05 0.62 0.98 1.95 215 -38'      # opening/end view: whole counter run + basket
XY_TOL, Z_TOL = .012, .006                      # perceived cup point accepted if within 12 mm (xy) / 6 mm (z) of truth


def gt_cup(dm, body):
    R, p = dm.pose(body)
    loc = np.array([0, 0, .010]) if body.startswith('plate') else np.array([0, 0, SD.BOWL_IN_BOTTOM])
    return p + R @ loc


def clutter_state(dm):
    out = {}
    for it in K.CLUTTER:
        R, p = dm.pose(it['name'])
        out[it['name']] = dict(p=p.tolist(), up=R[:, 2].tolist())
    return out


# ------------------------------------------------------------------ stage 1: capture
def capture():
    KOUT.mkdir(parents=True, exist_ok=True)
    dm = SD.Demo(video_path=None)
    c0 = clutter_state(dm)
    dm.hold(1.5)
    cam = K.Camera(dm.m)
    rgb, depth = cam.capture(dm.d, np.random.default_rng(0))
    Image.fromarray(rgb).save(KOUT / 'wall.png')
    np.save(KOUT / 'depth.npy', depth.astype(np.float32))
    cp, cR = cam.pose(dm.d)
    c1 = clutter_state(dm)
    moved = {k: round(float(np.linalg.norm(np.subtract(c1[k]['p'], c0[k]['p']))) * 1000, 2) for k in c0}
    tilt = {k: round(math.degrees(math.acos(min(1., c1[k]['up'][2]))), 2) for k in c1}
    json.dump(dict(cam_p=cp.tolist(), cam_R=cR.tolist(), gt_plate_top=gt_cup(dm, 'plate2').tolist(), gt_bowl_top=gt_cup(dm, 'bowl1').tolist(),
                   settle_moved_mm=moved, settle_tilt_deg=tilt), open(KOUT / 'capture.json', 'w'), indent=1)
    # opening still in the video view
    v = SD.Video(dm.m, None)
    v.cam.lookat[:] = [float(x) for x in OPEN_VIEW.split()[:3]]
    v.cam.distance, v.cam.azimuth, v.cam.elevation = [float(x) for x in OPEN_VIEW.split()[3:]]
    v.r.update_scene(dm.d, camera=v.cam, scene_option=v.opt)
    Image.fromarray(v.r.render()).save(KOUT / 'opening_raw.png')
    print('settle moved mm', moved, 'tilt', tilt)
    print('ncon', dm.d.ncon)


# ------------------------------------------------------------------ stage 2: GPT
def plan():
    obj, meta = K.gpt_recognise(KOUT / 'wall.png')
    json.dump(dict(plan=obj, meta=meta), open(KOUT / 'plan.json', 'w'), indent=1, ensure_ascii=False)
    print(json.dumps(meta))
    if obj:
        for it in obj['items']:
            print(it['id'], it['label_en'], it['label_zh'], it['count'], it['dishwasher_safe'], it['target'], it['box'], '|', it['reason_en'])


# ------------------------------------------------------------------ stage 3: run
def find_pile(items, kind):
    keys = ('plate',) if kind == 'plates' else ('bowl',)
    cands = [it for it in items if any(k in it['label_en'].lower() for k in keys) and it['dishwasher_safe']]
    cands.sort(key=lambda it: -it['count'])
    return cands[0] if cands else None


class KitchenDemo(SD.Demo):
    def __init__(self, video_path, plan_obj, log=print):
        super().__init__(video_path=video_path, keyframes=True, log=log)
        self.plan = plan_obj
        self.cam = K.Camera(self.m)
        self.rng = np.random.default_rng(1)
        self.perc = {}
        self.boxes = {'plates': find_pile(plan_obj['items'], 'plates'), 'bowls': find_pile(plan_obj['items'], 'bowls')}

    def perceive(self, body):
        kind = 'plates' if body.startswith('plate') else 'bowls'
        it = self.boxes[kind]
        truth = gt_cup(self, body)
        if it is None:
            return truth, dict(source='GROUND TRUTH (GPT found no pile)')
        rgb, depth = self.cam.capture(self.d, self.rng)
        cp, cR = self.cam.pose(self.d)
        est = K.perceive_pile(self.cam, cp, cR, depth, it['box'], kind)
        top = np.array(est['top'])
        cup = top + (np.array([-SD.PLATE_CUP_OFF, 0, 0]) if kind == 'plates' else 0)
        # perceived object pose (round items: yaw unobservable and irrelevant): plate origin = disc top - 10 mm,
        # bowl origin = inside bottom - 4 mm (catalogue geometry)
        self.obj_perc = (np.eye(3), top - [0, 0, .010 if kind == 'plates' else SD.BOWL_IN_BOTTOM])
        truth_cup = truth + (np.array([-SD.PLATE_CUP_OFF, 0, 0]) if kind == 'plates' else 0)
        err = cup - truth_cup
        exy, ez = float(np.linalg.norm(err[:2])), float(err[2])
        ok = exy < XY_TOL and abs(ez) < Z_TOL
        info = dict(source='PERCEIVED (GPT box + depth)' if ok else 'GROUND TRUTH fallback (perception error too large)',
                    err_xy_mm=round(exy * 1000, 1), err_z_mm=round(ez * 1000, 1), z_source=est['z_source'],
                    rim_radius_mm=round(est['rim_radius'] * 1000, 1), n_points=est['n_points'])
        self.log(f'  perceive {body}: {info}')
        if not ok:
            R_, p_ = self.pose(body)
            self.obj_perc = (R_, p_)
        return (cup if ok else truth_cup), info

    def suction_pick(self, body, cup_local, R_tool, label, p_cup_override=None):
        cup, info = self.perceive(body)
        self.perc[body] = info
        tag = 'perceived' if info['source'].startswith('PERCEIVED') else 'GT fallback'
        self.hud.caption = f'{label}: pose {tag} (err {info.get("err_xy_mm", 0):.1f} mm xy, {info.get("err_z_mm", 0):+.1f} mm z)'
        self.hold(.6)
        self.POSE_TAG = 'perceived pose' if tag == 'perceived' else 'GROUND-TRUTH fallback'
        return super().suction_pick(body, cup_local, R_tool, label, p_cup_override=cup - [0, 0, .002],
                                    obj_pose_override=self.obj_perc)

    # -------------------------------------------------------------- cards (frames written straight to ffmpeg)
    def card_frames(self, img, seconds):
        if self.video and self.video.proc:
            b = np.asarray(img.convert('RGB')).tobytes()
            for _ in range(int(seconds * 30)):
                self.video.proc.stdin.write(b)
                self.video.frames += 1

    def recognition_card(self, meta):
        f_s, f, f_b = (ImageFont.truetype(FONT, s) for s in (15, 19, 26))
        wall = Image.open(KOUT / 'wall.png').convert('RGB')
        sc = 0.60
        im = wall.resize((int(W * sc), int(H * sc)))
        canvas = Image.new('RGB', (W, H), (16, 20, 26))
        canvas.paste(im, (0, 56))
        d = ImageDraw.Draw(canvas, 'RGBA')
        d.rectangle((0, 0, W, 48), fill=(30, 36, 46))
        d.text((14, 10), 'Recognition: GPT via codex exec (ChatGPT subscription) on the wall-camera image', fill='white', font=f_b)
        rows = []
        for it in self.plan['items']:
            x0, y0, x1, y1 = [v * sc for v in it['box']]
            load = it['target'] in ('plate_rack', 'bowl_area')
            col = (70, 220, 110) if load else (240, 80, 70)
            d.rectangle((x0, y0 + 56, x1, y1 + 56), outline=col, width=3)
            d.rectangle((x0, y0 + 56 - 20, x0 + 26, y0 + 56), fill=col)
            d.text((x0 + 5, y0 + 56 - 19), it['id'], fill='black', font=f_s)
            rows.append((it, col))
        x, y = int(W * sc) + 12, 60
        for it, col in rows:
            n = f' ×{it["count"]}' if it['count'] > 1 else ''
            d.text((x, y), f'{it["id"]}  {it["label_en"]}{n} / {it["label_zh"]}', fill=col, font=f)
            tgt = {'plate_rack': '→ plate rack', 'bowl_area': '→ bowl area', 'leave_hand_wash': 'leave: hand-wash',
                   'leave_not_for_dishwasher': 'leave: not a dish'}[it['target']]
            r_ = it['reason_en'] if len(it['reason_en']) <= 52 else it['reason_en'][:50] + '…'
            d.text((x + 22, y + 21), f'{tgt} · {r_}', fill=(205, 210, 215), font=f_s)
            y += 44
            if y > H - 40:
                break
        sec = meta.get('seconds', 0)
        d.rectangle((0, H - 92, int(W * sc), H), fill=(60, 50, 10))
        d.text((14, H - 84), f'>>  GPT call took {sec:.0f} s (real time), fast-forwarded', fill=(255, 220, 110), font=f_b)
        d.text((14, H - 46), f'green = load into the dishwasher, red = leave on the counter · boxes are GPT\'s own pixel boxes',
               fill=(230, 230, 230), font=f_s)
        canvas.save(SD.FRAMES / 'k01_recognition.png')
        self.card_frames(canvas, 7.0)

    def end_card(self, table):
        f_s, f, f_b = (ImageFont.truetype(FONT, s) for s in (16, 20, 26))
        self.video.cam.lookat[:] = [float(x) for x in OPEN_VIEW.split()[:3]]
        self.video.cam.distance, self.video.cam.azimuth, self.video.cam.elevation = [float(x) for x in OPEN_VIEW.split()[3:]]
        self.video.r.update_scene(self.d, camera=self.video.cam, scene_option=self.video.opt)
        img = Image.fromarray(self.video.r.render())
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 0, W, 44), fill=(18, 22, 28, 230))
        n_ok = sum(r['ok'] for r in table)
        d.text((14, 8), f'Done: {n_ok}/{len(table)} loaded by suction (poses measured from physics) · counter items left in place',
               fill='white', font=f_b)
        bx, by, bw = 14, 56, 610
        d.rectangle((bx, by, bx + bw, by + 30 + 26 * len(table)), fill=(14, 18, 24, 215))
        d.text((bx + 12, by + 6), 'Loaded', fill=(90, 230, 120), font=f)
        for i, r in enumerate(table):
            d.text((bx + 12, by + 32 + 26 * i), f'{"✓" if r["ok"] else "✗"} {r["item"]} → {r["target"]}  ({r["pick_pose"]})',
                   fill=(90, 230, 120) if r['ok'] else (255, 90, 80), font=f_s)
        left = [it for it in self.plan['items'] if it['target'] not in ('plate_rack', 'bowl_area')]
        extra = [it for it in self.plan['items'] if it['target'] in ('plate_rack', 'bowl_area') and it is not self.boxes['plates']
                 and it is not self.boxes['bowls']]
        y0 = H - 60 - 24 * (len(left) + len(extra)) - 34
        d.rectangle((bx, y0, bx + 860, H - 56), fill=(14, 18, 24, 215))
        d.text((bx + 12, y0 + 6), 'Left on the counter, and why', fill=(255, 150, 120), font=f)
        y = y0 + 34
        for it in left:
            d.text((bx + 12, y), f'• {it["label_en"]} / {it["label_zh"]}: {it["reason_en"][:70]}', fill=(235, 235, 235), font=f_s)
            y += 24
        for it in extra:
            d.text((bx + 12, y), f'• {it["label_en"]} / {it["label_zh"]}: dishwasher-safe, but the Ø80 mm pump cannot reach its inside '
                   f'bottom → hand to a person', fill=(255, 220, 130), font=f_s)
            y += 24
        d.rectangle((0, H - 48, W, H), fill=(18, 22, 28, 235))
        d.text((16, H - 40), f'Clutter undisturbed: max displacement {self.clutter_moved:.1f} mm, max tilt change {self.clutter_tilt:.1f}°',
               fill=(255, 225, 130), font=f_b)
        img.save(SD.FRAMES / 'k99_end.png')
        self.card_frames(img, 6.0)

    def run_kitchen(self, meta):
        h = self.hud
        c0 = clutter_state(self)
        h.items = [[f'Bowl {j + 1} → bowl spot {j + 1}', None, ''] for j in range(2)] + \
                  [[f'Plate {i + 1} → slot {SD.PLATE_SLOTS[i] + 1}', None, ''] for i in range(3)]
        # 1. opening on the cluttered counter
        v = self.video
        main_view = (v.cam.lookat.copy(), v.cam.distance, v.cam.azimuth, v.cam.elevation)
        v.cam.lookat[:] = [float(x) for x in OPEN_VIEW.split()[:3]]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = [float(x) for x in OPEN_VIEW.split()[3:]]
        h.caption = 'A cluttered counter: 3 plates, 2 nested bowls and 8 other kitchen items'
        self.hold(4.0)
        self.key('k00_opening')
        # 2. recognition card (GPT wait fast-forwarded)
        self.recognition_card(meta)
        v.cam.lookat[:], v.cam.distance, v.cam.azimuth, v.cam.elevation = main_view
        fails = []
        for j in range(2):
            try:
                self.do_bowl(j, 2)
                h.items[j][1] = self.bowl_check(j)[0]
            except SD.Fail as e:
                fails.append(f'bowl {j}: {e}')
                h.items[j][1] = False
                self.recover()
        for i in range(3):
            try:
                self.do_plate(i, 3)
                h.items[2 + i][1] = self.plate_check(i)[0]
            except SD.Fail as e:
                fails.append(f'plate {i}: {e}')
                h.items[2 + i][1] = False
                self.recover()
        h.caption = 'Done: robot parks, everything settles (final poses measured)'
        self.lift_to(1.30)
        try:
            self.jmove(np.array([.42, .30, 1.45]), SD.R_DOWN, 3.0)   # park high over the cart: basket in full view
        except SD.Fail:
            self.jmove(np.array([.20, .62, 1.30]), SD.R_DOWN, 3.0)
        self.hold(2.0)
        table = []
        for j in range(2):
            ok, st = self.bowl_check(j)
            body = f'bowl{1 - j}'
            table.append(dict(item=f'bowl {j + 1}', target=f'bowl spot {j + 1}', ok=ok, pick_pose=self.perc.get(body, {}).get('source', '?').split(' (')[0],
                              perception=self.perc.get(body), **self.per_item.get(body, {}), **st))
            h.items[j][1] = ok
            h.items[j][2] = f'pos err {st["pos_err_mm"]:.1f} mm, tilt {st["tilt_deg"]:.0f}°'
        for i in range(3):
            ok, st = self.plate_check(i)
            body = f'plate{2 - i}'
            table.append(dict(item=f'plate {i + 1}', target=f'slot {SD.PLATE_SLOTS[i] + 1}', ok=ok, pick_pose=self.perc.get(body, {}).get('source', '?').split(' (')[0],
                              perception=self.perc.get(body), **self.per_item.get(body, {}), **st))
            h.items[2 + i][1] = ok
            h.items[2 + i][2] = f'rim-in-slot dx {st["bottom_dx_mm"]:+.1f} mm, lean {st["lean_deg"]:.1f}°'
        c1 = clutter_state(self)
        moved = {k: float(np.linalg.norm(np.subtract(c1[k]['p'], c0[k]['p']))) * 1000 for k in c0}
        tilt = {k: math.degrees(math.acos(min(1., float(np.dot(c1[k]['up'], c0[k]['up']))))) for k in c0}
        self.clutter_moved, self.clutter_tilt = max(moved.values()), max(tilt.values())
        h.final = True
        h.caption = f'{sum(r["ok"] for r in table)}/{len(table)} placed · peak suction {self.stats["peak_suction_N"]:.0f} N (cap {SD.F_CAP:.0f} N) · seal losses {len(self.stats["seal_losses"])}'
        self.hold(3.0)
        self.key('k98_checklist')
        h.final = False
        self.end_card(table)
        self.video.close()
        return dict(table=table, fails=fails, clutter_moved_mm={k: round(v, 2) for k, v in moved.items()},
                    clutter_tilt_change_deg={k: round(v, 2) for k, v in tilt.items()}, gpt_meta=meta,
                    sim_time_s=round(self.d.time, 2), wall_s=round(time.monotonic() - self.t_wall, 1),
                    peak_suction_N=round(self.stats['peak_suction_N'], 2), seal_losses=self.stats['seal_losses'],
                    unexpected_robot_contacts=self.stats['unexpected_contacts'], max_penetration_mm=round(self.stats['max_pen_mm'], 2),
                    video_frames=self.video.frames)


def run():
    P = json.load(open(KOUT / 'plan.json'))
    dm = KitchenDemo(SD.OUT / 'kitchen_suction_demo.mp4', P['plan'], log=lambda s: print(s, flush=True))
    res = dm.run_kitchen(P['meta'])
    (KOUT / 'result.json').write_text(json.dumps(res, indent=1, default=float, ensure_ascii=False))
    for r in res['table']:
        print(json.dumps(r, default=float, ensure_ascii=False))
    print('fails', res['fails'])
    print('clutter moved', res['clutter_moved_mm'], 'tilt', res['clutter_tilt_change_deg'])
    print('unexpected', res['unexpected_robot_contacts'], 'seal', res['seal_losses'], 'sim', res['sim_time_s'], 'wall', res['wall_s'],
          'frames', res['video_frames'])


if __name__ == '__main__':
    dict(capture=capture, plan=plan, run=run)[sys.argv[1]]()
