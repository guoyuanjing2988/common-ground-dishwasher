"""One trial: seeded cluttered counter -> GPT plan (one codex call on the wall + room images) -> robot executes what its
skills allow (plates), hands the rest to a human / defers / leaves unsafe items -> closes up and presses GPT's program
+ POWER -> scoring against ground truth. One MuJoCo process, sequential.

  python run_sort.py --seed 3 --lang en [--video out/sort_demo.mp4] [--keyframes frames/]
Writes out/runs/<tag>/result.json.
"""
from __future__ import annotations

import argparse
import json
import math
import time
import traceback
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import cfg
import items as IT
import planner as PN
import sort_robot as SRB
import sort_scene as SS
import truth as TR
from sort_robot import PL, PC, S, plan, dw

GAP = SRB.PLATE_SLOTS
PICK_BOX = dict(x=(-0.25, -0.115), y=(0.365, 0.59))      # flat-pick zone (reach_scan.py), small tolerance
FONT = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Unicode.ttf', 18)


def plan_overlay(rgb, p, tmatch=None):
    im = Image.fromarray(rgb).copy()
    d = ImageDraw.Draw(im, 'RGBA')
    for it in p['items']:
        col = (60, 220, 100) if it['dishwasher_safe'] else (240, 80, 70)
        x0, y0, x1, y1 = it['box']
        d.rectangle((x0, y0, x1, y1), outline=col, width=3)
        lab = f"{it['id']} {it['category']} {'SAFE' if it['dishwasher_safe'] else 'NOT safe'} -> {it['slot']}"
        tw = d.textlength(lab, font=FONT)
        d.rectangle((x0, max(0, y0 - 22), x0 + tw + 8, max(0, y0 - 22) + 22), fill=(15, 20, 28, 210))
        d.text((x0 + 4, max(0, y0 - 22)), lab, fill=col, font=FONT)
    return np.asarray(im)


class Trial:
    def __init__(self, seed, lang, video=None, keyframes=None, tag=None):
        self.seed, self.lang = seed, lang
        self.tag = tag or f's{seed}_{lang}'
        self.outdir = cfg.ROOT / 'out' / 'runs' / self.tag
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.items_gt, self.decal_seed = SS.sample_scene(seed)
        self.keyframes = Path(keyframes) if keyframes else None
        if self.keyframes:
            self.keyframes.mkdir(parents=True, exist_ok=True)
        title = f'GPT sorts a cluttered counter  (seed {seed}, goal: {lang})'
        self.r = SRB.SortRobot(self.items_gt, self.decal_seed, video=video, run_tag=self.tag, outdir=self.outdir, title=title)
        self.r.opt.approach_deg = 45.          # pinch approach (FLAT PoC alternative, 10/10 there); see GPT_SORT.md
        self.log = []
        self.t0 = time.time()
        self.kf = 0

    def keyframe(self, name):
        if self.keyframes is not None and self.r.rec.writer is not None:
            self.kf += 1
            self.r.rec.still(self.r.data, self.keyframes / f'{self.kf:02d}_{name}.png')

    def note(self, **kw):
        kw['t_sim'] = round(self.r.data.time, 1)
        self.log.append(kw)
        print(json.dumps(kw, default=str)[:400], flush=True)

    def call(self, name, fn, *a, **k):
        r = self.r
        try:
            out = fn(*a, **k) or {}
        except SRB.StepFail as e:
            out = dict(_ok=False, _reason=str(e), code=getattr(e, 'metrics', {}).get('code', 'FAILED'))
        except Exception as e:  # noqa: BLE001  a crash is a failed skill, reported as such
            out = dict(_ok=False, _reason=f'exception {e!r}', code='EXCEPTION', tb=traceback.format_exc()[-800:])
        clean = {k_: v for k_, v in out.items() if not isinstance(v, np.ndarray)}
        self.note(skill=name, **clean)
        return clean

    # ------------------------------------------------------------------ perception helpers (robot side, no ground truth)
    def item_footprints(self, depth, p):
        """Metric footprint of every GPT-listed item: depth points inside its box standing > 5 mm above the counter.
        Used for the slide-corridor check. Returns {id: (centre_xy, radius)}."""
        cam, d = self.r.cam, self.r.data
        out = {}
        for it in p['items']:
            x0, y0, x1, y1 = [int(round(v)) for v in it['box']]
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(PC.W - 1, x1), min(PC.H - 1, y1)
            if x1 - x0 < 3 or y1 - y0 < 3:
                continue
            vs, us = np.mgrid[y0:y1:2, x0:x1:2]
            us, vs = us.ravel().astype(float), vs.ravel().astype(float)
            dd = depth[vs.astype(int), us.astype(int)]
            X = cam.backproject(d, us, vs, dd)
            X = X[(X[:, 2] > S.COUNTER_Z + .005) & (X[:, 2] < S.COUNTER_Z + .4) & (X[:, 0] < .02)]
            if len(X) < 10:
                continue
            c = np.median(X[:, :2], 0)
            rad = float(np.percentile(np.linalg.norm(X[:, :2] - c, axis=1), 95))
            out[it['id']] = (c, rad)
        return out

    def slide_target(self, est, gid, fps, free_spots):
        """A plate outside the pick zone: pick a free spot in the zone reachable by a straight slide whose swept band
        (plate + hand) is clear of the other perceived items. Returns (target_xy | None, reason)."""
        c0 = (est['origin'])[:2]
        for tgt in free_spots:
            tgt = np.asarray(tgt, float)
            u = tgt - c0
            L = float(np.linalg.norm(u))
            if L < 1e-3:
                return tgt, 'already there'
            u /= L
            blocked = None
            for oid, (oc, orad) in fps.items():
                if oid == gid:
                    continue
                rel = oc - c0
                s_ = float(np.clip(rel @ u, -.16, L))           # include the hand behind the far rim
                dist = float(np.linalg.norm(rel - s_ * u))
                if dist < IT.R_OUT + orad + .02:
                    blocked = oid
                    break
            if blocked is None:
                return tgt, f'slide {L*1000:.0f} mm to the pick zone'
        return None, 'no clear straight slide into the pick zone'

    # ------------------------------------------------------------------ main
    def run(self):
        r = self.r
        res = dict(seed=self.seed, lang=self.lang, tag=self.tag, items_gt=SS.describe_gt(self.items_gt))
        r.start()
        rec = r.rec
        rec.set_view(*SRB.VIEWS['counter'], cut=True)
        r.hold('look', 1.0)
        self.keyframe('start')
        # ---- GPT plan
        rgb, depth, wall_png = r.capture('plan_wall')
        r.cam.r.update_scene(r.data, camera='roomcam', scene_option=PL.scene_option(hide_sites=True))
        room = r.cam.r.render().copy()
        room_png = self.outdir / 'plan_room.png'
        Image.fromarray(room).save(room_png)
        tboxes = TR.true_boxes(r.model, r.data, r.cam, [it['name'] for it in self.items_gt], PC.W, PC.H)   # scoring only
        r.substep('GPT (codex exec) plans from the wall-camera image ...')
        t = time.time()
        p, meta = PN.plan(wall_png, room_png, self.lang, f'{self.tag}_plan', PC.W, PC.H)
        res['plan_meta'] = meta
        if p is None:
            res['error'] = f'planner failed: {meta.get("error")}'
            self.finish(res, None)
            return res
        res['plan'] = p
        (self.outdir / 'plan.json').write_text(json.dumps(p, indent=1, ensure_ascii=False))
        ov = plan_overlay(rgb, p)
        Image.fromarray(ov).save(self.outdir / 'plan_overlay.png')
        fe = PN.feasibility(p)
        res['plan_feasibility'] = fe
        order = sorted([it for it in p['items'] if it['slot'] in GAP or it['slot'].startswith('B')], key=lambda it: it['order'])
        status = {}
        for it in p['items']:
            s = it['slot']
            status[it['id']] = 'hand wash' if s == 'none' else ('deferred' if s == 'defer' else 'queued')
        rec.table = [dict(id=it['id'], cat=f"{it['id']} {it['category']}", safe=it['dishwasher_safe'], slot=it['slot'],
                          order=it['order'], status=status[it['id']]) for it in sorted(p['items'], key=lambda x: (x['order'] == 0, x['order']))]
        rec.program = f"{p['program']} - {p['program_reason']}"
        rec.card(r.data, ov, f"GPT plan in {meta['seconds']:.0f} s (simulation paused). Green = dishwasher-safe, red = not.\n"
                 f"Program: {p['program']} - {p['program_reason']}\n{p['summary']}", 6.)
        self.keyframe('gpt_plan')

        def set_status(gid, st):
            status[gid] = st
            for row in rec.table:
                if row['id'] == gid:
                    row['status'] = st

        # ---- open up
        ex = dict()
        ok_door = self.call('open_door', r.sk_open_door).get('_ok')
        self.keyframe('door_open')
        ok_basket = ok_door and self.call('pull_basket', r.sk_pull_basket).get('_ok')
        self.keyframe('basket_out')
        r.sk_stow()
        loaded_zone_spots = []
        executed = {}
        if ok_basket:
            for it in order:
                gid = it['id']
                if it['slot'].startswith('B'):
                    why = ('no validated grasp for bowls / mugs: a rim pinch would leave the hand under the up-side-down item '
                           'in the basket (see GPT_SORT.md)') if it['category'] in ('bowl', 'mug') else f'{it["category"]} in a bowl position'
                    set_status(gid, f'to human ({it["slot"]})')
                    executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=why)
                    self.note(item=gid, action='handed_to_human', reason=why)
                    continue
                if it['slot'] not in SRB.ROBOT_SLOTS:
                    why = f'{it["slot"]} is out of the robot\'s reach (only P4 is robot-loadable)'
                    set_status(gid, f'to human ({it["slot"]})')
                    executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=why)
                    self.note(item=gid, action='handed_to_human', reason=why)
                    continue
                # plate position the robot can reach
                k = GAP[it['slot']]
                set_status(gid, f'▶ {it["slot"]}')
                r.substep(f'{gid}: depth fit inside the GPT box')
                _, depth, _ = r.capture(f'{gid}_depth')
                est, info = r.plate_from_box(it['box'], depth, tag=gid)
                if not info['ok']:
                    why = f'depth fit inside the GPT box does not look like a flat 8-inch plate (r {info["radius_mm"]} mm)'
                    set_status(gid, 'to human')
                    executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=why, fit=info)
                    self.note(item=gid, action='handed_to_human', reason=why, fit=info)
                    continue
                c = np.array(info['centre'])
                name = self.match_true_plate(c)                     # which simulated body the robot is about to handle
                if name is None:
                    executed[gid] = dict(action='handed_to_human', reason='no plate body at the perceived position')
                    set_status(gid, 'to human')
                    continue
                if not (PICK_BOX['x'][0] < c[0] < PICK_BOX['x'][1] and PICK_BOX['y'][0] < c[1] < PICK_BOX['y'][1]):
                    fps = self.item_footprints(depth, p)
                    spots = loaded_zone_spots + [(-.17, .47), (-.17, .40), (-.17, .56)]
                    tgt, why = self.slide_target(est, gid, fps, spots)
                    if tgt is None:
                        set_status(gid, 'to human')
                        executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=f'outside the pick zone; {why}')
                        self.note(item=gid, action='handed_to_human', reason=why)
                        continue
                    r.set_active(name)
                    r.percepts['plate'] = dict(est=est, t=float(r.data.time), valid=True)
                    o = self.call(f'slide_{gid}', r.sk_slide_plate, tgt)
                    r.sk_stow()
                    if not o.get('_ok'):
                        set_status(gid, 'FAILED slide')
                        executed[gid] = dict(action='failed', stage='slide', reason=o.get('_reason'))
                        continue
                    # predicted position -> predicted box -> depth fit (no Codex call)
                    pc = np.array([tgt[0], tgt[1], S.COUNTER_Z + S.MID_Z])
                    th = np.linspace(0, 2 * math.pi, 48)
                    uv = r.cam.project(r.data, pc + (IT.R_OUT + .01) * np.c_[np.cos(th), np.sin(th), 0 * th])
                    est, info = r.plate_from_box([*uv.min(0), *uv.max(0)], None, tag=f'{gid}_slid')
                    if not info['ok']:
                        set_status(gid, 'FAILED slide')
                        executed[gid] = dict(action='failed', stage='refit after slide', fit=info)
                        continue
                okp, whyp = r.plan_pick(est['origin'])
                if not okp:
                    set_status(gid, 'to human')
                    executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=f'pick not plannable: {whyp}')
                    self.note(item=gid, action='handed_to_human', reason=whyp)
                    continue
                # will the carry + insert be plannable? (nominal in-hand transform; avoids picking a plate it cannot place)
                wp, nom = plan.nominal_flat_sequence(tuple(est['origin'][:2]), approach_deg=r.opt.approach_deg)
                pn = nom['ppl_end'] + nom['Rpl_end'] @ nom['ph_p'] + [0, 0, .12]
                Rn = nom['Rpl_end'] @ nom['Rh_p']
                best, diag = r.plan_insert(k, pn, Rn, (nom['Rh_p'], nom['ph_p']), r.basket_m())
                if best is None:
                    set_status(gid, 'to human')
                    executed[gid] = dict(action='handed_to_human', slot=it['slot'], reason=f'insert into {it["slot"]} not plannable', diag=diag)
                    self.note(item=gid, action='handed_to_human', reason='insert not plannable', diag=diag)
                    continue
                r.rec.set_view(*SRB.VIEWS['counter'])
                r.substep(f'{gid}: pick the plate (edge push, re-perceive, pinch, tilt, lift)')
                o = self.call(f'pick_{gid}', r.pick_plate, name, est)
                self.keyframe(f'pick_{gid}')
                if not o.get('_ok'):
                    set_status(gid, 'FAILED pick')
                    executed[gid] = dict(action='failed', stage='pick', reason=o.get('_reason'), code=o.get('code'))
                    held, _, _ = r.plate_in_hand()
                    if held:
                        break
                    r.sk_stow()
                    continue
                o = self.call(f'insert_{gid}', r.insert_plate, k)
                self.keyframe(f'insert_{gid}')
                if not o.get('_ok'):
                    set_status(gid, 'FAILED insert')
                    executed[gid] = dict(action='failed', stage='insert', reason=o.get('_reason'), code=o.get('code'), body=name)
                    if r.plate_in_hand()[0]:
                        break                      # never open the hand over the cart: stop loading, report
                else:
                    set_status(gid, f'loaded {it["slot"]}')
                    executed[gid] = dict(action='robot_loaded', slot=it['slot'], gap=k, body=name)
                    loaded_zone_spots.append(tuple(c[:2]))
                r.sk_stow()
        res['executed'] = executed
        # ---- close up, program
        holding = r.plate_in_hand()[0]
        if ok_basket and not holding:
            self.call('push_basket', r.push_basket_front)
            self.keyframe('basket_in')
            self.call('close_door', r.sk_close_door)
            self.keyframe('door_closed')
            prog = p['program']
            r.substep(f'press {prog.upper()} then POWER')
            self.call(f'press_{prog}', r.sk_press, prog)
            self.call('press_power', r.sk_press, 'power')
            self.keyframe('program_started')
            r.sk_stow()
        r.substep('done: ' + p['summary'])
        r.hold('end', 2.)
        self.finish(res, p, tboxes)
        return res

    def match_true_plate(self, c):
        """The simulated body behind a perceived plate centre (bookkeeping for the plate-site tracking used in pick
        diagnostics; nearest plate body within 6 cm). Control uses only the perceived pose."""
        best, bd = None, .06
        for nm, it in self.r.items_gt.items():
            if it['type'] != 'plate':
                continue
            dd = float(np.linalg.norm(self.r.item_pos(nm)[:2] - c[:2]))
            if dd < bd:
                best, bd = nm, dd
        return best

    # ------------------------------------------------------------------ scoring
    def finish(self, res, p, tboxes=None):
        r = self.r
        gt = {it['name']: it for it in self.items_gt}
        sc = {}
        if p is not None:
            m, extra_g, missed_t = TR.match(p['items'], tboxes)
            inv = {v: k for k, v in m.items()}
            per = []
            for nm, it in gt.items():
                gid = inv.get(nm)
                g = next((x for x in p['items'] if x['id'] == gid), None)
                row = dict(name=nm, type=it['type'], safe_true=it['dishwasher_safe'], soil_true=it['soil'], greasy_true=it['greasy'],
                           gpt_id=gid, detected=g is not None)
                if g is not None:
                    row.update(category=g['category'], category_ok=g['category'] == TR.CAT_OF_TYPE[it['type']],
                               safe_pred=g['dishwasher_safe'], safe_ok=g['dishwasher_safe'] == it['dishwasher_safe'],
                               soil_pred=g['soil'], soil_ok=g['soil'] == it['soil'], greasy_pred=g['greasy'],
                               greasy_ok=g['greasy'] == it['greasy'], slot=g['slot'], order=g['order'],
                               box_iou=round(TR.iou(g['box'], tboxes[nm]), 2))
                else:
                    row.update(safe_ok=False, category_ok=False)
                per.append(row)
            sc['per_item'] = per
            sc['n_items'] = len(per)
            sc['detected'] = sum(r_['detected'] for r_ in per)
            sc['hallucinated_gpt_items'] = extra_g
            sc['safe_correct'] = sum(r_['safe_ok'] for r_ in per)
            sc['category_correct'] = sum(r_['category_ok'] for r_ in per)
            sc['soil_correct'] = sum(r_.get('soil_ok', False) for r_ in per if r_['safe_true'])
            sc['greasy_correct'] = sum(r_.get('greasy_ok', False) for r_ in per if r_['safe_true'])
            sc['n_safe'] = sum(r_['safe_true'] for r_ in per)
            # program ground truth
            load = [gt[r_['name']] for r_ in per if r_['detected'] and r_.get('slot') in PN.POS and r_['safe_true']]
            gt_prog = TR.program_rule(load)
            safe_all = [it for it in self.items_gt if it['dishwasher_safe']]
            n_cap_p, n_cap_b = 4, 3
            cap_set = [it for it in safe_all if it['type'] == 'plate'][:n_cap_p] + [it for it in safe_all if it['type'] != 'plate'][:n_cap_b]
            sc['program'] = dict(chosen=p['program'], gt_on_planned_load=gt_prog, ok=p['program'] == gt_prog,
                                 planned_load=[it['name'] for it in load],
                                 gt_on_all_safe_items=TR.program_rule(safe_all),
                                 capacity_feasible_all=len(cap_set) == len(safe_all))
            # slot plan feasibility: structural (plan only) + against ground truth
            gt_issues = []
            for r_ in per:
                if r_['detected'] and r_.get('slot') in PN.POS:
                    if not r_['safe_true']:
                        gt_issues.append(f"{r_['name']} ({r_['type']}, not dishwasher-safe) put in {r_['slot']}")
                    elif r_['slot'].startswith('P') and r_['type'] != 'plate':
                        gt_issues.append(f"{r_['name']} ({r_['type']}) in plate position {r_['slot']}")
                    elif r_['slot'].startswith('B') and r_['type'] not in ('bowl', 'mug'):
                        gt_issues.append(f"{r_['name']} ({r_['type']}) in bowl position {r_['slot']}")
                if r_['safe_true'] and (not r_['detected'] or r_.get('slot') == 'none'):
                    gt_issues.append(f"{r_['name']} ({r_['type']}) dishwasher-safe but not planned (slot {r_.get('slot')})")
            sc['plan_feasible_structural'] = res.get('plan_feasibility', {}).get('feasible')
            sc['plan_issues_structural'] = res.get('plan_feasibility', {}).get('issues')
            sc['plan_issues_vs_truth'] = gt_issues
            sc['plan_feasible'] = bool(sc['plan_feasible_structural'] and not gt_issues)
        # physical outcome
        ex = res.get('executed', {})
        phys = []
        for gid, e in ex.items():
            if e.get('action') == 'robot_loaded':
                s10 = r.score_plate_in_gap(e['body'], e['gap'])
                s10['seated_15'] = bool(s10['plane_tilt_deg'] < 15 and abs(s10['bottom_offset_mm']) < (dw.TINE_PITCH / 2 - dw.TINE_R) * 1000
                                        and not s10['wrong_tines'] and not s10['other_contacts'] and s10['speed_mm_s'] < 5)
                phys.append(dict(gpt_id=gid, body=e['body'], slot=e['slot'], where=r.where_is(e['body']), **s10))
        sc['robot_loaded'] = phys
        sc['n_robot_loaded_claimed'] = sum(e.get('action') == 'robot_loaded' for e in ex.values())
        sc['n_seated_15'] = sum(x['seated_15'] for x in phys)
        sc['n_seated_10'] = sum(x['in_slot'] for x in phys)
        sc['handed_to_human'] = {g: e for g, e in ex.items() if e.get('action') == 'handed_to_human'}
        sc['failed'] = {g: e for g, e in ex.items() if e.get('action') == 'failed'}
        moved = {}
        for nm in gt:
            if any(x['body'] == nm for x in phys):
                continue
            mm = r.item_moved_mm(nm)
            if mm > 10:
                moved[nm] = dict(moved_mm=mm, where=r.where_is(nm), type=gt[nm]['type'])
        sc['unintended_item_moves'] = moved
        sc['door_closed'] = abs(math.degrees(float(r.data.qpos[r.door_j]))) < 1.
        sc['basket_in'] = float(r.data.qpos[r.basket_j]) < .005
        sc['panel'] = r.panel_state()
        sc['program_running'] = bool(r.panel['running'] and p is not None and r.panel['program'] == p['program'])
        unexp = [dict(phase=k[1], robot=k[2], other=k[3], N=round(v, 1)) for k, v in r.contact_log.items()]
        sc['unplanned_contacts'] = unexp[:30]
        sc['max_unplanned_contact_N'] = max([u['N'] for u in unexp], default=0.)
        sc['max_penetration_mm'] = round(r.metrics['max_penetration_mm'], 2)
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(r.data.warning) if w.number}
        sc['mujoco_warnings'] = warnings
        # GPT cost
        calls = ([res['plan_meta']] if 'plan_meta' in res else []) + [mm for mm in r.codex_meta if mm.get('purpose') != 'DEBUG_TRUTH']
        sc['gpt_calls'] = len(calls)
        sc['gpt_seconds'] = round(sum(c_.get('seconds', 0) for c_ in calls), 1)
        sc['gpt_plan_seconds'] = res.get('plan_meta', {}).get('seconds')
        sc['gpt_tokens'] = sum(c_.get('tokens', 0) for c_ in calls)
        sc['gpt_backoffs'] = sum(len(c_.get('backoffs', [])) for c_ in calls)
        sc['reperception'] = [dict(seconds=c_.get('seconds')) for c_ in r.codex_meta]
        sc['sim_s'] = round(float(r.data.time), 1)
        sc['wall_s'] = round(time.time() - self.t0, 1)
        res['score'] = sc
        res['log'] = self.log
        r.close()
        (self.outdir / 'result.json').write_text(json.dumps(res, indent=1, default=str, ensure_ascii=False))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--lang', default='en', choices=['en', 'zh'])
    ap.add_argument('--video')
    ap.add_argument('--keyframes')
    ap.add_argument('--tag')
    a = ap.parse_args()
    tr = Trial(a.seed, a.lang, video=a.video, keyframes=a.keyframes, tag=a.tag)
    res = tr.run()
    s = res.get('score', {})
    print('SUMMARY', json.dumps({k: s.get(k) for k in ('n_items', 'detected', 'safe_correct', 'category_correct', 'program', 'plan_feasible',
                                                         'n_robot_loaded_claimed', 'n_seated_15', 'n_seated_10', 'gpt_calls', 'gpt_seconds',
                                                         'sim_s', 'wall_s', 'program_running', 'unintended_item_moves')}, default=str))
