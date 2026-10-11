"""Simulated robot for the sorting task: ../gpt_control SimRobot (M1 cart + Panda, honest skills, panel logic, collision
checker) on the cluttered-counter scene, extended to many items and parameterised basket slots.

Physics: mj_step only; no weld / connect / mocap / teleport; item qpos is written only by the scene builder at reset.
Robot-side signals used for control: joint + finger encoders, wrist F/T, wall RGB-D camera (+ Codex boxes), appliance
sensors (simulated access: door angle, basket travel, panel). Ground truth only in `score_*` and `_scoring` fields.
"""
from __future__ import annotations

import math
import textwrap
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

import cfg
import gpt
import items as IT
import sort_scene as SS

import sim_robot as SR  # noqa: E402  (../gpt_control, imported after cfg so perception uses this task's camera size)
from sim_robot import PL, PC, S, dw, plan  # noqa: E402
from util import angle_deg, rot_axis, rot_z  # noqa: E402

StepFail = PL.StepFail
PARK, RD = SR.PARK, SR.RD

# ---------------------------------------------------------------- basket slots (task design, see GPT_SORT.md)
# plate rack: 9 tines per row -> 8 gaps, k = 0 (back) .. 7 (front, behind the pull bar). A plate needs every other gap
# (the hand / fingers are ~29 mm wide across the plate, the tine pitch is 28 mm) -> physical positions P1-P4 = gaps 0, 2,
# 4, 6. The robot reaches only P4 (gap 6, the pipeline's validated slot): insert_scan.py shows the 0.2 m wide hand meets
# the control band / top above the basket for gaps <= 4, and slot tests of a second robot position (gaps 5 / 7) gave
# unreliable seating (14-22 deg lean, plate leaving its gap) and hand contact with the first plate (GPT_SORT.md).
PLATE_SLOTS = {'P1': 0, 'P2': 2, 'P3': 4, 'P4': 6}         # name -> gap index
ROBOT_SLOTS = {'P4'}
BOWL_SLOTS = ('B1', 'B2', 'B3')                             # bowl / mug area (left side): no validated skill


def slot_local_x(k):
    return (dw.TINE_X[k] + dw.TINE_X[k + 1]) / 2


def plate_slot_xyz(k, basket):
    """Seated-plate centre in gap k (generalises plan.plate_slot, which is gap 6)."""
    return np.array([S.DW_POS[0] + dw.BASKET_X0 + basket + slot_local_x(k), S.DW_POS[1] + sum(dw.PLATE_ROWS_Y) / 2,
                     S.DW_POS[2] + dw.BASKET_Z + dw.WIRE_R + dw.PLATE_R + .001])


# ====================================================================== video with a per-item decision HUD
class SortRecorder(PL.Recorder):
    """1280x720 H.264 streamed to ffmpeg (imageio-ffmpeg pipe), HUD = GPT's per-item plan + live status."""

    def __init__(self, model, path, title):
        super().__init__(model, path, title)
        self.title = title
        self.table = []          # rows: dict(id, cat, safe, slot, order, status)
        self.program = ''
        self.goal = ''
        self.footer2 = ''

    def hud(self, img, t):
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 0, 1280, 40), fill=(18, 22, 28, 235))
        d.text((14, 7), self.title[:70], fill='white', font=self.font_b)
        d.text((1100, 10), f't = {t:6.1f} s {self.speed}x', fill=(200, 210, 220), font=self.font_s)
        d.rectangle((0, 40, 1280, 64), fill=(18, 22, 28, 170))
        d.text((14, 43), (self.substep or '')[:150], fill=(255, 220, 120), font=self.font_s)
        if self.table:
            x0, y0 = 836, 72
            h = 22 * (len(self.table) + 3) + 8
            d.rectangle((x0 - 8, y0 - 6, 1276, y0 + h), fill=(10, 30, 45, 210))
            d.text((x0, y0), 'GPT plan (from the camera image)', fill=(120, 220, 255), font=self.font_s)
            d.text((x0, y0 + 20), '#  item              safe  slot   status', fill=(170, 180, 190), font=self.font_s)
            for i, r in enumerate(self.table):
                st = r.get('status', '')
                col = {'loaded': (90, 230, 120), 'FAILED': (255, 90, 80), 'to human': (255, 190, 90),
                       'deferred': (200, 170, 255), 'hand wash': (170, 175, 180)}.get(st.split(' ')[0] if st else '', (230, 235, 240))
                if st.startswith('▶'):
                    col = (255, 220, 120)
                safe = 'yes' if r['safe'] else 'NO'
                txt = f"{str(r['order']) if r['order'] else '-':>2} {r['cat'][:17]:17s} {safe:4s} {r['slot']:6s} {st}"
                d.text((x0, y0 + 42 + 22 * i), txt[:52], fill=col, font=self.font_s)
            d.text((x0, y0 + 42 + 22 * len(self.table) + 4), f'program: {self.program}'[:52], fill=(255, 230, 150), font=self.font_s)
        d.rectangle((0, 676, 1280, 720), fill=(18, 22, 28, 235))
        d.text((14, 679), 'Simulation (MuJoCo, physics only). GPT = Codex CLI (ChatGPT subscription) on the wall-camera image.',
               fill=(210, 214, 220), font=self.font_s)
        d.text((14, 698), (self.footer2 or 'Soil is a visual decal; water and washing are not simulated. Dishwasher internals estimated.')[:150],
               fill=(210, 214, 220), font=self.font_s)
        return img

    def card(self, data, image, text, seconds=3.):
        if self.writer is None:
            return
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        base = Image.fromarray(self.r.render())
        inset = Image.fromarray(image).resize((768, 432))
        base.paste(inset, (40, 90))
        d = ImageDraw.Draw(base, 'RGBA')
        lines = []
        for para in text.split('\n'):
            lines += textwrap.wrap(para, 84) or ['']
        d.rectangle((40, 526, 808, 530 + 20 * min(len(lines), 6)), fill=(18, 22, 28, 225))
        for k, line in enumerate(lines[:6]):
            d.text((48, 528 + 20 * k), line, fill=(255, 230, 150), font=self.font_s)
        frame = np.asarray(self.hud(base, data.time))
        for _ in range(int(30 * seconds)):
            self.writer.append_data(frame)
            self.frames += 1


VIEWS = dict(PL.VIEWS)
VIEWS.update(counter=((-0.15, 0.62, 0.95), 1.55, 200, -38), overview=((0.0, 0.40, 1.0), 2.2, 205, -24),
             insert=((0.08, 0.06, 1.12), 0.85, 140, -20))


# ====================================================================== robot
class SortRobot(SR.SimRobot):
    def __init__(self, items_gt, decal_seed, video=None, run_tag='sort', outdir=None, log=print, title='GPT sorts the counter'):
        SR.build_scene = lambda mount=None, plate_xy=None, plate_yaw=0., with_objects=True, with_dishwasher=True, grip_k=800., \
            grip_limit=70., layout=None: SS.build_model(items_gt, decal_seed, mount, grip_k, grip_limit)
        super().__init__(faults={}, plate_off=(0, 0, 0), video=None, seed=decal_seed, run_tag=run_tag, outdir=outdir, log=log)
        self.items_gt = {it['name']: it for it in items_gt}
        self.item_ids = {n: self.model.body(n).id for n in self.items_gt}
        self.item_bodies = set(self.item_ids.values())
        self.active = None
        if video is not None:
            self.rec = SortRecorder(self.model, video, title)
        self.loaded = {}                 # item name -> plate gap index (what the robot believes it loaded)
        self.basket_est = None
        self.gpt_s = []                  # codex seconds (perception) inside skills
        self.insert_aim = {}             # gap -> aim offset (m) toward the plate's convex side (pipeline: 3 mm)
        self.insert_clear = .006         # check_path: hand / links must keep >= 2 mm from the dishwasher on the insert path

    # ------------------------------------------------------------------ item bookkeeping
    def set_active(self, name):
        self.active = name
        self.pid = self.item_ids[name]

    def plate_c(self):
        if self.active is None:
            return super().plate_c()
        return self.data.site(f'{self.active}_center').xpos.copy()

    def item_pos(self, name):
        return self.data.xpos[self.item_ids[name]].copy()

    # ------------------------------------------------------------------ perception
    def snapshot(self, tag):
        """Wall camera RGB + noisy depth (arm must be stowed). Returns (rgb, depth, png path)."""
        return self.capture(tag)

    def plate_from_box(self, box, depth=None, tag='plate'):
        """GPT/Codex 2-D box + fresh depth -> metric pose of a flat plate. Clutter-robust variant of
        ../gpt_control estimate_flat_robust (which lost the plate when the dishwasher side wall or a neighbour fell into its
        35 % padded box): counter plane from points at counter height, plate points 3-30 mm above it inside the box,
        then a known-radius disc search: maximise points inside r = 101.5 mm minus 3 x points in the 4-20 mm ring
        outside it (the ring must be empty), refined on the outline. Ground truth is not used."""
        if depth is None:
            _, depth, _ = self.capture(f'{tag}_depth')
        cam, d = self.cam, self.data
        x0, y0, x1, y1 = box
        pad = 40
        u0, v0, u1, v1 = max(0, int(x0 - pad)), max(0, int(y0 - pad)), min(PC.W, int(x1 + pad)), min(PC.H, int(y1 + pad))
        vs, us = np.mgrid[v0:v1, u0:u1]
        us, vs = us.ravel().astype(float), vs.ravel().astype(float)
        dd = depth[vs.astype(int), us.astype(int)]
        ok = dd < 4
        us, vs, dd = us[ok], vs[ok], dd[ok]
        X = cam.backproject(d, us, vs, dd)
        near = np.abs(X[:, 2] - S.COUNTER_Z) < .006                 # known worktop height (calibrated kitchen model)
        c, n, _ = SR.PC.ransac_plane(X[near], thr=.002, iters=200, prefer_normal=np.array([0, 0, 1.]))
        if n[2] < 0:
            n = -n
        h = (X - c) @ n
        inbox = (us >= x0 - 6) & (us <= x1 + 6) & (vs >= y0 - 6) & (vs <= y1 + 6)
        P = X[(h > .003) & (h < .03)]
        Pin = X[(h > .003) & (h < .03) & inbox]
        if len(Pin) < 50:
            return dict(origin=np.zeros(3), P=np.eye(3), radius_fit=0.), dict(ok=False, centre=[0, 0, 0], radius_mm=0., n_points=int(len(Pin)))
        xy, xyin = P[:, :2], Pin[:, :2]
        best = None
        cx0, cy0 = xyin.mean(0)
        for step, span in ((.006, .08), (.0015, .009)):
            g = np.arange(-span, span + 1e-9, step)
            cands = [(cx0 + a, cy0 + b) for a in g for b in g]
            for cc in cands:
                r_ = np.linalg.norm(xy - cc, axis=1)
                sc = int((r_ < IT.R_OUT).sum()) - 3 * int(((r_ > IT.R_OUT + .004) & (r_ < IT.R_OUT + .02)).sum())
                if best is None or sc > best[0]:
                    best = (sc, np.array(cc))
            cx0, cy0 = best[1]
        ctr = best[1]
        # outline refinement: farthest plate point per 5 deg bin within r < R + 6 mm
        rel = xy - ctr
        rr = np.linalg.norm(rel, axis=1)
        m_ = rr < IT.R_OUT + .006
        ang = np.arctan2(rel[m_, 1], rel[m_, 0])
        bins = np.floor((ang + math.pi) / math.radians(5)).astype(int)
        pts = np.array([xy[m_][bins == b_][np.argmax(rr[m_][bins == b_])] for b_ in np.unique(bins)])
        ctr2, r_fit = SR.PC.circle_fit(pts)
        if np.linalg.norm(ctr2 - ctr) < .006 and abs(r_fit - IT.R_OUT) < .006:
            ctr = ctr2
        z_c = c[2] + ((c[:2] - ctr) @ n[:2]) / n[2] if abs(n[2]) > .5 else c[2]
        origin = np.array([ctr[0], ctr[1], z_c + .0003])
        x_ax = np.array([1., 0, 0]) - n[0] * n
        x_ax /= np.linalg.norm(x_ax)
        est = dict(origin=origin, P=np.column_stack([x_ax, np.cross(n, x_ax), n]), radius_fit=float(r_fit), mode='flat',
                   plane_normal=n.tolist(), counter_plane_pt=c.tolist(), disc_score=int(best[0]), outline_points=int(len(pts)))
        okk = abs(r_fit - IT.R_OUT) < .008 and n[2] > .99 and len(pts) > 40
        cen = origin + n * S.MID_Z
        return est, dict(ok=bool(okk), centre=[round(float(v), 4) for v in cen], radius_mm=round(r_fit * 1000, 1),
                         outline_points=int(len(pts)), disc_score=int(best[0]))

    def perceive(self, tag, mode):
        """Re-perception at the counter edge inside the pick skill (overrides SimRobot.perceive): Codex is asked for the
        ONE plate that now overhangs the counter front edge, with the predicted pixel position as a hint (several plates
        are on the counter). Same depth geometry afterwards."""
        rgb, depth, png = self.capture(tag)
        hint = ''
        if getattr(self, 'reperceive_hint', None) is not None:
            u, v = self.cam.project(self.data, self.reperceive_hint)
            hint = f' It was just pushed there by the robot and should be near pixel ({u:.0f}, {v:.0f}).'
        prompt = (f"The attached image is {PC.W}x{PC.H} pixels (x right, y down, origin top-left). It was taken by a camera on "
                  "the kitchen wall looking over a countertop toward a robot arm. Locate the white dinner plate whose rim overhangs "
                  f"the front edge of the counter (the counter edge nearest the robot).{hint} Return pixel coordinates in this image: "
                  "that plate's tight bounding box and its centre. Answer only with the JSON.")
        self.substep(f'Codex call: locate the plate at the counter edge ...')
        if getattr(self, 'debug_truth_boxes', False):      # DEVELOPMENT ONLY (slot / reach tests): box from ground truth
            th = np.linspace(0, 2 * math.pi, 64)
            c = self.plate_c()
            uv = self.cam.project(self.data, c + IT.R_OUT * np.c_[np.cos(th), np.sin(th), 0 * th])
            vlm, meta = dict(found=True, bbox=[*uv.min(0), *uv.max(0)], center=list(self.cam.project(self.data, c)), seconds=0.), \
                dict(purpose='DEBUG_TRUTH', seconds=0.)
        else:
            vlm, meta = gpt.locate(png, prompt, f'{self.run_tag}_{tag}', purpose='perception:plate_edge')
        self.codex_meta.append(meta)
        self.codex_s.append(vlm.get('seconds'))
        if not vlm.get('found'):
            self.fail(f'Codex did not find the plate at the edge ({vlm.get("error", "")})', code='NOT_FOUND')
        est = SR.estimate_flat_robust(self.cam, self.data, depth, vlm)
        est['mode'] = 'flat'
        true_c = self.plate_c()
        est_c = est['origin'] + est['P'][:, 2] * S.MID_Z
        res = dict(codex_s=vlm['seconds'], codex_bbox=[round(v) for v in vlm['bbox']], radius_fit_mm=round(est['radius_fit'] * 1000, 1),
                   scoring_centre_err_mm=round(float(np.linalg.norm(est_c - true_c)) * 1000, 2))
        ov = PC.overlay(rgb, vlm, self.cam, self.data, est)
        ovp = self.outdir / 'codex' / f'{self.run_tag}_{tag}_overlay.png'
        Image.fromarray(ov).save(ovp)
        self.last_overlay = ovp
        self.rec.card(self.data, ov, f'Re-perception at the counter edge: Codex box (yellow) {vlm["seconds"]:.0f} s, sim paused; '
                      f'green = depth circle fit r = {est["radius_fit"]*1000:.1f} mm (plate 101.5 mm)', 2.)
        self.substep('')
        self.reperceive = res
        return est, res

    # ------------------------------------------------------------------ plate: pick (existing skill) + insert into gap k
    def pick_plate(self, name, est):
        """../gpt_control _pick_flat_honest: edge push (8 N, wrist-F/T admittance), Codex re-perception at the edge,
        rim pinch, counter-supported tilt-up, lift. Verdict from finger width + wrist load."""
        self.set_active(name)
        self.percepts['plate'] = dict(est=est, t=float(self.data.time), valid=True)
        org = est['origin']
        pu = plan.push_poses(org)
        self.reperceive_hint = org + pu['target_travel'] * pu['drag_dir'] + [0, 0, S.MID_Z]
        try:
            return self.sk_pick_plate()
        finally:
            self.reperceive_hint = None

    def plan_insert(self, k, p, R, T_hp, basket):
        """Carry + insert joint path for gap k from hand pose (p, R) with in-hand transform T_hp (pipeline.step_insert
        geometry). Also collision-checks the insert segment against the dishwasher model. Returns (best | None, diag)."""
        Rh_p, ph_p = T_hp
        Pn = R @ Rh_p.T
        Rpl, o_est = Pn, p - Pn @ ph_p
        n = Rpl[:, 2]
        slot = plate_slot_xyz(k, basket)
        lift_h = 1.32 - (o_est[2] + S.MID_Z)
        via1 = p + [0, 0, max(0., lift_h)]
        best, diag = None, []
        tgt_n = (np.array([-1., 0, 0]) if n[0] < 0 else np.array([1., 0, 0]))
        ax = np.cross(n, tgt_n)
        Rc = rot_axis(ax, math.asin(min(1., np.linalg.norm(ax)))) if np.linalg.norm(ax) > 1e-9 else np.eye(3)
        for yaw_opt in (0., math.pi):                  # yaw 0 = the pipeline's validated insertion; pi only as fallback
            R_slot = rot_z(yaw_opt) @ Rc @ Rpl
            org = slot - self.insert_aim.get(k, .003) * R_slot[:, 2] - R_slot @ np.array([0, 0, S.MID_Z])
            Rh = R_slot @ Rh_p
            ph = org + R_slot @ ph_p
            via2 = np.array([ph[0], via1[1], via1[2]])
            via3 = np.array([ph[0], ph[1], via1[2]])
            f, qs, wm = self.plan_path([(via2, Rh), (via3, Rh), (ph + [0, 0, .09], Rh), (ph, Rh), (ph + [0, 0, .12], Rh)])
            hit = None
            if qs is not None:
                hit = self.check_path(qs[1:4], fingers=False, links_only_margin=self.insert_clear)
                if hit is None and (best is None or wm > best[0]):
                    best = (wm, yaw_opt, Rh, ph, via2, via3, qs, via1)
            diag.append(dict(yaw=round(math.degrees(yaw_opt)), joint_path=qs is not None, collision=hit))
            if best is not None:
                break
        return best, diag

    def safe_jmove(self, name, p1, R1, grip, q_goal=None):
        """Joint-space re-orientation that is collision-checked INCLUDING the fingers (SimRobot.jmove checks links only;
        the first runs had a finger hit the machine top while turning the plate 180 deg). Tries direct, then via the hub
        pose in front of the machine, then via a raised point; executes exactly the checked joint path."""
        q0 = self.cfg.q.copy()
        q0[self.ik_arm] = self.data.qpos[self.arm_q]
        if q_goal is None:
            q_goal, _, err = self.solve_q(np.asarray(p1, float), R1)
        cands = [[q0, q_goal]]
        p0, R0 = self.tcp()
        for pv, Rv in ((np.array(self.opt.hub) + [0, 0, .08], R1), (np.array(self.opt.hub) + [0, 0, .08], R0),
                       (p0 + [.12, 0, .10], R0), (np.r_[.30, p0[1], 1.40], R0)):
            qv, _, err = self.solve_q(pv, Rv, seeds=[q0[self.ik_arm], q_goal[self.ik_arm]])
            if err < 1e-3:
                cands.append([q0, qv, q_goal])
        for path in cands:
            if all(self.check_path([a, b], fingers=True) is None for a, b in zip(path[:-1], path[1:])):
                if len(path) > 2:
                    self.metrics['carry_rerouted'] = self.metrics.get('carry_rerouted', 0) + 1
                self.exec_qpath(name, path, 2.5, grip)
                self.ref['cmd'] = (np.asarray(p1, float), np.asarray(R1, float))
                return True
        self.metrics['carry_unchecked'] = self.metrics.get('carry_unchecked', 0) + 1
        self.goto(name, p1, R1, grip, speed=.15, wspeed=.5, q_hint=q_goal)
        return False

    def plan_pick(self, org):
        """Kinematic pre-check of the whole flat pick for a plate whose foot-plane centre is `org` (as reach_scan.py):
        push path, then pinch + tilt + lift at the predicted post-push pose. Nothing moves."""
        pu = plan.push_poses(org)
        _, q1, _ = self.plan_path([(pu['p_pre'] + [0, 0, .05], pu['R']), (pu['p_pre'], pu['R']), (pu['p_pre'] + pu['drag'] * pu['drag_dir'], pu['R'])])
        if q1 is None:
            return False, 'no joint path for the edge push'
        hit = self.check_path(q1, fingers=False, links_only_margin=.003)
        if hit is not None:
            return False, f'edge-push path would hit {hit["env_geom"]} with {hit["robot_geom"]}'
        org2 = org + pu['target_travel'] * pu['drag_dir']
        P2 = np.eye(3)
        tcp_g, Rg, _ = plan.pinch_pose(org2, P2, approach_deg=self.opt.approach_deg)
        path0 = plan.tilt_path(P2, org2, n=60)
        Rh, ph = P2.T @ Rg, P2.T @ (tcp_g - org2)
        tp = [(path0[i][2] + path0[i][1] @ ph, path0[i][1] @ Rh) for i in range(0, 61, 10)]
        kp = [(tcp_g - Rg[:, 2] * .12 + [0, 0, .03], Rg), (tcp_g - Rg[:, 2] * .12, Rg), (tcp_g, Rg)] + tp[1:] + [(tp[-1][0] + [0, 0, .12], tp[-1][1])]
        _, q2, _ = self.plan_path(kp)
        if q2 is None:
            return False, 'no joint path for pinch + tilt + lift at the counter edge'
        hit = self.check_path(q2, fingers=False, links_only_margin=.003)
        if hit is not None:
            return False, f'pinch / tilt path would hit {hit["env_geom"]} with {hit["robot_geom"]}'
        return True, ''

    def insert_plate(self, k):
        """pipeline.Runner.step_insert generalised to gap k (validated there for gap 6 only). Carry with the in-hand
        transform measured at the grasp, lower between the tines, stop on wrist-F/T touchdown, release, retract.
        Verdict from robot signals: touchdown felt + hand unloaded after release."""
        Rh_p, ph_p = self.T_hp
        p, R = self.tcp()
        self.ref['cmd'] = (p, R)
        if self.basket_est is None or abs(self.basket_est - self.basket_m()) > .01:
            self.basket_est = self.basket_m()
        best, diag = self.plan_insert(k, p, R, self.T_hp, self.basket_est)
        Pn = R @ Rh_p.T
        self.plate_R_est, self.plate_o_est = Pn, p - Pn @ ph_p
        cur_p, cur_R = p, R
        via1 = best[7] if best is not None else None
        if best is None:
            self.fail(f'no collision-free joint path to carry + insert into gap {k}: {diag}', code='UNREACHABLE')
        wm, yaw_opt, Rh, ph, via2, via3, qs, via1 = best
        self.substep(f'carry the plate to slot gap {k}')
        self.allowed = {self.pid}                     # the carried plate may touch the hand
        self.rec.set_view(*PL.VIEWS['carry'])
        self.goto('carry_up', via1, cur_R, 0., speed=.15)
        self.safe_jmove('carry_out', via2, Rh, 0., qs[0])
        above = ph + [0, 0, .09]
        self.move('carry_across', max(2., 1.875 * float(np.linalg.norm(via3 - via2)) / .15), via3, Rh, 0., qpath=qs[:2])
        self.rec.set_view(*VIEWS['insert'])
        self.move('carry_down', max(2., 1.875 * float(np.linalg.norm(above - via3)) / .12), above, Rh, 0., qpath=qs[1:3])
        self.hold('carry_settle', .5, 0.)
        w0 = float(self.hand_force_world()[2])
        sup = dict(filt=0., cnt=0, at=None)

        def stop():
            s_ = w0 - self.hand_force_world()[2]
            sup['filt'] += .2 * (s_ - sup['filt'])
            sup['peak'] = max(sup.get('peak', 0.), s_)
            near = abs(self.tcp()[0][2] - ph[2]) < .04
            sup['cnt'] = sup['cnt'] + 1 if (sup['filt'] > .5 and near) else 0
            if sup['cnt'] >= 5:
                sup['at'] = self.tcp()[0].copy()
                return True
            return False
        self.substep('insert between the tines (stop on wrist-F/T touchdown)')
        hit = self.move('insert', 2.5, ph + [0, 0, .025], Rh, 0., stop=stop, qpath=qs[2:4])
        if not hit:
            hit = self.move('insert_slow', 4.0, ph - [0, 0, .008], Rh, 0., stop=stop, qpath=qs[3:4])
        stop_p = sup['at'] if hit else ph - [0, 0, .006]
        self.ref['cmd'] = (stop_p, Rh)
        self.substep('release, retract')
        self.allowed = {self.pid}
        self.move('release', 1.0, stop_p, Rh, 200.)
        self.move('retract', 1.5, stop_p + [0, 0, .12], Rh, 200.)
        self.allowed = set()
        self.hold('settle', .8)
        held, w, load = self.plate_in_hand()
        self.T_hp = None
        ok = bool(hit) and load < 1.5
        out = dict(_ok=ok, touchdown_detected=bool(hit), slot_yaw_deg=round(math.degrees(yaw_opt)),
                   carry_plan_margin_deg=round(math.degrees(wm), 1), wrist_load_after_release_N=round(load, 2))
        if ok:
            self.loaded[self.active] = k
        else:
            out.update(code='INSERT_UNCONFIRMED', _reason='no touchdown felt by the wrist F/T' if not hit else 'hand still loaded after release')
        return out

    # ------------------------------------------------------------------ basket in: push the front frame beside the plate rack
    def push_basket_front(self, y_push=-0.12, max_force_N=30.):
        """Push the lower basket in with the closed fingertips on its front top frame wire in front of the bowl area
        (y = -0.12), not on the pull bar: the pull-bar grasp of the pipeline put the fingers against the foot ring of a
        plate standing in the front gap (gap 7). Verdict = basket travel (appliance sensor)."""
        b = self.basket_m()
        if b < .004:
            return dict(_ok=True, basket_m=round(b, 3), note='already in')
        bid = self.model.body('dw/basket').id
        R = SR.hand_frame([-1., 0, 0], [0, 1., 0])
        xf = S.DW_POS[0] + dw.BASKET_X0 + dw.BASKET_HX + b + dw.FRAME_R
        z = S.COUNTER_Z + dw.BASKET_Z + dw.BASKET_WALL_H - .010
        p_pre = np.array([xf + .06, y_push, z])
        p_touch = np.array([xf + .004, y_push, z])
        p_end = np.array([xf - b - .004, y_push, z])
        f, qs, wm = self.plan_path([(p_pre, R), (p_touch, R), (p_end, R)], flips=(1, -1))
        if qs is None:
            self.fail('no joint path for the basket push', code='UNREACHABLE')
        R = R if f > 0 else R @ self.FLIP
        self.rec.set_view(*PL.VIEWS['basket'])
        self.allowed = {bid}
        self.substep('push the basket in: closed fingertips on the front frame (left of the plate rack)')
        # high via point in front of the machine first: the plates standing in the rack top out at z ~1.23
        self.goto('bpush_via', np.array([xf + .16, y_push, 1.32]), R, PL.TOUCH, speed=.3)
        self.goto('bpush_pre', p_pre + [0, 0, .04], R, PL.TOUCH, speed=.25, q_hint=qs[0])
        self.move('bpush_pre', 1.0, p_pre, R, PL.TOUCH, qpath=[qs[0]])
        self.move('bpush_touch', 1.5, p_touch, R, PL.TOUCH, qpath=qs[:2])
        f0 = self.hand_force_world().copy()
        peak = [0.]

        def stop():
            fx = float((self.hand_force_world() - f0)[0])
            peak[0] = max(peak[0], fx)
            return fx > max_force_N and self.basket_m() < .01
        hit = self.move('bpush', max(2.5, 12. * b), p_end, R, PL.TOUCH, stop=stop, qpath=qs[1:])
        pe = self.tcp()[0]
        self.ref['cmd'] = (pe, R)
        self.hold('bpush', .3)
        self.move('bpush_back', 1.2, pe + [.10, 0, 0], R, PL.TOUCH)
        self.goto('bpush_back', pe + [.14, 0, .12], R, PL.TOUCH, speed=.3)
        self.allowed = set()
        self.hold('basket_settle', .5)
        bm = self.basket_m()
        out = dict(_ok=bm < .006, basket_m=round(bm, 3), peak_push_force_N=round(peak[0], 1))
        if not out['_ok']:
            out.update(code='BASKET_NOT_IN', _reason=f'basket at {bm*1000:.0f} mm')
        return out

    # ------------------------------------------------------------------ scoring (ground truth only)
    def score_plate_in_gap(self, name, k):
        """In the slot = lowest rim point between its own two tines (x), within the rows (y), plane < 10 deg from the slot
        plane, touching only its own tines / floor wires / frame (other plates allowed only if they too are in their slots),
        settled, not touching the robot. Same predicate as pipeline.plate_slot_state, for gap k."""
        m, d = self.model, self.data
        bid = self.item_ids[name]
        slot = plate_slot_xyz(k, float(d.qpos[self.basket_j]))
        R = d.xmat[bid].reshape(3, 3)
        nrm = R[:, 2]
        plane_deg = math.degrees(math.acos(min(1., abs(float(nrm[0])))))
        th = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        rim = d.xpos[bid] + (np.outer(np.cos(th), R[:, 0]) + np.outer(np.sin(th), R[:, 1])) * S.R_OUT + R[:, 2] * .0173
        low = rim[np.argmin(rim[:, 2])]
        bx = float(low[0] - slot[0])
        c = d.site(f'{name}_center').xpos
        err = c - slot
        touch = set()
        for i in range(d.ncon):
            con = d.contact[i]
            b1, b2 = int(m.geom_bodyid[con.geom[0]]), int(m.geom_bodyid[con.geom[1]])
            if bid == b1:
                touch.add(m.geom(con.geom[1]).name or m.body(b2).name)
            elif bid == b2:
                touch.add(m.geom(con.geom[0]).name or m.body(b1).name)
        own = {f'dw/plate_tine{r}_{j}' for r in (0, 1) for j in (k, k + 1)}
        wrong = sorted(t for t in touch if t.startswith('dw/plate_tine') and t not in own)
        other = sorted(t for t in touch if not (t.startswith('dw/plate_tine') or t.startswith('dw/bfloor') or t.startswith('dw/frame_b')))
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
        half_gap = dw.TINE_PITCH / 2 - dw.TINE_R
        robot = any(t.startswith('panda/') for t in other)
        in_slot = abs(bx) < half_gap and abs(err[1]) < .03 and plane_deg < 10 and not wrong and not other
        return dict(in_slot=bool(in_slot), bottom_offset_mm=round(bx * 1000, 1), plane_tilt_deg=round(plane_deg, 2),
                    wrong_tines=wrong, other_contacts=other, touching_robot=robot,
                    speed_mm_s=round(float(np.linalg.norm(vel[3:])) * 1000, 2))

    def where_is(self, name):
        """Ground truth location class of an item (scoring): 'basket' / 'counter' / 'floor' / 'elsewhere'."""
        p = self.item_pos(name)
        b = float(self.data.qpos[self.basket_j])
        bx = S.DW_POS[0] + dw.BASKET_X0 + b
        if abs(p[0] - bx) < dw.BASKET_HX + .02 and abs(p[1]) < dw.BASKET_HY + .02 and S.COUNTER_Z + dw.BASKET_Z - .01 < p[2] < S.COUNTER_Z + .45:
            return 'basket'
        if p[0] < .005 and S.COUNTER_Z - .005 < p[2] < S.COUNTER_Z + .3:
            return 'counter'
        if p[2] < .5:
            return 'floor'
        return 'elsewhere'

    def item_moved_mm(self, name):
        it = self.items_gt[name]
        p = self.item_pos(name)
        return round(float(np.linalg.norm(p[:2] - np.array(it['xy']))) * 1000, 1)
