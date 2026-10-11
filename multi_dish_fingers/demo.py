"""Multi-dish loading with CUSTOM wedge + rim-hook fingers (Franka Hand), physics only (mj_step; no welds,
no attach, no teleports). GT object poses are used instead of perception (stated simplification).

usage: demo.py --L 0.107 [--video out/fingers_demo.mp4] [--tag name] [--only plates|bowls]
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import mink
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import dishwasher as dw
import fingers as FG
import scene_f as S
from util import hand_frame, renderer, scene_option, unit, rot_axis

ROOT = Path(__file__).resolve().parent
CDT = .01
MAIN_VIEW = ([0.02, 0.30, 1.08], 1.55, 235, -38)
BASKET_VIEW = ([0.06, 0.04, 1.08], 0.95, 160, -60)
HI = .10                     # plate alignment height above the seated pose: hanging plate clears seated plates
OPEN = 255.
K = 800.


def grip_ctrl(q):            # ctrl 0 -> -GRIP_X (squeeze, force-capped at 70 N), 255 -> +0.04 (finger joint target, m)
    return float(np.clip((q + S.GRIP_X) / (.04 + S.GRIP_X) * 255, 0, 255))


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def ang(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class Rec:
    """Frames streamed to ffmpeg (H.264). Caption bar + checklist."""

    def __init__(self, model, path, W=1280, H=720, snaps=None):
        self.on = path is not None
        self.snaps = snaps
        self.W, self.H = W, H
        if snaps:
            self.r = renderer(model, H, W)
            self.cam = mujoco.MjvCamera()
            self.opt = scene_option()
            fp = '/System/Library/Fonts/Supplemental/Arial.ttf'
            self.font = ImageFont.truetype(fp, 26)
            self.small = ImageFont.truetype(fp, 20)
        self.n = 0
        self.every = 1 / 30
        self.next = 0.
        self.speed = 2.
        self.caption = ''
        self.checks = []
        self.closeup = None
        if not self.on:
            return
        self.W, self.H = W, H
        self.r = renderer(model, H, W)
        self.cam = mujoco.MjvCamera()
        self.set_view(*MAIN_VIEW)
        self.opt = scene_option()
        self.ff = subprocess.Popen(['/opt/homebrew/bin/ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                    '-crf', '23', '-preset', 'veryfast', str(path)], stdin=subprocess.PIPE)
        fp = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
        self.font = ImageFont.truetype(fp, 26)
        self.small = ImageFont.truetype(fp, 20)
        self.keyframes = []

    def set_view(self, lookat, dist, az, el):
        if not self.on and not self.snaps:
            return
        self.cam.lookat[:] = lookat
        self.cam.distance, self.cam.azimuth, self.cam.elevation = dist, az, el

    def render(self, data, overlay=True):
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        img = Image.fromarray(self.r.render())
        if overlay:
            dr = ImageDraw.Draw(img, 'RGBA')
            dr.rectangle((0, self.H - 64, self.W, self.H), fill=(15, 20, 28, 215))
            dr.text((24, self.H - 50), self.caption, fill=(255, 255, 255), font=self.font)
            spd = f'{self.speed:.0f}x'
            dr.text((self.W - 230, self.H - 46), f't = {data.time:5.1f} s  ({spd})', fill=(190, 200, 210), font=self.small)
            dr.rectangle((0, 0, self.W, 34), fill=(15, 20, 28, 170))
            dr.text((16, 6), 'Custom 3D-printed fingers (PETG + TPU): wedge blade + rim hook  |  MuJoCo, physics only',
                    fill=(255, 210, 140), font=self.small)
            if self.checks:
                x0, y0 = self.W - 400, 46
                dr.rectangle((x0 - 10, y0 - 6, self.W - 10, y0 + 28 * len(self.checks) + 4), fill=(15, 20, 28, 160))
                for i, (t, ok) in enumerate(self.checks):
                    dr.text((x0, y0 + 28 * i), ('✓ ' if ok else '• ') + t,
                            fill=(120, 230, 120) if ok else (220, 220, 220), font=self.small)
        return img

    def push(self, img, n=1):
        b = np.asarray(img.convert('RGB')).tobytes()
        for _ in range(n):
            self.ff.stdin.write(b)
            self.n += 1

    def capture(self, data):
        if not self.on or data.time + 1e-9 < self.next:
            return
        self.next = data.time + self.every * self.speed
        self.push(self.render(data))

    def keyframe(self, data, path):
        if self.on:
            self.render(data).save(path)

    def close(self):
        if self.on:
            self.ff.stdin.close()
            self.ff.wait()
            self.r.close()


class Fail(Exception):
    pass


class Runner:
    def __init__(self, L, video=None, log=print, snaps=None):
        self.L = L
        self.log = log
        self.model, _ = S.build(L)
        self.ik = S.build_ik(L)
        m, d, ik = self.model, mujoco.MjData(self.model), self.ik
        self.data = d
        dw.set_state(m, d, door_deg=90, basket=dw.BASKET_TRAVEL)
        self.arm = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.fq = [m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]
        self.act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 9)]
        self.ik_arm = [ik.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        q0 = self.solve(np.array([0.20, 0.25, 1.20]), hand_frame([0, 0, -1.], [0, 1., 0]),
                        seeds=[np.array([0, -.3, 0, -2.2, 0, 2.0, .785])])
        d.qpos[self.arm] = q0
        d.qpos[self.fq] = .04
        d.ctrl[self.act[:7]] = q0
        d.ctrl[self.act[7]] = OPEN
        mujoco.mj_forward(m, d)
        qik = np.zeros(ik.nq)
        qik[self.ik_arm] = q0
        self.cfg = mink.Configuration(ik, q=qik)
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=.5, lm_damping=1.)
        self.post = mink.PostureTask(ik, cost=1e-4)
        self.post.set_target(qik.copy())
        self.limits = [mink.ConfigurationLimit(ik), mink.VelocityLimit(ik, {f'panda/joint{i}': 1.5 for i in range(1, 8)})]
        self.rec = Rec(m, video, snaps=snaps)
        self.cmd = self.tcp()
        self.robot = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/')}
        self.fingers = {m.body('panda/left_finger').id, m.body('panda/right_finger').id}
        self.allowed = set()
        self.bad = []           # unexpected robot contacts (name, force)
        self.max_bad = 0.
        self.metrics = {}
        self.phi = 30.
        self.ptilt = float(__import__('os').environ.get('PTILT', '8'))
        self.nsign = float(__import__('os').environ.get('NSIGN', '1'))
        self.aim = float(__import__('os').environ.get('AIM', '.010'))
        self.slip = []

    # ------------------------------------------------------------------ kinematics
    def tcp(self):
        s = self.data.site('tcp')
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def solve(self, p, R, seeds, iters=300):
        ik = self.ik
        best = None
        R = np.asarray(R)
        for q0 in seeds:
            q = np.zeros(ik.nq)
            q[self.ik_arm] = q0
            cfg = mink.Configuration(ik, q=q)
            task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
            post = mink.PostureTask(ik, cost=1e-3)
            post.set_target(q.copy())
            for _ in range(iters):
                v = mink.solve_ik(cfg, [task, post], 1., solver='daqp', damping=1e-4, limits=[mink.ConfigurationLimit(ik, gain=.5)])
                cfg.integrate_inplace(v, 1.)
            err = task.compute_error(cfg)
            e = float(np.linalg.norm(err[:3])) + .05 * float(np.linalg.norm(err[3:]))
            e += .001 * float(np.linalg.norm(cfg.q[self.ik_arm] - seeds[0]))
            if best is None or e < best[1]:
                best = (cfg.q[self.ik_arm].copy(), e)
        if best[1] > .003 and not getattr(self, '_in_rand', False):
            self._in_rand = True
            rng = np.random.default_rng(1)
            lo, hi = ik.jnt_range[:7, 0], ik.jnt_range[:7, 1]
            alt = self.solve(p, R, [lo + (hi - lo) * rng.random(7) for _ in range(30)], iters=200)
            self._in_rand = False
            return alt
        return best[0]

    # ------------------------------------------------------------------ stepping
    def monitor(self):
        m, d = self.model, self.data
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            for a, b in ((b1, b2), (b2, b1)):
                if a in self.robot and b not in self.robot and b not in self.allowed:
                    f = np.zeros(6)
                    mujoco.mj_contactForce(m, d, i, f)
                    if abs(f[0]) > 2.:
                        self.max_bad = max(self.max_bad, abs(f[0]))
                        if len(self.bad) < 40:
                            self.bad.append((round(d.time, 2), m.body(a).name, m.body(b).name, round(abs(f[0]), 1),
                                             m.geom(c.geom1).name, m.geom(c.geom2).name, np.round(c.pos, 3).tolist(),
                                             np.round(self.tcp()[0], 3).tolist(), self.rec.caption[:30]))

    def seg(self, dur, target, grip, stop=None, track=None):
        m, d = self.model, self.data
        g0 = float(d.ctrl[self.act[7]])
        n, nc = max(1, round(dur / m.opt.timestep)), round(CDT / m.opt.timestep)
        for k in range(n):
            u = (k + 1) / n
            s = u * u * u * (10 + u * (-15 + 6 * u))
            if k % nc == 0:
                p, R = target(s)
                self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
                qa = self.cfg.q.copy()
                try:
                    v = mink.solve_ik(self.cfg, [self.task, self.post], CDT, solver='daqp', damping=1e-5, limits=self.limits)
                except Exception:
                    v = np.zeros(self.ik.nv)
                self.cfg.integrate_inplace(v, CDT)
                qb = self.cfg.q.copy()
            qi = qa + ((k % nc) + 1) / nc * (qb - qa)
            d.ctrl[self.act[:7]] = qi[self.ik_arm]
            d.ctrl[self.act[7]] = g0 + s * (grip - g0)
            mujoco.mj_step(m, d)
            if k % 5 == 0:
                self.monitor()
                if track:
                    track()
            self.rec.capture(d)
            if stop is not None and k % 5 == 0 and stop():
                return True
        return False

    def move(self, dur, p1, R1, grip=None, stop=None, track=None):
        p0, R0 = self.cmd
        so0 = mink.SO3.from_matrix(R0)
        delta = (so0.inverse() @ mink.SO3.from_matrix(R1)).log()
        g = float(self.data.ctrl[self.act[7]]) if grip is None else grip
        f = lambda s: (p0 + s * (p1 - p0), (so0 @ mink.SO3.exp(s * delta)).as_matrix())
        hit = self.seg(dur, f, g, stop, track)
        self.cmd = (np.asarray(p1, float), np.asarray(R1, float))
        return hit

    def goto(self, p1, R1, grip=None, speed=.35, wspeed=1.2, track=None):
        p0, R0 = self.cmd
        dur = max(.6, 1.875 * np.linalg.norm(p1 - p0) / speed, 1.875 * math.radians(ang(R0.T @ R1)) / wspeed)
        self.post.set_target(self.cfg.q.copy())
        return self.move(dur, p1, R1, grip, track=track)

    def jgoto(self, p1, R1, grip=None, track=None, speed=.9, seeds=None):
        """Joint-space move for large re-orientations (IK solved first, then quintic joint interpolation)."""
        m, d = self.model, self.data
        seeds = (seeds or []) + [self.cfg.q[self.ik_arm]] + [self.cfg.q[self.ik_arm] + np.r_[0, 0, 0, 0, 0, 0, dz] for dz in (-1.5, 1.5)] + \
            [np.array([0, -.3, 0, -2.2, 0, 2.0, .785]), np.array([.5, .2, -.3, -2.0, .2, 2.2, 0.])]
        def fk_err(q):
            dd = mujoco.MjData(self.ik)
            dd.qpos[self.ik_arm] = q
            mujoco.mj_forward(self.ik, dd)
            return 1000 * np.linalg.norm(dd.site('tcp').xpos - p1) + ang(dd.site('tcp').xmat.reshape(3, 3).T @ np.asarray(R1))
        cands = [self.solve(p1, R1, seeds=[sd]) for sd in seeds]
        errs = [fk_err(q) for q in cands]
        if min(errs) > 3:
            rng = np.random.default_rng(7)
            lo, hi = self.ik.jnt_range[:7, 0], self.ik.jnt_range[:7, 1]
            for _ in range(25):
                q = self.solve(p1, R1, seeds=[lo + (hi - lo) * rng.random(7)], iters=200)
                cands.append(q)
                errs.append(fk_err(q))
                if errs[-1] < 2:
                    break
        ok = [k for k, e in enumerate(errs) if e < 3]
        k = min(ok, key=lambda k: np.linalg.norm(cands[k] - self.cfg.q[self.ik_arm])) if ok else int(np.argmin(errs))
        qg = cands[k]
        if not ok and getattr(self, 'sink_mode', False):
            raise Fail(f'no IK solution for the pre-grasp ({min(errs):.0f} mm/deg error); skipped before moving')
        q0 = self.cfg.q[self.ik_arm].copy()
        dur = max(1.0, 1.875 * float(np.max(np.abs(qg - q0))) / speed)
        g0 = float(d.ctrl[self.act[7]])
        g = g0 if grip is None else grip
        n = round(dur / m.opt.timestep)
        for k in range(n):
            u = (k + 1) / n
            s = u * u * u * (10 + u * (-15 + 6 * u))
            q = q0 + s * (qg - q0)
            d.ctrl[self.act[:7]] = q
            d.ctrl[self.act[7]] = g0 + s * (g - g0)
            mujoco.mj_step(m, d)
            if k % 5 == 0:
                self.monitor()
                if track:
                    track()
            self.rec.capture(d)
        qq = self.cfg.q.copy()
        qq[self.ik_arm] = qg
        self.cfg.update(qq)
        self.post.set_target(qq)
        self.cmd = (np.asarray(p1, float), np.asarray(R1, float))
        e = 1000 * np.linalg.norm(self.tcp()[0] - p1)
        self.log(f'    jgoto: tcp err {e:.1f} mm, rot err {ang(self.tcp()[1].T @ R1):.1f} deg, q={np.round(qg, 2)}')
        if track:
            self.log(f'    slip so far: {track.__closure__ and [c.cell_contents for c in track.__closure__ if isinstance(c.cell_contents, dict)]}')
        return e

    HUB = np.array([0.20, 0.22, 1.24])

    def transfer(self, p1, R1, tr):
        """Pile -> hub in front of the machine (orientation kept) -> pivot about the hooked rim at the hub ->
        above the basket target. Cartesian, joint-speed limited."""
        p0, R0 = self.cmd
        self.goto(np.array([p0[0], p0[1], self.HUB[2]]), R0, track=tr, speed=.3)
        self.goto(self.HUB, R0, track=tr, speed=.3)
        self.goto(self.HUB, R1, track=tr, wspeed=.7)
        if p1[0] < .07:       # back slot: slide in horizontally from the front, below the control band
            self.goto(np.array([p1[0] + .13, p1[1], p1[2]]), R1, track=tr, speed=.25)
        else:
            self.goto(np.array([p1[0], p1[1], self.HUB[2] - .04]), R1, track=tr, speed=.25)
        self.goto(p1, R1, track=tr, speed=.15)
        self.hold(.3, track=tr)
        p, R = self.tcp()
        e = 1000 * np.linalg.norm(p - p1)
        self.log(f'    transfer: tcp err {e:.1f} mm, rot err {ang(R.T @ R1):.1f} deg; slip so far {[c.cell_contents for c in tr.__closure__ if isinstance(c.cell_contents, dict)]}')
        return e

    def to_hub(self):
        p, R = self.cmd
        if p[0] < .15:
            self.goto(np.array([p[0], p[1], max(p[2], self.HUB[2] - .04)]), R, speed=.3)
            self.goto(np.array([self.HUB[0], self.HUB[1], self.HUB[2]]), R, speed=.3)

    def hold(self, dur, grip=None, stop=None, track=None):
        return self.move(dur, *self.cmd, grip=grip, stop=stop, track=track)

    # ------------------------------------------------------------------ helpers
    def body_pose(self, name):
        b = self.data.body(name)
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def finger_contact(self, name):
        m, d = self.model, self.data
        bid = m.body(name).id
        n = 0
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 == bid and b2 in self.fingers) or (b2 == bid and b1 in self.fingers):
                n += 1
        return n

    def robot_touch(self, name):
        m, d = self.model, self.data
        bid = m.body(name).id
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 == bid and b2 in self.robot) or (b2 == bid and b1 in self.robot):
                return True
        return False

    def support_force(self, name):
        """Upward force on `name` from non-robot bodies (N)."""
        m, d = self.model, self.data
        bid = m.body(name).id
        fz = 0.
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if bid not in (b1, b2) or b1 in self.robot or b2 in self.robot:
                continue
            f = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, f)
            fw = c.frame.reshape(3, 3).T @ f[:3]
            fz += fw[2] * (1 if b2 == bid else -1)
        return fz

    def in_hand(self, name):
        p, R = self.tcp()
        q, Q = self.body_pose(name)
        return R.T @ (q - p), R.T @ Q

    def tracker(self, name, ref):
        """Grip slip: change of the object pose in the TCP frame vs `ref` (taken right after closing)."""
        st = dict(dp=0., da=0.)

        def f():
            p, R = self.in_hand(name)
            st['dp'] = max(st['dp'], 1000 * float(np.linalg.norm(p - ref[0])))
            st['da'] = max(st['da'], ang(ref[1].T @ R))
        return f, st

    def caption(self, t):
        if __import__('os').environ.get('DBGP'):
            n = self.body_pose('plate2')[1][:, 2]
            self.log(f'      dbg plate2 normal {np.round(n, 3)} pos {np.round(self.body_pose("plate2")[0], 3)}')
        if self.rec.snaps:
            self.rec.caption = self.rec.caption + ' [end]'
            bk = self.data.body('dw/basket').xpos
            self.rec.set_view(np.array([-.02, .25, 1.0]), 1.45, 106, -30)
            self.rec.render(self.data).save(f'{self.rec.snaps}_{self.data.time:06.2f}.png')
        self.rec.caption = t
        if self.rec.on:
            close = any(k in t for k in ('lower', 'release', 'Settle', 'Final'))
            self.rec.set_view(*(BASKET_VIEW if close else MAIN_VIEW))
        self.log(f'[{self.data.time:6.2f}] {t}')

    # ------------------------------------------------------------------ plates
    def plate_grasp_frame(self, name):
        """Rim grasp at the +x rim point, closing axis = rim normal (hook on the concave side), fingers along the
        rim slope toward the plate centre."""
        p, R = self.body_pose(name)
        n = R[:, 2]
        f = math.radians(self.phi)
        rd = np.array([math.cos(f), math.sin(f), 0.])
        radial = unit(rd - np.dot(rd, n) * n)
        along = math.cos(S.SLOPE) * radial + math.sin(S.SLOPE) * n         # outward-up along the rim
        rn = unit(np.cross(along, np.cross(n, radial)))                      # rim normal
        if rn @ n < 0:
            rn = -rn
        r_g = S.R_OUT - FG.TCP_IN                                             # TCP 5 mm inside the rim edge
        mid = p + R @ np.array([0, 0, .0075]) + (r_g - S.R_DISC) / math.cos(S.SLOPE) * along \
            + (S.R_DISC) * radial + .0 * n
        # mid-surface of the rim at radius r_g: start of slope at R_DISC, height .0075 (disc mid) + rise
        mid = p + S.R_DISC * radial + n * (.0075) + (r_g - S.R_DISC) / math.cos(S.SLOPE) * along
        Rh = hand_frame(-along, rn if FG.HOOK_ON_LEFT else -rn)
        return mid, Rh, rn, along

    def pick_plate(self, i, total):
        name = f'plate{i}'
        nm = f'Plate {total - i}/{total}'
        self.allowed = {self.model.body(name).id}
        mid, Rh, rn, along = self.plate_grasp_frame(name)
        q0 = .009                                                # pre-grasp opening per finger
        wedge_face = -(.0025 + .001)                             # 1 mm below the rim underside
        centre = mid + rn * (wedge_face + q0)
        self.caption(f'{nm} · approach stack edge, wedge level with the rim gap')
        self.to_hub()
        pre = centre + along * .05
        self.jgoto(pre + np.array([0, 0, .06]), Rh, grip_ctrl(q0), seeds=[np.array([1.02, -1.55, -2.07, -2.5, .12, 3.05, -2.9])])
        self.goto(pre, Rh, grip_ctrl(q0), speed=.15)
        self.caption(f'{nm} · wedge slides between top plate and the plate below')
        e = 1000 * np.linalg.norm(self.tcp()[0] - pre)
        self.move(1.2, centre, Rh)
        self.hold(.3)
        self.metrics.setdefault('approach_err_mm', []).append(round(1000 * float(np.linalg.norm(self.tcp()[0] - centre)), 2))
        self.caption(f'{nm} · close: wedge lifts the rim, hook pad clamps it (70 N)')
        self.hold(.8, grip=0.)
        ref = self.in_hand(name)
        tr, st = self.tracker(name, ref)
        self.caption(f'{nm} · lift off the stack')
        p, R = self.cmd
        self.move(1.2, p + np.array([0, 0, .015]) + .01 * along, R, track=tr)
        self.goto(p + np.array([.10, 0, .16]), R, track=tr)
        if self.finger_contact(name) == 0 or self.data.body(name).xpos[2] < S.COUNTER_Z + .06:
            raise Fail(f'{name}: not held after lift')
        return tr, st

    def insert_plate(self, i, slot_k, total, tr, st):
        name = f'plate{i}'
        nm = getattr(self, 'cur_label', None) or f'Plate {total - i}/{total}'
        m, d = self.model, self.data
        self.caption(f'{nm} · pivot about the hooked rim: plate upright, hand stays above the basket')
        # target plate orientation: normal along -x (as rotating the grasp pose -90 deg about world y)
        pr, Rr = self.in_hand(name)                    # object in TCP frame
        bk = d.body('dw/basket').xpos.copy()
        tx = bk[0] + dw.PLATE_C_X
        ty = bk[1] + (dw.PLATE_TINE_Y[slot_k] + dw.PLATE_TINE_Y[slot_k + 1]) / 2
        floor = bk[2] + dw.WIRE_R
        Rp = np.column_stack([[0, 1., 0], [0, 0, 1.], [1., 0, 0]])       # local x->y, y->z, normal-> +x?
        # choose normal sign closest to the current plate normal after the pivot
        _, Rcur = self.body_pose(name)
        f = math.radians(self.phi)
        t = np.array([-math.sin(f), math.cos(f), 0.])
        Rpiv = rot_axis(t, math.radians(90)) @ Rcur
        if (Rpiv @ Rr.T)[:, 2] @ [0, 0, 1.] > -.5:
            Rpiv = rot_axis(t, -math.radians(90)) @ Rcur
        Rpiv = rot_axis([0, 0, 1.], -f) @ Rpiv
        n_t = np.array([0, self.nsign * (np.sign(Rpiv[1, 2]) or 1.), 0])
        if getattr(self, 'sink_mode', False):        # plate already hangs from a top-rim grasp: no pivot
            Rpiv = Rcur
            n_t = np.array([0, -1., 0])
        Rpiv0 = Rpiv
        # plate in-plane: keep its current yaw-ish, just align normal to n_t
        z = n_t
        y = np.array([0, 0, 1.])
        x = np.cross(y, z)
        Rt = np.column_stack([x, y, z])
        # rotate about the normal so that the gripped rim point is at the top (minimal change from Rpiv)
        best = None
        for a in np.linspace(-math.pi, math.pi, 73):
            Rc = Rt @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
            Rh = Rc @ Rr.T
            c = ang(Rpiv.T @ Rc)
            if getattr(self, 'sink_mode', False):   # grip point on top, fingers pointing down (proven slot pose)
                c = math.degrees(math.acos(np.clip(-Rh[2, 2], -1, 1))) + .05 * c
            if best is None or c < best[0]:
                best = (c, Rc, Rh)
        _, Rc, Rh = best
        c_target = np.array([tx, ty, floor + S.R_OUT + .004]) - .010 * Rc[:, 2] * 0   # plate centre target
        # aim: 10 mm toward the convex side (gpt_engineer p2_lean): convex side = -normal
        aim = self.aim + (float(__import__('os').environ.get('AIM_FIRST', '0')) if slot_k == min(self.plate_slots) else 0.)
        c_target = c_target - aim * n_t
        org_t = c_target - Rc @ np.array([0, 0, S.MID_Z])
        tcp_t = org_t - Rh @ pr
        above = tcp_t + np.array([0, 0, .10])
        # pivot in joint space (the edge grasp needs a folded-elbow configuration), then Cartesian to the slot
        p0, R0 = self.cmd
        hub = np.array([float(v) for v in __import__('os').environ.get('PHUB', '0.16,0.30,1.30').split(',')])
        f = math.radians(self.phi)
        t = np.array([-math.sin(f), math.cos(f), 0.])
        Rpiv = rot_axis(t, math.radians(90)) @ R0
        if Rpiv[:, 2] @ [0, 0, 1.] > -.5:
            Rpiv = rot_axis(t, -math.radians(90)) @ R0
        if getattr(self, 'sink_mode', False):
            self.goto(hub, R0, track=tr, speed=.2)
        else:
            self.goto(hub, Rpiv, track=tr, speed=.2, wspeed=.6)               # pivot about the hooked rim
        self.goto(hub, Rh, track=tr, wspeed=.6)                               # turn the hanging plate to the slot
        e = 1000 * np.linalg.norm(self.tcp()[0] - hub)
        self.log(f'    pivot: tcp err {e:.1f} mm, rot err {ang(self.tcp()[1].T @ Rh):.1f} deg; slip so far {st}')
        # re-measure the plate in the hand after the pivot (GT pose). The plate has swung in its own plane (pendulum,
        # harmless); keep that in-plane angle and only rotate its normal onto the slot normal.
        pr, Rr = self.in_hand(name)
        _, Rcur = self.body_pose(name)
        n_c = Rcur[:, 2]
        n_t2 = n_t.copy()
        axc = np.cross(n_c, n_t2)
        Rc = (rot_axis(axc, math.asin(min(1., np.linalg.norm(axc)))) if np.linalg.norm(axc) > 1e-9 else np.eye(3)) @ Rcur
        org_t = c_target - Rc @ np.array([0, 0, S.MID_Z])
        Rh = Rc @ Rr.T
        tcp_t = org_t - Rh @ pr
        above = tcp_t + np.array([0, 0, HI])
        self.goto(hub, Rh, track=tr, wspeed=.6)
        # slide in along -x in the plate's own plane (normal = y): never crosses the seated plates
        zc = max(above[2], float(__import__('os').environ.get('ZCLR', '1.45')))   # hanging plate clears the seated plates
        self.goto(np.array([above[0] + .20, above[1], zc]), Rh, track=tr, speed=.2)
        self.goto(np.array([above[0], above[1], zc]), Rh, track=tr, speed=.12)
        self.goto(np.array([above[0], above[1], above[2]]), Rh, track=tr, speed=.08)
        self.goto(above, Rh, track=tr, speed=.15)
        self.hold(.4, track=tr)
        for it in range(6 if getattr(self, 'sink_mode', False) else 3):   # closed-loop alignment on the measured (GT) plate pose
            c = self.data.site(name + '_c').xpos.copy()
            _, Rnow = self.body_pose(name)
            n_now = Rnow[:, 2]
            n_goal = n_t2
            axc = np.cross(n_now, n_goal)
            Rfix = rot_axis(axc, math.asin(min(1., np.linalg.norm(axc)))) if np.linalg.norm(axc) > 1e-9 else np.eye(3)
            p_now, R_now = self.cmd
            # rotate the hand about the plate centre, then shift horizontally onto the target centre
            Rh = Rfix @ R_now
            p_new = c + Rfix @ (p_now - c)
            err = (c_target + np.array([0, 0, HI])) - c
            p_new = p_new + np.array([err[0], err[1], 0.])
            self.move(.8, p_new, Rh, track=tr)
            self.hold(.8 if getattr(self, 'sink_mode', False) else .5, track=tr)
            self.log(f'    align {it}: centre err {1000 * np.linalg.norm(err[:2]):.1f} mm, normal err {math.degrees(math.asin(min(1., np.linalg.norm(axc)))):.1f} deg')
        p_now, _ = self.cmd
        tcp_t = p_now - np.array([0, 0, HI])
        e = 1000 * np.linalg.norm(self.tcp()[0] - above)
        self.log(f'    plate above slot: tcp err {e:.1f} mm; slip so far {st}')
        self.metrics.setdefault('pivot_track_err_mm', []).append(round(float(e), 2))
        self.caption(f'{nm} · lower between the tines (slot {slot_k + 1})')
        self.allowed = {m.body(name).id}
        _, Rnow = self.body_pose(name)
        self.log(f'    plate normal before lowering {np.round(Rnow[:, 2], 3)}, target {np.round(Rc[:, 2], 3)}')
        zt = tcp_t[2]
        hit = self.move(3.6, tcp_t + np.array([0, 0, -.008]), Rh, track=tr,
                        stop=lambda: self.support_force(name) > float(__import__('os').environ.get('SUPN', '1.0')) and self.tcp()[0][2] < zt + .006)
        if hit:
            self.cmd = (self.tcp()[0], Rh)
            w = self.model.body(name).subtreemass[0] * 9.81
            self.move(2.0, self.tcp()[0] - np.array([0, 0, .010]), Rh, track=tr, stop=lambda: self.support_force(name) > .92 * w)
            self.cmd = (self.tcp()[0], Rh)
            self.hold(.3, track=tr)
            self.log(f'    unloaded: support {self.support_force(name):.2f} N of {w:.2f} N')
        low = min((self.body_pose(name)[0] + self.body_pose(name)[1] @ np.array([S.R_OUT * math.cos(u), S.R_OUT * math.sin(u), .0173]))[2]
                  for u in np.linspace(0, 2 * math.pi, 73))
        self.log(f'    touchdown: tcp z - target {1000 * (self.tcp()[0][2] - zt):.1f} mm, lowest rim above floor wire top '
                 f'{1000 * (low - d.body("dw/basket").xpos[2] - 2 * dw.WIRE_R):.1f} mm')
        _, Rnow = self.body_pose(name)
        self.log(f'    plate normal at support {np.round(Rnow[:, 2], 3)}, hit={hit}')
        self.hold(.2, track=tr)
        self.metrics.setdefault('plate_slip_mm', []).append(round(st['dp'], 2))
        self.metrics.setdefault('plate_rot_deg', []).append(round(st['da'], 2))
        self.caption(f'{nm} · release and retract')
        self.hold(1.5, grip=grip_ctrl(float(__import__('os').environ.get('POPEN', '.009'))))
        self.hold(.3)
        self.log(f'    after open: {json.dumps(self.eval_plate(i, slot_k))}')
        p, R = self.cmd
        self.allowed = {m.body(name).id}
        if getattr(self, 'sink_mode', False):
            self.move(1.2, p - .05 * R[:, 2], R)                                # slide the fingers off along the rim
            self.move(1.2, p - .05 * R[:, 2] + np.array([0, 0, .12]), R)
        else:
            self.move(1.6, p + np.array([0, 0, .15]), R)
        self.log(f'    after retract: {json.dumps(self.eval_plate(i, slot_k))}')
        if self.rec.snaps:
            bkp = self.data.body('dw/basket').xpos
            self.rec.set_view(bkp + np.array([.1, .07, .08]), .45, 90, -5)
            self.rec.render(self.data).save(f'{self.rec.snaps}_side_{name}.png')
            self.rec.set_view(bkp + np.array([.1, .07, .08]), .45, 0, -5)
            self.rec.render(self.data).save(f'{self.rec.snaps}_front_{name}.png')
        self.allowed = set()

    # ------------------------------------------------------------------ bowls
    def bowl_grasp_frame(self, name, psi=0.):
        p, R = self.body_pose(name)
        a = R[:, 2]
        rd = np.array([math.sin(psi), -math.cos(psi), 0.])
        radial = unit(rd - np.dot(rd, a) * a)   # grip the rim point facing -y (rotated by psi)
        alpha = math.atan2(S.RIM_R * math.cos(math.pi / 20) - .030, .056)
        down = unit(-math.sin(alpha) * radial - math.cos(alpha) * a)       # along the wall toward the foot
        n_in = unit(-math.cos(alpha) * radial + math.sin(alpha) * a)        # wall normal, into the bowl
        r_mid = S.RIM_R * math.cos(math.pi / 20) - .002 / math.cos(alpha)
        top = p + a * S.BOWL_H + radial * r_mid                             # rim top edge, mid-thickness
        g = top + down * FG.TCP_IN
        return g, hand_frame(down, n_in if FG.HOOK_ON_LEFT else -n_in), n_in, down

    def pick_bowl(self, i, total):
        name = f'bowl{i}'
        nm = f'Bowl {total - i}/{total}'
        self.allowed = {self.model.body(name).id}
        g, Rh, n_in, down = self.bowl_grasp_frame(name)
        q0 = .012
        centre = g + n_in * (-.004 + q0)                                    # wedge face 2 mm outside the wall
        self.caption(f'{nm} · approach nested stack from above')
        self.to_hub()
        pre = centre - down * .06
        self.goto(pre + np.array([0, 0, .05]), Rh, grip_ctrl(q0))
        self.goto(pre, Rh, grip_ctrl(q0), speed=.15)
        self.caption(f'{nm} · wedge slides down outside the top bowl, hook inside')
        self.move(1.2, centre, Rh)
        self.hold(.3)
        self.metrics.setdefault('approach_err_mm', []).append(round(1000 * float(np.linalg.norm(self.tcp()[0] - centre)), 2))
        self.caption(f'{nm} · close: rim seats on the hook shoulder')
        self.hold(.8, grip=0.)
        ref = self.in_hand(name)
        tr, st = self.tracker(name, ref)
        self.caption(f'{nm} · separate from the stack and lift')
        p, R = self.cmd
        self.move(1.5, p + np.array([0, 0, .14]), R, track=tr)
        if self.finger_contact(name) == 0 or self.data.body(name).xpos[2] < S.COUNTER_Z + .08:
            raise Fail(f'{name}: not held after lift')
        return tr, st

    def place_bowl(self, i, slot, total, tr, st, tilt_deg):
        slot_x, row = slot[:2]
        yaw = math.radians(slot[2]) if len(slot) > 2 else 0.
        if len(slot) > 3:
            tilt_deg = slot[3]
        name = f'bowl{i}'
        nm = f'Bowl {total - i}/{total}'
        m, d = self.model, self.data
        self.caption(f'{nm} · pivot about the hooked rim to {tilt_deg:.0f} deg (wrist roll only)')
        pr, Rr = self.in_hand(name)
        bk = d.body('dw/basket').xpos.copy()
        floor = bk[2] + dw.WIRE_R
        # target bowl orientation: axis tilted by `tilt` about world x, opening toward +y (back against the side wall)
        t = math.radians(tilt_deg)
        a_t = np.array([math.sin(t) * math.sin(yaw), math.sin(t) * math.cos(yaw), math.cos(t)])
        _, Rb = self.body_pose(name)
        a0 = Rb[:, 2]
        ax = np.cross(a0, a_t)
        Rc = rot_axis(ax, math.asin(min(1., np.linalg.norm(ax)))) @ Rb if np.linalg.norm(ax) > 1e-9 else Rb
        if a0 @ a_t < 0:
            raise Fail('bowl flipped')
        Rh = Rc @ Rr.T
        us = np.linspace(0, 2 * math.pi, 73)
        pts = [Rc @ np.array([S.RIM_R * math.cos(u), S.RIM_R * math.sin(u), S.BOWL_H]) for u in us] + \
              [Rc @ np.array([.025 * math.cos(u), .025 * math.sin(u), -.008]) for u in us] + \
              [Rc @ np.array([.032 * math.cos(u), .032 * math.sin(u), 0.]) for u in us]
        zmin = min(q[2] for q in pts)
        nb = np.array([0, math.cos(dw.BOWL_LEAN), math.sin(dw.BOWL_LEAN)])      # back-rest plane normal (+y, up)
        base = np.array([0, bk[1] + dw.BOWL_ROWS_Y[row], bk[2]])
        c = base @ nb + dw.WIRE_R + .004
        oz = floor + .002 - zmin + .004
        oy = (c - min(q @ nb for q in pts) - oz * nb[2]) / nb[1]
        org_t = np.array([bk[0] + slot_x, oy, oz])
        tcp_t = org_t - Rh @ pr
        e = self.transfer(tcp_t + np.array([0, 0, .02 if bk[0] + slot_x < .07 else .06]), Rh, tr)
        self.metrics.setdefault('pivot_track_err_mm', []).append(round(float(e), 2))
        dz = .02 if bk[0] + slot_x < .07 else .06
        for it in range(3):          # closed-loop alignment on the measured (GT) bowl pose, as from perception
            pb, Rb2 = self.body_pose(name)
            a_now = Rb2[:, 2]
            axc = np.cross(a_now, a_t)
            Rfix = rot_axis(axc, math.asin(min(1., np.linalg.norm(axc)))) if np.linalg.norm(axc) > 1e-9 else np.eye(3)
            p_now, R_now = self.cmd
            Rh = Rfix @ R_now
            p_new = pb + Rfix @ (p_now - pb) + (org_t + np.array([0, 0, dz]) - pb)
            self.move(.8, p_new, Rh, track=tr)
            self.hold(.4, track=tr)
            self.log(f'    bowl align {it}: pos err {1000 * np.linalg.norm(org_t + [0, 0, dz] - pb):.1f} mm, axis err {math.degrees(math.asin(min(1., np.linalg.norm(axc)))):.1f} deg')
        tcp_t = self.cmd[0] - np.array([0, 0, dz])
        self.caption(f'{nm} · lower into the bowl section, rim on the floor wires')
        hit = self.move(1.8, tcp_t + np.array([0, 0, -.004]), Rh, track=tr, stop=lambda: self.support_force(name) > 1.0)
        self.hold(.2, track=tr)
        self.metrics.setdefault('bowl_slip_mm', []).append(round(st['dp'], 2))
        self.metrics.setdefault('bowl_rot_deg', []).append(round(st['da'], 2))
        self.caption(f'{nm} · release, bowl leans on the tines')
        self.hold(.6, grip=grip_ctrl(.014))
        p, R = self.cmd
        self.move(1.0, p - R[:, 2] * .06, R)
        self.move(1.0, p - R[:, 2] * .06 + np.array([0, 0, .10]), R)
        self.allowed = set()

    # ------------------------------------------------------------------ evaluation
    def eval_plate(self, i, slot_k):
        d = self.data
        name = f'plate{i}'
        p, R = self.body_pose(name)
        c = d.site(name + '_c').xpos
        bk = d.body('dw/basket').xpos
        y0, y1 = bk[1] + dw.PLATE_TINE_Y[slot_k], bk[1] + dw.PLATE_TINE_Y[slot_k + 1]
        n = R[:, 2]
        tilt = math.degrees(math.asin(min(1., abs(n[2]))))            # plate plane vs vertical
        lean = math.degrees(math.acos(min(1., abs(n[1]))))              # normal vs slot normal (y)
        pts = [p + R @ np.array([S.R_OUT * math.cos(u), S.R_OUT * math.sin(u), .0173]) for u in np.linspace(0, 2 * math.pi, 145)]
        low = min(pts, key=lambda q: q[2])
        between = y0 < low[1] < y1
        v = np.linalg.norm(d.qvel[self.model.joint(name + '_free').dofadr[0]:][:3])
        err = 1000 * float(np.linalg.norm([c[0] - (bk[0] + dw.PLATE_C_X), c[1] - (y0 + y1) / 2]))
        ok = between and lean < 15 and v < .005 and not self.robot_touch(name) and low[2] < bk[2] + .02
        return dict(item=name, slot=f'plate slot {slot_k + 1}', centre_err_mm=round(err, 1), lean_deg=round(lean, 1),
                    plane_vs_vertical_deg=round(tilt, 1), lowest_rim_between_tines=bool(between), speed_mm_s=round(1000 * v, 2),
                    ok=bool(ok))

    def eval_bowl(self, i, slot, tilt_deg):
        slot_x, row = slot[:2]
        yaw = math.radians(slot[2]) if len(slot) > 2 else 0.
        d = self.data
        name = f'bowl{i}'
        p, R = self.body_pose(name)
        a = R[:, 2]
        tilt = math.degrees(math.acos(np.clip(a[2], -1, 1)))
        bk = d.body('dw/basket').xpos
        loc = p - bk
        in_area = (0.02 < loc[0] < dw.BASKET_HX) and (-dw.BASKET_HY < loc[1] < .04)
        v = np.linalg.norm(d.qvel[self.model.joint(name + '_free').dofadr[0]:][:3])
        ok = in_area and 25 < tilt < 115 and v < .005 and not self.robot_touch(name)
        return dict(item=name, slot=f'bowl rest row {row + 1}', pos_err_x_mm=round(1000 * abs(loc[0] - slot_x), 1),
                    tilt_from_upright_deg=round(tilt, 1), opening_dir=[round(float(x), 2) for x in a],
                    in_bowl_area=bool(in_area), speed_mm_s=round(1000 * v, 2), ok=bool(ok))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--L', type=float, default=.080)
    ap.add_argument('--video', default=None)
    ap.add_argument('--tag', default='run')
    ap.add_argument('--only', default='all')
    ap.add_argument('--tilt', type=float, default=60.)
    ap.add_argument('--keyframes', action='store_true')
    ap.add_argument('--snaps', default=None)
    a = ap.parse_args()
    t0 = time.monotonic()
    logs = []
    log = lambda s: (print(s, flush=True), logs.append(s))
    r = Runner(a.L, a.video, log, snaps=a.snaps)
    res = dict(L_mm=round(a.L * 1000, 1), items=[], fail=None)
    kf = ROOT / 'frames'
    kn = [0]

    def key(label):
        if a.keyframes and label in ('start', 'bowl1_grip', 'bowl1_placed', 'bowl2_placed', 'plate1_grip', 'plate1_placed', 'plate3_placed', 'final'):
            kn[0] += 1
            r.rec.keyframe(r.data, kf / f'{kn[0]:02d}_{label}.png')

    def recheck(checks):
        out = []
        for (t, ok), it in zip(checks, res['items']):
            if it['item'].startswith('bowl'):
                i = int(it['item'][-1])
                ok = r.eval_bowl(i, bowl_slots[S.N_BOWLS - 1 - i], a.tilt)['ok']
            else:
                i = int(it['item'][-1])
                ok = r.eval_plate(i, int(it['slot'].split()[-1]) - 1)['ok']
            out.append((t, ok))
        return out

    if r.rec.on:
        intro(r)
    # settle
    r.caption('Start: 3 stacked plates + 2 nested bowls on the counter; door open, basket out')
    r.hold(1.0)
    key('start')
    plate_slots = [int(v) for v in __import__('os').environ.get('PSLOTS', '1,3,2').split(',')]
    r.plate_slots = plate_slots
    bowl_slots = json.loads(__import__('os').environ.get('BSLOTS', '[[0.098, 0, 0], [0.098, 1, 0]]'))
    checks = []
    try:
        if a.only in ('all', 'bowls'):
            for j, i in enumerate(range(S.N_BOWLS - 1, -1, -1)):
                tr, st = r.pick_bowl(i, S.N_BOWLS)
                key(f'bowl{j + 1}_grip')
                r.place_bowl(i, bowl_slots[j], S.N_BOWLS, tr, st, a.tilt)
                r.hold(.8)
                ev = r.eval_bowl(i, bowl_slots[j], a.tilt)
                res['items'].append(ev)
                log(json.dumps(ev))
                checks.append((f'Bowl {j + 1}/{S.N_BOWLS} tilted in bowl area', ev['ok']))
                checks = recheck(checks)
                r.rec.checks = checks
                key(f'bowl{j + 1}_placed')
        if a.only in ('all', 'plates'):
            for j, i in enumerate(range(S.N_PLATES - 1, -1, -1)):
                tr, st = r.pick_plate(i, S.N_PLATES)
                key(f'plate{j + 1}_grip')
                r.insert_plate(i, plate_slots[j], S.N_PLATES, tr, st)
                r.hold(.8)
                ev = r.eval_plate(i, plate_slots[j])
                res['items'].append(ev)
                log(json.dumps(ev))
                checks.append((f'Plate {j + 1}/{S.N_PLATES} upright in tines', ev['ok']))
                checks = recheck(checks)
                r.rec.checks = checks
                key(f'plate{j + 1}_placed')
    except Fail as e:
        res['fail'] = str(e)
        log('FAIL ' + str(e))
    r.caption('Settle 2 s, then measure final poses')
    r.hold(2.0)
    # re-evaluate everything after the final settle
    final = []
    for it in res['items']:
        if it['item'].startswith('plate'):
            i = int(it['item'][-1])
            k = int(it['slot'].split()[-1]) - 1
            final.append(r.eval_plate(i, k))
        else:
            i = int(it['item'][-1])
            j = S.N_BOWLS - 1 - i
            final.append(r.eval_bowl(i, bowl_slots[j], a.tilt))
    res['final'] = final
    res['metrics'] = r.metrics
    res['unexpected_robot_contacts'] = r.bad[:20]
    res['max_unexpected_contact_N'] = round(r.max_bad, 1)
    res['sim_time_s'] = round(r.data.time, 1)
    res['wall_s'] = round(time.monotonic() - t0, 1)
    if r.rec.on:
        dn = lambda f: (f"Bowl {S.N_BOWLS - int(f['item'][-1])}" if f['item'].startswith('bowl') else f"Plate {S.N_PLATES - int(f['item'][-1])}")
        lab = lambda f: (f"{dn(f)} \u2192 {f['slot']} \u00b7 tilt {f['tilt_from_upright_deg']:.0f}\u00b0" if f['item'].startswith('bowl')
                         else f"{dn(f)} \u2192 {f['slot']} \u00b7 lean {f['lean_deg']:.1f}\u00b0")
        r.rec.checks = [(lab(f), f['ok']) for f in final]
        r.caption('Final measured poses: all items checked')
        img = r.rec.render(r.data)
        key('final')
        r.rec.push(img, 90)
    r.rec.close()
    if __import__('os').environ.get('VIEWTEST'):
        rr = Rec(r.model, None, snaps='x')
        ims = []
        for v in [([0.06, 0.06, 1.08], 1.0, 250, -55), ([0.06, 0.06, 1.08], 0.9, 290, -50), ([0.06, 0.04, 1.08], 0.9, 160, -60), ([0.06, 0.04, 1.08], 1.1, 200, -65)]:
            rr.set_view(*v)
            rr.caption = str(v)
            ims.append(rr.render(r.data).resize((640, 360)))
        W = Image.new('RGB', (1280, 720))
        for k, im in enumerate(ims):
            W.paste(im, ((k % 2) * 640, (k // 2) * 360))
        W.save(ROOT / 'logs' / 'viewtest.png')
    (ROOT / 'out' / f'{a.tag}.json').write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != 'items'}, indent=1))


def intro(r):
    """Close-up of the finger design at the start (rendered from the live model)."""
    rec = r.rec
    hp = r.data.body('panda/hand').xpos.copy()
    tp = r.tcp()[0]
    for k in range(75):
        rec.set_view((hp + tp) / 2, .30, 150 + k * 1.2, -12)
        rec.caption = 'Design: rim-hook finger (22 mm TPU pad + 3 mm shoulder) + wedge blade (2 mm tip), 80 mm'
        img = rec.render(r.data)
        rec.push(img)
        if k == 40:
            img.save(ROOT / 'frames' / '00_finger_closeup.png')
    rec.set_view(*MAIN_VIEW)


if __name__ == '__main__':
    main()
