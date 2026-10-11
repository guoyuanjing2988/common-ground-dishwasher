"""End-to-end dishwasher loading: one continuous mj_step simulation.

perceive plate -> open door -> pod into groove -> pull basket -> pick plate (flat: drag, re-perceive,
pinch, tilt, lift | rack: grasp upright) -> carry + insert upright -> push basket -> close door ->
press quick, power.

Rules: physics only via mj_step; object qpos only at reset (in scene.build); no weld / connect /
attach / mocap / adhesion / applied forces on objects; gravcomp=1 on the Panda bodies only.
The controller is a mink differential-IK reference generator (100 Hz, joint speed <= 1.2 rad/s,
rail <= 0.3 m/s) feeding the Panda's position servos, as in the PoCs. Robot-side sensing used for
control: joint encoders (TCP pose), a wrist force/torque sensor (MuJoCo force sensor at the hand
flange) and the camera. The dishwasher geometry / pose is a known, calibrated model. Ground-truth
object poses and dishwasher joint sensors are used ONLY in the success predicates and logs.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

import imageio.v2 as imageio
import mink
try:
    import qpsolvers as _qps
    SOLVER = 'daqp' if 'daqp' in _qps.available_solvers else _qps.available_solvers[0]
except Exception:
    SOLVER = 'daqp'
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import dishwasher as dw
import perception as PC
import plan
import scene as S
from util import angle_deg, hand_frame, renderer, rot_axis, rot_z, scene_option, unit

ROOT = Path(__file__).resolve().parent
CONTROL_DT = .01
OPEN = 255.
TOUCH = 100.                     # closed fingers as a tool, ~2 N squeeze (FLAT PoC)
CLEAN_N = 20.                    # 'clean' step: no unplanned robot-environment contact above this force
BOPEN = 157.                     # basket bar: fingers open to ~13 mm each (top frame wire 40 mm below the bar)
K_GRIP = 800.
FOOTER = ("Simulation (MuJoCo). Water, washing and touch-screen capacitance not simulated. Dishwasher internals "
          "estimated from product photos. Plate pose from camera + Codex vision model.")
STEPS = ['perceive_plate', 'open_door', 'load_pod', 'pull_basket', 'pick_plate', 'insert_plate', 'push_basket',
         'close_door', 'press_buttons']
STEP_LABEL = dict(perceive_plate='Perceive plate (camera + Codex)', open_door='Open door (follow hinge arc)',
                  load_pod='Pod into door groove', pull_basket='Pull lower basket out',
                  pick_plate='Pick plate', insert_plate='Insert plate upright',
                  push_basket='Push basket in', close_door='Close door (|angle| < 1 deg)',
                  press_buttons='Press QUICK, then POWER')


def grip_ctrl(q_target):
    """Force-limited gripper: ctrl 0 -> finger target -0.03 m, 255 -> +0.04 m."""
    return float(np.clip((q_target + .03) / .07 * 255, 0, 255))


def squeeze_ctrl(width, force_per_finger, floor=0.0):
    """Force-limited squeeze on an object of `width`; target clamped at `floor` (finger position) so that if
    the object slips out the fingers only close, instead of being driven into each other."""
    return grip_ctrl(max(floor, width / 2 - 2 * force_per_finger / K_GRIP))


class StepFail(Exception):
    pass


@dataclass
class Option:
    name: str
    mount: S.Mount
    layout: S.Layout = field(default_factory=S.Layout)
    q_home: tuple = None                      # None: solve IK for the park pose at start
    q_seed: tuple = (0, -.3, 0, -2.2, 0, 2.0, .785)
    hub: tuple = (0.18, 0.36, 1.22)           # transfer hub in front of the machine, right of it
    counter_clear_z: float = 1.08             # lift to this TCP height before travelling over the counter
    door: tuple = (.5, 0., 1)                 # kappa, psi_deg, flip  (mount_study choice)
    basket: tuple = (0., 0., 1)               # pitch, psi, flip
    approach_deg: float = 60.
    press: tuple = (0., 0., 'vertical', 1)
    park: tuple = (0.24, 0.15, 1.14)          # camera-clear TCP position (hand pointing down)
    safe: tuple = (0.30, 0.10, 1.25)          # via point in front of the machine
    rail: dict = field(default_factory=dict)  # M2/M4: step -> rail setpoint (m)
    ik_collision: bool = False                # IK with static kitchen + dishwasher shell collision avoidance
    door_pre: float = .10                     # pre-grasp stand-off for re-grasping the bar under the open door (m)
    door_regrasp_open: float = 157.           # gripper ctrl while re-grasping the bar under the open door
    note: str = ''
    builder: object = None


# ====================================================================== video
class Recorder:
    def __init__(self, model, path, option_name):
        self.writer = None
        self.next_time = 0.
        self.frames = 0
        self.status = {}
        self.current = ''
        self.substep = ''
        self.option = option_name
        self.speed = 1
        self.view = self.view_target = None
        if path is None:
            return
        self.r = renderer(model, 720, 1280)
        self.writer = imageio.get_writer(str(path), fps=30, codec='libx264', quality=8, macro_block_size=1)
        self.cam = mujoco.MjvCamera()
        self.opt = scene_option(hide_sites=True)
        fp = Path('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
        self.font = ImageFont.truetype(str(fp), 20)
        self.font_s = ImageFont.truetype(str(fp), 16)
        self.font_b = ImageFont.truetype(str(fp), 26)
        self.view = None
        self.view_target = None

    def set_view(self, lookat, dist, az, el, cut=False):
        v = np.array([*lookat, dist, az, el], float)
        self.view_target = v
        if self.view is None or cut:
            self.view = v.copy()

    def hud(self, img, t):
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 0, 1280, 44), fill=(18, 22, 28, 235))
        lab = STEP_LABEL.get(self.current, self.current)
        d.text((14, 9), f'{lab}', fill='white', font=self.font_b)
        d.text((700, 4), f'Franka Panda loads a countertop dishwasher', fill=(200, 210, 220), font=self.font_s)
        d.text((700, 23), f'{self.option}', fill=(200, 210, 220), font=self.font_s)
        d.rectangle((0, 44, 1280, 70), fill=(18, 22, 28, 170))
        d.text((14, 47), self.substep, fill=(255, 220, 120), font=self.font_s)
        d.text((1120, 47), f't = {t:6.1f} s   {self.speed}x', fill=(200, 210, 220), font=self.font_s)
        x0, y0 = 950, 84
        d.rectangle((x0 - 8, y0 - 8, 1272, y0 + 26 * len(STEPS) + 4), fill=(18, 22, 28, 190))
        for i, s in enumerate(STEPS):
            st = self.status.get(s)
            mark, col = ('✓', (90, 230, 120)) if st is True else (('✗', (255, 90, 80)) if st is False else
                                                                     (('▶', (255, 220, 120)) if s == self.current else ('·', (170, 175, 180))))
            d.text((x0, y0 + 26 * i), f'{mark}  {STEP_LABEL[s]}', fill=col, font=self.font_s)
        d.rectangle((0, 676, 1280, 720), fill=(18, 22, 28, 235))
        half = FOOTER.index('Dishwasher internals')
        d.text((14, 679), FOOTER[:half].strip(), fill=(210, 214, 220), font=self.font_s)
        d.text((14, 698), FOOTER[half:].strip(), fill=(210, 214, 220), font=self.font_s)
        return img

    def capture(self, data, force=False):
        if self.writer is None or (data.time + 1e-9 < self.next_time and not force):
            return
        self.next_time = data.time + self.speed / 30
        self.view += .06 * (self.view_target - self.view)
        self.cam.lookat[:] = self.view[:3]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = self.view[3:]
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        img = Image.fromarray(self.r.render())
        self.writer.append_data(np.asarray(self.hud(img, data.time)))
        self.frames += 1

    def card(self, data, image, text, seconds=3.):
        """Still frames (sim paused), e.g. the camera image with the Codex box and the depth fit."""
        if self.writer is None:
            return
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        base = Image.fromarray(self.r.render())
        inset = Image.fromarray(image).resize((800, 450))
        base.paste(inset, (60, 120))
        d = ImageDraw.Draw(base, 'RGBA')
        d.rectangle((60, 572, 860, 640), fill=(18, 22, 28, 220))
        for k, line in enumerate(text.split('\n')[:3]):
            d.text((70, 576 + 20 * k), line, fill=(255, 230, 150), font=self.font_s)
        frame = np.asarray(self.hud(base, data.time))
        for _ in range(int(30 * seconds)):
            self.writer.append_data(frame)
            self.frames += 1

    def still(self, data, path):
        if self.writer is None:
            return
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        self.hud(Image.fromarray(self.r.render()), data.time).save(path)

    def close(self):
        if self.writer is not None:
            self.writer.close()
            self.r.close()


VIEWS = dict(
    overview=((0.05, 0.22, 1.0), 2.1, 205, -22),
    perceive=((-0.05, 0.40, 0.95), 1.3, 215, -35),
    door=((0.18, 0.02, 1.08), 1.25, 128, -12),
    pod=((0.02, 0.15, 1.00), 1.10, 150, -38),
    basket=((0.10, 0.02, 1.10), 1.20, 140, -25),
    flat=((-0.03, 0.40, 0.93), 0.95, 148, -38),
    rack=((-0.20, 0.62, 1.00), 1.0, 225, -25),
    carry=((0.05, 0.22, 1.15), 1.4, 200, -22),
    insert=((0.13, 0.07, 1.15), 0.75, 145, -18),
    buttons=((0.0, 0.06, 1.31), 0.9, 148, -14),
)


# ====================================================================== runner
class Runner:
    def __init__(self, opt: Option, plate_off=(0, 0, 0), video=None, outdir=None, codex=True, seed=0, log=print,
                 keyframes_dir=None, run_tag=''):
        self.opt = opt
        self.log_fn = log
        self.outdir = Path(outdir or ROOT / 'out')
        self.codex = codex
        self.rng = np.random.default_rng(seed)
        self.keyframes_dir = keyframes_dir
        lay = opt.layout
        if lay.plate_mode == 'flat':
            xy, yaw = np.array(lay.plate0) + np.array(plate_off[:2]), plate_off[2]
        else:
            xy, yaw = np.array(plate_off[:2]), plate_off[2]
        self.plate_off = [float(plate_off[0]), float(plate_off[1]), math.degrees(plate_off[2])]
        self.pname = 'plate'
        self.slot_k = len(dw.TINE_X) - 3
        self.model = opt.builder() if getattr(opt, 'builder', None) else S.build(opt.mount, plate_xy=xy, plate_yaw=yaw, layout=lay)[0]
        self.ik = S.build_ik(opt.mount, opt.layout, env=opt.ik_collision)
        m, d = self.model, mujoco.MjData(self.model)
        self.data = d
        self.arm_q = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.fq = [m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]
        self.act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 9)]
        self.has_rail = opt.mount.kind in ('M2', 'M4')
        ik = self.ik
        self.ik_arm = [ik.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        if self.has_rail:
            self.rail_qs = m.joint('rail_slide').qposadr[0]
            self.rail_act = m.actuator('rail_act').id
            self.ik_rail = ik.joint('rail_slide').qposadr[0]
            self.ik_rail_dof = ik.joint('rail_slide').dofadr[0]
        q0 = np.array(opt.q_home if opt.q_home is not None else self.solve_home(), float)
        self.q_park = q0.copy()
        d.qpos[self.arm_q] = q0
        d.qpos[self.fq] = .04
        d.ctrl[self.act[:7]] = q0
        d.ctrl[self.act[7]] = OPEN
        mujoco.mj_forward(m, d)
        qik = np.zeros(ik.nq)
        qik[self.ik_arm] = q0
        qik[[ik.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]] = .04
        self.cfg = mink.Configuration(ik, q=qik)
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=.5, lm_damping=1.)
        self.posture = mink.PostureTask(ik, cost=1e-4)
        self.posture.set_target(qik.copy())
        vel = {f'panda/joint{i}': 1.2 for i in range(1, 8)}
        if self.has_rail:
            vel['rail_slide'] = .3
        self.coll_limit = []
        if opt.ik_collision:
            rg, eg = S.ik_collision_pairs(ik)
            self.coll_limit = [mink.CollisionAvoidanceLimit(ik, [(rg, eg)], gain=.5, minimum_distance_from_collisions=.01,
                                                            collision_detection_distance=.06)]
        self.limits = [mink.ConfigurationLimit(ik), mink.VelocityLimit(ik, vel)] + self.coll_limit
        arm_dofs = [ik.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)]
        self.rail_freeze = mink.DofFreezingTask(ik, [i for i in range(ik.nv) if i not in arm_dofs])
        # ids
        self.pid = -1
        try:
            self.pod_id, self.dish_id = m.body('pod').id, m.body('dish').id
        except KeyError:
            self.pod_id = self.dish_id = -1
        self.robot_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/')}
        self.finger_bodies = {m.body('panda/left_finger').id, m.body('panda/right_finger').id}
        self.hand_body = m.body('panda/hand').id
        self.door_j = m.joint('dw/door_hinge').qposadr[0]
        self.basket_j = m.joint('dw/basket_slide').qposadr[0]
        self.rec = Recorder(m, video, f'option {opt.name}: {opt.mount.label()}')
        self.cam = None
        self.ref = {}
        self.events = []            # unexpected contacts
        self.wrench = []            # (t, F_world on link0 from cart, M about mount, world)
        self.steps = []
        self.metrics = dict(max_penetration_mm=0., max_pen_where=None)
        self.allowed = set()        # body ids the robot may touch in the current phase
        self.phase = ''
        self.ft_hist = []
        self.contact_log = {}
        self.t_wall0 = time.monotonic()
        self.codex_s = []
        self.run_tag = run_tag or f'{opt.name}_{os.getpid()}'

    def solve_home(self):
        """Initial arm configuration = IK solution of the camera-clear park pose (hand down), from several seeds;
        the first solution with >= 10 mm clearance to the static kitchen / dishwasher shell is used."""
        ik = self.ik
        arm = [ik.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        arm_dofs = [ik.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)]
        cons = [mink.DofFreezingTask(ik, [i for i in range(ik.nv) if i not in arm_dofs])]
        task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(hand_frame([0, 0, -1.], [0, 1., 0])),
                                                               np.array(self.opt.park)))
        rg, eg = S.ik_collision_pairs(ik) if self.opt.ik_collision else ([], [])
        dd = mujoco.MjData(ik)
        rng = np.random.default_rng(3)
        lo = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        hi = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        seeds = [np.array(self.opt.q_seed)] + [rng.uniform(lo + .2, hi - .2) for _ in range(30)]
        best = None
        for sd in seeds:
            q = np.zeros(ik.nq)
            q[arm] = sd
            if self.opt.mount.kind in ('M2', 'M4') and 'start' in self.opt.rail:
                q[ik.joint('rail_slide').qposadr[0]] = self.opt.rail['start']
            cfg = mink.Configuration(ik, q=q)
            post = mink.PostureTask(ik, cost=1e-3)
            post.set_target(q)
            for _ in range(300):
                v = mink.solve_ik(cfg, [task, post], 1., solver=SOLVER, damping=1e-4, limits=[mink.ConfigurationLimit(ik, gain=.5)],
                                  constraints=cons)
                cfg.integrate_inplace(v, 1.)
            e = task.compute_error(cfg)
            if np.linalg.norm(e[:3]) > 2e-3:
                continue
            clear = .05
            if rg:
                dd.qpos[:] = cfg.q
                mujoco.mj_kinematics(ik, dd)
                mujoco.mj_collision(ik, dd) if False else None
                for a in rg:
                    for b in eg:
                        clear = min(clear, mujoco.mj_geomDistance(ik, dd, a, b, .05, None))
            qa = cfg.q[arm]
            marg = float(np.min(np.minimum(qa - lo, hi - qa)))
            sc = (clear >= .01, min(marg, .3))
            if best is None or sc > best[0]:
                best = (sc, qa.copy())
            if sc[0] and marg > .3:
                break
        if best is None:
            raise RuntimeError('park pose unreachable')
        return best[1]

    def _ik(self, cfg, tasks, dt, limits, cons, damping=1e-5):
        """solve_ik; if the collision-avoidance limit makes the QP infeasible, solve that tick without it."""
        try:
            return mink.solve_ik(cfg, tasks, dt, solver=SOLVER, damping=damping, limits=limits, constraints=cons)
        except mink.exceptions.NoSolutionFound:
            self.metrics['ik_collision_fallbacks'] = self.metrics.get('ik_collision_fallbacks', 0) + 1
            lim = [l for l in limits if not isinstance(l, mink.CollisionAvoidanceLimit)]
            return mink.solve_ik(cfg, tasks, dt, solver=SOLVER, damping=damping, limits=lim, constraints=cons)

    # ------------------------------------------------------------------ configuration-level planning helpers
    FLIP = np.diag([-1., -1, 1])          # 180 deg about the approach axis (parallel gripper is symmetric)

    def solve_q(self, p, R, seeds=None, iters=250):
        """Goal configuration for a TCP pose on the robot-only IK model: multi-seed, scored by joint-limit
        margin and distance from the current command. Returns (q_full_ik, min_margin_rad, err_m)."""
        ik = self.ik
        cur = self.cfg.q.copy()
        seeds = seeds if seeds is not None else [cur[self.ik_arm], self.q_park] + \
            [self.q_park + np.random.default_rng(k).normal(0, .4, 7) for k in range(3)]
        lo = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        hi = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
        post = mink.PostureTask(ik, cost=1e-3)
        cons = [self.rail_freeze]
        best = None
        for sd in seeds:
            q = cur.copy()
            q[self.ik_arm] = np.clip(sd, lo + .05, hi - .05)
            cfg = mink.Configuration(ik, q=q)
            post.set_target(q)
            for _ in range(iters):
                v = self._ik(cfg, [task, post], 1., [mink.ConfigurationLimit(ik, gain=.5)] + self.coll_limit, cons, damping=1e-4)
                cfg.integrate_inplace(v, 1.)
                e = task.compute_error(cfg)
                if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
                    break
            e = task.compute_error(cfg)
            qa = cfg.q[self.ik_arm]
            marg = float(np.min(np.minimum(qa - lo, hi - qa)))
            err = float(np.linalg.norm(e[:3]))
            ok = err < 1e-3 and np.linalg.norm(e[3:]) < 1e-2
            score = (ok, round(min(marg, .25), 2), -float(np.linalg.norm(qa - cur[self.ik_arm])))
            if best is None or score > best[0]:
                best = (score, cfg.q.copy(), marg, err)
        return best[1], best[2], best[3]

    def plan_path(self, poses, flips=(1,), extra_seeds=6, _rev=False):
        """Joint path for a contact path (list of (p, R)), solved sequentially from several seeds and both
        hand flips; keeps the branch with the largest worst-case joint margin (capped at 0.3 rad), ties broken by
        distance from the current configuration. Returns (flip, [q_ik ...], worst_margin_rad)."""
        ik = self.ik
        cur = self.cfg.q.copy()
        lo = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        hi = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        rng = np.random.default_rng(7)
        # densify: <= 4 cm and <= 10 deg between consecutive poses; keep the indices of the given poses
        orig_poses = list(poses)
        dense, keep = [poses[0]], [0]
        for (pa, Ra), (pb, Rb) in zip(poses[:-1], poses[1:]):
            n_ = int(max(1, math.ceil(np.linalg.norm(np.asarray(pb) - pa) / .04), math.ceil(angle_deg(Ra.T @ Rb) / 10)))
            fn = self.line(pa, Ra, pb, Rb)
            for k_ in range(1, n_ + 1):
                dense.append(fn(k_ / n_))
            keep.append(len(dense) - 1)
        poses = dense
        seeds = [cur[self.ik_arm], self.q_park] + [self.q_park + rng.normal(0, .6, 7) for _ in range(extra_seeds)]
        best = None
        for f in flips:
            for sd in seeds:
                qs, prev, worst, ok = [], [sd], 9., True
                for p, R in poses:
                    Rf = R if f == 1 else R @ self.FLIP
                    q, marg, err = self.solve_q(p, Rf, seeds=[np.clip(prev[-1] if not qs else qs[-1][self.ik_arm], lo + .02, hi - .02)])
                    if err > 1e-3 or (qs and np.max(np.abs(q[self.ik_arm] - qs[-1][self.ik_arm])) > .6):
                        ok = False
                        break
                    worst = min(worst, marg)
                    qs.append(q)
                if not ok:
                    continue
                score = (round(min(worst, .3), 2), -float(np.linalg.norm(qs[0][self.ik_arm] - cur[self.ik_arm])))
                if best is None or score > best[0]:
                    best = (score, f, qs, worst)
        if best is None:
            if not _rev:                           # try solving the path backwards (end pose first), then reverse it
                f, qs, wm = self.plan_path(list(reversed(orig_poses)), flips, extra_seeds + 6, _rev=True)
                return (f, list(reversed(qs)), wm) if qs is not None else (1, None, -1.)
            return 1, None, -1.
        self.plan_log = getattr(self, 'plan_log', []) + [(self.rec.current, best[1], round(math.degrees(best[3]), 1))]
        return best[1], [best[2][k_] for k_ in keep], best[3]

    def choose_flip(self, poses):
        """Pick hand frame R or R*FLIP for a whole step (list of (p, R)): sequentially solvable with the
        larger worst-case joint margin. Returns +1 / -1."""
        res = {}
        cur = self.cfg.q[self.ik_arm].copy()
        for f in (1, -1):
            worst, prev, dist = 9., None, 0.
            for p, R in poses:
                Rf = R if f == 1 else R @ self.FLIP
                q, marg, err = self.solve_q(p, Rf, seeds=None if prev is None else [prev[self.ik_arm]])
                if err > 1e-3:
                    marg = -1.
                if prev is None:
                    dist = float(np.linalg.norm(q[self.ik_arm] - cur))
                worst = min(worst, marg)
                prev = q
            res[f] = (round(min(worst, .25), 2), -dist)
        self.flip_log = getattr(self, 'flip_log', []) + [(self.rec.current, {k: [round(x, 2) for x in v] for k, v in res.items()})]
        return 1 if res[1] >= res[-1] else -1

    def in_counter_area(self, p):
        """Over the counter beside the dishwasher (plate / dish area): reach it from above only."""
        return p[0] < .12 and abs(p[1] - S.DW_POS[1]) > .25

    def transfer(self, name, p1, R1, grip, speed=.3, wspeed=1.0, rail=None, q_hint=None):
        """Collision-aware-by-construction transfer: lift off the counter area, pass the hub point in front
        of the machine (orientation changes there), then descend from above into the counter area."""
        p0, R0 = self.ref.get('cmd', self.tcp())
        p1 = np.asarray(p1, float)
        zc = self.opt.counter_clear_z
        hub = np.array(self.opt.hub)
        legs = []
        a0, a1 = self.in_counter_area(p0), self.in_counter_area(p1)
        # close in front of the machine (door / basket / slot / buttons): back straight out first, keep the hand attitude
        if -.05 < p0[0] < .28 and abs(p0[1] - S.DW_POS[1]) < .32 and p0[2] > S.COUNTER_Z + .02:
            legs.append((np.r_[.30, p0[1], max(p0[2], 1.12)], R0))
        if a0 and p0[2] < zc:
            legs.append((np.r_[p0[:2], zc], R0))
        if a0 != a1 or (a0 and a1 and abs(p0[1] - p1[1]) > .4):
            legs.append((hub, R1))
        if a1:
            legs.append((np.r_[p1[:2], max(p1[2], zc)], R1))
        legs.append((p1, R1))
        for k, (p, R) in enumerate(legs):
            last = k == len(legs) - 1
            self.goto(name, p, R, grip, speed=speed, wspeed=wspeed, rail=rail if k == 0 else None,
                      q_hint=q_hint if (R is R1 and p is not hub) else None)

    # ------------------------------------------------------------------ sensing helpers
    def tcp(self):
        s = self.data.site('tcp')
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def hand_force_world(self):
        """Wrist F/T: force from the arm (link7) on the hand subtree, world frame. At rest without contact
        this is +m_hand g; external upward contact load reduces its z component."""
        R = self.data.site('hand_ft').xmat.reshape(3, 3)
        return R @ self.data.sensor('hand_force').data

    def body_contacts(self, body_a, others=None):
        """Sum of contact forces ON body_a (world) from bodies in `others` (None = any), count."""
        m, d = self.model, self.data
        f6 = np.zeros(6)
        tot, n = np.zeros(3), 0
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom[0]]), int(m.geom_bodyid[c.geom[1]])
            if body_a not in (b1, b2):
                continue
            other = b2 if b1 == body_a else b1
            if others is not None and other not in others:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            fw = c.frame.reshape(3, 3).T @ f6[:3]
            tot += fw if b2 == body_a else -fw
            n += 1
        return tot, n

    def touching(self, body_a):
        m, d = self.model, self.data
        out = set()
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom[0]]), int(m.geom_bodyid[c.geom[1]])
            if body_a == b1:
                out.add(m.geom(c.geom[1]).name or m.body(b2).name)
            elif body_a == b2:
                out.add(m.geom(c.geom[0]).name or m.body(b1).name)
        return out

    def _monitor(self):
        m, d = self.model, self.data
        for i in range(d.ncon):
            c = d.contact[i]
            pen = -float(c.dist)
            if pen > self.metrics['max_penetration_mm'] / 1000:
                self.metrics['max_penetration_mm'] = pen * 1000
                self.metrics['max_pen_where'] = [m.geom(g).name or m.body(int(m.geom_bodyid[g])).name for g in c.geom] + [self.phase]
            b1, b2 = int(m.geom_bodyid[c.geom[0]]), int(m.geom_bodyid[c.geom[1]])
            r1, r2 = b1 in self.robot_bodies, b2 in self.robot_bodies
            if r1 == r2:
                continue
            rb, ob = (b1, b2) if r1 else (b2, b1)
            og = c.geom[1] if r1 else c.geom[0]
            oname = m.geom(og).name or m.body(ob).name
            key = (m.body(rb).name, oname)
            allowed = rb in self.finger_bodies and (ob in self.allowed or oname in self.allowed)
            if not allowed:
                f6 = np.zeros(6)
                mujoco.mj_contactForce(m, d, i, f6)
                k = (self.rec.current, self.phase) + key
                prev = self.contact_log.get(k, 0.)
                self.contact_log[k] = max(prev, float(abs(f6[0])))

    def _sample_wrench(self):
        d = self.data
        R = d.site('mount_ft').xmat.reshape(3, 3)
        F = R @ d.sensor('mount_force').data
        M = R @ d.sensor('mount_torque').data
        self.wrench.append((d.time, *F, *M))

    # ------------------------------------------------------------------ motion
    def seg(self, name, dur, target, grip, hook=None, stop=None, rail=None, qpath=None):
        m, d = self.model, self.data
        if qpath is not None:
            self.posture.cost[:] = 1e-2
        self.phase = name
        g0 = float(d.ctrl[self.act[7]])
        steps, nctl = max(1, round(dur / m.opt.timestep)), round(CONTROL_DT / m.opt.timestep)
        r0 = float(d.ctrl[self.rail_act]) if self.has_rail else 0.
        for k in range(steps):
            u = (k + 1) / steps
            s = u * u * u * (10 + u * (-15 + 6 * u))
            if k % nctl == 0:
                p, R = target(s)
                if qpath is not None:
                    x = s * (len(qpath) - 1)
                    i0 = min(int(x), len(qpath) - 2) if len(qpath) > 1 else 0
                    w = x - i0 if len(qpath) > 1 else 0.
                    self.posture.set_target(qpath[i0] * (1 - w) + qpath[min(i0 + 1, len(qpath) - 1)] * w)
                if hook is not None:
                    p = hook(p)
                self.ref['last_target'] = (np.asarray(p, float).copy(), np.asarray(R, float).copy())
                self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
                q_prev = self.cfg.q.copy()
                if self.has_rail and rail is not None:     # commanded rail motion: set, arm IK continues
                    q = self.cfg.q.copy()
                    q[self.ik_rail] = r0 + s * (rail - r0)
                    self.cfg.update(q)
                cons = [self.rail_freeze]
                v = self._ik(self.cfg, [self.task, self.posture], CONTROL_DT, self.limits, cons)
                self.cfg.integrate_inplace(v, CONTROL_DT)
                self.ref['q_seg'] = (q_prev, self.cfg.q.copy())
            qa, qb = self.ref['q_seg']
            qi = qa + ((k % nctl) + 1) / nctl * (qb - qa)
            d.ctrl[self.act[:7]] = qi[self.ik_arm]
            if self.has_rail:
                d.ctrl[self.rail_act] = qi[self.ik_rail]
            d.ctrl[self.act[7]] = g0 + s * (grip - g0)
            mujoco.mj_step(m, d)
            self._monitor()
            if k % 5 == 0:
                self._sample_wrench()
            if 'cb' in self.ref:
                self.ref['cb']()
            self.rec.capture(d)
            if k % nctl == 0 and 'last_target' in self.ref:
                e = 1000 * float(np.linalg.norm(self.tcp()[0] - self.ref['last_target'][0]))
                tm = self.metrics.setdefault('track_err_max_mm', {})
                tm[name] = round(max(tm.get(name, 0.), e), 2)
            if stop is not None and stop():
                if qpath is not None:
                    self.posture.cost[:] = 1e-4
                return True
        if qpath is not None:
            self.posture.cost[:] = 1e-4
        return False

    def jmove(self, name, q_goal, dur, grip, p1=None, R1=None):
        """Joint-space quintic move of the IK reference to q_goal (free-space re-orientation)."""
        m, d = self.model, self.data
        self.phase = name
        g0 = float(d.ctrl[self.act[7]])
        q0 = self.cfg.q.copy()
        steps = max(1, round(dur / m.opt.timestep))
        for k in range(steps):
            u = (k + 1) / steps
            s = u * u * u * (10 + u * (-15 + 6 * u))
            q = q0 + s * (q_goal - q0)
            if k % 5 == 0:
                self.cfg.update(q)
            d.ctrl[self.act[:7]] = q[self.ik_arm]
            if self.has_rail:
                d.ctrl[self.rail_act] = q[self.ik_rail]
            d.ctrl[self.act[7]] = g0 + s * (grip - g0)
            mujoco.mj_step(m, d)
            self._monitor()
            if k % 5 == 0:
                self._sample_wrench()
            if 'cb' in self.ref:
                self.ref['cb']()
            self.rec.capture(d)
        self.cfg.update(q_goal)
        if p1 is not None:
            self.ref['cmd'] = (np.asarray(p1, float), np.asarray(R1, float))
            self.ref['last_target'] = self.ref['cmd']

    def line(self, p0, R0, p1, R1):
        so0 = mink.SO3.from_matrix(R0)
        delta = (so0.inverse() @ mink.SO3.from_matrix(R1)).log()
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        return lambda s: (p0 + s * (p1 - p0), (so0 @ mink.SO3.exp(s * delta)).as_matrix())

    def move(self, name, dur, p1, R1, grip, hook=None, stop=None, rail=None, qpath=None):
        p0, R0 = self.ref.get('cmd', self.tcp())
        hit = self.seg(name, dur, self.line(p0, R0, p1, R1), grip, hook, stop, rail, qpath)
        self.ref['cmd'] = (np.asarray(p1, float), np.asarray(R1, float))
        return hit

    def goto(self, name, p1, R1, grip, speed=.25, wspeed=1.0, min_dur=1.0, rail=None, joint=None, q_hint=None):
        p0, R0 = self.ref.get('cmd', self.tcp())
        dist = float(np.linalg.norm(np.asarray(p1) - p0))
        ang = math.radians(angle_deg(R0.T @ R1))
        dur = max(min_dur, 1.875 * dist / speed, 1.875 * ang / wspeed)   # quintic peak speed = 1.875 * mean
        if rail is not None and self.has_rail:
            dur = max(dur, 1.875 * abs(rail - float(self.data.ctrl[self.rail_act])) / .3)
        qg, marg, err = self.solve_q(np.asarray(p1, float), np.asarray(R1, float),
                                     seeds=None if q_hint is None else [q_hint[self.ik_arm]])
        if joint is None:
            joint = ang > math.radians(45)
        if joint and err < 1e-3:
            dq = float(np.max(np.abs(qg[self.ik_arm] - self.cfg.q[self.ik_arm])))
            jd = max(1.0, 1.875 * dq / .8, dur * .8)
            self.rec.speed = 2 if jd > 2.5 else 1
            self.posture.set_target(qg)
            self.jmove(name, qg, jd, grip, p1, R1)
            self.rec.speed = 1
            self.jmoves = getattr(self, 'jmoves', 0) + 1
            return
        self.posture.set_target(qg)
        self.posture.cost[:] = 1e-2
        self.rec.speed = 2 if dur > 2.5 else 1
        self.move(name, dur, p1, R1, grip, rail=rail)
        self.rec.speed = 1
        self.posture.cost[:] = 1e-4

    def hold(self, name, dur, grip=None, hook=None, stop=None):
        p, R = self.ref.get('cmd', self.tcp())
        g = float(self.data.ctrl[self.act[7]]) if grip is None else grip
        return self.move(name, dur, p, R, g, hook=hook, stop=stop)

    def rail_to(self, step):
        """M2/M4: drive the linear axis to this step's setpoint (arm joints follow the IK, TCP target fixed in the arm frame)."""
        if not self.has_rail or step not in self.opt.rail:
            return
        y1 = self.opt.rail[step]
        y0 = float(self.data.ctrl[self.rail_act])
        if abs(y1 - y0) < 1e-4:
            return
        p, R = self.ref.get('cmd', self.tcp())
        dur = max(1.0, 1.875 * abs(y1 - y0) / .3)
        self.rec.speed = 2 if dur > 2.5 else 1
        self.substep(f'linear axis {y0:+.2f} -> {y1:+.2f} m')
        self.move('rail', dur, p + [0, y1 - y0, 0], R, float(self.data.ctrl[self.act[7]]), rail=y1)
        self.rec.speed = 1

    def substep(self, text):
        self.rec.substep = text

    # ------------------------------------------------------------------ step wrapper
    def run_step(self, name, fn):
        self.rec.current = name
        self.rec.status[name] = None
        t0, w0 = self.data.time, time.monotonic()
        rec = dict(step=name, t_start=round(t0, 3))
        try:
            out = fn() or {}
            ok, reason = out.pop('_ok', True), out.pop('_reason', '')
        except StepFail as e:
            ok, reason, out = False, str(e), getattr(e, 'metrics', {})
        rec.update(ok=bool(ok), reason=reason, t_end=round(self.data.time, 3), sim_s=round(self.data.time - t0, 3),
                   wall_s=round(time.monotonic() - w0, 1), metrics=out)
        if self.has_rail:
            rec['rail_m_end'] = round(float(self.data.qpos[self.rail_qs]), 4)
        self.steps.append(rec)
        self.rec.status[name] = bool(ok)
        self.log_fn(f"  [{'OK ' if ok else 'FAIL'}] {name:15s} t={t0:6.1f}-{self.data.time:6.1f}s  {reason}  "
                    + ', '.join(f'{k}={v}' for k, v in out.items() if not isinstance(v, (dict, list)))[:400])
        if self.keyframes_dir is not None:
            self.rec.still(self.data, Path(self.keyframes_dir) / f'{len(self.steps):02d}_{name}.png')
        if not ok:
            raise StepFail(f'{name}: {reason}')
        return out

    def fail(self, reason, **metrics):
        e = StepFail(reason)
        e.metrics = metrics
        raise e

    # ================================================================== perception
    def perceive(self, tag, mode):
        if self.cam is None:
            self.cam = PC.Camera(self.model)
        rgb, depth, seg = self.cam.capture(self.data, self.rng)
        if not self.codex:
            raise StepFail('perception needs Codex (--no-codex given)')
        self.substep(f'Codex call ({tag}) ...')
        tag = f'{self.run_tag}_{tag}'                 # unique per run: parallel runs must not share image files
        vlm = PC.ask_codex(rgb, tag, mode, self.outdir / 'codex')
        self.codex_s.append(vlm.get('seconds'))
        if not vlm.get('found'):
            self.fail(f'Codex did not find the plate ({vlm.get("error", "")})', codex_s=vlm.get('seconds'))
        est = (PC.estimate_flat if mode == 'flat' else PC.estimate_rack)(self.cam, self.data, depth, vlm)
        est['mode'] = mode
        # scoring only
        true_c = self.data.site('plate_center').xpos.copy()
        if mode == 'flat':
            est_c = est['origin'] + est['P'][:, 2] * S.MID_Z
        else:
            est_c = est['origin'] + est['P'][:, 2] * S.MID_Z
        err = est_c - true_c
        px_err = float(np.linalg.norm(np.array(vlm['center']) - self.cam.project(self.data, true_c)))
        ntrue = self.data.xmat[self.pid].reshape(3, 3)[:, 2]
        res = dict(codex_s=vlm['seconds'], codex_bbox=[round(v) for v in vlm['bbox']], codex_center=vlm['center'],
                   codex_center_px_err=round(px_err, 1), radius_fit_mm=round(est['radius_fit'] * 1000, 1),
                   centre_err_mm=round(float(np.linalg.norm(err)) * 1000, 2),
                   centre_err_xy_mm=[round(float(v) * 1000, 2) for v in err[:2]],
                   normal_err_deg=round(math.degrees(math.acos(min(1., abs(float(est['P'][:, 2] @ ntrue))))), 2))
        ov = PC.overlay(rgb, vlm, self.cam, self.data, est)
        Image.fromarray(ov).save(self.outdir / 'codex' / f'{tag}_overlay.png')
        self.rec.card(self.data, ov, f'Wall RGB-D camera -> Codex ({vlm["seconds"]:.1f} s, sim paused): yellow = Codex box/centre\n'
                      f'Green = depth fit: circle r = {est["radius_fit"]*1000:.1f} mm  (true 101.5 mm)\n'
                      f'Scoring only: centre error vs ground truth {res["centre_err_mm"]:.1f} mm', 3.)
        self.substep('')
        return est, res

    # ================================================================== steps
    def step_perceive(self):
        o = self.opt
        cl = np.array(o.layout.cam_look)
        self.rec.set_view((cl[0] + .05, cl[1], cl[2] + .05), 1.3, 215, -35, cut=True)
        self.rail_to('perceive_plate')
        Rd = hand_frame([0, 0, -1.], [0, 1., 0])
        self.substep('arm to camera-clear pose')
        self.transfer('park', np.array(o.park), Rd, OPEN)
        self.hold('park', .5)
        est, res = self.perceive('perceive1', o.layout.plate_mode)
        self.est = est
        sane = .095 < est['radius_fit'] < .110 and res['normal_err_deg'] < 5 if o.layout.plate_mode == 'flat' else .09 < est['radius_fit'] < .115
        ok = sane and res['centre_err_mm'] < 5.
        return dict(_ok=ok, _reason='' if ok else f'perception check failed (radius {res["radius_fit_mm"]} mm, error {res["centre_err_mm"]} mm)', **res)

    # ------------------------------------------------------------------ door
    def door_frames(self, th):
        kappa, psi, flip = self.opt.door
        R = plan.door_bar_frame(th, kappa, psi, flip)
        if getattr(self, 'door_flip', 1) < 0:
            R = R @ self.FLIP
        return plan.handle_pos(th) + .0085 * R[:, 2], R

    def pick_door_flip(self, ths):
        self.door_flip = 1
        poses = [self.door_frames(math.radians(t)) for t in ths]
        self.door_flip = self.choose_flip(poses)

    def step_open_door(self):
        self.rec.set_view(*VIEWS['door'])
        self.rail_to('open_door')
        self.allowed = {self.model.body('dw/door').id}
        self.door_flip = 1
        ths = np.radians(np.arange(0, 91, 10))
        p0_, R0_ = self.door_frames(0.)
        poses = [(p0_ - .10 * R0_[:, 2], R0_), (p0_, R0_)] + [self.door_frames(t) for t in ths[1:]]
        self.door_flip, qs, wm = self.plan_path(poses, flips=(1, -1))
        if qs is None:
            self.fail('no joint path for the door arc from this mount')
        self.door_open_plan = (self.door_flip, qs)
        p0, R0 = self.door_frames(0.)
        self.substep('approach the pull bar')
        self.transfer('door_pre', p0 - .10 * R0[:, 2] + [0, 0, .02], R0, OPEN, q_hint=qs[0])
        self.move('door_pre', 1.0, p0 - .10 * R0[:, 2], R0, OPEN, qpath=[qs[0]])
        self.move('door_in', 1.5, p0, R0, OPEN, qpath=[qs[0], qs[1]])
        self.substep('grasp the bar (light squeeze, bar may turn between the pads)')
        g = squeeze_ctrl(2 * dw.HANDLE_R, 5.)
        self.move('door_grasp', 1.0, p0, R0, g)
        self.hold('door_grasp', .3)
        n_f = self.body_contacts(self.model.body('dw/door').id, self.finger_bodies)[1]
        fq_ = float(np.mean(self.data.qpos[self.fq]))
        if n_f == 0 or fq_ > dw.HANDLE_R + .004:
            self.fail(f'bar not grasped (finger opening {fq_*1000:.1f} mm each)')
        self.substep('follow the hinge arc 0 -> 90 deg')
        tmax = [0.]

        def arc(s):
            th = s * math.pi / 2
            return self.door_frames(th)

        def cb():
            tmax[0] = max(tmax[0], float(np.linalg.norm(self.body_contacts(self.model.body('dw/door').id, self.finger_bodies)[0])))
        self.ref['cb'] = cb
        self.seg('door_arc', 8.0, arc, g, qpath=qs[1:])
        self.ref['cmd'] = arc(1.)
        self.ref.pop('cb')
        self.hold('door_arc', .5)
        p, R = self.ref['cmd']
        self.substep('release and back off')
        self.move('door_release', .8, p, R, OPEN)
        self.move('door_back', 1.2, p - .10 * R[:, 2], R, OPEN)
        self.goto('door_back', p - .10 * R[:, 2] + [.08, 0, .30], R, OPEN, speed=.3)
        self.allowed = set()
        self.hold('door_settle', .5)
        ang = math.degrees(self.data.qpos[self.door_j])
        ok = ang > 85.
        return dict(_ok=ok, _reason='' if ok else f'door only at {ang:.1f} deg', door_deg=round(ang, 2),
                    peak_finger_force_N=round(tmax[0], 1))

    # ------------------------------------------------------------------ pod
    def pod_state(self):
        """Pod pose in the door frame and whether it is inside the groove recess (scoring)."""
        d = self.data
        pc = d.xpos[self.pod_id]
        door = d.body('dw/door')
        loc = door.xmat.reshape(3, 3).T @ (pc - door.xpos)
        gx0 = -dw.DOOR_T                           # inner face (door-local x)
        in_y = abs(loc[1] - dw.GROOVE_Y) < dw.GROOVE[0] / 2
        in_z = abs(loc[2] - dw.GROOVE_Z) < dw.GROOVE[1] / 2
        depth = loc[0] - gx0                       # > 0: below the inner face, inside the recess
        return dict(in_groove=bool(in_y and in_z and depth > 0), local=[round(float(v) * 1000, 1) for v in loc],
                    depth_below_face_mm=round(float(depth) * 1000, 1),
                    on_tub_floor=bool(not (in_y and in_z and depth > 0) and pc[2] < S.COUNTER_Z + dw.TUB_FLOOR + .02
                                      and abs(pc[0] - S.DW_POS[0]) < .17))

    def step_load_pod(self):
        m, d = self.model, self.data
        self.rec.set_view(*VIEWS['pod'])
        self.rail_to('load_pod')
        dish = np.array(self.opt.layout.dish)
        pod0 = d.xpos[self.pod_id].copy()                 # pod sits in a known dish (fixed place in the kitchen)
        # known dish location -> nominal pod pose (robot does not read the pod's true pose)
        pod_nom = np.array([dish[0], dish[1], S.COUNTER_Z + .0003 + S.DISH_FLOOR_T + S.POD_SIZE[2] / 2])
        Rpick = plan.pod_down_frame([1., 0, 0])            # fingers close across the 32 mm length
        Rplace = plan.pod_down_frame([0, 1., 0])           # rotate 90 deg: 24 mm across the 35 mm groove height
        self.allowed = {self.pod_id, self.dish_id}
        grasp = pod_nom + [0, 0, -.002]
        f, qa_, _ = self.plan_path([(grasp + [0, 0, .08], Rpick), (grasp, Rpick), (grasp + [0, 0, .08], Rpick)], flips=(1, -1))
        if qa_ is None:
            self.fail('no joint path to pick the pod')
        Rpick = Rpick if f > 0 else Rpick @ self.FLIP
        gc0, face0 = plan.groove_center(), plan.groove_face_z()
        place0 = np.array([gc0[0], gc0[1], face0 + .003 + S.POD_SIZE[2] / 2 + .002])
        f, qb_, _ = self.plan_path([(place0 + [0, 0, .08], Rplace), (place0, Rplace)], flips=(1, -1))
        if qb_ is None:
            self.fail('no joint path to place the pod over the groove')
        Rplace = Rplace if f > 0 else Rplace @ self.FLIP
        self.substep('to the pod dish')
        self.transfer('pod_above', grasp + [0, 0, .08], Rpick, grip_ctrl(.022), q_hint=qa_[0])
        self.move('pod_down', 1.5, grasp, Rpick, grip_ctrl(.022), qpath=qa_[:2])
        self.substep('grasp pod (~5 N per finger)')
        g = squeeze_ctrl(S.POD_SIZE[0], 5.)
        self.move('pod_grasp', 1.0, grasp, Rpick, g)
        self.move('pod_lift', 1.2, grasp + [0, 0, .08], Rpick, g, qpath=qa_[1:])
        if self.body_contacts(self.pod_id, self.finger_bodies)[1] == 0 or d.xpos[self.pod_id][2] < pod0[2] + .05:
            self.fail('pod not lifted', pod_z=float(d.xpos[self.pod_id][2]))
        gc = plan.groove_center()
        face = plan.groove_face_z()
        rel = self.tcp()[1].T @ (d.xpos[self.pod_id] - self.tcp()[0])     # pod offset in the hand (proprioception of a
        # grasp at a known point; true pod pose is NOT used for the aim below)
        place = np.array([gc[0], gc[1], face + .003 + S.POD_SIZE[2] / 2 + .002])
        self.substep('carry over the open door, rotate 90 deg')
        self.transfer('pod_over_groove', place + [0, 0, .08], Rplace, g, speed=.25, q_hint=qb_[0])
        self.move('pod_lower', 1.5, place, Rplace, g, qpath=qb_)
        self.substep('release into the groove')
        self.move('pod_release', .8, place, Rplace, grip_ctrl(.026))
        self.hold('pod_release', .4)
        self.move('pod_up', 1.0, place + [0, 0, .08], Rplace, grip_ctrl(.026))
        self.allowed = set()
        self.hold('pod_settle', .6)
        st = self.pod_state()
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.pod_id, vel, 0)
        st['speed_mm_s'] = round(float(np.linalg.norm(vel[3:])) * 1000, 2)
        ok = st['in_groove'] and st['speed_mm_s'] < 5
        return dict(_ok=ok, _reason='' if ok else f'pod not settled in the groove ({st})', **st)

    # ------------------------------------------------------------------ basket
    def basket_frames(self, b):
        pitch, psi, flip = self.opt.basket
        R = plan.basket_bar_frame(pitch, psi, flip)
        if getattr(self, 'basket_flip', 1) < 0:
            R = R @ self.FLIP
        return plan.basket_handle(b) + .0085 * R[:, 2], R

    def pick_basket_flip(self, bs):
        self.basket_flip = 1
        self.basket_flip = self.choose_flip([self.basket_frames(b) for b in bs])

    def basket_pod_clearance(self):
        m, d = self.model, self.data
        best = .05
        bid = m.body('dw/basket').id
        for g in range(m.body_geomadr[self.pod_id], m.body_geomadr[self.pod_id] + m.body_geomnum[self.pod_id]):
            if not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            for h in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
                if np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[h]) > m.geom_rbound[g] + m.geom_rbound[h] + .05:
                    continue
                best = min(best, mujoco.mj_geomDistance(m, d, g, h, .05, None))
        return best

    def step_pull_basket(self):
        d = self.data
        self.rec.set_view(*VIEWS['basket'])
        self.rail_to('pull_basket')
        bid = self.model.body('dw/basket').id
        self.allowed = {bid}
        self.basket_flip = 1
        pa, Ra = self.basket_frames(0.)
        pb, _ = self.basket_frames(dw.BASKET_TRAVEL)
        self.basket_flip, qs, wm = self.plan_path([(pa - .10 * Ra[:, 2], Ra), (pa, Ra), ((pa + pb) / 2, Ra), (pb, Ra)], flips=(1, -1))
        if qs is None:
            self.fail('no joint path for the basket pull from this mount')
        p0, R = self.basket_frames(0.)
        self.substep('grasp the basket pull bar')
        self.transfer('basket_pre', p0 - .10 * R[:, 2] + [0, 0, .03], R, BOPEN, q_hint=qs[0])
        self.move('basket_pre', 1.0, p0 - .10 * R[:, 2], R, BOPEN, qpath=[qs[0]])
        self.move('basket_in', 1.5, p0, R, BOPEN, qpath=[qs[0], qs[1]])
        g = squeeze_ctrl(2 * .004, 5.)
        self.move('basket_grasp', 1.0, p0, R, g)
        clear = [1.]

        def cb():
            clear[0] = min(clear[0], self.basket_pod_clearance())
        self.ref['cb'] = cb
        self.substep('pull out to the end stop (0.25 m)')
        p1 = p0 + [dw.BASKET_TRAVEL + .004, 0, 0]
        self.move('basket_pull', 3.0, p1, R, g, qpath=qs[1:])
        self.hold('basket_pull', .3)
        self.ref.pop('cb')
        # robot's own estimate of the basket position: where the hand ended up (proprioception)
        # the hand was commanded 4 mm past the end stop: the basket sits at its (known) travel limit if the hand
        # stopped beyond it; otherwise trust the hand position (proprioception)
        self.basket_est = float(min(self.tcp()[0][0] - p0[0], dw.BASKET_TRAVEL))
        self.substep('release')
        self.move('basket_release', .8, p1, R, BOPEN)
        self.move('basket_back', 1.2, p1 - .08 * R[:, 2], R, BOPEN)
        self.goto('basket_back', p1 - .08 * R[:, 2] + [0, 0, .15], R, BOPEN, speed=.3)
        self.allowed = set()
        b = float(d.qpos[self.basket_j])
        pod = self.pod_state()
        ok = b >= .245 and pod['in_groove']
        return dict(_ok=ok, _reason='' if ok else f'basket at {b:.3f} m, pod in groove {pod["in_groove"]}', basket_m=round(b, 4),
                    basket_est_m=round(self.basket_est, 4), min_pod_basket_clearance_mm=round(clear[0] * 1000, 1),
                    pod_in_groove=pod['in_groove'])

    def step_push_basket(self):
        d = self.data
        self.rec.set_view(*VIEWS['basket'])
        self.rail_to('push_basket')
        bid = self.model.body('dw/basket').id
        self.allowed = {bid}
        b = self.basket_est
        self.basket_flip = 1
        pa, Ra = self.basket_frames(b)
        pb, _ = self.basket_frames(0.)
        self.basket_flip, qs, wm = self.plan_path([(pa - .10 * Ra[:, 2], Ra), (pa, Ra), ((pa + pb) / 2, Ra), (pb, Ra)], flips=(1, -1))
        if qs is None:
            self.fail('no joint path for the basket push from this mount')
        p1, R = self.basket_frames(b)
        self.substep('grasp the basket pull bar')
        self.transfer('basket_pre', p1 - .10 * R[:, 2] + [0, 0, .03], R, BOPEN, q_hint=qs[0])
        self.move('basket_pre', 1.0, p1 - .10 * R[:, 2], R, BOPEN, qpath=[qs[0]])
        self.move('basket_in', 1.5, p1, R, BOPEN, qpath=[qs[0], qs[1]])
        g = squeeze_ctrl(2 * .004, 5.)
        self.move('basket_grasp', 1.0, p1, R, g)
        self.substep('push in to the back stop')
        p0 = self.basket_frames(0.)[0] - [.004, 0, 0]
        self.move('basket_push', 3.0, p0, R, g, qpath=qs[1:])
        self.hold('basket_push', .3)
        self.move('basket_release', .8, p0, R, BOPEN)
        self.move('basket_back', 1.2, p0 - .10 * R[:, 2], R, BOPEN)
        self.goto('basket_back', p0 - .10 * R[:, 2] + [.05, 0, .15], R, BOPEN, speed=.3)
        self.allowed = set()
        self.hold('basket_settle', .5)
        bpos = float(d.qpos[self.basket_j])
        ps = self.plate_slot_state()
        pod = self.pod_state()
        ok = bpos < .005 and ps['in_slot'] and pod['in_groove']
        return dict(_ok=ok, _reason='' if ok else f'basket {bpos:.3f} m, plate in slot {ps["in_slot"]}, pod {pod["in_groove"]}',
                    basket_m=round(bpos, 4), plate_in_slot=ps['in_slot'], pod_in_groove=pod['in_groove'])

    # ------------------------------------------------------------------ plate
    def plate_c(self):
        return self.data.site(f'{self.pname}_center').xpos.copy()

    def pR(self):
        return self.data.xmat[self.pid].reshape(3, 3).copy()

    def step_pick_plate(self):
        return self.pick_flat() if self.opt.layout.plate_mode == 'flat' else self.pick_rack()

    def pick_flat(self):
        """FLAT_NOTES method A with perceived poses: edge push (open-loop distance from perception #1),
        re-perception #2 at the edge, rim-face-aligned pinch, counter-supported tilt-up, lift."""
        m, d = self.model, self.data
        o = self.opt
        self.rec.set_view(*VIEWS['flat'])
        self.rail_to('pick_plate')
        out = {}
        est = self.est
        org, P = est['origin'], est['P']
        pu = plan.push_poses(org)
        Rp = pu['R']
        _, qp_, _ = self.plan_path([(pu['p_pre'] + [0, 0, .05], Rp), (pu['p_pre'], Rp),
                                    (pu['p_pre'] + pu['drag'] * pu['drag_dir'], Rp)])
        if qp_ is None:
            self.fail('no joint path for the edge push from this mount')
        self.allowed = {self.pid}
        self.substep('push: finger side face on the far rim edge')
        self.transfer('push_approach', pu['p_pre'] + [0, 0, .05], Rp, TOUCH, speed=.25, q_hint=qp_[0])
        self.move('push_approach', 1.0, pu['p_pre'], Rp, TOUCH, qpath=qp_[:2])
        self.hold('push_approach', .3)
        base_ft = np.mean([self.hand_force_world()[2] for _ in range(1)])
        F = dict(z=None, filt=0., active=False, log=[], ft0=float(base_ft))

        def press_hook(p):
            if F['z'] is None:
                F['z'] = p[2]
            if not F['active']:
                if F['filt'] > .5:
                    F['active'] = True
                return np.array([p[0], p[1], F['z']])
            err = F['filt'] - 8.0
            F['z'] += np.clip(.0005 * err * CONTROL_DT, -.0003, .0003)
            return np.array([p[0], p[1], F['z']])

        def cb():
            press = F['ft0'] - self.hand_force_world()[2]     # wrist F/T: downward press on the plate
            F['filt'] += .1 * (press - F['filt'])
            if self.phase == 'push_drag':
                F['log'].append((d.time, press, *self.plate_c()[:2], self.pR()[2, 2]))
        self.ref['cb'] = cb
        self.substep(f'drag {pu["drag"]*1000:.0f} mm toward the edge, 8 N press (wrist F/T admittance)')
        start = self.plate_c()
        # open-loop drag distance from perception #1 (no ground truth in the loop)
        self.move('push_drag', pu['drag'] / .04, pu['p_pre'] + pu['drag'] * pu['drag_dir'], Rp, TOUCH, hook=press_hook,
                  qpath=qp_[1:])
        self.ref.pop('cb')
        ph = self.tcp()[0]
        self.move('push_release', .6, ph + .015 * plan.E_FAR + [0, 0, .005], Rp, TOUCH)
        self.move('push_release', 1.0, ph + .02 * plan.E_FAR + [0, 0, .08], hand_frame([0, 0, -1.], [0, 1., 0]), TOUCH)
        lg = np.array(F['log']) if F['log'] else np.zeros((1, 5))
        mv = np.r_[0, np.linalg.norm(np.diff(lg[:, 2:4], axis=0), axis=1)] > 1e-5
        c1 = self.plate_c()
        overhang = float((c1[0] + S.R_OUT) * 1000)            # counter edge at x = 0 (scoring)
        out.update(push_travel_mm=round(float((c1 - start)[0]) * 1000, 1), overhang_true_mm=round(overhang, 1),
                   push_press_mean_N=round(float(lg[mv, 1].mean()), 2) if mv.any() else None,
                   push_press_peak_N=round(float(lg[:, 1].max()), 1),
                   push_tilt_max_deg=round(float(np.degrees(np.arccos(np.clip(lg[:, 4].min(), -1, 1)))), 2) if len(lg) > 1 else None,
                   push_lateral_drift_mm=round(float(c1[1] - start[1]) * 1000, 1))
        if not 25 < overhang < 70:
            self.fail(f'push left {overhang:.1f} mm overhang (need ~45)', **out)
        # ---- re-perceive at the edge
        self.substep('re-perceive the plate at the edge')
        self.transfer('park', np.array(o.park), hand_frame([0, 0, -1.], [0, 1., 0]), OPEN)
        self.hold('park', .4)
        est2, res2 = self.perceive('perceive2', 'flat')
        out.update({f'perc2_{k}': v for k, v in res2.items()})
        out['overhang_est_mm'] = round(float((est2['origin'][0] + S.R_OUT) * 1000), 1)
        self.rec.set_view(*VIEWS['flat'])
        org2, P2 = est2['origin'], est2['P']
        tcp_g, Rg, pinch = plan.pinch_pose(org2, P2, approach_deg=o.approach_deg)
        path0 = plan.tilt_path(P2, org2, n=60)
        Rh_p0, ph_p0 = P2.T @ Rg, P2.T @ (tcp_g - org2)
        tilt_poses = [(path0[i_][2] + path0[i_][1] @ ph_p0, path0[i_][1] @ Rh_p0) for i_ in range(0, 61, 10)]
        kposes = ([(tcp_g - Rg[:, 2] * .12 + [0, 0, .03], Rg), (tcp_g - Rg[:, 2] * .12, Rg), (tcp_g, Rg)]
                  + tilt_poses[1:] + [(tilt_poses[-1][0] + [0, 0, .12], tilt_poses[-1][1])])
        _, qk_, _ = self.plan_path(kposes)
        if qk_ is None:
            names = ['pinch_pre', 'pinch_in', 'pinch'] + [f'tilt{15 * i_}' for i_ in range(1, 7)] + ['lift']
            diag = {}
            for nm_, (pp_, RR_) in zip(names, kposes):
                q_, mg_, er_ = self.solve_q(pp_, RR_)
                diag[nm_] = dict(pos_err_mm=round(er_ * 1000, 1), margin_deg=round(math.degrees(mg_), 1))
            self.fail('no joint path for pinch + tilt + lift from this mount (per-pose IK, independent seeds: '
                      + ', '.join(f"{k_} {v_['pos_err_mm']} mm" for k_, v_ in diag.items()) + ')', reach=diag, **out)
        def chain(Rg_):
            Rh_p_, ph_p_ = P2.T @ Rg_, P2.T @ (tcp_g - org2)
            out_ = [(tcp_g, Rg_)]
            for i_ in (20, 40, 60):
                _, Rpl_, ppl_ = path0[i_]
                out_.append((ppl_ + Rpl_ @ ph_p_, Rpl_ @ Rh_p_))
            out_.append((out_[-1][0] + [0, 0, .12], out_[-1][1]))
            return out_
        hz = Rg[:, 2]
        self.substep('pinch the overhang, pads on the 15 deg rim face')
        self.transfer('pinch_pre', tcp_g - hz * .12 + [0, 0, .03], Rg, OPEN, speed=.25, q_hint=qk_[0])
        self.move('pinch_pre', 1.0, tcp_g - hz * .12, Rg, OPEN, qpath=qk_[:2])
        self.move('pinch_in', 2.0, tcp_g, Rg, OPEN, qpath=qk_[1:3])
        self.move('close', 1.2, tcp_g, Rg, 0.)
        self.move('close', .3, tcp_g, Rg, 0.)
        nf = self.body_contacts(self.pid, self.finger_bodies)[1]
        out['pinch_contacts'] = nf
        if nf == 0:
            self.fail('no finger contact after the pinch', **out)
        p, R = self.tcp()
        # measured-at-grasp in-hand transform: hand pose (encoders) relative to the PERCEIVED plate pose
        self.T_hp = (P2.T @ R, P2.T @ (p - org2))
        ref_rel = R.T @ (self.plate_c() - p)                 # scoring: true slip in the hand frame
        ref_R = R.T @ self.pR()
        mx = dict(slip=0., rot=0.)

        def track():
            p_, R_ = self.tcp()
            mx['slip'] = max(mx['slip'], float(np.linalg.norm(R_.T @ (self.plate_c() - p_) - ref_rel)))
            mx['rot'] = max(mx['rot'], angle_deg((R_.T @ self.pR()) @ ref_R.T))
        self.ref['cb'] = track
        self.substep('tilt up about the far rim (rim stays on the counter), 6 s')
        path = plan.tilt_path(P2, org2)
        Rh_p, ph_p = self.T_hp

        def tgt(s):
            i = min(len(path) - 1, int(round(s * (len(path) - 1))))
            _, Rpl, ppl = path[i]
            return ppl + Rpl @ ph_p, Rpl @ Rh_p
        self.seg('tilt', 6.0, tgt, 0., qpath=qk_[2:-1])
        self.ref['cmd'] = tgt(1.)
        out['tilt_inhand_slip_mm'] = round(mx['slip'] * 1000, 2)
        self.plate_R_est, self.plate_o_est = path[-1][1], path[-1][2]
        self.substep('lift 0.12 m, hold 1 s')
        p1, R1 = self.ref['cmd']
        self.move('lift', 2.0, p1 + [0, 0, .12], R1, 0., qpath=qk_[-2:])
        p_h, R_h = self.tcp()
        hold_rel, hold_R = R_h.T @ (self.plate_c() - p_h), R_h.T @ self.pR()
        mh = dict(slip=0., rot=0.)

        def track_hold():
            track()
            p_, R_ = self.tcp()
            mh['slip'] = max(mh['slip'], float(np.linalg.norm(R_.T @ (self.plate_c() - p_) - hold_rel)))
            mh['rot'] = max(mh['rot'], angle_deg((R_.T @ self.pR()) @ hold_R.T))
        self.ref['cb'] = track_hold
        self.hold('hold', 1.0, 0.)
        self.ref.pop('cb')
        self.plate_o_est = self.plate_o_est + [0, 0, .12]
        nf = self.body_contacts(self.pid, self.finger_bodies)[1]
        onc = 'counter_top' in self.touching(self.pid)
        n = self.pR()[:, 2]
        tilt = math.degrees(math.asin(min(1., abs(float(n[2])))))
        held = nf > 0 and not onc and self.plate_c()[2] > S.COUNTER_Z + S.R_OUT
        ok = held and tilt < 10 and mh['slip'] < .005
        out.update(held=held, plane_from_vertical_deg=round(tilt, 2), hold_slip_mm=round(mh['slip'] * 1000, 2),
                   inhand_slip_since_grasp_mm=round(mx['slip'] * 1000, 2), inhand_rot_since_grasp_deg=round(mx['rot'], 2),
                   finger_contacts=nf)
        return dict(_ok=ok, _reason='' if ok else 'plate not held upright', **out)

    def pick_rack(self):
        """../poc_plate method: rim pinch at the top of the upright plate, pads on the rim face, lift 0.17 m."""
        o = self.opt
        rk = self.opt.layout.rack
        self.rec.set_view((rk[0] + .05, rk[1] - .05, 1.02), 1.1, 150, -22)
        self.rail_to('pick_plate')
        est = self.est
        P, origin = est['P'], est['origin']
        sgn = 1. if P[:, 1][2] > 0 else -1.
        up = sgn * P[:, 1]
        nrm = P @ np.array([0, -sgn * math.sin(S.SLOPE), math.cos(S.SLOPE)])
        hy = unit(nrm)
        hz = unit(-up - (-up @ hy) * hy)
        Rg = np.column_stack([np.cross(hy, hz), hy, hz])
        r_p = S.R_OUT - .012
        z_rim = .0075 + S.RIM_RISE * (r_p - S.R_DISC) / (S.R_OUT - S.R_DISC)
        grasp = origin + P @ np.array([0, sgn * r_p, z_rim])
        f, qr_, _ = self.plan_path([(grasp - hz * .12, Rg), (grasp, Rg), (grasp + up * .17, Rg)], flips=(1, -1))
        if qr_ is None:
            self.fail('no joint path for the rack grasp from this mount')
        Rg = Rg if f > 0 else Rg @ self.FLIP
        self.allowed = {self.pid}
        self.substep('approach the rim from above, pads on the rim face')
        self.transfer('rack_approach', grasp - hz * .12, Rg, 200., speed=.25, q_hint=qr_[0])
        self.move('rack_descend', 2.0, grasp, Rg, 200., qpath=qr_[:2])
        self.move('close', 1.2, grasp, Rg, 0.)
        p, R = self.tcp()
        self.T_hp = (P.T @ R, P.T @ (p - origin))
        ref_rel, ref_R = R.T @ (self.plate_c() - p), R.T @ self.pR()
        mx = dict(slip=0.)

        def track():
            p_, R_ = self.tcp()
            mx['slip'] = max(mx['slip'], float(np.linalg.norm(R_.T @ (self.plate_c() - p_) - ref_rel)))
        self.ref['cb'] = track
        self.substep('lift 0.17 m along the plate plane')
        start = self.plate_c()
        self.move('lift', 2.5, grasp + up * .17, Rg, 0., qpath=qr_[1:])
        self.hold('hold', .5, 0.)
        self.ref.pop('cb')
        rise = float((self.plate_c() - start) @ up)
        self.plate_R_est, self.plate_o_est = P, origin + up * .17
        nf = self.body_contacts(self.pid, self.finger_bodies)[1]
        ok = rise > .16 and mx['slip'] < getattr(self, 'rack_slip_tol', .005) and nf > 0
        return dict(_ok=ok, _reason='' if ok else f'rise {rise:.3f} m, slip {mx["slip"]*1000:.1f} mm',
                    lift_rise_mm=round(rise * 1000, 1), lift_slip_mm=round(mx['slip'] * 1000, 2), finger_contacts=nf)

    # ------------------------------------------------------------------ insert
    def plate_slot_state(self):
        """Scoring. In the slot = the plate's lowest point sits between its own two tines (along x, the slot
        normal) and within the tine rows (y), plane < 10 deg from the slot plane, touching only its own slot's
        tines (+ floor wires), settled, robot not touching it. The centre offset is reported but not used: a
        plate in a 24 mm gap legitimately leans onto one of its tines."""
        m, d = self.model, self.data
        sx = (dw.TINE_X[self.slot_k] + dw.TINE_X[self.slot_k + 1]) / 2
        slot_site = d.xpos[m.body('dw/basket').id] + np.array([sx, sum(dw.PLATE_ROWS_Y) / 2, dw.WIRE_R + dw.PLATE_R + .001])
        err = self.plate_c() - slot_site
        R = self.pR()
        n = R[:, 2]
        plane_deg = math.degrees(math.acos(min(1., abs(float(n[0])))))
        # lowest point of the rim circle (outer edge, lip height)
        th = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        rim = d.xpos[self.pid] + (np.outer(np.cos(th), R[:, 0]) + np.outer(np.sin(th), R[:, 1])) * S.R_OUT + R[:, 2] * .0173
        low = rim[np.argmin(rim[:, 2])]
        bx = float(low[0] - slot_site[0])
        touch = self.touching(self.pid)
        k0, k1 = self.slot_k, self.slot_k + 1
        own = {f'dw/plate_tine{r}_{k}' for r in (0, 1) for k in (k0, k1)}
        wrong = sorted(t for t in touch if t.startswith('dw/plate_tine') and t not in own)
        other = sorted(t for t in touch if not (t.startswith('dw/plate_tine') or t.startswith('dw/bfloor') or t.startswith('dw/frame_b')))
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.pid, vel, 0)
        half_gap = dw.TINE_PITCH / 2 - dw.TINE_R
        in_slot = (abs(bx) < half_gap and abs(err[1]) < .03 and plane_deg < 10 and not wrong and not other)
        return dict(in_slot=bool(in_slot), bottom_offset_mm=round(bx * 1000, 1), centre_err_mm=[round(float(v) * 1000, 1) for v in err],
                    plane_tilt_deg=round(plane_deg, 2), wrong_tines=wrong, other_contacts=other,
                    speed_mm_s=round(float(np.linalg.norm(vel[3:])) * 1000, 2),
                    touching_robot=any(t.startswith('panda/') for t in touch))

    def step_insert(self):
        d = self.data
        o = self.opt
        self.rec.set_view(*VIEWS['carry'])
        self.rail_to('insert_plate')
        Rh_p, ph_p = self.T_hp
        Rpl = self.plate_R_est
        n = Rpl[:, 2]
        cur_p, cur_R = self.ref['cmd']
        slot = plan.plate_slot(self.basket_est, self.slot_k)
        lift_h = 1.32 - (self.plate_o_est[2] + S.MID_Z)      # plate centre to z 1.32 (clear of basket frame + bar)
        via1 = cur_p + [0, 0, max(0., lift_h)]
        best = None
        for yaw_opt in (0., math.pi):
            # target attitude: in-hand estimate rotated minimally so its normal is along x (the slot normal);
            # yaw_opt = pi: the plate faces the other way (it is symmetric, the slot accepts both)
            tgt_n = (np.array([-1., 0, 0]) if n[0] < 0 else np.array([1., 0, 0]))
            ax = np.cross(n, tgt_n)
            Rc = rot_axis(ax, math.asin(min(1., np.linalg.norm(ax)))) if np.linalg.norm(ax) > 1e-9 else np.eye(3)
            R_slot = rot_z(yaw_opt) @ Rc @ Rpl
            # aim 10 mm toward the convex side (-normal) (gpt_engineer p2_lean fix, was 3 mm): less free tipping onto
            # its own tine; the lowest lip (+6.8 mm along the normal) remains inside the 24 mm gap
            org = slot - .010 * R_slot[:, 2] - R_slot @ np.array([0, 0, S.MID_Z])
            Rh = R_slot @ Rh_p
            ph = org + R_slot @ ph_p
            via2 = np.array([ph[0], via1[1], via1[2]])
            via3 = np.array([ph[0], ph[1], via1[2]])
            f, qs, wm = self.plan_path([(via2, Rh), (via3, Rh), (ph + [0, 0, .09], Rh), (ph, Rh), (ph + [0, 0, .12], Rh)])
            if qs is not None and (best is None or wm > best[0]):
                best = (wm, yaw_opt, Rh, ph, via2, via3, qs)
            if best is not None and best[1] == 0. and best[0] > math.radians(8):
                break                     # yaw 0 (validated insertion) is preferred when it has >= 8 deg margin
        if best is None:
            self.fail('no joint path for carry + insert with the plate in hand from this mount')
        wm, yaw_opt, Rh, ph, via2, via3, qs = best
        self.substep('carry: up, out in front of the machine, across, over the slot')
        self.goto('carry_up', via1, cur_R, 0., speed=.15)
        self.goto('carry_out', via2, Rh, 0., speed=.15, wspeed=.5, q_hint=qs[0])
        above = ph + [0, 0, .09]
        self.rec.set_view(*VIEWS['carry'])
        self.move('carry_across', max(2., 1.875 * float(np.linalg.norm(via3 - via2)) / .15), via3, Rh, 0., qpath=qs[:2])
        self.rec.set_view(*VIEWS['insert'])
        self.move('carry_down', max(2., 1.875 * float(np.linalg.norm(above - via3)) / .12), above, Rh, 0., qpath=qs[1:3])
        insert_meta = dict(slot_yaw_deg=round(math.degrees(yaw_opt)), carry_plan_margin_deg=round(math.degrees(wm), 1))
        self.hold('carry_settle', .5, 0.)
        # touchdown detection with the wrist F/T: support = loss of carried weight
        w0 = float(np.mean([self.hand_force_world()[2]]))
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
        rel_open = grip_ctrl(.006)          # multi_dish_b: open only ~6 mm per finger so the hand stays off the neighbouring plate
        self.move('release', 1.0, stop_p, Rh, rel_open)
        self.move('retract', 1.5, stop_p + [0, 0, .12], Rh, rel_open)
        self.allowed = set()
        self.hold('settle', 1.0)
        st = self.plate_slot_state()
        ok = st['in_slot'] and st['speed_mm_s'] < 5 and not st['touching_robot']
        return dict(_ok=ok, _reason='' if ok else f'plate not seated: {st}', touchdown_detected=bool(hit), **insert_meta,
                    touchdown_support_filt_N=round(sup['filt'], 2), touchdown_support_peak_N=round(sup.get('peak', 0.), 2), **st)

    # ------------------------------------------------------------------ close door, buttons
    def step_close_door(self):
        self.rec.set_view(*VIEWS['door'])
        self.rail_to('close_door')
        did = self.model.body('dw/door').id
        self.allowed = {did}
        self.door_flip = 1
        ths = np.radians(np.arange(90, -1, -10))
        p9_, R9_ = self.door_frames(math.pi / 2)
        poses = [(p9_ - self.opt.door_pre * R9_[:, 2], R9_)] + [self.door_frames(t) for t in ths]
        self.door_flip, qs, wm = self.plan_path(poses, flips=(1, -1))
        if qs is None and getattr(self, 'door_open_plan', None) is not None:
            # same arc backwards: reuse the opening joint path (same hand flip); pre-grasp solved next to its end
            self.door_flip, qo = self.door_open_plan
            p9_, R9_ = self.door_frames(math.pi / 2)
            qpre, _, err = self.solve_q(p9_ - self.opt.door_pre * R9_[:, 2], R9_, seeds=[qo[-1][self.ik_arm]])
            qs = [qpre] + list(reversed(qo[2:])) + [qo[1]] if err < 1e-3 else None
            self.plan_log = getattr(self, 'plan_log', []) + [('close_door', 'reversed opening path', qs is not None)]
        if qs is None:
            self.fail('no joint path for closing the door from this mount')
        p9, R9 = self.door_frames(math.pi / 2)
        self.substep('re-grasp the bar under the open door')
        self.goto('door_via', np.array([p9[0] + .16, p9[1], 1.15]), R9, self.opt.door_regrasp_open, speed=.3)
        self.goto('door_pre', p9 - self.opt.door_pre * R9[:, 2] + [.03, 0, -.02], R9, self.opt.door_regrasp_open, speed=.3, q_hint=qs[0])
        self.move('door_pre', 1.0, p9 - self.opt.door_pre * R9[:, 2], R9, self.opt.door_regrasp_open, qpath=[qs[0]])
        self.move('door_in', 1.5, p9, R9, self.opt.door_regrasp_open, qpath=[qs[0], qs[1]])
        g = squeeze_ctrl(2 * dw.HANDLE_R, 5.)
        self.move('door_grasp', 1.0, p9, R9, g)
        fq_ = float(np.mean(self.data.qpos[self.fq]))
        if self.body_contacts(did, self.finger_bodies)[1] == 0 or fq_ > dw.HANDLE_R + .004:
            self.fail(f'bar not grasped (finger opening {fq_*1000:.1f} mm each)')
        self.substep('push the door up along the hinge arc to 0 deg')

        def arc(s):
            th = (1 - s) * math.pi / 2 + s * math.radians(-1.0)       # 1 deg past closed: press onto the stop
            return self.door_frames(th)
        self.seg('door_close_arc', 8.0, arc, g, qpath=qs[1:])
        self.ref['cmd'] = arc(1.)
        self.hold('door_close_arc', .4)
        p, R = self.ref['cmd']
        self.move('door_release', .8, p, R, OPEN)
        self.move('door_back', 1.2, p - .10 * R[:, 2], R, OPEN)
        self.allowed = set()
        self.hold('door_settle', 1.0)
        ang = math.degrees(self.data.qpos[self.door_j])
        ok = abs(ang) < 1.
        return dict(_ok=ok, _reason='' if ok else f'door at {ang:.2f} deg', door_deg=round(ang, 3),
                    door_closed_sensor=bool(dw.door_closed(self.model, self.data)))

    def step_buttons(self):
        d = self.data
        self.rec.set_view(*VIEWS['buttons'])
        self.rail_to('press_buttons')
        out = {}
        pitch, yaw, close, flip = self.opt.press
        R = plan.press_frame(pitch, yaw, close, flip)
        f, qbt_, _ = self.plan_path([(plan.button_pos('quick') - .04 * R[:, 2], R), (plan.button_pos('quick'), R),
                                     (plan.button_pos('power') - .04 * R[:, 2], R), (plan.button_pos('power'), R)], flips=(1, -1))
        if qbt_ is None:
            self.fail('no joint path to the buttons from this mount')
        R = R if f > 0 else R @ self.FLIP
        for b in ('quick', 'power'):
            pad_body = self.model.body(f'dw/btn_{b}_body').id
            self.allowed = {pad_body}
            face = plan.button_pos(b)
            self.substep(f'press {b.upper()}: advance until the wrist F/T reads 3 N')
            if b == 'quick':
                self.transfer(f'btn_{b}_pre', face - .04 * R[:, 2], R, TOUCH, speed=.25, q_hint=qbt_[0])
            else:
                self.goto(f'btn_{b}_pre', face - .04 * R[:, 2], R, TOUCH, speed=.25)  # noqa
            self.hold(f'btn_{b}_pre', .3)
            f0 = self.hand_force_world().copy()
            peak = [0.]

            def stop():
                f = float((self.hand_force_world() - f0) @ R[:, 2])   # pad pushes back along -a -> link7 pushes +a
                peak[0] = max(peak[0], float(d.sensor(f'dw/btn_{b}_touch').data[0]))
                return f > 3.0
            self.move(f'btn_{b}_near', 1.5, face - .008 * R[:, 2], R, TOUCH)
            hit = self.move(f'btn_{b}_press', 6.0, face + .006 * R[:, 2], R, TOUCH, stop=stop)
            self.ref['cmd'] = (self.tcp()[0], R)

            def cbk():
                peak[0] = max(peak[0], float(d.sensor(f'dw/btn_{b}_touch').data[0]))
            self.ref['cb'] = cbk
            self.hold(f'btn_{b}_hold', .3)
            self.ref.pop('cb')
            self.move(f'btn_{b}_off', 1.0, face - .05 * R[:, 2], R, TOUCH)
            out[f'{b}_touch_peak_N'] = round(peak[0], 2)
            out[f'{b}_ft_stop'] = bool(hit)
        self.allowed = set()
        ok = out['quick_touch_peak_N'] > 1.0 and out['power_touch_peak_N'] > 1.0
        return dict(_ok=ok, _reason='' if ok else 'a touch pad did not register > 1 N', touch_threshold_N=1.0, **out)

    # ================================================================== run
    def run(self):
        self.rec.set_view(*VIEWS['overview'], cut=True)
        self.phase = 'settle'
        self.ref['cmd'] = self.tcp()
        self.hold('settle', .5, OPEN)
        failed = None
        fns = dict(perceive_plate=self.step_perceive, open_door=self.step_open_door, load_pod=self.step_load_pod,
                   pull_basket=self.step_pull_basket, pick_plate=self.step_pick_plate, insert_plate=self.step_insert,
                   push_basket=self.step_push_basket, close_door=self.step_close_door, press_buttons=self.step_buttons)
        for s in STEPS:
            try:
                self.run_step(s, fns[s])
            except StepFail as e:
                failed = str(e)
                self.substep(f'STOPPED: {failed}'[:150])
                self.hold('stopped', 2.0)
                break
            except Exception as e:  # noqa: BLE001  (a crash is a failure of that step, reported as such)
                failed = f'{s}: exception {e!r}'
                self.steps.append(dict(step=s, ok=False, reason=failed, t_start=round(self.data.time, 3)))
                self.rec.status[s] = False
                break
        if failed is None:
            self.rec.set_view(*VIEWS['overview'])
            self.substep('done')
            self.ref['cmd'] = self.ref.get('cmd', self.tcp())
            self.transfer('home', np.array(self.opt.park), hand_frame([0, 0, -1.], [0, 1., 0]), TOUCH)
            self.hold('end', 1.5)
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(self.data.warning) if w.number}
        self.rec.close()
        if self.cam is not None:
            self.cam.close()
        unexpected = [dict(step=k[0], phase=k[1], robot=k[2], other=k[3], max_force_N=round(v, 2)) for k, v in self.contact_log.items()]
        for st in self.steps:
            fs = [v for k, v in self.contact_log.items() if k[0] == st['step']]
            st['unplanned_contact_max_N'] = round(max(fs), 1) if fs else 0.
            st['clean'] = bool(st['ok'] and (max(fs) if fs else 0.) <= CLEAN_N)
        return dict(option=self.opt.name, mount=self.opt.mount.__dict__, plate_offset_mm_deg=self.plate_off,
                    success=failed is None, clean_success=failed is None and all(s_['clean'] for s_ in self.steps),
                    failed=failed, steps=self.steps, plan_log=getattr(self, 'plan_log', []),
                    ik_collision_fallbacks=self.metrics.get('ik_collision_fallbacks', 0),
                    rail_setpoints=self.opt.rail,
                    sim_time_s=round(self.data.time, 2), wall_s=round(time.monotonic() - self.t_wall0, 1),
                    codex_s=self.codex_s, warnings=warnings, max_penetration_mm=round(self.metrics['max_penetration_mm'], 3),
                    max_pen_where=self.metrics['max_pen_where'], unexpected_robot_contacts=unexpected,
                    track_err_max_mm=self.metrics.get('track_err_max_mm'), video_frames=self.rec.frames,
                    wrench=np.array(self.wrench, float))
