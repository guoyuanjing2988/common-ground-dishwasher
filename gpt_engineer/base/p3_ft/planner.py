"""MotionPlanner = the main real-time computer (100 Hz): mink IK + joint-path planning for the arm, base navigation and
docking. It is the 9-step task code of ../full_pipeline (pipeline.Runner, reused) with its I/O replaced:

  * it never writes MuJoCo controls: every 10 ms it sends an ArmSetpoint (joint targets over the next period) to the
    ArmController and a BaseCmd (twist) / Brake to the BaseMCU;
  * it never reads the robot's true state: the TCP / wrist frame are forward kinematics of the joint encoders
    (ArmState from the ArmController) placed at the MEASURED base pose (lidar + odometry + IMU tilt, or the `truth`
    base-pose source = a perfect base sensor). The IK model has a free base that is set to this pose every tick;
  * the camera / Codex is asked through the Perception node (asynchronous; the arm waits parked).
Ground truth is still read for the success predicates, scoring and logs (as in ../full_pipeline), never for control.

Execution model: the planner's (blocking, sequential) task code runs in its own thread in strict lock-step with the
scheduler (a coroutine): `self.tick()` hands control back and resumes at the next 100 Hz planner tick. Nothing runs
concurrently. All 500 Hz ArmState samples received during a period are replayed through the step's F/T filters and
stop conditions, so guarded moves keep their 500 Hz semantics (decision applied at the next planner tick, <= 10 ms).
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import mink
import mujoco
import numpy as np
from PIL import Image

import dishwasher as dw
import perception as PC
import pipeline as PL
import plan
import scene as S
import scene_mobile as SM
from util import angle_deg, hand_frame

PLAN_DT = .01
NCTL = 5


class Coroutine:
    """Thread-backed coroutine in strict lock-step with the caller (no concurrency)."""

    def __init__(self, fn):
        self.fn = fn
        self._go, self._back = threading.Semaphore(0), threading.Semaphore(0)
        self.done, self.exc, self.result = False, None, None
        self.th = threading.Thread(target=self._run, daemon=True)
        self.started = False

    def _run(self):
        self._go.acquire()
        try:
            self.result = self.fn()
        except BaseException as e:  # noqa: BLE001
            self.exc = e
        self.done = True
        self._back.release()

    def resume(self):
        if self.done:
            return
        if not self.started:
            self.started = True
            self.th.start()
        self._go.release()
        self._back.acquire()
        if self.exc is not None:
            e, self.exc = self.exc, None
            raise e

    def yield_(self):
        self._back.release()
        self._go.acquire()


class Abort(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


class MotionPlanner(PL.Runner):
    name, rate = 'MotionPlanner', 100

    def __init__(self, sysm, opt, codex=True, seed=0, log=print, keyframes_dir=None, run_tag='', base_pose_source='lidar',
                 base_start_est=None, believed_base=None, tcp_cap=None, press_tool='stylus', outdir=None):
        self.sys, self.bus = sysm, sysm.bus
        self.period = 5
        self.opt = opt
        self.log_fn = log
        self.outdir = outdir
        self.codex = codex
        self.rng = np.random.default_rng(seed)
        self.keyframes_dir = keyframes_dir
        self.plate_off = [0, 0, 0]
        self.model, self.data = sysm.m, sysm.d
        m = self.model
        self.ik = SM.build_ik(sysm.cfg)
        ik = self.ik
        self.arm_q = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.fq = [m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]
        self.act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 9)]
        self.has_rail = False
        self.ik_arm = [ik.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.ik_f = [ik.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]
        q0 = np.array(self.data.qpos[self.arm_q])
        self.q_park = SM.Q_STOW.copy()
        qik = np.zeros(ik.nq)
        qik[3] = 1.
        qik[self.ik_arm] = q0
        qik[self.ik_f] = .04
        self.cfg = mink.Configuration(ik, q=qik)
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=.5, lm_damping=1.)
        self.posture = mink.PostureTask(ik, cost=1e-4)
        self.posture.set_target(qik.copy())
        self.coll_limit = []
        self.limits = [mink.ConfigurationLimit(ik), mink.VelocityLimit(ik, {f'panda/joint{i}': 1.2 for i in range(1, 8)})]
        arm_dofs = [ik.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)]
        self.rail_freeze = mink.DofFreezingTask(ik, [i for i in range(ik.nv) if i not in arm_dofs])
        self.fk = mujoco.MjData(ik)
        self.pid, self.pod_id, self.dish_id = m.body('plate').id, m.body('pod').id, m.body('dish').id
        self.robot_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/')}
        self.finger_bodies = {m.body('panda/left_finger').id, m.body('panda/right_finger').id}
        self.hand_body = m.body('panda/hand').id
        self.door_j = m.joint('dw/door_hinge').qposadr[0]
        self.basket_j = m.joint('dw/basket_slide').qposadr[0]
        self.rec = sysm.rec
        self.cam = None
        self.ref, self.events, self.wrench, self.steps = {}, [], [], []
        self.metrics = dict(max_penetration_mm=0., max_pen_where=None)
        self.allowed, self.phase, self.ft_hist, self.contact_log = set(), '', [], {}
        self.t_wall0 = time.monotonic()
        self.codex_s = []
        self.run_tag = run_tag
        # ---- mobile state
        self.base_pose_source = base_pose_source
        self.believed_base = believed_base           # spec pose-tolerance sweep: planner believes this base pose
        self.base_est = np.array(base_start_est if base_start_est is not None else SM.START_NOMINAL[:2] + (math.radians(SM.START_NOMINAL[2]),), float)
        self.tilt = (0., 0.)
        self.odom_prev = None
        self.dock_fixes = deque(maxlen=30)
        self.docked = False
        self.base_state = None
        self.arm_state = None
        self.samples = []
        self.paused = False
        self.cmds = deque()
        self.perc_result = None
        self.q_sent = q0.copy()
        self.g_sent = float(self.data.ctrl[self.act[7]])
        self.job = ''
        self.cur = None
        self.tcp_cap = tcp_cap
        self.press_tool = press_tool
        self.step_stats = {}
        self.fix_count = dict(front=0, rear=0, used=0, rejected=0)
        self.nav_log = []
        self.cancel = False
        self.touchdown_filter = (.2, .5, 5)                 # insert touchdown: EMA alpha, threshold N, consecutive samples
        self.insert_aim = .008                              # insert aim offset toward -plate normal (m)
        self.co = Coroutine(self.main)
        self.k = 0

    # ================================================================== scheduler interface
    def tick_node(self, t):
        """Called by the scheduler every 10 ms (main thread): resume the planner thread for one period."""
        self.co.resume()

    def tick(self):
        """Planner thread: give control back; resumes at the next 100 Hz tick with the inbox processed."""
        self.co.yield_()
        self.k += 1
        self._inbox()
        if self.cancel and self.job:
            self.cancel = False
            raise PL.StepFail('cancelled by the TaskSupervisor (operator abort)')

    def send(self, dst, typ, data=None, info=None):
        return self.bus.send(self.data.time, self.name, dst, typ, data, info)

    def _inbox(self):
        self.samples = []
        for msg in self.bus.recv(self.name):
            t = msg.typ
            if t == 'ArmState':
                self.samples.append(msg.data)
                self.arm_state = msg.data
            elif t == 'BaseState':
                self._odom(msg.data)
            elif t == 'LidarFix':
                self._fix(msg.data)
            elif t == 'Command':
                self.cmds.append(msg.data)
            elif t == 'EStop' or t == 'Pause':
                self.paused = True
            elif t == 'Resume':
                self.paused = False
            elif t == 'Cancel':
                self.cancel = True
            elif t == 'PerceiveResult':
                self.perc_result = msg.data
        if self.k % 10 == 0:
            self.send('TaskSupervisor', 'PlannerStatus', dict(job=self.job, phase=self.phase, paused=self.paused))

    # ================================================================== base pose estimate
    def _odom(self, st):
        self.base_state = st
        self.tilt = (st['roll'], st['pitch'])
        o = st['odom']
        if self.odom_prev is not None:
            d_ = o - self.odom_prev
            c, s_ = math.cos(-self.odom_prev[2]), math.sin(-self.odom_prev[2])
            dx_b, dy_b = c * d_[0] - s_ * d_[1], s_ * d_[0] + c * d_[1]       # increment in the base frame
            th = self.base_est[2]
            self.base_est += [math.cos(th) * dx_b - math.sin(th) * dy_b, math.sin(th) * dx_b + math.cos(th) * dy_b, d_[2]]
            self.base_est[2] = wrap(self.base_est[2])
        self.odom_prev = o.copy()

    def _fix(self, f):
        if not f.get('ok'):
            return
        self.fix_count[f['lidar']] += 1
        e = self.base_est
        dth = wrap(f['th'] - e[2])
        if abs(dth) > math.radians(12) or abs(f['x'] - e[0]) > .15 or ('y' in f and abs(f['y'] - e[1]) > .15):
            self.fix_count['rejected'] += 1                 # gate: not the cabinet line / not the strip
            return
        self.fix_count['used'] += 1
        if self.docked:
            self.dock_fixes.append((f['x'], f.get('y', np.nan), f['th']))
            return
        k = .3
        self.base_est[0] += k * (f['x'] - e[0])
        self.base_est[2] = wrap(e[2] + k * dth)
        if 'y' in f:
            self.base_est[1] += (k if f['lidar'] == 'rear' or abs(dth) < .5 else .15) * (f['y'] - e[1])

    def base_pose_arm(self):
        """(x, y, yaw) used by the arm. 'truth' = perfect base sensor; 'lidar' = mean of the last 3 s of lidar fixes
        while docked & braked (odometry-propagated estimate otherwise). believed_base overrides (pose sweep)."""
        if self.believed_base is not None:
            return np.array(self.believed_base, float)
        if self.base_pose_source == 'truth':
            p = self.data.sensor('base_pos').data
            q = self.data.sensor('base_quat').data
            return np.array([p[0], p[1], 2 * math.atan2(q[3], q[0])])
        if self.docked and len(self.dock_fixes) >= 5:
            F = np.array(self.dock_fixes)
            y = np.nanmean(F[:, 1]) if np.isfinite(F[:, 1]).any() else self.base_est[1]
            th = math.atan2(np.mean(np.sin(F[:, 2])), np.mean(np.cos(F[:, 2])))
            return np.array([np.mean(F[:, 0]), y, th])
        return self.base_est.copy()

    def base_q7(self):
        x, y, th = self.base_pose_arm()
        roll, pitch = self.tilt if self.believed_base is None else (0., 0.)
        return np.r_[x, y, 0., quat_rpy(roll, pitch, th)]

    def update_ik_base(self):
        q = self.cfg.q.copy()
        q[:7] = self.base_q7()
        self.cfg.update(q)

    # ================================================================== estimated robot state
    def _fk(self, st=None):
        st = st or self.cur or self.arm_state
        self.fk.qpos[:7] = self.base_q7()
        self.fk.qpos[self.ik_arm] = st['q']
        self.fk.qpos[self.ik_f] = st['f']
        mujoco.mj_kinematics(self.ik, self.fk)
        return st

    def tcp(self):
        self._fk()
        s = self.fk.site('tcp')
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def hand_force_world(self):
        st = self._fk()
        return self.fk.site('hand_ft').xmat.reshape(3, 3) @ st['F']

    def stylus_world(self):
        self._fk()
        return self.fk.site('stylus').xpos.copy()

    # ================================================================== arm motion (replaces Runner.seg / jmove)
    def _send_arm(self, q1, g1):
        self.send('ArmController', 'ArmSetpoint', dict(q0=self.q_sent.copy(), q1=np.asarray(q1, float).copy(), g0=self.g_sent,
                                                      g1=float(g1), n=NCTL))
        self.q_sent, self.g_sent = np.asarray(q1, float).copy(), float(g1)

    def _wait_unpaused(self):
        while self.paused:
            self.tick()

    def _replay(self, cb, stop):
        hit = False
        for s in self.samples:
            self.cur = s
            if cb is not None:
                cb()
            if stop is not None and not hit and stop():
                hit = True
        self.cur = None
        return hit

    def seg(self, name, dur, target, grip, hook=None, stop=None, rail=None, qpath=None):
        if qpath is not None:
            self.posture.cost[:] = 1e-2
        self.phase = name
        g0 = self.g_sent
        nt = max(1, round(dur / PLAN_DT))
        hit = False
        for k in range(nt):
            self._wait_unpaused()
            u = (k + 1) / nt
            s = u * u * u * (10 + u * (-15 + 6 * u))
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
            self.update_ik_base()
            v = self._ik(self.cfg, [self.task, self.posture], PLAN_DT, self.limits, [self.rail_freeze])
            self.cfg.integrate_inplace(v, PLAN_DT)
            self._send_arm(self.cfg.q[self.ik_arm], g0 + s * (grip - g0))
            self.tick()
            if self._replay(self.ref.get('cb'), stop):
                hit = True
            if self.arm_state is not None:
                e = 1000 * float(np.linalg.norm(self.tcp()[0] - self.ref['last_target'][0]))
                tm = self.metrics.setdefault('track_err_max_mm', {})
                tm[name] = round(max(tm.get(name, 0.), e), 2)
            if hit:
                break
        if qpath is not None:
            self.posture.cost[:] = 1e-4
        return hit

    def jmove(self, name, q_goal, dur, grip, p1=None, R1=None):
        self.phase = name
        g0 = self.g_sent
        q0 = self.cfg.q.copy()
        if self.tcp_cap:
            dur = max(dur, self._jmove_min_dur(q0, q_goal))
        nt = max(1, round(dur / PLAN_DT))
        for k in range(nt):
            self._wait_unpaused()
            u = (k + 1) / nt
            s = u * u * u * (10 + u * (-15 + 6 * u))
            q = q0 + s * (q_goal - q0)
            q[:7] = self.base_q7()
            self.cfg.update(q)
            self._send_arm(q[self.ik_arm], g0 + s * (grip - g0))
            self.tick()
            self._replay(self.ref.get('cb'), None)
        q = q_goal.copy()
        q[:7] = self.base_q7()
        self.cfg.update(q)
        if p1 is not None:
            self.ref['cmd'] = (np.asarray(p1, float), np.asarray(R1, float))
            self.ref['last_target'] = self.ref['cmd']

    def _jmove_min_dur(self, q0, q1):
        """Duration so that the quintic joint move keeps the TCP and elbow below the speed cap (sampled FK)."""
        dd = mujoco.MjData(self.ik)
        pts = []
        for s in np.linspace(0, 1, 21):
            dd.qpos[:] = q0 + s * (q1 - q0)
            dd.qpos[:7] = self.base_q7()
            mujoco.mj_kinematics(self.ik, dd)
            pts.append((dd.site('tcp').xpos.copy(), dd.body('panda/link4').xpos.copy()))
        L = max(sum(np.linalg.norm(pts[i + 1][j] - pts[i][j]) for i in range(20)) for j in (0, 1))
        return 1.875 * L / self.tcp_cap

    def goto(self, name, p1, R1, grip, speed=.25, wspeed=1.0, min_dur=1.0, rail=None, joint=None, q_hint=None):
        if self.tcp_cap:
            speed = min(speed, self.tcp_cap)
        return super().goto(name, p1, R1, grip, speed, wspeed, min_dur, rail, joint, q_hint)

    def move(self, name, dur, p1, R1, grip, hook=None, stop=None, rail=None, qpath=None):
        if self.tcp_cap:
            p0, _ = self.ref.get('cmd', self.tcp())
            dur = max(dur, 1.875 * float(np.linalg.norm(np.asarray(p1) - p0)) / self.tcp_cap)
        return super().move(name, dur, p1, R1, grip, hook, stop, rail, qpath)

    def solve_home(self):
        """Park configuration (camera-clear pose, hand down) for the CURRENT measured base pose."""
        ik = self.ik
        task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(hand_frame([0, 0, -1.], [0, 1., 0])),
                                                               np.array(self.opt.park)))
        lo = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        hi = ik.jnt_range[[ik.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        rng = np.random.default_rng(3)
        best = None
        for sd in [np.array(self.opt.q_seed)] + [rng.uniform(lo + .2, hi - .2) for _ in range(20)]:
            q = self.cfg.q.copy()
            q[:7] = self.base_q7()
            q[self.ik_arm] = np.clip(sd, lo + .05, hi - .05)
            cfg = mink.Configuration(ik, q=q)
            post = mink.PostureTask(ik, cost=1e-3)
            post.set_target(q)
            for _ in range(300):
                v = mink.solve_ik(cfg, [task, post], 1., solver='daqp', damping=1e-4, limits=[mink.ConfigurationLimit(ik, gain=.5)],
                                  constraints=[self.rail_freeze])
                cfg.integrate_inplace(v, 1.)
            if np.linalg.norm(task.compute_error(cfg)[:3]) > 2e-3:
                continue
            qa = cfg.q[self.ik_arm]
            marg = float(np.min(np.minimum(qa - lo, hi - qa)))
            if best is None or marg > best[0]:
                best = (marg, qa.copy())
            if marg > .3:
                break
        if best is None:
            raise PL.StepFail('park pose unreachable from this base pose')
        return best[1]

    # ================================================================== perception (async via the Perception node)
    def perceive(self, tag, mode):
        self.substep(f'waiting for {"Codex" if self.codex else "DEBUG truth perception"} ({tag}); arm parked')
        self.perc_result = None
        self.send('Perception', 'PerceiveRequest', dict(tag=tag, mode=mode), info=dict(tag=tag, mode=mode))
        t0 = self.data.time
        while self.perc_result is None:
            self.tick()
            if self.data.time - t0 > 400:
                self.fail('perception timeout')
        r = self.perc_result
        vlm = r['vlm']
        self.codex_s.append(vlm.get('seconds'))
        cam = self.sys.perception.cam
        if vlm.get('debug_truth'):
            o = self.data.xpos[self.pid].copy() + np.r_[self.rng.normal(0, .001, 2), 0]
            est = dict(origin=o, P=np.eye(3), radius_fit=S.R_OUT, mode=mode)
            vlm.update(bbox=[0, 0, 0, 0], center=[0, 0])
        else:
            if not vlm.get('found'):
                self.fail(f'Codex did not find the plate ({vlm.get("error", "")})', codex_s=vlm.get('seconds'))
            est = PC.estimate_flat(cam, self.data, r['depth'], vlm)
            est['mode'] = mode
        true_c = self.data.site('plate_center').xpos.copy()
        if self.believed_base is not None:            # emulate the per-dock camera->robot re-calibration (spec)
            est = self._to_believed_frame(est)
            true_c = self._to_believed_frame(dict(origin=true_c, P=np.eye(3)))['origin']   # score in the same frame
        est_c = est['origin'] + est['P'][:, 2] * S.MID_Z
        err = est_c - true_c
        ntrue = self.data.xmat[self.pid].reshape(3, 3)[:, 2]
        res = dict(codex_s=vlm.get('seconds'), debug_truth=bool(vlm.get('debug_truth')), radius_fit_mm=round(est['radius_fit'] * 1000, 1),
                   centre_err_mm=round(float(np.linalg.norm(err)) * 1000, 2),
                   centre_err_xy_mm=[round(float(v) * 1000, 2) for v in err[:2]],
                   normal_err_deg=round(math.degrees(math.acos(min(1., abs(float(est['P'][:, 2] @ ntrue))))), 2))
        if not vlm.get('debug_truth'):
            res.update(codex_bbox=[round(v) for v in vlm['bbox']], codex_center=vlm['center'],
                       codex_center_px_err=round(float(np.linalg.norm(np.array(vlm['center']) - cam.project(self.data, true_c))), 1))
            ov = PC.overlay(r['rgb'], vlm, cam, self.data, est)
            Image.fromarray(ov).save(self.outdir / f'{self.run_tag}_{tag}_overlay.png')
            txt = (f'Wall RGB-D camera -> Codex ({vlm["seconds"]:.1f} s real latency; robot waited, arm parked): yellow = Codex box\n'
                   f'Green = depth fit: circle r = {est["radius_fit"]*1000:.1f} mm (true 101.5 mm)\n'
                   f'Scoring only: centre error vs ground truth {res["centre_err_mm"]:.1f} mm')
            self.sys.main_queue.append(lambda: self.rec.card(self.data, ov, txt, 3.))
        self.substep('')
        return est, res

    def _to_believed_frame(self, est):
        """Perception expressed in the robot's (re-calibrated) frame while the robot believes base = believed_base:
        world point p -> T_believed * T_true^-1 * p."""
        p = self.data.sensor('base_pos').data.copy()
        q = self.data.sensor('base_quat').data
        th_t = 2 * math.atan2(q[3], q[0])
        bx, by, th_b = self.believed_base
        dth = th_b - th_t
        Rz = np.array([[math.cos(dth), -math.sin(dth), 0], [math.sin(dth), math.cos(dth), 0], [0, 0, 1.]])
        pt = np.array([p[0], p[1], 0.])
        pb = np.array([bx, by, 0.])
        est = dict(est)
        est['origin'] = pb + Rz @ (est['origin'] - pt)
        est['P'] = Rz @ est['P']
        return est

    # ================================================================== step wrapper: keyframes rendered by the main thread
    def run_step(self, name, fn):
        kd = self.keyframes_dir
        self.keyframes_dir = None
        self.step_stats[name] = dict(t0=self.data.time)
        self.sys.step_name = name
        try:
            return super().run_step(name, fn)
        finally:
            self.keyframes_dir = kd
            if kd is not None:
                path = kd / f'{len(self.steps):02d}_{name}.png'
                self.sys.main_queue.append(lambda: self.rec.still(self.data, path))
            self.step_stats[name]['t1'] = self.data.time

    # ================================================================== base motion primitives
    def base_cmd(self, v, w):
        if self.k % 2 == 0:                                   # twist at 50 Hz (spec)
            self.send('BaseMCU', 'BaseCmd', dict(v=float(v), w=float(w)))

    def brake(self, engage):
        self.send('BaseMCU', 'Brake', dict(engage=bool(engage)), info=dict(engage=bool(engage)))
        t0 = self.data.time
        while self.base_state is None or self.base_state['brake'] != engage or self.data.time - t0 < .05:
            self.tick()
            if self.data.time - t0 > 1.:
                break

    def stop_base(self, settle=.4):
        t0 = self.data.time
        while self.data.time - t0 < settle:
            self._wait_unpaused()
            self.base_cmd(0., 0.)
            self.tick()

    def rotate_to(self, th_goal, tol=math.radians(.4)):
        self.phase = f'rotate to {math.degrees(th_goal):.0f} deg'
        if self.base_state is not None and self.base_state['brake_level'] > 0:
            self.brake(False)
        wmax = self.sys.bs['max_yaw_rate_rad_s']
        t0 = self.data.time
        while True:
            self._wait_unpaused()
            e = wrap(th_goal - self.base_est[2])
            w = float(np.clip(1.8 * e, -wmax, wmax))
            if abs(w) < .04 and abs(e) > tol:
                w = math.copysign(.04, e)
            if abs(e) < tol:
                break
            self.base_cmd(0., w)
            self.tick()
            if self.data.time - t0 > 20:
                raise PL.StepFail('rotate timeout')
        self.stop_base(.3)

    def drive_to(self, goal, direction=1, vmax=None, stop=None, name='drive', k1=8., k2=2.5, min_v=.01):
        """Follow the straight line from the current estimate to `goal` (x, y), forward (+1) or reverse (-1), with a
        trapezoidal speed profile (accel limit applied again in the BaseMCU) and lateral / heading feedback."""
        self.phase = name
        if self.base_state is not None and self.base_state['brake_level'] > 0:
            self.brake(False)
        vmax = vmax or self.sys.bs['max_speed_m_s']
        amax = self.sys.bs['max_accel_m_s2']
        p0 = self.base_est[:2].copy()
        goal = np.asarray(goal, float)
        dvec = goal - p0
        L = float(np.linalg.norm(dvec))
        if L < 1e-3:
            return True
        dh = dvec / L
        nh = np.array([-dh[1], dh[0]])
        psi = math.atan2(dh[1], dh[0])
        v = 0.
        t0 = self.data.time
        while True:
            self._wait_unpaused()
            p = self.base_est[:2]
            rem = float((goal - p) @ dh)
            if rem <= .001:
                break
            if stop is not None and stop():
                self.stop_base(.2)
                return 'stopped'
            el = float((p - p0) @ nh)
            phi = self.base_est[2] + (0 if direction > 0 else math.pi)
            eph = wrap(phi - psi)
            v = min(vmax, math.sqrt(2 * amax * max(rem, 0.)), v + amax * PLAN_DT)
            v = max(v, min_v)
            w = -k1 * v * el - k2 * math.sin(eph)
            w = w if direction > 0 else w          # motion-direction heading obeys the same law
            self.base_cmd(direction * v, float(np.clip(w, -.8, .8)))
            self.tick()
            if self.data.time - t0 > 60:
                raise PL.StepFail(f'{name}: timeout')
        self.stop_base(.4)
        return True

    # ================================================================== jobs
    def job_localize(self, a=None):
        """Wait for lidar fixes (start-pose uncertainty +-5 cm, +-5 deg is resolved by the first fixes)."""
        self.phase = 'localize'
        n0 = self.fix_count['used']
        t0 = self.data.time
        while self.fix_count['used'] - n0 < 12:
            self.tick()
            if self.data.time - t0 > 5:
                raise PL.StepFail('no lidar fix')
        return dict(base_est=[round(v, 4) for v in self.base_est], base_true=self._true_pose())

    def _true_pose(self):
        p = self.data.sensor('base_pos').data
        q = self.data.sensor('base_quat').data
        return [round(float(p[0]), 4), round(float(p[1]), 4), round(math.degrees(2 * math.atan2(q[3], q[0])), 3)]

    def job_undock_charger(self, a=None):
        self.docked = False
        self.brake(False)
        th = self.base_est[2]
        g = self.base_est[:2] + .30 * np.array([math.cos(th), math.sin(th)])
        self.drive_to(g, +1, vmax=.2, name='undock from charger')
        return dict(base_est=[round(v, 4) for v in self.base_est])

    def predock(self, dx=0., dy=0., dyaw=0.):
        x, y, yaw = SM.dock_base_pose(dx, dy, dyaw)
        th = math.radians(yaw)
        return np.array([x, y]) + .55 * np.array([math.cos(th), math.sin(th)]), th, np.array([x, y])

    def job_nav_predock(self, a):
        pre, th, _ = self.predock(*a.get('offset', (0, 0, 0)))
        d_ = pre - self.base_est[:2]
        self.rotate_to(math.atan2(d_[1], d_[0]))
        self.drive_to(pre, +1, name='drive to pre-dock')
        self.rotate_to(th)
        self.stop_base(.6)
        return dict(base_est=[round(v, 4) for v in self.base_est], base_true=self._true_pose())

    def job_dock(self, a):
        """Reverse along the dock line (lidar-corrected) at 0.05 m/s into the V-blocks until the dock-plate force
        reaches the preload (spec 100 N), then brake. Without V-blocks (sweeps): stop at the offset target pose."""
        off = a.get('offset', (0, 0, 0))
        pre, th, dock = self.predock(*off)
        F_target = self.sys.bs.get('dock_preload_N', 100.)
        vapp = self.sys.bs.get('dock_approach_speed_m_s', .05)
        hold = dict(n=0, peak=0.)

        def stop():
            F = self.base_state['dock_F'] if self.base_state else 0.
            hold['peak'] = max(hold['peak'], F)
            hold['n'] = hold['n'] + 1 if F >= F_target else 0
            return hold['n'] >= 2
        # fast part to 6 cm before contact, then the slow approach the rest of the way (+3 cm overtravel allowed)
        self.drive_to(dock + .06 * np.array([math.cos(th), math.sin(th)]), -1, vmax=.15, name='dock approach')
        r = self.drive_to(dock - .03 * np.array([math.cos(th), math.sin(th)]), -1, vmax=vapp, min_v=vapp,
                          stop=stop if a.get('vblocks', True) else None, name='dock: reverse into V-blocks', k1=4.)
        self.brake(True)
        if a.get('vblocks', True) and r != 'stopped':
            raise PL.StepFail(f'dock: preload {F_target:.0f} N not reached (peak {hold["peak"]:.0f} N)')
        self.docked = True
        self.dock_fixes.clear()
        t0 = self.data.time
        while self.data.time - t0 < 3.0:                      # 3 s of lidar fixes while braked
            self.tick()
        est = self.base_pose_arm()
        tru = self._true_pose()
        ax, ay, ayaw = SM.arm_base_pose(tru[:2], tru[2])
        ex, ey, eyaw = SM.arm_base_pose(est[:2], math.degrees(est[2]))
        return dict(dock_force_peak_N=round(hold['peak'], 1), dock_force_after_brake_N=round(self.base_state['dock_F'], 1),
                    arm_base_true=[round(ax, 4), round(ay, 4), round(ayaw, 3)],
                    dock_err_mm_deg=[round((ax - SM.DOCK_ARM[0]) * 1000, 2), round((ay - SM.DOCK_ARM[1]) * 1000, 2),
                                     round(wrap(math.radians(ayaw - SM.DOCK_ARM_YAW)) * 180 / math.pi, 3)],
                    base_est_err_mm_deg=[round((ex - ax) * 1000, 2), round((ey - ay) * 1000, 2),
                                         round(wrap(math.radians(eyaw - ayaw)) * 180 / math.pi, 3)])

    def job_localize_docked(self, a=None):
        """Sweeps that start already at the (offset) dock pose: brakes on, 3 s of lidar fixes."""
        self.docked = True
        self.dock_fixes.clear()
        t0 = self.data.time
        while self.data.time - t0 < 3.0:
            self.tick()
        est = self.base_pose_arm()
        tru = self._true_pose()
        ax, ay, ayaw = SM.arm_base_pose(tru[:2], tru[2])
        ex, ey, eyaw = SM.arm_base_pose(est[:2], math.degrees(est[2]))
        return dict(arm_base_true=[round(ax, 4), round(ay, 4), round(ayaw, 3)],
                    base_est_err_mm_deg=[round((ex - ax) * 1000, 2), round((ey - ay) * 1000, 2), round(eyaw - ayaw, 3)])

    def job_unstow(self, a=None):
        self.update_ik_base()
        self.q_park = self.solve_home()
        qg = self.cfg.q.copy()
        qg[self.ik_arm] = self.q_park
        self.ref['cmd'] = self.tcp()
        self.jmove('unstow', qg, max(2., 1.875 * float(np.max(np.abs(self.q_park - self.cfg.q[self.ik_arm]))) / .8), PL.OPEN,
                   p1=np.array(self.opt.park), R1=hand_frame([0, 0, -1.], [0, 1., 0]))
        self.hold('unstow', .3)
        return {}

    def job_stow(self, a=None):
        self.rec.set_view(*PL.VIEWS['overview'])
        if self.q_park is not None and not np.allclose(self.q_park, SM.Q_STOW):
            self.ref['cmd'] = self.ref.get('cmd', self.tcp())
            self.transfer('to park', np.array(self.opt.park), hand_frame([0, 0, -1.], [0, 1., 0]), PL.OPEN)
        qg = self.cfg.q.copy()
        qg[self.ik_arm] = SM.Q_STOW
        self.jmove('stow', qg, max(2., 1.875 * float(np.max(np.abs(SM.Q_STOW - self.cfg.q[self.ik_arm]))) / .8), PL.OPEN)
        self._hold_q(.4)
        self.q_park = SM.Q_STOW.copy()
        return {}

    def _hold_q(self, dur):
        t0 = self.data.time
        while self.data.time - t0 < dur:
            self._wait_unpaused()
            self._send_arm(self.q_sent, self.g_sent)
            self.tick()

    def job_undock_cabinet(self, a=None):
        if self.base_pose_source == 'lidar' and self.believed_base is None:
            self.base_est = self.base_pose_arm()
        self.docked = False
        self.brake(False)
        th = self.base_est[2]
        g = self.base_est[:2] + .50 * np.array([math.cos(th), math.sin(th)])
        self.drive_to(g, +1, vmax=.2, name='undock: forward out of the V-blocks')
        return {}

    def job_nav_charger(self, a=None):
        cx, cy, cth = SM.START_NOMINAL
        goal = np.array([cx, cy])
        d_ = goal - self.base_est[:2]
        mv = math.atan2(d_[1], d_[0])
        self.rotate_to(wrap(mv + math.pi))                    # reverse toward the charger (contacts on the rear)
        self.drive_to(goal, -1, name='drive back to the charger')
        self.rotate_to(math.radians(cth))
        return dict(base_est=[round(v, 4) for v in self.base_est], base_true=self._true_pose())

    def job_dock_charger(self, a=None):
        hold = dict(n=0, peak=0.)

        def stop():
            F = self.base_state['charge_F'] if self.base_state else 0.
            hold['peak'] = max(hold['peak'], F)
            hold['n'] = hold['n'] + 1 if F >= 15. else 0
            return hold['n'] >= 5
        self.docked = False
        if self.base_state is not None and self.base_state['brake']:
            self.brake(False)
        th = self.base_est[2]
        goal = np.array(SM.CHARGER_POSE[:2]) - .03 * np.array([math.cos(th), math.sin(th)])
        r = self.drive_to(goal, -1, vmax=.03, min_v=.03, stop=stop, name='dock onto the charging contacts', k1=4.)
        self.brake(True)
        if r != 'stopped':
            raise PL.StepFail(f'charging contacts not reached (peak {hold["peak"]:.1f} N)')
        t0 = self.data.time
        while self.data.time - t0 < 1.:
            self.tick()
        tru = self._true_pose()
        return dict(charge_force_N=round(self.base_state['charge_F'], 1), charger_pose_err_mm_deg=[
            round((tru[0] - SM.CHARGER_POSE[0]) * 1000, 1), round((tru[1] - SM.CHARGER_POSE[1]) * 1000, 1),
            round(tru[2] - SM.CHARGER_POSE[2], 2)])

    def job_press(self, a):
        """One panel touch: stylus (conductive) or plain finger pad; approach until the wrist F/T reads 3 N, hold 0.5 s
        (spec), retract. Success of the touch is judged by the TaskSupervisor from the LED readout, not here."""
        b = a['button']
        tool = a.get('tool', self.press_tool)
        pitch, yaw, close, flip = self.opt.press
        R = plan.press_frame(pitch, yaw, close, flip)
        if getattr(self, 'press_R', None) is None:
            f, qbt_, _ = self.plan_path([(plan.button_pos('quick') - .04 * R[:, 2], R), (plan.button_pos('quick'), R),
                                         (plan.button_pos('power') - .04 * R[:, 2], R), (plan.button_pos('power'), R)], flips=(1, -1))
            if qbt_ is None:
                self.fail('no joint path to the buttons from this base pose')
            self.press_R = R if f > 0 else R @ self.FLIP
            self.press_q0 = qbt_[0]
        R = self.press_R
        self.rec.set_view(*PL.VIEWS['buttons'])
        pad_body = self.model.body(f'dw/btn_{b}_body').id
        self.allowed = {pad_body}
        face = plan.button_pos(b)
        # offset the TCP target so the chosen contact point (stylus tip / right finger pad) lands on the pad centre
        st_closed = dict(self.arm_state, f=np.zeros(2))          # offset for the CLOSED fingers used to press
        self._fk(st_closed)
        Rt = self.fk.site('tcp').xmat.reshape(3, 3)
        if tool == 'stylus':
            off_local = Rt.T @ (self.fk.site('stylus').xpos - self.fk.site('tcp').xpos)
            off_local[2] = 0.
        else:
            off_local = np.array([0., -.006, 0.])          # right finger pad centre (no stylus on that finger)
        shift = R @ off_local
        tgt = face - shift
        self.substep(f'press {b.upper()} with the {"conductive stylus" if tool == "stylus" else "plain finger pad"}')
        if self.phase not in ('btn_quick_off', 'btn_power_off'):
            self.transfer(f'btn_{b}_pre', tgt - .04 * R[:, 2], R, PL.TOUCH, speed=.25, q_hint=self.press_q0)
        else:
            self.goto(f'btn_{b}_pre', tgt - .04 * R[:, 2], R, PL.TOUCH, speed=.25)
        self.hold(f'btn_{b}_pre', .8)
        f0 = self.hand_force_world().copy()
        self.sys.panel.raw[b] = dict(cond_peak=0., noncond_peak=0., noncond_dwell=0.)

        def f_axis():
            return float((self.hand_force_world() - f0) @ R[:, 2])

        dbg = []
        flt = dict(f=0., n=0)

        def stop():          # contact = wrist F/T rise along the approach axis above 3 N
            fa, za = f_axis(), float((self.tcp()[0] - tgt) @ R[:, 2])
            flt['f'] = fa
            dbg.append((round(self.data.time, 3), round(fa, 2), round(flt['f'], 2), round(za * 1000, 2)))
            if len(dbg) > 600:
                dbg.pop(0)
            return fa > 3.0
        self.move(f'btn_{b}_near', 1.5, tgt - .010 * R[:, 2], R, PL.TOUCH)
        hit = self.move(f'btn_{b}_press', 6.0, tgt + .006 * R[:, 2], R, PL.TOUCH, stop=stop)
        # press-and-hold 0.5 s (spec) with wrist-F/T admittance on the approach axis: regulate ~3 N
        p_hold = self.ref['last_target'][0].copy()
        A = dict(off=0.)

        def hook(p):
            flt['f'] += .5 * (f_axis() - flt['f'])
            A['off'] += float(np.clip(.00012 * (3.0 - flt['f']), -.00006, .00006))
            return p_hold + A['off'] * R[:, 2]
        self.ref['cmd'] = (p_hold, R)
        self.move(f'btn_{b}_hold', .6, p_hold, R, PL.TOUCH, hook=hook)
        self.ref['cmd'] = (self.ref['last_target'][0].copy(), R)
        self.move(f'btn_{b}_off', 1.0, tgt - .05 * R[:, 2], R, PL.TOUCH)
        self.phase = f'btn_{b}_off'
        self.allowed = set()
        raw = self.sys.panel.raw[b]
        self.press_dbg = dbg
        return dict(button=b, tool=tool, ft_stop=bool(hit), f_axis_at_stop=dbg[-1] if dbg else None,  conductive_peak_N=round(raw['cond_peak'], 2),
                    nonconductive_peak_N=round(raw['noncond_peak'], 2), nonconductive_dwell_s=round(raw['noncond_dwell'], 3))

    def job_arm_step(self, a):
        name = a['step']
        fns = dict(perceive_plate=self.step_perceive, open_door=self.step_open_door, load_pod=self.step_load_pod,
                   pull_basket=self.step_pull_basket, pick_plate=self.step_pick_plate, insert_plate=self.step_insert,
                   push_basket=self.step_push_basket, close_door=self.step_close_door)
        if name == 'perceive_plate':
            self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        return self.run_step(name, fns[name])

    def job_safe_stow(self, a=None):
        """Abort path: open the hand, back away from the machine to the park pose, stow."""
        self.ref['cmd'] = self.tcp()
        p, R = self.ref['cmd']
        self.allowed = set(range(self.model.nbody))          # anything may be touched while backing out
        self.move('abort_release', .8, p, R, PL.OPEN)
        self.move('abort_up', 1.5, p + [0, 0, .10], R, PL.OPEN)
        self.move('abort_out', 1.5, p + [.15, 0, .10], R, PL.OPEN)
        self.allowed = set()
        return self.job_stow()

    def job_drive_to(self, a):
        """base.drive_to(pose=(x, y, heading_deg), direction=+1|-1): rotate toward the goal, follow the line, rotate."""
        x, y, hd = a['pose']
        direction = a.get('direction', 1)
        d_ = np.array([x, y]) - self.base_est[:2]
        if np.linalg.norm(d_) > .005:
            mv = math.atan2(d_[1], d_[0])
            self.rotate_to(mv if direction > 0 else wrap(mv + math.pi))
            self.drive_to((x, y), direction, vmax=a.get('vmax'), name='base.drive_to')
        if hd is not None:
            self.rotate_to(math.radians(hd))
        return dict(base_est=[round(v, 4) for v in self.base_est], base_true=self._true_pose())

    def job_brake_cmd(self, a):
        self.brake(bool(a.get('on', True)))
        return dict(brake=bool(self.base_state and self.base_state['brake']))

    def job_get_state(self, a=None):
        st = self.base_state or {}
        out = dict(base_est=[round(v, 4) for v in self.base_est], base_pose_for_arm=[round(float(v), 4) for v in self.base_pose_arm()],
                   base_true_scoring=self._true_pose(), docked=self.docked, brake=st.get('brake'), dock_force_N=st.get('dock_F'),
                   charge_force_N=st.get('charge_F'), estop=st.get('estop'))
        if self.arm_state is not None:
            out.update(arm_q=[round(float(v), 4) for v in self.arm_state['q']], gripper_m=[round(float(v), 4) for v in self.arm_state['f']],
                       arm_mode=self.arm_state['mode'], tcp=[round(float(v), 4) for v in self.tcp()[0]])
        return out

    def job_get_soc(self, a=None):
        st = self.base_state or {}
        return dict(soc=st.get('soc'), V=st.get('V'), I=st.get('I'), charging=st.get('charging'))

    def job_wait(self, a):
        t0 = self.data.time
        while self.data.time - t0 < a.get('s', 1.):
            self.tick()
        return {}

    # ================================================================== main loop (planner thread)
    def main(self):
        self.tick()
        while True:
            while not self.cmds:
                self.tick()
            c = self.cmds.popleft()
            if c['job'] == 'shutdown':
                return
            self.job = c['job']
            fn = getattr(self, f'job_{c["job"]}')
            t0 = self.data.time
            try:
                out = fn(c.get('args', {})) or {}
                ok, reason = out.pop('_ok', True), out.pop('_reason', '')
            except PL.StepFail as e:
                ok, reason, out = False, str(e), getattr(e, 'metrics', {})
            except Exception as e:  # noqa: BLE001  (a crash fails that job; reported)
                import traceback
                ok, reason, out = False, f'exception {e!r}', dict(trace=traceback.format_exc()[-1500:])
            self.job = ''
            self.send('TaskSupervisor', 'JobDone', dict(id=c['id'], skill=c.get('skill'), job=c['job'], ok=bool(ok), reason=reason,
                                                        metrics=out, t0=t0, t1=self.data.time),
                      info=dict(id=c['id'], skill=c.get('skill'), job=c['job'], ok=bool(ok), reason=reason[:120]))
