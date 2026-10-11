"""Mobile simulation for GPT control: ../mobile_pipeline (wheeled FR3-limited Franka, spec v1.0, controller nodes on a
message bus) driven by a GPT / hybrid mission client instead of the hard-wired TaskSupervisor sequence.

Nothing in ../mobile_pipeline is modified; its modules are imported and a few classes are subclassed or swapped in this
process only (planner.MotionPlanner, system.DishwasherPanel / MobileRecorder, scene_mobile._add_kitchen).

What this file adds
  * GPTPlanner: the MotionPlanner with *honest* skill jobs (verdicts from robot-side signals: encoders, finger width,
    wrist F/T, lidar fix counts, appliance sensors, LED readout). Ground truth goes only into `_scoring` fields.
    Ported from ../gpt_control/sim_robot.py (open_door with hold_at_stop_s + half-open re-grasp, load_pod finger-width
    verdict, pull_basket with a wrist-force guard, honest pick / insert verdicts).
  * verify jobs: SIMULATED SENSING (truth + noise) for the two outcomes no robot sensor sees (pod in groove, plate seated).
  * fault mechanisms inside the mj_step loop: weak door stay, panel asleep (first press ignored), dock y-bias, person
    (visual mocap figure + MODELLED safety scanner), e-stop (the mobile Operator node).
  * GPTSupervisor node (bus name 'TaskSupervisor', 10 Hz): runs the mission client (Full-GPT or Hybrid) as a lock-step
    coroutine, dispatches skill jobs, tracks e-stop / BMS / panel readout. While GPT thinks, the simulation keeps running
    (robot idle, loads drawing power) throttled to wall time, so sim time and energy include the real GPT latency.
  * HUD with the GPT panel (reasoning_brief + action) on top of the mobile HUD (SoC, base state, dock force, panel).
"""
from __future__ import annotations

import math
import sys
import textwrap
import time
from collections import deque
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)
from types import SimpleNamespace

sys.dont_write_bytecode = True
MP = _ROOT / 'mobile_pipeline'
if str(MP) not in sys.path:
    sys.path.insert(0, str(MP))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

import dishwasher as dw  # noqa: E402
import nodes as ND  # noqa: E402
import perception as PC  # noqa: E402
import pipeline as PL  # noqa: E402
import plan  # noqa: E402
import planner as PLN  # noqa: E402
import run_mobile as RM  # noqa: E402
import scene as S  # noqa: E402
import scene_mobile as SM  # noqa: E402
import system as SYS  # noqa: E402
from util import hand_frame, rot_z  # noqa: E402

ROOT = Path(__file__).resolve().parent
DEG = math.pi / 180
PLATE_MASS = sum(S.PLATE_MASS.values())          # 0.45 kg -> 4.4 N on the wrist F/T
RD = hand_frame([0, 0, -1.], [0, 1., 0])          # hand down (park attitude)
StepFail = PL.StepFail

# ---------------------------------------------------------------- fault parameters
LATCH_ON_DEG, LATCH_K, LATCH_DWELL_S = 45., 3.0, 3.0          # weak door stay (same mechanism as ../gpt_control)
DOCK_Y_BIAS = .040                                             # strip-fix y bias during the first approach (m)
PERSON_NEAR_R = 1.5                                            # modelled scanner: person flagged within 1.5 m of the base
PROTECT_R = .95                                                # protective field: base centre -> person (0.33 half length + 0.6)
V_PERSON = .25                                                 # m/s cap (arm TCP and base) while a person is flagged
SCEN = {
    'nominal': {},
    'low_battery': dict(soc0=.22),
    'dock_y': dict(dock_y_bias=DOCK_Y_BIAS, vblocks=False),
    'person_path': dict(person=dict(kind='path', after='base.undock_charger', pos=(0.83, 0.55), dur_s=45.)),
    'person_arm': dict(person=dict(kind='arm', after='skills.open_door', pos=(1.02, 0.92), dur_s=30.)),
    'estop': dict(estop=dict(estop_phase='carry_across', delay=.5, hold=3., ack_after=1.5, action='resume')),
    'door_latch': dict(door_latch=True),
    'button_miss': dict(button_miss=True),
}


# ====================================================================== scene: hidden person figure (visual only)
_orig_kitchen = SM._add_kitchen


def _kitchen_with_person(spec, cfg, mu):
    _orig_kitchen(spec, cfg, mu)
    b = spec.worldbody.add_body(name='person', mocap=True, pos=[4., 4., 0.])
    CAP, SPH = mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_SPHERE
    kw = dict(contype=0, conaffinity=0, group=5)
    b.add_geom(name='person_legs', type=CAP, size=[.09, .40, 0], pos=[0, 0, .45], rgba=[.20, .25, .45, 1], **kw)
    b.add_geom(name='person_torso', type=CAP, size=[.16, .27, 0], pos=[0, 0, 1.17], rgba=[.75, .30, .25, 1], **kw)
    b.add_geom(name='person_head', type=SPH, size=[.11, 0, 0], pos=[0, 0, 1.68], rgba=[.93, .78, .66, 1], **kw)


SM._add_kitchen = _kitchen_with_person


# ====================================================================== appliance MCU with the 'asleep' fault
class SimPanel(ND.DishwasherPanel):
    """button_miss fault: the panel is asleep; the first registered touch only wakes it (no state change, no LED)."""

    WAKE_S = 1.5                                       # wake-up time; touches during it are ignored (modelled firmware)

    def __init__(self, sysm, **kw):
        super().__init__(sysm, **kw)
        self.asleep = bool(sysm.faults.get('button_miss'))
        self.wake_until = None

    def _press(self, t, b, door_closed):
        if self.asleep:
            self.asleep = False
            self.wake_until = t + self.WAKE_S
            self.beeps += 1
            self._event(t, f'woke_up_touch_{b}_ignored')
            self.sys.fault_events.append((round(t, 2), f'panel was asleep: touch of {b.upper()} only woke it'))
            return
        if self.wake_until is not None and t < self.wake_until:
            self._event(t, f'touch_{b}_ignored_while_waking')
            return
        super()._press(t, b, door_closed)


# ====================================================================== motion planner with honest skills
_MotionPlanner = PLN.MotionPlanner


class GPTPlanner(_MotionPlanner):
    def __init__(self, sysm, opt, **kw):
        super().__init__(sysm, opt, **kw)
        m = self.model
        self.F = sysm.faults
        self.sens_rng = np.random.default_rng(kw.get('seed', 0) + 7)
        self.base_cap = None                           # base speed cap (m/s), set by the API speed limit
        self.ft0 = None                                # calibrated hand weight (world N), empty hand
        self.percept = None
        self.T_hp = None
        self.basket_est = None
        self.door_dof = m.joint('dw/door_hinge').dofadr[0]
        self.latch = dict(active=bool(self.F.get('door_latch')), latched=False, dwell=0., peak=0.)
        self.ybias = dict(active=bool(self.F.get('dock_y_bias')), val=float(self.F.get('dock_y_bias', 0.)))
        self.y_fix_t = deque(maxlen=400)               # sim times of used fixes that carried a strip y
        self.fix_t = deque(maxlen=400)
        self.blocked_info = None
        self.n_perc = 0

    # ------------------------------------------------------------------ robot-side sensing
    def door_deg(self):
        """SIMULATED appliance sensor: door angle = truth + 0.3 deg noise (real: wall-node vision / appliance API)."""
        return float(math.degrees(self.data.qpos[self.door_j]) + self.sens_rng.normal(0, .3))

    def basket_m(self):
        return float(np.clip(self.data.qpos[self.basket_j] + self.sens_rng.normal(0, .002), 0, dw.BASKET_TRAVEL))

    def grip_width_mm(self):
        f = self.arm_state['f'] if self.arm_state is not None else self.data.qpos[self.fq]
        return max(0., float(1000 * np.sum(f)))                    # finger encoders

    def wrist_avg(self, dur=.3, name='ft_avg'):
        """Hold still for `dur` and average the wrist F/T over every 500 Hz sample (the sprung base adds +-2 N)."""
        acc = []
        prev = self.ref.get('cb')

        def cb():
            acc.append(self.hand_force_world().copy())
            if prev is not None:
                prev()
        self.ref['cb'] = cb
        self.hold(name, dur)
        if prev is None:
            self.ref.pop('cb', None)
        else:
            self.ref['cb'] = prev
        return np.mean(acc, 0) if acc else self.hand_force_world()

    def load(self, dur=.3):
        f = self.wrist_avg(dur)
        return f - (self.ft0 if self.ft0 is not None else f)

    def arm_pose_name(self):
        if self.arm_state is None:
            return 'unknown'
        q = np.asarray(self.arm_state['q'])
        if float(np.max(np.abs(q - SM.Q_STOW))) < .06:
            return 'stowed'
        if float(np.linalg.norm(self.tcp()[0] - np.array(self.opt.park))) < .03:
            return 'park'
        return 'working'

    def holding(self, load_z=None):
        w = self.grip_width_mm()
        g = float(self.data.ctrl[self.act[7]])
        target = max(0., g / 255 * .07 - .03) * 2000
        if w - target < 3.0:
            return f'nothing (fingers free at {w:.0f} mm)'
        if load_z is not None and 3. < w < 12. and load_z > 2.5:
            return f'plate-like object (~{load_z / 9.81 * 1000:.0f} g)'
        if 20. < w < 36.:
            return f'small object, width {w:.0f} mm'
        return f'object or bar, width {w:.0f} mm'

    # ------------------------------------------------------------------ localization (with the dock y-bias fault)
    def _fix(self, f):
        if f.get('ok') and 'y' in f and self.ybias['active'] and self.job == 'bm' and self.ybias.get('in_approach'):
            f = dict(f, y=f['y'] + self.ybias['val'])          # MODELLED fault: strip-edge y misdetection
        n_used = self.fix_count['used']
        super()._fix(f)
        if self.fix_count['used'] > n_used:
            self.fix_t.append(self.data.time)
            if 'y' in f:
                self.y_fix_t.append(self.data.time)

    def _inbox(self):
        super()._inbox()
        if self.arm_state is not None and self.k % 2 == 0:        # 50 Hz EMA of the wrist force (state display only)
            f = self.hand_force_world()
            self.f_ema = f if getattr(self, 'f_ema', None) is None else self.f_ema + .1 * (f - self.f_ema)

    def wrist_load_ema(self):
        f = getattr(self, 'f_ema', None)
        if f is None or self.ft0 is None:
            return None
        return f - self.ft0

    def loc_quality(self, window=2.0):
        t = self.data.time
        nf = sum(1 for x in self.fix_t if t - x <= window)
        ny = sum(1 for x in self.y_fix_t if t - x <= window)
        return dict(lidar_fixes_last_2s=nf, strip_y_fixes_last_2s=ny,
                    y_source='reflective strip (lidar)' if ny > 0 else 'dead-reckoned (strip not visible)')

    # ------------------------------------------------------------------ fault mechanisms (after every mj_step)
    def _monitor(self):
        super()._monitor()
        L = self.latch
        if L['active'] and not L['latched']:
            d = self.data
            th = float(d.qpos[self.door_j])
            d.qfrc_applied[self.door_dof] = -LATCH_K * (th - LATCH_ON_DEG * DEG) if th > LATCH_ON_DEG * DEG else 0.
            L['dwell'] = L['dwell'] + self.model.opt.timestep if th >= 88 * DEG else 0.
            L['peak'] = max(L['peak'], th)
            if L['dwell'] >= LATCH_DWELL_S:
                L['latched'] = True
                d.qfrc_applied[self.door_dof] = 0.
                self.sys.fault_events.append((round(d.time, 2), 'door stay latched at the open stop'))

    # ------------------------------------------------------------------ base driving: speed cap + protective stop
    def drive_to(self, goal, direction=1, vmax=None, stop=None, name='drive', **kw):
        if self.base_cap:
            vmax = min(vmax or self.sys.bs['max_speed_m_s'], self.base_cap)
        sc = self.sys.scanner
        hit = dict(blocked=False)

        def stop2():
            if stop is not None and stop():
                return True
            if sc.blocked(direction):
                hit['blocked'] = True
                return True
            return False
        for _ in range(3):
            hit['blocked'] = False
            r = super().drive_to(goal, direction, vmax, stop2, name, **kw)
            if not hit['blocked']:
                return r
            # protective stop (MODELLED scanner): standing, wait up to 3 s for the field to clear, else fail
            self.phase = 'protective stop'
            t0 = self.data.time
            while sc.blocked(direction) and self.data.time - t0 < 3.:
                self._wait_unpaused()
                self.base_cmd(0., 0.)
                self.tick()
            if sc.blocked(direction):
                self.stop_base(.2)
                dist = sc.distance()
                self.fail(f'protective stop: the safety scanner field is occupied ({dist:.2f} m from the base centre, '
                          'in the driving direction); the base stopped and did not reach its goal', code='PATH_BLOCKED',
                          obstacle_distance_m=round(dist, 2), base_est=[round(v, 4) for v in self.base_est])
        return r

    # ------------------------------------------------------------------ jobs: base
    def job_bm(self, a):
        """Base motion job wrapper: run the mobile job, then engage the parking brakes (also on failure)."""
        inner = a['inner']
        self.ybias['in_approach'] = inner in ('nav_predock', 'dock', 'dock_pos')     # fault acts only on the first approach + dock
        try:
            out = getattr(self, f'job_{inner}')(a) or {}
        finally:
            if inner not in ('dock', 'dock_pos', 'dock_charger'):
                self.brake(True)
        if inner in ('dock', 'dock_pos'):
            if self.ybias['active']:
                self.ybias['active'] = False
                self.sys.fault_events.append((round(self.data.time, 2), 'dock y-bias fault cleared after the first dock'))
            q = self.loc_quality(2.5)
            out['_scoring'] = dict(dock_err_mm_deg=out.pop('dock_err_mm_deg', None), base_est_err_mm_deg=out.pop('base_est_err_mm_deg', None),
                                   arm_base_true=out.pop('arm_base_true', None))
            out.update(localization=q, base_pose_for_arm=[round(float(v), 4) for v in self.base_pose_arm()])
        return out

    def job_dock_pos(self, a):
        """Dock without V-blocks (dock_y scenario, as the mobile_pipeline dock sweep): reverse at 0.05 m/s along the dock line to
        the dock pose (lidar + odometry estimate, no mechanical stop), brake, average 3 s of lidar fixes."""
        pre, th, dock = self.predock(0, 0, 0)
        vapp = self.sys.bs.get('dock_approach_speed_m_s', .05)
        self.drive_to(dock + .06 * np.array([math.cos(th), math.sin(th)]), -1, vmax=.15, name='dock approach')
        self.drive_to(dock, -1, vmax=vapp, min_v=vapp, name='dock: reverse to the dock pose (no V-blocks)', k1=4.)
        self.brake(True)
        self.docked = True
        self.dock_fixes.clear()
        t0 = self.data.time
        while self.data.time - t0 < 3.0:
            self.tick()
        est = self.base_pose_arm()
        tru = self._true_pose()
        ax, ay, ayaw = SM.arm_base_pose(tru[:2], tru[2])
        ex, ey, eyaw = SM.arm_base_pose(est[:2], math.degrees(est[2]))
        return dict(dock_force_after_brake_N=round(self.base_state['dock_F'], 1), arm_base_true=[round(ax, 4), round(ay, 4), round(ayaw, 3)],
                    dock_err_mm_deg=[round((ax - SM.DOCK_ARM[0]) * 1000, 2), round((ay - SM.DOCK_ARM[1]) * 1000, 2),
                                     round(PLN.wrap(math.radians(ayaw - SM.DOCK_ARM_YAW)) * 180 / math.pi, 3)],
                    base_est_err_mm_deg=[round((ex - ax) * 1000, 2), round((ey - ay) * 1000, 2),
                                         round(PLN.wrap(math.radians(eyaw - ayaw)) * 180 / math.pi, 3)])

    def job_charge(self, a):
        tgt, tmax = float(a['target_soc']), float(a['max_minutes']) * 60.
        t0, s0 = self.data.time, self.sys.battery.soc
        self.phase = f'charging to {tgt:.2f}'
        while self.sys.battery.soc < tgt and self.data.time - t0 < tmax:
            self.tick()
        return dict(soc_start=round(s0, 4), soc=round(self.sys.battery.soc, 4), charged_s=round(self.data.time - t0, 1),
                    reached=bool(self.sys.battery.soc >= tgt))

    # ------------------------------------------------------------------ jobs: arm basics
    def job_g_unstow(self, a=None):
        self.job_unstow()
        self.ft0 = self.wrist_avg(.4, 'ft_calibration')          # hand weight with the hand empty (tare)
        return dict(arm_pose=self.arm_pose_name(), base_pose_for_arm=[round(float(v), 4) for v in self.base_pose_arm()])

    def job_g_stow(self, a=None):
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        self.job_stow()
        return dict(arm_pose=self.arm_pose_name())

    def job_g_locate(self, a=None):
        if self.arm_pose_name() not in ('stowed', 'park'):
            self.ref['cmd'] = self.ref.get('cmd', self.tcp())
            self.transfer('park', np.array(self.opt.park), RD, PL.OPEN)
            self.hold('park', .5)
        self.n_perc += 1
        try:
            est, res = self.perceive(f'locate{self.n_perc}', 'flat')
        except StepFail as e:
            self.fail(f'plate not found: {e}', code='NOT_FOUND')
        self.est = est
        c = est['origin'] + est['P'][:, 2] * S.MID_Z
        sane = .095 < est['radius_fit'] < .110
        self.percept = dict(est=est, t=float(self.data.time), valid=bool(sane))
        self.last_overlay = self.outdir / f'{self.run_tag}_locate{self.n_perc}_overlay.png'
        pub = dict(found=True, centre_xyz=[round(float(v), 4) for v in c], radius_fit_mm=res['radius_fit_mm'],
                   codex_s=res['codex_s'], codex_center_px=res.get('codex_center'))
        pub['_scoring'] = {k: v for k, v in res.items() if k in ('centre_err_mm', 'centre_err_xy_mm', 'normal_err_deg', 'codex_center_px_err')}
        if not sane:
            pub.update(_ok=False, code='PERCEPTION_SUSPECT', _reason=f'depth circle fit radius {est["radius_fit"]*1000:.0f} mm '
                       '(expected 95-110 mm): the detection is not trustworthy')
        return pub

    # ------------------------------------------------------------------ door
    def job_g_open_door(self, a):
        hold_at_stop_s = float(a.get('hold_at_stop_s', .5))
        self.rec.set_view(*PL.VIEWS['door'])
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        th0 = max(0., self.door_deg())
        if th0 > 86:
            return dict(door_deg=round(th0, 1), note='already open')
        self.allowed = {self.model.body('dw/door').id}
        self.door_flip = 1
        ths = np.radians(np.r_[th0, np.arange(math.ceil(th0 / 10 + 1e-6) * 10, 91, 10)])
        ths = np.unique(np.round(ths, 4))
        p0_, R0_ = self.door_frames(ths[0])
        poses = [(p0_ - .10 * R0_[:, 2], R0_), (p0_, R0_)] + [self.door_frames(t) for t in ths[1:]]
        self.door_flip, qs, wm = self.plan_path(poses, flips=(1, -1))
        if qs is None:
            self.fail(f'no joint path for the door arc from {th0:.0f} deg', code='UNREACHABLE')
        self.door_open_plan = (self.door_flip, qs) if th0 < 1 else getattr(self, 'door_open_plan', None)
        p0, R0 = self.door_frames(ths[0])
        g_open = PL.OPEN if th0 < 20 else self.opt.door_regrasp_open      # half-open door: narrow re-grasp
        self.substep(f'approach the pull bar (door at {th0:.0f} deg)')
        self.transfer('door_pre', p0 - .10 * R0[:, 2] + [0, 0, .02], R0, g_open, q_hint=qs[0])
        self.move('door_pre', 1.0, p0 - .10 * R0[:, 2], R0, g_open, qpath=[qs[0]])
        if th0 >= 20:
            th1 = math.radians(max(0., self.door_deg()))
            if abs(th1 - ths[0]) > math.radians(.5):
                ths[0] = th1
                p0, R0 = self.door_frames(th1)
                self.move('door_pre', .6, p0 - .10 * R0[:, 2], R0, g_open)
        self.move('door_in', 1.5, p0, R0, g_open, qpath=[qs[0], qs[1]])
        g = PL.squeeze_ctrl(2 * dw.HANDLE_R, 5.)
        self.move('door_grasp', 1.0, p0, R0, g)
        self.hold('door_grasp', .3)
        w = self.grip_width_mm()
        if w > 2000 * (dw.HANDLE_R + .004) or w < 1000 * dw.HANDLE_R:
            self.move('door_release', .6, p0, R0, PL.OPEN)
            self.move('door_back', 1.0, p0 - .10 * R0[:, 2], R0, PL.OPEN)
            self.allowed = set()
            self.fail(f'bar not grasped (finger width {w:.1f} mm)', code='GRASP_FAILED', finger_width_mm=round(w, 1))
        self.substep(f'follow the hinge arc {math.degrees(ths[0]):.0f} -> 90 deg')
        a0 = ths[0]

        def arc(s):
            return self.door_frames(a0 + s * (math.pi / 2 - a0))
        self.seg('door_arc', max(3., 8.0 * (math.pi / 2 - a0) / (math.pi / 2)), arc, g, qpath=qs[1:])
        self.ref['cmd'] = arc(1.)
        self.substep(f'hold at the open stop {hold_at_stop_s:.1f} s')
        self.hold('door_arc', max(.1, hold_at_stop_s))
        held = self.door_deg()
        p, R = self.ref['cmd']
        self.substep('release and back off')
        self.move('door_release', .8, p, R, PL.OPEN)
        self.move('door_back', 1.2, p - .10 * R[:, 2], R, PL.OPEN)
        self.goto('door_back', p - .10 * R[:, 2] + [.08, 0, .30], R, PL.OPEN, speed=.3)
        self.allowed = set()
        self.hold('door_settle', 1.0)
        ang = self.door_deg()
        out = dict(door_deg=round(ang, 1), door_deg_while_held=round(held, 1))
        if ang <= 85:
            out.update(_ok=False, code='DOOR_NOT_LATCHED' if held > 87 else 'DOOR_NOT_AT_STOP',
                       _reason=(f'door reached {held:.0f} deg while held but returned to {ang:.0f} deg after release'
                                if held > 87 else f'door only reached {held:.0f} deg'))
        return out

    def job_g_close_door(self, a=None):
        th = self.door_deg()
        if th < 1.:
            return dict(door_deg=round(th, 2), note='already closed')
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        r = self.step_close_door()            # inherited bar-grasp check counts finger contacts (tactile proxy)
        ang = self.door_deg()
        out = dict(door_deg=round(ang, 2))
        if abs(ang) >= 1.:
            out.update(_ok=False, code='DOOR_NOT_CLOSED', _reason=f'door at {ang:.1f} deg after the close')
        out['_scoring'] = dict(door_deg_true=r.get('door_deg'))
        return out

    # ------------------------------------------------------------------ pod
    def job_g_load_pod(self, a=None):
        m, d = self.model, self.data
        self.rec.set_view(*PL.VIEWS['pod'])
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        dish = np.array(self.opt.layout.dish)
        z = S.COUNTER_Z + .0003 + S.DISH_FLOOR_T + S.POD_SIZE[2] / 2
        Rpick = plan.pod_down_frame([1., 0, 0])
        Rplace = plan.pod_down_frame([0, 1., 0])
        self.allowed = {self.pod_id, self.dish_id, m.body('dw/door').id}
        grasp = np.array([dish[0], dish[1], z - .002])
        f, qa_, _ = self.plan_path([(grasp + [0, 0, .08], Rpick), (grasp, Rpick), (grasp + [0, 0, .08], Rpick)], flips=(1, -1))
        if qa_ is None:
            self.fail('no joint path to pick the pod', code='UNREACHABLE')
        Rpick = Rpick if f > 0 else Rpick @ self.FLIP
        gc, face = plan.groove_center(), plan.groove_face_z()
        place = np.array([gc[0], gc[1], face + .003 + S.POD_SIZE[2] / 2 + .002])
        f, qb_, _ = self.plan_path([(place + [0, 0, .08], Rplace), (place, Rplace)], flips=(1, -1))
        if qb_ is None:
            self.fail('no joint path to place the pod over the groove', code='UNREACHABLE')
        Rplace = Rplace if f > 0 else Rplace @ self.FLIP
        pod0 = d.xpos[self.pod_id].copy()
        self.substep('to the pod dish')
        self.transfer('pod_above', grasp + [0, 0, .08], Rpick, PL.grip_ctrl(.022), q_hint=qa_[0])
        self.move('pod_down', 1.5, grasp, Rpick, PL.grip_ctrl(.022), qpath=qa_[:2])
        self.substep('grasp pod (~5 N per finger)')
        g = PL.squeeze_ctrl(S.POD_SIZE[0], 5.)
        self.move('pod_grasp', 1.0, grasp, Rpick, g)
        self.move('pod_lift', 1.2, grasp + [0, 0, .08], Rpick, g, qpath=qa_[1:])
        w = self.grip_width_mm()
        if w < 18.:                     # fingers closed past any pod dimension: nothing in the hand
            self.move('pod_open', .6, grasp + [0, 0, .08], Rpick, PL.OPEN)
            self.allowed = set()
            self.fail(f'nothing grasped at the pick point (finger width {w:.1f} mm after closing)', code='POD_NOT_GRASPED',
                      finger_width_mm=round(w, 1), _scoring=dict(pod_moved_mm=round(float(np.linalg.norm(d.xpos[self.pod_id] - pod0)) * 1000, 1)))
        self.substep('carry over the open door, rotate 90 deg')
        self.transfer('pod_over_groove', place + [0, 0, .08], Rplace, g, speed=.25, q_hint=qb_[0])
        self.move('pod_lower', 1.5, place, Rplace, g, qpath=qb_)
        self.substep('release into the groove')
        self.move('pod_release', .8, place, Rplace, PL.grip_ctrl(.026))
        self.hold('pod_release', .4)
        self.move('pod_up', 1.0, place + [0, 0, .08], Rplace, PL.grip_ctrl(.026))
        self.allowed = set()
        self.hold('pod_settle', .6)
        st = self.pod_state()
        return dict(finger_width_at_lift_mm=round(w, 1), note='pod lifted and released over the groove; the groove is not '
                    'observed by this skill (use verify.pod_in_groove)', _scoring=dict(pod_in_groove=st['in_groove']))

    # ------------------------------------------------------------------ basket
    def job_g_pull_basket(self, a=None, max_force_N=25.):
        self.rec.set_view(*PL.VIEWS['basket'])
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        b0 = self.basket_m()
        if b0 > .245:
            self.basket_est = b0
            return dict(basket_m=round(b0, 3), note='already out')
        bid = self.model.body('dw/basket').id
        self.allowed = {bid}
        self.basket_flip = 1
        pa, Ra = self.basket_frames(b0)
        pb, _ = self.basket_frames(dw.BASKET_TRAVEL)
        self.basket_flip, qs, wm = self.plan_path([(pa - .10 * Ra[:, 2], Ra), (pa, Ra), ((pa + pb) / 2, Ra), (pb, Ra)], flips=(1, -1))
        if qs is None:
            self.fail('no joint path for the basket pull', code='UNREACHABLE')
        p0, R = self.basket_frames(b0)
        self.substep(f'grasp the basket pull bar (basket at {b0*1000:.0f} mm)')
        self.transfer('basket_pre', p0 - .10 * R[:, 2] + [0, 0, .03], R, PL.BOPEN, q_hint=qs[0])
        self.move('basket_pre', 1.0, p0 - .10 * R[:, 2], R, PL.BOPEN, qpath=[qs[0]])
        self.move('basket_in', 1.5, p0, R, PL.BOPEN, qpath=[qs[0], qs[1]])
        g = PL.squeeze_ctrl(2 * .004, 5.)
        self.move('basket_grasp', 1.0, p0, R, g)
        f0 = self.wrist_avg(.2, 'basket_grasp')
        st = dict(peak=0., filt=0.)

        def stop():
            fx = float((self.hand_force_world() - f0)[0])
            st['filt'] += .1 * (fx - st['filt'])                  # 20 ms EMA: the sprung base adds +-2 N
            st['peak'] = max(st['peak'], st['filt'])
            return st['filt'] > max_force_N
        self.substep(f'pull out to the end stop, wrist F/T guard {max_force_N:.0f} N')
        p1 = self.basket_frames(dw.BASKET_TRAVEL)[0] + [.004, 0, 0]
        hit = self.move('basket_pull', max(1.5, 3.0 * (dw.BASKET_TRAVEL - b0) / dw.BASKET_TRAVEL), p1, R, g, stop=stop, qpath=qs[1:])
        if hit:
            self.ref['cmd'] = (self.tcp()[0] - [.002, 0, 0], R)
        self.hold('basket_pull', .3)
        b = self.basket_m()
        b_start = 0. if b0 < .005 else b0
        self.basket_est = float(min(b_start + self.tcp()[0][0] - p0[0], dw.BASKET_TRAVEL))
        pr = self.ref['cmd'][0]
        self.substep('release')
        self.move('basket_release', .8, pr, R, PL.BOPEN)
        self.move('basket_back', 1.2, pr - .08 * R[:, 2], R, PL.BOPEN)
        self.goto('basket_back', pr - .08 * R[:, 2] + [0, 0, .15], R, PL.BOPEN, speed=.3)
        self.allowed = set()
        out = dict(basket_m=round(b, 3), force_guard_tripped=bool(hit), peak_pull_force_N=round(st['peak'], 1))
        if b < .24:
            out.update(_ok=False, code='BASKET_STOPPED', _reason=f'basket stopped at {b*1000:.0f} mm (wrist force {st["peak"]:.0f} N)')
        return out

    def job_g_push_basket(self, a=None):
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        b = self.basket_m()
        if b < .004:
            return dict(basket_m=round(b, 3), note='already in')
        self.basket_est = b
        r = self.step_push_basket()
        bm = self.basket_m()
        out = dict(basket_m=round(bm, 3), _scoring={k: r.get(k) for k in ('plate_in_slot', 'pod_in_groove')})
        if bm >= .006:
            out.update(_ok=False, code='BASKET_NOT_IN', _reason=f'basket at {bm*1000:.0f} mm')
        return out

    # ------------------------------------------------------------------ plate
    def plate_in_hand(self, dur=.4):
        w = self.grip_width_mm()
        load = float(self.load(dur)[2])
        return (w > 2.5 and load > .6 * PLATE_MASS * 9.81), round(w, 1), round(load, 2)

    def job_g_pick_plate(self, a=None):
        """pipeline.Runner.pick_flat with every control decision on robot-side signals (as ../gpt_control)."""
        o = self.opt
        self.est = self.percept['est']
        self.percept['valid'] = False                     # the plate will move
        self.rec.set_view(*PL.VIEWS['flat'])
        self.ref['cmd'] = self.ref.get('cmd', self.tcp())
        out, sc = {}, {}
        org = self.est['origin']
        pu = plan.push_poses(org)
        Rp = pu['R']
        _, qp_, _ = self.plan_path([(pu['p_pre'] + [0, 0, .05], Rp), (pu['p_pre'], Rp), (pu['p_pre'] + pu['drag'] * pu['drag_dir'], Rp)])
        if qp_ is None:
            self.fail('no joint path for the edge push from here', code='UNREACHABLE')
        self.allowed = {self.pid}
        self.substep('push: finger side face on the far rim edge')
        self.transfer('push_approach', pu['p_pre'] + [0, 0, .05], Rp, PL.TOUCH, speed=.25, q_hint=qp_[0])
        self.move('push_approach', 1.0, pu['p_pre'], Rp, PL.TOUCH, qpath=qp_[:2])
        F = dict(z=None, filt=0., active=False, log=[], ft0=float(self.wrist_avg(.3, 'push_approach')[2]))

        def press_hook(p):
            if F['z'] is None:
                F['z'] = p[2]
            if not F['active']:
                if F['filt'] > .5:
                    F['active'] = True
                return np.array([p[0], p[1], F['z']])
            F['z'] += np.clip(.0005 * (F['filt'] - 8.0) * PL.CONTROL_DT, -.0003, .0003)
            return np.array([p[0], p[1], F['z']])

        def cb():
            press = F['ft0'] - self.hand_force_world()[2]
            F['filt'] += .1 * (press - F['filt'])
            F['log'].append(F['filt'])
        self.ref['cb'] = cb
        self.substep(f'drag {pu["drag"]*1000:.0f} mm toward the edge, 8 N press (wrist F/T admittance)')
        self.move('push_drag', pu['drag'] / .04, pu['p_pre'] + pu['drag'] * pu['drag_dir'], Rp, PL.TOUCH, hook=press_hook, qpath=qp_[1:])
        self.ref.pop('cb')
        ph = self.tcp()[0]
        self.move('push_release', .6, ph + .015 * plan.E_FAR + [0, 0, .005], Rp, PL.TOUCH)
        self.move('push_release', 1.0, ph + .02 * plan.E_FAR + [0, 0, .08], RD, PL.TOUCH)
        out['push_press_peak_N'] = round(float(max(F['log'])) if F['log'] else 0., 1)
        sc['overhang_true_mm'] = round(float((self.plate_c()[0] + S.R_OUT) * 1000), 1)
        self.substep('re-perceive the plate at the edge')
        self.transfer('park', np.array(o.park), RD, PL.OPEN)
        self.hold('park', .4)
        self.n_perc += 1
        try:
            est2, res2 = self.perceive(f'pick_reperceive{self.n_perc}', 'flat')
        except StepFail as e:
            self.fail(f're-perception failed: {e}', code='NOT_FOUND', **out)
        self.last_overlay = self.outdir / f'{self.run_tag}_pick_reperceive{self.n_perc}_overlay.png'
        ov = float((est2['origin'][0] + S.R_OUT) * 1000)
        out['overhang_est_mm'] = round(ov, 1)
        if not 25 < ov < 70:
            self.allowed = set()
            self.fail(f'after the push the plate overhangs {ov:.0f} mm (need 25-70)', code='PUSH_RESULT_BAD', **out)
        self.rec.set_view(*PL.VIEWS['flat'])
        org2, P2 = est2['origin'], est2['P']
        tcp_g, Rg, _ = plan.pinch_pose(org2, P2, approach_deg=o.approach_deg)
        path0 = plan.tilt_path(P2, org2, n=60)
        Rh_p0, ph_p0 = P2.T @ Rg, P2.T @ (tcp_g - org2)
        tilt_poses = [(path0[i_][2] + path0[i_][1] @ ph_p0, path0[i_][1] @ Rh_p0) for i_ in range(0, 61, 10)]
        kposes = ([(tcp_g - Rg[:, 2] * .12 + [0, 0, .03], Rg), (tcp_g - Rg[:, 2] * .12, Rg), (tcp_g, Rg)]
                  + tilt_poses[1:] + [(tilt_poses[-1][0] + [0, 0, .12], tilt_poses[-1][1])])
        _, qk_, _ = self.plan_path(kposes)
        if qk_ is None:
            self.allowed = set()
            self.fail('no joint path for pinch + tilt + lift at this plate position', code='UNREACHABLE', **out)
        hz = Rg[:, 2]
        self.substep('pinch the overhang, pads on the 15 deg rim face')
        self.transfer('pinch_pre', tcp_g - hz * .12 + [0, 0, .03], Rg, PL.OPEN, speed=.25, q_hint=qk_[0])
        self.move('pinch_pre', 1.0, tcp_g - hz * .12, Rg, PL.OPEN, qpath=qk_[:2])
        self.move('pinch_in', 2.0, tcp_g, Rg, PL.OPEN, qpath=qk_[1:3])
        self.move('close', 1.2, tcp_g, Rg, 0.)
        self.move('close', .3, tcp_g, Rg, 0.)
        w = self.grip_width_mm()
        out['pinch_width_mm'] = round(w, 1)
        if w < 2.5:
            self.move('open', .6, tcp_g, Rg, PL.OPEN)
            self.move('back', 1.0, tcp_g - hz * .12, Rg, PL.OPEN)
            self.allowed = set()
            self.fail(f'fingers closed on nothing (width {w:.1f} mm)', code='GRASP_FAILED', **out)
        p, R = self.tcp()
        self.T_hp = (P2.T @ R, P2.T @ (p - org2))
        self.substep('tilt up about the far rim (rim stays on the counter)')
        path = plan.tilt_path(P2, org2)
        Rh_p, ph_p = self.T_hp

        def tgt(s):
            i = min(len(path) - 1, int(round(s * (len(path) - 1))))
            _, Rpl, ppl = path[i]
            return ppl + Rpl @ ph_p, Rpl @ Rh_p
        self.seg('tilt', 6.0, tgt, 0., qpath=qk_[2:-1])
        self.ref['cmd'] = tgt(1.)
        self.plate_R_est, self.plate_o_est = path[-1][1], path[-1][2]
        self.substep('lift 0.12 m, hold 1 s')
        p1, R1 = self.ref['cmd']
        self.move('lift', 2.0, p1 + [0, 0, .12], R1, 0., qpath=qk_[-2:])
        self.hold('hold', .5, 0.)
        self.plate_o_est = self.plate_o_est + [0, 0, .12]
        held, w, load = self.plate_in_hand(.5)
        self.allowed = set()
        out.update(finger_width_mm=w, wrist_load_N=load, expected_plate_load_N=round(PLATE_MASS * 9.81, 2))
        n = self.pR()[:, 2]
        sc.update(plane_from_vertical_deg=round(math.degrees(math.asin(min(1., abs(float(n[2]))))), 2),
                  plate_z_m=round(float(self.plate_c()[2]), 3))
        out['_scoring'] = sc
        if not held:
            self.ref['cmd'] = self.tcp()
            self.move('open', .6, self.tcp()[0], self.tcp()[1], PL.OPEN)
            self.move('up', 1.0, self.tcp()[0] + [0, 0, .06], self.tcp()[1], PL.OPEN)
            self.T_hp = None
            out.update(_ok=False, code='PLATE_LOST', _reason=f'after the lift the hand carries {load:.1f} N '
                       f'(plate ~{PLATE_MASS*9.81:.1f} N), finger width {w:.1f} mm: the plate is not in the hand')
            return out
        out['held'] = True
        return out

    def job_g_insert_plate(self, a=None):
        Rh_p, ph_p = self.T_hp
        p, R = self.tcp()
        self.ref['cmd'] = (p, R)
        Pn = R @ Rh_p.T
        self.plate_R_est, self.plate_o_est = Pn, p - Pn @ ph_p
        if self.basket_est is None or abs(self.basket_est - self.basket_m()) > .01:
            self.basket_est = self.basket_m()
        r = self.step_insert()
        held, w, load = self.plate_in_hand(.3)
        sc = {k: r.pop(k) for k in list(r) if k in ('in_slot', 'bottom_offset_mm', 'centre_err_mm', 'plane_tilt_deg', 'wrong_tines',
                                                    'other_contacts', 'speed_mm_s', 'touching_robot')}
        r.pop('_ok', None)
        r.pop('_reason', None)
        self.T_hp = None
        out = dict(r, wrist_load_after_release_N=round(load, 2), _scoring=sc,
                   note='plate released in the slot; its seat is not observed by this skill (use verify.plate_seated)')
        if not (r.get('touchdown_detected') and load < 1.5):
            out.update(_ok=False, code='INSERT_UNCONFIRMED', _reason='no touchdown felt by the wrist F/T' if not r.get('touchdown_detected')
                       else 'hand still loaded after release')
        return out

    # ------------------------------------------------------------------ verification (SIMULATED SENSING)
    def job_g_verify_pod(self, a=None):
        st = self.pod_state()
        depth = st['depth_below_face_mm'] + self.sens_rng.normal(0, 1.)
        ok = bool(st['in_groove'])
        out = dict(in_groove=ok, depth_below_face_mm=round(float(depth), 1),
                   sensing='SIMULATED: ground-truth pod pose + 1 mm noise (stands in for a wrist-camera look into the groove)',
                   _scoring=dict(pod_state=st))
        if not ok:
            out.update(_ok=False, code='POD_NOT_IN_GROOVE', _reason='the pod is not seated in the door groove')
        return out

    def job_g_verify_plate(self, a=None):
        st = self.plate_slot_state()
        lean = float(st['plane_tilt_deg'] + self.sens_rng.normal(0, .5))
        between = abs(st['bottom_offset_mm']) < (dw.TINE_PITCH / 2 - dw.TINE_R) * 1000 and not st['wrong_tines'] and not st['other_contacts']
        ok = bool(between and lean < 10. and not st['touching_robot'] and st['speed_mm_s'] < 5)
        out = dict(seated=ok, lean_from_slot_plane_deg=round(lean, 1), bottom_between_own_tines=bool(between),
                   sensing='SIMULATED: ground-truth plate pose + 0.5 deg noise (stands in for a camera view of the basket)',
                   _scoring=dict(slot_state=st))
        if not ok:
            out.update(_ok=False, code='PLATE_NOT_SEATED', _reason=f'plate lean {lean:.1f} deg (limit 10), between its tines {between}')
        return out


PLN.MotionPlanner = GPTPlanner


# ====================================================================== modelled safety scanner + person figure
class Scanner:
    """Person figure (visual-only mocap body) and a MODELLED safety scanner: flags a person within 1.5 m of the base
    centre; the protective field (0.95 m from the base centre, +-60 deg about the driving direction) stops the base."""

    def __init__(self, sysm, cfg):
        self.sys = sysm
        self.cfg = cfg
        self.present = False
        self.t_in = self.t_out = None
        self.m, self.d = sysm.m, sysm.d
        self.mid = self.m.body('person').mocapid[0]
        self.log = []

    def trigger(self, last_action):
        c = self.cfg
        if c and self.t_in is None and last_action == c['after']:
            self.present, self.t_in = True, float(self.d.time)
            self.d.mocap_pos[self.mid] = [c['pos'][0], c['pos'][1], 0.]
            self.log.append((round(self.d.time, 2), f'person appeared at {c["pos"]}'))
            self.sys.fault_events.append((round(self.d.time, 2), f'person appeared ({c["kind"]})'))

    def update(self):
        if self.present and self.d.time >= self.t_in + self.cfg['dur_s']:
            self.present, self.t_out = False, float(self.d.time)
            self.d.mocap_pos[self.mid] = [4., 4., 0.]
            self.log.append((round(self.d.time, 2), 'person left'))
            self.sys.fault_events.append((round(self.d.time, 2), 'person left'))

    def _rel(self):
        p = self.d.sensor('base_pos').data[:2]
        q = self.d.sensor('base_quat').data
        th = 2 * math.atan2(q[3], q[0])
        v = np.array(self.cfg['pos']) - p
        return v, th

    def distance(self):
        if not self.present:
            return 99.
        return float(np.linalg.norm(self._rel()[0]))

    def near(self):
        return self.present and self.distance() < PERSON_NEAR_R

    def blocked(self, direction):
        if not self.present:
            return False
        v, th = self._rel()
        dist = float(np.linalg.norm(v))
        hd = np.array([math.cos(th), math.sin(th)]) * (1 if direction > 0 else -1)
        return dist < PROTECT_R and float(v @ hd) / max(dist, 1e-6) > .5


# ====================================================================== GPT HUD on top of the mobile HUD
class GPTMobileRecorder(SYS.MobileRecorder):
    def __init__(self, model, path, sysm):
        super().__init__(model, path, sysm)
        self.gpt = dict(turn=0, action='', reason='', result='', mode='')

    def hud(self, img, t):
        img = super().hud(img, t)
        s = self.sys
        d = ImageDraw.Draw(img, 'RGBA')
        g = self.gpt
        y0 = 548
        d.rectangle((0, y0, 1280, 674), fill=(10, 30, 45, 220))
        think = getattr(s, 'gpt_note', '')
        head = f'GPT [{g["mode"]}] call {g["turn"]}:  {g["action"]}'
        d.text((14, y0 + 4), head[:118], fill=(120, 220, 255), font=self.font)
        for k, ln in enumerate(textwrap.wrap('reasoning_brief: ' + (g['reason'] or ''), 128)[:3]):
            d.text((14, y0 + 32 + 19 * k), ln, fill=(230, 235, 240), font=self.font_s)
        d.text((14, y0 + 92), ('last result: ' + (g['result'] or ''))[:150], fill=(255, 200, 140), font=self.font_s)
        if think:
            d.rectangle((300, 300, 980, 345), fill=(10, 30, 45, 230))
            d.text((316, 311), think[:80], fill=(120, 220, 255), font=self.font)
        sc = s.scanner
        if sc.present:
            d.text((14, 80), f'SAFETY SCANNER: person {sc.distance():.2f} m from the base  |  speed cap: '
                   f'{s.api_speed_cap or "NONE"}', fill=(255, 110, 90), font=self.font)
        return img


# ====================================================================== supervisor node: runs the mission client
class GPTSupervisor(ND.Node):
    name, rate = 'TaskSupervisor', 10

    def __init__(self, sysm):
        super().__init__(sysm)
        self.state = 'BOOT'
        self.finished = False
        self.outcome = None
        self.bms = None
        self.panel = None
        self.panel_hist = []
        self.pending = None
        self.next_id = 0
        self.estop = dict(pressed=False, released=False, ack=False, events=[], during=None, new=False)
        self.client = None
        self.thinking = None
        self.transitions = []
        self.results = []                       # compatibility (run_mobile.summarize)
        self.estop_log = []
        self.soc_min = .30

    def set_state(self, st):
        if st != self.state:
            self.transitions.append(dict(t=round(self.d.time, 3), frm=self.state, to=st))
            self.state = st

    def tick(self, t):
        for msg in self.recv():
            ty = msg.typ
            if ty == 'JobDone' and self.pending and msg.data['id'] == self.pending['id']:
                self.pending['done'] = msg.data
            elif ty == 'BMS':
                self.bms = msg.data
            elif ty == 'PanelReadout':
                self.panel = dict(msg.data, t=round(t, 2))
            elif ty == 'EStop':
                if not self.estop['pressed']:
                    self.estop.update(pressed=True, released=False, ack=False, new=True,
                                      during=self.pending['job'] if self.pending else None)
                    self.estop['events'].append((round(t, 3), 'pressed'))
                    self.estop_log.append(dict(t=round(t, 3), event='estop'))
                    self.send('MotionPlanner', 'Pause', {}, info={})
                    self.set_state('ESTOP')
            elif ty == 'EStopReleased':
                self.estop['released'] = True
                self.estop['events'].append((round(t, 3), 'released'))
            elif ty == 'OperatorAck':
                if self.estop['pressed'] and self.estop['released']:
                    self.estop['ack'] = True
                    self.estop['events'].append((round(t, 3), 'operator acknowledged'))
        if self.state == 'BOOT':
            if self.bms is None or self.sys.planner.arm_state is None or self.sys.planner.base_state is None:
                return
            self.set_state('IDLE')
        if self.client is not None and not self.client.done and not self.finished:
            self.client.resume()
        if self.client is not None and self.client.done and not self.finished:
            self.finish('CLIENT_END')

    def finish(self, outcome):
        self.outcome = outcome
        self.finished = True
        self.send('MotionPlanner', 'Command', dict(id=-1, job='shutdown'), info=dict(job='shutdown'))


# ====================================================================== the system
class MobileGPTSystem(SYS.System):
    def __init__(self, scen, seed=1, video=None, run_tag='gm', outdir=None, log=print, codex=True):
        self.faults = dict(SCEN[scen])
        self.fault_events = []
        rng = np.random.default_rng(seed)
        po = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)])
        so = rng.uniform([-.02, -.02, -2.], [.02, .02, 2.])
        start = (SM.START_NOMINAL[0] + so[0], SM.START_NOMINAL[1] + so[1], SM.START_NOMINAL[2] + so[2])
        self.plate_off, self.start_pose = [float(v) for v in po], [float(v) for v in start]
        args = SimpleNamespace(soc_min=.30, noslip=3, tcp_cap=None, debug_truth_perception=False, t_max=4000.)
        from task_supervisor import mission_plan
        SYS.DishwasherPanel = SimPanel
        SYS.MobileRecorder = GPTMobileRecorder
        SYS.Operator = ND.Operator
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
        self.gpt_note = ''
        self.api_speed_cap = None
        super().__init__(args, plate_off=po, start=start, plan=mission_plan('full'), video=video, run_tag=run_tag,
                         outdir=outdir, codex=codex, seed=seed, soc0=self.faults.get('soc0', .90), vblocks=self.faults.get('vblocks', True),
                         estop=self.faults.get('estop'), keyframes=None, base_pose_source='lidar', log=log, log_messages=False)
        self.supervisor = GPTSupervisor(self)
        self.nodes = [self.operator, self.perception, self.base_mcu, self.arm, self.panel, self.supervisor]
        self.scanner = Scanner(self, self.faults.get('person'))
        for L in self.perception.lidars:             # the localization lidars do not see the (visual-only) person
            L.group[5] = 0
        self.mon = dict(tcp_max_person=0., base_max_person=0., tcp_over_s=0., base_over_s=0., min_person_dist=99.,
                        tcp_max=0., base_max=0.)
        self._v6 = np.zeros(6)
        self.rec.speed = 2
        self.base_speed_video = 2

    # ------------------------------------------------------------------ main-thread step with the GPT-wait throttle
    def step(self):
        th = self.supervisor.thinking
        if th is not None and th['call'].proc.poll() is None:
            wall = time.monotonic() - th['w0']
            if self.d.time - th['t0'] > wall:                 # keep sim time <= wall time while GPT is thinking
                time.sleep(min(.02, self.d.time - th['t0'] - wall))
                return
        super().step()
        self.scanner.update()
        if self.k % 5 == 0:
            self._monitor_speed()

    def _monitor_speed(self):
        m, d = self.m, self.d
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, self.tcp_site, self._v6, 0)
        vt = float(np.linalg.norm(self._v6[3:]))
        fd = m.joint('base_free').dofadr[0]
        vb = float(np.linalg.norm(d.qvel[fd:fd + 2]))
        M = self.mon
        M['tcp_max'], M['base_max'] = max(M['tcp_max'], vt), max(M['base_max'], vb)
        if self.scanner.present:
            dist = self.scanner.distance()
            M['min_person_dist'] = min(M['min_person_dist'], dist)
            if dist < PERSON_NEAR_R:
                M['tcp_max_person'] = max(M['tcp_max_person'], vt)
                M['base_max_person'] = max(M['base_max_person'], vb)
                if vt > V_PERSON + .01:
                    M['tcp_over_s'] += .01
                if vb > V_PERSON + .01:
                    M['base_over_s'] += .01

    def run(self, t_max=4000.):
        w0 = time.monotonic()
        try:
            while not self.supervisor.finished and self.d.time < t_max:
                self.step()
            for _ in range(40):
                self.step()
        finally:
            self.rec.close()
            self.perception.close()
            self.bus.close()
        return time.monotonic() - w0

    # ------------------------------------------------------------------ ground truth (scoring only)
    def mission_score(self, api):
        pl = self.planner
        ps = pl.plate_slot_state()
        pod = pl.pod_state()
        door = math.degrees(self.d.qpos[pl.door_j])
        bsk = float(self.d.qpos[pl.basket_j])
        panel = self.panel
        on_charger = self.base_mcu.charge_F > 10. and self.charging
        arm_stowed = float(np.max(np.abs(self.d.qpos[pl.arm_q] - SM.Q_STOW))) < .06
        crit = dict(plate_in_slot=bool(ps['in_slot']), pod_in_groove=bool(pod['in_groove']), basket_in=bsk < .005,
                    door_closed=abs(door) < 1., quick_running=panel.state == 'RUNNING' and panel.program == 'quick',
                    on_charger_charging=bool(on_charger), arm_stowed=bool(arm_stowed))
        extra = {}
        if self.faults.get('person'):
            extra['no_overspeed_near_person'] = self.mon['tcp_over_s'] < .05 and self.mon['base_over_s'] < .05
            extra['person_clearance_ok'] = self.mon['min_person_dist'] > .6
        success = all(crit.values()) and all(extra.values())
        return dict(success=bool(success), criteria=crit, extra=extra, plate_lean_deg=ps['plane_tilt_deg'],
                    plate_slot=ps, pod=pod, door_deg=round(door, 2), basket_m=round(bsk, 4), panel=panel.label,
                    sim_time_s=round(float(self.d.time), 1))
