"""Scheduler + physics + monitors + video. One System = one mission in one continuous mj_step simulation.

Per 2 ms step: deliver due bus messages -> tick every node whose period divides the step counter (Operator,
Perception, BaseMCU, ArmController, DishwasherPanel, TaskSupervisor, MotionPlanner) -> mj_step -> monitors (scoring only:
contacts, wheel / caster loads, centre of pressure, base creep, brake torque, joint margins, TCP speed) -> electrical
power -> battery -> video frame (streamed to ffmpeg, nothing accumulated).
"""
from __future__ import annotations

import math
import time
from collections import defaultdict
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import pipeline as PL
import power as PW
import scene_mobile as SM
from bus import Bus
from nodes import DT, ArmController, BaseMCU, DishwasherPanel, Operator, Perception
from task_supervisor import TaskSupervisor

FOOTER = ("Simulation (MuJoCo). Physics: rigid bodies, contacts, wheels, brakes, joints. Modelled: motor/battery electrics, "
          "bus latency, panel MCU, lidar noise. Plate pose from camera + Codex.")


def hull2d(P):
    P = sorted(map(tuple, P))
    if len(P) < 3:
        return np.array(P)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def inside_margin(H, c):
    if len(H) < 3:
        return -1.
    m = 1e9
    for i in range(len(H)):
        a, b = H[i], H[(i + 1) % len(H)]
        e = b - a
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)      # inward for a CCW hull
        m = min(m, float((c - a) @ n))
    return m


class MobileRecorder(PL.Recorder):
    def __init__(self, model, path, sysm):
        super().__init__(model, path, 'mobile')
        self.sys = sysm
        self.follow = False
        self.status_steps = []

    def hud(self, img, t):
        s = self.sys
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 0, 1280, 44), fill=(18, 22, 28, 235))
        sup = s.supervisor
        d.text((14, 9), f'{getattr(s, "mission_label", "")}', fill='white', font=self.font_b)
        d.text((880, 4), 'Mobile Franka (FR3 limits) loads a dishwasher', fill=(200, 210, 220), font=self.font_s)
        d.text((880, 23), f'mission: {s.run_tag}', fill=(200, 210, 220), font=self.font_s)
        d.rectangle((0, 44, 1280, 70), fill=(18, 22, 28, 170))
        d.text((14, 47), (s.status_note or self.substep)[:120], fill=(255, 220, 120), font=self.font_s)
        d.text((1110, 47), f't = {t:6.1f} s  {self.speed}x', fill=(200, 210, 220), font=self.font_s)
        # chips panel
        x0, y0 = 900, 84
        b = s.battery
        mcu = s.base_mcu
        arm = s.arm
        pl = s.planner
        rows = [('TaskSupervisor 10 Hz', sup.state, (255, 220, 120)),
                ('MotionPlanner 100 Hz', (pl.job + ': ' + (pl.phase or '')[:24]) if pl.job else 'idle', (200, 230, 255)),
                ('ArmController 500 Hz', arm.mode, (90, 230, 120) if arm.mode == 'RUN' else (255, 90, 80)),
                ('BaseMCU 500 Hz', f'{"BRAKED" if mcu.brake_level >= 1 else ("brake " + str(round(mcu.brake_level, 1)) if mcu.brake_level > 0 else "driving")}'
                 f'{" E-STOP" if mcu.estop else ""}  v {mcu.v_ref:+.2f} m/s', (255, 90, 80) if mcu.estop else (200, 230, 255)),
                ('Battery (BMS)', f'SoC {b.soc * 100:5.1f} %  {b.V:4.1f} V  {b.I:5.1f} A', (90, 230, 120) if b.soc > .3 else (255, 140, 80)),
                ('Dock force', f'{mcu.dock_F:5.0f} N   charge {"YES" if s.charging else "no"}', (200, 230, 255)),
                ('Dishwasher MCU', s.panel.label, (90, 230, 120) if s.panel.state == 'RUNNING' else (200, 230, 255))]
        d.rectangle((x0 - 8, y0 - 8, 1272, y0 + 22 * len(rows) * 2 + 40), fill=(18, 22, 28, 200))
        for i, (k, v, col) in enumerate(rows):
            d.text((x0, y0 + 44 * i), k, fill=(170, 175, 180), font=self.font_s)
            d.text((x0 + 12, y0 + 44 * i + 19), str(v)[:40], fill=col, font=self.font_s)
        # panel LEDs
        ly = y0 + 44 * len(rows) + 4
        leds = s.panel.leds()
        for j, nm in enumerate(['eco', 'intensive', 'quick', 'dry', 'fruit', 'power']):
            st = leds.get(nm, 'off')
            on = (st is True) or st in ('steady',)
            dim = st == 'dim'
            c = (90, 230, 120) if on else ((60, 110, 70) if dim else (60, 60, 60))
            d.ellipse((x0 + 58 * j, ly, x0 + 58 * j + 12, ly + 12), fill=c)
            d.text((x0 + 58 * j - 4, ly + 14), nm[:5], fill=(170, 175, 180), font=self.font_s)
        d.rectangle((0, 676, 1280, 720), fill=(18, 22, 28, 235))
        cut = FOOTER.rfind(' ', 0, 118)
        d.text((14, 679), FOOTER[:cut], fill=(210, 214, 220), font=self.font_s)
        d.text((14, 698), FOOTER[cut + 1:], fill=(210, 214, 220), font=self.font_s)
        return img

    def capture(self, data, force=False):
        if self.writer is None:
            return
        if self.follow and data.time + 1e-9 >= self.next_time:
            p = data.body('base').xpos
            self.view_target = np.array([p[0] - .2, p[1], .55, 3.4, 200., -30.])
        super().capture(data, force)


class System:
    def __init__(self, args, plate_off=(0, 0, 0), start=SM.START_NOMINAL, plan=None, video=None, run_tag='mobile',
                 outdir=None, codex=True, seed=0, soc0=.90, vblocks=True, estop=None, keyframes=None, base_pose_source='lidar',
                 believed_base=None, tcp_cap=None, tool='stylus', log=print, log_messages=True):
        import run_mobile as RM
        self.args = args
        self.run_tag = run_tag
        self.outdir = Path(outdir)
        self.cfg = SM.MobileConfig(start=tuple(start), vblocks=vblocks)
        self.bs = SM.SPEC_BASE
        lay = RM.OPTION.layout
        xy = np.array(lay.plate0) + np.array(plate_off[:2])
        self.m, self.info = SM.build(self.cfg, plate_xy=xy, plate_yaw=plate_off[2], layout=lay)
        m = self.m
        m.opt.noslip_iterations = getattr(args, 'noslip', 3)   # removes soft-friction creep of the braked tyres
        self.d = mujoco.MjData(m)
        d = self.d
        qa = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        d.qpos[qa] = SM.Q_STOW
        d.qpos[[m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]] = .04
        d.ctrl[[m.actuator(f'panda/actuator{i}').id for i in range(1, 8)]] = SM.Q_STOW
        d.ctrl[m.actuator('panda/actuator8').id] = PL.OPEN
        mujoco.mj_forward(m, d)
        self.bus = Bus(self.outdir / f'{run_tag}_messages.jsonl.gz' if log_messages else None)
        batt = SM.SPEC_BATT
        self.battery = PW.Battery(capacity_Ah=batt.get('capacity_Ah', 30.), nominal_V=batt.get('nominal_V', 51.2), soc=soc0)
        self.power = PW.PowerModel(m, SM.SPEC_POWER)
        self.charging = False
        self.status_note = ''
        self.mission_label = ''
        self.step_name = ''
        self.main_queue = []
        self.rec = MobileRecorder(m, video, self)
        self.rec.set_view(*PL.VIEWS['overview'], cut=True)
        self.rec.follow = True
        codex_dir = self.outdir / 'codex'
        codex_dir.mkdir(parents=True, exist_ok=True)
        self.perception = Perception(self, codex=codex, outdir=codex_dir, run_tag=run_tag, rng=np.random.default_rng(seed + 100))
        self.base_mcu = BaseMCU(self, self.bs)
        self.arm = ArmController(self)
        self.panel = DishwasherPanel(self)
        self.supervisor = TaskSupervisor(self, plan, soc_min=args.soc_min)
        self.operator = Operator(self, **(estop or {}))
        import planner as PLN
        est0 = (start[0], start[1], math.radians(start[2])) if base_pose_source == 'truth' else None
        if est0 is None:
            est0 = SM.START_NOMINAL[:2] + (math.radians(SM.START_NOMINAL[2]),) if plan[0]['job'] == 'localize' else \
                (SM.dock_base_pose()[0], SM.dock_base_pose()[1], 0.)
        self.planner = PLN.MotionPlanner(self, RM.OPTION, codex=codex, seed=seed, log=log, keyframes_dir=keyframes, run_tag=run_tag,
                                         base_pose_source=base_pose_source, base_start_est=est0, believed_base=believed_base,
                                         tcp_cap=tcp_cap, press_tool=tool, outdir=codex_dir)
        self.planner.tick_node_period = 5
        self.nodes = [self.operator, self.perception, self.base_mcu, self.arm, self.panel, self.supervisor]
        self.k = 0
        # ---- monitors
        self.wheel_geoms = {m.geom(n).id: n for n in ['wheel_l_tyre', 'wheel_r_tyre'] + [f'caster{i}_ball' for i in range(4)]}
        self.floor = m.geom('floor').id
        self.load_buf = []
        self.phase_stats = defaultdict(lambda: dict(min_load={}, max_load={}, cop_margin_min=9., brake_tq_max=0.,
                                                    dock_F_max=0., dock_F_min=1e9, creep_mm_max=0., creep_deg_max=0., n=0))
        self.task_ref_pose = None
        self.step_speed = defaultdict(lambda: dict(tcp_max=0., elbow_max=0., q6_low_margin_min=9., q4_high_margin_min=9.,
                                                   margin_min=9., margin_joint=None))
        self.energy_state = defaultdict(float)
        self.time_state = defaultdict(float)
        self.wd = [m.joint('wheel_l').dofadr[0], m.joint('wheel_r').dofadr[0]]
        self.qadr = qa
        self.lo = m.jnt_range[[m.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        self.hi = m.jnt_range[[m.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        self.tcp_site = m.site('tcp').id
        self.l4 = m.body('panda/link4').id
        self.f6 = np.zeros(6)
        self.v6 = np.zeros(6)
        self.peakP = defaultdict(float)
        self.estop_probe = None

    # ------------------------------------------------------------------
    def step(self):
        d, m = self.d, self.m
        t = d.time
        self.bus.deliver(t)
        for n in self.nodes:
            if self.k % n.period == 0:
                n.tick(t)
        if self.k % 5 == 0:
            self.planner.tick_node(t)
        mujoco.mj_step(m, d)
        self.k += 1
        self.planner._monitor()
        if self.k % 5 == 0:
            self.planner._sample_wrench()
            self._monitor_base()
            self._monitor_arm()
        self._power()
        while self.main_queue:
            self.main_queue.pop(0)()
        self.rec.capture(d)

    def _power(self):
        d = self.d
        P = self.power.power(d, self.base_mcu.brakes_released, self.perception.lidar_on)
        st = self.supervisor.state
        cF = self.base_mcu.charge_F
        self.charging = cF > 10. and self.supervisor.state not in ('BOOT',)
        I_ch = 11.3 if self.charging else 0.
        self.battery.step(P['total'] - I_ch * self.battery.V, DT)
        self.power.accumulate(P, DT)
        self.energy_state[st] += P['total'] * DT / 3600.
        self.time_state[st] += DT
        if self.status_note:                               # robot waiting for perception (Codex)
            self.energy_state['_codex_wait'] += P['total'] * DT / 3600.
            self.time_state['_codex_wait'] += DT
        self._estop_probe(st)
        for k_ in ('total', 'arm', 'wheel'):
            self.peakP[k_] = max(self.peakP[k_], P[k_])

    def _estop_probe(self, st):
        """Scoring: plate pose in the hand and door angle at the e-stop, at the end of the stop and after resuming."""
        d, m = self.d, self.m
        if st == 'ESTOP' and self.estop_probe is None:
            self.estop_probe = dict(t_press=round(d.time, 3))
            self._ep_ref = self._plate_in_hand()
            self.estop_probe['door_deg_at_estop'] = round(math.degrees(d.qpos[m.joint('dw/door_hinge').qposadr[0]]), 2)
            self.estop_probe['plate_finger_contacts_at_estop'] = self._finger_contacts()
        if self.estop_probe is not None and 'slip_during_stop_mm' not in self.estop_probe and st != 'ESTOP':
            p, R = self._plate_in_hand()
            self.estop_probe.update(t_resume=round(d.time, 3),
                                    slip_during_stop_mm=round(float(np.linalg.norm(p - self._ep_ref[0])) * 1000, 2),
                                    rot_during_stop_deg=round(math.degrees(math.acos(np.clip((np.trace(R @ self._ep_ref[1].T) - 1) / 2, -1, 1))), 2),
                                    door_deg_at_resume=round(math.degrees(d.qpos[m.joint('dw/door_hinge').qposadr[0]]), 2),
                                    plate_finger_contacts_at_resume=self._finger_contacts(),
                                    tcp_travel_after_estop_mm=self.arm.stats.get('estop_tcp_travel_mm'),
                                    arm_stop_time_s=self.arm.stats.get('estop_stop_s'))

    def _plate_in_hand(self):
        d = self.d
        h = d.body('panda/hand')
        R = h.xmat.reshape(3, 3)
        pl = d.body('plate')
        return R.T @ (pl.xpos - h.xpos), R.T @ pl.xmat.reshape(3, 3)

    def _finger_contacts(self):
        m, d = self.m, self.d
        pid = m.body('plate').id
        fb = {m.body('panda/left_finger').id, m.body('panda/right_finger').id}
        n = 0
        for i in range(d.ncon):
            b1, b2 = m.geom_bodyid[d.contact[i].geom[0]], m.geom_bodyid[d.contact[i].geom[1]]
            if (b1 == pid and b2 in fb) or (b2 == pid and b1 in fb):
                n += 1
        return n

    def _monitor_base(self):
        m, d = self.m, self.d
        loads = {n: 0. for n in self.wheel_geoms.values()}
        pts, fs = [], []
        for i in range(d.ncon):
            c = d.contact[i]
            g = c.geom[0] if c.geom[0] in self.wheel_geoms else (c.geom[1] if c.geom[1] in self.wheel_geoms else None)
            if g is None:
                continue
            mujoco.mj_contactForce(m, d, i, self.f6)
            loads[self.wheel_geoms[g]] += self.f6[0]
            pts.append(c.pos[:2].copy())
            fs.append(self.f6[0])
        self.load_buf.append(loads)
        if len(self.load_buf) > 5:
            self.load_buf.pop(0)
        avg = {n: float(np.mean([b[n] for b in self.load_buf])) for n in loads}     # 10 ms average (tyre skid chatter)
        st = self.supervisor.state
        ps = self.phase_stats[st]
        ps['n'] += 1
        for n, v in avg.items():
            ps['min_load'][n] = min(ps['min_load'].get(n, 1e9), v)
            ps['max_load'][n] = max(ps['max_load'].get(n, 0.), v)
        # centre of pressure vs support polygon of the 6 nominal contact points (base frame -> world)
        bp = d.body('base')
        R = bp.xmat.reshape(3, 3)
        sup = [bp.xpos[:2] + (R @ np.array([x, y, 0.]))[:2] for x, y in
               [(0, .25), (0, -.25)] + list(SM.CASTER_XY)]
        if fs and sum(fs) > 1:
            cop = np.sum(np.array(pts) * np.array(fs)[:, None], 0) / sum(fs)
            ps['cop_margin_min'] = min(ps['cop_margin_min'], inside_margin(hull2d(sup), cop))
        # brake torque in use: dry-friction constraint force on the wheel dofs
        bt = 0.
        bt = float(np.max(np.abs(d.actuator_force[self.base_mcu.ba])))
        ps['brake_tq_max'] = max(ps['brake_tq_max'], bt)
        fd = m.joint('base_free').dofadr[0]
        ps['base_speed_max'] = max(ps.get('base_speed_max', 0.), float(np.linalg.norm(d.qvel[fd:fd + 3])))
        ps['dock_F_max'] = max(ps['dock_F_max'], self.base_mcu.dock_F)
        ps['dock_F_min'] = min(ps['dock_F_min'], self.base_mcu.dock_F)
        if st.startswith('TASK_') or st.startswith('PANEL_') or st == 'ESTOP':
            p = d.sensor('base_pos').data[:2].copy()
            q = d.sensor('base_quat').data
            yaw = 2 * math.atan2(q[3], q[0])
            if self.task_ref_pose is None:
                self.task_ref_pose = (p, yaw)
            ps['creep_mm_max'] = max(ps['creep_mm_max'], float(np.linalg.norm(p - self.task_ref_pose[0])) * 1000)
            ps['creep_deg_max'] = max(ps['creep_deg_max'], abs(math.degrees(yaw - self.task_ref_pose[1])))

    def _monitor_arm(self):
        m, d = self.m, self.d
        st = self.supervisor.state
        if not (st.startswith('TASK_') or st.startswith('PANEL_') or st in ('UNSTOW_ARM', 'STOW_ARM')):
            return
        s = self.step_speed[st]
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, self.tcp_site, self.v6, 0)
        s['tcp_max'] = max(s['tcp_max'], float(np.linalg.norm(self.v6[3:])))
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.l4, self.v6, 0)
        s['elbow_max'] = max(s['elbow_max'], float(np.linalg.norm(self.v6[3:])))
        q = d.qpos[self.qadr]
        mg = np.minimum(q - self.lo, self.hi - q)
        s['q6_low_margin_min'] = min(s['q6_low_margin_min'], float(q[5] - self.lo[5]))
        s['q4_high_margin_min'] = min(s['q4_high_margin_min'], float(self.hi[3] - q[3]))
        j = int(np.argmin(mg))
        if mg[j] < s['margin_min']:
            s['margin_min'], s['margin_joint'] = float(mg[j]), j + 1

    # ------------------------------------------------------------------
    def run(self, t_max=1500.):
        w0 = time.monotonic()
        try:
            while not self.supervisor.finished and self.d.time < t_max:
                self.step()
            # let the planner thread exit cleanly
            for _ in range(20):
                self.step()
        finally:
            self.rec.close()
            self.perception.close()
            self.bus.close()
        return time.monotonic() - w0
