"""Controller modules ("which chip does what"). Each node has a fixed loop rate and talks to the others ONLY through the
message bus (bus.py, simulated link latency). The simulation scheduler (system.py) steps MuJoCo at 500 Hz and ticks
each node on its own period.

  ArmController   500 Hz  arm control box: setpoint interpolation, joint position servo setpoints, gravity torque,
                          joint-velocity / torque-limit monitoring, wrist F/T, Cat-1 e-stop (decelerate, then hold).
  BaseMCU         500 Hz  base MCU + wheel drivers: wheel velocity PI, accel / yaw-rate limits, command watchdog,
                          spring brakes, dock-force and charge-contact sensing, IMU tilt, odometry, BMS, e-stop chain.
  DishwasherPanel 500 Hz  the appliance's own MCU: capacitive pad scanning (conductive contact >= 0.5 N for >= 80 ms),
                          door sensor, OFF / STANDBY / SELECTED / RUNNING state machine, LEDs, beeps.
  Perception       10 Hz  2 lidars -> localization fixes; wall camera + Codex (async, real latency); panel LED readout.
  TaskSupervisor   10 Hz  mission state machine with timeouts / retries / e-stop handling (task_supervisor.py).
  MotionPlanner   100 Hz  main real-time computer: IK, joint paths, base navigation / docking (planner.py).
  Operator         10 Hz  the human / environment: e-stop button, reset, acknowledgement (scripted per scenario).
"""
from __future__ import annotations

import math
import subprocess
import time
from collections import deque
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

import dishwasher as dw
import lidar as LI
import perception as PC
import scene as S
import scene_mobile as SM

DT = .002


class Node:
    name = 'Node'
    rate = 500

    def __init__(self, sysm):
        self.sys = sysm
        self.bus = sysm.bus
        self.m, self.d = sysm.m, sysm.d
        self.period = max(1, int(round(1. / self.rate / DT)))

    def send(self, dst, typ, data=None, info=None, lat=None):
        return self.bus.send(self.d.time, self.name, dst, typ, data, info, lat)

    def recv(self):
        return self.bus.recv(self.name)


# ====================================================================== arm control box
class ArmController(Node):
    name, rate = 'ArmController', 500
    VMAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])      # FR3 joint speed limits (rad/s, datasheet)
    TAU_MAX = np.array([87, 87, 87, 87, 12, 12, 12.])
    STOP_T, BLEND_T = .20, .50

    def __init__(self, sysm):
        super().__init__(sysm)
        m = self.m
        self.act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 8)]
        self.gact = m.actuator('panda/actuator8').id
        self.qadr = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.dof = [m.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)]
        self.fq = [m.joint(f'panda/finger_joint{i}').qposadr[0] for i in (1, 2)]
        self.fdof = [m.joint(f'panda/finger_joint{i}').dofadr[0] for i in (1, 2)]
        self.gdofs = self.dof + self.fdof
        self.bodies = [b for b in range(m.nbody) if m.body(b).name.startswith('panda/')]
        self.lo = m.jnt_range[[m.joint(f'panda/joint{i}').id for i in range(1, 8)], 0]
        self.hi = m.jnt_range[[m.joint(f'panda/joint{i}').id for i in range(1, 8)], 1]
        self.fs = m.sensor('hand_force').adr[0]
        self.ts = m.sensor('hand_torque').adr[0]
        self.gfull = np.zeros(m.nv)
        q = self.d.qpos[self.qadr].copy()
        self.q_cmd = q.copy()
        self.g_cmd = float(self.d.ctrl[self.gact])
        self.sp = None
        self.i = 0
        self.mode = 'RUN'
        self.t_mode = 0.
        self.v_stop = np.zeros(7)
        self.q_hold = q.copy()
        self.stats = dict(vel_clamps=0, tau_ratio_max=0., tau_ratio_max_joint=None, estop_stop_s=None, estop_tcp_travel_mm=None)
        self.k = 0

    def gravity(self):
        m, d = self.m, self.d
        self.gfull[:] = 0
        g = m.opt.gravity
        for b in self.bodies:
            mujoco.mj_applyFT(m, d, -m.body_mass[b] * g, np.zeros(3), d.xipos[b], b, self.gfull)
        return self.gfull[self.gdofs]

    def tick(self, t):
        d = self.d
        for msg in self.recv():
            if msg.typ == 'ArmSetpoint':
                if self.mode == 'ESTOP':
                    continue                         # setpoints ignored while stopped
                self.sp, self.i = msg.data, 0
            elif msg.typ == 'EStop':
                if self.mode != 'ESTOP':
                    self.mode, self.t_mode = 'ESTOP', t
                    self.v_stop = self.v_last.copy() if hasattr(self, 'v_last') else np.zeros(7)
                    self.estop_tcp0 = d.site('tcp').xpos.copy()
                    self.sp = None
            elif msg.typ == 'Resume':
                if self.mode == 'ESTOP':
                    self.mode, self.t_mode = 'BLEND', t
                    self.q_hold = self.q_cmd.copy()
                    self.sp = None
        q_prev = self.q_cmd.copy()
        if self.mode == 'ESTOP':
            # Cat-1 stop: decelerate the commanded motion linearly to zero within STOP_T, then hold (joint brakes)
            tau = t - self.t_mode
            if tau < self.STOP_T:
                self.q_cmd = self.q_cmd + self.v_stop * (1 - tau / self.STOP_T) * DT
            elif self.stats['estop_stop_s'] is None and np.max(np.abs(d.qvel[self.dof])) < .01:
                self.stats['estop_stop_s'] = round(tau, 3)
                self.stats['estop_tcp_travel_mm'] = round(float(np.linalg.norm(d.site('tcp').xpos - self.estop_tcp0)) * 1000, 2)
        elif self.sp is not None:
            self.i = min(self.i + 1, self.sp['n'])
            a = self.i / self.sp['n']
            q_sp = self.sp['q0'] + a * (self.sp['q1'] - self.sp['q0'])
            self.g_cmd = self.sp['g0'] + a * (self.sp['g1'] - self.sp['g0'])
            if self.mode == 'BLEND':
                w = min(1., (t - self.t_mode) / self.BLEND_T)
                q_sp = self.q_hold + w * (q_sp - self.q_hold)
                if w >= 1:
                    self.mode = 'RUN'
            self.q_cmd = q_sp
        # joint speed limit + joint range (FR3)
        dq = self.q_cmd - q_prev
        lim = self.VMAX * DT
        if np.any(np.abs(dq) > lim + 1e-12):
            self.stats['vel_clamps'] += 1
            self.q_cmd = q_prev + np.clip(dq, -lim, lim)
        self.q_cmd = np.clip(self.q_cmd, self.lo, self.hi)
        self.v_last = (self.q_cmd - q_prev) / DT
        d.ctrl[self.act] = self.q_cmd
        d.ctrl[self.gact] = self.g_cmd
        g = self.gravity()
        d.qfrc_applied[self.gdofs] = g
        tau = np.abs(d.actuator_force[self.act] + g[:7]) / self.TAU_MAX
        j = int(np.argmax(tau))
        if tau[j] > self.stats['tau_ratio_max']:
            self.stats['tau_ratio_max'], self.stats['tau_ratio_max_joint'] = float(tau[j]), j + 1
        st = self.sys.supervisor.state
        ps = self.stats.setdefault('per_state', {}).setdefault(st, [0., 0])
        if tau[j] > ps[0]:
            ps[0], ps[1] = round(float(tau[j]), 3), j + 1
        self.k += 1
        self.send('MotionPlanner', 'ArmState', dict(t=t, q=d.qpos[self.qadr].copy(), dq=d.qvel[self.dof].copy(),
                                                    f=d.qpos[self.fq].copy(), F=d.sensordata[self.fs:self.fs + 3].copy(),
                                                    T=d.sensordata[self.ts:self.ts + 3].copy(), mode=self.mode))
        if self.k % 50 == 0:
            self.send('TaskSupervisor', 'ArmStatus', dict(mode=self.mode, tau_ratio_max=round(self.stats['tau_ratio_max'], 3)))


def apply_brake(m, d, ba, wq, lock, lvl, bt, kp, kv):
    """Spring brake = wheel-locking position hold with a holding-torque limit (`lvl` 0..1 = engagement). The lock angle
    is latched while engaged; when the torque limit is reached the brake slips and the lock is re-latched so that the
    brake acts as Coulomb friction (no spring-back). Released: gains zeroed (no drag)."""
    q = d.qpos[wq]
    if lvl <= 0:
        m.actuator_gainprm[ba, 0] = 0.
        m.actuator_biasprm[ba, 1] = 0.
        m.actuator_biasprm[ba, 2] = 0.
        d.ctrl[ba] = q
        return q.copy()
    m.actuator_gainprm[ba, 0] = kp
    m.actuator_biasprm[ba, 1] = -kp
    m.actuator_biasprm[ba, 2] = -kv
    lim = lvl * bt
    m.actuator_forcerange[ba] = [[-lim, lim]] * len(ba)
    f = d.actuator_force[ba]
    slip = np.abs(f) >= lim - 1e-6
    lock = np.where(slip, q + np.sign(f) * lim / kp * .999, lock)
    d.ctrl[ba] = lock
    return lock


# ====================================================================== base MCU + wheel drivers + BMS
class BaseMCU(Node):
    name, rate = 'BaseMCU', 500
    KP, KI = 15., 150.
    BRAKE_ENGAGE_T, BRAKE_RELEASE_T = .030, .020       # EST: spring-applied brake engage / release times
    WATCHDOG = .10

    def __init__(self, sysm, bs):
        super().__init__(sysm)
        m = self.m
        self.bs = bs
        self.r, self.b2 = bs['wheel_radius_m'], bs['track_width_m'] / 2
        self.wq = [m.joint('wheel_l').qposadr[0], m.joint('wheel_r').qposadr[0]]
        self.wd = [m.joint('wheel_l').dofadr[0], m.joint('wheel_r').dofadr[0]]
        self.ma = [m.actuator('motor_l').id, m.actuator('motor_r').id]
        self.ba = [m.actuator('brake_l').id, m.actuator('brake_r').id]
        self.bkp = m.actuator_gainprm[self.ba[0], 0]
        self.bkv = -m.actuator_biasprm[self.ba[0], 2]
        self.lock = apply_brake(m, self.d, self.ba, [m.joint('wheel_l').qposadr[0], m.joint('wheel_r').qposadr[0]],
                                self.d.qpos[[m.joint('wheel_l').qposadr[0], m.joint('wheel_r').qposadr[0]]].copy(), 1.,
                                self.bs['brake_holding_torque_Nm_per_wheel'], self.bkp, self.bkv)
        self.bt = self.bs['brake_holding_torque_Nm_per_wheel']
        self.lock = self.d.qpos[self.wq].copy()
        self.integ = np.zeros(2)
        self.v_ref = self.w_ref = 0.
        self.v_tgt = self.w_tgt = 0.
        self.t_cmd = -1.
        self.brake_cmd = True                # power-off brakes: engaged at boot
        self.brake_level = 1.
        self.estop = False
        self.enc_prev = self.d.qpos[self.wq].copy()
        self.odom = np.zeros(3)
        self.rms = np.zeros(2)
        self.stats = dict(tau_peak=0., tau_rms10_max=0., derate_s=0., watchdog_trips=0)
        self.rng = np.random.default_rng(11)
        self.k = 0
        self.charge_F = self.dock_F = 0.

    def tick(self, t):
        m, d = self.m, self.d
        for msg in self.recv():
            if msg.typ == 'BaseCmd':
                self.v_tgt, self.w_tgt, self.t_cmd = msg.data['v'], msg.data['w'], t
            elif msg.typ == 'Brake':
                self.brake_cmd = bool(msg.data['engage'])
                if self.brake_cmd:
                    self.v_tgt = self.w_tgt = 0.
            elif msg.typ == 'EStopButton':
                if msg.data['pressed'] and not self.estop:
                    self.estop = True
                    self.t_estop = t
                    for dst in ('ArmController', 'MotionPlanner', 'TaskSupervisor'):
                        self.send(dst, 'EStop', dict(source='button'), info=dict(source='button'))
                elif not msg.data['pressed'] and self.estop:
                    self.estop = False
                    self.send('TaskSupervisor', 'EStopReleased', {}, info={})
        # watchdog: no twist command for 100 ms -> zero speed (spec: + brake)
        if t - self.t_cmd > self.WATCHDOG and (self.v_tgt or self.w_tgt):
            self.v_tgt = self.w_tgt = 0.
            self.stats['watchdog_trips'] += 1
        brake = self.brake_cmd or self.estop
        # accel / yaw-rate limits
        amax, vmax, wmax = self.bs['max_accel_m_s2'], self.bs['max_speed_m_s'], self.bs['max_yaw_rate_rad_s']
        self.v_ref += np.clip(np.clip(self.v_tgt, -vmax, vmax) - self.v_ref, -amax * DT, amax * DT)
        self.w_ref += np.clip(np.clip(self.w_tgt, -wmax, wmax) - self.w_ref, -1.5 * DT, 1.5 * DT)
        # brakes (dry friction on the wheel joints), ramped engage / release
        if brake:
            self.brake_level = min(1., self.brake_level + DT / self.BRAKE_ENGAGE_T)
        else:
            self.brake_level = max(0., self.brake_level - DT / self.BRAKE_RELEASE_T)
        self.set_brake(self.brake_level)
        wv = d.qvel[self.wd]
        if brake or self.brake_level > 0:
            self.integ[:] = 0
            self.v_ref = self.w_ref = 0.
            tau = np.zeros(2)
        else:
            ref = np.array([(self.v_ref - self.w_ref * self.b2) / self.r, (self.v_ref + self.w_ref * self.b2) / self.r])
            e = ref - wv
            self.integ = np.clip(self.integ + e * DT, -3, 3)
            tq = self.bs['motor_torque_peak_Nm_per_wheel']
            tau = np.clip(self.KP * e + self.KI * self.integ, -tq, tq)
            # thermal derating: 10 s RMS above the continuous rating -> clamp to continuous
            if np.any(np.sqrt(self.rms) > self.bs['motor_torque_continuous_Nm_per_wheel']):
                c = self.bs['motor_torque_continuous_Nm_per_wheel']
                tau = np.clip(tau, -c, c)
                self.stats['derate_s'] += DT
        self.rms += DT / 10. * (tau ** 2 - self.rms)
        self.stats['tau_peak'] = max(self.stats['tau_peak'], float(np.max(np.abs(tau))))
        self.stats['tau_rms10_max'] = max(self.stats['tau_rms10_max'], float(np.sqrt(np.max(self.rms))))
        d.ctrl[self.ma] = tau
        # odometry (wheel encoders)
        enc = d.qpos[self.wq].copy()
        dl, dr = (enc - self.enc_prev) * self.r
        self.enc_prev = enc
        ds, dth = (dl + dr) / 2, (dr - dl) / (2 * self.b2)
        th = self.odom[2] + dth / 2
        self.odom += [ds * math.cos(th), ds * math.sin(th), dth]
        self.k += 1
        if self.k % 5 == 0:
            q = d.sensor('base_quat').data
            roll = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)) + self.rng.normal(0, math.radians(.02))
            pitch = math.asin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)) + self.rng.normal(0, math.radians(.02))
            self.dock_F = SM.DOCK_K * float(d.sensor('dock_plate').data[0]) + self.rng.normal(0, 1.)
            self.charge_F = SM.CHARGE_K * float(d.sensor('charge_contact').data[0])
            st = dict(odom=self.odom.copy(), v=float((wv[0] + wv[1]) / 2 * self.r), w=float((wv[1] - wv[0]) * self.r / (2 * self.b2)),
                      roll=roll, pitch=pitch, dock_F=self.dock_F, charge_F=self.charge_F, brake=self.brake_level >= 1.,
                      brake_level=self.brake_level, estop=self.estop, soc=self.sys.battery.soc, V=self.sys.battery.V,
                      I=self.sys.battery.I, charging=self.sys.charging)
            self.send('MotionPlanner', 'BaseState', st)
        if self.k % 50 == 0:
            b = self.sys.battery
            self.send('TaskSupervisor', 'BMS', dict(soc=b.soc, V=b.V, I=b.I, charging=self.sys.charging, estop=self.estop,
                                                   brake=self.brake_level >= 1.))

    def set_brake(self, lvl):
        self.lock = apply_brake(self.m, self.d, self.ba, self.wq, self.lock, lvl, self.bt, self.bkp, self.bkv)

    @property
    def brakes_released(self):
        return 2 * (1 - self.brake_level >= .999)


# ====================================================================== dishwasher's own MCU
class DishwasherPanel(Node):
    """Capacitive touch panel emulation. A pad registers a touch only when a CONDUCTIVE geom (the robot's stylus)
    presses it with >= 0.5 N for >= 80 ms. Panel logic (chosen; the real E804 manual is not available - plausible
    6-pad sequence without a START pad): OFF --POWER--> STANDBY (power LED on, program LEDs dim) --program pad-->
    SELECTED(p) (that LED steady) --POWER (short touch, door closed, tank ok)--> RUNNING(p). POWER with the door open:
    3 beeps, door LED blinks, stays SELECTED. POWER held >= 2 s: OFF. STANDBY/SELECTED auto-off after 120 s idle."""
    name, rate = 'DishwasherPanel', 500
    F_MIN, DWELL, RELEASE, LONG = .5, .080, .100, 2.0

    def __init__(self, sysm, conductive=SM.CONDUCTIVE_GEOMS):
        super().__init__(sysm)
        m = self.m
        self.pads = {m.geom(f'dw/btn_{b}_pad').id: b for b in dw.BUTTONS}
        self.cond = {m.geom(g).id for g in conductive}
        self.state, self.program = 'OFF', None
        self.t_touch = {b: 0. for b in dw.BUTTONS}
        self.t_off = {b: 1. for b in dw.BUTTONS}
        self.latched = {b: False for b in dw.BUTTONS}
        self.t_idle = 0.
        self.events = []
        self.beeps = 0
        self.door_led = False
        self.tank_ok = True
        self.raw = {b: dict(cond_peak=0., noncond_peak=0., noncond_dwell=0.) for b in dw.BUTTONS}
        self.k = 0
        self.f6 = np.zeros(6)
        self.trace = []

    def leds(self):
        on = self.state != 'OFF'
        return dict(power=on, door=self.door_led, **{b: ('steady' if self.program == b else ('dim' if on else 'off'))
                                                     for b in dw.BUTTONS if b != 'power'})

    def _event(self, t, what, **kw):
        e = dict(t=round(t, 3), event=what, state=self.state, program=self.program, **kw)
        self.events.append(e)
        self.send('Perception', 'PanelEvent', e, info=dict(event=what, state=self.state))

    def tick(self, t):
        m, d = self.m, self.d
        f_cond = {b: 0. for b in dw.BUTTONS}
        f_non = {b: 0. for b in dw.BUTTONS}
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom[0], c.geom[1]
            pad = self.pads.get(g1, self.pads.get(g2))
            if pad is None:
                continue
            other = g2 if g1 in self.pads else g1
            mujoco.mj_contactForce(m, d, i, self.f6)
            if other in self.cond:
                f_cond[pad] += self.f6[0]
            else:
                f_non[pad] += self.f6[0]
        door_closed = dw.door_closed(m, d)
        for b in dw.BUTTONS:
            if f_cond[b] > 0 or f_non[b] > 0:
                self.trace.append((round(t, 3), b, round(f_cond[b], 2), round(f_non[b], 2), round(self.t_touch[b], 3), self.latched[b]))
            r = self.raw[b]
            r['cond_peak'] = max(r['cond_peak'], f_cond[b])
            r['noncond_peak'] = max(r['noncond_peak'], f_non[b])
            if f_non[b] >= self.F_MIN:
                r['noncond_dwell'] += DT
            if f_cond[b] >= self.F_MIN:
                self.t_touch[b] += DT
                self.t_off[b] = 0.
                if not self.latched[b] and self.t_touch[b] >= self.DWELL:
                    self.latched[b] = True
                    self._press(t, b, door_closed)
                if b == 'power' and self.latched[b] and self.t_touch[b] >= self.LONG and self.state != 'OFF':
                    self.state, self.program = 'OFF', None
                    self._event(t, 'long_press_off')
            else:
                self.t_off[b] += DT
                if self.t_off[b] >= self.RELEASE:
                    self.t_touch[b] = 0.
                    self.latched[b] = False
        if self.state in ('STANDBY', 'SELECTED'):
            self.t_idle += DT
            if self.t_idle > 120.:
                self.state, self.program = 'OFF', None
                self._event(t, 'auto_off')
        if door_closed:
            self.door_led = False
        self.k += 1
        if self.k % 50 == 0:
            self.send('Perception', 'PanelLEDs', dict(state=self.state, program=self.program, leds=self.leds()))

    def _press(self, t, b, door_closed):
        self.t_idle = 0.
        self.beeps += 1
        if self.state == 'OFF':
            if b == 'power':
                self.state = 'STANDBY'
            self._event(t, f'touch_{b}')
        elif self.state == 'STANDBY':
            if b == 'power':
                self._event(t, 'touch_power_ignored_in_standby')
            else:
                self.state, self.program = 'SELECTED', b
                self._event(t, f'touch_{b}')
        elif self.state == 'SELECTED':
            if b == 'power':
                if door_closed and self.tank_ok:
                    self.state = 'RUNNING'
                    self._event(t, 'start')
                else:
                    self.beeps += 2
                    self.door_led = not door_closed
                    self._event(t, 'start_refused', door_closed=bool(door_closed), tank_ok=self.tank_ok)
            else:
                self.program = b
                self._event(t, f'touch_{b}')
        else:
            self._event(t, f'touch_{b}_ignored')

    @property
    def label(self):
        return self.state + (f' [{self.program.upper()}]' if self.program else '')


# ====================================================================== perception computer (lidars, camera + Codex, LED readout)
class Perception(Node):
    name, rate = 'Perception', 10
    CODEX_TRUTH_DELAY = 15.8                         # s, mean real Codex latency in ../full_pipeline (truth mode only)

    def __init__(self, sysm, codex=True, outdir=None, run_tag='run', rng=None):
        super().__init__(sysm)
        self.lidars = [LI.Lidar(self.m, 'front'), LI.Lidar(self.m, 'rear')]
        self.rng = rng or np.random.default_rng(5)
        self.codex = codex
        self.outdir = Path(outdir)
        self.run_tag = run_tag
        self.cam = None
        self.pending = None
        self.panel = None
        self.last_readout = None
        self.k = 0
        self.lidar_on = True
        self.codex_log = []

    def tick(self, t):
        for msg in self.recv():
            if msg.typ == 'PerceiveRequest':
                self._start(t, msg.data)
            elif msg.typ == 'PanelLEDs':
                self.panel = msg.data
                ro = (msg.data['state'], msg.data['program'], msg.data['leds']['door'])
                if ro != self.last_readout or self.k % 10 == 0:
                    self.send('TaskSupervisor', 'PanelReadout', msg.data,
                              info=dict(state=msg.data['state'], program=msg.data['program']) if ro != self.last_readout else None)
                    self.last_readout = ro
            elif msg.typ == 'PanelEvent':
                pass
            elif msg.typ == 'LidarPower':
                self.lidar_on = bool(msg.data['on'])
        self.k += 1
        if self.lidar_on:
            for L in self.lidars:
                r, strip = L.scan(self.d, self.rng)
                f = LI.localize(r, strip, L.mount, L.ang, self.rng)
                f['lidar'] = L.which
                self.send('MotionPlanner', 'LidarFix', f)
        if self.pending is not None:
            self._poll(t)

    # ---------------- wall camera + Codex, asynchronous; the simulation keeps running (arm parked) while waiting.
    # Sim time is throttled to wall time during a Codex call, so the result arrives after its REAL latency.
    def _start(self, t, req):
        if self.cam is None:
            self.cam = PC.Camera(self.m)
        rgb, depth, _ = self.cam.capture(self.d, self.rng)
        tag = f'{self.run_tag}_{req["tag"]}'
        p = dict(req=req, t0=t, w0=time.monotonic(), rgb=rgb, depth=depth, tag=tag, proc=None, out=None)
        if self.codex:
            img, out = self.outdir / f'{tag}.png', self.outdir / f'{tag}.json'
            self.outdir.mkdir(parents=True, exist_ok=True)
            Image.fromarray(rgb).save(img)
            out.unlink(missing_ok=True)
            if not PC.SCHEMA.exists():
                PC.write_schema()
            p['out'] = out
            p['proc'] = subprocess.Popen(['codex', 'exec', '--skip-git-repo-check', '--sandbox', 'read-only', '--output-schema',
                                          str(PC.SCHEMA), '-o', str(out), f'--image={img}', PC.PROMPTS[req['mode']].format(w=PC.W, h=PC.H)],
                                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.pending = p
        self.sys.rec.speed = 4                            # video fast-forward while the robot waits
        self.sys.status_note = f'waiting for {"Codex" if self.codex else "DEBUG truth perception"} ({req["tag"]})'

    def _poll(self, t):
        p = self.pending
        elapsed_sim = t - p['t0']
        if self.codex:
            done = p['proc'].poll() is not None
            if not done:
                wall = time.monotonic() - p['w0']
                if elapsed_sim > wall:                   # keep sim time <= wall time during the call
                    while p['proc'].poll() is None and time.monotonic() - p['w0'] < elapsed_sim:
                        time.sleep(.01)
                    done = p['proc'].poll() is not None
                if not done:
                    if time.monotonic() - p['w0'] > 300:
                        p['proc'].kill()
                    return
            if 'latency' not in p:
                p['latency'] = round(time.monotonic() - p['w0'], 1)
            if elapsed_sim < p['latency']:
                return                                   # result exists but its real latency has not elapsed in sim yet
            if p['out'].exists():
                import json
                vlm = json.loads(p['out'].read_text())
            else:
                vlm = dict(found=False, error=p['proc'].stderr.read().decode()[-300:] if p['proc'].stderr else '')
            vlm['seconds'] = p['latency']
        else:
            if elapsed_sim < self.CODEX_TRUTH_DELAY:
                return
            vlm = dict(found=True, debug_truth=True, seconds=self.CODEX_TRUTH_DELAY)
        self.pending = None
        self.sys.rec.speed = 1
        self.sys.status_note = ''
        self.codex_log.append(dict(tag=p['tag'], seconds=vlm.get('seconds'), found=vlm.get('found')))
        self.send('MotionPlanner', 'PerceiveResult', dict(req=p['req'], vlm=vlm, rgb=p['rgb'], depth=p['depth'], t0=p['t0']),
                  info=dict(tag=p['req']['tag'], codex_s=vlm.get('seconds'), found=vlm.get('found')))

    def close(self):
        if self.cam is not None:
            self.cam.close()
        if self.pending is not None and self.pending.get('proc') is not None:
            self.pending['proc'].kill()


# ====================================================================== operator / environment (scripted)
class Operator(Node):
    """Scripted human: presses the e-stop when the robot is in `estop_phase` (+ delay), releases it after `hold` s,
    acknowledges the resume (or aborts) `ack_after` s later."""
    name, rate = 'Operator', 10

    def __init__(self, sysm, estop_phase=None, delay=.5, hold=3., ack_after=1.5, action='resume'):
        super().__init__(sysm)
        self.estop_phase, self.delay, self.hold, self.ack_after, self.action = estop_phase, delay, hold, ack_after, action
        self.t_seen = self.t_press = self.t_release = None
        self.done = False
        self.log = []

    def tick(self, t):
        if self.estop_phase is None or self.done:
            return
        ph = getattr(self.sys.planner, 'phase', '')
        if self.t_seen is None and ph == self.estop_phase:
            self.t_seen = t
        if self.t_seen is not None and self.t_press is None and t >= self.t_seen + self.delay:
            self.t_press = t
            self.send('BaseMCU', 'EStopButton', dict(pressed=True), info=dict(pressed=True), lat=.001)
            self.log.append(dict(t=round(t, 3), event='estop_pressed', phase=ph))
        if self.t_press is not None and self.t_release is None and t >= self.t_press + self.hold:
            self.t_release = t
            self.send('BaseMCU', 'EStopButton', dict(pressed=False), info=dict(pressed=False), lat=.001)
            self.log.append(dict(t=round(t, 3), event='estop_released'))
        if self.t_release is not None and t >= self.t_release + self.ack_after:
            self.send('TaskSupervisor', 'OperatorAck', dict(action=self.action), info=dict(action=self.action), lat=.001)
            self.log.append(dict(t=round(t, 3), event=f'operator_{self.action}'))
            self.done = True
