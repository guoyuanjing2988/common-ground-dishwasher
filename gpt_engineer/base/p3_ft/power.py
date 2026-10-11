"""Electrical power and battery model.

Physically simulated inputs (from MuJoCo, every 2 ms step): joint torques and joint speeds of the 7 arm joints, the
gripper and the two wheel motors (tau = servo/motor actuator force + the ArmController's gravity torque).
Modelled (EST, stated assumptions):
  * motor + drive electrical input per joint:  P = max(tau*omega, 0) / eta  +  (tau / k_m)^2
      - eta = 0.80 (arm joints: harmonic drive + motor + inverter), 0.85 (wheel hub motors)
      - (tau/k_m)^2 = copper loss; k_m = motor constant referred to the joint output, N m / sqrt(W):
        arm joints 1-4: 8.7 (87 N m continuous at ~100 W copper loss), joints 5-7: 2.7 (12 N m at ~20 W),
        hub motors: 1.9 (15 N m continuous at ~60 W). Negative mechanical power (braking) is NOT regenerated: it is
        dumped in the drive's brake resistor, so it costs only the copper loss.
  * fixed loads from spec.json v1.0 `power.components` (active values while awake): main PC 30 W, 2 lidars 5 W, GbE
    switch 4 W, base MCU 1 W, PILZ safety relay 2.5 W, Bota F/T 1 W (24 V rail through a DC/DC, eta 0.92); 2 wheel
    drivers standby 8 W and the 2 brake coils 12 W while RELEASED (48 V bus direct). The FR3 is an AC load through the
    48 V -> 230 V inverter (eta 0.91, 9.5 W no-load): arm electrical = (45 W control electronics, EST, + the physics-based
    joint power above) / 0.91 (the spec's datasheet figure is ~60 W idle / ~80 W active for the whole arm).
  * battery (spec): 16S LiFePO4, 51.2 V nominal, 30 Ah, OCV curve per cell (typical LFP table), internal resistance
    60 mOhm (pack, EST), BMS >= 60 A continuous. SoC by coulomb counting. Charger 11.3 A when on the contacts.
Not modelled: motor electrical dynamics (inductance, PWM), temperature, ageing, cell balancing, charger CV phase.
"""
from __future__ import annotations

import math

import numpy as np

OCV_CELL = np.array([(0., 2.50), (.05, 3.00), (.10, 3.15), (.20, 3.22), (.30, 3.26), (.50, 3.29), (.70, 3.31),
                     (.90, 3.34), (.97, 3.40), (1., 3.55)])
ARM_KM = np.array([8.7, 8.7, 8.7, 8.7, 2.7, 2.7, 2.7])
ARM_ETA, WHEEL_ETA, WHEEL_KM = .80, .80, 5.0           # wheel: BLV-R motor + 50:1 gearhead; k_m at the wheel (EST)
FIXED_24V = dict(main_pc=30., lidars=5., gbe_switch=4., base_mcu=1., safety_relay=2.5, ft_sensor=1.)
FIXED_48V = dict(wheel_drivers_standby=8.)
ARM_ELECTRONICS_W, INV_ETA, INV_NOLOAD_W = 45., .91, 9.5
BRAKE_RELEASE_W = 6.                                   # per brake coil (spec: 12 W for both)
DCDC_ETA = .92


class Battery:
    def __init__(self, capacity_Ah=30., nominal_V=51.2, soc=.90, R=.060, I_cont=60., I_peak=120.):
        self.cells = int(round(nominal_V / 3.2))
        self.cap = capacity_Ah
        self.soc = soc
        self.soc0 = soc
        self.R = R
        self.I_cont, self.I_peak = I_cont, I_peak
        self.I = 0.
        self.V = self.ocv()
        self.E_out_Wh = 0.          # at the terminals
        self.Ah_out = 0.
        self.I_peak_seen = 0.
        self.I_over_cont_s = 0.
        self.I_filt = 0.            # 10 s RMS-ish window for the continuous limit

    def ocv(self, soc=None):
        s = self.soc if soc is None else soc
        return self.cells * float(np.interp(min(max(s, 0.), 1.), OCV_CELL[:, 0], OCV_CELL[:, 1]))

    def step(self, P, dt):
        """P = power drawn at the terminals (W, > 0 discharge, < 0 charge)."""
        V0 = self.ocv()
        disc = V0 * V0 - 4 * self.R * P
        I = (V0 - math.sqrt(max(disc, 0.))) / (2 * self.R)
        self.I = I
        self.V = V0 - I * self.R
        self.soc -= I * dt / 3600. / self.cap
        self.Ah_out += I * dt / 3600.
        self.E_out_Wh += P * dt / 3600.
        self.I_peak_seen = max(self.I_peak_seen, I)
        self.I_filt += (dt / 10.) * (I * I - self.I_filt)
        if math.sqrt(max(self.I_filt, 0.)) > self.I_cont:
            self.I_over_cont_s += dt
        return I

    def state(self):
        return dict(soc=round(self.soc, 5), V=round(self.V, 2), I=round(self.I, 2), I_peak=round(self.I_peak_seen, 1),
                    E_Wh=round(self.E_out_Wh, 3))


class PowerModel:
    def __init__(self, m, spec_power=None):
        self.m = m
        self.arm_act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 8)]
        self.arm_dof = [m.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)]
        self.grip_act = m.actuator('panda/actuator8').id
        self.f_dof = [m.joint(f'panda/finger_joint{i}').dofadr[0] for i in (1, 2)]
        self.w_act = [m.actuator('motor_l').id, m.actuator('motor_r').id]
        self.w_dof = [m.joint('wheel_l').dofadr[0], m.joint('wheel_r').dofadr[0]]
        self.fixed24 = dict(FIXED_24V)
        self.fixed48 = dict(FIXED_48V)
        for c in (spec_power or {}).get('components', []):            # adopt the spec's active numbers when present
            nm = c.get('load', '').lower()
            for key, pat in (('main_pc', 'main pc'), ('lidars', 'lidar'), ('gbe_switch', 'switch'), ('base_mcu', 'base mcu'),
                             ('safety_relay', 'pilz'), ('ft_sensor', 'f/t')):
                if pat in nm:
                    self.fixed24[key] = float(c.get('active_W', self.fixed24[key]))
            if 'drivers' in nm and 'standby' in nm:
                self.fixed48['wheel_drivers_standby'] = float(c.get('active_W', 8.))
        self.E = dict(arm_mech_pos=0., arm_elec=0., wheel_mech_pos=0., wheel_elec=0., fixed=0., brakes=0., total=0.)
        self.P_peak = 0.
        self.arm_on = True

    def power(self, d, brakes_released, lidars_on=True):
        tau = d.actuator_force[self.arm_act] + d.qfrc_applied[self.arm_dof]
        w = d.qvel[self.arm_dof]
        pm = tau * w
        joints = float(np.sum(np.maximum(pm, 0.) / ARM_ETA + (tau / ARM_KM) ** 2))
        joints += abs(float(d.actuator_force[self.grip_act] * np.mean(d.qvel[self.f_dof]))) / .5
        arm = (ARM_ELECTRONICS_W + joints) / INV_ETA + INV_NOLOAD_W if self.arm_on else 0.
        tw = d.actuator_force[self.w_act]
        ww = d.qvel[self.w_dof]
        pw = tw * ww
        wheel = float(np.sum(np.maximum(pw, 0.) / WHEEL_ETA + (tw / WHEEL_KM) ** 2))
        f24 = sum(v for k, v in self.fixed24.items() if lidars_on or k != 'lidars') + (0. if lidars_on else .5)
        fixed = f24 / DCDC_ETA + sum(self.fixed48.values())
        brakes = BRAKE_RELEASE_W * brakes_released
        return dict(arm=arm, arm_mech=float(np.sum(np.maximum(pm, 0.))), wheel=wheel, wheel_mech=float(np.sum(np.maximum(pw, 0.))),
                    fixed=fixed, brakes=brakes, total=arm + wheel + fixed + brakes)

    def accumulate(self, P, dt):
        h = dt / 3600.
        self.E['arm_mech_pos'] += P['arm_mech'] * h
        self.E['arm_elec'] += P['arm'] * h
        self.E['wheel_mech_pos'] += P['wheel_mech'] * h
        self.E['wheel_elec'] += P['wheel'] * h
        self.E['fixed'] += P['fixed'] * h
        self.E['brakes'] += P['brakes'] * h
        self.E['total'] += P['total'] * h
        self.P_peak = max(self.P_peak, P['total'])
