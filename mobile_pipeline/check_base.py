"""Development check of the wheeled base physics (spec v1.0): rest loads, V-block seating, braking."""
import math
import sys

import mujoco
import numpy as np

import scene_mobile as SM


class Rig:
    def __init__(self, start, vblocks=True, base=None):
        self.m, self.info = SM.build(SM.MobileConfig(start=start, vblocks=vblocks, base=base), with_objects=False)
        m = self.m
        self.d = mujoco.MjData(m)
        aq = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.adof = [m.joint(f'panda/joint{i}').dofadr[0] for i in range(1, 8)] + [m.joint(f'panda/finger_joint{i}').dofadr[0] for i in (1, 2)]
        self.pb = [b for b in range(m.nbody) if m.body(b).name.startswith('panda/')]
        acts = [m.actuator(f'panda/actuator{i}').id for i in range(1, 8)]
        self.d.qpos[aq] = SM.Q_STOW
        self.d.ctrl[acts] = SM.Q_STOW
        self.d.ctrl[m.actuator('panda/actuator8').id] = 255
        mujoco.mj_forward(m, self.d)
        self.g = np.zeros(m.nv)
        self.wd = [m.joint('wheel_l').dofadr[0], m.joint('wheel_r').dofadr[0]]
        self.ma = [m.actuator('motor_l').id, m.actuator('motor_r').id]
        self.bs = self.info['base_spec']
        self.integ = np.zeros(2)
        self.names = ['wheel_l_tyre', 'wheel_r_tyre'] + [f'caster{i}_ball' for i in range(4)]
        self.gid = {m.geom(n).id: n for n in self.names}
        self.bid = m.body('base').id
        self.fd = m.joint('base_free').dofadr[0]
        self.ba = [m.actuator('brake_l').id, m.actuator('brake_r').id]
        self.wq = [m.joint('wheel_l').qposadr[0], m.joint('wheel_r').qposadr[0]]
        self.kp, self.kv = SM.BRAKE_KP, SM.BRAKE_KV
        self.lock = self.d.qpos[self.wq].copy()

    def brake(self, lvl):
        from nodes import apply_brake
        self.lock = apply_brake(self.m, self.d, self.ba, self.wq, self.lock, lvl, self.bs['brake_holding_torque_Nm_per_wheel'],
                                self.kp, self.kv)

    def loads(self):
        f6 = np.zeros(6)
        res = {n: 0. for n in self.names}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for gg in c.geom:
                if gg in self.gid:
                    mujoco.mj_contactForce(self.m, self.d, i, f6)
                    res[self.gid[gg]] += f6[0]
        return res

    def step(self, v=0., w=0., brake=False):
        m, d = self.m, self.d
        self.g[:] = 0
        for b in self.pb:
            mujoco.mj_applyFT(m, d, -m.body_mass[b] * m.opt.gravity, np.zeros(3), d.xipos[b], b, self.g)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[self.adof] = self.g[self.adof]
        r, b2 = self.bs['wheel_radius_m'], self.bs['track_width_m'] / 2
        if brake:
            self.brake(1.)
            d.ctrl[self.ma] = 0
            self.integ[:] = 0
        else:
            self.brake(0.)
            e = np.array([(v - w * b2) / r, (v + w * b2) / r]) - d.qvel[self.wd]
            self.integ[:] = np.clip(self.integ + e * m.opt.timestep, -3, 3)
            tq = self.bs['motor_torque_peak_Nm_per_wheel']
            d.ctrl[self.ma] = np.clip(15. * e + 150. * self.integ, -tq, tq)
        mujoco.mj_step(m, d)

    def speed(self):
        return float(np.linalg.norm(self.d.qvel[self.fd:self.fd + 2]))


def rest_and_dock():
    dx, dy, dyaw = SM.dock_base_pose()
    R = Rig((dx + .03, dy + .006, dyaw + 1.0))
    for _ in range(500):
        R.step(brake=True)
    L = R.loads()
    W = R.m.body_subtreemass[R.bid] * 9.81
    print('rest loads N', {k: round(v, 1) for k, v in L.items()}, 'sum', round(sum(L.values()), 1), 'weight', round(W, 1),
          'drive fraction', round((L['wheel_l_tyre'] + L['wheel_r_tyre']) / W, 3))
    # reverse at 0.05 m/s into the V-blocks until dock plate force >= 100 N, then brake
    F = 0
    for k in range(4000):
        R.step(v=-.05)
        F = SM.DOCK_K * R.d.sensor('dock_plate').data[0]
        if F >= 100:
            break
    for _ in range(500):
        R.step(brake=True)
    q = R.d.xquat[R.bid]
    yaw = math.degrees(2 * math.atan2(q[3], q[0]))
    p = R.d.xpos[R.bid]
    print('docked after', round(k * .002, 2), 's: base err mm', np.round((p[:2] - [dx, dy]) * 1000, 2), 'yaw err deg',
          round(yaw - dyaw, 3), 'plate force N', round(SM.DOCK_K * R.d.sensor('dock_plate').data[0], 1),
          'arm base', SM.arm_base_pose(p[:2], yaw))


def braking(base=None, label='v1.0', direction=1.):
    R = Rig((1.5, -2.5, 90.), vblocks=False, base=base)
    for _ in range(300):
        R.step(brake=True)
    vmax, amax = R.bs['max_speed_m_s'], R.bs['max_accel_m_s2']
    v = 0.
    while abs(v) < vmax - 1e-9:
        v = direction * min(vmax, abs(v) + amax * .002)
        R.step(v=v)
    for _ in range(500):
        R.step(v=v)
    p0, v0 = R.d.xpos[R.bid].copy(), R.speed()
    minl = {n: 1e9 for n in R.names}
    t = 0.
    while R.speed() > 1e-3 or t < .3:
        R.step(brake=True)
        t += .002
        for n, f in R.loads().items():
            minl[n] = min(minl[n], f)
    print(f'[{label}] {"fwd" if direction > 0 else "rev"} e-stop from {v0:.3f} m/s: stop {np.linalg.norm(R.d.xpos[R.bid][:2] - p0[:2]) * 1000:.1f} mm '
          f'in {t:.2f} s; min normal N', {k: round(v_, 1) for k, v_ in minl.items()})


if __name__ == '__main__':
    rest_and_dock()
    for dirn in (1., -1.):
        braking(direction=dirn)
        braking(base=SM.PROVISIONAL, label='provisional', direction=dirn)
