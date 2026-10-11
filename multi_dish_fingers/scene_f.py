"""Scene: counter + countertop dishwasher (door open, basket out), M1 docked cart + Panda with CUSTOM fingers,
a pile of 3 stacked 8" plates and a pile of 2 nested bowls on the counter right of the machine.
Kitchen / cart / Panda attach copied from ../full_pipeline/scene.py; plate from ../poc_plate_flat; bowl from
../poc_bowl_grasp. World: counter front edge x = 0, +x into the room, z up. Counter top z = 0.90."""
from __future__ import annotations

import math
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

import mujoco
import numpy as np

import dishwasher as dw
import fingers as FG

PANDA = str(_ROOT / 'mujoco_menagerie/franka_emika_panda/panda.xml')
COUNTER_Z, COUNTER_D = 0.90, 0.60
DW_POS = (-dw.D / 2, 0.0, COUNTER_Z)
MOUNT = dict(x=0.45, y=0.30, z=0.90, yaw=180.)       # M1 docked cart (full_pipeline MOUNTING.md)

# plate (../poc_plate_flat): local z = normal, foot-ring bottom at z = 0
R_OUT, R_DISC, R_FOOT, PL_T, RIM_RISE = .1015, .065, .060, .005, .0098
SLOPE = math.atan2(RIM_RISE, R_OUT - R_DISC)
MID_Z = .0105
PLATE_DZ = .010                                     # stacking pitch (foot ring on the disc below)
# bowl (../poc_bowl_grasp): local z = opening axis, foot-ring bottom at z = -0.008, rim top at z = 0.060
RIM_R, BOWL_H, FOOT_Z = .0575, .060, -.004
N_PLATES, N_BOWLS = 3, 2
PLATE_PILE = np.array([-0.125, 0.42])               # EST layout
BOWL_PILE = np.array([-0.120, 0.615])
BOWL_DZ0 = .0135
GRIP_X = .17                                        # squeeze target: 800 N/m * ~175 mm = 140 N tendon = 70 N jaw clamp (cap)                                    # initial nesting pitch; settles physically

OBJ = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])
ENV = dict(friction=[.6, .005, .0001], solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])


def quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


HEADER = '''<mujoco model="multi_dish_fingers">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" iterations="60" tolerance="1e-8"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048" offsamples="4"/>
    <headlight ambient=".38 .38 .38" diffuse=".45 .45 .45"/></visual>
  <asset><texture name="sky" type="skybox" builtin="gradient" rgb1=".93 .93 .92" rgb2=".62 .63 .65" width="256" height="256"/>
    <texture name="floortex" type="2d" builtin="checker" rgb1=".62 .58 .52" rgb2=".56 .52 .47" width="512" height="512"/>
    <material name="floormat" texture="floortex" texrepeat="8 8"/></asset>
  <worldbody>
    <light name="key" pos="1.6 1.2 2.6" dir="-.55 -.4 -.75" diffuse=".7 .7 .68" castshadow="true"/>
    <light name="fill" pos="-.2 -1.0 2.4" dir=".2 .5 -1" diffuse=".25 .25 .27" castshadow="false"/>
  </worldbody></mujoco>'''


def add_kitchen(spec):
    wb = spec.worldbody
    BOX = mujoco.mjtGeom.mjGEOM_BOX
    wb.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, .1], material='floormat', **ENV)
    wb.add_geom(name='counter_top', type=BOX, pos=[-COUNTER_D / 2, 0, COUNTER_Z - .02], size=[COUNTER_D / 2, 1.25, .02],
                rgba=[.27, .27, .29, 1], **ENV)
    wb.add_geom(name='cabinet', type=BOX, pos=[-.31, 0, .49], size=[.29, 1.24, .39], rgba=[.80, .78, .74, 1], **ENV)
    wb.add_geom(name='wall', type=BOX, pos=[-.615, 0, 1.35], size=[.015, 1.55, 1.35], rgba=[.93, .92, .89, 1],
                contype=0, conaffinity=0)


def add_robot(spec, L, grip_k=800., grip_limit=140.):
    wb = spec.worldbody
    BOX = mujoco.mjtGeom.mjGEOM_BOX
    cart = wb.add_body(name='cart')
    x0, x1, deck = -.02, .63, .32
    cart.add_geom(name='cart_chassis', type=BOX, pos=[(x0 + x1) / 2, .30, .19], size=[(x1 - x0) / 2, .30, .13],
                  rgba=[.30, .33, .38, 1], **ENV)
    h = MOUNT['z'] - deck
    cart.add_geom(name='column', type=BOX, pos=[MOUNT['x'], MOUNT['y'], deck + h / 2], size=[.09, .09, h / 2],
                  rgba=[.85, .86, .88, 1], **ENV)
    a = math.radians(MOUNT['yaw'])
    site = cart.add_site(name='robot_mount', pos=[MOUNT['x'], MOUNT['y'], MOUNT['z']],
                         quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)])
    spec.attach(mujoco.MjSpec.from_file(PANDA), prefix='panda/', site=site)
    FG.install(spec, L)
    spec.body('panda/hand').add_site(name='tcp', pos=[0, 0, FG.tcp_offset(L)], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    g = spec.actuator('panda/actuator8')            # force-limited gripper (PoC common ground, k = 800 N/m)
    g.gainprm[0] = grip_k * (.04 + GRIP_X) / 255     # ctrl 255 -> +0.04 open, ctrl 0 -> -GRIP_X (saturates at 70 N)
    g.biasprm[0], g.biasprm[1], g.biasprm[2] = -grip_k * GRIP_X, -grip_k, -10
    g.forcerange = [-grip_limit, grip_limit]
    for body in spec.bodies:
        if body.name.startswith('panda/'):
            body.gravcomp = 1


def add_plate(spec, name, pos, R):
    plate = spec.worldbody.add_body(name=name, pos=list(pos), quat=list(quat(R)))
    plate.add_freejoint(name=name + '_free')
    white = [.96, .96, .93, 1]
    plate.add_geom(name=name + '_disc', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .0075],
                   size=[R_DISC + .002, PL_T / 2, 0], mass=.15, rgba=white, **OBJ)
    n = 24
    width = R_OUT - R_DISC
    half_len = .5 * math.hypot(width, RIM_RISE)
    for i in range(n):
        a = 2 * math.pi * i / n
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.cos(SLOPE) * radial + np.array([0, 0, math.sin(SLOPE)])
        normal = np.cross(along, tangent)
        centre = (R_DISC + width / 2) * radial + np.array([0, 0, .0075 + RIM_RISE / 2])
        plate.add_geom(name=f'{name}_rim{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre),
                       quat=quat(np.column_stack([along, tangent, normal])),
                       size=[half_len, (R_DISC + width / 2) * math.sin(math.pi / n) * 1.15, PL_T / 2],
                       mass=.27 / n, rgba=white, **OBJ)
    m = 12
    for i in range(m):
        a = 2 * math.pi * i / m
        plate.add_geom(name=f'{name}_foot{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                       pos=[R_FOOT * math.cos(a), R_FOOT * math.sin(a), .0025], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
                       size=[.002, R_FOOT * math.sin(math.pi / m) * 1.1, .0025], mass=.03 / m, rgba=white, **OBJ)
    plate.add_site(name=name + '_c', pos=[0, 0, MID_Z], size=[.003, 0, 0], rgba=[1, .3, .1, 1], group=4)


def add_bowl(spec, name, pos, R, rgba):
    bowl = spec.worldbody.add_body(name=name, pos=list(pos), quat=list(quat(R)))
    bowl.add_freejoint(name=name + '_free')
    common = dict(OBJ, rgba=rgba)
    nseg = 20
    rim_apothem = RIM_R * math.cos(math.pi / nseg)
    slope = math.atan2(rim_apothem - .030, .056)
    for i in range(nseg):
        a = i * 2 * math.pi / nseg
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.sin(slope) * radial + np.array([0, 0, math.cos(slope)])
        normal = math.cos(slope) * radial - np.array([0, 0, math.sin(slope)])
        half_length = (.028 - .002 * math.sin(slope)) / math.cos(slope)
        centre_r = rim_apothem - .002 * math.cos(slope) - half_length * math.sin(slope)
        bowl.add_geom(name=f'{name}_wall{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre_r * radial + [0, 0, .032]),
                      quat=quat(np.column_stack([normal, tangent, along])),
                      size=[.002, RIM_R * math.sin(math.pi / nseg), half_length], mass=.120 / nseg, **common)
    bowl.add_geom(name=f'{name}_bottom', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .002], size=[.030, .002, 0],
                  mass=.045, **common)
    for i in range(12):
        a = i * 2 * math.pi / 12
        bowl.add_geom(name=f'{name}_foot{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                      pos=[(.025 * math.cos(math.pi / 12) - .002) * math.cos(a), (.025 * math.cos(math.pi / 12) - .002) * math.sin(a), FOOT_Z],
                      quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], size=[.002, .025 * math.sin(math.pi / 12), .004],
                      mass=.015 / 12, **common)
    bowl.add_site(name=name + '_foot', pos=[0, 0, FOOT_Z], size=[.002, 0, 0], group=4)
    bowl.add_site(name=name + '_rimc', pos=[0, 0, BOWL_H], size=[.002, 0, 0], group=4)


def build(L, objects=True):
    spec = mujoco.MjSpec.from_string(HEADER)
    add_kitchen(spec)
    info = dw.add_dishwasher(spec, pos=DW_POS, yaw=0.0, prefix='dw/')
    add_robot(spec, L)
    if objects:
        for i in range(N_PLATES):
            add_plate(spec, f'plate{i}', [PLATE_PILE[0], PLATE_PILE[1], COUNTER_Z + .0003 + i * PLATE_DZ], np.eye(3))
        cols = [[.55, .72, .86, 1], [.86, .62, .50, 1], [.70, .82, .60, 1]]
        for i in range(N_BOWLS):
            add_bowl(spec, f'bowl{i}', [BOWL_PILE[0], BOWL_PILE[1], COUNTER_Z + .0083 + i * BOWL_DZ0], np.eye(3), cols[i])
    model = spec.compile()
    return model, info


def build_ik(L):
    spec = mujoco.MjSpec.from_string(HEADER)
    add_robot(spec, L)
    return spec.compile()


if __name__ == '__main__':
    import sys
    L = float(sys.argv[1]) if len(sys.argv) > 1 else .107
    m, info = build(L)
    d = mujoco.MjData(m)
    dw.set_state(m, d, door_deg=90, basket=dw.BASKET_TRAVEL)
    for _ in range(1500):
        mujoco.mj_step(m, d)
    print('ngeom', m.ngeom, 'ncon', d.ncon, 'finger masses', [round(m.body(f'panda/{b}').mass[0] * 1000, 1) for b in ('left_finger', 'right_finger')])
    for i in range(N_PLATES):
        print('plate', i, np.round(d.body(f'plate{i}').xpos, 4))
    for i in range(N_BOWLS):
        print('bowl', i, np.round(d.body(f'bowl{i}').xpos, 4))
    print('slot', np.round(d.site('dw/plate_slot_target').xpos, 4), 'bowlslot', np.round(d.site('dw/bowl_slot_target').xpos, 4))
    for nm in ('panda/left_finger', 'panda/right_finger'):
        print(nm, np.round(d.body(nm).xpos, 4))
    print('basket', d.qpos[m.joint('dw/basket_slide').qposadr[0]], 'door', d.qpos[m.joint('dw/door_hinge').qposadr[0]])
