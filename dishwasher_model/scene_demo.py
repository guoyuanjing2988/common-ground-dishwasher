#!/usr/bin/env python3
"""Kitchen scene: floor, counter (top z=0.90, depth 0.60), dishwasher flush with the counter
front edge, Franka Panda attached via MjSpec.attach at a mount site beside the dishwasher.

World frame: counter front edge is the line x=0 (counter occupies -0.60 <= x <= 0), +x points
out of the counter into the room, z up. The dishwasher front faces +x (yaw 0).

    python scene_demo.py            # build, print summary, step 1 s with the arm holding 'home'
Import `build_scene(mount=...)` from other scripts (tools/reach_check.py, render.py).
"""
from __future__ import annotations

import math
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

import mujoco
import numpy as np

import dishwasher as dw

ROOT = Path(__file__).resolve().parent
PANDA = _ROOT / 'mujoco_menagerie/franka_emika_panda/panda.xml'
COUNTER_Z, COUNTER_D = 0.90, 0.60            # task spec
COUNTER_T = 0.04                             # EST: worktop thickness
DW_POS = (-dw.D / 2, 0.0, COUNTER_Z)         # front face flush with the counter edge (x = 0)
# Panda mount (x, y, yaw_deg) on the counter, chosen with tools/reach_check.py --search (see MODEL_NOTES)
MOUNT = (-0.08, -0.50, 45.0)
TCP_OFFSET = 0.1034                          # Franka flange->TCP 0.1034 m (hand frame z) - Franka datasheet

COUNTER_RGBA = [.78, .72, .62, 1]            # warm stone worktop
CABINET_RGBA = [.55, .58, .60, 1]


def build_scene(mount=MOUNT, with_robot=True, prefix='dw/'):
    """Returns (model, info). `mount` = (x, y, yaw_deg) of the Panda base on the counter top."""
    spec = mujoco.MjSpec.from_string(f'''<mujoco model="dishwasher_scene">
      <compiler angle="radian"/>{dw.SCENE_OPTIONS}
      <asset><texture name="sky" type="skybox" builtin="gradient" rgb1=".93 .93 .92" rgb2=".62 .63 .65" width="256" height="256"/></asset>
      <worldbody>
        <light name="key" pos="1.6 -1.2 2.6" dir="-.55 .4 -.75" diffuse=".75 .75 .73" castshadow="true"/>
        <light name="fill" pos="-.4 1.5 2.4" dir=".2 -.5 -1" diffuse=".25 .25 .27" castshadow="false"/>
        <geom name="floor" type="plane" size="4 4 .1" rgba=".55 .53 .50 1"/>
        <geom name="counter_top" type="box" pos="{-COUNTER_D/2} -.10 {COUNTER_Z - COUNTER_T/2}"
              size="{COUNTER_D/2} 1.0 {COUNTER_T/2}" rgba="{' '.join(map(str, COUNTER_RGBA))}"/>
        <geom name="cabinet" type="box" pos="-.31 -.10 {(COUNTER_Z - COUNTER_T + .10)/2}"
              size=".27 .99 {(COUNTER_Z - COUNTER_T - .10)/2}" rgba="{' '.join(map(str, CABINET_RGBA))}"/>
        <geom name="toe_kick" type="box" pos="-.34 -.10 .05" size=".24 .99 .05" rgba=".25 .25 .26 1"/>
        <geom name="backsplash" type="box" pos="-.615 -.10 1.2" size=".015 1.0 .3" rgba=".90 .90 .88 1"
              contype="0" conaffinity="0"/>
        <site name="robot_mount" pos="0 0 {COUNTER_Z}" size=".01" rgba="0 0 0 0"/>
      </worldbody></mujoco>''')
    info = dw.add_dishwasher(spec, pos=DW_POS, yaw=0.0, prefix=prefix)
    if with_robot:
        mx, my, myaw = mount
        site = spec.site('robot_mount')
        site.pos = [mx, my, COUNTER_Z]
        site.quat = [math.cos(math.radians(myaw) / 2), 0, 0, math.sin(math.radians(myaw) / 2)]
        spec.attach(mujoco.MjSpec.from_file(str(PANDA)), prefix='panda/', site=site)
        spec.body('panda/hand').add_site(name='tcp', pos=[0, 0, TCP_OFFSET], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
        for body in spec.bodies:
            if body.name.startswith('panda/'):
                body.gravcomp = 1
    model = spec.compile()
    return model, info


def robot_ids(model):
    arm = [model.joint(f'panda/joint{i}') for i in range(1, 8)]
    fingers = [model.joint(f'panda/finger_joint{i}') for i in (1, 2)]
    return arm, fingers


def main():
    model, info = build_scene()
    data = mujoco.MjData(model)
    key = model.key('panda/home') if model.nkey else None
    arm, fingers = robot_ids(model)
    home = np.array([0, 0, 0, -1.57079, 0, 1.57079, -.7853])
    for j, q in zip(arm, home):
        data.qpos[j.qposadr[0]] = q
    for j in fingers:
        data.qpos[j.qposadr[0]] = .04
    for i in range(1, 8):
        data.ctrl[model.actuator(f'panda/actuator{i}').id] = home[i - 1]
    data.ctrl[model.actuator('panda/actuator8').id] = 255
    mujoco.mj_forward(model, data)
    print(f'scene: nbody={model.nbody} ngeom={model.ngeom} nu={model.nu} mount={MOUNT} key={key is not None}')
    for s in info['sites']:
        print(f'  site {s:28s} {np.round(data.site(s).xpos, 4)}')
    for _ in range(500):
        mujoco.mj_step(model, data)
    pen = max([-data.contact[i].dist for i in range(data.ncon)] + [0])
    print(f'1 s settle: ncon={data.ncon}, max penetration {pen*1e3:.3f} mm, door {math.degrees(data.sensor("dw/door_angle").data[0]):.3f} deg')


if __name__ == '__main__':
    main()
