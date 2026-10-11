#!/usr/bin/env python3
"""Renders (1280x720 PNG) into renders/:
  01_door_closed_3q.png      dishwasher on the counter, door closed, front 3/4 view
  02_door_open_basket_out.png door at 90 deg, basket pulled out 0.25 m
  03_basket_tines_closeup.png close-up of plate tines + 35 deg bowl row (basket out)
  04_panda_scene_3q.png       Panda + dishwasher, arm at the IK solution for plate_slot_target
The Panda pose in 04 comes from tools/reach_results.json (run tools/reach_check.py first).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import dishwasher as dw
import render_util as ru
import scene_demo as sd

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'renders'
FONT = Path('/System/Library/Fonts/Supplemental/Arial.ttf')


def caption(img, text):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(str(FONT), 20) if FONT.exists() else ImageFont.load_default()
    d.rectangle((0, 676, 1280, 720), fill=(24, 26, 30))
    d.text((16, 686), text, fill=(235, 235, 235), font=f)
    return im


def main():
    OUT.mkdir(exist_ok=True)
    res = json.loads((ROOT / 'tools/reach_results.json').read_text())
    mount = tuple(res['mount'])
    # scenes 1-3: dishwasher + counter only (robot hidden by building without it)
    model, _ = sd.build_scene(with_robot=False)
    data = mujoco.MjData(model)
    r = ru.make_renderer(model)
    cx = -dw.D / 2
    dw.set_state(model, data, 0, 0)
    caption(ru.render(r, data, [cx + .05, 0, 1.13], 1.55, 148, -16),
            'Countertop dishwasher 0.430 x 0.390 x 0.460 m, 13 kg - door closed (latched: |angle| < 1 deg)').save(OUT / '01_door_closed_3q.png')
    dw.set_state(model, data, 90, dw.BASKET_TRAVEL)
    caption(ru.render(r, data, [cx + .25, 0, 1.05], 1.65, 145, -28),
            'Door open 90 deg (on its stop), lower basket pulled out 0.25 m on its slide').save(OUT / '02_door_open_basket_out.png')
    caption(ru.render(r, data, [cx + .28, -.02, 1.07], .62, 160, -38, sites=True),
            'Basket: plate tines r=2 mm h=60 mm pitch 28 mm, rows 140 mm apart; bowl row leaning 35 deg; green = slot targets').save(
        OUT / '03_basket_tines_closeup.png')
    r.close()

    # scene 4: with the Panda at the plate-slot IK solution
    model, _ = sd.build_scene(mount=mount)
    data = mujoco.MjData(model)
    row = next(x for x in res['results'] if x['target'].startswith('plate_slot_target'))
    dw.set_state(model, data, row['door_deg'], row['basket_m'])
    for i, q in enumerate(row['q_arm']):
        data.qpos[model.joint(f'panda/joint{i + 1}').qposadr[0]] = q
    for i in (1, 2):
        data.qpos[model.joint(f'panda/finger_joint{i}').qposadr[0]] = row['q_fingers']
    mujoco.mj_forward(model, data)
    r = ru.make_renderer(model)
    caption(ru.render(r, data, [-.18, -.22, 1.12], 2.0, 138, -22),
            f'Panda on the counter (mount x={mount[0]:.2f} y={mount[1]:.2f} yaw={mount[2]:.0f} deg), TCP at plate_slot_target '
            f'(IK err {row["pos_err_mm"]:.2f} mm)').save(OUT / '04_panda_scene_3q.png')
    r.close()
    print('wrote', sorted(p.name for p in OUT.glob('*.png')))


if __name__ == '__main__':
    main()
