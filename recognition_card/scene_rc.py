"""Recognition-card scene: kitchen + countertop dishwasher (../full_pipeline, read-only) and a hand-laid messy counter:
3 plates in a pile, 2 bowls, mug (../gpt_sort/items.py builders, read-only) + chef's knife on a wooden board, cast-iron
pan, vacuum flask, and NEW primitive items defined here: dish-soap bottle, sponge, stovetop kettle.
No robot. Settled with mj_step, then one RGB + depth + segmentation render from a room-side camera.
Ground truth (types, boxes from segmentation, settled poses) is saved for SCORING ONLY and never shown to GPT.

usage (always via ./runw.sh): python scene_rc.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
GW = Path.home() / 'Documents/gyj_work'
for p in (GW / 'full_pipeline', GW / 'gpt_sort'):
    sys.path.insert(0, str(p))
import scene as S  # noqa: E402
import dishwasher as dw  # noqa: E402
import items as IT  # noqa: E402
from util import renderer, scene_option  # noqa: E402

import os
V = os.environ.get('RC_VARIANT', '')   # '' = original scene; '_5dish' = same scene without the mug
W, H, FOVY = 1280, 720, 44.0
CAM_POS = np.array([0.62, 0.66, 1.78])
CAM_LOOK = np.array([-0.30, 0.66, 0.90])
CZ = IT.COUNTER_Z
VIS, OBJ = IT.VIS, IT.OBJ
BOX, CYL, CAP = IT.BOX, IT.CYL, IT.CAP


def add_soap(spec, name, xy, yaw):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], CZ + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    b.add_geom(name=f'{name}_body', type=BOX, pos=[0, 0, .085], size=[.022, .034, .085], mass=.45, rgba=[.22, .68, .30, .92], **OBJ)
    b.add_geom(name=f'{name}_label', type=BOX, pos=[.0222, 0, .07], size=[.0003, .026, .038], rgba=[.97, .97, .94, 1], **VIS)
    b.add_geom(name=f'{name}_stripe', type=BOX, pos=[.0226, 0, .085], size=[.0003, .026, .008], rgba=[.95, .75, .10, 1], **VIS)
    b.add_geom(name=f'{name}_label2', type=BOX, pos=[-.0222, 0, .07], size=[.0003, .026, .038], rgba=[.97, .97, .94, 1], **VIS)
    b.add_geom(name=f'{name}_shoulder', type=CYL, pos=[0, 0, .176], size=[.018, .006, 0], rgba=[.22, .68, .30, .92], **VIS)
    b.add_geom(name=f'{name}_cap', type=CYL, pos=[0, 0, .192], size=[.013, .011, 0], rgba=[.96, .96, .96, 1], **VIS)
    b.add_geom(name=f'{name}_nozzle', type=CYL, pos=[0, 0, .208], size=[.004, .006, 0], rgba=[.96, .96, .96, 1], **VIS)
    return b


def add_sponge(spec, name, xy, yaw):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], CZ + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    b.add_geom(name=f'{name}_foam', type=BOX, pos=[0, 0, .013], size=[.055, .036, .013], mass=.02, rgba=[.98, .82, .20, 1], **OBJ)
    b.add_geom(name=f'{name}_scrub', type=BOX, pos=[0, 0, .029], size=[.055, .036, .004], mass=.005, rgba=[.16, .52, .24, 1], **OBJ)
    return b


def add_kettle(spec, name, xy, yaw):
    """Stovetop kettle, spout along local +x."""
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], CZ + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    if 'mat_steel' not in {m.name for m in spec.materials}:
        spec.add_material(name='mat_steel', rgba=[.80, .82, .85, 1], specular=1., shininess=.9, reflectance=.3)
    b.add_geom(name=f'{name}_body', type=CYL, pos=[0, 0, .065], size=[.088, .065, 0], mass=.7, material='mat_steel', **OBJ)
    b.add_geom(name=f'{name}_shoulder', type=CYL, pos=[0, 0, .137], size=[.068, .008, 0], material='mat_steel', **VIS)
    b.add_geom(name=f'{name}_lid', type=CYL, pos=[0, 0, .149], size=[.045, .005, 0], material='mat_steel', **VIS)
    b.add_geom(name=f'{name}_knob', type=CYL, pos=[0, 0, .162], size=[.014, .009, 0], rgba=[.06, .06, .07, 1], **VIS)
    for s in (-1, 1):
        b.add_geom(name=f'{name}_post{s}', type=CAP, fromto=[s * .055, 0, .14, s * .06, 0, .215], size=[.006, 0, 0],
                   rgba=[.06, .06, .07, 1], **VIS)
    b.add_geom(name=f'{name}_grip', type=CAP, fromto=[-.06, 0, .218, .06, 0, .218], size=[.012, 0, 0], rgba=[.06, .06, .07, 1], **VIS)
    b.add_geom(name=f'{name}_spout', type=CAP, fromto=[.075, 0, .05, .15, 0, .135], size=[.013, 0, 0], material='mat_steel', **VIS)
    b.add_geom(name=f'{name}_whistle', type=CYL, pos=[.152, 0, .14], size=[.016, .008, 0], rgba=[.06, .06, .07, 1], **VIS)
    return b


# ---- hand-laid messy layout (x: -0.60 back wall .. 0 front edge, y: right of the dishwasher which spans |y| < 0.215)
# type, (x, y), yaw (deg), soil, greasy, dz (initial height above the counter), tilt (deg about local y)
LAYOUT = [
    ('plate', (-0.193, 0.404), 10, 'heavy', True, .0000),
    ('plate', (-0.200, 0.426), 75, 'medium', False, .0125),
    ('plate', (-0.212, 0.408), 140, 'heavy', True, .0250),
    ('bowl', (-0.430, 0.400), 20, 'medium', False, 0.),
    ('bowl', (-0.255, 0.700), 0, 'heavy', True, 0.),
    ('mug', (-0.090, 0.600), 215, 'medium', False, 0.),
    ('board', (-0.420, 0.900), 98, 'medium', False, 0.),
    ('knife', (-0.405, 0.915), 128, 'light', False, .0185),
    ('pan', (-0.170, 1.010), -40, 'heavy', True, 0.),
    ('flask', (-0.530, 0.620), 0, 'light', False, 0.),
    ('soap', (-0.525, 0.300), 15, None, False, 0.),
    ('sponge', (-0.080, 0.290), 25, None, False, 0.),
    ('kettle', (-0.470, 1.170), 200, None, False, 0.),
]
NEW = dict(soap=add_soap, sponge=add_sponge, kettle=add_kettle)
SAFE = dict(IT.SAFE, soap=False, sponge=False, kettle=False)
NAMES = dict(plate='plate', bowl='bowl', mug='mug', board='wooden cutting board', knife="chef's knife", pan='cast-iron frying pan',
             flask='vacuum flask', soap='dish-soap bottle', sponge='sponge', kettle='stovetop kettle')


def build(decal_seed=7):
    spec = mujoco.MjSpec.from_string(S._header())
    S._add_kitchen(spec)
    fwd = CAM_LOOK - CAM_POS
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    spec.worldbody.add_camera(name='cardcam', pos=list(CAM_POS), quat=list(IT.quat(np.column_stack([right, up, -fwd]))), fovy=FOVY)
    dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    rng = np.random.default_rng(decal_seed)
    gt = []
    for k, (t, xy, yaw, soil, greasy, dz) in enumerate(LAYOUT):
        nm = f'{t}{k}'
        if V == '_5dish' and t == 'mug':      # build into a throwaway spec: keeps the decal RNG stream (other items) identical
            IT.BUILDERS[t](mujoco.MjSpec(), nm, xy, math.radians(yaw), soil, greasy, rng)
            continue
        if t in NEW:
            b = NEW[t](spec, nm, xy, math.radians(yaw))
        else:
            b = IT.BUILDERS[t](spec, nm, xy, math.radians(yaw), soil, greasy, rng)
        b.pos = [xy[0], xy[1], CZ + .0003 + dz]
        gt.append(dict(name=nm, type=t, label=NAMES[t], dishwasher_safe=SAFE[t], soil=soil, greasy=greasy))
    return spec.compile(), gt


def main():
    m, gt = build()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    p0 = {g['name']: d.body(g['name']).xpos.copy() for g in gt}
    for _ in range(2500):                  # 5 s settle
        mujoco.mj_step(m, d)
    v = float(np.abs(d.qvel).max())
    r = renderer(m, H, W)
    opt = scene_option(hide_sites=True)
    r.update_scene(d, camera='cardcam', scene_option=opt)
    rgb = r.render().copy()
    r.enable_depth_rendering()
    r.update_scene(d, camera='cardcam', scene_option=opt)
    depth = r.render().copy()
    r.disable_depth_rendering()
    r.enable_segmentation_rendering()
    r.update_scene(d, camera='cardcam', scene_option=opt)
    seg = r.render().copy()
    r.disable_segmentation_rendering()
    r.close()
    rng = np.random.default_rng(1)
    depth_noisy = depth + rng.normal(0, .0015, depth.shape) * depth     # same noise model as full_pipeline/perception.py
    cid = m.camera('cardcam').id
    f = (H / 2) / math.tan(math.radians(FOVY) / 2)
    # GT boxes from segmentation: geom id -> body
    gid = seg[..., 0]
    is_geom = seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
    body_of = m.geom_bodyid
    for g in gt:
        bid = m.body(g['name']).id
        mask = is_geom & (gid >= 0) & np.isin(gid, np.where(body_of == bid)[0])
        ys, xs = np.nonzero(mask)
        g['box'] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())] if len(xs) else None
        g['visible_px'] = int(mask.sum())
        g['pos'] = [round(float(x), 5) for x in d.body(g['name']).xpos]
        g['moved_mm'] = round(float(np.linalg.norm(d.body(g['name']).xpos[:2] - p0[g['name']][:2]) * 1000), 1)
        zax = d.body(g['name']).xmat.reshape(3, 3)[:, 2]
        g['tilt_deg'] = round(math.degrees(math.acos(min(1., abs(zax[2])))), 1)
    from PIL import Image
    Image.fromarray(rgb).save(ROOT / 'out' / f'counter{V}.png')
    np.savez_compressed(ROOT / 'out' / f'rgbd{V}.npz', rgb=rgb, depth=depth_noisy.astype(np.float32), cam_pos=d.cam_xpos[cid],
                        cam_mat=d.cam_xmat[cid].reshape(3, 3), f=f)
    (ROOT / 'out' / f'gt{V}.json').write_text(json.dumps(dict(items=gt, max_qvel_after_settle=v), indent=1))
    for g in gt:
        print(g['name'], g['box'], g['visible_px'], g['pos'], 'moved', g['moved_mm'], 'tilt', g['tilt_deg'])
    print('max |qvel| after settle', v)


if __name__ == '__main__':
    main()
