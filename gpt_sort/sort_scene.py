"""Cluttered-countertop scene: kitchen + BREMED E804 model + M1 cart/Panda (all from ../full_pipeline, unchanged) +
6-9 random items with visual soil, a back-wall camera. Layout and soil are drawn from a seeded RNG.

Ground truth (types, soil, grease, poses) is returned for SCORING ONLY and never reaches GPT.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

import cfg
import items as IT

S = cfg.S
import dishwasher as dw  # noqa: E402

# clutter zone on the counter right of the machine (camera-visible, robot-reachable region found in reach_study.py)
ZONE_X = (-0.47, -0.035)
ZONE_Y = (0.27, 1.18)
# plates must be pickable by the edge-push + pinch skill: centre zone and a free corridor to the counter edge
PICK_X = (-0.24, -0.13)          # flat-pick zone (reach_scan.py: push + pinch + tilt + lift plannable, approach 45 deg)
PICK_Y = (0.37, 0.58)              # y < ~0.35: link 5 hit the machine side wall during the edge push (seed 0, 136 N)
FRONT_FREE_X = -0.10              # non-plate items stay behind this line (room for the hand at the counter edge)
SOILS = ('light', 'medium', 'heavy')
PERSON_POS = (1.05, 1.00)
M1_MOUNT = S.Mount('M1', x=.48, y=.42, z=.80, yaw=195., cart_lx=.65, cart_ly=.60, cart_mass=120.)   # = ../full_pipeline run.py OPTIONS['M1']


CIRCLES = {   # top-view footprint as circles (local x, local y, r): tighter than one bounding circle for long items
    'plate': [(0, 0, .1015)], 'bowl': [(0, 0, .066)], 'mug': [(0, 0, .042), (.062, 0, .022)],
    'board': [(-.08, 0, .10), (0, 0, .10), (.08, 0, .10)],
    'knife': [(x, 0, .026) for x in (-.12, -.07, -.02, .03, .08, .13, .17)],
    'pan': [(0, 0, .13), (.16, 0, .03), (.22, 0, .03), (.28, 0, .025), (-.145, 0, .03)],
    'flask': [(0, 0, .037)]}


def _circles(it):
    c, s_ = math.cos(it['yaw']), math.sin(it['yaw'])
    x0, y0 = it['xy']
    return [(x0 + c * x - s_ * y, y0 + s_ * x + c * y, r) for x, y, r in CIRCLES[it['type']]]


def _footprint(it):
    """(centre, radius) of the bounding circle (used by the robot-side helpers / corridor test)."""
    cs = np.array(_circles(it))
    c = cs[:, :2].mean(0)
    return c, float(max(np.linalg.norm(cs[:, :2] - c, axis=1) + cs[:, 2]))


def _corridor_clear(plate, others):
    """The edge push drags the plate straight toward +x (counter edge) and the hand needs ~6 cm behind its far rim:
    that band must be free."""
    x0, y0 = plate['xy']
    for o in others:
        for cx, cy, r in _circles(o):
            if cx + r > x0 - IT.R_OUT - .06 and abs(cy - y0) < IT.R_OUT + r + .02:
                return False
    return True


def _ok_zone(it):
    """Inside the counter zone; non-plates keep the front strip (x > -0.10) free for the edge push / pinch."""
    xmax = ZONE_X[1] if it['type'] == 'plate' else FRONT_FREE_X
    return all(ZONE_X[0] + r * .3 < cx < xmax - r * .8 and ZONE_Y[0] + r < cy < ZONE_Y[1] - r for cx, cy, r in _circles(it))


def _clear(a_, b_, gap=.025):
    return all(math.hypot(ax - bx, ay - by) > ar + br + gap for ax, ay, ar in _circles(a_) for bx, by, br in _circles(b_))


def sample_scene(seed, n_items=None):
    """Random layout (seeded). 2-3 plates; the first two lie in the robot's flat-pick zone (reach_scan.py), a third one
    (if any) anywhere. Returns (items ground truth, decal seed)."""
    rng = np.random.default_rng(seed)
    profile = str(rng.choice(['light', 'normal', 'heavy'], p=[.4, .3, .3]))       # scene soil profile -> balanced program ground truth
    big = [str(rng.choice(['board', 'pan']))]                      # every scene has at least one large non-safe item
    # composition drawn ONCE per seed (re-drawing it on layout failure biased scenes toward few / small items)
    while True:
        n_plate = int(rng.integers(2, 4))
        n_mug = int(rng.integers(1, 3))
        n_bowl = int(rng.integers(0, 3))
        if profile == 'light' and rng.uniform() < .7:
            n_plate, n_mug, n_bowl = 2, 1, 0
        rest = [t for t in ('board', 'knife', 'pan', 'flask') if t not in big]
        n_unsafe = int(rng.integers(2, 5))
        unsafe = big + [str(t) for t in rng.choice(rest, n_unsafe - 1, replace=False)]
        if 6 <= n_plate + n_mug + n_bowl + len(unsafe) <= 9:
            break
    for attempt in range(12000):
        if attempt and attempt % 2000 == 0:   # very crowded draw: drop a non-safe item, then a bowl, then a mug
            if len(unsafe) > 2:
                unsafe = unsafe[:-1]
            elif n_bowl:
                n_bowl -= 1
            elif n_mug > 1:
                n_mug -= 1
        placed, ok = [], True
        # one plate where the robot can pick it; the other plates anywhere
        placed.append(dict(type='plate', xy=(float(rng.uniform(*PICK_X)), float(rng.uniform(*PICK_Y))), yaw=float(rng.uniform(-math.pi, math.pi))))
        types = sorted(unsafe, key=lambda t: t not in ('pan', 'board')) + ['plate'] * (n_plate - 1) + ['mug'] * n_mug + ['bowl'] * n_bowl
        for t in types:
            for _ in range(600):
                yaw = float(rng.uniform(-math.pi, math.pi))
                if t in ('board', 'knife'):
                    yaw = float(rng.uniform(-.4, .4) + (math.pi / 2 if rng.uniform() < .5 else 0.))
                it = dict(type=t, xy=(float(rng.uniform(*ZONE_X)), float(rng.uniform(*ZONE_Y))), yaw=yaw)
                if not _ok_zone(it):
                    continue
                if not all(_clear(it, o) for o in placed):
                    continue
                if not _corridor_clear(placed[0], [it]):
                    continue
                placed.append(it)
                break
            else:
                ok = False
                break
        if ok:
            break
    else:
        raise RuntimeError('layout sampling failed')
    soil_p = dict(light=([1., 0, 0], 0.), normal=([.35, .65, 0], 0.), heavy=([.2, .4, .4], .35))[profile]
    for it in placed:
        if it['type'] in ('plate', 'bowl', 'mug'):
            it['soil'] = str(rng.choice(SOILS, p=soil_p[0]))
            it['greasy'] = bool(it['type'] != 'mug' and rng.uniform() < soil_p[1])
        else:
            it['soil'] = str(rng.choice(SOILS))
            it['greasy'] = bool(it['type'] == 'pan' and rng.uniform() < .6)
        it['dishwasher_safe'] = IT.SAFE[it['type']]
    if profile == 'heavy' and not any(it['dishwasher_safe'] and (it['soil'] == 'heavy' or it['greasy']) for it in placed):
        next(it for it in placed if it['type'] == 'plate')['soil'] = 'heavy'
    order = rng.permutation(len(placed))
    placed = [placed[i] for i in order]
    for k, it in enumerate(placed):
        it['name'] = f'item{k}'
    return placed, int(rng.integers(1 << 30))  # (soil profile is implicit in the items)


def build_model(items_gt, decal_seed, mount=None, grip_k=800., grip_limit=70.):
    """Compile the scene. Bodies 'plate' / 'pod' / 'dish' are inert placeholders the reused Runner / SimRobot constructors
    look up by name; they sit hidden under the counter, have no collision and are never used here."""
    mount = mount or M1_MOUNT
    spec = mujoco.MjSpec.from_string(S._header())
    S._add_kitchen(spec)
    lay = S.Layout(cam_pos=cfg.CAM_POS, cam_look=cfg.CAM_LOOK)
    S._add_camera(spec, lay)
    fwd = np.subtract(cfg.ROOMCAM_LOOK, cfg.ROOMCAM_POS)
    fwd = fwd / np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    spec.worldbody.add_camera(name='roomcam', pos=list(cfg.ROOMCAM_POS), quat=list(S.quat(np.column_stack([right, np.cross(right, fwd), -fwd]))),
                              fovy=55.)
    info = dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    site = S._add_robot(spec, mount)
    S._attach_panda(spec, site, grip_k, grip_limit)
    rng = np.random.default_rng(decal_seed)
    for it in items_gt:
        IT.BUILDERS[it['type']](spec, it['name'], it['xy'], it['yaw'], it['soil'], it['greasy'], rng)
    wb = spec.worldbody
    for nm in ('plate', 'pod', 'dish'):          # inert placeholders (see docstring)
        b = wb.add_body(name=nm, pos=[-.3, -1.0, .3])
        b.add_geom(name=f'{nm}_placeholder', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[.005, 0, 0], rgba=[0, 0, 0, 0],
                   contype=0, conaffinity=0, mass=.001)
        if nm == 'plate':
            b.add_site(name='plate_center', pos=[0, 0, 0], size=[.003, 0, 0], group=4)
        if nm == 'pod':
            b.add_freejoint(name='pod_free')
    x, y = PERSON_POS
    for nm, kw in (('person_legs', dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[x, y, .12, x, y, .85], size=[.13, 0, 0])),
                   ('person_torso', dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[x, y, .95, x, y, 1.40], size=[.17, 0, 0])),
                   ('person_head', dict(type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[x, y, 1.66], size=[.11, 0, 0]))):
        wb.add_geom(name=nm, rgba=[.86, .48, .30, 0.], contype=0, conaffinity=0, group=1, **kw)
    return spec.compile(), info


def describe_gt(items_gt):
    return [dict(name=it['name'], type=it['type'], xy=[round(v, 3) for v in it['xy']], yaw_deg=round(math.degrees(it['yaw'])),
                 soil=it['soil'], greasy=it['greasy'], dishwasher_safe=it['dishwasher_safe']) for it in items_gt]


if __name__ == '__main__':
    import sys
    from PIL import Image
    from util import renderer, scene_option
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    its, ds = sample_scene(seed)
    for it in describe_gt(its):
        print(it)
    m, _ = build_model(its, ds)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(400):
        mujoco.mj_step(m, d)
    r = renderer(m, S.CAM_H, S.CAM_W)
    r.update_scene(d, camera='wallcam', scene_option=scene_option())
    Image.fromarray(r.render()).save(cfg.ROOT / 'out' / f'scene_s{seed}.png')
    r.update_scene(d, camera='roomcam', scene_option=scene_option())
    Image.fromarray(r.render()).save(cfg.ROOT / 'out' / f'scene_s{seed}_overview.png')
    # settle check: items must not move when nothing touches them
    for it in its:
        print(it['name'], it['type'], np.round(d.body(it['name']).xpos - [it['xy'][0], it['xy'][1], 0], 4))
