"""Messy kitchen SINK scene (reusable module). Own counter with a sink cut-out (the full_pipeline counter is one solid box),
stainless basin (floor, 4 walls, drain), gooseneck faucet, collision back wall; the full_pipeline countertop dishwasher
(read-only) at its usual place left of the sink. Dishes (../gpt_sort/items.py builders, soil decals) are DROPPED into the
basin from perturbed poses (seeded) and settled with mj_step: plates lean on basin walls, one bowl lands on the plate pile.
Around the sink: dish-soap bottle, sponge, dish cloth, cutting board leaning on the wall, knife, cast-iron pan, flask.

Ground truth for scoring only: settled poses, visible-pixel boxes (segmentation) and a PHYSICAL support test: each dish is
removed in turn (teleported away) and the others are stepped 0.6 s; any dish that moves > 5 mm was resting on it.

  build(seed)            -> (model, gt list)      used by run()
  run(seed)              -> settle, render RGB-D + seg from 'sinkcam', support test, write out/<prefix>_*.{png,npz,json}
Layouts (build(seed, layout=...) / run(seed, layout=...)):
  'default' : sink right of the dishwasher, chaotic pile, overhead camera; prefix 'sink'   (seed 3 = sink_recognition.mp4)
  'sink6'   : matches ../sink_refs/sink_6.png: sink LEFT of the dishwasher, loose offset stack of 4 plates sliding against
              the right basin wall, 2 bowls wedged between the stack and the left wall, faucet / soap / sponge on the back
              ledge, checked cloth on the left counter; front-oblique camera; prefix 'sink6'          (seed 6)
usage (always via ./runw.sh): python sink_scene.py [seed] [layout]
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
for p in (GW / 'full_pipeline', GW / 'gpt_sort', ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import scene as S  # noqa: E402
import dishwasher as dw  # noqa: E402
import items as IT  # noqa: E402
from scene_rc import add_soap, add_sponge  # noqa: E402
from util import renderer, scene_option  # noqa: E402

W, H, FOVY = 1280, 720, 44.0
CAM_POS = np.array([0.20, 0.86, 2.02])
CAM_LOOK = np.array([-0.33, 0.86, 0.80])
CZ = .90
BOX, CYL, CAP = IT.BOX, IT.CYL, IT.CAP
ENV = dict(contype=1, conaffinity=1, condim=3, friction=[.8, .005, .0001])
VIS = IT.VIS
# sink cut-out (world): x in [SX0, SX1] (x = 0 is the counter front edge, -0.60 the wall), y in [SY0, SY1]
SX0, SX1, SY0, SY1 = -0.50, -0.12, 0.34, 0.90
DEPTH, T = 0.20, 0.010
FLOOR_Z = CZ - DEPTH
CY = (-1.25, 1.80)


def _q(R):
    return list(IT.quat(R))


def rot(axis, ang):
    c, s = math.cos(ang), math.sin(ang)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def add_kitchen_with_sink(spec):
    wb = spec.worldbody
    if 'mat_sink' not in {m.name for m in spec.materials}:
        spec.add_material(name='mat_sink', rgba=[.70, .72, .74, 1], specular=.8, shininess=.6, reflectance=.15)
        spec.add_material(name='mat_chrome', rgba=[.86, .87, .89, 1], specular=1., shininess=1., reflectance=.4)
    wb.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, .1], material='floormat', **ENV)
    col = S.COUNTER_RGBA
    zc = CZ - .02
    def slab(nm, x0, x1, y0, y1):
        wb.add_geom(name=nm, type=BOX, pos=[(x0 + x1) / 2, (y0 + y1) / 2, zc], size=[(x1 - x0) / 2, (y1 - y0) / 2, .02], rgba=col, **ENV)
    slab('ct_back', -.60, SX0, CY[0], CY[1])
    slab('ct_front', SX1, 0., CY[0], CY[1])
    slab('ct_left', SX0, SX1, CY[0], SY0)
    slab('ct_right', SX0, SX1, SY1, CY[1])
    # cabinets / toe kick / wall: visual only, except a collision wall slab behind the counter (the board leans on it)
    yc, hy = (CY[0] + CY[1]) / 2, (CY[1] - CY[0]) / 2
    wb.add_geom(name='cabinet', type=BOX, pos=[-.30, yc, .38], size=[.28, hy - .01, .28], rgba=S.CABINET_RGBA, **VIS)
    if CHECKED:      # 'sink6': cabinet apron under the front counter strip (hides the basin walls below the counter)
        wb.add_geom(name='apron', type=BOX, pos=[-.025, yc, .76], size=[.005, hy - .01, .10], rgba=S.CABINET_RGBA, **VIS)
    wb.add_geom(name='wall', type=BOX, pos=[-.615, yc, 1.35], size=[.015, hy + .3, 1.35], rgba=[.93, .92, .89, 1], **ENV)
    # tiled splash-back band (visual)
    for k in range(int((CY[1] - CY[0]) / .15)):
        wb.add_geom(name=f'tile{k}', type=BOX, pos=[-.5995, CY[0] + .075 + k * .15, 1.0], size=[.0005, .073, .098],
                    rgba=[.86, .89, .90, 1], **VIS)
    # basin: floor + 4 walls just inside the cut-out, rim lip on the counter
    cx, cy = (SX0 + SX1) / 2, (SY0 + SY1) / 2
    hx, hyb = (SX1 - SX0) / 2, (SY1 - SY0) / 2
    wb.add_geom(name='sink_floor', type=BOX, pos=[cx, cy, FLOOR_Z - T / 2], size=[hx, hyb, T / 2], material='mat_sink', **ENV)
    zw = FLOOR_Z + DEPTH / 2 - .001
    wb.add_geom(name='sink_wb', type=BOX, pos=[SX0 + T / 2, cy, zw], size=[T / 2, hyb, DEPTH / 2], material='mat_sink', **ENV)
    wb.add_geom(name='sink_wf', type=BOX, pos=[SX1 - T / 2, cy, zw], size=[T / 2, hyb, DEPTH / 2], material='mat_sink', **ENV)
    wb.add_geom(name='sink_wl', type=BOX, pos=[cx, SY0 + T / 2, zw], size=[hx, T / 2, DEPTH / 2], material='mat_sink', **ENV)
    wb.add_geom(name='sink_wr', type=BOX, pos=[cx, SY1 - T / 2, zw], size=[hx, T / 2, DEPTH / 2], material='mat_sink', **ENV)
    for nm, p_, s_ in (('lip_b', [SX0 - .012, cy, CZ + .002], [.012, hyb + .024, .002]), ('lip_f', [SX1 + .012, cy, CZ + .002], [.012, hyb + .024, .002]),
                       ('lip_l', [cx, SY0 - .012, CZ + .002], [hx, .012, .002]), ('lip_r', [cx, SY1 + .012, CZ + .002], [hx, .012, .002])):
        wb.add_geom(name=nm, type=BOX, pos=p_, size=s_, material='mat_sink', **VIS)
    dx, dy = cx + .05, cy + .12
    wb.add_geom(name='drain', type=CYL, pos=[dx, dy, FLOOR_Z + .0004], size=[.042, .0004, 0], rgba=[.35, .36, .38, 1], **VIS)
    wb.add_geom(name='drain_hole', type=CYL, pos=[dx, dy, FLOOR_Z + .0008], size=[.030, .0004, 0], rgba=[.06, .06, .07, 1], **VIS)
    for k in range(4):
        wb.add_geom(name=f'drain_bar{k}', type=BOX, pos=[dx, dy, FLOOR_Z + .0011], quat=IT.q_yaw(k * math.pi / 4),
                    size=[.030, .002, .0003], rgba=[.55, .56, .58, 1], **VIS)
    # gooseneck faucet behind the basin (visual only: dishes never reach it)
    fx, fy = FAUCET if FAUCET else (-.555, cy - .02)
    chrome = dict(material='mat_chrome', **VIS)
    wb.add_geom(name='fct_base', type=CYL, pos=[fx, fy, CZ + .02], size=[.028, .02, 0], **chrome)
    wb.add_geom(name='fct_col', type=CAP, fromto=[fx, fy, CZ + .03, fx, fy, CZ + .30], size=[.016, 0, 0], **chrome)
    pts = [(fx + .09 * (1 - math.cos(a)) , CZ + .30 + .09 * math.sin(a)) for a in np.linspace(0, math.pi, 9)]
    for k in range(len(pts) - 1):
        wb.add_geom(name=f'fct_neck{k}', type=CAP, fromto=[pts[k][0], fy, pts[k][1], pts[k + 1][0], fy, pts[k + 1][1]], size=[.014, 0, 0], **chrome)
    ex = pts[-1][0]
    wb.add_geom(name='fct_spout', type=CAP, fromto=[ex, fy, CZ + .30, ex, fy, CZ + .25], size=[.015, 0, 0], **chrome)
    wb.add_geom(name='fct_lever', type=CAP, fromto=[fx, fy + .03, CZ + .14, fx + .07, fy + .05, CZ + .17], size=[.008, 0, 0], **chrome)


def add_cloth(spec, name, xy, yaw, checked=False):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], CZ + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    if checked:      # white tea towel with blue checks, folded
        b.add_geom(name=f'{name}_cloth', type=BOX, pos=[0, 0, .002], size=[.12, .09, .002], mass=.03, rgba=[.94, .93, .90, 1], **IT.OBJ)
        for k in range(7):
            b.add_geom(name=f'{name}_cx{k}', type=BOX, pos=[-.105 + k * .035, 0, .0042], size=[.004, .09, .0002], rgba=[.20, .30, .55, 1], **VIS)
        for k in range(5):
            b.add_geom(name=f'{name}_cy{k}', type=BOX, pos=[0, -.075 + k * .0375, .0043], size=[.12, .004, .0002], rgba=[.20, .30, .55, 1], **VIS)
        b.add_geom(name=f'{name}_fold', type=BOX, pos=[.05, .02, .006], quat=IT.q_yaw(.35), size=[.06, .07, .002], mass=.01, rgba=[.92, .91, .88, 1], **IT.OBJ)
        for k in range(3):
            b.add_geom(name=f'{name}_fx{k}', type=BOX, pos=[.05, .02, .0082], quat=IT.q_yaw(.35 + k * 0.0), size=[.06, .004, .0002], rgba=[.20, .30, .55, 1], **VIS)
        return b
    for k in range(5):
        b.add_geom(name=f'{name}_st{k}', type=BOX, pos=[-.09 + k * .045, 0, .0042], size=[.006, .075, .0002], rgba=[.95, .95, .95, 1], **VIS)
    b.add_geom(name=f'{name}_fold', type=BOX, pos=[.06, .01, .006], quat=IT.q_yaw(.3), size=[.05, .06, .002], mass=.01, rgba=[.27, .48, .74, 1], **IT.OBJ)
    return b


# dishes: type, centre (x, y, z), tilt axis, tilt deg, yaw deg, soil, greasy  (perturbed by the seed before the drop)
DISHES = [
    ('plate', (-.33, .50, FLOOR_Z + .006), 'y', 4, 10, 'heavy', True),        # flat on the basin floor
    ('plate', (-.29, .55, FLOOR_Z + .045), 'x', 9, 60, 'medium', False),       # dropped onto plate 1
    ('plate', (-.26, .80, FLOOR_Z + .108), 'x', 62, 0, 'heavy', False),        # leans on the right wall
    ('plate', (-.415, .70, FLOOR_Z + .100), 'y', 58, 0, 'medium', True),      # leans on the back wall
    ('bowl', (-.29, .56, FLOOR_Z + .110), 'x', 6, 30, 'heavy', True),          # lands on the plate pile
    ('bowl', (-.19, .415, FLOOR_Z + .060), 'y', -28, 0, 'medium', False),      # tipped in the front-left corner
    ('mug', (-.20, .72, FLOOR_Z + .060), 'x', 90, 25, 'medium', False),        # lying on its side
]
COUNTER = [   # type, (x, y), yaw deg
    ('soap', (-.545, .26), 20), ('sponge', (-.27, .275), 75), ('cloth', (-.28, .99), -15),
    ('knife', (-.085, 1.20), 95), ('pan', (-.29, 1.33), 140), ('flask', (-.535, 1.04), 0)]
SAFE = dict(IT.SAFE, soap=False, sponge=False, cloth=False)
NAMES = dict(plate='plate', bowl='bowl', mug='mug', board='wooden cutting board', knife="chef's knife", pan='cast-iron frying pan',
             flask='vacuum flask', soap='dish-soap bottle', sponge='sponge', cloth='dish cloth')


BOARD_Y, FAUCET, PREFIX, CHECKED, SETTLE = 1.42, None, 'sink', False, 6000
_TH = math.radians(34)
_N = np.array([0, -math.sin(_TH), math.cos(_TH)])        # plate-stack normal for 'sink6' (leaning on the right wall)
_C0 = np.array([-.27, -.396, FLOOR_Z + .062])
DISHES6 = [('plate', tuple(_C0 + k * .015 * _N + np.array([dx, 0, 0])), 'x', 34 + dt, 0, soil, gr)
           for k, (dx, dt, soil, gr) in enumerate([(0, 0, 'medium', False), (.018, -2, 'heavy', True), (-.012, 2, 'medium', False),
                                                   (.024, -3, 'heavy', True)])] + [
    ('bowl', (-.33, -.70, FLOOR_Z + .012), 'x', 12, 0, 'medium', False),      # back-left, against the left wall
    ('bowl', (-.25, -.545, FLOOR_Z + .006), 'x', 8, 0, 'heavy', True)]        # wedged against the foot of the stack
COUNTER6 = [('soap', (-.535, -.76), 10), ('sponge', (-.53, -.355), 80), ('cloth', (-.24, -.93), 25),
            ('knife', (-.075, -1.06), 95), ('pan', (-.30, -1.12), -130), ('flask', (-.53, -.90), 0)]
LAYOUTS = {
    'default': dict(sink=(-0.50, -0.12, 0.34, 0.90), cam=((0.20, 0.86, 2.02), (-0.33, 0.86, 0.80)), fovy=44., dishes=DISHES,
                    counter=COUNTER, board_y=1.42, faucet=None, prefix='sink', checked=False, settle=6000),
    'sink6': dict(sink=(-0.44, -0.08, -0.80, -0.30), cam=((0.42, -0.60, 1.98), (-0.27, -0.50, 0.76)), fovy=48., dishes=DISHES6,
                  counter=COUNTER6, board_y=-1.20, faucet=(-.53, -.60), prefix='sink6', checked=True, settle=20000),
}


def use_layout(name):
    global SX0, SX1, SY0, SY1, CAM_POS, CAM_LOOK, FOVY, DISHES_, COUNTER_, BOARD_Y, FAUCET, PREFIX, CHECKED, SETTLE
    L = LAYOUTS[name]
    SX0, SX1, SY0, SY1 = L['sink']
    CAM_POS, CAM_LOOK = np.array(L['cam'][0]), np.array(L['cam'][1])
    FOVY, DISHES_, COUNTER_, BOARD_Y = L['fovy'], L['dishes'], L['counter'], L['board_y']
    FAUCET, PREFIX, CHECKED, SETTLE = L['faucet'], L['prefix'], L['checked'], L['settle']


def build(seed=3, layout='default'):
    use_layout(layout)
    spec = mujoco.MjSpec.from_string(S._header().replace('timestep="0.002"', 'timestep="0.001"'))
    add_kitchen_with_sink(spec)
    fwd = CAM_LOOK - CAM_POS
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    spec.worldbody.add_camera(name='sinkcam', pos=list(CAM_POS), quat=_q(np.column_stack([right, np.cross(right, fwd), -fwd])), fovy=FOVY)
    dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    rng = np.random.default_rng(seed)
    gt = []
    for k, (t, (x, y, z), ax, tilt, yaw, soil, greasy) in enumerate(DISHES_):
        nm = f'{t}{k}'
        b = IT.BUILDERS[t](spec, nm, (x, y), 0., soil, greasy, rng)
        x += rng.uniform(-.008, .008)
        y += rng.uniform(-.008, .008)
        R = rot('z', math.radians(yaw + rng.uniform(-15, 15))) @ rot(ax, math.radians(tilt + rng.uniform(-3, 3)))
        b.pos = [x, y, z + .01]                     # small drop
        b.quat = _q(R)
        gt.append(dict(name=nm, type=t, label=NAMES[t], dishwasher_safe=True, dish=True, soil=soil, greasy=greasy))
    for k, (t, xy, yaw) in enumerate(COUNTER_):
        nm = f'{t}{k + len(DISHES_)}'
        if t == 'soap':
            add_soap(spec, nm, xy, math.radians(yaw))
        elif t == 'sponge':
            add_sponge(spec, nm, xy, math.radians(yaw))
        elif t == 'cloth':
            add_cloth(spec, nm, xy, math.radians(yaw), CHECKED)
        else:
            IT.BUILDERS[t](spec, nm, xy, math.radians(yaw), 'medium', t == 'pan', rng)
        gt.append(dict(name=nm, type=t, label=NAMES[t], dishwasher_safe=SAFE[t], dish=False))
    # cutting board leaning on the wall (long axis 15 deg from vertical, top face to the room)
    a = math.radians(15)
    u, v = np.array([-math.sin(a), 0, math.cos(a)]), np.array([0, -1., 0])
    n = np.cross(u, v)
    nm = f'board{len(gt)}'
    b = IT.add_board(spec, nm, (0, 0), 0., 'light', False, rng)
    c = np.array([-.598 + .16 * math.sin(a) + .009 * math.cos(a), BOARD_Y, CZ + .16 * math.cos(a) + .009 * math.sin(a) + .004])
    b.quat = _q(np.column_stack([u, v, n]))
    b.pos = list(c - n * .009)
    gt.append(dict(name=nm, type='board', label=NAMES['board'], dishwasher_safe=False, dish=False))
    return spec.compile(), gt


def render(m, d):
    r = renderer(m, H, W)
    opt = scene_option(hide_sites=True)
    out = []
    for mode in ('rgb', 'depth', 'seg'):
        if mode == 'depth':
            r.enable_depth_rendering()
        if mode == 'seg':
            r.enable_segmentation_rendering()
        r.update_scene(d, camera='sinkcam', scene_option=opt)
        out.append(r.render().copy())
        r.disable_depth_rendering()
        r.disable_segmentation_rendering()
    r.close()
    return out


def run(seed=3, layout='default'):
    m, gt = build(seed, layout)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(SETTLE):                  # settle (dt = 1 ms): 6 s default, 20 s sink6
        mujoco.mj_step(m, d)
    vmax = float(np.abs(d.qvel).max())
    rgb, depth, seg = render(m, d)
    bid = {g['name']: m.body(g['name']).id for g in gt}
    for g in gt:
        xm = d.xmat[bid[g['name']]].reshape(3, 3)
        g['pos'] = [round(float(v), 5) for v in d.xpos[bid[g['name']]]]
        g['axis'] = [round(float(v), 4) for v in xm[:, 2]]
        g['tilt_deg'] = round(math.degrees(math.acos(min(1., abs(xm[2, 2])))), 1)
        g['in_basin'] = bool(SX0 < g['pos'][0] < SX1 and SY0 < g['pos'][1] < SY1 and g['pos'][2] < CZ)
        mask = (seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(seg[..., 0], np.where(m.geom_bodyid == bid[g['name']])[0])
        ys, xs = np.nonzero(mask)
        g['box'] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())] if len(xs) else None
        g['visible_px'] = int(mask.sum())
    # physical support test: remove one dish, step 0.6 s, see which other dishes move
    dishes = [g for g in gt if g['dish']]
    q0, v0 = d.qpos.copy(), d.qvel.copy()
    # control run (nothing removed): residual drift is subtracted so a still-creeping pile does not fake support
    p0 = {o['name']: d.xpos[bid[o['name']]].copy() for o in dishes}
    for _ in range(600):
        mujoco.mj_step(m, d)
    drift = {o['name']: float(np.linalg.norm(d.xpos[bid[o['name']]] - p0[o['name']])) * 1000 for o in dishes}
    for k, g in enumerate(dishes):
        mujoco.mj_resetData(m, d)
        d.qpos[:], d.qvel[:] = q0, v0
        ja = m.jnt_qposadr[m.body(g['name']).jntadr[0]]
        d.qpos[ja:ja + 3] = [2.5, -2.0 + .4 * k, .3]
        mujoco.mj_forward(m, d)
        p0 = {o['name']: d.xpos[bid[o['name']]].copy() for o in dishes}
        for _ in range(600):
            mujoco.mj_step(m, d)
        mv = {o['name']: round(float(np.linalg.norm(d.xpos[bid[o['name']]] - p0[o['name']])) * 1000, 1) for o in dishes if o is not g}
        g['removal_moves_mm'] = mv
        g['control_drift_mm'] = round(drift[g['name']], 1)
        g['supports'] = [nm for nm, v in mv.items() if v - drift[nm] > 5.]
    for g in dishes:
        g['rests_on'] = [o['name'] for o in dishes if g['name'] in o['supports']]   # g moves when any of these is removed
    cid = m.camera('sinkcam').id
    mujoco.mj_resetData(m, d)
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)
    from PIL import Image
    Image.fromarray(rgb).save(ROOT / 'out' / f'{PREFIX}.png')
    rng = np.random.default_rng(1)
    depth_n = depth + rng.normal(0, .0015, depth.shape) * depth
    np.savez_compressed(ROOT / 'out' / f'{PREFIX}_rgbd.npz', rgb=rgb, depth=depth_n.astype(np.float32), seg=seg, cam_pos=d.cam_xpos[cid],
                        cam_mat=d.cam_xmat[cid].reshape(3, 3), f=(H / 2) / math.tan(math.radians(FOVY) / 2),
                        geom_body=m.geom_bodyid, body_names=np.array([m.body(i).name for i in range(m.nbody)]))
    (ROOT / 'out' / f'{PREFIX}_gt.json').write_text(json.dumps(dict(seed=seed, layout=layout, items=gt, max_qvel_after_settle=vmax), indent=1))
    for g in gt:
        print(g['name'], g['box'], g['visible_px'], g['pos'], 'tilt', g['tilt_deg'], 'basin', g['in_basin'],
              'supports', g.get('supports'), 'rests_on', g.get('rests_on'))
    print('max |qvel| after settle', vmax)


if __name__ == '__main__':
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 3, sys.argv[2] if len(sys.argv) > 2 else 'default')
