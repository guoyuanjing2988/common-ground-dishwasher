"""Messy kitchen sink right of the dishwasher: recessed steel basin (walls + drain), faucet, soap, sponge, dish cloth,
cutting board against the wall, cup with chopsticks. Dishes are DROPPED into the basin from perturbed poses and
settle under mj_step (no pose is written after t = 0)."""
from __future__ import annotations

import math

import mujoco
import numpy as np

import dishwasher as dw
import items as IT
import kitchen as K
import multi as M
import scene as S

BOX, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
# basin (EST: 30 x 65 cm bar sink, 12 cm deep), interior
BX0, BX1, BY0, BY1 = -0.36, -0.06, 0.30, 0.95
FLOOR_Z = 0.78
WALL_T = .02
STEEL = [.72, .74, .77, 1]
LEAN = math.radians(25)


def _cut_counter(spec):
    """Replace counter top + cabinet by pieces that leave the sink opening."""
    for nm in ('counter_top', 'cabinet'):
        spec.delete(spec.geom(nm))
    wb = spec.worldbody
    z, t = S.COUNTER_Z, S.COUNTER_T
    hx0, hx1, hy0, hy1 = BX0 - WALL_T, BX1 + WALL_T, BY0 - WALL_T, BY1 + WALL_T
    y0, y1 = S.COUNTER_Y
    pieces = [((-S.COUNTER_D, y0), (0., hy0)), ((-S.COUNTER_D, hy1), (0., y1)),
              ((-S.COUNTER_D, hy0), (hx0, hy1)), ((hx1, hy0), (0., hy1))]
    for k, ((xa, ya), (xb, yb)) in enumerate(pieces):
        wb.add_geom(name=f'counter_top{k}', type=BOX, pos=[(xa + xb) / 2, (ya + yb) / 2, z - t / 2],
                    size=[(xb - xa) / 2, (yb - ya) / 2, t / 2], rgba=S.COUNTER_RGBA, **S.ENV_CONTACT)
    cz0, cz1 = S.TOE_H, z - t
    cx0, cx1 = -S.COUNTER_D + .02, S.CAB_FRONT_X
    for k, (ya, yb, top) in enumerate([(y0 + .01, hy0, cz1), (hy0, hy1, FLOOR_Z - .06), (hy1, y1 - .01, cz1)]):
        wb.add_geom(name=f'cabinet{k}', type=BOX, pos=[(cx0 + cx1) / 2, (ya + yb) / 2, (cz0 + top) / 2],
                    size=[(cx1 - cx0) / 2, (yb - ya) / 2, (top - cz0) / 2], rgba=S.CABINET_RGBA, **S.ENV_CONTACT)
    # basin: floor, 4 walls, drain (visual), rim lip
    fz = FLOOR_Z
    wb.add_geom(name='sink_floor', type=BOX, pos=[(BX0 + BX1) / 2, (BY0 + BY1) / 2, fz - .005],
                size=[(BX1 - BX0) / 2 + WALL_T, (BY1 - BY0) / 2 + WALL_T, .005], rgba=STEEL, **S.ENV_CONTACT)
    hz = (z - fz) / 2
    for nm, c, sz in (('sink_wall_back', [BX0 - WALL_T / 2, (BY0 + BY1) / 2], [WALL_T / 2, (BY1 - BY0) / 2 + WALL_T]),
                      ('sink_wall_front', [BX1 + WALL_T / 2, (BY0 + BY1) / 2], [WALL_T / 2, (BY1 - BY0) / 2 + WALL_T]),
                      ('sink_wall_l', [(BX0 + BX1) / 2, BY0 - WALL_T / 2], [(BX1 - BX0) / 2, WALL_T / 2]),
                      ('sink_wall_r', [(BX0 + BX1) / 2, BY1 + WALL_T / 2], [(BX1 - BX0) / 2, WALL_T / 2])):
        wb.add_geom(name=nm, type=BOX, pos=[c[0], c[1], fz + hz], size=[sz[0], sz[1], hz], rgba=STEEL, **S.ENV_CONTACT)
    wb.add_geom(name='sink_drain', type=CYL, pos=[(BX0 + BX1) / 2 - .02, (BY0 + BY1) / 2 + .1, fz + .0004], size=[.035, .0004, 0],
                rgba=[.25, .26, .28, 1], contype=0, conaffinity=0)
    wb.add_geom(name='sink_drain_c', type=CYL, pos=[(BX0 + BX1) / 2 - .02, (BY0 + BY1) / 2 + .1, fz + .0008], size=[.012, .0004, 0],
                rgba=[.08, .08, .09, 1], contype=0, conaffinity=0)


def _accessories(spec):
    wb = spec.worldbody
    chrome = [.82, .84, .87, 1]
    # faucet behind the basin (base + column + spout over the basin + lever)
    fx, fy = BX0 - WALL_T - .05, BY0 + .07
    f = wb.add_body(name='faucet', pos=[fx, fy, S.COUNTER_Z])
    f.add_geom(name='faucet_base', type=CYL, pos=[0, 0, .01], size=[.03, .01, 0], rgba=chrome, **S.ENV_CONTACT)
    f.add_geom(name='faucet_col', type=CYL, pos=[0, 0, .14], size=[.015, .13, 0], rgba=chrome, **S.ENV_CONTACT)
    f.add_geom(name='faucet_spout', type=CYL, fromto=[0, 0, .27, .13, 0, .27], size=[.012, 0, 0], rgba=chrome, contype=0, conaffinity=0)
    f.add_geom(name='faucet_tip', type=CYL, fromto=[.13, 0, .27, .13, 0, .23], size=[.012, 0, 0], rgba=chrome, contype=0, conaffinity=0)
    f.add_geom(name='faucet_lever', type=BOX, pos=[-.02, 0, .23], size=[.035, .006, .005], rgba=chrome, contype=0, conaffinity=0)
    # dish cloth on the counter right of the sink, soap + sponge behind the basin
    K._simple(spec, 'cloth', (-0.20, 1.10), [(BOX, [0, 0, .003], [.11, .08, .003], .05, [.35, .55, .80, 1]),
                                            (BOX, [0, 0, .0062], [.11, .006, .0002], 0, [.95, .95, .95, 1])], yaw=math.radians(12))
    K._simple(spec, 'soap2', (BX0 - WALL_T - .06, BY1 - .06), [
        (BOX, [0, 0, .085], [.028, .018, .085], .45, [.85, .35, .55, 1]), (CYL, [0, 0, .182], [.011, .012, 0], .02, [.95, .95, .95, 1])])
    K._simple(spec, 'sponge2', (BX0 - WALL_T - .055, BY0 + .10), [
        (BOX, [0, 0, .012], [.045, .032, .012], .02, [.96, .83, .25, 1]), (BOX, [0, 0, .027], [.045, .032, .003], .005, [.15, .45, .20, 1])],
        yaw=math.radians(-20))
    # cup with chopsticks (visual chopsticks: sticks are welded-free visual geoms of the cup body)
    c = K._simple(spec, 'cup', (-0.47, 1.12), [(CYL, [0, 0, .05], [.035, .05, 0], .25, [.30, .45, .70, 1])])
    for k, (dx, dy, tx) in enumerate([(-.01, .006, .02), (.008, -.004, -.015), (.0, .012, .005)]):
        c.add_geom(name=f'chop{k}', type=CYL, fromto=[dx, dy, .02, dx + tx, dy + .03, .23], size=[.003, 0, 0], rgba=[.55, .35, .18, 1],
                   contype=0, conaffinity=0, mass=0)
    # cutting board leaning against the back wall (static, against the wall: decorative)
    wb.add_geom(name='board2', type=BOX, pos=[-0.585, 1.12, S.COUNTER_Z + .14], quat=[math.cos(-.12), 0, math.sin(-.12), 0],
                size=[.012, .11, .14], rgba=[.62, .45, .26, 1], **S.ENV_CONTACT)


SINK_CLUTTER = ['cloth', 'soap2', 'sponge2', 'cup']


def _lean_plate_pose(y, yaw_off=0.):
    """Plate standing on its rim on the basin floor, top leaning on the FRONT wall's inner top edge."""
    u = np.array([math.sin(LEAN), 0, math.cos(LEAN)])            # in-plane up
    n = np.array([-math.cos(LEAN), 0, math.sin(LEAN)])            # concave side faces the basin
    v = np.cross(u, n)
    Rz = np.array([[math.cos(yaw_off), -math.sin(yaw_off), 0], [math.sin(yaw_off), math.cos(yaw_off), 0], [0, 0, 1]])
    R = Rz @ np.column_stack([v, u, n])
    top_x = BX1 - .004
    centre = np.array([top_x - 2 * S.R_OUT * math.sin(LEAN) / 2 - .0, y, FLOOR_Z + .004 + S.R_OUT * math.cos(LEAN)])
    centre[0] = top_x - S.R_OUT * math.sin(LEAN)
    origin = centre - R[:, 2] * S.MID_Z + np.array([0, 0, .02])   # dropped from 2 cm
    return origin, R


SEED = 3


def build():
    rng = np.random.default_rng(SEED)
    spec = mujoco.MjSpec.from_string(S._header())
    S._add_kitchen(spec)
    _cut_counter(spec)
    S._add_camera(spec, M.LAYOUT)
    dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    site = S._add_robot(spec, M.MOUNT)
    S._attach_panda(spec, site)
    tints = [[.96, .96, .93, 1], [.97, .93, .86, 1], [.92, .95, .97, 1], [.95, .92, .90, 1]]
    j = lambda s_: rng.uniform(-s_, s_)
    # messy sink (sink_6.png approximated where it stays graspable): 2 plates slid down against the front wall with their rims
    # sticking up over its edge, a 3rd plate flat at the back with a bowl (opening up, tilted) on it, 2 bowls upside down
    # wedged between the plates and the back wall. Seeded roughly, settled by physics.
    for i, y in enumerate((0.43 + j(.01), 0.64 + j(.01))):
        org, R = _lean_plate_pose(y, j(math.radians(6)))
        S._add_plate(spec, None, 0., pos=org, R=R, name=f'plate{i}', rgba=tints[i])
    a = j(.2)
    S._add_plate(spec, None, 0., pos=[-0.215 + j(.01), 0.83 + j(.01), FLOOR_Z + .03],
                 R=np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]]) @ _rx(j(.08)),
                 name='plate2', rgba=tints[2])
    M.add_bowl(spec, 'bowl0', (-0.215 + j(.006), 0.44 + j(.01)))
    M.add_bowl(spec, 'bowl1', (-0.215 + j(.006), 0.62 + j(.01)))
    for nm, z in (('bowl0', FLOOR_Z + M.BOWL_HEIGHT + .03), ('bowl1', FLOOR_Z + M.BOWL_HEIGHT + .05)):
        spec.body(nm).pos = [spec.body(nm).pos[0], spec.body(nm).pos[1], z]
        spec.body(nm).quat = _quat_mul([0, 1, 0, 0], _axis_quat([1, 0, 0], j(.12)))
    M.add_bowl(spec, 'bowl2', (-0.20 + j(.01), 0.84 + j(.01)))
    b2 = spec.body('bowl2')
    b2.pos = [b2.pos[0], b2.pos[1], FLOOR_Z + .03 + .008 + .02]
    b2.quat = _axis_quat([1, 0, 0], .25 + j(.1))                    # opening UP, tilted, resting on plate 2
    K.add_clutter(spec)
    _accessories(spec)
    K.add_camera(spec)
    # sink camera: under the wall cabinet behind the sink, looking down into the basin (sees the leaning plates' faces)
    pos, look = np.array(SINKCAM_POS), np.array(SINKCAM_LOOK)
    fwd = unit_(look - pos)
    right = unit_(np.cross(fwd, [0, 0, 1.]))
    up = np.cross(right, fwd)
    spec.worldbody.add_camera(name='sinkcam', pos=list(pos), quat=list(S.quat(np.column_stack([right, up, -fwd]))), fovy=SINKCAM_FOVY)
    return spec.compile()


SINKCAM_POS, SINKCAM_LOOK, SINKCAM_FOVY = (-0.56, 0.66, 1.62), (-0.17, 0.63, 0.82), 62.


def unit_(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rx(a):
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _axis_quat(ax, a):
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    return [math.cos(a / 2), *(math.sin(a / 2) * ax)]


def _quat_mul(q1, q2):
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, np.asarray(q1, float), np.asarray(q2, float))
    return list(q)
