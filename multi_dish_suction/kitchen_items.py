"""Countertop items for the sorting task (MjSpec builders, primitives only, all free bodies).

Dishwasher-safe:  plate (8", identical geometry to ../full_pipeline scene._add_plate), bowl, mug.
Not dishwasher-safe: wooden cutting board, chef's knife, cast-iron pan, vacuum flask.

Soil is VISUAL ONLY: thin decal geoms (contype=conaffinity=0, mass 0) attached to the item body, so they never
change the physics. Two independent cues:
  * food soil, severity 'light' / 'medium' / 'heavy' (count + size of brown / red / yellow spots, crust);
  * grease (bool): translucent amber glossy film patches.
Dimensions are EST (typical kitchenware) unless noted.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

COUNTER_Z = 0.90
OBJ = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])
VIS = dict(contype=0, conaffinity=0, mass=0, group=1)
BOX, CYL, CAP = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE

# ---- plate (../full_pipeline/scene.py, unchanged numbers)
R_OUT, R_DISC, R_FOOT, PL_T, RIM_RISE = .1015, .065, .060, .005, .0098
SLOPE = math.atan2(RIM_RISE, R_OUT - R_DISC)
PLATE_MASS = dict(disc=.15, rim=.27, foot=.03)
MID_Z = .0105
# ---- mug (EST: large-handled 330 ml ceramic mug)
MUG_R, MUG_H, MUG_T, MUG_BOT = .041, .095, .005, .007
MUG_HOLE = .030                     # radial gap body -> handle bar (room for the inner finger)
MUG_BAR_T, MUG_BAR_W = .010, .014   # bar radial thickness, tangential width
MUG_BAR_R = MUG_R + MUG_HOLE + MUG_BAR_T / 2       # bar centre radius 0.076
MUG_BAR_Z = (.022, .078)            # bar span (height above the mug foot)
MUG_MASS = .32
# ---- bowl (EST: 13 cm cereal bowl)
BOWL_RT, BOWL_RB, BOWL_H, BOWL_T = .065, .036, .060, .004
BOWL_MASS = .26

WHITE = [.96, .96, .93, 1]
SOIL_COLORS = {'sauce': [.62, .16, .08, 1], 'gravy': [.42, .26, .12, 1], 'egg': [.92, .72, .18, 1],
               'coffee': [.30, .18, .10, 1], 'crumb': [.55, .40, .22, 1], 'green': [.25, .42, .15, 1]}


def q_yaw(a):
    return [math.cos(a / 2), 0., 0., math.sin(a / 2)]


def quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


class Decals:
    """Visual soil generator for one item. rng-driven; returns a GT summary."""

    def __init__(self, spec, body, name, rng):
        self.spec, self.b, self.n, self.rng, self.k = spec, body, name, rng, 0
        self._mat_done = set()

    def mat(self, kind):
        nm = f'mat_{kind}'
        if nm not in {m.name for m in self.spec.materials}:
            if kind == 'grease':
                self.spec.add_material(name=nm, rgba=[.78, .58, .22, .42], specular=1., shininess=1., reflectance=.25)
            else:
                self.spec.add_material(name=nm, rgba=[1, 1, 1, 1], specular=.15, shininess=.2)
        return nm

    def spot(self, pos, normal, r, color, h=.0004, grease=False, aspect=1.0, yaw=0.):
        """Flat ellipse (cylinder) lying on a surface with the given outward normal."""
        n = np.asarray(normal, float)
        n /= np.linalg.norm(n)
        a = np.cross(n, [0, 0, 1.]) if abs(n[2]) < .9 else np.array([1., 0, 0])
        a = a - (a @ n) * n
        a /= np.linalg.norm(a)
        c, s = math.cos(yaw), math.sin(yaw)
        a2 = c * a + s * np.cross(n, a)
        R = np.column_stack([a2, np.cross(n, a2), n])
        self.k += 1
        g = self.b.add_geom(name=f'{self.n}_soil{self.k}', type=CYL, pos=list(np.asarray(pos) + n * h), quat=list(quat(R)),
                            size=[r, h, 0], rgba=color if not grease else [.78, .58, .22, .42], **VIS)
        g.material = self.mat('grease' if grease else 'soil')
        if aspect != 1.0:                       # ellipse: a squashed box-capped cylinder is not available -> 2 overlapping discs
            self.k += 1
            off = a2 * r * (aspect - 1) * .6
            self.b.add_geom(name=f'{self.n}_soil{self.k}', type=CYL, pos=list(np.asarray(pos) + off + n * h), quat=list(quat(R)),
                            size=[r * .9, h, 0], rgba=color if not grease else [.78, .58, .22, .42], material=g.material, **VIS)
        return g


def soil_spec(level, rng):
    """(n_spots, r_range m, crust) for a soil level."""
    return {'light': (rng.integers(1, 3), (.004, .008), False),
            'medium': (rng.integers(4, 7), (.007, .014), False),
            'heavy': (rng.integers(8, 12), (.012, .024), True)}[level]


def _disc_points(rng, rmax, n, rmin=0.):
    out = []
    for _ in range(n):
        r = math.sqrt(rng.uniform((rmin / rmax) ** 2, 1)) * rmax
        a = rng.uniform(0, 2 * math.pi)
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


# ====================================================================== dishwasher-safe
def add_plate(spec, name, xy, yaw, soil, greasy, rng):
    plate = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    plate.add_freejoint(name=f'{name}_free')
    plate.add_geom(name=f'{name}_disc', type=CYL, pos=[0, 0, .0075], size=[R_DISC + .002, PL_T / 2, 0],
                   mass=PLATE_MASS['disc'], rgba=WHITE, **OBJ)
    n = 36
    width = R_OUT - R_DISC
    half_len = .5 * math.hypot(width, RIM_RISE)
    for i in range(n):
        a = 2 * math.pi * i / n
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.cos(SLOPE) * radial + np.array([0, 0, math.sin(SLOPE)])
        normal = np.cross(along, tangent)
        centre = (R_DISC + width / 2) * radial + np.array([0, 0, .0075 + RIM_RISE / 2])
        plate.add_geom(name=f'{name}_rim{i}', type=BOX, pos=list(centre), quat=quat(np.column_stack([along, tangent, normal])),
                       size=[half_len, (R_DISC + width / 2) * math.sin(math.pi / n) * 1.15, PL_T / 2],
                       mass=PLATE_MASS['rim'] / n, rgba=WHITE, **OBJ)
    m = 16
    for i in range(m):
        a = 2 * math.pi * i / m
        plate.add_geom(name=f'{name}_foot{i}', type=BOX, pos=[R_FOOT * math.cos(a), R_FOOT * math.sin(a), .0025],
                       quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], size=[.002, R_FOOT * math.sin(math.pi / m) * 1.1, .0025],
                       mass=PLATE_MASS['foot'] / m, rgba=WHITE, **OBJ)
    # decorative blue band on the rim (real dinnerware look; helps the VLM see the plate outline)
    plate.add_geom(name=f'{name}_band', type=CYL, pos=[0, 0, .0101], size=[R_DISC + .0015, .0002, 0], rgba=[.80, .84, .90, 1], **VIS)
    plate.add_geom(name=f'{name}_band2', type=CYL, pos=[0, 0, .01025], size=[R_DISC - .002, .0002, 0], rgba=WHITE, **VIS)
    plate.add_site(name=f'{name}_center', pos=[0, 0, MID_Z], size=[.003, 0, 0], rgba=[1, .3, .1, 1], group=4)
    D = Decals(spec, plate, name, rng)
    top = .0105
    ns, (r0, r1), crust = soil_spec(soil, rng)
    cols = ['sauce', 'gravy', 'green', 'crumb', 'sauce']
    for (x, y) in _disc_points(rng, R_DISC - .012, ns):
        D.spot([x, y, top], [0, 0, 1], rng.uniform(r0, r1), SOIL_COLORS[cols[rng.integers(len(cols))]], aspect=rng.uniform(1, 1.6),
               yaw=rng.uniform(0, 6.3))
    if crust:                                   # dried-on smear crossing onto the rim
        for (x, y) in _disc_points(rng, R_DISC - .01, 3):
            D.spot([x, y, top + .0012], [0, 0, 1], rng.uniform(.018, .026), [.36, .20, .09, 1], h=.0008, aspect=1.5, yaw=rng.uniform(0, 6.3))
    if greasy:
        for (x, y) in _disc_points(rng, R_DISC - .02, int(rng.integers(2, 4))):
            D.spot([x, y, top + .0006], [0, 0, 1], rng.uniform(.018, .03), None, grease=True, h=.0003)
    return plate


def _tapered_shell(body, name, n, r_bot, r_top, z_bot, z_top, t, mass, rgba):
    sl = math.atan2(z_top - z_bot, r_top - r_bot)
    hl = .5 * math.hypot(r_top - r_bot, z_top - z_bot)
    rm, zm = (r_bot + r_top) / 2, (z_bot + z_top) / 2
    for i in range(n):
        a = 2 * math.pi * i / n
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.cos(sl) * radial + np.array([0, 0, math.sin(sl)])
        normal = np.cross(along, tangent)
        body.add_geom(name=f'{name}_w{i}', type=BOX, pos=list(rm * radial + [0, 0, zm]), quat=quat(np.column_stack([along, tangent, normal])),
                      size=[hl, rm * math.sin(math.pi / n) * 1.12, t / 2], mass=mass / n, rgba=rgba, **OBJ)


def add_bowl(spec, name, xy, yaw, soil, greasy, rng, color=(.93, .91, .86, 1)):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    foot_h = .008
    b.add_geom(name=f'{name}_bottom', type=CYL, pos=[0, 0, foot_h + .002], size=[BOWL_RB, .002, 0], mass=.05, rgba=list(color), **OBJ)
    _tapered_shell(b, name, 28, BOWL_RB, BOWL_RT, foot_h, BOWL_H, BOWL_T, BOWL_MASS - .07, list(color))
    for i in range(12):
        a = 2 * math.pi * i / 12
        b.add_geom(name=f'{name}_foot{i}', type=BOX, pos=[.030 * math.cos(a), .030 * math.sin(a), foot_h / 2],
                   quat=q_yaw(a), size=[.002, .030 * math.sin(math.pi / 12) * 1.1, foot_h / 2], mass=.02 / 12, rgba=list(color), **OBJ)
    b.add_site(name=f'{name}_center', pos=[0, 0, BOWL_H / 2], size=[.003, 0, 0], group=4)
    D = Decals(spec, b, name, rng)
    ns, (r0, r1), crust = soil_spec(soil, rng)
    cols = ['sauce', 'gravy', 'crumb', 'green']
    sl = math.atan2(BOWL_H - foot_h, BOWL_RT - BOWL_RB)
    for k in range(ns):                         # spots on the inner wall (visible from above) and the bottom
        if rng.uniform() < .35:
            x, y = _disc_points(rng, BOWL_RB - .006, 1)[0]
            D.spot([x, y, foot_h + .004], [0, 0, 1], rng.uniform(min(r0, .014 * .7), min(r1, .014)), SOIL_COLORS[cols[rng.integers(len(cols))]])
        else:
            a = rng.uniform(0, 2 * math.pi)
            f = rng.uniform(.25, .9)
            r = BOWL_RB + f * (BOWL_RT - BOWL_RB) - BOWL_T / 2 / math.sin(sl)
            z = foot_h + f * (BOWL_H - foot_h)
            nrm = np.array([-math.sin(sl) * math.cos(a), -math.sin(sl) * math.sin(a), math.cos(sl)])
            D.spot([r * math.cos(a), r * math.sin(a), z], nrm, rng.uniform(min(r0, .016 * .7), min(r1, .016)), SOIL_COLORS[cols[rng.integers(len(cols))]])
    if crust:
        D.spot([0, 0, foot_h + .0045], [0, 0, 1], BOWL_RB - .004, SOIL_COLORS['gravy'], h=.0008)
    if greasy:
        for _ in range(int(rng.integers(2, 4))):
            a = rng.uniform(0, 2 * math.pi)
            f = rng.uniform(.3, .8)
            r = BOWL_RB + f * (BOWL_RT - BOWL_RB) - BOWL_T / 2 / math.sin(sl)
            nrm = np.array([-math.sin(sl) * math.cos(a), -math.sin(sl) * math.sin(a), math.cos(sl)])
            D.spot([r * math.cos(a), r * math.sin(a), foot_h + f * (BOWL_H - foot_h) + .001], nrm, rng.uniform(.014, .02), None, grease=True)
    return b


def add_mug(spec, name, xy, yaw, soil, greasy, rng, color=(.86, .30, .22, 1)):
    """Body local: z up from the foot, handle along local +x."""
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    col = list(color)
    inner = [.95, .95, .92, 1]
    b.add_geom(name=f'{name}_bottom', type=CYL, pos=[0, 0, MUG_BOT / 2], size=[MUG_R - .001, MUG_BOT / 2, 0], mass=.08, rgba=col, **OBJ)
    n = 22
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = MUG_R - MUG_T / 2
        b.add_geom(name=f'{name}_w{i}', type=BOX, pos=[rc * math.cos(a), rc * math.sin(a), MUG_H / 2], quat=q_yaw(a),
                   size=[MUG_T / 2, rc * math.sin(math.pi / n) * 1.12, MUG_H / 2], mass=(MUG_MASS - .08 - .04) / n, rgba=col, **OBJ)
    # white glaze inside (visual)
    b.add_geom(name=f'{name}_glaze', type=CYL, pos=[0, 0, MUG_BOT + .0003], size=[MUG_R - MUG_T - .0005, .0003, 0], rgba=inner, **VIS)
    # handle: top + bottom connectors and the outer bar (local +x)
    z0, z1 = MUG_BAR_Z
    r_in = MUG_R - .003
    for k, z in enumerate((z0, z1)):
        hl = (MUG_BAR_R + MUG_BAR_T / 2 - r_in) / 2
        b.add_geom(name=f'{name}_hconn{k}', type=BOX, pos=[r_in + hl, 0, z], size=[hl, MUG_BAR_W / 2, .005], mass=.01, rgba=col, **OBJ)
    b.add_geom(name=f'{name}_hbar', type=BOX, pos=[MUG_BAR_R, 0, (z0 + z1) / 2], size=[MUG_BAR_T / 2, MUG_BAR_W / 2, (z1 - z0) / 2 + .005],
               mass=.02, rgba=col, **OBJ)
    b.add_site(name=f'{name}_center', pos=[0, 0, MUG_H / 2], size=[.003, 0, 0], group=4)
    b.add_site(name=f'{name}_handle', pos=[MUG_BAR_R, 0, (z0 + z1) / 2], size=[.003, 0, 0], group=4)
    D = Decals(spec, b, name, rng)
    ns, (r0, r1), crust = soil_spec(soil, rng)
    # coffee ring / drips: inside bottom and on the inner wall near the rim, drips outside
    D.spot([0, 0, MUG_BOT + .0008], [0, 0, 1], MUG_R - MUG_T - .004, SOIL_COLORS['coffee'] if soil != 'light' else [.55, .40, .28, 1])
    for k in range(ns):
        a = rng.uniform(0, 2 * math.pi)
        if k % 2 == 0:              # inner wall
            r = MUG_R - MUG_T - .0005
            D.spot([r * math.cos(a), r * math.sin(a), rng.uniform(.06, .09)], [-math.cos(a), -math.sin(a), 0], rng.uniform(min(r0, .012 * .7), min(r1, .012)),
                   SOIL_COLORS['coffee'])
        else:                       # outer drip
            r = MUG_R + .0002
            D.spot([r * math.cos(a), r * math.sin(a), rng.uniform(.03, .085)], [math.cos(a), math.sin(a), 0], rng.uniform(min(r0, .010 * .7), min(r1, .010)),
                   SOIL_COLORS['coffee'], aspect=1.5, yaw=math.pi / 2)
    if crust:
        for _ in range(3):
            a = rng.uniform(0, 2 * math.pi)
            r = MUG_R - MUG_T - .0005
            D.spot([r * math.cos(a), r * math.sin(a), .088], [-math.cos(a), -math.sin(a), 0], .010, SOIL_COLORS['gravy'], h=.0008)
    if greasy:
        for _ in range(2):
            a = rng.uniform(0, 2 * math.pi)
            r = MUG_R + .0003
            D.spot([r * math.cos(a), r * math.sin(a), rng.uniform(.04, .08)], [math.cos(a), math.sin(a), 0], .013, None, grease=True)
    return b


# ====================================================================== NOT dishwasher-safe
def add_board(spec, name, xy, yaw, soil, greasy, rng):
    L, Wd, T = .32, .20, .018
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    wood = [.74, .55, .33, 1]
    b.add_geom(name=f'{name}_slab', type=BOX, pos=[0, 0, T / 2], size=[L / 2, Wd / 2, T / 2], mass=.9, rgba=wood, **OBJ)
    for k in range(9):                          # wood grain (visual)
        y = -Wd / 2 + .015 + k * (Wd - .03) / 8 + rng.uniform(-.005, .005)
        b.add_geom(name=f'{name}_grain{k}', type=BOX, pos=[rng.uniform(-.02, .02), y, T + .0002], size=[L / 2 - .03, .0012, .0002],
                   rgba=[.58, .40, .22, 1], **VIS)
    b.add_geom(name=f'{name}_hole', type=CYL, pos=[L / 2 - .035, 0, T + .0003], size=[.012, .0003, 0], rgba=[.20, .14, .08, 1], **VIS)
    b.add_geom(name=f'{name}_hole2', type=CYL, pos=[L / 2 - .052, 0, T + .0003], size=[.012, .0003, 0], rgba=[.20, .14, .08, 1], **VIS)
    D = Decals(spec, b, name, rng)
    ns, (r0, r1), _ = soil_spec(soil, rng)
    for k in range(ns):                         # chopped vegetable bits
        x, y = rng.uniform(-L / 2 + .04, L / 2 - .07), rng.uniform(-Wd / 2 + .03, Wd / 2 - .03)
        D.spot([x, y, T], [0, 0, 1], rng.uniform(.004, .009), SOIL_COLORS[['green', 'sauce', 'egg'][k % 3]], h=.002)
    return b


def add_knife(spec, name, xy, yaw, soil, greasy, rng):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    steel = [.80, .82, .85, 1]
    if 'mat_steel' not in {m.name for m in spec.materials}:
        spec.add_material(name='mat_steel', rgba=steel, specular=1., shininess=.9, reflectance=.3)
    # blade 200 mm (spine along local +y side), tapering to the tip at local +x
    b.add_geom(name=f'{name}_blade', type=BOX, pos=[.07, 0, .011], size=[.07, .022, .001], mass=.10, material='mat_steel', **OBJ)
    tip = b.add_geom(name=f'{name}_tip', type=BOX, pos=[.165, .006, .011], quat=q_yaw(math.radians(12)), size=[.036, .015, .001],
                     mass=.02, material='mat_steel', **OBJ)
    b.add_geom(name=f'{name}_bolster', type=BOX, pos=[-.006, 0, .011], size=[.006, .016, .007], mass=.03, material='mat_steel', **OBJ)
    b.add_geom(name=f'{name}_handle', type=BOX, pos=[-.07, 0, .011], size=[.058, .013, .010], mass=.07, rgba=[.08, .08, .09, 1], **OBJ)
    for k, x in enumerate((-.10, -.07, -.04)):
        b.add_geom(name=f'{name}_rivet{k}', type=CYL, pos=[x, 0, .0212], size=[.003, .0003, 0], material='mat_steel', **VIS)
    # supports so the blade lies flat (the handle is thicker than the blade): thin pads under the blade
    b.add_geom(name=f'{name}_padA', type=BOX, pos=[.10, 0, .005], size=[.004, .004, .005], mass=.001, rgba=[0, 0, 0, 0], **OBJ)
    D = Decals(spec, b, name, rng)
    for k in range(2 if soil != 'light' else 1):
        D.spot([rng.uniform(.03, .15), rng.uniform(-.012, .012), .012], [0, 0, 1], rng.uniform(.004, .007), SOIL_COLORS['sauce'])
    return b


def add_pan(spec, name, xy, yaw, soil, greasy, rng):
    """Cast-iron skillet, 25 cm, handle along local +x."""
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    if 'mat_iron' not in {m.name for m in spec.materials}:
        spec.add_material(name='mat_iron', rgba=[.13, .13, .14, 1], specular=.35, shininess=.4)
    Rb, Rt, Hh = .105, .125, .048
    b.add_geom(name=f'{name}_bottom', type=CYL, pos=[0, 0, .004], size=[Rb + .004, .004, 0], mass=1.0, material='mat_iron', **OBJ)
    sl = math.atan2(Hh, Rt - Rb)
    hl = .5 * math.hypot(Rt - Rb, Hh)
    n = 30
    for i in range(n):
        a = 2 * math.pi * i / n
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.cos(sl) * radial + np.array([0, 0, math.sin(sl)])
        nrm = np.cross(along, tangent)
        b.add_geom(name=f'{name}_w{i}', type=BOX, pos=list((Rb + Rt) / 2 * radial + [0, 0, Hh / 2]),
                   quat=quat(np.column_stack([along, tangent, nrm])), size=[hl, (Rb + Rt) / 2 * math.sin(math.pi / n) * 1.12, .003],
                   mass=.8 / n, material='mat_iron', **OBJ)
    b.add_geom(name=f'{name}_handle', type=BOX, pos=[Rt + .075, 0, Hh - .008], quat=quat(np.array([[math.cos(.12), 0, -math.sin(.12)], [0, 1, 0], [math.sin(.12), 0, math.cos(.12)]])),
               size=[.08, .014, .009], mass=.35, material='mat_iron', **OBJ)
    b.add_geom(name=f'{name}_lug', type=BOX, pos=[-Rt - .022, 0, Hh - .006], size=[.022, .025, .005], mass=.08, material='mat_iron', **OBJ)
    b.add_geom(name=f'{name}_hole', type=CYL, pos=[Rt + .135, 0, Hh - .0005], size=[.006, .0004, 0], rgba=[.85, .82, .78, 1], **VIS)
    D = Decals(spec, b, name, rng)
    ns, (r0, r1), crust = soil_spec(soil, rng)
    for (x, y) in _disc_points(rng, Rb - .02, ns):
        D.spot([x, y, .008], [0, 0, 1], rng.uniform(r0, r1), SOIL_COLORS[['gravy', 'egg', 'crumb'][int(rng.integers(3))]])
    if greasy:
        for (x, y) in _disc_points(rng, Rb - .03, 3):
            D.spot([x, y, .0086], [0, 0, 1], rng.uniform(.025, .04), None, grease=True, h=.0003)
    return b


def add_flask(spec, name, xy, yaw, soil, greasy, rng):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    if 'mat_steel' not in {m.name for m in spec.materials}:
        spec.add_material(name='mat_steel', rgba=[.80, .82, .85, 1], specular=1., shininess=.9, reflectance=.3)
    R, H = .036, .205
    b.add_geom(name=f'{name}_body', type=CYL, pos=[0, 0, H / 2], size=[R, H / 2, 0], mass=.42, material='mat_steel', **OBJ)
    b.add_geom(name=f'{name}_band', type=CYL, pos=[0, 0, .06], size=[R + .0005, .025, 0], rgba=[.12, .36, .30, 1], **VIS)
    b.add_geom(name=f'{name}_shoulder', type=CYL, pos=[0, 0, H + .006], size=[R - .006, .006, 0], mass=.03, material='mat_steel', **OBJ)
    b.add_geom(name=f'{name}_cap', type=CYL, pos=[0, 0, H + .012 + .02], size=[R - .004, .02, 0], mass=.06, rgba=[.08, .08, .09, 1], **OBJ)
    b.add_geom(name=f'{name}_capring', type=CYL, pos=[0, 0, H + .0525], size=[.012, .0006, 0], rgba=[.6, .6, .62, 1], **VIS)
    b.add_geom(name=f'{name}_logo', type=BOX, pos=[R + .0003, 0, .13], size=[.0003, .012, .006], rgba=[.15, .15, .16, 1], **VIS)
    return b


BUILDERS = dict(plate=add_plate, bowl=add_bowl, mug=add_mug, board=add_board, knife=add_knife, pan=add_pan, flask=add_flask)
SAFE = dict(plate=True, bowl=True, mug=True, board=False, knife=False, pan=False, flask=False)
# footprint radius (m) around the body origin used for layout spacing (EST: bounding circle of the top view)
FOOT_R = dict(plate=.1015, bowl=.066, mug=.06, board=.19, knife=.13, pan=.22, flask=.037)
# footprint centre offset along local +x (the long side of pan / knife / mug handle)
FOOT_OFF = dict(plate=0., bowl=0., mug=.012, board=0., knife=.03, pan=.06, flask=0.)
TRUE_NAME = dict(plate='dinner plate', bowl='bowl', mug='mug', board='wooden cutting board', knife="chef's knife",
                 pan='cast-iron pan', flask='vacuum flask')
