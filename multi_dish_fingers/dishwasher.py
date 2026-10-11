"""Countertop dishwasher (drop-down door, pull-out lower basket) for MuJoCo, built with MjSpec.

Usage:
    spec = mujoco.MjSpec.from_string(...)
    info = add_dishwasher(spec, pos=(x, y, z), yaw=0.0, prefix='dw/')
    model = spec.compile()

Local frame of the dishwasher (before `yaw`): origin = centre of the footprint on the
surface it stands on; +x = out of the front face (door opening direction); +y = to the
viewer's RIGHT when standing in front of the machine; +z = up.

Sources: "listing" = Taobao listing (outer 0.430 W x 0.390 D x 0.460 H m, 13 kg, drop-down
door with window, top-front touch band, wire basket on rollers, removable upper tray, spray
arm + cylindrical filter on the tub floor, capacity claim 8 plates + 8 bowls).
"seller" = seller answer 2026-10-10 (one pod on tub floor under the rack, or in a groove on
the inner door). Every other number is an estimate and carries a `# EST:` comment.
Primitives only (box / capsule / cylinder); no meshes.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

BOX, CAP, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER

# ---------------------------------------------------------------- overall (listing)
W, D, H = 0.430, 0.390, 0.460          # outer width (y), depth (x), height (z) - listing
MASS_TOTAL = 13.0                       # net mass, kg - listing
X_FRONT = D / 2                         # outer front face (door outer face / control band face)
# ---------------------------------------------------------------- shell
T = 0.015                               # EST: wall thickness, typical moulded PP shell (task spec ~0.015)
FOOT_H, FOOT_R = 0.015, 0.015           # EST: rubber feet, photo proportion
DOOR_T = 0.035                          # EST: door thickness incl. double window, photo proportion
X_OPEN = X_FRONT - DOOR_T               # 0.160: plane of the door inner face when closed
GAP = 0.001                             # EST: clearance between closed door and tub front edge
X_TUB = X_OPEN - GAP                    # 0.159: front-most x of any fixed part behind the door
HINGE_Z = 0.048                         # EST: hinge axis height; set so the open door's pull bar clears the support surface by 10 mm
DOOR_TOP = 0.400                        # EST: top edge of door; control band above, photo proportion
BAND_Z0 = 0.405                         # EST: bottom of control band (5 mm shut gap)
TUB_FLOOR = 0.075                       # EST: tub floor top; pump/sump/heater below, photo proportion
TUB_CEIL = H - T                        # 0.445
TUB_BACK = -X_FRONT + T                 # -0.180
TUB_Y = W / 2 - T                       # 0.200 (inner half-width)
# ---------------------------------------------------------------- door
DOOR_H = DOOR_TOP - HINGE_Z             # 0.370
DOOR_HW = W / 2 - 0.002                 # EST: door half-width 0.213 (2 mm side gap)
RAIL_B, RAIL_T = 0.100, 0.070           # EST: bottom / top rail heights (door-local z), photo proportion
WIN_HW = 0.160                          # EST: window half-width -> 0.32 x 0.20 m "large viewing window"
WIN_T = 0.004                           # EST: window pane thickness (one pane modelled)
HANDLE_Z = DOOR_H - 0.035               # EST: pull-bar height, 35 mm below the top edge
HANDLE_OFF = 0.030                      # EST: bar centre stand-off from outer face (finger room)
HANDLE_R, HANDLE_HL = 0.008, 0.100      # EST: bar radius, half-length (typical pull bar)
GROOVE = (0.045, 0.035, 0.012)          # task spec: groove interior (along door width, along door height, depth)
GROOVE_WALL = 0.002                     # EST: groove wall/floor thickness
GROOVE_Y, GROOVE_Z = -0.100, 0.055      # EST: groove centre on door (viewer's left, in bottom rail)
LINER = 0.012                           # inner-liner layer thickness = groove depth
DOOR_MASS = dict(frame=1.10, window=0.40, handle=0.06, groove=0.04)   # EST: 1.6 kg door (moulded PP + glass)
DOOR_DAMPING = 1.6                      # N m s/rad, chosen in MODEL_NOTES (slow drift, no slam)
DOOR_FRICTION = 0.30                    # EST: hinge + gasket friction, N m
DOOR_ARMATURE = 0.005                   # EST: hinge/spring rotor inertia, kg m^2
DOOR_NET_CLOSED = -0.10                 # design: net torque at 0 deg  = -0.10 * m g z_com (keeps door shut)
DOOR_NET_OPEN = +0.25                   # design: net torque at 90 deg = +0.25 * m g z_com (rests on stop)
# ---------------------------------------------------------------- lower basket
BASKET_X0 = -0.010                      # basket centre x when pushed in (back -0.170, front +0.150)
BASKET_HX, BASKET_HY = 0.160, 0.185     # EST: half depth / half width (fits 0.34 x 0.40 tub interior)
BASKET_Z = 0.121                        # EST: floor-wire centreline; rollers clear the open door's inner face (z 0.083)
BASKET_WALL_H = 0.100                   # EST: side wall height, typical
BASKET_TRAVEL = 0.250                   # task spec ~0.25
BASKET_DAMPING = 2.0                    # EST: N s/m, roller drag (small)
BASKET_FRICTION = 0.30                  # EST: N, roller friction (low)
BASKET_MASS = 0.90                      # EST: coated-steel wire basket
WIRE_R, FRAME_R = 0.002, 0.0025         # EST: wire radii (PoC tine radius 0.002)
FLOOR_PITCH = 0.014                     # PoC floor-wire pitch (PLATE/POC notes)
TINE_R, TINE_H, TINE_PITCH = 0.002, 0.060, 0.028   # PoC-proven for 8" plates
PLATE_ROWS_Y = (-0.005, 0.135)          # two plate rows 0.14 apart (PoC); plate section centre y=0.065
TINE_X = [-0.096 + TINE_PITCH * k for k in range(9)]   # 9 tines/row -> 8 slots (capacity claim: 8 plates)
# multi_dish_fingers COPY (DEMO_NOTES.md): the plate section is turned 90 deg -- two tine rows ALONG y at x = PLATE_C_X -/+ 0.07,
# tines at y = PLATE_TINE_Y (same 28 mm pitch, same 60 mm tines). Plates stand with their normal along y, centred at x = PLATE_C_X,
# so the hand above a plate stays in front of the machine (x > 0) instead of over the control band.
PLATE_C_X = 0.030
PLATE_ROWS_X = (PLATE_C_X - 0.070, PLATE_C_X + 0.070)
PLATE_TINE_Y = [0.043 + TINE_PITCH * j for j in range(5)]       # slots centred at y = 0.057, 0.085, 0.113, 0.141
BOWL_ROWS_Y = (-0.120, -0.045)          # COPY: two back-rest rows; row 2 only in the front half (x >= 0.04)
BOWL_ROW_Y, BOWL_LEAN = -0.120, math.radians(30)  # multi_dish_fingers COPY (DEMO_NOTES.md): bowl BACK-REST row, base y=-0.110,
                                        # leaning 20 deg toward -y (was 35 deg, base -0.061), 80 mm tines; a bowl rests on edge, tilted
                                        # 70 deg, its back on this row, rim on the floor wires, opening toward the plate section (+y)
BOWL_TINE_H = 0.080
                                        # basket side wall at y=-0.185 is the second support, 0.124 away as in PoC
HANDLE_B_Z, HANDLE_B_HL = 0.140, 0.060  # EST: basket pull-bar height above floor wires, half-length
ROLLER_R, ROLLER_HW = 0.010, 0.004      # EST: roller radius, half-width
PLATE_R = 0.1015                        # 8" plate radius (PoC plate)
# ---------------------------------------------------------------- rails / upper tray / floor parts
RAIL_Z = (0.086, 0.092)                 # EST: basket rail ledge on tub side walls
TRAY_Z = 0.350                          # EST: upper tray floor-wire height (plates top out at ~0.31)
TRAY_H = 0.035                          # EST: shallow tray wall height (cups / spoons)
TRAY_MASS = 0.30                        # EST
ARM_C = (-0.010, 0.0)                   # EST: spray-arm hub at tub centre
ARM_Z, ARM_HL = 0.102, 0.150            # EST: arm centre height (22 mm above floor: room for a pod), half-length
FILTER_C, FILTER_R, FILTER_H = (0.080, -0.100), 0.030, 0.020   # EST: cylindrical filter, front-left
POD_ZONE = (0.080, 0.110)               # seller: pod on tub floor under rack -> free floor spot (EST position)
SHELL_MASS = 4.5                        # EST: moulded shell + tub + panel; rest = machinery (pump, heater, motor)
# ---------------------------------------------------------------- control band (listing: touch buttons)
BUTTONS = ['eco', 'intensive', 'quick', 'dry', 'fruit', 'power']   # listing order 节能洗 强力洗 快速洗 烘干/除菌 水果洗 电源
BTN_Y0, BTN_PITCH = -0.160, 0.064       # EST: evenly spaced, listing order left->right as seen from the front
BTN_Z = (BAND_Z0 + H) / 2               # band centre height 0.4325
BTN_HALF = (0.0015, 0.012, 0.008)       # EST: 24 x 16 mm pads, 3 mm proud

WHITE = [.94, .94, .92, 1]
TUB_C = [.86, .87, .88, 1]
PANEL = [.16, .17, .19, 1]
BTN_C = [.62, .78, .92, 1]
WIRE = [.58, .61, .65, 1]
WINDOW = [.42, .52, .62, .35]
DARK = [.25, .26, .28, 1]

CONTACT = dict(friction=[.6, .005, .0001], solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])  # EST: plastic / coated wire
VISUAL = dict(contype=0, conaffinity=0)


def _q_yaw(a):
    return [math.cos(a / 2), 0., 0., math.sin(a / 2)]


class _Builder:
    def __init__(self, prefix):
        self.p = prefix

    def box(self, body, name, lo, hi, rgba, mass=None, **kw):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        assert np.all(hi > lo), name
        g = body.add_geom(name=self.p + name, type=BOX, pos=(lo + hi) / 2, size=(hi - lo) / 2, rgba=rgba, **kw)
        if mass is not None:
            g.mass = mass
        return g

    def cap(self, body, name, a, b, r, rgba, mass=None, **kw):
        g = body.add_geom(name=self.p + name, type=CAP, fromto=[*a, *b], size=[r, 0, 0], rgba=rgba, **kw)
        if mass is not None:
            g.mass = mass
        return g

    def cyl(self, body, name, a, b, r, rgba, mass=None, **kw):
        g = body.add_geom(name=self.p + name, type=CYL, fromto=[*a, *b], size=[r, 0, 0], rgba=rgba, **kw)
        if mass is not None:
            g.mass = mass
        return g


def _set_masses(geoms, total):
    """Distribute `total` kg over geoms in proportion to their volume."""
    def vol(g):
        s = g.size
        if g.type == BOX:
            return 8 * s[0] * s[1] * s[2]
        L = np.linalg.norm(np.subtract(g.fromto[3:], g.fromto[:3]))
        v = math.pi * s[0] ** 2 * L
        return v + (4 / 3 * math.pi * s[0] ** 3 if g.type == CAP else 0)
    v = np.array([vol(g) for g in geoms])
    for g, vi in zip(geoms, v):
        g.mass = total * vi / v.sum()


def door_spring(m, xc, zc, g=9.81):
    """Linear torsion spring (stiffness k, reference angle th_ref, radians) for the door.

    Gravity torque about the hinge (positive = opening): m g (xc cos th + zc sin th).
    Spring torque: -k (th - th_ref). Chosen so the NET torque is DOOR_NET_CLOSED * m g zc at
    0 deg (holds the door shut against its closed stop - the 'latch') and DOOR_NET_OPEN * m g zc
    at 90 deg (door rests on its open stop). In between the net torque stays small, so with
    damping + friction the door drifts instead of falling (see MODEL_NOTES)."""
    G = m * g * zc
    k = ((1 - DOOR_NET_OPEN + DOOR_NET_CLOSED) * G - m * g * xc) / (math.pi / 2)
    th_ref = (DOOR_NET_CLOSED * G - m * g * xc) / k
    return k, th_ref


def add_dishwasher(spec: mujoco.MjSpec, pos, yaw: float = 0.0, prefix: str = 'dw/') -> dict:
    """Add the dishwasher to `spec.worldbody`. Returns a dict of useful names.

    `pos` = centre of the footprint (bottom of the feet) in world coordinates; `yaw` in radians
    about world z (0 = front faces world +x)."""
    deg = spec.compiler.degree
    ang = (lambda r: math.degrees(r)) if deg else (lambda r: r)
    b = _Builder(prefix)
    P = prefix
    base = spec.worldbody.add_body(name=P + 'base', pos=list(pos), quat=_q_yaw(yaw))
    shell = []

    def sbox(name, lo, hi, rgba=WHITE, **kw):
        g = b.box(base, name, lo, hi, rgba, **CONTACT, **kw)
        shell.append(g)
        return g

    # ---- feet, bottom, plinth (pump compartment), tub floor
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        g = b.cyl(base, f'foot{i}', [sx * .165, sy * .185, 0], [sx * .165, sy * .185, FOOT_H], FOOT_R, DARK, **CONTACT)
        shell.append(g)
    sbox('bottom', [-X_FRONT, -W / 2, FOOT_H], [X_TUB, W / 2, FOOT_H + T])
    sbox('kick', [X_TUB, -W / 2, FOOT_H], [X_FRONT, W / 2, HINGE_Z - .0005])           # strip under the door
    sbox('plinth_front', [X_TUB - T, -TUB_Y, FOOT_H + T], [X_TUB, TUB_Y, TUB_FLOOR - T])
    sbox('tub_floor', [TUB_BACK, -TUB_Y, TUB_FLOOR - T], [X_TUB, TUB_Y, TUB_FLOOR], TUB_C)
    # ---- walls, top, header above opening
    for s, nm in ((1, 'wall_r'), (-1, 'wall_l')):
        lo, hi = sorted([s * TUB_Y, s * W / 2])
        sbox(nm, [-X_FRONT, lo, FOOT_H], [X_TUB, hi, TUB_CEIL])
    sbox('wall_back', [-X_FRONT, -TUB_Y, FOOT_H + T], [TUB_BACK, TUB_Y, TUB_CEIL])
    sbox('top', [-X_FRONT, -W / 2, TUB_CEIL], [X_TUB, W / 2, H])
    sbox('header', [X_TUB - T, -TUB_Y, DOOR_TOP], [X_TUB, TUB_Y, TUB_CEIL])
    # ---- control band (front top), dark glass strip (visual)
    sbox('band', [X_TUB, -W / 2, BAND_Z0], [X_FRONT, W / 2, H])
    b.box(base, 'band_glass', [X_FRONT - .0005, -W / 2 + .01, BAND_Z0 + .006], [X_FRONT + .0005, W / 2 - .01, H - .006],
          PANEL, mass=0, **VISUAL)
    # ---- basket rails and tray ledges on the tub side walls
    for s, nm in ((1, 'r'), (-1, 'l')):
        lo, hi = sorted([s * (TUB_Y - .012), s * TUB_Y])
        sbox(f'rail_{nm}', [TUB_BACK + .002, lo, RAIL_Z[0]], [X_TUB - .004, hi, RAIL_Z[1]], TUB_C)
        lo, hi = sorted([s * (TUB_Y - .010), s * TUB_Y])
        sbox(f'ledge_{nm}', [TUB_BACK + .002, lo, TRAY_Z - .015], [X_TUB - .004, hi, TRAY_Z - .010], TUB_C)
    # ---- machinery mass (pump, heater, motor) - invisible, non-colliding
    mach = b.box(base, 'machinery', [-.15, -.15, FOOT_H + T], [.10, .15, TUB_FLOOR - T - .001], [0, 0, 0, 0],
                 group=3, **VISUAL)
    # ---- filter (cylinder on tub floor) + pod floor zone
    b.cyl(base, 'filter', [*FILTER_C, TUB_FLOOR - .004], [*FILTER_C, TUB_FLOOR + FILTER_H], FILTER_R,
          [.55, .6, .66, 1], mass=0, **CONTACT)
    b.cyl(base, 'filter_cap', [*FILTER_C, TUB_FLOOR + FILTER_H], [*FILTER_C, TUB_FLOOR + FILTER_H + .003], .012,
          DARK, mass=0, **CONTACT)
    base.add_site(name=P + 'filter_zone', pos=[*FILTER_C, TUB_FLOOR + FILTER_H / 2], size=[FILTER_R + .01, 0, 0],
                  rgba=[1, .3, .2, .25])
    base.add_site(name=P + 'floor_pod_zone', pos=[*POD_ZONE, TUB_FLOOR + .001], size=[.025, 0, 0], rgba=[.2, .8, .3, .25])

    # ---- spray arm (fixed here; rotates in reality)
    arm = base.add_body(name=P + 'spray_arm', pos=[*ARM_C, 0])
    b.cyl(arm, 'arm_hub', [0, 0, TUB_FLOOR], [0, 0, ARM_Z - .002], .018, [.8, .8, .82, 1], mass=.03, **CONTACT)
    b.box(arm, 'arm_bar', [-.011, -ARM_HL, ARM_Z - .005], [.011, ARM_HL, ARM_Z + .005], [.8, .8, .82, 1], mass=.06, **CONTACT)
    for i in range(-4, 5):
        if i:
            b.cyl(arm, f'arm_nozzle{i + 4}', [0, i * .032, ARM_Z + .004], [0, i * .032, ARM_Z + .006], .003, DARK,
                  mass=0, **VISUAL)

    # ---- control buttons: one static body per pad, touch sensor on a site enclosing the pad surface
    for i, nm in enumerate(BUTTONS):
        y = BTN_Y0 + BTN_PITCH * i
        bb = base.add_body(name=P + f'btn_{nm}_body', pos=[X_FRONT, y, BTN_Z])
        b.box(bb, f'btn_{nm}_pad', [0, -BTN_HALF[1], -BTN_HALF[2]], [2 * BTN_HALF[0], BTN_HALF[1], BTN_HALF[2]],
              BTN_C if nm in ('quick', 'power') else [.75, .78, .80, 1], mass=.002, **CONTACT)
        bb.add_site(name=P + f'btn_{nm}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[BTN_HALF[0], 0, 0],
                    size=[.004, BTN_HALF[1] + .001, BTN_HALF[2] + .001], rgba=[1, 1, 1, 0])
        spec.add_sensor(name=P + f'btn_{nm}_touch', type=mujoco.mjtSensor.mjSENS_TOUCH,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname=P + f'btn_{nm}')

    # ================================================================ door
    door = base.add_body(name=P + 'door', pos=[X_FRONT, 0, HINGE_Z])
    dg = []

    def dbox(name, lo, hi, rgba=WHITE, part='frame', **kw):
        g = b.box(door, name, lo, hi, rgba, **CONTACT, **kw)
        dg.append((g, part))
        return g
    xo, xi = 0.0, -DOOR_T                 # outer / inner face (door-local x)
    xl = xi + LINER                       # liner / outer-slab boundary = groove floor surface
    gy, gz = GROOVE_Y, GROOVE_Z
    hy, hz = GROOVE[0] / 2 + GROOVE_WALL, GROOVE[1] / 2 + GROOVE_WALL     # hole half-size in liner
    # bottom rail: outer slab full width + liner with a hole for the groove
    dbox('rail_b_outer', [xl, -DOOR_HW, 0], [xo, DOOR_HW, RAIL_B])
    dbox('rail_b_liner_l', [xi, -DOOR_HW, 0], [xl, gy - hy, RAIL_B])
    dbox('rail_b_liner_r', [xi, gy + hy, 0], [xl, DOOR_HW, RAIL_B])
    dbox('rail_b_liner_lo', [xi, gy - hy, 0], [xl, gy + hy, gz - hz])
    dbox('rail_b_liner_hi', [xi, gy - hy, gz + hz], [xl, gy + hy, RAIL_B])
    # groove: 4 thin walls + floor (recess on the inner face)
    gc = [.80, .83, .87, 1]
    dbox('groove_floor', [xl, gy - hy, gz - hz], [xl + GROOVE_WALL, gy + hy, gz + hz], gc, part='groove')
    dbox('groove_wall_lo', [xi, gy - hy, gz - hz], [xl, gy + hy, gz - hz + GROOVE_WALL], gc, part='groove')
    dbox('groove_wall_hi', [xi, gy - hy, gz + hz - GROOVE_WALL], [xl, gy + hy, gz + hz], gc, part='groove')
    dbox('groove_wall_l', [xi, gy - hy, gz - hz + GROOVE_WALL], [xl, gy - hy + GROOVE_WALL, gz + hz - GROOVE_WALL], gc, part='groove')
    dbox('groove_wall_r', [xi, gy + hy - GROOVE_WALL, gz - hz + GROOVE_WALL], [xl, gy + hy, gz + hz - GROOVE_WALL], gc, part='groove')
    door.add_site(name=P + 'door_groove', pos=[xi + GROOVE[2] / 2, gy, gz], size=[.006, 0, 0], rgba=[.1, .7, .3, .6])
    # stiles, top rail, window
    dbox('stile_l', [xi, -DOOR_HW, RAIL_B], [xo, -WIN_HW, DOOR_H - RAIL_T])
    dbox('stile_r', [xi, WIN_HW, RAIL_B], [xo, DOOR_HW, DOOR_H - RAIL_T])
    dbox('rail_t', [xi, -DOOR_HW, DOOR_H - RAIL_T], [xo, DOOR_HW, DOOR_H])
    dbox('window', [xi / 2 - WIN_T / 2, -WIN_HW, RAIL_B], [xi / 2 + WIN_T / 2, WIN_HW, DOOR_H - RAIL_T], WINDOW, part='window')
    # pull bar ("handle / lip" at the top edge) on two posts
    hb = b.cap(door, 'handle_bar', [HANDLE_OFF, -HANDLE_HL, HANDLE_Z], [HANDLE_OFF, HANDLE_HL, HANDLE_Z], HANDLE_R, WHITE, **CONTACT)
    dg.append((hb, 'handle'))
    for s in (-1, 1):
        g = b.cap(door, f'handle_post{(s + 1) // 2}', [.002, s * (HANDLE_HL - .012), HANDLE_Z],
                  [HANDLE_OFF, s * (HANDLE_HL - .012), HANDLE_Z], .006, WHITE, **CONTACT)
        dg.append((g, 'handle'))
    door.add_site(name=P + 'door_handle', pos=[HANDLE_OFF, 0, HANDLE_Z], size=[.006, 0, 0], rgba=[1, .5, .1, .6])
    for part, mtot in DOOR_MASS.items():
        _set_masses([g for g, p in dg if p == part], mtot)
    # door mass properties -> counterbalance spring
    m_d = sum(DOOR_MASS.values())
    com = sum(np.asarray(g.pos) * g.mass if g.type == BOX else
              (np.asarray(g.fromto[:3]) + g.fromto[3:]) / 2 * g.mass for g, _ in dg) / m_d
    k_spring, th_ref = door_spring(m_d, com[0], com[2])
    door.add_joint(name=P + 'door_hinge', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0],
                   range=[0, ang(math.pi / 2)], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                   stiffness=k_spring, springref=ang(th_ref), damping=DOOR_DAMPING, frictionloss=DOOR_FRICTION,
                   armature=DOOR_ARMATURE)
    spec.add_sensor(name=P + 'door_angle', type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                    objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=P + 'door_hinge')

    # ================================================================ lower basket
    bk = base.add_body(name=P + 'basket', pos=[BASKET_X0, 0, BASKET_Z])
    bg = []

    def wire(name, a, bb_, r=WIRE_R, rgba=WIRE):
        g = b.cap(bk, name, a, bb_, r, rgba, **CONTACT)
        bg.append(g)
        return g
    HX, HY, HZ = BASKET_HX, BASKET_HY, BASKET_WALL_H
    # floor: longitudinal wires (along x) at FLOOR_PITCH, cross wires (along y) every 0.04
    ny = int(round(2 * HY / FLOOR_PITCH))
    for i in range(ny + 1):
        y = -HY + i * 2 * HY / ny
        wire(f'bfloor_x{i}', [-HX, y, 0], [HX, y, 0])
    for i in range(9):
        x = -HX + i * 2 * HX / 8
        wire(f'bfloor_y{i}', [x, -HY, 0], [x, HY, 0])
    # perimeter frame (bottom + top) and vertical side wires
    for z, tag in ((0, 'b'), (HZ, 't')):
        wire(f'frame_{tag}_f', [HX, -HY, z], [HX, HY, z], FRAME_R)
        wire(f'frame_{tag}_k', [-HX, -HY, z], [-HX, HY, z], FRAME_R)
        wire(f'frame_{tag}_l', [-HX, -HY, z], [HX, -HY, z], FRAME_R)
        wire(f'frame_{tag}_r', [-HX, HY, z], [HX, HY, z], FRAME_R)
    for i in range(1, 9):
        y = -HY + i * 2 * HY / 9
        wire(f'front_v{i}', [HX, y, 0], [HX, y, HZ])
        wire(f'back_v{i}', [-HX, y, 0], [-HX, y, HZ])
    for i in range(0, 9):
        x = -HX + i * 2 * HX / 8
        wire(f'side_l_v{i}', [x, -HY, 0], [x, -HY, HZ])
        wire(f'side_r_v{i}', [x, HY, 0], [x, HY, HZ])
    # plate section: two vertical tine rows 0.14 apart, 8 slots at 0.028 pitch
    for r, x in enumerate(PLATE_ROWS_X):
        for j, y in enumerate(PLATE_TINE_Y):
            wire(f'plate_tine{r}_{j}', [x, y, 0], [x, y, TINE_H], TINE_R)
    for r, y in enumerate(()):
        for k, x in enumerate(TINE_X):
            wire(f'plate_tine{r}_{k}', [x, y, 0], [x, y, TINE_H], TINE_R)
    # bowl section: one row leaning 35 deg toward the basket side wall (-y)
    lean = np.array([0, -math.sin(BOWL_LEAN), math.cos(BOWL_LEAN)]) * BOWL_TINE_H
    for r, yb in enumerate(BOWL_ROWS_Y):
        for k, x in enumerate(TINE_X):
            if r == 1 and x < .04:
                continue
            wire(f'bowl_tine{r}_{k}', [x, yb, 0], np.array([x, yb, 0]) + lean, TINE_R)
    # pull bar on two posts above the front top rail
    wire('bhandle_bar', [HX, -HANDLE_B_HL, HANDLE_B_Z], [HX, HANDLE_B_HL, HANDLE_B_Z], .004, WHITE)
    for s in (-1, 1):
        wire(f'bhandle_post{(s + 1) // 2}', [HX, s * (HANDLE_B_HL - .008), HZ], [HX, s * (HANDLE_B_HL - .008), HANDLE_B_Z], FRAME_R)
    # rollers on the side rails (roller bottom 1.5 mm above the rail top)
    rz = RAIL_Z[1] + .0015 + ROLLER_R - BASKET_Z
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        yc = sy * (HY + .006)
        g = b.cyl(bk, f'roller{i}', [sx * .13, yc - ROLLER_HW, rz], [sx * .13, yc + ROLLER_HW, rz], ROLLER_R, DARK, **CONTACT)
        bg.append(g)
        wire(f'axle{i}', [sx * .13, sy * HY, rz], [sx * .13, yc, rz], .0015, DARK)
        wire(f'roller_strut{i}', [sx * .13, sy * HY, rz], [sx * .13, sy * HY, 0], FRAME_R)
    _set_masses(bg, BASKET_MASS)
    bk.add_joint(name=P + 'basket_slide', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0],
                 range=[0, BASKET_TRAVEL], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                 damping=BASKET_DAMPING, frictionloss=BASKET_FRICTION, armature=0.0)
    spec.add_sensor(name=P + 'basket_pos', type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                    objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=P + 'basket_slide')
    bk.add_site(name=P + 'basket_handle', pos=[HX, 0, HANDLE_B_Z], size=[.006, 0, 0], rgba=[1, .5, .1, .6])
    # slot targets (see MODEL_NOTES): plate centre of a seated 8" plate in the front-most slot,
    # foot-ring centre of an inverted bowl leaning on the 35 deg row (bowl PoC geometry)
    slot_x = (TINE_X[-2] + TINE_X[-3]) / 2      # 2nd slot from the front (front slot sits behind the pull bar)
    bk.add_site(name=P + 'plate_slot_target', pos=[slot_x, sum(PLATE_ROWS_Y) / 2, WIRE_R + PLATE_R + .001],
                size=[.005, 0, 0], rgba=[.2, .9, .3, .6])
    bk.add_site(name=P + 'bowl_slot_target', pos=[TINE_X[6], BOWL_ROW_Y - .0786, .0787],
                size=[.005, 0, 0], rgba=[.2, .9, .3, .6])

    # ================================================================ upper tray (fixed; removable in reality)
    tr = base.add_body(name=P + 'upper_tray', pos=[BASKET_X0, 0, TRAY_Z])
    tg = []
    for i in range(16):
        y = -HY + i * 2 * HY / 15
        tg.append(b.cap(tr, f'tray_x{i}', [-HX, y, 0], [HX, y, 0], WIRE_R, WIRE, **CONTACT))
    for i in range(5):
        x = -HX + i * 2 * HX / 4
        tg.append(b.cap(tr, f'tray_y{i}', [x, -HY, 0], [x, HY, 0], WIRE_R, WIRE, **CONTACT))
    for z, tag in ((0, 'b'), (TRAY_H, 't')):
        for nm, a, c in (('f', [HX, -HY], [HX, HY]), ('k', [-HX, -HY], [-HX, HY]),
                         ('l', [-HX, -HY], [HX, -HY]), ('r', [-HX, HY], [HX, HY])):
            tg.append(b.cap(tr, f'tray_{tag}_{nm}', [*a, z], [*c, z], FRAME_R, WIRE, **CONTACT))
    for sx in (-1, 1):
        for sy in (-1, 1):
            tg.append(b.cap(tr, f'tray_post{sx}{sy}', [sx * HX, sy * HY, 0], [sx * HX, sy * HY, TRAY_H], FRAME_R, WIRE, **CONTACT))
    for s in (-1, 1):   # runners resting on the ledges
        tg.append(b.cap(tr, f'tray_runner{(s + 1) // 2}', [-HX, s * (TUB_Y - .005), -.0065], [HX, s * (TUB_Y - .005), -.0065],
                        .0025, WIRE, **CONTACT))
        tg.append(b.cap(tr, f'tray_runner_strut{(s + 1) // 2}', [0, s * HY, 0], [0, s * (TUB_Y - .005), -.0065], .002, WIRE, **CONTACT))
    _set_masses(tg, TRAY_MASS)

    # ---- shell mass: shell parts by volume, remainder to machinery so the machine totals MASS_TOTAL
    _set_masses(shell, SHELL_MASS)
    moving = sum(DOOR_MASS.values()) + BASKET_MASS + TRAY_MASS + .09 + .002 * len(BUTTONS)
    mach.mass = MASS_TOTAL - SHELL_MASS - moving
    assert mach.mass > 0

    return dict(base=P + 'base', door=P + 'door', basket=P + 'basket', door_joint=P + 'door_hinge',
                basket_joint=P + 'basket_slide', door_spring_k=k_spring, door_spring_ref=th_ref, door_mass=m_d,
                door_com=com.tolist(), sites=[P + s for s in (
                    'door_handle', 'door_groove', 'basket_handle', 'plate_slot_target', 'bowl_slot_target',
                    'filter_zone', 'floor_pod_zone', 'btn_quick', 'btn_power')])


def door_closed(model, data, prefix='dw/', tol_deg=1.0) -> bool:
    """'Latch' notion: door counts as closed when |hinge angle| < 1 deg (jointpos sensor)."""
    return abs(float(data.sensor(prefix + 'door_angle').data[0])) < math.radians(tol_deg)


def set_state(model, data, door_deg=0.0, basket=0.0, prefix='dw/'):
    """Reset-time helper: write door angle / basket travel into qpos (only for resets)."""
    data.qpos[model.joint(prefix + 'door_hinge').qposadr[0]] = math.radians(door_deg)
    data.qpos[model.joint(prefix + 'basket_slide').qposadr[0]] = basket
    mujoco.mj_forward(model, data)


SCENE_OPTIONS = '''
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" iterations="100" tolerance="1e-10"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/>
    <headlight ambient=".35 .35 .35" diffuse=".45 .45 .45"/></visual>
'''
