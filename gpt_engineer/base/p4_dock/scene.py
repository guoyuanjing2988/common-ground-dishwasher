"""Kitchen scene for the full pipeline: floor, base cabinets + counter (top z=0.90, depth 0.60),
countertop dishwasher flush with the counter edge, a free 8" plate lying flat beside it, a detergent
pod in a small dish, a wall camera, and the Franka Panda on a docked floor cart (M1) / a cart with a
linear axis (M2) / clamped on the counter (M3).

World frame: the counter front edge is the line x=0 (counter at -0.60 <= x <= 0), +x points out
of the counter into the room, +y to the right as seen from the room, z up. Dishwasher front faces +x.

Sources: dishwasher geometry = ../dishwasher_model (copied, unchanged); plate = ../poc_plate_flat;
contact / solver settings = PoC common ground. Everything else marked EST.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[3]))   # repo root (override with env GYJ_WORK)

import mujoco
import numpy as np

import dishwasher as dw

PANDA = _ROOT / 'mujoco_menagerie/franka_emika_panda/panda.xml'
COUNTER_Z, COUNTER_D, COUNTER_T = 0.90, 0.60, 0.04       # spec / EST worktop thickness
CAB_FRONT_X = -0.020                                      # EST: cabinet door faces, 20 mm counter overhang
TOE_H, TOE_D = 0.10, 0.075                                # EST: standard plinth 100 mm high, 75 mm recess
COUNTER_Y = (-1.25, 1.25)                                 # EST: run of worktop
DW_POS = (-dw.D / 2, 0.0, COUNTER_Z)                      # front flush with the counter edge (x = 0)
TCP_OFFSET = 0.1114                                       # hand -> pad tip, as in the PoCs

# ---- plate (identical to ../poc_plate_flat): local z = normal, foot-ring bottom at local z = 0
R_OUT, R_DISC, R_FOOT, PL_T, RIM_RISE = .1015, .065, .060, .005, .0098
SLOPE = math.atan2(RIM_RISE, R_OUT - R_DISC)
PLATE_MASS = dict(disc=.15, rim=.27, foot=.03)
MID_Z = .0105
PLATE0 = np.array([-0.15, 0.40])         # EST layout: plate start centre, 0.15 m behind the edge, right of the dishwasher
# ---- detergent pod + dish (EST: typical tablet 32x24x13 mm, 18 g; small ceramic dish)
POD_SIZE = np.array([.032, .024, .013])
POD_MASS = .018
DISH_C = np.array([-0.085, 0.655])       # EST layout: dish right of the plate start area
DISH_R, DISH_FLOOR_T, DISH_RIM_H = .045, .004, .010
# ---- wall camera (EST: under the front edge of a wall cabinet, looking down at the work area)
CAM_POS = np.array([-0.27, 0.42, 1.46])
CAM_LOOK = np.array([-0.07, 0.42, 0.90])
CAM_W, CAM_H, CAM_FOVY = 960, 540, 60.0

OBJ_CONTACT = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])
ENV_CONTACT = dict(friction=[.6, .005, .0001], solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])

COUNTER_RGBA = [.27, .27, .29, 1]        # dark stone worktop (plate contrast)
CABINET_RGBA = [.80, .78, .74, 1]
CART_RGBA = [.30, .33, .38, 1]


@dataclass
class Mount:
    """Robot mounting. kind: 'M1' docked floor cart / 'M2' cart + linear axis along y / 'M3' counter clamp.
    (x, y, z) = Panda base (link0 origin); yaw in degrees. For M2, y is the rail centre."""
    kind: str = 'M1'
    x: float = 0.45
    y: float = 0.30
    z: float = 0.90
    yaw: float = 180.0
    rail_travel: float = 0.0             # M2 only: total travel (m), centred on y
    cart_lx: float = 0.65                # cart footprint along x (from the dock bumper outwards)
    cart_ly: float = 0.60                # cart footprint along y
    cart_mass: float = 120.0             # EST: steel cart + ballast (for the statics check)
    note: str = ''

    def label(self):
        r = f' rail {self.rail_travel:.2f}' if self.kind == 'M2' else ''
        return f'{self.kind} base ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) yaw {self.yaw:.0f}{r}'


DEFAULT_MOUNT = Mount()


@dataclass
class Layout:
    """Where the loose items start (all EST layout choices). plate_mode 'flat' = plate lying flat on the
    counter (FLAT PoC); 'rack' = plate upright in a drying rack on the counter (plate PoC), used only
    where the flat pick is unreachable (M3)."""
    plate0: tuple = (-0.15, 0.40)
    plate_mode: str = 'flat'
    dish: tuple = (-0.085, 0.655)
    cam_pos: tuple = (-0.27, 0.42, 1.46)
    cam_look: tuple = (-0.07, 0.42, 0.90)
    rack: tuple = (-0.20, 0.62)          # rack centre (rack mode only)


DEFAULT_LAYOUT = Layout()


def _rotz(a):
    c, s_ = math.cos(a), math.sin(a)
    return [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]


def _q_yaw(deg):
    a = math.radians(deg)
    return [math.cos(a / 2), 0., 0., math.sin(a / 2)]


def quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def _header():
    return f'''<mujoco model="full_pipeline">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" iterations="100" tolerance="1e-10"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096" offsamples="4"/>
    <headlight ambient=".35 .35 .35" diffuse=".45 .45 .45"/></visual>
  <asset><texture name="sky" type="skybox" builtin="gradient" rgb1=".93 .93 .92" rgb2=".62 .63 .65" width="256" height="256"/>
    <texture name="floortex" type="2d" builtin="checker" rgb1=".62 .58 .52" rgb2=".56 .52 .47" width="512" height="512"/>
    <material name="floormat" texture="floortex" texrepeat="8 8"/></asset>
  <worldbody>
    <light name="key" pos="1.6 1.2 2.6" dir="-.55 -.4 -.75" diffuse=".7 .7 .68" castshadow="true"/>
    <light name="fill" pos="-.2 -1.0 2.4" dir=".2 .5 -1" diffuse=".25 .25 .27" castshadow="false"/>
  </worldbody></mujoco>'''


def _add_kitchen(spec):
    wb = spec.worldbody
    BOX = mujoco.mjtGeom.mjGEOM_BOX
    y0, y1 = COUNTER_Y
    yc, hy = (y0 + y1) / 2, (y1 - y0) / 2
    wb.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, .1], material='floormat', **ENV_CONTACT)
    wb.add_geom(name='counter_top', type=BOX, pos=[-COUNTER_D / 2, yc, COUNTER_Z - COUNTER_T / 2],
                size=[COUNTER_D / 2, hy, COUNTER_T / 2], rgba=COUNTER_RGBA, **ENV_CONTACT)
    cz0, cz1 = TOE_H, COUNTER_Z - COUNTER_T
    wb.add_geom(name='cabinet', type=BOX, pos=[(-COUNTER_D + .02 + CAB_FRONT_X) / 2, yc, (cz0 + cz1) / 2],
                size=[(CAB_FRONT_X + COUNTER_D - .02) / 2, hy - .01, (cz1 - cz0) / 2], rgba=CABINET_RGBA, **ENV_CONTACT)
    tx = CAB_FRONT_X - TOE_D
    wb.add_geom(name='toe_kick', type=BOX, pos=[(-COUNTER_D + .02 + tx) / 2, yc, TOE_H / 2],
                size=[(tx + COUNTER_D - .02) / 2, hy - .01, TOE_H / 2], rgba=[.22, .22, .23, 1], **ENV_CONTACT)
    # cabinet door seams + knobs (visual only)
    for i, yy in enumerate(np.arange(y0 + .01, y1, .60)):
        wb.add_geom(name=f'cab_seam{i}', type=BOX, pos=[CAB_FRONT_X + .0005, yy, (cz0 + cz1) / 2],
                    size=[.001, .002, (cz1 - cz0) / 2 - .01], rgba=[.55, .54, .52, 1], contype=0, conaffinity=0)
    # back wall, wall cabinets (visual; the camera sits under the cabinet front edge)
    wb.add_geom(name='wall', type=BOX, pos=[-COUNTER_D - .015, yc, 1.35], size=[.015, hy + .3, 1.35],
                rgba=[.93, .92, .89, 1], contype=0, conaffinity=0)
    wb.add_geom(name='wall_cabinet', type=BOX, pos=[-COUNTER_D + .17, .55, 1.85], size=[.17, .45, .35],
                rgba=CABINET_RGBA, contype=0, conaffinity=0)


def _add_camera(spec, lay):
    CAM_POS, CAM_LOOK = np.array(lay.cam_pos, float), np.array(lay.cam_look, float)
    fwd = CAM_LOOK - CAM_POS
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    spec.worldbody.add_camera(name='wallcam', pos=list(CAM_POS), quat=list(quat(np.column_stack([right, up, -fwd]))),
                              fovy=CAM_FOVY)
    spec.worldbody.add_geom(name='cam_housing', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(CAM_POS - .03 * fwd),
                            quat=list(quat(np.column_stack([right, up, -fwd]))), size=[.045, .015, .02],
                            rgba=[.1, .1, .1, 1], contype=0, conaffinity=0)


def _add_robot(spec, mount: Mount, robot_only=False):
    """Cart (static body) + optional linear axis + Panda. Returns name of the site the Panda sits on."""
    BOX = mujoco.mjtGeom.mjGEOM_BOX
    CYL = mujoco.mjtGeom.mjGEOM_CYLINDER
    wb = spec.worldbody
    if mount.kind == 'M3':
        site = wb.add_site(name='robot_mount', pos=[mount.x, mount.y, mount.z], quat=_q_yaw(mount.yaw), size=[.01, 0, 0])
        return site
    cart = wb.add_body(name='cart', pos=[0, 0, 0])
    x0 = CAB_FRONT_X                                    # dock bumper face against the cabinet doors
    x1 = x0 + mount.cart_lx
    ly = mount.cart_ly + (mount.rail_travel if mount.kind == 'M2' else 0.)
    yc = mount.y
    deck_z = 0.32                                       # EST: chassis + ballast height
    cart.add_geom(name='cart_chassis', type=BOX, pos=[(x0 + .015 + x1) / 2, yc, (0.06 + deck_z) / 2],
                  size=[(x1 - x0 - .015) / 2, ly / 2, (deck_z - .06) / 2], rgba=CART_RGBA, **ENV_CONTACT)
    cart.add_geom(name='cart_bumper', type=BOX, pos=[x0 + .0075, yc, .20], size=[.0075, ly / 2 - .02, .05],
                  rgba=[.1, .1, .1, 1], **ENV_CONTACT)
    for i, (sx, sy) in enumerate([(0, -1), (0, 1), (1, -1), (1, 1)]):
        xw = x0 + .06 if sx == 0 else x1 - .06
        cart.add_geom(name=f'cart_wheel{i}', type=CYL, pos=[xw, yc + sy * (ly / 2 - .06), .03], quat=[.7071, .7071, 0, 0],
                      size=[.03, .015, 0], rgba=[.08, .08, .08, 1], contype=0, conaffinity=0)
    if mount.kind == 'M2':                              # rail along y on the deck + carriage column
        cart.add_geom(name='rail', type=BOX, pos=[mount.x, yc, deck_z + .015], size=[.04, ly / 2 - .02, .015],
                      rgba=[.6, .62, .65, 1], **ENV_CONTACT)
        car = cart.add_body(name='carriage', pos=[mount.x, yc, deck_z + .032])   # 2 mm above the rail
        car.add_joint(name='rail_slide', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0],
                      range=[-mount.rail_travel / 2, mount.rail_travel / 2], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                      damping=200., armature=5.)        # EST: ball-screw axis, reflected inertia
        h = mount.z - (deck_z + .032)
        car.add_geom(name='column', type=BOX, pos=[0, 0, h / 2], size=[.09, .09, h / 2], rgba=[.85, .86, .88, 1],
                     mass=15., **ENV_CONTACT)
        car.gravcomp = 1
        site = car.add_site(name='robot_mount', pos=[0, 0, h], quat=_q_yaw(mount.yaw), size=[.01, 0, 0])
        spec.add_actuator(name='rail_act', target='rail_slide', trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[20000.] + [0] * 9, biasprm=[0, -20000., -2000.] + [0] * 7,
                          ctrlrange=[-mount.rail_travel / 2, mount.rail_travel / 2], ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                          forcerange=[-400, 400], forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
        return site
    h = mount.z - deck_z
    cart.add_geom(name='column', type=BOX, pos=[mount.x, mount.y, deck_z + h / 2], size=[.09, .09, h / 2],
                  rgba=[.85, .86, .88, 1], **ENV_CONTACT)
    cart.add_geom(name='column_plate', type=BOX, pos=[mount.x, mount.y, mount.z - .006], size=[.12, .12, .006],
                  rgba=[.5, .52, .55, 1], **ENV_CONTACT)
    return cart.add_site(name='robot_mount', pos=[mount.x, mount.y, mount.z], quat=_q_yaw(mount.yaw), size=[.01, 0, 0])


def _attach_panda(spec, site, grip_k=800., grip_limit=70.):
    spec.attach(mujoco.MjSpec.from_file(str(PANDA)), prefix='panda/', site=site)
    spec.body('panda/hand').add_site(name='tcp', pos=[0, 0, TCP_OFFSET], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    spec.body('panda/hand').add_site(name='hand_ft', pos=[0, 0, 0], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    spec.body('panda/link0').add_site(name='mount_ft', pos=[0, 0, 0], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    # Force-limited grasp (PoC common ground): ctrl 255 -> +0.04 (open), ctrl 0 -> -0.03 (squeeze)
    g = spec.actuator('panda/actuator8')
    g.gainprm[0] = grip_k * .07 / 255
    g.biasprm[0], g.biasprm[1], g.biasprm[2] = -grip_k * .03, -grip_k, -10
    g.forcerange = [-grip_limit, grip_limit]
    for body in spec.bodies:
        if body.name.startswith('panda/'):
            body.gravcomp = 1
    for nm, typ, site_nm in (('mount_force', mujoco.mjtSensor.mjSENS_FORCE, 'mount_ft'),
                             ('mount_torque', mujoco.mjtSensor.mjSENS_TORQUE, 'mount_ft'),
                             ('hand_force', mujoco.mjtSensor.mjSENS_FORCE, 'hand_ft'),
                             ('hand_torque', mujoco.mjtSensor.mjSENS_TORQUE, 'hand_ft')):
        spec.add_sensor(name=nm, type=typ, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site_nm)


def _add_plate(spec, xy, yaw, pos=None, R=None):
    if pos is None:
        pos, R = [xy[0], xy[1], COUNTER_Z + .0003], np.array(_rotz(yaw))
    plate = spec.worldbody.add_body(name='plate', pos=list(pos), quat=list(quat(R)))
    plate.add_freejoint(name='plate_free')
    white = [.96, .96, .93, 1]
    plate.add_geom(name='plate_disc', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .0075],
                   size=[R_DISC + .002, PL_T / 2, 0], mass=PLATE_MASS['disc'], rgba=white, **OBJ_CONTACT)
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
        plate.add_geom(name=f'plate_rim{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre),
                       quat=quat(np.column_stack([along, tangent, normal])),
                       size=[half_len, (R_DISC + width / 2) * math.sin(math.pi / n) * 1.15, PL_T / 2],
                       mass=PLATE_MASS['rim'] / n, rgba=white, **OBJ_CONTACT)
    m = 16
    for i in range(m):
        a = 2 * math.pi * i / m
        plate.add_geom(name=f'plate_foot{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                       pos=[R_FOOT * math.cos(a), R_FOOT * math.sin(a), .0025], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
                       size=[.002, R_FOOT * math.sin(math.pi / m) * 1.1, .0025], mass=PLATE_MASS['foot'] / m, rgba=white,
                       **OBJ_CONTACT)
    plate.add_site(name='plate_center', pos=[0, 0, MID_Z], size=[.003, 0, 0], rgba=[1, .3, .1, 1], group=4)


RACK_FLOOR_H, RACK_PITCH, RACK_ROWS_D, RACK_H = .005, .035, .14, .10      # ../poc_plate (EST)
RACK_SLOT_K = 0


def rack_plate_pose(rack_c, offset=(0, 0, 0)):
    """Upright plate in the rack: normal along -x (concave face toward the counter back), local y up.
    Returns (origin, R) of the plate body (origin = foot plane centre)."""
    # local x -> world -y, local y -> world z (up), local z (normal) -> world -x
    R = np.column_stack([[0, -1., 0], [0, 0, 1.], [-1., 0, 0]])
    R = np.array(_rotz(offset[2])) @ R
    floor = COUNTER_Z + RACK_FLOOR_H
    slot_x = rack_c[0] + RACK_SLOT_K * RACK_PITCH          # between the prongs at +-pitch/2 (as ../poc_plate)
    centre = np.array([slot_x + offset[0], rack_c[1] + offset[1], floor + .002 + R_OUT + .002])
    return centre - R @ np.array([0, 0, MID_Z]), R


def _add_rack_and_plate(spec, rack_c, offset, yaw):
    wb = spec.worldbody
    CAP = mujoco.mjtGeom.mjGEOM_CAPSULE
    floor = COUNTER_Z + RACK_FLOOR_H
    rgba = [.7, .7, .72, 1]
    for i in range(-8, 9):
        y = rack_c[1] + .014 * i
        wb.add_geom(name=f'rack_floor{i + 8}', type=CAP, fromto=[rack_c[0] - .12, y, floor, rack_c[0] + .12, y, floor],
                    size=[.002, 0, 0], rgba=rgba, **ENV_CONTACT)
    for s_ in (-1, 1):
        wb.add_geom(name=f'rack_foot{s_ + 1}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[rack_c[0], rack_c[1] + s_ * .11, COUNTER_Z + .0025],
                    size=[.12, .006, .0025], rgba=rgba, **ENV_CONTACT)
    for row in (-1, 1):
        y = rack_c[1] + row * RACK_ROWS_D / 2
        for j in range(-3, 4):
            x = rack_c[0] + (j + .5) * RACK_PITCH
            wb.add_geom(name=f'rack_prong{row + 1}_{j + 3}', type=CAP, fromto=[x, y, floor, x, y, floor + RACK_H],
                        size=[.002, 0, 0], rgba=rgba, **ENV_CONTACT)
    off = (0, 0, 0) if offset is None else (offset[0], offset[1], yaw)
    org, R = rack_plate_pose(rack_c, off)
    _add_plate(spec, None, 0, pos=org, R=R)


def _add_pod_and_dish(spec, DISH_C):
    dish = spec.worldbody.add_body(name='dish', pos=[DISH_C[0], DISH_C[1], COUNTER_Z + .0003])
    dish.add_freejoint(name='dish_free')
    c = [.62, .80, .78, 1]
    dish.add_geom(name='dish_floor', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, DISH_FLOOR_T / 2],
                  size=[DISH_R, DISH_FLOOR_T / 2, 0], mass=.12, rgba=c, **OBJ_CONTACT)
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        dish.add_geom(name=f'dish_rim{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                      pos=[DISH_R * math.cos(a), DISH_R * math.sin(a), DISH_RIM_H / 2], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
                      size=[.003, DISH_R * math.sin(math.pi / n) * 1.12, DISH_RIM_H / 2], mass=.004, rgba=c, **OBJ_CONTACT)
    pod = spec.worldbody.add_body(name='pod', pos=[DISH_C[0], DISH_C[1], COUNTER_Z + .0003 + DISH_FLOOR_T + POD_SIZE[2] / 2 + .0002])
    pod.add_freejoint(name='pod_free')
    pod.add_geom(name='pod_box', type=mujoco.mjtGeom.mjGEOM_BOX, size=list(POD_SIZE / 2), mass=POD_MASS,
                 rgba=[.25, .45, .85, 1], **OBJ_CONTACT)
    pod.add_geom(name='pod_stripe', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, POD_SIZE[2] / 2], size=[.008, POD_SIZE[1] / 2 + .0002, .0002],
                 rgba=[.95, .95, .95, 1], contype=0, conaffinity=0, mass=0, group=1)


def build(mount: Mount = DEFAULT_MOUNT, plate_xy=None, plate_yaw=0.0, with_objects=True, with_dishwasher=True,
          grip_k=800., grip_limit=70., layout: Layout = DEFAULT_LAYOUT):
    """Full scene. Returns (model, info)."""
    spec = mujoco.MjSpec.from_string(_header())
    _add_kitchen(spec)
    _add_camera(spec, layout)
    info = dw.add_dishwasher(spec, pos=DW_POS, yaw=0.0, prefix='dw/') if with_dishwasher else {}
    site = _add_robot(spec, mount)
    _attach_panda(spec, site, grip_k, grip_limit)
    if with_objects:
        if layout.plate_mode == 'flat':
            _add_plate(spec, layout.plate0 if plate_xy is None else plate_xy, plate_yaw)
        else:
            _add_rack_and_plate(spec, layout.rack, plate_xy, plate_yaw)
        _add_pod_and_dish(spec, np.array(layout.dish, float))
    model = spec.compile()
    return model, info


def build_ik(mount: Mount = DEFAULT_MOUNT, layout: Layout = None, env=False):
    """Kinematic model for the IK reference generator: cart [+ rail] + Panda. With env=True it also contains the
    static kitchen and the dishwasher (door / basket joints are frozen by the controller); only static shell geoms
    are used for collision avoidance (no objects, no moving parts)."""
    spec = mujoco.MjSpec.from_string(_header())
    if env:
        _add_kitchen(spec)
        dw.add_dishwasher(spec, pos=DW_POS, yaw=0.0, prefix='dw/')
        if layout is not None and layout.plate_mode == 'rack':
            _add_rack_and_plate(spec, layout.rack, None, 0.)
            spec.delete(spec.body('plate'))
    site = _add_robot(spec, mount)
    _attach_panda(spec, site)
    return spec.compile()


def ik_collision_pairs(ik):
    """(robot link + hand geoms, static environment geoms) for mink.CollisionAvoidanceLimit."""
    rb = {b for b in range(ik.nbody) if ik.body(b).name.startswith('panda/') and 'finger' not in ik.body(b).name}
    coll = lambda g: ik.geom_contype[g] or ik.geom_conaffinity[g]
    robot = [g for g in range(ik.ngeom) if ik.geom_bodyid[g] in rb and coll(g) and ik.body(ik.geom_bodyid[g]).name != 'panda/link0']
    base = ik.body('dw/base').id
    envg = [g for g in range(ik.ngeom) if coll(g) and (ik.geom_bodyid[g] == base or ik.geom(g).name in ('counter_top', 'cabinet')
                                                       or ik.geom(g).name.startswith('rack_'))]
    return robot, envg


if __name__ == '__main__':
    m, info = build()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print('nbody', m.nbody, 'ngeom', m.ngeom, 'nq', m.nq, 'nu', m.nu, 'nsensor', m.nsensor)
    for s in info['sites']:
        print(f'  {s:24s}', np.round(d.site(s).xpos, 4))
    print('plate', np.round(d.site('plate_center').xpos, 4), 'pod', np.round(d.body('pod').xpos, 4))
