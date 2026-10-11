"""Kitchen scene for the MOBILE pipeline (system_design/spec.json v1.0).

The M1 kitchen (counter, countertop dishwasher, flat plate, pod dish, wall camera) with the counter run extended to
y = 2.65 m, a charging station at the start spot ~2 m along the counter, two V-blocks on the plinth at the work dock, a
lidar y-marker on the cabinet front, and the arm on a WHEELED base that is a free body (freejoint):

  * 2 driven wheels (sphere contact r 0.10 m, track 0.50 m) on hinge joints, torque-limited motor actuators
    (27.4 N m continuous / 55 N m peak at the wheel), each on a sprung arm (vertical slide + spring preload) that
    carries ~55 % of the weight (spec);
  * 4 passive casters at the corners, modelled as ball casters (sphere r 0.05 on a ball joint, small rolling
    friction) - the spec's swivel casters with 35 mm offset are approximated (no swivel dynamics / flutter);
  * failsafe power-off brakes = a wheel-locking position-hold actuator per wheel whose force is limited to the 27 N m
    holding torque (it slips beyond that), engaged / released by the BaseMCU with 30 / 20 ms ramps; floor / tyre
    friction mu 0.5 (spec);
  * dock side = REAR (-x): a spring-mounted dock plate carrying 2 cone pins at z 0.075 (spec ~0.05) that seat in the
    two plinth V-blocks; its compression (k 20 kN/m) is the dock force sensor ("bumper"); spring-loaded charge
    collector on the rear at z 0.062-0.098 (inside the plinth recess);
  * 2 x 2-D lidars (front-left, rear-right corners, z 0.25, in cut-outs; the rear one recessed 0.08 m so the cabinet
    stays beyond its 5 cm minimum range when docked).
  * arm: Franka Panda MJCF with the FR3 joint limits from spec.json, plus the Bota SensONE F/T (0.21 kg, 28 mm,
    height UNVERIFIED) between flange and hand. The FR3 is 0.9 kg heavier than the MJCF Panda (not added).

Base frame (spec): origin on the floor under the footprint centre, +x towards the aisle when docked (dock face = -x),
+y along the counter, z up. Docked: base centre (0.305, 0.42), yaw 0 -> arm base world (0.48, 0.42, 0.80), yaw 195.
World frame as ../full_pipeline: counter front edge = line x = 0, +x into the room.

Gravity compensation: NOT MuJoCo body gravcomp (on a free base it would also cancel the arm's weight on the base:
wrong wheel loads / tipping). The ArmController applies the gravity torque on the arm joints (qfrc_applied = internal
joint torque), as the real arm controller does; the arm's weight reaches the base through the joints.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[3]))   # repo root (override with env GYJ_WORK)

import mujoco
import numpy as np

import dishwasher as dw
import scene as S

ROOT = Path(__file__).resolve().parent
SPEC_PATH = _ROOT / 'system_design/spec.json'

PROVISIONAL = dict(wheel_radius_m=.0825, track_width_m=.50, footprint_m=[.65, .60], mass_kg_incl_battery_excl_arm=90.,
                   cog_m_base_frame_base_only=[.03, 0., .19], motor_torque_continuous_Nm_per_wheel=15.,
                   motor_torque_peak_Nm_per_wheel=30., max_speed_m_s=.5, max_accel_m_s2=.5, max_yaw_rate_rad_s=.8,
                   brake_holding_torque_Nm_per_wheel=20., floor_mu_assumed=1.0, drive_wheel_load_fraction=.57,
                   caster_wheel_radius_m=.025, dock_approach_speed_m_s=.05, dock_preload_N=40., wheel_width_m=.04)
FR3_Q_MIN = [-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508]
FR3_Q_MAX = [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]


def load_spec():
    """(base dict, battery dict, power dict, arm dict, meta). spec.json if present, else provisional numbers."""
    if not SPEC_PATH.exists():
        return dict(PROVISIONAL), dict(nominal_V=48., capacity_Ah=30.), {}, {}, dict(source='provisional')
    js = json.loads(SPEC_PATH.read_text())
    base = dict(PROVISIONAL)
    base.update({k: v for k, v in js.get('base', {}).items() if isinstance(v, (int, float, list))})
    base.update({k: v for k, v in js.get('docking', {}).items() if isinstance(v, (int, float))})
    lim = js.get('arm_choice', {}).get('sim_recheck_required', {}).get('replace_panda_joint_limits_with_fr3_rad', {})
    arm = dict(q_min=lim.get('q_min', FR3_Q_MIN), q_max=lim.get('q_max', FR3_Q_MAX), model=js.get('arm_choice', {}).get('model'))
    return base, js.get('battery', {}), js.get('power', {}), arm, dict(source=str(SPEC_PATH), version=js.get('version'),
                                                                      date=js.get('date'))


SPEC_BASE, SPEC_BATT, SPEC_POWER, SPEC_ARM, SPEC_META = load_spec()

# ------------------------------------------------------------------ layout (world)
COUNTER_Y = (-1.25, 2.65)
DOCK_ARM = np.array([.48, .42, .80])
DOCK_ARM_YAW = 195.
CHARGER_POSE = (0.60, 2.00, -90.)      # base centre when on the charging contacts: facing -y, rear (dock side) to +y
START_NOMINAL = (0.60, 1.92, -90.)     # mission start: 8 cm in front of the charging contacts (randomised +-5 cm, +-5 deg)
MARKER_Y, MARKER_Z, MARKER_W, MARKER_D = .42, .25, .05, .001  # 50 mm retro-reflective strip, flush on the cabinet front
# at lidar height (spec: 'a 50 mm reflective strip on the plinth'; moved up to the lidars' z 0.25 here)
CAB_X = S.CAB_FRONT_X
TOE_X = S.CAB_FRONT_X - S.TOE_D         # plinth (toe-kick) face, x = -0.095

# ------------------------------------------------------------------ base geometry (base frame)
HX, HY = .325, .30
REAR_X = -.318                          # chassis rear face (7 mm clear of the cabinet doors when docked)
CHASSIS_Z = (.06, .32)                  # spec envelope
ARM_MOUNT_LOCAL = np.array([.175, 0., .80])
ARM_YAW_LOCAL = 195.
LIDARS = dict(front=(np.array([.29, .26, .25]), 0.), rear=(np.array([REAR_X + .08, -.20, .25]), math.pi))
LIDAR_FOV = math.radians(70.)           # usable half-angle out of each corner cut-out (EST)
PIN_Y, PIN_Z, PIN_R, PIN_LEN = .20, .075, .012, .045
DOCK_K, DOCK_C, DOCK_TRAVEL = 20000., 400., .010          # sprung dock plate: 100 N at 5 mm (EST)
CHARGE_K, CHARGE_TRAVEL = 4000., .008
CASTER_XY = [(.265, .24), (.265, -.24), (-.265, .24), (-.265, -.24)]
WHEEL_MASS, SUSP_MASS, CASTER_MASS = 4.0, 1.5, .4      # EST: wheel + tyre, sprung arm (motor/gearbox in base body)
SUSP_K, SUSP_C = 60000., 2000.
BRAKE_KP, BRAKE_KV = 3000., 60.         # brake lock stiffness (N m/rad) and damping (N m s/rad), EST
BASE_RGBA = [.30, .33, .38, 1]
BASE_CONTACT = dict(friction=[.6, .005, .0001], solref=[.004, 1], solimp=[.95, .99, .001, .5, 2])
FT_H, FT_MASS, FT_R = .028, .21, .035   # Bota SensONE between flange and hand (height UNVERIFIED)
STYLUS_R = .004                         # conductive silicone fingertip, 8 mm dia (spec: 8-10 mm)
CONDUCTIVE_GEOMS = ('panda/stylus_tip',)
Q_STOW = np.array([0., -.45, 0., -2.70, 0., 2.25, .785])  # compact, hand over the base


@dataclass
class MobileConfig:
    start: tuple = START_NOMINAL
    base: dict = None                   # None -> SPEC_BASE
    vblocks: bool = True                # plinth V-blocks present
    arm_limits: str = 'fr3'             # 'fr3' | 'panda'
    ft_sensor: bool = True


def _q_yaw(deg):
    a = math.radians(deg)
    return [math.cos(a / 2), 0., 0., math.sin(a / 2)]


def _header():
    return S._header().replace('model="full_pipeline"', 'model="mobile_pipeline"')


def _box(body, name, lo, hi, rgba, **kw):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list((lo + hi) / 2), size=list((hi - lo) / 2),
                         rgba=rgba, **kw)


def bs_preload(cfg):
    return float((SPEC_BASE if cfg.base is None else cfg.base).get('dock_preload_N', 100.))


def _add_kitchen(spec, cfg, mu):
    wb = spec.worldbody
    BOX = mujoco.mjtGeom.mjGEOM_BOX
    y0, y1 = COUNTER_Y
    yc, hy = (y0 + y1) / 2, (y1 - y0) / 2
    fl = {**S.ENV_CONTACT, 'friction': [mu, .005, .0001]}
    wb.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, .1], material='floormat', contype=1,
                conaffinity=3, **fl)
    wb.add_geom(name='counter_top', type=BOX, pos=[-S.COUNTER_D / 2, yc, S.COUNTER_Z - S.COUNTER_T / 2],
                size=[S.COUNTER_D / 2, hy, S.COUNTER_T / 2], rgba=S.COUNTER_RGBA, **S.ENV_CONTACT)
    cz0, cz1 = S.TOE_H, S.COUNTER_Z - S.COUNTER_T
    wb.add_geom(name='cabinet', type=BOX, pos=[(-S.COUNTER_D + .02 + CAB_X) / 2, yc, (cz0 + cz1) / 2],
                size=[(CAB_X + S.COUNTER_D - .02) / 2, hy - .01, (cz1 - cz0) / 2], rgba=S.CABINET_RGBA, **S.ENV_CONTACT)
    wb.add_geom(name='toe_kick', type=BOX, pos=[(-S.COUNTER_D + .02 + TOE_X) / 2, yc, S.TOE_H / 2],
                size=[(TOE_X + S.COUNTER_D - .02) / 2, hy - .01, S.TOE_H / 2], rgba=[.22, .22, .23, 1], **S.ENV_CONTACT)
    for i, yy in enumerate(np.arange(y0 + .01, y1, .60)):
        wb.add_geom(name=f'cab_seam{i}', type=BOX, pos=[CAB_X + .0005, yy, (cz0 + cz1) / 2],
                    size=[.001, .002, (cz1 - cz0) / 2 - .01], rgba=[.55, .54, .52, 1], contype=0, conaffinity=0)
    wb.add_geom(name='wall', type=BOX, pos=[-S.COUNTER_D - .015, yc, 1.35], size=[.015, hy + .3, 1.35],
                rgba=[.93, .92, .89, 1], contype=0, conaffinity=0)
    wb.add_geom(name='wall_cabinet', type=BOX, pos=[-S.COUNTER_D + .17, .55, 1.85], size=[.17, .45, .35],
                rgba=S.CABINET_RGBA, contype=0, conaffinity=0)
    wb.add_geom(name='dock_marker', type=BOX, pos=[CAB_X + MARKER_D / 2, MARKER_Y, MARKER_Z],
                size=[MARKER_D / 2, MARKER_W / 2, .025], rgba=[.95, .95, .98, 1], contype=0, conaffinity=0)
    if cfg.vblocks:
        # two V-blocks on the plinth face (x = -0.095): 90 deg V opening to +x, apex where the pin tips seat
        dx, dy, _ = dock_base_pose()
        # pin hemisphere centre when docked with the dock plate compressed by the preload (base then exactly at the
        # design pose)
        seat_x = dx + REAR_X - PIN_LEN + PIN_R + bs_preload(cfg) / DOCK_K
        apex = seat_x - PIN_R * math.sqrt(2)
        L, T = .060, .010
        for s_ in (-1, 1):
            yp = dy + s_ * PIN_Y
            wb.add_geom(name=f'vblock_base{s_ + 1}', type=BOX, pos=[(TOE_X + apex - .004) / 2, yp, PIN_Z],
                        size=[(apex - .004 - TOE_X) / 2, .045, .03], rgba=[.55, .57, .6, 1], **S.ENV_CONTACT)
            for f in (-1, 1):                            # flank inner faces on the lines y - yp = f (x - apex)
                u = np.array([1., f]) / math.sqrt(2)     # along the flank, away from the apex
                nrm = np.array([-u[1], u[0]]) if f > 0 else np.array([u[1], -u[0]])   # outward: away from V interior
                c = np.array([apex, yp]) + u * L / 2 + nrm * T / 2
                yaw = math.degrees(math.atan2(u[1], u[0]))
                wb.add_geom(name=f'vblock{s_ + 1}_{f + 1}', type=BOX, pos=[c[0], c[1], PIN_Z], quat=_q_yaw(yaw),
                            size=[L / 2, T / 2, .03], rgba=[.55, .57, .6, 1],
                            **{**S.ENV_CONTACT, 'friction': [.3, .005, .0001]})
    # charging station at the start spot; narrow housing so the dock pins pass beside it
    cx, cy, _ = CHARGER_POSE
    yb = cy - REAR_X + .008                              # pad face: touches the collector (8 mm proud) when centred
    wb.add_geom(name='charger_pad', type=BOX, pos=[cx, yb + .005, .08], size=[.08, .005, .02], rgba=[.80, .65, .20, 1],
                **S.ENV_CONTACT)
    wb.add_geom(name='charger_housing', type=BOX, pos=[cx, yb + .07, .20], size=[.11, .06, .20], rgba=[.20, .22, .25, 1],
                **S.ENV_CONTACT)
    wb.add_geom(name='charger_led', type=BOX, pos=[cx, yb + .009, .36], size=[.03, .001, .008], rgba=[.2, .9, .3, 1],
                contype=0, conaffinity=0)


def _add_base(spec, cfg, bs):
    BOX, SPH, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CYLINDER
    x0, y0, yaw = cfg.start
    b = spec.worldbody.add_body(name='base', pos=[x0, y0, 0.], quat=_q_yaw(yaw))
    b.add_freejoint(name='base_free')
    r = bs['wheel_radius_m']
    tw = bs['track_width_m'] / 2
    ww = bs.get('wheel_width_m', .05)
    z0, z1 = CHASSIS_Z
    zw = min(z1 - .02, 2 * r + .01)
    kw = dict(mass=0, **BASE_CONTACT)
    _box(b, 'chassis_rear', [REAR_X, -HY, z0], [-.13, HY, z1], BASE_RGBA, **kw)
    _box(b, 'chassis_front', [.13, -HY, z0], [HX, HY, z1], BASE_RGBA, **kw)
    _box(b, 'chassis_mid', [-.13, -(tw - ww / 2 - .01), z0], [.13, tw - ww / 2 - .01, z1], BASE_RGBA, **kw)
    for s_ in (-1, 1):
        lo, hi = sorted([s_ * (tw - ww / 2 - .01), s_ * HY])
        _box(b, f'chassis_well_top{s_ + 1}', [-.13, lo, zw], [.13, hi, z1], BASE_RGBA, **kw)
    ax, ay, az = ARM_MOUNT_LOCAL
    _box(b, 'column', [ax - .09, ay - .09, z1], [ax + .09, ay + .09, az - .012], [.85, .86, .88, 1], **kw)
    _box(b, 'column_plate', [ax - .11, ay - .11, az - .012], [ax + .11, ay + .11, az], [.5, .52, .55, 1], **kw)
    for nm, (p, a) in LIDARS.items():
        b.add_geom(name=f'lidar_{nm}_vis', type=CYL, pos=list(p), size=[.04, .02, 0], rgba=[.12, .12, .14, 1],
                   contype=0, conaffinity=0, mass=0)
        b.add_site(name=f'lidar_{nm}', pos=list(p), quat=_q_yaw(math.degrees(a)), size=[.005, 0, 0], rgba=[1, 0, 0, 0])
    b.add_geom(name='estop_vis', type=CYL, pos=[ax + .091, .05, .70], quat=[.7071, 0, .7071, 0], size=[.02, .006, 0],
               rgba=[.85, .1, .1, 1], contype=0, conaffinity=0, mass=0)
    for s_ in (-1, 1):
        b.add_geom(name=f'edge_strip{s_ + 1}', type=BOX, pos=[0, s_ * (HY - .004), z0 + .02], size=[HX - .01, .004, .015],
                   rgba=[.1, .1, .1, 1], contype=0, conaffinity=0, mass=0)
    b.add_site(name='base_imu', pos=[0, 0, .20], size=[.005, 0, 0], rgba=[0, 1, 0, 0])
    # rear dock plate (sprung) with the two cone pins; compression = dock force sensor
    dp = b.add_body(name='dock_plate', pos=[0, 0, 0])
    dp.add_joint(name='dock_plate', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, DOCK_TRAVEL],
                 limited=mujoco.mjtLimited.mjLIMITED_TRUE, stiffness=DOCK_K, springref=0., damping=DOCK_C)
    _box(dp, 'dock_plate', [REAR_X - .006, -.24, PIN_Z - .02], [REAR_X, .24, PIN_Z + .02], [.15, .15, .17, 1], mass=.6,
         **BASE_CONTACT)
    for s_ in (-1, 1):
        dp.add_geom(name=f'dock_pin{s_ + 1}', type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    fromto=[REAR_X - .006, s_ * PIN_Y, PIN_Z, REAR_X - PIN_LEN + PIN_R, s_ * PIN_Y, PIN_Z],
                    size=[PIN_R, 0, 0], rgba=[.75, .76, .78, 1], mass=.1, **{**BASE_CONTACT, 'friction': [.3, .005, .0001]})
    spec.add_sensor(name='dock_plate', type=mujoco.mjtSensor.mjSENS_JOINTPOS, objtype=mujoco.mjtObj.mjOBJ_JOINT,
                    objname='dock_plate')
    cp = b.add_body(name='charge_plate', pos=[0, 0, 0])
    cp.add_joint(name='charge_plate', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, CHARGE_TRAVEL],
                 limited=mujoco.mjtLimited.mjLIMITED_TRUE, stiffness=CHARGE_K, springref=0., damping=50.)
    _box(cp, 'charge_plate', [REAR_X - .008, -.06, .062], [REAR_X - .002, .06, .098], [.80, .65, .20, 1], mass=.1,
         **BASE_CONTACT)
    spec.add_exclude(bodyname1='dock_plate', bodyname2='charge_plate')
    spec.add_sensor(name='charge_contact', type=mujoco.mjtSensor.mjSENS_JOINTPOS, objtype=mujoco.mjtObj.mjOBJ_JOINT,
                    objname='charge_plate')
    # drive wheels on sprung arms; spring preload so each carries load_fraction/2 of the weight at rest
    m_tot = bs['mass_kg_incl_battery_excl_arm'] + 18.0
    wheel_load = bs['drive_wheel_load_fraction'] / 2 * m_tot * 9.81
    spring_ref = -(wheel_load - (WHEEL_MASS + SUSP_MASS) * 9.81) / SUSP_K
    mu = bs['floor_mu_assumed']
    for s_, nm in ((1, 'l'), (-1, 'r')):
        sus = b.add_body(name=f'susp_{nm}', pos=[0, s_ * tw, r])
        sus.add_joint(name=f'susp_{nm}', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1], range=[-.03, .03],
                      limited=mujoco.mjtLimited.mjLIMITED_TRUE, stiffness=SUSP_K, springref=spring_ref, damping=SUSP_C)
        sus.add_geom(name=f'susp_{nm}_vis', type=BOX, size=[.04, .015, .03], pos=[0, -s_ * .04, .03], rgba=[.2, .2, .22, 1],
                     contype=0, conaffinity=0, mass=SUSP_MASS)
        wh = sus.add_body(name=f'wheel_{nm}')
        wj = wh.add_joint(name=f'wheel_{nm}', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0], damping=.2, armature=.05)
        wh.add_geom(name=f'wheel_{nm}_tyre', type=SPH, size=[r, 0, 0], mass=WHEEL_MASS, rgba=[0, 0, 0, 0], group=3,
                    friction=[mu, .005, .0001], condim=3, solref=[.004, 1], solimp=[.95, .99, .001, .5, 2],
                    contype=2, conaffinity=0)
        wh.add_geom(name=f'wheel_{nm}_vis', type=CYL, size=[r, ww / 2, 0], quat=[.7071, .7071, 0, 0], rgba=[.06, .06, .06, 1],
                    contype=0, conaffinity=0, mass=0)
        wh.add_geom(name=f'wheel_{nm}_spoke', type=BOX, size=[r * .8, ww / 2 + .0015, .008], rgba=[.85, .55, .15, 1],
                    contype=0, conaffinity=0, mass=0)
        tq = bs['motor_torque_peak_Nm_per_wheel']
        # spring-applied brake = wheel-locking position-hold actuator, holding torque limited (slips beyond it);
        # gains are switched by the BaseMCU (0 = released)
        bt = bs['brake_holding_torque_Nm_per_wheel']
        spec.add_actuator(name=f'brake_{nm}', target=f'wheel_{nm}', trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[BRAKE_KP] + [0] * 9, biasprm=[0, -BRAKE_KP, -BRAKE_KV] + [0] * 7,
                          forcerange=[-bt, bt], forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
        spec.add_actuator(name=f'motor_{nm}', target=f'wheel_{nm}', trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[1.] + [0] * 9, ctrlrange=[-tq, tq],
                          ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE, forcerange=[-tq, tq],
                          forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
    cr = bs['caster_wheel_radius_m']
    for i, (cx, cy) in enumerate(CASTER_XY):
        cb = b.add_body(name=f'caster{i}', pos=[cx, cy, cr])
        cb.add_joint(name=f'caster{i}', type=mujoco.mjtJoint.mjJNT_BALL, damping=.002, frictionloss=.01)
        cb.add_geom(name=f'caster{i}_ball', type=SPH, size=[cr, 0, 0], mass=CASTER_MASS, rgba=[.15, .15, .15, 1],
                    friction=[.6, .005, .0001], condim=3, solref=[.004, 1], solimp=[.95, .99, .001, .5, 2],
                    contype=2, conaffinity=0)
    for nm, typ in (('base_pos', mujoco.mjtSensor.mjSENS_FRAMEPOS), ('base_quat', mujoco.mjtSensor.mjSENS_FRAMEQUAT)):
        spec.add_sensor(name=nm, type=typ, objtype=mujoco.mjtObj.mjOBJ_SITE, objname='base_imu')
    spec.add_sensor(name='base_gyro', type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE, objname='base_imu')
    return b, b.add_site(name='robot_mount', pos=list(ARM_MOUNT_LOCAL), quat=_q_yaw(ARM_YAW_LOCAL), size=[.01, 0, 0])


def _attach_panda(spec, site, cfg, grip_k=800., grip_limit=70.):
    spec.attach(mujoco.MjSpec.from_file(str(S.PANDA)), prefix='panda/', site=site)
    hand = spec.body('panda/hand')
    if cfg.ft_sensor:                                    # Bota SensONE between flange and hand
        hand.pos = [hand.pos[0], hand.pos[1], hand.pos[2] + FT_H]
        l7 = spec.body('panda/link7')
        l7.add_geom(name='panda/ft_sensor', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .107 + FT_H / 2],
                    size=[FT_R, FT_H / 2, 0], rgba=[.15, .15, .17, 1], mass=FT_MASS, contype=0, conaffinity=0, group=2)
    hand.add_site(name='tcp', pos=[0, 0, S.TCP_OFFSET], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    hand.add_site(name='hand_ft', pos=[0, 0, 0], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    spec.body('panda/link0').add_site(name='mount_ft', pos=[0, 0, 0], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    # conductive silicone fingertip (stylus) on the OUTER side of the left finger, 2 mm beyond the pad tips
    lf = spec.body('panda/left_finger')
    tip_z = .053 + .002 - STYLUS_R
    lf.add_geom(name='panda/stylus_tip', type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, .013, tip_z], size=[STYLUS_R, 0, 0],
                rgba=[.95, .75, .1, 1], mass=.003, friction=[.8, .005, .0001], solref=[.006, 1], solimp=[.95, .99, .001, .5, 2])
    lf.add_geom(name='panda/stylus_body', type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, .013, .030, 0, .013, tip_z],
                size=[.003, 0, 0], rgba=[.3, .3, .3, 1], mass=.003, contype=0, conaffinity=0)
    lf.add_site(name='stylus', pos=[0, .013, tip_z + STYLUS_R], size=[.002, 0, 0], rgba=[1, 1, 0, 0])
    g = spec.actuator('panda/actuator8')
    g.gainprm[0] = grip_k * .07 / 255
    g.biasprm[0], g.biasprm[1], g.biasprm[2] = -grip_k * .03, -grip_k, -10
    g.forcerange = [-grip_limit, grip_limit]
    if cfg.arm_limits == 'fr3':
        for i in range(7):
            j = spec.joint(f'panda/joint{i + 1}')
            j.range = [SPEC_ARM.get('q_min', FR3_Q_MIN)[i], SPEC_ARM.get('q_max', FR3_Q_MAX)[i]]
            spec.actuator(f'panda/actuator{i + 1}').ctrlrange = list(j.range)
    for body in spec.bodies:
        if body.name.startswith('panda/'):
            body.gravcomp = 0
    for nm, typ, site_nm in (('mount_force', mujoco.mjtSensor.mjSENS_FORCE, 'mount_ft'),
                             ('mount_torque', mujoco.mjtSensor.mjSENS_TORQUE, 'mount_ft'),
                             ('hand_force', mujoco.mjtSensor.mjSENS_FORCE, 'hand_ft'),
                             ('hand_torque', mujoco.mjtSensor.mjSENS_TORQUE, 'hand_ft')):
        spec.add_sensor(name=nm, type=typ, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site_nm)


def build(cfg: MobileConfig = None, plate_xy=None, plate_yaw=0.0, layout=S.DEFAULT_LAYOUT, with_objects=True):
    """Full mobile scene. Returns (model, info)."""
    cfg = cfg or MobileConfig()
    bs = dict(SPEC_BASE if cfg.base is None else cfg.base)
    spec = mujoco.MjSpec.from_string(_header())
    _add_kitchen(spec, cfg, bs['floor_mu_assumed'])
    S._add_camera(spec, layout)
    dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    base, site = _add_base(spec, cfg, bs)
    _attach_panda(spec, site, cfg)
    if with_objects:
        S._add_plate(spec, layout.plate0 if plate_xy is None else plate_xy, plate_yaw)
        S._add_pod_and_dish(spec, np.array(layout.dish, float))
    # base body inertial: base-only mass and CoG from the spec (minus the separately modelled wheels / arms / casters)
    parts = 2 * (WHEEL_MASS + SUSP_MASS) + 4 * CASTER_MASS + .8 + .1
    mb = bs['mass_kg_incl_battery_excl_arm'] - parts
    com = np.array(bs.get('cog_m_base_frame_base_only', [0, 0, .2]), float)
    base.explicitinertial = True
    base.mass = mb
    base.ipos = list(com)
    base.inertia = [mb / 12 * (.6 ** 2 + .26 ** 2), mb / 12 * (.65 ** 2 + .26 ** 2), mb / 12 * (.65 ** 2 + .6 ** 2)]
    model = spec.compile()
    return model, dict(base_spec=bs, base_body_mass=round(mb, 2))


def build_ik(cfg: MobileConfig = None):
    """Kinematic model for the MotionPlanner: a free 'base' (set to the MEASURED base pose every tick) + column + arm."""
    cfg = cfg or MobileConfig()
    spec = mujoco.MjSpec.from_string(_header())
    b = spec.worldbody.add_body(name='base', pos=[0, 0, 0])
    b.add_freejoint(name='base_free')
    ax, ay, az = ARM_MOUNT_LOCAL
    b.add_geom(name='column', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[ax, ay, (.32 + az) / 2], size=[.09, .09, (az - .32) / 2],
               mass=10.)
    site = b.add_site(name='robot_mount', pos=list(ARM_MOUNT_LOCAL), quat=_q_yaw(ARM_YAW_LOCAL), size=[.01, 0, 0])
    _attach_panda(spec, site, cfg)
    return spec.compile()


def dock_base_pose(dx=0., dy=0., dyaw_deg=0.):
    """Base-centre pose (x, y, heading deg) that puts the arm base at the M1 design pose, plus an offset."""
    yaw = math.radians(DOCK_ARM_YAW - ARM_YAW_LOCAL + dyaw_deg)
    c, s_ = math.cos(yaw), math.sin(yaw)
    off = np.array([c * ARM_MOUNT_LOCAL[0] - s_ * ARM_MOUNT_LOCAL[1], s_ * ARM_MOUNT_LOCAL[0] + c * ARM_MOUNT_LOCAL[1]])
    xy = DOCK_ARM[:2] - off + np.array([dx, dy])
    return float(xy[0]), float(xy[1]), (math.degrees(yaw) + 180) % 360 - 180


def arm_base_pose(base_xy, base_yaw_deg):
    yaw = math.radians(base_yaw_deg)
    c, s_ = math.cos(yaw), math.sin(yaw)
    p = np.array(base_xy) + np.array([c * ARM_MOUNT_LOCAL[0] - s_ * ARM_MOUNT_LOCAL[1], s_ * ARM_MOUNT_LOCAL[0] + c * ARM_MOUNT_LOCAL[1]])
    return float(p[0]), float(p[1]), (base_yaw_deg + ARM_YAW_LOCAL) % 360


if __name__ == '__main__':
    m, info = build()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = m.body('base').id
    print(json.dumps({k: v for k, v in info.items() if k != 'base_spec'}), SPEC_META)
    print('nbody', m.nbody, 'ngeom', m.ngeom, 'nq', m.nq, 'nv', m.nv, 'nu', m.nu)
    print('total base subtree mass', round(m.body_subtreemass[bid], 2), 'dock pose', dock_base_pose())
    print('joint ranges', [list(np.round(m.jnt_range[m.joint(f"panda/joint{i}").id], 3)) for i in range(1, 8)])
