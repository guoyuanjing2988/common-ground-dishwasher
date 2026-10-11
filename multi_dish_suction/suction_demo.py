#!/usr/bin/env python3
"""multi_dish_suction: Franka Panda + Franka Cobot Pump (Schmalz vacuum end effector) destacks a pile of 3 plates
and a nested pile of 2 bowls from the counter and loads them into the countertop dishwasher basket.

One continuous mj_step simulation. Suction = MuJoCo adhesion actuator on a compliant cup (ring of lip contacts),
force capped at cup area x vacuum x efficiency, gated by a seal check. No welds, no attachment, no teleports.

  python suction_demo.py --no-video          # fast check
  python suction_demo.py                     # video -> out/suction_demo.mp4 + frames/
  python suction_demo.py --still             # render the start frame only
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

import mink
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import dishwasher as dw
import scene as S

ROOT = Path(__file__).resolve().parent
OUT, FRAMES = ROOT / 'out', ROOT / 'frames'
PANDA_NOHAND = _ROOT / 'mujoco_menagerie/franka_emika_panda/panda_nohand.xml'

# ------------------------------------------------------------------ Franka Cobot Pump (Schmalz) - geometry EST
# Product: mounts on the ISO 9409-1-A50 flange, powered through it, <= 750 mbar vacuum, 12 L/min, 810 g, no air.
PUMP_R, PUMP_L = .040, .095          # EST: pump housing Ø80 x 95 mm (flange to housing front face)
ADAPT_R, ADAPT_L = .010, .035        # EST: cup adapter/stem Ø20 x 35 mm (lets the housing clear a bowl wall)
CUP_R, CUP_H = .020, .018            # Ø40 mm bellows cup, 18 mm high (EST, typical Schmalz FSGA/SPB2 40)
LIP_N, LIP_SR = 8, .003              # lip modelled as a ring of 8 compliant spheres r=3 mm on the Ø40 lip circle
TOOL_LEN = PUMP_L + ADAPT_L + CUP_H  # 0.148 m flange -> cup lip plane
PUMP_MASS, CUP_MASS = .76, .05       # 0.81 kg total (product spec)
# ------------------------------------------------------------------ suction force model
VACUUM_PA = 70e3                     # 700 mbar working vacuum (spec max 750 mbar)
CUP_AREA = math.pi * CUP_R ** 2      # 1.257e-3 m^2
EFFICIENCY = .50                     # curved / wet glazed surface, leakage, safety (documented in DEMO_NOTES)
F_THEORY = CUP_AREA * VACUUM_PA      # 88 N
F_CAP = F_THEORY * EFFICIENCY        # 44 N   <- adhesion actuator gain (ctrl in [0, 1])
import os
if os.environ.get('SD_FCAP'):          # sanity test only: e.g. SD_FCAP=3 -> weaker than a plate's weight -> it must drop
    F_CAP = float(os.environ['SD_FCAP'])
SEAL_KEEP = float(os.environ.get('SD_SEAL_KEEP', '.001'))  # maintenance gap limit (plain demo 1 mm; sink demo 1.5 mm)
CUP_GIMBAL = bool(os.environ.get('SD_CUP_GIMBAL'))
CUP_GIMBAL_K = 1.0                   # N m / rad, EST bellows bending stiffness
GIMBAL_ON_BUILD = [True]             # IK model is built without it (nominal cup pose)
LIP_MIN_TOUCH = int(os.environ.get('SD_LIP_MIN', '8'))   # plain demo: all 8 lips; sink demo: 6 + the rest within 1 mm
SEAL_MAX_DEG = 15.                   # cup axis must be within 15 deg of the surface normal
SEAL_DIST = .001                     # sealed cup stays sealed while every lip point is within 1 mm of the surface (bellows lip follows)
VAC_TAU = .15                        # s, vacuum build-up time constant once sealed (12 L/min pump, small cup volume)
CUP_MU = .8                          # rubber (NBR/silicone) on glazed ceramic, EST (wet: lower; see notes)

# ------------------------------------------------------------------ layout (EST)
MOUNT = S.Mount('M1', x=.55, y=.30, z=.90, yaw=180., cart_lx=.65, cart_ly=.60)   # M1 docked cart; base 0.10 further out than full_pipeline
                                     # so the horizontal-tool insert poses are not folded against joint 4/6 limits
PLATE_PILE = np.array([-.11, .45])   # 3 plates stacked
BOWL_PILE = np.array([-.10, .66])    # 2 bowls nested, opening up
N_PLATES, N_BOWLS = 3, 2
PLATE_SLOTS = [4, 5, 6]              # slot k between TINE_X[k], TINE_X[k+1]; inserted BACK -> FRONT (tool sticks out to the front)
PLATE_LEAN = math.radians(3)         # plate held leaning back 3 deg -> tool axis rises 3 deg; a dished 8" plate is ~20 mm thick in the
                                     # 24 mm tine gap, so it cannot lean more than ~4 deg while inside the 60 mm tines
PLATE_CUP_OFF = .040                 # cup centre 40 mm from plate centre (toward the top rim when upright)
PLATE_MIDTHICK = .0099
INSERT_AIM = .008                    # plate mid-thickness at its lowest point aimed 8 mm toward the back tine (-normal);
                                     # gpt_engineer p2_lean finding: a larger aim offset cuts the seated lean
BOWL_X_LOCAL = [-.035, .093]         # bowl spots along the prop wire (basket frame); as far front as the basket allows (wrist clears the tub face)
BOWL_TILT = math.radians(40)         # bowl axis 40 deg from vertical, opening toward +y (plate section) and up (rest pose on the prop wire)
BOWL_Y_LOCAL = -.155                 # bowl origin (outer bottom centre) y in the basket frame
PLATE_SEAT_Z = dw.WIRE_R             # floor-wire top in basket frame
SPEED = 2.5                          # sim seconds per video second
W, H = 1280, 720
TRANSIT_Z = 1.30                     # cup height for carries (carry path stays in front of / beside the dishwasher)
PILE_Z = 1.22                        # cup height above the piles
MID = np.array([.20, .32])           # carry waypoint in front of the dishwasher face (x > 0)
import os
DEBUG = bool(os.environ.get('SD_DEBUG'))

# bowl geometry: ../poc_bowl_grasp (rim OD 0.115, height 0.060, foot ring OD 0.050, 0.180 kg)
RIM_R, BOWL_HEIGHT, FOOT_Z = .0575, .06, -.004
BOWL_RGBA = [.55, .72, .86, 1]
BOWL_IN_BOTTOM = .004                # inside bottom surface height (bowl frame)


def add_bowl(spec, name, pos, quat=(1, 0, 0, 0), rgba=BOWL_RGBA):
    """Bowl from ../poc_bowl_grasp (copied from multi_dish_b.add_bowl); local z = axis, opening toward +z."""
    bowl = spec.worldbody.add_body(name=name, pos=list(pos), quat=list(quat))
    bowl.add_freejoint(name=f'{name}_free')
    common = dict(friction=[.6, .005, .0001], condim=4, priority=1, solref=[.006, 1], solimp=[.95, .99, .001, .5, 2], rgba=rgba)
    rim_apothem = RIM_R * math.cos(math.pi / 24)
    slope = math.atan2(rim_apothem - .030, .056)
    for i in range(24):
        a = i * 2 * math.pi / 24
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.sin(slope) * radial + np.array([0, 0, math.cos(slope)])
        normal = math.cos(slope) * radial - np.array([0, 0, math.sin(slope)])
        half_length = (.028 - .002 * math.sin(slope)) / math.cos(slope)
        centre_r = rim_apothem - .002 * math.cos(slope) - half_length * math.sin(slope)
        bowl.add_geom(name=f'{name}_wall_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre_r * radial + [0, 0, .032]),
                      quat=S.quat(np.column_stack([normal, tangent, along])),
                      size=[.002, RIM_R * math.sin(math.pi / 24), half_length], mass=.120 / 24, **common)
    bowl.add_geom(name=f'{name}_bottom', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .002], size=[.030, .002, 0],
                  mass=.045, **common)
    for i in range(16):
        a = i * 2 * math.pi / 16
        r = .025 * math.cos(math.pi / 16) - .002
        bowl.add_geom(name=f'{name}_foot_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[r * math.cos(a), r * math.sin(a), FOOT_Z],
                      quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)], size=[.002, .025 * math.sin(math.pi / 16), .004],
                      mass=.015 / 16, **common)


def add_pump(spec):
    """Franka Cobot Pump on the flange (panda/attachment, z = flange normal). Collision geometry = housing cylinder,
    stem cylinder, compliant lip spheres. The cup is its own body so the adhesion actuator acts only on lip contacts."""
    att = spec.body('panda/attachment')
    pump = att.add_body(name='pump', pos=[0, 0, 0])
    CYL, SPH = mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE
    pump.add_geom(name='pump_housing', type=CYL, pos=[0, 0, PUMP_L / 2], size=[PUMP_R, PUMP_L / 2, 0], mass=PUMP_MASS - .03,
                  rgba=[.18, .19, .21, 1])
    pump.add_geom(name='pump_band', type=CYL, pos=[0, 0, PUMP_L * .7], size=[PUMP_R + .0008, .006, 0], rgba=[.05, .35, .75, 1],
                  contype=0, conaffinity=0, mass=0)
    pump.add_geom(name='pump_stem', type=CYL, pos=[0, 0, PUMP_L + ADAPT_L / 2], size=[ADAPT_R, ADAPT_L / 2, 0], mass=.03,
                  rgba=[.6, .62, .65, 1])
    cup = pump.add_body(name='cup', pos=[0, 0, PUMP_L + ADAPT_L])
    if CUP_GIMBAL and GIMBAL_ON_BUILD[0]:   # bellows tilt compliance (sink demo): spring-damped ball joint, +-10 deg
        cup.add_joint(name='cup_gimbal', type=mujoco.mjtJoint.mjJNT_BALL, stiffness=CUP_GIMBAL_K, damping=.05, armature=1e-4,
                      range=[0, math.radians(10)], limited=mujoco.mjtLimited.mjLIMITED_TRUE)
    cup.add_geom(name='cup_bellows', type=CYL, pos=[0, 0, CUP_H / 2 - .001], size=[CUP_R - .002, CUP_H / 2 - .001, 0],
                 rgba=[.08, .08, .09, 1], contype=0, conaffinity=0, mass=CUP_MASS)
    for i in range(LIP_N):
        a = 2 * math.pi * i / LIP_N
        cup.add_geom(name=f'cup_lip{i}', type=SPH, size=[LIP_SR, 0, 0],
                     pos=[(CUP_R - LIP_SR) * math.cos(a), (CUP_R - LIP_SR) * math.sin(a), CUP_H - LIP_SR],
                     rgba=[.1, .1, .1, 1], mass=.001, priority=2, friction=[CUP_MU, .005, .0001], condim=3,
                     solref=[.015, 1], solimp=[.9, .95, .002, .5, 2])
    cup.add_site(name='tcp', pos=[0, 0, CUP_H], size=[.003, 0, 0], rgba=[1, .3, .1, 0])
    for b in (pump, cup):
        b.gravcomp = 1                      # payload model of the arm controller (as for the Franka hand)
    spec.add_actuator(name='suction', target='cup', trntype=mujoco.mjtTrn.mjTRN_BODY,
                      gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[F_CAP] + [0] * 9,
                      biastype=mujoco.mjtBias.mjBIAS_NONE, ctrlrange=[0, 1], ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE)


def attach_robot(spec, site):
    spec.attach(mujoco.MjSpec.from_file(str(PANDA_NOHAND)), prefix='panda/', site=site)
    for body in spec.bodies:
        if body.name.startswith('panda/'):
            body.gravcomp = 1
    add_pump(spec)


EXTRA_BUILD = []                     # kitchen_demo.py hooks (clutter, wall camera); empty for the plain demo


def build(objects=True, env=True):
    spec = mujoco.MjSpec.from_string(S._header())
    if env:
        S._add_kitchen(spec)
        dw.add_dishwasher(spec, pos=S.DW_POS, yaw=0.0, prefix='dw/')
    site = S._add_robot(spec, MOUNT)
    attach_robot(spec, site)
    if objects:
        tints = [[.96, .96, .93, 1], [.97, .93, .86, 1], [.92, .95, .97, 1]]
        for i in range(N_PLATES):        # plate0 = bottom ... plate2 = top
            S._add_plate(spec, None, 0, pos=[*PLATE_PILE, S.COUNTER_Z + .0003 + .0105 * i], R=np.eye(3), name=f'plate{i}', rgba=tints[i])
        for j in range(N_BOWLS):         # bowl0 = bottom ... bowl1 = top (nested)
            add_bowl(spec, f'bowl{j}', [*BOWL_PILE, S.COUNTER_Z + .0083 + .016 * j],
                     rgba=[[.55, .72, .86, 1], [.86, .62, .52, 1]][j])
        for f in EXTRA_BUILD:
            f(spec)
    return spec.compile()


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def T(R, p):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, p
    return M


R_DOWN = np.column_stack([[0, 1., 0], [1., 0, 0], [0, 0, -1.]])   # tcp z (tool axis, flange->lip) pointing down
R_DOWN_PLATE = R_DOWN @ np.array([[0, -1., 0], [1., 0, 0], [0, 0, 1.]])   # tool roll 90 deg: wrist clears the basket front rail (clear_test.py)


# ------------------------------------------------------------------ IK (mink on a robot-only kinematic model)
class IK:
    def __init__(self):
        GIMBAL_ON_BUILD[0] = False
        self.m = build(objects=False, env=False)
        GIMBAL_ON_BUILD[0] = True
        self.cfg = mink.Configuration(self.m)
        self.task = mink.FrameTask('tcp', 'site', position_cost=1., orientation_cost=1., lm_damping=1e-3)
        self.post = mink.PostureTask(self.m, cost=1e-3)
        self.limits = [mink.ConfigurationLimit(self.m, gain=.5)]
        self.q_nom = np.array([0, -.2, 0, -2.1, 0, 1.95, .785])
        self.post.set_target(self.q_nom)

    def solve(self, p, R, q0, iters=300, tol=(5e-4, 2e-3)):
        self.cfg.update(np.array(q0, float))
        self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R), p))
        for _ in range(iters):
            v = mink.solve_ik(self.cfg, [self.task, self.post], 1., solver='daqp', damping=1e-4, limits=self.limits)
            self.cfg.integrate_inplace(v, 1.)
            e = self.task.compute_error(self.cfg)
            if np.linalg.norm(e[:3]) < tol[0] and np.linalg.norm(e[3:]) < tol[1]:
                return self.cfg.q.copy(), True
        e = self.task.compute_error(self.cfg)
        return self.cfg.q.copy(), bool(np.linalg.norm(e[:3]) < 3e-3 and np.linalg.norm(e[3:]) < 2e-2)

    SEEDS = [np.array(q) for q in ([0, -.2, 0, -2.1, 0, 1.95, .785], [0, .3, 0, -1.6, 0, 3.0, .785], [0, -.6, 0, -2.6, 0, 3.4, .785],
                                   [.5, 0, 0, -2.0, 0, 2.5, 0], [-.5, 0, 0, -2.0, 0, 2.5, 1.5], [0, .5, 0, -1.2, 0, 2.2, -.8])]

    def solve_multi(self, p, R, q_cur):
        """Current q first, then fixed seeds; return the converged solution closest to q_cur."""
        best = None
        for sd in [q_cur] + self.SEEDS:
            q, ok = self.solve(p, R, sd, iters=400)
            if ok:
                dq = float(np.linalg.norm(q - q_cur))
                if best is None or dq < best[1]:
                    best = (q, dq)
                if sd is q_cur and dq < 1.0:
                    break
        return (best[0], True) if best else (q_cur, False)

    def line(self, p0, R0, p1, R1, q0, step=.01):
        """Straight Cartesian line (SLERP orientation) -> joint waypoints."""
        n = max(2, int(math.ceil(max(np.linalg.norm(p1 - p0) / step, 1))) + 1)
        r0, r1 = mink.SO3.from_matrix(R0), mink.SO3.from_matrix(R1)
        d = (r0.inverse() @ r1).log()
        qs, q, ok = [], np.array(q0, float), True
        for k in range(1, n + 1):
            s = k / n
            R = (r0 @ mink.SO3.exp(d * s)).as_matrix()
            q, o = self.solve(p0 + (p1 - p0) * s, R, q, iters=100)
            ok &= o
            qs.append(q)
        return qs, ok


# ------------------------------------------------------------------ HUD / video
class Video:
    def __init__(self, model, path):
        self.path = path
        self.r = mujoco.Renderer(model, H, W)
        self.cam = mujoco.MjvCamera()
        v = [float(x) for x in os.environ.get('SD_VIEW', '0.00 0.30 1.02 1.75 220 -42').split()]
        self.cam.lookat[:] = v[:3]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = v[3:]
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0
        fp = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
        self.f_s, self.f, self.f_b = (ImageFont.truetype(fp, s) for s in (17, 22, 26))
        self.proc = None
        if path is not None:
            self.proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                          '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-preset', 'medium',
                                          '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)],
                                         stdin=subprocess.PIPE)
        self.frames = 0
        self.next_t = 0.

    def render(self, data, hud):
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        img = Image.fromarray(self.r.render())
        return hud.draw(img, data.time, self)

    def capture(self, data, hud, force=False):
        if self.proc is None or (data.time + 1e-9 < self.next_t and not force):
            return
        self.next_t = data.time + SPEED / 30
        self.proc.stdin.write(np.asarray(self.render(data, hud)).tobytes())
        self.frames += 1

    def still(self, data, hud, path):
        self.render(data, hud).save(path)

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            self.proc.wait()
        self.r.close()


class Hud:
    def __init__(self):
        self.caption = ''
        self.items = []          # [label, status None/True/False, detail]
        self.final = False
        self.force = 0.
        self.seal = False

    def draw(self, img, t, v):
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 0, W, 34), fill=(18, 22, 28, 205))
        d.text((12, 7), 'Franka Panda + Franka Cobot Pump (Ø40 mm suction cup) destacks plates & bowls into a dishwasher  ·  MuJoCo, contact-force suction',
               fill=(225, 230, 236), font=v.f_s)
        x0, y0 = W - 330, 48
        d.rectangle((x0 - 12, y0 - 8, W - 10, y0 + 26 * len(self.items) + 30), fill=(18, 22, 28, 180))
        col = (90, 230, 120) if self.seal else (170, 175, 180)
        d.text((x0, y0), f'vacuum: {"SEALED" if self.seal else "off / open"}   F = {self.force:4.1f} / {F_CAP:.0f} N', fill=col, font=v.f_s)
        for i, (lab, st, det) in enumerate(self.items):
            mark, c = ('✓', (90, 230, 120)) if st is True else (('✗', (255, 90, 80)) if st is False else ('·', (180, 185, 190)))
            d.text((x0, y0 + 28 + 26 * i), f'{mark} {lab}', fill=c, font=v.f_s)
        d.rectangle((0, H - 48, W, H), fill=(18, 22, 28, 235))
        cap = self.caption + (f'  ·  suction {self.force:.0f} N' if self.seal else '')
        d.text((16, H - 40), cap, fill=(255, 225, 130), font=v.f_b if len(cap) < 92 else v.f)
        d.text((W - 160, H - 36), f't={t:5.1f}s  ×{SPEED:g}', fill=(200, 210, 220), font=v.f_s)
        if self.final:
            bw, bh = 700, 84 + 40 * len(self.items)
            bx, by = 20, H - bh - 62
            d.rectangle((bx, by, bx + bw, by + bh), fill=(14, 18, 24, 228), outline=(90, 230, 120), width=3)
            d.text((bx + 24, by + 18), 'Final check (poses measured from physics after settling)', fill='white', font=v.f_b)
            for i, (lab, st, det) in enumerate(self.items):
                c = (90, 230, 120) if st else (255, 90, 80)
                d.text((bx + 24, by + 66 + 40 * i), f'{"✓" if st else "✗"}  {lab}   {det}', fill=c, font=v.f)
        return img


class Fail(Exception):
    pass


# ------------------------------------------------------------------ controller
class Demo:
    def __init__(self, video_path=None, keyframes=False, log=print):
        self.m = build()
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        self.log = log
        self.ik = IK()
        self.jq = [m.joint(f'panda/joint{i}').qposadr[0] for i in range(1, 8)]
        self.act = [m.actuator(f'panda/actuator{i}').id for i in range(1, 8)]
        self.suc = m.actuator('suction').id
        self.cup = m.body('cup').id
        self.lip = [m.geom(f'cup_lip{i}').id for i in range(LIP_N)]
        self.tcp_site = m.site('tcp').id
        self.robot_bodies = {b for b in range(m.nbody) if m.body(b).name.startswith('panda/') or m.body(b).name in ('pump', 'cup')}
        dw.set_state(m, d, door_deg=90., basket=dw.BASKET_TRAVEL)       # simplification: door open, basket out
        q0 = self.ik.q_nom.copy()
        p_home = np.array([.25, .30, 1.30])
        q0, ok = self.ik.solve(p_home, R_DOWN, q0)
        self.p_home = p_home
        d.qpos[self.jq] = q0
        d.ctrl[self.act] = q0
        mujoco.mj_forward(m, d)
        self.q_ref = q0.copy()
        self.cmd = (p_home, R_DOWN)
        self.vac_cmd = False
        self.vac = 0.
        self.held = None            # body id of the object currently under the cup (seal)
        self.hud = Hud()
        self.video = Video(m, video_path) if (video_path or keyframes) else None
        if self.video and video_path is None:
            self.video.proc = None
        self.keyframes = keyframes
        self.kf = 0
        self.stats = dict(peak_suction_N=0., seal_losses=[], unexpected_contacts={}, max_pen_mm=0.)
        self.per_item = {}
        self.cur_item = None
        self.t_wall = time.monotonic()
        self.held_since = None
        self.carrying = False

    # -------------------------------------------------------------- suction: seal check + vacuum + adhesion force
    def seal_state(self):
        """Seal ACQUISITION: all 8 lip points touching (contact, dist <= 0) the same object, and the cup axis within
        SEAL_MAX_DEG of the contact normals. Returns (body, n_lips, worst_angle) or (None, n, angle)."""
        m, d = self.m, self.d
        axis = d.site_xmat[self.tcp_site].reshape(3, 3)[:, 2]
        touched = {}
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if g1 in self.lip:
                lg, og = g1, g2
            elif g2 in self.lip:
                lg, og = g2, g1
            else:
                continue
            if c.dist > 0:
                continue
            nrm = np.array(c.frame[:3]) * (1 if g1 == lg else -1)     # from the lip into the object
            ang = math.degrees(math.acos(min(1., abs(float(np.dot(nrm, axis))))))
            prev = touched.get(lg)
            if prev is None or ang < prev[2]:
                touched[lg] = (m.geom_bodyid[og], og, ang, nrm)
        if len(touched) < LIP_MIN_TOUCH:
            return None, len(touched), None
        bodies = {v[0] for v in touched.values()}
        worst = max(v[2] for v in touched.values())
        if len(bodies) != 1 or worst > SEAL_MAX_DEG:
            return None, len(touched), worst
        if len(touched) < LIP_N:                     # bellows-lip compliance: untouched lips must be within SEAL_DIST
            ft = np.zeros(6)
            gs = {v[1] for v in touched.values()}
            any_v = next(iter(touched.values()))
            for lg in self.lip:
                if lg in touched:
                    continue
                dmin = min(mujoco.mj_geomDistance(m, d, lg, og, .01, ft) for og in gs)
                if dmin > SEAL_DIST:
                    return None, len(touched), worst
                og = min(gs, key=lambda g: mujoco.mj_geomDistance(m, d, lg, g, .01, ft))
                touched[lg] = (any_v[0], og, any_v[2], any_v[3])
        self._acq = touched
        return bodies.pop(), LIP_N, worst

    def seal_kept(self):
        """Seal MAINTENANCE: every lip within SEAL_DIST of the geom it sealed on (mj_geomDistance) and the cup axis
        within SEAL_MAX_DEG of the surface normal (normal stored in the object frame at acquisition)."""
        m, d = self.m, self.d
        axis = d.site_xmat[self.tcp_site].reshape(3, 3)[:, 2]
        Ro = d.xmat[self.held].reshape(3, 3)
        ang = math.degrees(math.acos(min(1., abs(float(np.dot(Ro @ self.seal_n_local, axis))))))
        ft = np.zeros(6)
        worst = max(mujoco.mj_geomDistance(m, d, lg, og, .01, ft) for lg, og in self.seal_pairs)
        self.seal_gap = worst
        return worst <= SEAL_KEEP and ang <= SEAL_MAX_DEG, worst, ang

    def suction_update(self):
        dt = self.m.opt.timestep
        if not self.vac_cmd:
            self.vac, self.held = 0., None
            sealed = False
        elif self.held is None:                                  # trying to acquire
            body, n, ang = self.seal_state()
            sealed = body is not None and self.m.body(body).name.startswith(('plate', 'bowl'))
            if sealed:
                self.held = body
                Ro = self.d.xmat[body].reshape(3, 3)
                nrm = np.mean([v[3] for v in self._acq.values()], axis=0)
                self.seal_n_local = Ro.T @ (nrm / np.linalg.norm(nrm))
                self.seal_pairs = [(lg, v[1]) for lg, v in self._acq.items()]
        else:
            sealed, gap, ang = self.seal_kept()
            if not sealed:
                if self.vac > .5:
                    self.stats['seal_losses'].append(dict(t=round(self.d.time, 3), item=self.cur_item, lip_gap_mm=round(gap * 1000, 2),
                                                          axis_vs_normal_deg=round(ang, 1)))
                    self.per_item.setdefault(self.cur_item, dict(peak_suction_N=0., seal_losses=0))['seal_losses'] += 1
                    self.log(f'  !! seal lost t={self.d.time:.2f} item={self.cur_item} lip gap {gap * 1000:.2f} mm, angle {ang:.1f} deg')
                self.held = None
        if sealed:
            self.vac += (1 - self.vac) * dt / VAC_TAU
            if self.cur_item and int(round(self.d.time / dt)) % 10 == 0:
                fn = 0.
                fc = np.zeros(6)
                for i in range(self.d.ncon):
                    c = self.d.contact[i]
                    if c.geom1 in self.lip or c.geom2 in self.lip:
                        mujoco.mj_contactForce(self.m, self.d, i, fc)
                        fn += fc[0]
                pi = self.per_item.setdefault(self.cur_item, dict(peak_suction_N=0., seal_losses=0))
                pi['peak_lip_contact_N'] = round(max(pi.get('peak_lip_contact_N', 0.), fn), 1)
                pi['min_lip_contact_N'] = round(min(pi.get('min_lip_contact_N', 1e9), fn), 1)
        else:
            self.vac = 0.
        self.d.ctrl[self.suc] = self.vac
        self.hud.seal = sealed
        f = float(self.vac * F_CAP)
        self.hud.force = f
        self.stats['peak_suction_N'] = max(self.stats['peak_suction_N'], f)
        if self.cur_item:
            pi = self.per_item.setdefault(self.cur_item, dict(peak_suction_N=0., seal_losses=0))
            pi['peak_suction_N'] = round(max(pi['peak_suction_N'], f), 2)

    # -------------------------------------------------------------- stepping
    def monitor(self):
        m, d = self.m, self.d
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist < 0:
                self.stats['max_pen_mm'] = max(self.stats['max_pen_mm'], -c.dist * 1000)
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = b1 in self.robot_bodies, b2 in self.robot_bodies
            if r1 == r2:
                continue
            g_r, b_o = (c.geom1, b2) if r1 else (c.geom2, b1)
            if g_r in self.lip and b_o == self.held:
                continue
            if g_r in self.lip and self.m.body(b_o).name.startswith(('plate', 'bowl')):
                continue        # cup touching the item it is about to pick
            if c.dist > 0:
                continue
            f = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, f)
            g_o = c.geom2 if r1 else c.geom1
            key = f'{m.geom(g_r).name or m.body(m.geom_bodyid[g_r]).name} x {m.geom(g_o).name or m.body(b_o).name}'
            uc = self.stats['unexpected_contacts']
            if key not in uc:
                uc[key] = dict(first_t=round(d.time, 2), max_N=0., item=self.cur_item)
                if DEBUG:
                    self.log(f'  contact {key} at t={d.time:.2f} basket_x={self.basket()[0]:.4f} tcp={np.round(self.tcp()[0], 3)} pos={np.round(c.pos, 3)}\n     qref-q={np.round(self.q_ref - d.qpos[self.jq], 3)} qref={np.round(self.q_ref, 2)} frc={np.round(d.actuator_force[self.act], 1)}')
            uc[key]['max_N'] = round(max(uc[key]['max_N'], abs(f[0])), 2)

    def step(self, q=None):
        if self.carrying and self.vac < .5:
            self.carrying = False
            raise Fail(f'{self.cur_item} dropped: seal lost while carrying')
        if q is not None:
            self.q_ref = q
        self.d.ctrl[self.act] = self.q_ref
        self.suction_update()
        mujoco.mj_step(self.m, self.d)
        if int(self.d.time / self.m.opt.timestep) % 5 == 0:
            self.monitor()
        if self.video:
            self.video.capture(self.d, self.hud)

    def hold(self, dur):
        for _ in range(int(dur / self.m.opt.timestep)):
            self.step()

    def run_qpath(self, qs, dur, stop=None):
        """Execute joint waypoints with a min-jerk time law over dur seconds (piecewise-linear in joint space)."""
        qs = [self.q_ref.copy()] + list(qs)
        n = len(qs) - 1
        steps = max(1, int(dur / self.m.opt.timestep))
        for k in range(1, steps + 1):
            s = k / steps
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            x = s * n
            i = min(int(x), n - 1)
            q = qs[i] + (qs[i + 1] - qs[i]) * (x - i)
            self.step(q)
            if stop is not None and stop():
                return True
        return False

    def tcp(self):
        return self.d.site_xpos[self.tcp_site].copy(), self.d.site_xmat[self.tcp_site].reshape(3, 3).copy()

    def line(self, p, R, dur, stop=None):
        if DEBUG:
            self.log(f'  t={self.d.time:6.2f} line -> {np.round(p, 3)}  [{self.hud.caption[:60]}]')
        qs, ok = self.ik.line(self.cmd[0], self.cmd[1], p, R, self.q_ref)
        if not ok:
            raise Fail(f'IK line failed to {np.round(p, 3)}')
        hit = self.run_qpath(qs, dur, stop)
        self.cmd = (p, R)
        return hit

    def jmove(self, p, R, dur):
        if DEBUG:
            self.log(f'  t={self.d.time:6.2f} jmove -> {np.round(p, 3)}')
        q, ok = self.ik.solve_multi(p, R, self.q_ref)
        if not ok:
            raise Fail(f'IK failed for {np.round(p, 3)}')
        self.run_qpath([q], dur)
        self.cmd = (p, R)

    def key(self, name):
        if self.keyframes and self.video:
            self.video.still(self.d, self.hud, FRAMES / f'{self.kf:02d}_{name}.png')
            self.kf += 1

    # -------------------------------------------------------------- geometry helpers
    def pose(self, body):
        b = self.m.body(body).id
        return self.d.xmat[b].reshape(3, 3).copy(), self.d.xpos[b].copy()

    def basket(self):
        return self.d.xpos[self.m.body('dw/basket').id].copy()

    def plate_target(self, k):
        """Plate body pose for an upright plate in slot k, leaning back PLATE_LEAN. The plate's LOWEST rim point is
        aimed INSERT_AIM behind the slot centre line, 4 mm above the floor wires (released there)."""
        lam = PLATE_LEAN
        n = np.array([math.cos(lam), 0, math.sin(lam)])
        up = np.array([-math.sin(lam), 0, math.cos(lam)])
        R = np.column_stack([-up, [0, 1., 0], n])               # local -x -> up (cup side), local z (top face) -> front
        b = self.basket()
        slot_x = (dw.TINE_X[k] + dw.TINE_X[k + 1]) / 2
        slot = b + np.array([slot_x, sum(dw.PLATE_ROWS_Y) / 2, PLATE_SEAT_Z])
        bottom_local = np.array([S.R_OUT, 0, PLATE_MIDTHICK])   # lowest point, mid-thickness (foot z=0 .. rim top z=.0198)
        bottom_world = slot + np.array([-INSERT_AIM, 0, .004 + S.PL_T / 2])
        return R, bottom_world - R @ bottom_local, slot

    def bowl_target(self, j):
        R = Rx(-BOWL_TILT)                                         # local z (opening) -> (0, sin45, cos45)
        # lowest point of the bowl geometry in this orientation (foot-ring edge) -> rests on the floor-wire tops
        b = self.basket()
        z_low = self.bowl_lowest(R)
        o = b + np.array([BOWL_X_LOCAL[j], BOWL_Y_LOCAL, PLATE_SEAT_Z - z_low])
        return R, o

    def bowl_lowest(self, R):
        m = self.m
        bid = m.body('bowl0').id
        zs = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid:
                continue
            sz = m.geom_size[g]
            Rg = np.zeros(9)
            mujoco.mju_quat2Mat(Rg, m.geom_quat[g])
            Rg = Rg.reshape(3, 3)
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                corners = [np.array([sx * sz[0], sy * sz[1], sz2 * sz[2]]) for sx in (-1, 1) for sy in (-1, 1) for sz2 in (-1, 1)]
            else:
                corners = [np.array([sz[0] * math.cos(a), sz[0] * math.sin(a), h * sz[1]]) for a in np.linspace(0, 2 * math.pi, 24) for h in (-1, 1)]
            for c in corners:
                zs.append((R @ (m.geom_pos[g] + Rg @ c))[2])
        return min(zs)

    def tcp_for(self, R_obj, p_obj, rel):
        Tt = T(R_obj, p_obj) @ np.linalg.inv(rel)
        return Tt[:3, :3], Tt[:3, 3]

    # -------------------------------------------------------------- pick by suction (straight down onto a face)
    POSE_TAG = 'ground-truth pose'

    def suction_pick(self, body, cup_local, R_tool, label, p_cup_override=None, obj_pose_override=None):
        if os.environ.get('SD_PICK_TILT'):      # seal-model test only: approach with the cup tilted vs the face normal
            R_tool = R_tool @ Rx(math.radians(float(os.environ['SD_PICK_TILT'])))
        R_o, p_o = self.pose(body)
        p_o0 = p_o.copy()
        p_cup = p_o + R_o @ cup_local if p_cup_override is None else np.array(p_cup_override, float)
        above = p_cup + [0, 0, .09]
        self.hud.caption = f'{label}: move above the pile ({self.POSE_TAG})'
        self.lift_to(TRANSIT_Z)
        if self.cmd[0][0] > .05:
            self.jmove(np.r_[MID, TRANSIT_Z], R_tool, 2.0)
        self.jmove(np.r_[above[:2], PILE_Z], R_tool, 2.2)
        self.line(above, R_tool, 1.2)
        self.hud.caption = f'{label}: descend straight down onto the face, cup normal to the surface'
        self.vac_cmd = True
        sealed = lambda: self.held is not None and self.vac > .02
        hit = self.line(p_cup - [0, 0, .003], R_tool, 1.6, stop=sealed)
        self.hold(.6)                                            # vacuum builds (tau 0.15 s)
        if DEBUG:
            pt, Rt = self.tcp()
            self.log(f'  pick diag: tcp {np.round(pt, 4)} target {np.round(p_cup, 4)} seal {self.seal_state()} held {self.held} vac {self.vac:.2f}')
        if self.held != self.m.body(body).id or self.vac < .9:
            raise Fail(f'no seal on {body} (held={self.held}, vac={self.vac:.2f}, hit={hit})')
        self.cmd = (self.tcp()[0], R_tool)
        p_seal, R_seal = self.tcp()                              # cup pose at seal (robot kinematics)
        self.carrying = True
        self.key(f'{body}_sealed')
        self.hud.caption = f'{label}: sealed, {self.vac * F_CAP:.0f} N - lift the top item off the pile'
        p0 = self.tcp()[0]
        self.line(p0 + [0, 0, .12], R_tool, 1.8)
        self.hold(.3)
        R_o, p_o = self.pose(body)
        p_t, R_t = self.tcp()
        if obj_pose_override is None:
            rel = np.linalg.inv(T(R_t, p_t)) @ T(R_o, p_o)      # object-in-cup transform from simulator truth (plain demo)
            rise = p_o[2] - p_o0[2]                              # lift check from simulator truth (plain demo)
        else:                                                    # perception-only: perceived object pose at the seal
            rel = np.linalg.inv(T(R_seal, p_seal)) @ T(*obj_pose_override)
            rise = 1. if (self.held is not None and self.vac > .9) else 0.   # lift check = vacuum still sealed (pump sensor)
        if rise < .08:
            raise Fail(f'{body} did not lift (rise {rise:.3f})')
        self.lift_to(PILE_Z)
        return rel

    def lift_to(self, z):
        p, R = self.cmd
        if p[2] < z - 1e-3:
            self.line(np.r_[p[:2], z], R, max(.8, (z - p[2]) / .12))

    def release(self, back_dir, label):
        self.hud.caption = f'{label}: vent the vacuum (seal released), back the cup off'
        self.carrying = False
        self.vac_cmd = False
        self.hold(.4)
        p, R = self.cmd
        self.line(p + back_dir * .03 + [0, 0, .01], R, 1.2)

    # -------------------------------------------------------------- items
    def do_plate(self, i, n):
        body = f'plate{N_PLATES - 1 - i}'                       # top of the pile first
        k = PLATE_SLOTS[i]
        label = f'Plate {i + 1}/{n} → slot {k + 1}'
        self.cur_item = body
        cup_local = np.array([-PLATE_CUP_OFF, 0, .010])          # disc top face, 40 mm toward -x (becomes 'up' in the slot)
        rel = self.suction_pick(body, cup_local, R_DOWN_PLATE, label)
        R_p, p_p, _ = self.plate_target(k)
        R_t, p_t = self.tcp_for(R_p, p_p, rel)
        self.hud.caption = f'{label}: carry, rotate upright (cup high on the face, tool leans back 10°)'
        hub = p_t + np.array([.05, 0, .15])
        self.jmove(np.r_[MID, TRANSIT_Z], R_DOWN_PLATE, 2.4)
        self.jmove(hub, R_t, 2.8)
        self.line(p_t + [0, 0, .10], R_t, 1.4)
        self.hud.caption = f'{label}: lower between the tines (back → front order: tool never over a placed plate)'
        self.key(f'plate{i + 1}_insert')
        self.line(p_t, R_t, 2.0)
        self.hold(.2)
        n_dir = R_t[:, 2]
        self.release(-n_dir, label)
        p, R = self.cmd
        self.line(p + [0, 0, .12], R, 1.2)
        self.cur_item = None

    def do_bowl(self, j, n):
        body = f'bowl{N_BOWLS - 1 - j}'                         # top of the nest first
        label = f'Bowl {j + 1}/{n} → bowl spot {j + 1}'
        self.cur_item = body
        cup_local = np.array([0, 0, BOWL_IN_BOTTOM])            # inside bottom, centre
        rel = self.suction_pick(body, cup_local, R_DOWN, label)
        R_b, p_b = self.bowl_target(j)
        p_b = p_b + [0, 0, .004]
        R_t, p_t = self.tcp_for(R_b, p_b, rel)
        self.hud.caption = f'{label}: carry and tilt 40° (opening toward the plate section)'
        hub = p_t + np.array([.06, .08, .18])
        self.jmove(np.r_[MID, TRANSIT_Z], R_DOWN, 2.4)
        self.jmove(hub, R_t, 2.8)
        self.line(p_t + [0, 0, .08], R_t, 1.4)
        self.hud.caption = f'{label}: set down on the floor wires against the bowl prop'
        self.key(f'bowl{j + 1}_place')
        self.line(p_t, R_t, 1.6)
        self.hold(.2)
        self.release(-R_t[:, 2], label)
        p, R = self.cmd
        self.line(p + [0, .04, .07], R, 1.2)                    # out of the bowl, over the basket
        self.line(p + [.10, .04, .07], R, 1.2)                  # away from the dishwasher face before going higher
        self.cur_item = None

    # -------------------------------------------------------------- checks (measured)
    def plate_check(self, i):
        body = f'plate{N_PLATES - 1 - i}'
        k = PLATE_SLOTS[i]
        R, p = self.pose(body)
        c = p + R @ np.array([0, 0, S.MID_Z])
        _, _, slot = self.plate_target(k)
        n = R[:, 2]
        lean = math.degrees(math.asin(min(1., abs(n[2]))))      # 0 = perfectly upright
        yaw = math.degrees(math.atan2(abs(n[1]), abs(n[0])))
        rim = [p + R @ np.array([S.R_OUT * math.cos(a), S.R_OUT * math.sin(a), PLATE_MIDTHICK]) for a in np.linspace(0, 2 * math.pi, 145)]
        lowest = min(rim, key=lambda v: v[2])                    # lowest point of the plate mid-thickness circle = between the tines
        low = lowest[2] - slot[2]
        dx, dy = lowest[0] - slot[0], c[1] - slot[1]
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, self.m.body(body).id, v, 0)
        robot = self.touch_robot(body)
        ok = abs(dx) < .012 and abs(dy) < .02 and lean < 15 and yaw < 10 and low < .015 and np.linalg.norm(v[3:]) < .005 and not robot
        return bool(ok), dict(bottom_dx_mm=round(dx * 1000, 1), dy_mm=round(dy * 1000, 1), rim_above_wires_mm=round(low * 1000, 1),
                              lean_deg=round(lean, 1), yaw_deg=round(yaw, 1), speed_mm_s=round(float(np.linalg.norm(v[3:])) * 1000, 2),
                              touching_robot=robot)

    def bowl_check(self, j):
        body = f'bowl{N_BOWLS - 1 - j}'
        R, p = self.pose(body)
        R_t, p_t = self.bowl_target(j)
        e = p - p_t
        tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
        az = math.degrees(math.atan2(R[1, 2], R[0, 2]))         # opening azimuth, 90 = toward +y
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, self.m.body(body).id, v, 0)
        robot = self.touch_robot(body)
        ok = np.linalg.norm(e) < .02 and abs(tilt - math.degrees(BOWL_TILT)) < 10 and abs(az - 90) < 15 and \
            np.linalg.norm(v[3:]) < .005 and not robot
        return bool(ok), dict(pos_err_mm=round(float(np.linalg.norm(e)) * 1000, 1), err_xyz_mm=[round(float(x) * 1000, 1) for x in e],
                              tilt_deg=round(tilt, 1), opening_azimuth_deg=round(az, 1),
                              speed_mm_s=round(float(np.linalg.norm(v[3:])) * 1000, 2), touching_robot=robot)

    def touch_robot(self, body):
        b = self.m.body(body).id
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if (b1 == b and b2 in self.robot_bodies) or (b2 == b and b1 in self.robot_bodies):
                if c.dist < 0:
                    return True
        return False

    # -------------------------------------------------------------- run
    def run(self, plates=N_PLATES, bowls=N_BOWLS):
        h = self.hud
        h.items = [[f'Bowl {j + 1} → bowl spot {j + 1}', None, ''] for j in range(bowls)] + \
                  [[f'Plate {i + 1} → slot {PLATE_SLOTS[i] + 1}', None, ''] for i in range(plates)]
        h.caption = 'Start: door open, basket out · 3 stacked plates + 2 nested bowls on the counter'
        self.hold(1.2)
        self.key('start')
        fails = []
        # bowls first: their tool pose leans over the (still empty) plate section
        for j in range(bowls):
            try:
                self.do_bowl(j, bowls)
                h.items[j][1] = self.bowl_check(j)[0]
            except Fail as e:
                fails.append(f'bowl {j}: {e}')
                h.items[j][1] = False
                self.log(f'FAIL bowl {j}: {e}')
                self.recover()
        for i in range(plates):
            try:
                self.do_plate(i, plates)
                h.items[bowls + i][1] = self.plate_check(i)[0]
            except Fail as e:
                fails.append(f'plate {i}: {e}')
                h.items[bowls + i][1] = False
                self.log(f'FAIL plate {i}: {e}')
                self.recover()
        h.caption = 'Done: robot returns home, everything settles (final poses measured)'
        self.lift_to(1.30)
        self.jmove(np.array([.20, .62, 1.30]), R_DOWN, 3.0)     # park beside the piles: basket in full view
        self.hold(2.0)
        table = []
        for j in range(bowls):
            ok, st = self.bowl_check(j)
            table.append(dict(item=f'bowl {j + 1} ({N_BOWLS - 1 - j and "top" or "bottom"} of nest)', target=f'bowl spot {j + 1}', ok=ok,
                              **self.per_item.get(f'bowl{N_BOWLS - 1 - j}', {}), **st))
            h.items[j][1] = ok
            h.items[j][2] = f'pos err {st["pos_err_mm"]:.1f} mm, tilt {st["tilt_deg"]:.0f}°'
        for i in range(plates):
            ok, st = self.plate_check(i)
            table.append(dict(item=f'plate {i + 1}', target=f'slot {PLATE_SLOTS[i] + 1}', ok=ok,
                              **self.per_item.get(f'plate{N_PLATES - 1 - i}', {}), **st))
            h.items[bowls + i][1] = ok
            h.items[bowls + i][2] = f'rim-in-slot dx {st["bottom_dx_mm"]:+.1f} mm, lean {st["lean_deg"]:.1f}°'
        h.final = True
        h.caption = f'{sum(r["ok"] for r in table)}/{len(table)} items placed · peak suction {self.stats["peak_suction_N"]:.0f} N (cap {F_CAP:.0f} N) · seal losses {len(self.stats["seal_losses"])}'
        self.hold(5.0)
        self.key('final')
        if self.video:
            self.video.close()
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(self.d.warning) if w.number}
        return dict(table=table, fails=fails, sim_time_s=round(self.d.time, 2), wall_s=round(time.monotonic() - self.t_wall, 1),
                    warnings=warnings, peak_suction_N=round(self.stats['peak_suction_N'], 2), F_cap_N=round(F_CAP, 2),
                    F_theory_N=round(F_THEORY, 2), seal_losses=self.stats['seal_losses'],
                    unexpected_robot_contacts=self.stats['unexpected_contacts'], max_penetration_mm=round(self.stats['max_pen_mm'], 2),
                    video_frames=self.video.frames if self.video else 0)

    def recover(self):
        self.carrying = False
        self.vac_cmd = False
        self.held = None
        self.cur_item = None
        p, R = self.tcp()
        self.cmd = (p, R)
        self.hold(.5)
        try:
            self.lift_to(1.30)
        except Fail:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--still', action='store_true')
    ap.add_argument('--plates', type=int, default=N_PLATES)
    ap.add_argument('--bowls', type=int, default=N_BOWLS)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    FRAMES.mkdir(exist_ok=True)
    if a.still:
        dmo = Demo(video_path=None, keyframes=True)
        dmo.hold(1.0)
        dmo.hud.items = [['Bowl 1', None, ''], ['Plate 1', None, '']]
        dmo.hud.caption = 'still test'
        dmo.video.still(dmo.d, dmo.hud, FRAMES / 'view_test.png')
        for nm in ('plate2', 'plate1', 'bowl1', 'bowl0'):
            R, p = dmo.pose(nm)
            print(nm, np.round(p, 4), 'tilt', round(math.degrees(math.acos(np.clip(R[2, 2], -1, 1))), 2))
        print('bowl target', [np.round(dmo.bowl_target(j)[1], 4) for j in range(2)], 'plate target', np.round(dmo.plate_target(2)[1], 4))
        print('ncon', dmo.d.ncon, 'F_cap', F_CAP)
        return
    vid = None if a.no_video else OUT / f'suction_demo{a.tag}.mp4'
    dmo = Demo(video_path=vid, keyframes=not a.no_video, log=lambda s: print(s, flush=True))
    res = dmo.run(a.plates, a.bowls)
    (OUT / f'result{a.tag}.json').write_text(json.dumps(res, indent=1, default=float))
    for r in res['table']:
        print(json.dumps(r, default=float))
    print('fails', res['fails'])
    print('peak', res['peak_suction_N'], 'cap', res['F_cap_N'], 'seal_losses', res['seal_losses'])
    print('unexpected', res['unexpected_robot_contacts'])
    print('sim', res['sim_time_s'], 'wall', res['wall_s'], 'warn', res['warnings'], 'pen', res['max_penetration_mm'], 'frames', res['video_frames'])


if __name__ == '__main__':
    main()
