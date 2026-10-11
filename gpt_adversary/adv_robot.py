"""Instantiate a DSL scenario in MuJoCo and run the scripted Runner (or hand the robot to the GPT-L1 commander).

Nothing in ../full_pipeline or ../gpt_control is modified: this module imports them and
  * replaces sim_robot.build_scene (which SimRobot installs as scene.build) by `build_adv`, which builds the same
    kitchen but with the scenario's physical changes (plate geometry/mass/friction/colour/pose, pod + dish, clutter
    bodies, dishwasher pose, cart dock error, bumped wall camera, person figure, preloaded basket slots);
  * changes joint / light parameters on the compiled model (door hinge, basket rail, lighting);
  * subclasses SimRobot: wrist F/T noise + drift, button actuation threshold / debounce, person flag checked every
    physics step, and ground-truth outcome extraction (clutter knocked over, objects on the floor, ...).
The robot's own knowledge is NOT changed: its IK / collision model (scene.build_ik), its plan.* constants (8" plate,
taught dish spot, nominal dishwasher pose) and the camera calibration stay nominal. That gap is what the scenario tests.
Physics: only mj_step. Object poses written only at build time.
"""
from __future__ import annotations

import copy
import math
import subprocess
import sys
import textwrap
import time
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

import numpy as np

ROOT = Path(__file__).resolve().parent
GC = _ROOT / 'gpt_control'
FP = _ROOT / 'full_pipeline'
sys.path.insert(0, str(GC))
sys.path.insert(0, str(FP))
sys.dont_write_bytecode = True

import mujoco                    # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

import codex_client as CC        # noqa: E402
import dishwasher as dw          # noqa: E402
import perception as PC          # noqa: E402
import pipeline as PL            # noqa: E402
import scene as S                # noqa: E402
import sim_robot as SR           # noqa: E402

import codex_util                # noqa: E402
import dsl                       # noqa: E402

codex_util.install()

# keep every artefact of codex calls inside gpt_adversary/
CC.WORK = ROOT / 'codex_work'
CC.LEDGER = ROOT / 'logs' / 'codex_calls.jsonl'
DEPTH_NOISE0 = PC.DEPTH_NOISE
DEG = math.pi / 180


def _q(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1., 0], [-s, 0, c]])


def cam_frame(pos, look):
    fwd = np.asarray(look, float) - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return np.column_stack([right, up, -fwd])


def kelvin_rgb(K):
    """Approximate black-body colour (Tanner Helland fit), normalised to max 1."""
    t = K / 100.
    r = 255. if t <= 66 else 329.698727446 * (t - 60) ** -0.1332047592
    g = 99.4708025861 * math.log(t) - 161.1195681661 if t <= 66 else 288.1221695283 * (t - 60) ** -0.0755148492
    b = 255. if t >= 66 else (0. if t <= 19 else 138.5177312231 * math.log(t - 10) - 305.0447927307)
    v = np.clip([r, g, b], 0, 255) / 255.
    return v / v.max()


# ====================================================================== scene construction
def add_plate(spec, s):
    """scene._add_plate with the scenario's radius / mass / friction / colour / face-down option."""
    p = s['plate']
    k = p['radius_m'] / S.R_OUT
    R_OUT, R_DISC, R_FOOT, PL_T, RIM_RISE = S.R_OUT * k, S.R_DISC * k, S.R_FOOT * k, S.PL_T, S.RIM_RISE * k
    slope = math.atan2(RIM_RISE, R_OUT - R_DISC)
    mu = max(.1, p['mu'] * (1 - .6 * p['wetness']))
    contact = dict(S.OBJ_CONTACT)
    contact['friction'] = [mu, .005, .0001]
    mscale = p['mass_kg'] / sum(S.PLATE_MASS.values())
    yaw = 0.
    Rb = _rz(yaw)
    z = S.COUNTER_Z + .0003
    if p['upside_down']:
        Rb = Rb @ _rx(math.pi)
        z = S.COUNTER_Z + .0003 + .0075 + RIM_RISE + PL_T / 2 + .001
    plate = spec.worldbody.add_body(name='plate', pos=[p['x'], p['y'], z], quat=list(_q(Rb)))
    plate.add_freejoint(name='plate_free')
    rgba = list(p['rgb']) + [1]
    plate.add_geom(name='plate_disc', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, .0075], size=[R_DISC + .002, PL_T / 2, 0],
                   mass=S.PLATE_MASS['disc'] * mscale, rgba=rgba, **contact)
    n = 36
    width = R_OUT - R_DISC
    half_len = .5 * math.hypot(width, RIM_RISE)
    for i in range(n):
        a = 2 * math.pi * i / n
        radial = np.array([math.cos(a), math.sin(a), 0.])
        tangent = np.array([-math.sin(a), math.cos(a), 0.])
        along = math.cos(slope) * radial + np.array([0, 0, math.sin(slope)])
        normal = np.cross(along, tangent)
        centre = (R_DISC + width / 2) * radial + np.array([0, 0, .0075 + RIM_RISE / 2])
        plate.add_geom(name=f'plate_rim{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(centre),
                       quat=list(_q(np.column_stack([along, tangent, normal]))),
                       size=[half_len, (R_DISC + width / 2) * math.sin(math.pi / n) * 1.15, PL_T / 2],
                       mass=S.PLATE_MASS['rim'] / n * mscale, rgba=rgba, **contact)
    m = 16
    for i in range(m):
        a = 2 * math.pi * i / m
        plate.add_geom(name=f'plate_foot{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                       pos=[R_FOOT * math.cos(a), R_FOOT * math.sin(a), .0025], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
                       size=[.002, R_FOOT * math.sin(math.pi / m) * 1.1, .0025], mass=S.PLATE_MASS['foot'] / m * mscale,
                       rgba=rgba, **contact)
    plate.add_site(name='plate_center', pos=[0, 0, S.MID_Z], size=[.003, 0, 0], rgba=[1, .3, .1, 1], group=4)


def add_pod_and_dish(spec, s):
    dc = np.array(dsl.DISH_C) + [s['dish']['dx'], s['dish']['dy']]
    dish = spec.worldbody.add_body(name='dish', pos=[dc[0], dc[1], S.COUNTER_Z + .0003])
    dish.add_freejoint(name='dish_free')
    c = [.62, .80, .78, 1]
    dish.add_geom(name='dish_floor', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[0, 0, S.DISH_FLOOR_T / 2],
                  size=[S.DISH_R, S.DISH_FLOOR_T / 2, 0], mass=.12, rgba=c, **S.OBJ_CONTACT)
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        dish.add_geom(name=f'dish_rim{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                      pos=[S.DISH_R * math.cos(a), S.DISH_R * math.sin(a), S.DISH_RIM_H / 2], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)],
                      size=[.003, S.DISH_R * math.sin(math.pi / n) * 1.12, S.DISH_RIM_H / 2], mass=.004, rgba=c, **S.OBJ_CONTACT)
    po = s['pod']
    size = np.array(S.POD_SIZE) * po['size_scale']
    if po['location'] == 'dish':
        x, y, z0 = dc[0] + po['dx'], dc[1] + po['dy'], S.COUNTER_Z + .0003 + S.DISH_FLOOR_T
    else:
        x, y, z0 = po['x'], po['y'], S.COUNTER_Z + .0003
    yaw = math.radians(po['yaw_deg'])
    pod = spec.worldbody.add_body(name='pod', pos=[x, y, z0 + size[2] / 2 + .0002], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    pod.add_freejoint(name='pod_free')
    pod.add_geom(name='pod_box', type=mujoco.mjtGeom.mjGEOM_BOX, size=list(size / 2), mass=po['mass_kg'],
                 rgba=[.25, .45, .85, 1], **S.OBJ_CONTACT)
    pod.add_geom(name='pod_stripe', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, size[2] / 2],
                 size=[.008 * po['size_scale'], size[1] / 2 + .0002, .0002], rgba=[.95, .95, .95, 1], contype=0, conaffinity=0,
                 mass=0, group=1)


def add_clutter(spec, s):
    d = s['dishwasher']
    for i, c in enumerate(s['clutter']):
        shape, dims, mass, _ = dsl.CLUTTER_TYPES[c['type']]
        k = c['size_scale']
        h = dims[-1] * k
        if c['surface'] == 'counter':
            x, y, z0 = c['x'], c['y'], S.COUNTER_Z
        else:
            a = math.radians(d['yaw_deg'])
            cx, cy = -dsl.DW_D / 2 + d['dx'], d['dy']
            x = cx + math.cos(a) * c['x'] - math.sin(a) * c['y']
            y = cy + math.sin(a) * c['x'] + math.cos(a) * c['y']
            z0 = S.COUNTER_Z + dsl.DW_H
        yaw = math.radians(c['yaw_deg'])
        b = spec.worldbody.add_body(name=f'clutter{i}', pos=[x, y, z0 + h / 2 + .0005], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        b.add_freejoint(name=f'clutter{i}_free')
        rgba = list(c['rgb']) + [1]
        kw = dict(friction=[.6, .005, .0001], solref=[.006, 1], solimp=[.95, .99, .001, .5, 2], condim=4)
        if shape == 'cyl':
            b.add_geom(name=f'clutter{i}_{c["type"]}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[dims[0] * k, h / 2, 0],
                       mass=mass, rgba=rgba, **kw)
        else:
            b.add_geom(name=f'clutter{i}_{c["type"]}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[dims[0] * k / 2, dims[1] * k / 2, h / 2],
                       mass=mass, rgba=rgba, **kw)


def add_preload(spec, s):
    bk = spec.body('dw/basket')
    for k in s['basket']['preload_slots']:
        x = (dw.TINE_X[k] + dw.TINE_X[k + 1]) / 2
        bk.add_geom(name=f'preload_plate{k}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[x, sum(dw.PLATE_ROWS_Y) / 2, dw.WIRE_R + dw.PLATE_R + .001],
                    quat=[math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0], size=[dw.PLATE_R, .005, 0], mass=.45,
                    rgba=[.93, .90, .80, 1], **dw.CONTACT)


def make_builder(s, nominal_layout):
    def build_adv(mount=S.DEFAULT_MOUNT, plate_xy=None, plate_yaw=0.0, with_objects=True, with_dishwasher=True,
                  grip_k=800., grip_limit=70., layout=S.DEFAULT_LAYOUT):
        spec = mujoco.MjSpec.from_string(S._header())
        S._add_kitchen(spec)
        # wall camera: true (bumped) pose rendered; the robot keeps the nominal calibration (BumpCamera.pose)
        cb = s['camera_bump']
        pos0 = np.array(layout.cam_pos, float)
        R0 = cam_frame(pos0, layout.cam_look)
        Rb = R0 @ _rz(math.radians(cb['roll_deg'])) @ _rx(math.radians(cb['pitch_deg'])) @ _ry(math.radians(cb['yaw_deg']))
        pb = pos0 + [cb['dx'], cb['dy'], cb['dz']]
        spec.worldbody.add_camera(name='wallcam', pos=list(pb), quat=list(_q(Rb)), fovy=S.CAM_FOVY)
        spec.worldbody.add_geom(name='cam_housing', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(pb + .03 * Rb[:, 2]),
                                quat=list(_q(Rb)), size=[.045, .015, .02], rgba=[.1, .1, .1, 1], contype=0, conaffinity=0)
        d = s['dishwasher']
        info = dw.add_dishwasher(spec, pos=(S.DW_POS[0] + d['dx'], S.DW_POS[1] + d['dy'], S.DW_POS[2]), yaw=math.radians(d['yaw_deg']),
                                 prefix='dw/')
        site = S._add_robot(spec, mount)
        dk = s['dock']
        if dk['dx'] or dk['dy'] or dk['yaw_deg']:
            # rotate the whole cart about the arm base axis, then translate
            a = math.radians(dk['yaw_deg'])
            pbase = np.array([mount.x, mount.y, 0.])
            cart = spec.body('cart')
            cart.pos = list(pbase + [dk['dx'], dk['dy'], 0] - _rz(a) @ pbase)
            cart.quat = [math.cos(a / 2), 0, 0, math.sin(a / 2)]
        S._attach_panda(spec, site, grip_k, grip_limit)
        add_plate(spec, s)
        add_pod_and_dish(spec, s)
        add_clutter(spec, s)
        add_preload(spec, s)
        # person (visual only, hidden until the flag fires), along the nominal bearing at the scenario distance
        per = s['person'] or {}
        dist = per.get('distance_m', .81)
        u = np.array(dsl.PERSON_BEARING) / np.linalg.norm(dsl.PERSON_BEARING)
        x, y = np.array(dsl.ARM_BASE) + dist * u
        wb = spec.worldbody
        for nm, kw in (('person_legs', dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[x, y, .12, x, y, .85], size=[.13, 0, 0])),
                       ('person_torso', dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[x, y, .95, x, y, 1.40], size=[.17, 0, 0])),
                       ('person_head', dict(type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[x, y, 1.66], size=[.11, 0, 0]))):
            wb.add_geom(name=nm, rgba=[.86, .48, .30, 0.], contype=0, conaffinity=0, group=1, **kw)
        m = spec.compile()
        # ---- appliance joints (compiled-model parameters, no derived constants)
        dr = s['door']
        j = m.joint('dw/door_hinge')
        m.jnt_stiffness[j.id] *= dr['spring_scale']
        m.dof_frictionloss[j.dofadr[0]] = dr['hinge_friction_Nm']
        m.dof_damping[j.dofadr[0]] = dr['damping']
        jb = m.joint('dw/basket_slide')
        m.dof_frictionloss[jb.dofadr[0]] = s['basket']['rail_friction_N']
        m.dof_damping[jb.dofadr[0]] = s['basket']['damping']
        # ---- lighting (render-only parameters)
        L = s['lighting']
        tint = kelvin_rgb(L['color_temp_K'])
        tint0 = kelvin_rgb(5500.)
        tint = tint / tint0 / max(tint / tint0)
        kid = m.light('key').id
        m.light_diffuse[kid] = m.light_diffuse[kid] * L['key_scale'] * tint
        m.light_specular[kid] = m.light_specular[kid] * L['key_scale']
        a = math.radians(L['key_azimuth_deg'])
        if a:
            c0 = np.array([-.10, .40, S.COUNTER_Z])
            m.light_pos[kid] = c0 + _rz(a) @ (m.light_pos[kid] - c0)
            m.light_dir[kid] = _rz(a) @ m.light_dir[kid]
        for nm in ('fill',):
            i = m.light(nm).id
            m.light_diffuse[i] = m.light_diffuse[i] * L['ambient_scale'] * tint
        m.vis.headlight.ambient[:] = np.array(m.vis.headlight.ambient) * L['ambient_scale'] * tint
        m.vis.headlight.diffuse[:] = np.array(m.vis.headlight.diffuse) * L['ambient_scale'] * tint
        return m, info
    return build_adv


class BumpCamera(PC.Camera):
    """Renders from the (possibly bumped) model camera, but back-projects with the robot's calibration = nominal pose."""
    CAL = None

    def pose(self, data):
        return self.CAL[0].copy(), self.CAL[1].copy()


# ====================================================================== video (streamed to ffmpeg, low memory)
class StreamRecorder(PL.Recorder):
    """PL.Recorder drawing into an ffmpeg pipe. Frames are written only after sim time `record_from`; playback speed
    `base_speed` x the pipeline's own speed hints; a caption bar names the scenario / failure mode."""

    def __init__(self, model, path, caption, record_from=0., base_speed=4., size=(960, 540), record_until=1e9):
        super().__init__(model, None, 'adversarial scenario')
        from util import renderer, scene_option
        from PIL import ImageFont
        self.r = renderer(model, 720, 1280)
        self.cam = mujoco.MjvCamera()
        self.opt = scene_option(hide_sites=True)
        fp = Path('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
        self.font = ImageFont.truetype(str(fp), 20)
        self.font_s = ImageFont.truetype(str(fp), 16)
        self.font_b = ImageFont.truetype(str(fp), 26)
        self.caption = caption
        self.record_from = record_from
        self.record_until = record_until
        self.base_speed = base_speed
        self.proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x720',
                                      '-r', '30', '-i', '-', '-vf', f'scale={size[0]}:{size[1]}', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                      '-crf', '26', str(path)], stdin=subprocess.PIPE)
        rec = self

        class W:
            def append_data(self, frame):
                rec.proc.stdin.write(np.ascontiguousarray(frame[:, :, :3]).tobytes())
        self.writer = W()
        self.status = {}

    def hud(self, img, t):
        img = super().hud(img, t)
        d = ImageDraw.Draw(img, 'RGBA')
        d.rectangle((0, 600, 1280, 676), fill=(60, 10, 10, 210))
        for k, ln in enumerate(textwrap.wrap(self.caption, 115)[:3]):
            d.text((14, 604 + 22 * k), ln, fill=(255, 225, 200), font=self.font_s)
        return img

    def capture(self, data, force=False):
        if data.time < self.record_from or data.time > self.record_until:
            return
        if data.time + 1e-9 < self.next_time and not force:
            return
        self.next_time = data.time + self.speed * self.base_speed / 30
        self.view += .06 * (self.view_target - self.view)
        self.cam.lookat[:] = self.view[:3]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = self.view[3:]
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        img = Image.fromarray(self.r.render())
        self.writer.append_data(np.asarray(self.hud(img, data.time)))
        self.frames += 1

    def card(self, data, image, text, seconds=3.):
        if data.time < self.record_from or data.time > self.record_until:
            return
        super().card(data, image, text, seconds * .5)

    def hold_frames(self, data, seconds):
        if self.view is None:
            return
        self.r.update_scene(data, camera=self.cam, scene_option=self.opt)
        fr = np.asarray(self.hud(Image.fromarray(self.r.render()), data.time))
        for _ in range(int(30 * seconds)):
            self.writer.append_data(fr)

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=120)
        finally:
            self.r.close()


# ====================================================================== the robot under test
class AdvRobot(SR.SimRobot):
    def __init__(self, scn, seed=1, run_tag='adv', outdir=None, video=None, caption='', record_from=0., base_speed=4.,
                 keyframes_dir=None, log=print, record_until=1e9):
        self.scn = s = copy.deepcopy(scn)
        SR.build_scene = make_builder(s, SR.OPT.layout)
        lay = SR.OPT.layout
        BumpCamera.CAL = (np.array(lay.cam_pos, float), cam_frame(np.array(lay.cam_pos, float), lay.cam_look))
        PC.Camera = BumpCamera
        PC.DEPTH_NOISE = DEPTH_NOISE0 * s['sensors']['depth_noise_scale']
        faults = {}
        if s['person']:
            faults['person'] = dict(at_s=s['person']['at_s'], dur_s=s['person']['dur_s'])
        outdir = Path(outdir or ROOT / 'runs' / run_tag)
        outdir.mkdir(parents=True, exist_ok=True)
        # plate pose is built by build_adv from the scenario; plate_off only feeds the Runner's bookkeeping
        off = (s['plate']['x'] - lay.plate0[0], s['plate']['y'] - lay.plate0[1], 0.)
        super().__init__(faults=faults, plate_off=off, video=None, seed=seed, run_tag=run_tag, outdir=outdir, log=log)
        if video is not None:
            self.rec = StreamRecorder(self.model, video, caption, record_from, base_speed, record_until=record_until)
        self.keyframes_dir = keyframes_dir
        self.ft_rng = np.random.default_rng(seed + 101)
        self.btn_thr = s['buttons']['threshold_N']
        self.btn_dwell = s['buttons']['dwell_ms'] / 1000.
        m, d = self.model, self.data
        self.clutter_ids = [m.body(f'clutter{i}').id for i in range(len(s['clutter']))]
        mujoco.mj_forward(m, d)
        self.clutter0 = [(d.xpos[b].copy(), d.xmat[b].reshape(3, 3)[:, 2].copy()) for b in self.clutter_ids]
        self.drop = dict(plate_min_z=9., pod_min_z=9., clutter_min_z=9.)

    # ---- wrist F/T: white noise + drift after the tare
    def hand_force_world(self):
        f = super().hand_force_world()
        sn = self.scn['sensors']
        if sn['ft_noise_N'] > 0:
            f = f + self.ft_rng.normal(0, sn['ft_noise_N'], 3)
        if sn['ft_drift_N_per_min'] > 0:
            f = f + np.array([.3, .2, -1.]) / math.sqrt(1.13) * sn['ft_drift_N_per_min'] * self.data.time / 60.
        return f

    # ---- per-physics-step monitor: SimRobot._monitor with scenario button parameters and a per-step person flag
    def _monitor(self):
        PL.Runner._monitor(self)
        m, d = self.model, self.data
        dt = m.opt.timestep
        self.update_person()
        for b in dw.BUTTONS:                      # st = [time above threshold, latched, time below 0.3 threshold]
            st = self._btn[b]
            f = float(d.sensor(f'dw/btn_{b}_touch').data[0])
            if f > self.btn_thr:
                st[0] += dt
                st[2] = 0.
                if st[0] >= self.btn_dwell - 1e-9 and not st[1]:
                    st[1] = True
                    self._panel_press(b)
            else:
                st[0] = 0.
                st[2] = st[2] + dt if f < .3 * self.btn_thr else 0.
                if st[2] >= .04:
                    st[1] = False
        self._k = getattr(self, '_k', 0) + 1
        if self._k % 5 == 0:
            p = d.site('tcp').xpos.copy()
            if self.spd['prev'] is not None:
                v = float(np.linalg.norm(p - self.spd['prev'])) / (5 * dt)
                self.spd['max'] = max(self.spd['max'], v)
                if self.person['present']:
                    self.spd['max_person'] = max(self.spd['max_person'], v)
                    if v > .25 + .005:
                        self.spd['over_person_s'] += 5 * dt
            self.spd['prev'] = p
            self.drop['plate_min_z'] = min(self.drop['plate_min_z'], float(d.xpos[self.pid][2]))
            self.drop['pod_min_z'] = min(self.drop['pod_min_z'], float(d.xpos[self.pod_id][2]))
            for b in self.clutter_ids:
                self.drop['clutter_min_z'] = min(self.drop['clutter_min_z'], float(d.xpos[b][2]))
        P = self.person
        if P['present'] and d.time >= P['t_in'] + P['cfg']['dur_s']:
            P.update(present=False, t_out=float(d.time))
            m.geom_rgba[self.person_geoms, 3] = 0.
            self.events.append((round(d.time, 2), 'person left'))

    # ---- seating predicate: 15 deg lean instead of 10 (nominal seated lean is 7.5-9.5 deg; GPT_CONTROL.md's 'seated'
    #      column uses < 15 deg). Used by the Runner's own checks and by the mission score alike; the 10 deg verdict is kept.
    def plate_slot_state(self):
        st = super().plate_slot_state()
        st['in_slot_10deg'] = st['in_slot']
        if (not st['in_slot'] and 10 <= st['plane_tilt_deg'] < 15 and abs(st['bottom_offset_mm']) < 1000 * (dw.TINE_PITCH / 2 - dw.TINE_R)
                and abs(st['centre_err_mm'][1]) < 30 and not st['wrong_tines'] and not st['other_contacts']):
            st['in_slot'] = True
        return st

    # ---- ground-truth outcome (scoring only, never fed to the robot)
    def outcome(self):
        m, d = self.model, self.data
        sc = self.mission_score()
        cl = []
        for i, b in enumerate(self.clutter_ids):
            p0, z0 = self.clutter0[i]
            p, z = d.xpos[b], d.xmat[b].reshape(3, 3)[:, 2]
            cl.append(dict(type=self.scn['clutter'][i]['type'], moved_mm=round(float(np.linalg.norm(p[:2] - p0[:2])) * 1000, 1),
                           tilt_deg=round(math.degrees(math.acos(np.clip(z @ z0, -1, 1))), 1), on_floor=bool(p[2] < S.COUNTER_Z - .15)))
        pc = d.xpos[self.pid]
        plate_where = ('slot' if sc['plate_slot']['in_slot'] else 'floor' if pc[2] < .5 else
                       'hand' if self.body_contacts(self.pid, self.finger_bodies)[1] else
                       'counter' if 'counter_top' in self.touching(self.pid) else
                       'basket' if any(t.startswith('dw/') for t in self.touching(self.pid)) else 'other')
        unexpected = sc['unexpected_contacts']
        worst = max(unexpected, key=lambda u: u['max_force_N'], default=None)
        return dict(score=sc, clutter=cl, plate_where=plate_where, plate_final_xyz=[round(float(v), 3) for v in pc],
                    pod_final_xyz=[round(float(v), 3) for v in d.xpos[self.pod_id]],
                    min_z=dict(plate=round(self.drop['plate_min_z'], 3), pod=round(self.drop['pod_min_z'], 3),
                               clutter=round(self.drop['clutter_min_z'], 3) if self.clutter_ids else None),
                    worst_contact=worst)


# ====================================================================== scripted Runner (as gpt_control/scripted.py run_hardcoded)
def run_scripted(scn, seed, tag, video=None, caption='', record_from=0., base_speed=4., keyframes=None, log=print, record_until=1e9):
    r = AdvRobot(scn, seed=seed, run_tag=tag, video=video, caption=caption, record_from=record_from, base_speed=base_speed,
                 keyframes_dir=keyframes, log=log, record_until=record_until)
    r.start()
    t0 = time.monotonic()
    failed, failed_step = None, None

    def perceive_no_gt():
        """Baseline perceive without the pipeline's ground-truth centre-error gate (a real runner has no GT)."""
        out = r.step_perceive()
        if not out.get('_ok') and .095 < r.est['radius_fit'] < .110:
            out.update(_ok=True, _reason='')
        return out
    fns = dict(perceive_plate=perceive_no_gt, open_door=r.step_open_door, load_pod=r.step_load_pod, pull_basket=r.step_pull_basket,
               pick_plate=r.step_pick_plate, insert_plate=r.step_insert, push_basket=r.step_push_basket, close_door=r.step_close_door,
               press_buttons=r.step_buttons)
    for st in PL.STEPS:
        try:
            r.run_step(st, fns[st])
        except PL.StepFail as e:
            failed, failed_step = str(e), st
            break
        except Exception as e:  # noqa: BLE001  (a crash inside a step is that step's failure)
            failed, failed_step = f'{st}: exception {e!r}', st
            r.steps.append(dict(step=st, ok=False, reason=failed, t_start=round(float(r.data.time), 3)))
            break
    if failed is None:
        try:
            r.transfer('home', np.array(r.opt.park), SR.RD, PL.TOUCH)
            r.hold('end', 1.0)
        except Exception as e:  # noqa: BLE001
            failed, failed_step = f'home: exception {e!r}', 'home'
    else:
        r.hold('stopped', 2.0)                     # let things settle (falling objects) before scoring
    out = r.outcome()
    if keyframes is not None and isinstance(r.rec, StreamRecorder):
        r.rec.still(r.data, Path(keyframes) / 'zz_final.png')
    if isinstance(r.rec, StreamRecorder):
        r.rec.caption = r.rec.caption + f'   ->  RESULT: {"runner stopped at " + failed_step if failed else "runner claims success"}; ' \
                                        f'GT mission {"OK" if out["score"]["success"] else "FAILED"}'
        r.rec.hold_frames(r.data, 2.0)
    steps = [dict(step=x['step'], ok=x['ok'], reason=x.get('reason', ''), t_start=x.get('t_start'), t_end=x.get('t_end'),
                  metrics={k: v for k, v in (x.get('metrics') or {}).items() if not isinstance(v, (dict, list)) or k in ('centre_err_mm',)})
             for x in r.steps]
    perc = [dict(seconds=mm['seconds'], ok=mm['ok'], tokens=mm.get('tokens')) for mm in r.codex_meta]
    res = dict(tag=tag, seed=seed, runner_claimed_success=failed is None, runner_failed=failed, runner_failed_step=failed_step,
               steps=steps, outcome=out, wall_s=round(time.monotonic() - t0, 1), codex_perception=perc)
    r.close()
    return res
