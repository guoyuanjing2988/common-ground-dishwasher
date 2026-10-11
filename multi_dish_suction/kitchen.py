"""Cluttered-counter extension of the suction demo: 8 kitchen items next to the plate stack and the bowl nest, a back-wall
RGB-D camera, the GPT recognition step (codex exec, ChatGPT subscription) and depth back-projection perception of the
two piles.

Item models: kitchen_items.py = ../gpt_sort/items.py copied unchanged (board, knife, pan, flask, mug); soap bottle,
sponge and oil bottle are added here (primitives, EST dimensions).
Ground truth is used only for scoring and as a labelled fallback.
"""
from __future__ import annotations

import json
import math
import re
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

import kitchen_items as IT

ROOT = Path(__file__).resolve().parent
KOUT = ROOT / 'out' / 'kitchen'
COUNTER_Z = IT.COUNTER_Z
BOX, CYL = IT.BOX, IT.CYL
OBJ, VIS = IT.OBJ, IT.VIS

# ---------------------------------------------------------------- wall RGB-D camera (as ../gpt_sort/cfg.py, re-aimed)
CAM_POS = np.array([-0.57, 0.66, 1.58])
CAM_LOOK = np.array([-0.12, 0.70, 0.88])
CAM_W, CAM_H, CAM_FOVY = 1280, 720, 60.0
DEPTH_NOISE = .0015                     # sigma at 1 m (x range), as ../full_pipeline/perception.py


def add_soap(spec, name, xy, yaw, *_):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    b.add_geom(name=f'{name}_body', type=BOX, pos=[0, 0, .085], size=[.032, .021, .085], mass=.45, rgba=[.20, .62, .35, 1], **OBJ)
    b.add_geom(name=f'{name}_cap', type=CYL, pos=[0, 0, .18], size=[.012, .01, 0], mass=.02, rgba=[.95, .95, .95, 1], **OBJ)
    b.add_geom(name=f'{name}_label', type=BOX, pos=[.0323, 0, .08], size=[.0003, .016, .03], rgba=[.97, .97, .9, 1], **VIS)
    return b


def add_sponge(spec, name, xy, yaw, *_):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    b.add_geom(name=f'{name}_foam', type=BOX, pos=[0, 0, .011], size=[.048, .032, .011], mass=.015, rgba=[.98, .85, .25, 1], **OBJ)
    b.add_geom(name=f'{name}_scour', type=BOX, pos=[0, 0, .026], size=[.048, .032, .004], mass=.005, rgba=[.20, .55, .25, 1], **OBJ)
    return b


def add_oil(spec, name, xy, yaw, *_):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], COUNTER_Z + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    glass = [.86, .72, .22, .85]
    b.add_geom(name=f'{name}_body', type=CYL, pos=[0, 0, .095], size=[.031, .095, 0], mass=.75, rgba=glass, **OBJ)
    b.add_geom(name=f'{name}_neck', type=CYL, pos=[0, 0, .215], size=[.013, .025, 0], mass=.03, rgba=glass, **OBJ)
    b.add_geom(name=f'{name}_cap', type=CYL, pos=[0, 0, .247], size=[.015, .008, 0], mass=.01, rgba=[.75, .1, .1, 1], **OBJ)
    return b


BUILDERS = dict(IT.BUILDERS, soap=add_soap, sponge=add_sponge, oil=add_oil)
# EST layout: everything behind / beside the two piles, outside the arm's sweep (piles: plates (-0.11, 0.45), bowls (-0.10, 0.66))
CLUTTER = [
    dict(name='board', type='board', xy=(-.46, .48), yaw=math.radians(90), soil='medium'),
    dict(name='knife', type='knife', xy=(-.13, 1.07), yaw=math.radians(90), soil='medium'),
    dict(name='pan', type='pan', xy=(-.43, .92), yaw=math.radians(60), soil='heavy', greasy=True),
    dict(name='flask', type='flask', xy=(-.24, .90), yaw=0., soil='light'),
    dict(name='mug', type='mug', xy=(-.27, .71), yaw=math.radians(180), soil='medium'),
    dict(name='soap', type='soap', xy=(-.42, 1.17), yaw=math.radians(20), soil='light'),
    dict(name='sponge', type='sponge', xy=(-.10, .88), yaw=math.radians(15), soil='light'),
    dict(name='oil', type='oil', xy=(-.54, 1.10), yaw=0., soil='light'),
]
TRUE = dict(board=("wooden cutting board", False), knife=("chef's knife", False), pan=("cast-iron frying pan", False),
            flask=("vacuum flask", False), mug=("mug", True), soap=("dish-soap bottle", False), sponge=("sponge", False),
            oil=("oil bottle", False))


def add_clutter(spec, seed=3):
    rng = np.random.default_rng(seed)
    for it in CLUTTER:
        BUILDERS[it['type']](spec, it['name'], it['xy'], it['yaw'], it.get('soil', 'light'), it.get('greasy', False), rng)


def add_wallcam(spec):
    fwd = CAM_LOOK - CAM_POS
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.column_stack([right, up, -fwd]).ravel())
    spec.worldbody.add_camera(name='wallcam', pos=list(CAM_POS), quat=list(q), fovy=CAM_FOVY)


# ---------------------------------------------------------------- RGB-D capture + back-projection (../full_pipeline/perception.py)
class Camera:
    def __init__(self, model):
        self.r = mujoco.Renderer(model, CAM_H, CAM_W)
        self.cid = model.camera('wallcam').id
        self.f = (CAM_H / 2) / math.tan(math.radians(CAM_FOVY) / 2)
        self.cx, self.cy = (CAM_W - 1) / 2, (CAM_H - 1) / 2
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0

    def capture(self, data, rng):
        self.r.update_scene(data, camera='wallcam', scene_option=self.opt)
        rgb = self.r.render().copy()
        self.r.enable_depth_rendering()
        self.r.update_scene(data, camera='wallcam', scene_option=self.opt)
        depth = self.r.render().copy()
        self.r.disable_depth_rendering()
        depth = depth + rng.normal(0, DEPTH_NOISE, depth.shape) * depth
        return rgb, depth

    def pose(self, data):
        return data.cam_xpos[self.cid].copy(), data.cam_xmat[self.cid].reshape(3, 3).copy()

    def backproject(self, p, R, us, vs, ds):
        pc = np.stack([(us - self.cx) / self.f * ds, -(vs - self.cy) / self.f * ds, -ds], -1)
        return pc @ R.T + p

    def project(self, p, R, X):
        c = (np.asarray(X) - p) @ R
        return np.array([self.cx + self.f * c[..., 0] / -c[..., 2], self.cy - self.f * c[..., 1] / -c[..., 2]]).T

    def close(self):
        self.r.close()


def ransac_plane(X, thr=.003, iters=300, rng=None, prefer_normal=None):
    rng = rng or np.random.default_rng(0)
    best, bn = None, 0
    for _ in range(iters):
        a, b, c = X[rng.choice(len(X), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if prefer_normal is not None and abs(n @ prefer_normal) < .9:
            continue
        inl = np.abs((X - a) @ n) < thr
        if inl.sum() > bn:
            best, bn = inl, inl.sum()
    P = X[best]
    c = P.mean(0)
    n = np.linalg.svd(P - c, full_matrices=False)[2][2]
    return c, (n if n[2] > 0 else -n)


def circle_fit(xy):
    A = np.c_[2 * xy, np.ones(len(xy))]
    b = (xy ** 2).sum(1)
    (cx, cy, k), *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([cx, cy]), math.sqrt(max(k + cx * cx + cy * cy, 0.))


def perceive_pile(cam, cp, cR, depth, box, kind):
    """Metric pose of the TOP item of a pile inside a GPT box (pixels). Counter plane by RANSAC on a ring around the box;
    pile points = 3..90 mm above the plane; outline = farthest point per 5-deg bin around the box centre -> circle fit
    (plate rim R 101.5 / bowl rim R 57.5). Height:
      plate stack: top disc surface = 85th percentile height of points within 45 mm of the centre;
      bowl nest:   inside bottom = 15th percentile height of points within 15 mm of the centre (seen through the opening).
    Returns the suction target (cup point) in world coordinates + diagnostics."""
    x0, y0, x1, y1 = box
    pad = int(.35 * max(x1 - x0, y1 - y0)) + 10
    u0, v0, u1, v1 = max(0, int(x0 - pad)), max(0, int(y0 - pad)), min(CAM_W, int(x1 + pad)), min(CAM_H, int(y1 + pad))
    vs, us = np.mgrid[v0:v1, u0:u1]
    us, vs = us.ravel().astype(float), vs.ravel().astype(float)
    d = depth[vs.astype(int), us.astype(int)]
    ok = d < 4
    us, vs, d = us[ok], vs[ok], d[ok]
    X = cam.backproject(cp, cR, us, vs, d)
    inbox = (us >= x0 - 4) & (us <= x1 + 4) & (vs >= y0 - 4) & (vs <= y1 + 4)
    c, n = ransac_plane(X[~inbox], prefer_normal=np.array([0, 0, 1.]))
    h = (X - c) @ n
    sel = (h > .003) & (h < .09) & inbox
    P = X[sel]
    hp = h[sel]
    ctr = P[:, :2].mean(0)
    for _ in range(3):
        rel = P[:, :2] - ctr
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        rad = np.linalg.norm(rel, axis=1)
        bins = np.floor((ang + math.pi) / math.radians(5)).astype(int)
        pts = np.array([P[bins == b_][np.argmax(rad[bins == b_]), :2] for b_ in np.unique(bins)])
        ctr, r = circle_fit(pts)
        res = np.abs(np.linalg.norm(pts - ctr, axis=1) - r)
        keep = res < max(.004, 2.5 * np.median(res))
        ctr, r = circle_fit(pts[keep])
    rr = np.linalg.norm(P[:, :2] - ctr, axis=1)
    if kind == 'plates':
        inner = hp[rr < .045]
        z_top = float(np.percentile(inner, 85)) if len(inner) > 20 else float(hp.max())
        z_src = 'depth: disc top'
    else:
        inner = hp[rr < .015]
        if len(inner) > 15:
            z_top, z_src = float(np.percentile(inner, 15)), 'depth: inside bottom seen through the opening'
        else:
            z_top, z_src = float(np.percentile(hp[rr > r - .01], 90)) - .056, 'depth rim top - catalogue depth 56 mm'
    top = np.array([ctr[0], ctr[1], c[2] + z_top])
    return dict(centre_xy=ctr.tolist(), rim_radius=float(r), top=top.tolist(), z_source=z_src, n_points=int(sel.sum()),
                counter_z=float(c[2]))


# ---------------------------------------------------------------- GPT recognition (codex exec, ChatGPT subscription)
TARGETS = ['plate_rack', 'bowl_area', 'leave_hand_wash', 'leave_not_for_dishwasher']
SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["items", "summary"],
    "properties": {
        "items": {"type": "array", "items": {
            "type": "object", "additionalProperties": False,
            "required": ["id", "box", "label_en", "label_zh", "count", "dishwasher_safe", "reason_en", "reason_zh", "target"],
            "properties": {
                "id": {"type": "string"},
                "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "description": "[x_min,y_min,x_max,y_max] tight pixel box in the image"},
                "label_en": {"type": "string"}, "label_zh": {"type": "string"},
                "count": {"type": "integer", "description": "number of pieces (e.g. a stack of 3 plates -> 3)"},
                "dishwasher_safe": {"type": "boolean"},
                "reason_en": {"type": "string", "description": "short reason, <= 12 words"},
                "reason_zh": {"type": "string", "description": "short reason in Chinese, <= 20 characters"},
                "target": {"type": "string", "enum": TARGETS}}}},
        "summary": {"type": "string"}}}

PROMPT = """You are the perception/planning module of a home robot arm standing next to a small countertop dishwasher.
The image ({w}x{h} px, x right, y down) is from a camera on the back wall of the kitchen looking over the countertop
toward the robot (the counter's front edge is toward the TOP of the image; the dishwasher is at the right side).
The dishwasher's lower basket has a PLATE RACK (plates stand upright between tines) and a BOWL AREA (bowls/cups tilted).

List EVERY loose object on the countertop (ignore the robot, the dishwasher, walls, floor). A stack of identical plates
or a nest of identical bowls is ONE entry with its count. For each give: a tight pixel box, an English and a Chinese
label, whether it is dishwasher-safe and why (short), and a target: plate_rack / bowl_area for dishwasher-safe
tableware, leave_hand_wash for items that must be washed by hand, leave_not_for_dishwasher for things that are not
dishes at all (cleaning supplies, food containers). Answer only with the JSON."""


def gpt_recognise(png, tag='kitchen_plan', timeout=600):
    work = KOUT / 'codex_work'
    work.mkdir(parents=True, exist_ok=True)
    sp, out = work / f'{tag}_schema.json', work / f'{tag}_out.json'
    sp.write_text(json.dumps(SCHEMA))
    out.unlink(missing_ok=True)
    cmd = ['codex', 'exec', '--skip-git-repo-check', '--ephemeral', '--sandbox', 'read-only', '--output-schema', str(sp),
           '-o', str(out), f'--image={png}', PROMPT.format(w=CAM_W, h=CAM_H)]
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=str(work), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                           timeout=timeout, check=False)
        err = p.stderr.decode(errors='replace')
    except subprocess.TimeoutExpired:
        err = 'timeout'
    sec = round(time.time() - t0, 1)
    meta = dict(seconds=sec, ok=out.exists())
    m = re.search(r'tokens used\s*\n\s*([\d,]+)', err)
    if m:
        meta['tokens'] = int(m.group(1).replace(',', ''))
    if not out.exists():
        meta['error'] = err[-500:]
        return None, meta
    return json.loads(out.read_text()), meta
