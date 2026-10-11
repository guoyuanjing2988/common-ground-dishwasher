"""Kitchen clutter, the overhead counter camera, GPT (codex exec) recognition and depth back-projection poses.

Clutter: gpt_sort/items.py builders (board, knife, cast-iron pan, flask, mug) + simple primitive soap bottle, sponge and
oil bottle. All clutter stands LEFT of the dishwasher (y < -0.27), outside every arm path of the loading sequence.
GPT never sees ground truth: one image + the prompt. Depth (with noise) gives metric poses for plates / bowls.
"""
from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np
try:
    import scipy.ndimage as ndi
except ImportError:                      # fallback: simple 4-connected labelling (BFS) if scipy is unavailable
    ndi = None
from PIL import Image, ImageDraw, ImageFont

import items as IT
import scene as S
from util import renderer, scene_option

ROOT = Path(__file__).resolve().parent
CAM_W, CAM_H, CAM_FOVY = 1280, 720, 56.
CAM_POS, CAM_LOOK = np.array([0.42, 0.04, 2.20]), np.array([-0.26, 0.04, 0.90])
DEPTH_NOISE = .0015                      # sigma at 1 m, grows with range (as perception.py)
BOX, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER


# ------------------------------------------------------------------ clutter
def _simple(spec, name, xy, parts, yaw=0.):
    b = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], S.COUNTER_Z + .0003], quat=IT.q_yaw(yaw))
    b.add_freejoint(name=f'{name}_free')
    for k, (typ, pos, size, mass, rgba) in enumerate(parts):
        if mass:
            b.add_geom(name=f'{name}_g{k}', type=typ, pos=pos, size=size, mass=mass, rgba=rgba, **IT.OBJ)
        else:
            b.add_geom(name=f'{name}_g{k}', type=typ, pos=pos, size=size, rgba=rgba, **IT.VIS)
    return b


def add_clutter(spec):
    rng = np.random.default_rng(5)
    IT.add_board(spec, 'board', (-0.28, -0.62), 0., 'medium', False, rng)
    IT.add_knife(spec, 'knife', (-0.065, -0.63), math.radians(90), 'medium', False, rng)
    IT.add_pan(spec, 'pan', (-0.17, -0.99), math.pi, 'medium', True, rng)
    IT.add_flask(spec, 'flask', (-0.47, -0.38), 0., 'light', False, rng)
    IT.add_mug(spec, 'mug', (-0.46, -0.84), math.radians(200), 'medium', False, rng)
    _simple(spec, 'soap', (-0.22, -0.34), [            # dish-soap bottle: green body, white cap
        (BOX, [0, 0, .085], [.028, .018, .085], .45, [.20, .62, .32, 1]),
        (CYL, [0, 0, .182], [.011, .012, 0], .02, [.95, .95, .95, 1]),
        (BOX, [.0285, 0, .08], [.0005, .014, .03], 0, [.95, .95, .80, 1])])
    _simple(spec, 'sponge', (-0.07, -0.33), [          # yellow sponge with green scrub layer
        (BOX, [0, 0, .012], [.05, .035, .012], .02, [.96, .83, .25, 1]),
        (BOX, [0, 0, .027], [.05, .035, .003], .005, [.15, .45, .20, 1])], yaw=math.radians(15))
    _simple(spec, 'oil', (-0.50, -0.56), [             # cooking-oil bottle: amber body, neck, red cap
        (CYL, [0, 0, .11], [.036, .11, 0], .9, [.86, .66, .18, 1]),
        (CYL, [0, 0, .24], [.014, .02, 0], .03, [.86, .70, .30, 1]),
        (CYL, [0, 0, .27], [.017, .012, 0], .02, [.75, .12, .10, 1]),
        (CYL, [0, 0, .10], [.0365, .035, 0], 0, [.95, .92, .85, 1])])


CLUTTER = ['board', 'knife', 'pan', 'flask', 'mug', 'soap', 'sponge', 'oil']


def add_camera(spec):
    fwd = CAM_LOOK - CAM_POS
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    spec.worldbody.add_camera(name='countercam', pos=list(CAM_POS), quat=list(S.quat(np.column_stack([right, up, -fwd]))),
                              fovy=CAM_FOVY)


# ------------------------------------------------------------------ camera
class Cam:
    def __init__(self, model, name='countercam', fovy=None):
        self.r = renderer(model, CAM_H, CAM_W)
        self.name = name
        self.cid = model.camera(name).id
        self.f = (CAM_H / 2) / math.tan(math.radians(fovy or CAM_FOVY) / 2)
        self.cx, self.cy = (CAM_W - 1) / 2, (CAM_H - 1) / 2

    def capture(self, data, rng):
        opt = scene_option(hide_sites=True)
        self.r.update_scene(data, camera=self.name, scene_option=opt)
        rgb = self.r.render().copy()
        self.r.enable_depth_rendering()
        self.r.update_scene(data, camera=self.name, scene_option=opt)
        depth = self.r.render().copy()
        self.r.disable_depth_rendering()
        depth = depth + rng.normal(0, DEPTH_NOISE, depth.shape) * depth
        return rgb, depth

    def pose(self, data):
        return data.cam_xpos[self.cid].copy(), data.cam_xmat[self.cid].reshape(3, 3).copy()

    def backproject(self, data, us, vs, ds):
        p, R = self.pose(data)
        pc = np.stack([(us - self.cx) / self.f * ds, -(vs - self.cy) / self.f * ds, -ds], -1)
        return pc @ R.T + p

    def project(self, data, X):
        p, R = self.pose(data)
        c = (np.asarray(X) - p) @ R
        return np.array([self.cx + self.f * c[..., 0] / -c[..., 2], self.cy - self.f * c[..., 1] / -c[..., 2]]).T

    def close(self):
        self.r.close()


# ------------------------------------------------------------------ depth -> poses
def height_map(cam, data, depth):
    """World points for every pixel + height above the counter plane (known counter height z = 0.90)."""
    vs, us = np.mgrid[0:CAM_H, 0:CAM_W]
    X = cam.backproject(data, us.ravel().astype(float), vs.ravel().astype(float), depth.ravel()).reshape(CAM_H, CAM_W, 3)
    return X, X[..., 2] - S.COUNTER_Z


def _label(mask):
    from collections import deque
    lab = np.zeros(mask.shape, int)
    n = 0
    H_, W_ = mask.shape
    for v0, u0 in zip(*np.nonzero(mask)):
        if lab[v0, u0]:
            continue
        n += 1
        q = deque([(v0, u0)])
        lab[v0, u0] = n
        while q:
            v, u = q.popleft()
            for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = v + dv, u + du
                if 0 <= a < H_ and 0 <= b < W_ and mask[a, b] and not lab[a, b]:
                    lab[a, b] = n
                    q.append((a, b))
    return lab, n


def component(mask, box=None, seed_uv=None):
    """Connected blob of `mask` overlapping the box most (or containing / nearest to seed_uv)."""
    lab, n = ndi.label(mask) if ndi is not None else _label(mask)
    if n == 0:
        return None
    if box is not None:
        x0, y0, x1, y1 = [int(round(v)) for v in box]
        sub = lab[max(0, y0):y1, max(0, x0):x1]
        ids, cnt = np.unique(sub[sub > 0], return_counts=True)
        if len(ids) == 0:
            return None
        return lab == ids[np.argmax(cnt)]
    u, v = seed_uv
    vv, uu = np.nonzero(lab)
    i = np.argmin((uu - u) ** 2 + (vv - v) ** 2)
    return lab == lab[vv[i], uu[i]]


def estimate_plate(X, h, blob):
    """Flat plate: outline of the blob (farthest point per 5 deg bin) -> circle fit (perception.py estimate_flat)."""
    P = X[blob & (h > .003) & (h < .04)]
    Q = P[:, :2]
    centre = Q.mean(0)
    for _ in range(3):
        rel = Q - centre
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        rad = np.linalg.norm(rel, axis=1)
        bins = np.floor((ang + math.pi) / math.radians(5)).astype(int)
        pts = np.array([Q[bins == b][np.argmax(rad[bins == b])] for b in np.unique(bins)])
        A = np.c_[2 * pts, np.ones(len(pts))]
        (cx, cy, k), *_ = np.linalg.lstsq(A, (pts ** 2).sum(1), rcond=None)
        r = math.sqrt(max(k + cx * cx + cy * cy, 0.))
        res = np.abs(np.linalg.norm(pts - [cx, cy], axis=1) - r)
        keep = res < max(.004, 2.5 * np.median(res))
        A = A[keep]
        (cx, cy, k), *_ = np.linalg.lstsq(A, (pts[keep] ** 2).sum(1), rcond=None)
        r = math.sqrt(max(k + cx * cx + cy * cy, 0.))
        centre = np.array([cx, cy])
    return dict(origin=np.r_[centre, S.COUNTER_Z + .0003], radius_fit=float(r), n_points=int(len(P)))


def estimate_bowl(X, h, blob):
    """Inverted bowl: the top (foot ring + bottom disc) is the highest 8 mm of the blob; its centroid = foot-ring centre."""
    hb = np.where(blob, h, -1)
    top = hb.max()
    P = X[blob & (h > top - .008)]
    c = P[:, :2].mean(0)
    # height: the bowl rests on the counter, so z comes from the counter plane + the known bowl model (foot-ring centre
    # 64.3 mm up); the noisy depth maximum (top_height_mm) is only a sanity check (it reads ~4 mm high from noise)
    return dict(foot=np.r_[c, S.COUNTER_Z + .0643], top_height_mm=float(top * 1000), n_points=int(len(P)))


# ------------------------------------------------------------------ GPT recognition (codex exec, ChatGPT subscription)
SLOTS = ['plate slot 4', 'plate slot 5', 'plate slot 6', 'bowl spot 1', 'bowl spot 2', 'leave / hand-wash', 'not loaded this cycle']
SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["items"],
    "properties": {"items": {"type": "array", "items": {
        "type": "object", "additionalProperties": False,
        "required": ["box", "label_en", "label_zh", "category", "dishwasher_safe", "reason_en", "reason_zh", "target"],
        "properties": {
            "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                    "description": "[x_min,y_min,x_max,y_max] tight pixel box"},
            "label_en": {"type": "string"}, "label_zh": {"type": "string"},
            "category": {"type": "string", "enum": ['plate', 'bowl', 'mug', 'cutting_board', 'knife', 'pan', 'flask',
                                                    'soap_bottle', 'sponge', 'oil_bottle', 'other']},
            "dishwasher_safe": {"type": "boolean"},
            "reason_en": {"type": "string", "description": "max 8 words"},
            "reason_zh": {"type": "string", "description": "max 12 characters"},
            "target": {"type": "string", "enum": SLOTS}}}}}}

PROMPT = """You are the vision + planning module of a dish-loading robot. The image ({w}x{h} px) is from a camera above a
kitchen counter. A small countertop dishwasher (white box, door open, lower wire basket pulled out) is on the counter;
a robot arm is in front of it.
List EVERY loose item lying on the counter (do not list the dishwasher, the robot, the basket or the cart).
For each item give a tight pixel box, an English and a Chinese label, whether it may go in a dishwasher and a short reason
(English <= 8 words, Chinese <= 12 characters), and a target:
- The lower basket has three free plate slots ("plate slot 4", "plate slot 5", "plate slot 6"; one dinner plate each)
  and two bowl spots ("bowl spot 1", "bowl spot 2"; one bowl each, placed upside down and tilted).
- Assign each dinner plate a different plate slot and each bowl a different bowl spot.
- Items that must not go in a dishwasher: "leave / hand-wash".
- Dishwasher-safe items for which this load has no place (e.g. a mug, the upper tray is too low): "not loaded this cycle".
Return JSON only."""


def ask_gpt(png, outdir, tag='recog'):
    outdir.mkdir(parents=True, exist_ok=True)
    sp, out = outdir / f'{tag}_schema.json', outdir / f'{tag}_gpt.json'
    sp.write_text(json.dumps(SCHEMA))
    out.unlink(missing_ok=True)
    t = time.time()
    p = subprocess.run(['codex', 'exec', '--skip-git-repo-check', '--sandbox', 'read-only', '--output-schema', str(sp),
                        '-o', str(out), f'--image={png}', PROMPT.format(w=CAM_W, h=CAM_H)],
                       cwd=str(outdir), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                       timeout=420, check=False)
    sec = round(time.time() - t, 1)
    if not out.exists():
        return None, dict(seconds=sec, error=f'codex exit {p.returncode}: {p.stderr.decode(errors="replace")[-300:]}')
    return json.loads(out.read_text()), dict(seconds=sec)


# ------------------------------------------------------------------ card drawing
def _font(sz):
    for fp in ('/System/Library/Fonts/Hiragino Sans GB.ttc', '/System/Library/Fonts/STHeiti Medium.ttc',
               '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'):
        try:
            return ImageFont.truetype(fp, sz)
        except OSError:
            continue
    return ImageFont.load_default()


def _is_load(it):
    return it['dishwasher_safe'] and not it['target'].startswith(('leave', 'not'))


def draw_card(rgb, items, title):
    img = Image.fromarray(rgb).convert('RGB')
    d = ImageDraw.Draw(img, 'RGBA')
    f, fs, ft = _font(16), _font(15), _font(15)
    cols = []
    for k, it in enumerate(items):
        x0, y0, x1, y1 = it['box']
        col = (60, 220, 100) if _is_load(it) else ((245, 190, 60) if it['dishwasher_safe'] else (240, 70, 60))
        cols.append(col)
        d.rectangle((x0, y0, x1, y1), outline=col + (255,), width=3)
        d.rectangle((x0, y0, x0 + 22, y0 + 20), fill=col + (235,))
        d.text((x0 + 4, y0 + 1), str(k + 1), fill=(0, 0, 0), font=ft)
    load = [k for k, it in enumerate(items) if _is_load(it)]
    rest = [k for k, it in enumerate(items) if not _is_load(it)]
    # top-right panel: items to load
    px, py, pw = 735, 48, 540
    d.rectangle((px, py, px + pw, py + 30 + 22 * len(load)), fill=(14, 18, 24, 225))
    d.text((px + 10, py + 5), 'Dishwasher-safe -> load  可入洗碗机 -> 装入', fill=(60, 220, 100), font=f)
    for n, k in enumerate(load):
        it = items[k]
        d.text((px + 10, py + 28 + 22 * n), f"{k + 1}  {it['label_en']} / {it['label_zh']}  -> {it['target']}", fill=(235, 235, 235), font=fs)
    # bottom-left panel: items left on the counter
    px, pw = 6, 482
    ph = 30 + 22 * len(rest)
    py = CAM_H - ph - 6
    d.rectangle((px, py, px + pw, py + ph), fill=(14, 18, 24, 225))
    d.text((px + 10, py + 5), 'Not loaded (hand-wash / not this load)  不装入', fill=(240, 90, 70), font=f)
    for n, k in enumerate(rest):
        it = items[k]
        t = f"{k + 1}  {it['label_en']} / {it['label_zh']}: {it['reason_en']}"
        if d.textlength(t, font=fs) > pw - 20:
            while d.textlength(t + '…', font=fs) > pw - 20:
                t = t[:-1]
            t += '…'
        d.text((px + 10, py + 28 + 22 * n), t, fill=cols[k], font=fs)
    d.rectangle((0, 0, CAM_W, 40), fill=(14, 18, 24, 225))
    d.text((12, 8), title, fill='white', font=_font(20))
    return np.asarray(img)
