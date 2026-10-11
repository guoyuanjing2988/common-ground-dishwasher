"""Plate perception: wall RGB-D camera -> Codex (via `codex exec`, ChatGPT subscription) for the
semantic 2-D detection -> depth geometry for the metric pose. Adapted from ../poc_perception.

flat mode (plate lying on the counter): counter plane by RANSAC on the depth ring around the
  Codex box, plate = points > 3 mm above that plane, outline = farthest point per angular bin
  around the Codex centre, circle fit on the FULL outline (outlier-trimmed). Pose = circle centre on
  the counter plane, normal = plane normal, yaw = 0 (the plate is rotationally symmetric).
rack mode (plate upright in a drying rack): exactly ../poc_perception/perceive.py (RANSAC plate
  plane seeded at the Codex centre, circle fit on the unoccluded upper arc).
Ground truth (true pose, segmentation) is used only for scoring, never for control.
"""
from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

import scene as S
from util import renderer, scene_option

ROOT = Path(__file__).resolve().parent
W, H = S.CAM_W, S.CAM_H
DEPTH_NOISE = .0015                      # sigma at 1 m range (grows ~linearly with range), as in the perception PoC
SCHEMA = ROOT / 'codex' / 'schema_plate.json'
PROMPTS = {
    'flat': ("The attached image is {w}x{h} pixels (x right, y down, origin top-left). It was taken by a camera mounted "
             "above a kitchen counter. Locate the round dinner plate lying flat on the counter (ignore the small dish, the "
             "robot and the dishwasher). Return pixel coordinates in this image: the plate's tight bounding box and its "
             "centre. Answer only with the JSON."),
    'rack': ("The attached image is {w}x{h} pixels (x right, y down, origin top-left). It shows a kitchen counter with a "
             "robot arm and a dinner plate standing upright in a dish drying rack. Locate the dinner plate. Return pixel "
             "coordinates in this image: its tight bounding box and its centre. Answer only with the JSON."),
}


def write_schema():
    SCHEMA.parent.mkdir(exist_ok=True)
    SCHEMA.write_text(json.dumps({
        "type": "object", "additionalProperties": False, "required": ["found", "bbox", "center"],
        "properties": {"found": {"type": "boolean"},
                       "bbox": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                                "description": "[x_min,y_min,x_max,y_max] pixels of the dinner plate"},
                       "center": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2,
                                  "description": "[x,y] pixel of the plate centre"}}}))


class Camera:
    def __init__(self, model):
        self.model = model
        self.r = renderer(model, H, W)
        self.cid = model.camera('wallcam').id
        self.f = (H / 2) / math.tan(math.radians(S.CAM_FOVY) / 2)
        self.cx, self.cy = (W - 1) / 2, (H - 1) / 2

    def capture(self, data, rng):
        opt = scene_option(hide_sites=True)          # marker sites hidden: no hints for the VLM
        self.r.update_scene(data, camera='wallcam', scene_option=opt)
        rgb = self.r.render().copy()
        self.r.enable_depth_rendering()
        self.r.update_scene(data, camera='wallcam', scene_option=opt)
        depth = self.r.render().copy()
        self.r.disable_depth_rendering()
        self.r.enable_segmentation_rendering()
        self.r.update_scene(data, camera='wallcam', scene_option=opt)
        seg = self.r.render().copy()                 # ground truth, scoring only
        self.r.disable_segmentation_rendering()
        depth = depth + rng.normal(0, DEPTH_NOISE, depth.shape) * depth
        return rgb, depth, seg

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


def ask_codex(rgb, tag, mode, outdir):
    outdir.mkdir(parents=True, exist_ok=True)
    img, out = outdir / f'{tag}.png', outdir / f'{tag}.json'
    Image.fromarray(rgb).save(img)
    out.unlink(missing_ok=True)
    if not SCHEMA.exists():
        write_schema()
    t = time.time()
    p = subprocess.run(['codex', 'exec', '--skip-git-repo-check', '--sandbox', 'read-only', '--output-schema', str(SCHEMA),
                        '-o', str(out), f'--image={img}', PROMPTS[mode].format(w=W, h=H)],
                       stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=300, check=False)
    sec = round(time.time() - t, 1)
    if not out.exists():
        return dict(found=False, error=f'codex exit {p.returncode}: {p.stderr.decode()[-300:]}', seconds=sec)
    res = json.loads(out.read_text())
    res['seconds'] = sec
    return res


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
    return c, n, best


def circle_fit(xy):
    A = np.c_[2 * xy, np.ones(len(xy))]
    b = (xy ** 2).sum(1)
    (cx, cy, k), *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([cx, cy]), math.sqrt(max(k + cx * cx + cy * cy, 0.))


def estimate_flat(cam, data, depth, vlm):
    """Metric pose of a plate lying flat (see module doc)."""
    x0, y0, x1, y1 = vlm['bbox']
    bw, bh = x1 - x0, y1 - y0
    pad = int(.35 * max(bw, bh)) + 10
    u0, v0, u1, v1 = max(0, int(x0 - pad)), max(0, int(y0 - pad)), min(W, int(x1 + pad)), min(H, int(y1 + pad))
    vs, us = np.mgrid[v0:v1, u0:u1]
    us, vs = us.ravel().astype(float), vs.ravel().astype(float)
    d = depth[vs.astype(int), us.astype(int)]
    ok = d < 4
    us, vs, d = us[ok], vs[ok], d[ok]
    X = cam.backproject(data, us, vs, d)
    inbox = (us >= x0 - 4) & (us <= x1 + 4) & (vs >= y0 - 4) & (vs <= y1 + 4)
    ring = ~inbox
    c, n, _ = ransac_plane(X[ring], prefer_normal=np.array([0, 0, 1.]))
    if n[2] < 0:
        n = -n
    h = (X - c) @ n
    plate = (h > .003) & (h < .04) & ((us >= x0 - pad / 2) & (us <= x1 + pad / 2) & (vs >= y0 - pad / 2) & (vs <= y1 + pad / 2))
    e1 = np.cross(n, [0, 1., 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    Q = np.c_[(X[plate] - c) @ e1, (X[plate] - c) @ e2]
    # seed centre: Codex centre pixel back-projected onto the counter plane + ~15 mm (disc/rim surface)
    cu, cv = vlm['center']
    ray_p, Rcam = cam.pose(data)
    ray = Rcam @ np.array([(cu - cam.cx) / cam.f, -(cv - cam.cy) / cam.f, -1.])
    tt = ((c + .012 * n) - ray_p) @ n / (ray @ n)
    seed = ray_p + tt * ray
    s2 = np.array([(seed - c) @ e1, (seed - c) @ e2])
    out = None
    centre = s2
    for _ in range(3):                                  # outline = farthest point per 5 deg bin, refit around new centre
        rel = Q - centre
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        rad = np.linalg.norm(rel, axis=1)
        bins = np.floor((ang + math.pi) / math.radians(5)).astype(int)
        pts = []
        for b in np.unique(bins):
            m = bins == b
            i = np.argmax(rad[m])
            pts.append(Q[m][i])
        pts = np.array(pts)
        ctr, r = circle_fit(pts)
        res = np.abs(np.linalg.norm(pts - ctr, axis=1) - r)
        keep = res < max(.004, 2.5 * np.median(res))
        ctr, r = circle_fit(pts[keep])
        centre, out = ctr, dict(n_outline=int(len(pts)), n_kept=int(keep.sum()))
    origin = c + centre[0] * e1 + centre[1] * e2 + .0003 * n      # plate foot plane rests on the counter
    Rp = np.column_stack([e1, e2, n])
    # yaw is unobservable for a round plate: use the frame whose x axis is closest to world x
    x_ax = np.array([1., 0, 0]) - (np.array([1., 0, 0]) @ n) * n
    x_ax /= np.linalg.norm(x_ax)
    Rp = np.column_stack([x_ax, np.cross(n, x_ax), n])
    return dict(origin=origin, P=Rp, radius_fit=float(r), plane_normal=n.tolist(), counter_plane_pt=c.tolist(),
                n_plate_points=int(plate.sum()), max_height_mm=float(h[plate].max() * 1000) if plate.any() else 0.,
                **out)


def estimate_rack(cam, data, depth, vlm):
    """../poc_perception/perceive.py estimate() (upright plate in a drying rack), unchanged in substance."""
    x0, y0, x1, y1 = vlm['bbox']
    pad = 12
    u0, v0 = max(0, int(x0 - pad)), max(0, int(y0 - pad))
    u1, v1 = min(W, int(x1 + pad)), min(H, int(y1 + pad))
    vs, us = np.mgrid[v0:v1, u0:u1]
    us, vs = us.ravel().astype(float), vs.ravel().astype(float)
    d = depth[vs.astype(int), us.astype(int)]
    ok = d < 3
    X = cam.backproject(data, us[ok], vs[ok], d[ok])
    cu, cv = vlm['center']
    near = (np.abs(us[ok] - cu) < (x1 - x0) * .3) & (np.abs(vs[ok] - cv) < (y1 - y0) * .3)
    c, n, _ = ransac_plane(X[near], thr=.004, iters=400)
    cam_pos = cam.pose(data)[0]
    if n @ (cam_pos - c) < 0:
        n = -n
    keep = np.abs((X - c) @ n) < .015
    up = np.array([0, 0, 1.]) - (np.array([0, 0, 1.]) @ n) * n
    up /= np.linalg.norm(up)
    e1 = np.cross(up, n)
    Q = np.c_[(X[keep] - c) @ e1, (X[keep] - c) @ up]
    bins = np.round(Q[:, 0] / .004).astype(int)
    top = {}
    for b_, q in zip(bins, Q):
        if b_ not in top or q[1] > top[b_][1]:
            top[b_] = q
    arc = np.array(list(top.values()))
    ctr2, r = circle_fit(arc[arc[:, 1] > np.median(arc[:, 1])])
    centre_surface = c + ctr2[0] * e1 + ctr2[1] * up
    origin = centre_surface - n * .010
    Pm = np.column_stack([np.cross(up, n), up, n])
    return dict(origin=origin, P=Pm, radius_fit=float(r), n_points=int(keep.sum()), arc_points=len(arc))


def overlay(rgb, vlm, cam, data, est):
    """Codex box + centre (yellow) and the fitted metric circle re-projected (green), for the video / key frames."""
    im = Image.fromarray(rgb)
    dr = ImageDraw.Draw(im)
    if vlm.get('found'):
        dr.rectangle(vlm['bbox'], outline=(255, 210, 0), width=3)
        cu, cv = vlm['center']
        dr.ellipse((cu - 5, cv - 5, cu + 5, cv + 5), outline=(255, 210, 0), width=2)
    if est is not None:
        n = est['P'][:, 2]
        a = est['P'][:, 0]
        b = est['P'][:, 1]
        ctr = est['origin'] + n * (S.MID_Z if est.get('mode', 'flat') == 'flat' else .010)
        th = np.linspace(0, 2 * math.pi, 73)
        pts = ctr + est['radius_fit'] * (np.outer(np.cos(th), a) + np.outer(np.sin(th), b))
        px = cam.project(data, pts)
        dr.line([tuple(p) for p in px], fill=(40, 230, 90), width=2)
        c2 = cam.project(data, ctr)
        dr.ellipse((c2[0] - 4, c2[1] - 4, c2[0] + 4, c2[1] + 4), fill=(40, 230, 90))
    return np.asarray(im)
