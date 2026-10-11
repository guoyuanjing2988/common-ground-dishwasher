"""(1) Depth back-projection pose fit for the plate stack (top plate) and the bowls, seeded ONLY by GPT's boxes;
(2) scoring of GPT's recognition against ground truth. No simulation here (numpy on the saved RGB-D frame).

Fit (adapted from ../full_pipeline/perception.py estimate_flat, whose ransac_plane / circle_fit are reused):
counter plane by RANSAC on the depth ring around the GPT box -> points above the plane inside the box -> height band
(plates: top-plate band of the stack; bowls: upper half of the wall, which excludes flat neighbours like the board) ->
top-view outline = farthest point per 5 deg bin -> RANSAC circle (3 mm inliers, radius 4-13 cm) -> least-squares refit.
GT (settled body poses) is used only for the error numbers."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

import numpy as np

ROOT = Path(__file__).resolve().parent
import os
V = os.environ.get('RC_VARIANT', '')   # '' = original scene; '_5dish' = same scene without the mug

sys.path.insert(0, str(_ROOT / 'full_pipeline'))
from perception import ransac_plane, circle_fit  # noqa: E402

W, H = 1280, 720
CAT_OF = dict(plate='plate', bowl='bowl', mug='mug', board='cutting_board', knife='knife', pan='pan', flask='flask',
              soap='soap_bottle', sponge='sponge', kettle='kettle')
TRUE_R = dict(plate=.1015, bowl=.065)


class Cam:
    def __init__(self, z):
        self.p, self.R, self.f = z['cam_pos'], z['cam_mat'], float(z['f'])
        self.cx, self.cy = (W - 1) / 2, (H - 1) / 2

    def back(self, us, vs, ds):
        pc = np.stack([(us - self.cx) / self.f * ds, -(vs - self.cy) / self.f * ds, -ds], -1)
        return pc @ self.R.T + self.p

    def proj(self, X):
        c = (np.asarray(X) - self.p) @ self.R
        return np.array([self.cx + self.f * c[..., 0] / -c[..., 2], self.cy - self.f * c[..., 1] / -c[..., 2]]).T


def ransac_circle(P, rng, rmin=.04, rmax=.13, thr=.003, iters=600):
    best, bn = None, 0
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        try:
            ctr, r = circle_fit(np.array([a, b, c]))
        except Exception:
            continue
        if not rmin < r < rmax:
            continue
        inl = np.abs(np.linalg.norm(P - ctr, axis=1) - r) < thr
        if inl.sum() > bn:
            best, bn = inl, inl.sum()
    ctr, r = circle_fit(P[best])
    return ctr, r, int(bn)


def fit(cam, depth, box, kind, rng):
    x0, y0, x1, y1 = box
    pad = int(.35 * max(x1 - x0, y1 - y0)) + 10
    u0, v0, u1, v1 = max(0, int(x0 - pad)), max(0, int(y0 - pad)), min(W, int(x1 + pad)), min(H, int(y1 + pad))
    vs, us = np.mgrid[v0:v1, u0:u1]
    us, vs = us.ravel().astype(float), vs.ravel().astype(float)
    d = depth[vs.astype(int), us.astype(int)].astype(float)
    ok = d < 4
    us, vs, d = us[ok], vs[ok], d[ok]
    X = cam.back(us, vs, d)
    inbox = (us >= x0 - 4) & (us <= x1 + 4) & (vs >= y0 - 4) & (vs <= y1 + 4)
    c, n, _ = ransac_plane(X[~inbox], prefer_normal=np.array([0, 0, 1.]), rng=rng)
    if n[2] < 0:
        n = -n
    h = (X - c) @ n
    near = (us >= x0 - pad / 3) & (us <= x1 + pad / 3) & (vs >= y0 - pad / 3) & (vs <= y1 + pad / 3)
    if kind == 'plate':
        cand = near & (h > .003) & (h < .055)
        top = np.percentile(h[cand], 99)
        sel = cand & (h > top - .012)
    else:
        sel = near & (h > .030) & (h < .075)
    e1 = np.cross(n, [0, 1., 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    Q = np.c_[(X[sel] - c) @ e1, (X[sel] - c) @ e2]
    centre = np.median(Q, 0)
    for _ in range(3):
        rel = Q - centre
        ang = np.arctan2(rel[:, 1], rel[:, 0])
        rad = np.linalg.norm(rel, axis=1)
        bins = np.floor((ang + math.pi) / math.radians(5)).astype(int)
        pts = np.array([Q[bins == b][np.argmax(rad[bins == b])] for b in np.unique(bins)])
        centre, r, nin = ransac_circle(pts, rng)
    hz = float(np.percentile(h[sel], 95))
    ctr3 = c + centre[0] * e1 + centre[1] * e2 + hz * n
    th = np.linspace(0, 2 * math.pi, 97)
    ring = ctr3 + r * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
    return dict(center=ctr3.tolist(), radius=float(r), n_points=int(sel.sum()), n_outline=int(len(pts)), n_inliers=nin,
                ring_px=cam.proj(ring).round(1).tolist(), center_px=cam.proj(ctr3).round(1).tolist())


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.


def main(tag=''):
    z = np.load(ROOT / 'out' / f'rgbd{V}.npz')
    cam = Cam(z)
    gt = json.loads((ROOT / 'out' / f'gt{V}.json').read_text())['items']
    g = json.loads((ROOT / 'out' / f'gpt{V}{tag}.json').read_text())
    gi = g['items']
    # ---- GT entries: the three plates form one stack entry
    plates = [o for o in gt if o['type'] == 'plate']
    stack = dict(type='plate', count=len(plates), dishwasher_safe=True, label='plate stack',
                 box=[min(o['box'][0] for o in plates), min(o['box'][1] for o in plates), max(o['box'][2] for o in plates),
                      max(o['box'][3] for o in plates)])
    gte = [stack] + [dict(o, count=1) for o in gt if o['type'] != 'plate']
    # ---- greedy IoU matching
    pairs = sorted(((iou(a['box'], b['box']), i, j) for i, a in enumerate(gi) for j, b in enumerate(gte)), reverse=True)
    mi, mj, match = set(), set(), {}
    for s, i, j in pairs:
        if s < .3 or i in mi or j in mj:
            continue
        mi.add(i), mj.add(j)
        match[j] = (i, s)
    rows = []
    for j, e in enumerate(gte):
        r = dict(gt=e['type'], gt_count=e['count'], gt_safe=e['dishwasher_safe'], gt_box=e['box'])
        if j in match:
            i, s = match[j]
            a = gi[i]
            r.update(gpt_idx=i, iou=round(s, 2), gpt_label=f"{a['label_en']} / {a['label_zh']}", gpt_cat=a['category'],
                     cat_ok=a['category'] == CAT_OF[e['type']], count_ok=a['count'] == e['count'],
                     safe_ok=a['dishwasher_safe'] == e['dishwasher_safe'], reason=f"{a['reason_en']} / {a['reason_zh']}",
                     slots=a['slots'])
        rows.append(r)
    extra = [i for i in range(len(gi)) if i not in mi]
    slots = [s for a in gi for s in a['slots']]
    slot_ok = len(slots) == len(set(slots)) and all(
        (s.startswith('P') if a['category'] == 'plate' else s.startswith('B')) for a in gi for s in a['slots']) and all(
        len(a['slots']) == (a['count'] if a['dishwasher_safe'] else 0) for a in gi)
    # ---- depth fits seeded by GPT boxes
    rng = np.random.default_rng(0)
    fits = []
    for j, e in enumerate(gte):
        if e['type'] not in ('plate', 'bowl') or j not in match:
            continue
        i = match[j][0]
        f = fit(cam, z['depth'], gi[i]['box'], e['type'], rng)
        if e['type'] == 'plate':
            truth = max(plates, key=lambda o: o['pos'][2])           # the top plate of the stack
        else:
            truth = e
        err = math.hypot(f['center'][0] - truth['pos'][0], f['center'][1] - truth['pos'][1]) * 1000
        f.update(kind=e['type'], gpt_idx=i, truth=truth['name'], truth_xy=truth['pos'][:2], err_mm=round(err, 1),
                 radius_err_mm=round((f['radius'] - TRUE_R[e['type']]) * 1000, 1))
        fits.append(f)
    n = len(gte)
    summ = dict(gt_entries=n, gt_objects=len(gt), gpt_entries=len(gi), detected=len(match), extra_gpt_entries=extra,
                category_ok=sum(r.get('cat_ok', False) for r in rows), count_ok=sum(r.get('count_ok', False) for r in rows),
                safe_ok=sum(r.get('safe_ok', False) for r in rows), slots_valid=slot_ok,
                mean_iou=round(float(np.mean([r['iou'] for r in rows if 'iou' in r])), 3),
                latency_s=g['latency_s'], tokens=g.get('tokens'))
    (ROOT / 'out' / f'score{V}{tag}.json').write_text(json.dumps(dict(summary=summ, rows=rows, fits=fits), indent=1, ensure_ascii=False))
    print(json.dumps(summ, ensure_ascii=False))
    for r in rows:
        print({k: v for k, v in r.items() if k not in ('gt_box',)})
    for f in fits:
        print(f['kind'], f['truth'], 'err', f['err_mm'], 'mm  r_fit', round(f['radius'] * 1000, 1), 'r_err', f['radius_err_mm'],
              'pts', f['n_points'], 'inl', f['n_inliers'], '/', f['n_outline'])


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '')
