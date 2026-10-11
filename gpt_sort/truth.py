"""Ground truth for SCORING ONLY: projected item boxes in the wall image, GPT-item <-> true-item matching, and the
program rule (defined before any GPT run; documented in GPT_SORT.md)."""
from __future__ import annotations

import numpy as np

CAT_OF_TYPE = dict(plate='plate', bowl='bowl', mug='mug', board='cutting_board', knife='knife', pan='pan', flask='flask')


def program_rule(items):
    """Ground-truth program for a set of loaded items (each with GT 'soil' and 'greasy').
    intensive : any item heavily soiled or greasy
    quick     : every item lightly soiled and at most 3 items
    eco       : otherwise (normal everyday load)
    None for an empty load."""
    if not items:
        return None
    if any(it['soil'] == 'heavy' or it['greasy'] for it in items):
        return 'intensive'
    if all(it['soil'] == 'light' for it in items) and len(items) <= 3:
        return 'quick'
    return 'eco'


def true_boxes(model, data, cam, names, W, H):
    """2-D box of every item in the wall image: project the 8 corners of every geom's local AABB (incl. visual decals)."""
    out = {}
    for nm in names:
        bid = model.body(nm).id
        pts = []
        for g in range(model.body_geomadr[bid], model.body_geomadr[bid] + model.body_geomnum[bid]):
            if model.geom_rgba[g][3] == 0:
                continue
            c, hs = model.geom_aabb[g][:3], model.geom_aabb[g][3:]
            R = data.geom_xmat[g].reshape(3, 3)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        pts.append(data.geom_xpos[g] + R @ (c + hs * [sx, sy, sz]))
        uv = cam.project(data, np.array(pts))
        x0, y0 = np.clip(uv.min(0), 0, [W - 1, H - 1])
        x1, y1 = np.clip(uv.max(0), 0, [W - 1, H - 1])
        out[nm] = [float(x0), float(y0), float(x1), float(y1)]
    return out


def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.


def match(gpt_items, tboxes, thr=.25):
    """Greedy one-to-one matching by IoU (>= thr). Returns {gpt_id: true_name}, unmatched gpt ids, unmatched true names."""
    pairs = sorted(((iou(g['box'], tb), g['id'], n) for g in gpt_items for n, tb in tboxes.items()), reverse=True)
    m, used_g, used_t = {}, set(), set()
    for v, gid, n in pairs:
        if v < thr:
            break
        if gid in used_g or n in used_t:
            continue
        m[gid] = n
        used_g.add(gid)
        used_t.add(n)
    return m, [g['id'] for g in gpt_items if g['id'] not in used_g], [n for n in tboxes if n not in used_t]
