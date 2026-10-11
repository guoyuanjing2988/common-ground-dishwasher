"""Sink: score GPT (out/sink_gpt.json) vs ground truth (out/sink_gt.json) incl. PHYSICAL pick-order check, and fit a plane
to the exposed surface of every dish GPT boxed (depth back-projection + RANSAC; numpy only, no sim) -> out/sink_score.json.

Pick-order check: GPT's step k is physically valid if every dish it supports (support test in sink_scene.py: removing it moves
that dish > 5 mm) was already picked. A mutual pair (A and B lean on each other) counts as valid in either order.
Surface fit: points inside the GPT box, above the basin floor (+6 mm; basin geometry is known), RANSAC plane (3 mm) ->
largest exposed flat patch; scored by segmentation purity (share of patch pixels on the matched dish) and normal vs dish axis."""
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
P = os.environ.get('SINK_PREFIX', 'sink')   # 'sink' (default layout) or 'sink6' (layout matching sink_refs/sink_6.png)

sys.path.insert(0, str(_ROOT / 'full_pipeline'))
sys.path.insert(0, str(ROOT))
from perception import ransac_plane  # noqa: E402
from fit_score import Cam, iou  # noqa: E402
import sink_scene as SS  # noqa: E402

CAT_OF = dict(plate='plate', bowl='bowl', mug='mug', board='cutting_board', knife='knife', pan='pan', flask='flask',
              soap='soap_bottle', sponge='sponge', cloth='dish_cloth')


def main():
    z = np.load(ROOT / 'out' / f'{P}_rgbd.npz')
    cam = Cam(z)
    gt = json.loads((ROOT / 'out' / f'{P}_gt.json').read_text())['items']
    g = json.loads((ROOT / 'out' / f'{P}_gpt.json').read_text())
    gi = g['items']
    pairs = sorted(((iou(a['box'], b['box']) + (.5 if a['category'] == CAT_OF[b['type']] else 0), i, j)
                    for i, a in enumerate(gi) for j, b in enumerate(gt)), reverse=True)
    mi, mj, match = set(), set(), {}
    for s, i, j in pairs:
        if s < .3 or i in mi or j in mj:
            continue
        mi.add(i), mj.add(j)
        match[i] = j
    rows = []
    for i, a in enumerate(gi):
        r = dict(gpt_idx=i, gpt=f"{a['label_en']} / {a['label_zh']}", cat=a['category'], pick=a['pick_order'])
        if i in match:
            o = gt[match[i]]
            r.update(gt=o['name'], iou=round(iou(a['box'], o['box']), 2), cat_ok=a['category'] == CAT_OF[o['type']],
                     safe_ok=a['dishwasher_safe'] == o['dishwasher_safe'])
        rows.append(r)
    # ---- physical pick order
    seq = sorted([i for i in range(len(gi)) if gi[i]['pick_order'] > 0], key=lambda i: gi[i]['pick_order'])
    by = {o['name']: o for o in gt}
    picked, steps = set(), []
    for k, i in enumerate(seq):
        nm = gt[match[i]]['name'] if i in match else None
        if nm is None or not by[nm].get('dish'):
            steps.append(dict(step=k + 1, gpt_idx=i, gt=nm, valid=False, why='not a dish in GT'))
            continue
        o = by[nm]
        pending = [s for s in o['supports'] if s not in picked and nm not in by[s]['supports']]
        mutual = [s for s in o['supports'] if s not in picked and nm in by[s]['supports']]
        steps.append(dict(step=k + 1, gpt_idx=i, gt=nm, valid=not pending, still_resting_on_it=pending,
                          moves_mm={s: o['removal_moves_mm'][s] for s in pending}, mutual_lean=mutual))
        picked.add(nm)
    dishes_gt = [o['name'] for o in gt if o.get('dish')]
    # ---- exposed-surface plane fits
    rng = np.random.default_rng(0)
    seg, gb, names = z['seg'], z['geom_body'], list(z['body_names'])
    fits = []
    for i in seq:
        x0, y0, x1, y1 = [int(round(v)) for v in gi[i]['box']]
        vs, us = np.mgrid[max(0, y0):min(720, y1), max(0, x0):min(1280, x1)]
        us, vs = us.ravel(), vs.ravel()
        X = cam.back(us.astype(float), vs.astype(float), z['depth'][vs, us].astype(float))
        keep = X[:, 2] > SS.FLOOR_Z + .006
        if keep.sum() < 50:
            continue
        c, n, inl = ransac_plane(X[keep], thr=.003, iters=400, rng=rng)
        if n[2] < 0:
            n = -n
        uu, vv = us[keep][inl], vs[keep][inl]
        sg = seg[vv, uu]
        bodies = [names[gb[q]] if t == 5 and q >= 0 else '' for q, t in zip(sg[:, 0], sg[:, 1])]
        nm = gt[match[i]]['name'] if i in match else None
        purity = float(np.mean([b == nm for b in bodies])) if nm else 0.
        ax = np.array(by[nm]['axis']) if nm else np.array([0, 0, 1.])
        ang = math.degrees(math.acos(min(1., abs(float(n @ ax)))))
        tilt = math.degrees(math.acos(min(1., abs(n[2]))))
        tip = c + .06 * n
        fits.append(dict(gpt_idx=i, gt=nm, n_inliers=int(inl.sum()), purity=round(purity, 2), tilt_deg=round(tilt, 1),
                         gt_tilt_deg=by[nm]['tilt_deg'] if nm else None, normal_err_deg=round(ang, 1),
                         center_px=cam.proj(c).round(1).tolist(), tip_px=cam.proj(tip).round(1).tolist(),
                         patch_px=np.c_[uu, vv][::max(1, int(inl.sum()) // 400)].tolist()))
    summ = dict(gt_objects=len(gt), gpt_entries=len(gi), matched=len(match), category_ok=sum(r.get('cat_ok', False) for r in rows),
                safe_ok=sum(r.get('safe_ok', False) for r in rows), mean_iou=round(float(np.mean([r['iou'] for r in rows if 'iou' in r])), 3),
                dishes_gt=len(dishes_gt), dishes_ordered=len(seq), pick_steps_valid=sum(s['valid'] for s in steps),
                first_pick_free=bool(steps and steps[0]['valid']), latency_s=g['latency_s'], tokens=g.get('tokens'))
    (ROOT / 'out' / f'{P}_score.json').write_text(json.dumps(dict(summary=summ, rows=rows, steps=steps, fits=fits), indent=1, ensure_ascii=False))
    print(json.dumps(summ))
    for r in rows:
        print(r)
    for s in steps:
        print(s)
    for f in fits:
        print({k: v for k, v in f.items() if k != 'patch_px'})


if __name__ == '__main__':
    main()
