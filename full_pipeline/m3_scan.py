#!/usr/bin/env python3
"""M3 (counter clamp) scan: robot clamped on the worktop beside the dishwasher; the plate start area / drying
rack and the pod dish beside the robot (same distances as in M1). Writes out/m3_scan.json."""
import itertools, json
from multiprocessing import Pool
from pathlib import Path
import mount_study as MS


def layout_for(x, y, mode):
    s = 1 if y > 0 else -1
    py = y + s * .42
    lay = dict(plate0=(-0.15, py), plate_mode=mode, dish=(-0.47, y + s * .20), rack=(-0.22, py),
               cam_pos=(-0.27, py, 1.46), cam_look=(-0.07, py, 0.90))
    return lay


if __name__ == '__main__':
    jobs = []
    for (x, y), yaw, mode in itertools.product([(-.10, .38), (-.20, .38), (-.30, .40), (-.10, -.40), (-.20, -.40), (-.08, -.50)],
                                               (0., 45., 90., 135., 180., 225., 270., 315.), ('flat', 'rack')):
        jobs.append(dict(kind='M3', x=x, y=y, z=.90, yaw=yaw, layout=layout_for(x, y, mode)))
    with Pool(16) as p:
        res = p.map(MS._job, jobs)
    for r, j in zip(res, jobs):
        r['layout'] = j['layout']
    res.sort(key=lambda r: (r.get('n_pass', -1), r.get('worst_margin_deg', 0)), reverse=True)
    Path('out/m3_scan.json').write_text(json.dumps(res, indent=1, default=float))
    for r in res[:40]:
        if 'error' in r:
            print('ERR', r['mount'], r['error'][:200]); continue
        print(f"{r['label']:40s} {r['layout']['plate_mode']:4s} {r['n_pass']}/{r['n']} margin {r['worst_margin_deg']:5.1f} "
              + ' '.join(('1' if x['passed'] else '0') for x in r['results']))
    print([x['target'] for x in res[0]['results']])
