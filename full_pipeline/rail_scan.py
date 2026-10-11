#!/usr/bin/env python3
"""M2 design helper: per-task reach/collision results versus base position along the counter (y), at a fixed
pedestal x / height. Each rail position is evaluated as a fixed base (identical arm kinematics); the M2 study
then fixes one rail setpoint per step. Writes out/rail_scan.json."""
import itertools, json, sys
from multiprocessing import Pool
from pathlib import Path
import mount_study as MS
import scene as S


def job(a):
    x, y, z = a
    res = MS.evaluate_mount(S.Mount('M1', x=x, y=y, z=z, yaw=195.))
    return dict(x=x, y=y, z=z, res=[dict(target=r['target'], passed=r['passed'], margin=round(r['min_margin_deg'], 1),
                                         clear=round(r['min_clearance_mm'], 1), choice=r['choice']) for r in res])


if __name__ == '__main__':
    xs = [float(v) for v in sys.argv[1].split(',')] if len(sys.argv) > 1 else [.48]
    zs = [float(v) for v in sys.argv[2].split(',')] if len(sys.argv) > 2 else [.80, .90]
    ys = [0.0, .08, .16, .24, .32, .40, .48, .56]
    with Pool(16) as p:
        out = p.map(job, list(itertools.product(xs, ys, zs)))
    Path('out/rail_scan.json').write_text(json.dumps(out, indent=1))
    names = [r['target'] for r in out[0]['res']]
    for x, z in itertools.product(xs, zs):
        print(f'x={x} z={z}')
        for r in [o for o in out if o['x'] == x and o['z'] == z]:
            print(f"  y={r['y']:+.2f} " + '  '.join(f"{(str(t['margin']) if t['passed'] else 'FAIL'):>5s}" for t in r['res']))
    print('columns:', names)
