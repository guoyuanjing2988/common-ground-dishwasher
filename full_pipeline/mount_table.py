#!/usr/bin/env python3
"""Reach / collision table for the mounting options that were run end to end (MOUNTING.md).
Uses mount_study.evaluate_mount (IK on the full scene, door / basket in the state of each step, open-door and
counter collisions included). M2 is evaluated at each step's rail setpoint (fixed-base equivalent at that rail
position). Writes out/mount_table.json."""
import json
from multiprocessing import Pool
from pathlib import Path

import mount_study as MS
import run as RN
import scene as S

ROOT = Path(__file__).resolve().parent
FLAT = S.Layout()
M3_FLAT = S.Layout(plate_mode='flat', plate0=(-0.15, 0.80), dish=(-0.47, 0.58), cam_pos=(-0.27, 0.80, 1.46),
                   cam_look=(-0.07, 0.80, 0.90))
CONFIGS = {
    'M1': [(S.Mount('M1', .48, .42, .80, 195.), FLAT, None)],
    # M2: rail setpoint -> base y; target groups use the setpoint of their step
    'M2': [(S.Mount('M1', .40, .40 + dy, .80, 195., cart_lx=.57), FLAT, dy) for dy in (0., .08, -.04)],
    'M3 (rack)': [(S.Mount('M3', -.10, .38, .90, 315.), RN.M3_RACK, None)],
    'M3 (flat)': [(S.Mount('M3', -.10, .38, .90, 315.), M3_FLAT, None)],
    'M3a yaw 45 (rack)': [(S.Mount('M3', -.10, .38, .90, 45.), RN.M3_RACK, None)],
    'M3 back x=-0.30 yaw 135 (flat)': [(S.Mount('M3', -.30, .40, .90, 135.), S.Layout(
        plate_mode='flat', plate0=(-0.15, 0.82), dish=(-0.47, 0.60), cam_pos=(-0.27, 0.82, 1.46), cam_look=(-0.07, 0.82, 0.9)), None)],
}
# which rail setpoint each M2 target uses (from run.py OPTIONS['M2'].rail)
M2_SET = {'door open/close arc (0-90 deg)': 0., 'pod: dish -> door groove': 0., 'basket pull / push (0-0.25 m)': .08,
          'flat pick: drag, pinch, tilt, lift': -.04, 'carry + insert into plate slot': -.04, 'btn_quick press': .08,
          'btn_power press': .08}


def job(a):
    name, i = a
    m, lay, dy = CONFIGS[name][i]
    res = MS.evaluate_mount(m, layout=lay)
    return name, dy, [{k: v for k, v in r.items() if k != 'rows'} for r in res]


if __name__ == '__main__':
    jobs = [(n, i) for n, c in CONFIGS.items() for i in range(len(c))]
    with Pool(12) as p:
        out = p.map(job, jobs)
    table = {}
    for name, dy, res in out:
        if name == 'M2':
            for r in res:
                if M2_SET.get(r['target']) == dy:
                    r['rail_setpoint_m'] = dy
                    table.setdefault(name, []).append(r)
        else:
            table[name] = res
    order = list(M2_SET)
    for name in table:
        table[name].sort(key=lambda r: order.index(r['target']) if r['target'] in order else 3.5)
    (ROOT / 'out' / 'mount_table.json').write_text(json.dumps(table, indent=1, default=float))
    for name, rows in table.items():
        print(name)
        for r in rows:
            print(f"   {'PASS' if r['passed'] else 'FAIL'} {r['target']:36s} margin {r['min_margin_deg']:5.1f} deg  "
                  f"clear {r['min_clearance_mm']:6.1f} mm @{r['min_clearance_at']}  "
                  + ('' if r['passed'] else f"fail@{r['fail_at']['name']} err {r['fail_at']['pos_err_mm']:.1f} mm coll {r['fail_at']['collisions'][:2]} jump {r['fail_at']['jump_rad']:.2f}"))
