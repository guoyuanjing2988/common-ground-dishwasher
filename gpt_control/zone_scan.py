"""Kinematic scan (IK + joint-path only, no physics): where does pick_plate plan, and from where can slide_plate
bring the plate to the nominal start (-0.15, 0.40)? -> out/zone_scan.json"""
import json, math
import numpy as np
import sim_robot as SR
from sim_robot import plan, S, rot_y, unit

r = SR.SimRobot(run_tag='zonescan')
res = []
for x in (-0.30, -0.26, -0.22, -0.19, -0.16, -0.13):
    for y in (0.30, 0.34, 0.38, 0.42, 0.46, 0.50, 0.54, 0.58):
        if np.hypot(x - SR.OPT.layout.dish[0], y - SR.OPT.layout.dish[1]) < S.R_OUT + S.DISH_R + .01:
            continue
        org = np.array([x, y, S.COUNTER_Z + .0003])
        pu = plan.push_poses(org)
        _, q1, _ = r.plan_path([(pu['p_pre'] + [0, 0, .05], pu['R']), (pu['p_pre'], pu['R']), (pu['p_pre'] + pu['drag'] * pu['drag_dir'], pu['R'])])
        org2 = org + pu['target_travel'] * pu['drag_dir']
        P2 = np.eye(3)
        tcp_g, Rg, _ = plan.pinch_pose(org2, P2, approach_deg=SR.OPT.approach_deg)
        path0 = plan.tilt_path(P2, org2, n=60)
        Rh, ph = P2.T @ Rg, P2.T @ (tcp_g - org2)
        tp = [(path0[i][2] + path0[i][1] @ ph, path0[i][1] @ Rh) for i in range(0, 61, 10)]
        kp = [(tcp_g - Rg[:, 2] * .12 + [0, 0, .03], Rg), (tcp_g - Rg[:, 2] * .12, Rg), (tcp_g, Rg)] + tp[1:] + [(tp[-1][0] + [0, 0, .12], tp[-1][1])]
        _, q2, _ = r.plan_path(kp) if q1 is not None else (1, None, 0)
        # slide to nominal
        tgt = np.array([-0.15, 0.40])
        d = tgt - org[:2]
        slide = None
        if np.linalg.norm(d) > .01:
            e_far = -unit(np.r_[d, 0])
            Rp = plan.flat_map(e_far) @ rot_y(-math.radians(plan.PUSH_PITCH)) @ plan.DOWN_F
            c0 = org + [0, 0, S.MID_Z]
            corner = c0 + .1009 * e_far + np.array([0, 0, -S.MID_Z + .0197])
            p_pre = corner + .0105 * Rp[:, 0] + plan.PUSH_S * Rp[:, 2] + .015 * e_far
            drag = np.linalg.norm(d) + plan.PUSH_LAG + .015
            _, q3, _ = r.plan_path([(p_pre + [0, 0, .05], Rp), (p_pre, Rp), (p_pre + drag * -e_far, Rp)], flips=(1, -1))
            slide = q3 is not None
        row = dict(x=x, y=y, push=q1 is not None, pinch_tilt=q2 is not None, slide_to_nominal=slide)
        print(row, flush=True)
        res.append(row)
json.dump(res, open('out/zone_scan.json', 'w'), indent=1)
