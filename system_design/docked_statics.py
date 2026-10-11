"""Re-run the M1 docked-cart statics with the REAL base (mass, CoG, braked drive wheels only, dock preload).

numpy only: reads the arm->base wrench samples already recorded by the sim
(../full_pipeline/out/M1_*_wrench.npy, rows t,Fx,Fy,Fz,Mx,My,Mz = force of the base ON the arm at the link0 origin,
world frame). Same conventions as ../full_pipeline/statics.py, but:
  * base mass / CoG from calc.py (arm, hand, F/T excluded: their weight is already in the wrench),
  * only the two drive wheels are braked (fail-safe brakes), carrying W_DRIVE_FRAC of the normal load,
  * the dock is preloaded by DOCK_PRELOAD (drive pushes the cart into the V-blocks, then brakes apply),
    so the brakes must hold preload + pull-away force.
Run: /opt/homebrew/bin/python3 docked_statics.py   -> docked_statics.json
"""
import json, pathlib
import numpy as np
import calc

HERE = pathlib.Path(__file__).parent
OUT = HERE.parent / "full_pipeline" / "out"
G = 9.81
CAB_FRONT_X, LX, LY, ARM = -0.02, 0.65, 0.60, (0.48, 0.42, 0.80)
INSET = 0.06
MU = calc.MU_FLOOR
W_DRIVE_FRAC = 0.55
DOCK_PRELOAD = calc.DOCK_PRELOAD


def main():
    W = np.concatenate([np.load(OUT / f) for f in ("M1_single_wrench.npy", "M1_trials_wrench.npy")])
    F, M = -W[:, 1:4], -W[:, 4:7]
    base = [p for p in calc.MASSES if not p[0].startswith(("FR3 arm", "Franka Hand", "Wrist F/T"))]
    mb, cx, cy, cz = calc.mass_props(base)
    x0 = CAB_FRONT_X
    xc_w = x0 + LX / 2 + cx        # base CoG in world (base frame origin = footprint centre)
    yc_w = ARM[1] + cy
    sx0, sx1 = x0 + INSET, x0 + LX - INSET
    sy0, sy1 = ARM[1] - LY / 2 + INSET, ARM[1] + LY / 2 - INSET
    rm = np.array(ARM)
    MO = M + np.cross(rm, F)
    Wb = mb * G
    N = Wb - F[:, 2]
    xp = (xc_w * Wb + MO[:, 1]) / N
    yp = (yc_w * Wb - MO[:, 0]) / N
    marg = dict(front=float(np.min(sx1 - xp)), rear=float(np.min(xp - sx0)),
                left=float(np.min(yp - sy0)), right=float(np.min(sy1 - yp)))
    away = np.maximum(F[:, 0], 0) + DOCK_PRELOAD
    util = np.hypot(away, F[:, 1]) / (MU * W_DRIVE_FRAC * N)
    toward = float(max(0., -F[:, 0].min()))
    res = {
        "samples": int(len(W)),
        "base_mass_excl_arm_kg": round(mb, 1),
        "base_cog_world_m": [round(xc_w, 3), round(yc_w, 3), round(cz, 3)],
        "cop_margin_m": {k: round(v, 3) for k, v in marg.items()},
        "min_free_edge_margin_m (front/left/right; rear is the dock)": round(min(marg["front"], marg["left"], marg["right"]), 3),
        "sim_reference_margin_m": 0.183,
        "dock_preload_N": DOCK_PRELOAD,
        "max_force_toward_counter_N": round(toward, 1),
        "dock_contact_force_max_N": round(DOCK_PRELOAD + toward, 1),
        "brake_utilisation_max (drive wheels only, mu=%.2f)" % MU: round(float(util.max()), 3),
        "note": "Quasi-static, recorded task wrenches only (no human push, no e-stop with arm extended).",
    }
    (HERE / "docked_statics.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
