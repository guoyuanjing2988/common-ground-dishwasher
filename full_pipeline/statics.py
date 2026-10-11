#!/usr/bin/env python3
"""Quasi-static stability check of the robot mounting, from the simulated arm -> base wrench.

Input: out/<OPT>_single_wrench.npy and out/<OPT>_trials_wrench.npy, rows (t, Fx, Fy, Fz, Mx, My, Mz) = MuJoCo
force/torque sensor at the Panda link0 origin (force from the base ON the arm, world frame, torque about the mount
point). The arm's action ON the base is the negative. It contains the arm weight (gravity compensation is internal
joint torque in the model, the sensor still sees the weight), arm dynamics and every contact reaction (door, basket,
plate push, button presses, collisions).

Assumptions (all stated in MOUNTING.md, all rough):
  M1 / M2 cart: rigid steel cart with ballast, mass CART_MASS at the footprint centre, height irrelevant for
  quasi-statics; column 15 kg at the base xy; footprint as built in scene.py; braked castors 60 mm inside the
  footprint edges; braked-castor friction mu = 0.5 on kitchen tiles. The cabinet doors are behind the cart's
  rear bumper: they can only PUSH the cart away from the counter (+x), so tipping/sliding toward the counter is
  blocked, tipping/sliding away from it is not.
  M2: carriage anywhere on its 0.40 m travel (worst case taken for every sample, conservative).
  M3: Panda base plate clamped on the worktop; clamp/bolt square 0.20 m (Franka base footprint ~0.2 m); the
  required preload is what keeps the base plate from lifting about its edge and from sliding (mu = 0.3 on stone).
Outputs out/statics.json and prints a table.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import run as RN
import scene as S

ROOT = Path(__file__).resolve().parent
G = 9.81
INSET = .06
MU_CASTOR, MU_CLAMP = .5, .3
COLUMN_MASS = 15.
CLAMP_B = .20


def load(opt):
    arrs = []
    for f in (f'{opt}_single_wrench.npy', f'{opt}_trials_wrench.npy'):
        p = ROOT / 'out' / f
        if p.exists():
            a = np.load(p)
            if a.size:
                arrs.append(a)
    return np.concatenate(arrs) if arrs else np.zeros((0, 7))


def cart_check(m: S.Mount, W):
    F = -W[:, 1:4]          # arm on base
    M = -W[:, 4:7]          # about the mount point
    x0, x1 = S.CAB_FRONT_X, S.CAB_FRONT_X + m.cart_lx
    ly = m.cart_ly + (m.rail_travel if m.kind == 'M2' else 0.)
    y0, y1 = m.y - ly / 2, m.y + ly / 2
    sx0, sx1, sy0, sy1 = x0 + INSET, x1 - INSET, y0 + INSET, y1 - INSET
    xc, yc = (x0 + x1) / 2, m.y
    Wc = m.cart_mass * G
    Wcol = COLUMN_MASS * G
    rails = [0.] if m.kind != 'M2' else [-m.rail_travel / 2, m.rail_travel / 2]
    worst = dict(front=9., left=9., right=9., rear=9.)
    for dy in rails:
        rm = np.array([m.x, m.y + dy, m.z])
        MO = M + np.cross(rm, F)                       # arm wrench moment about the ground origin
        N = Wc + Wcol - F[:, 2]                        # F_z < 0: the arm pushes down
        xp = (xc * Wc + m.x * Wcol + MO[:, 1]) / N
        yp = (yc * Wc + (m.y + dy) * Wcol - MO[:, 0]) / N
        worst['front'] = min(worst['front'], float(np.min(sx1 - xp)))
        worst['rear'] = min(worst['rear'], float(np.min(xp - sx0)))
        worst['left'] = min(worst['left'], float(np.min(yp - sy0)))
        worst['right'] = min(worst['right'], float(np.min(sy1 - yp)))
    N = Wc + Wcol - F[:, 2]
    bumper = float(max(0., -F[:, 0].min()))           # toward the counter: taken by the cabinet / bumper
    away = float(max(0., F[:, 0].max()))
    side = float(np.abs(F[:, 1]).max())
    brake_util = float(np.max(np.hypot(np.maximum(F[:, 0], 0), F[:, 1]) / (MU_CASTOR * N)))
    # minimum cart mass that keeps the CoP >= 20 mm inside every free edge (front, left, right)
    def margin_for(mass):
        Wc_ = mass * G
        res = 9.
        for dy in rails:
            rm = np.array([m.x, m.y + dy, m.z])
            MO = M + np.cross(rm, F)
            N_ = Wc_ + Wcol - F[:, 2]
            xp = (xc * Wc_ + m.x * Wcol + MO[:, 1]) / N_
            yp = (yc * Wc_ + (m.y + dy) * Wcol - MO[:, 0]) / N_
            res = min(res, float(np.min(sx1 - xp)), float(np.min(yp - sy0)), float(np.min(sy1 - yp)))
        return res
    lo, hi = 0., 400.
    if margin_for(hi) < .02:
        mmin = None
    else:
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if margin_for(mid) < .02 else (lo, mid)
        mmin = hi
    return dict(footprint_m=[round(m.cart_lx, 3), round(ly, 3)], support_polygon_x=[round(sx0, 3), round(sx1, 3)],
                support_polygon_y=[round(sy0, 3), round(sy1, 3)], cart_mass_kg=m.cart_mass, column_mass_kg=COLUMN_MASS,
                cop_margin_m={k: round(v, 3) for k, v in worst.items()},
                min_free_edge_margin_m=round(min(worst['front'], worst['left'], worst['right']), 3),
                max_force_toward_counter_N=round(bumper, 1), max_force_away_from_counter_N=round(away, 1),
                max_side_force_N=round(side, 1), brake_friction_utilisation=round(brake_util, 3),
                min_cart_mass_for_20mm_margin_kg=None if mmin is None else round(mmin, 1),
                arm_vertical_load_N=[round(float(F[:, 2].min()), 1), round(float(F[:, 2].max()), 1)],
                max_overturning_moment_at_mount_Nm=round(float(np.linalg.norm(M[:, :2], axis=1).max()), 1),
                samples=int(len(W)))


def clamp_check(m: S.Mount, W):
    F, M = -W[:, 1:4], -W[:, 4:7]
    Mh = np.linalg.norm(M[:, :2], axis=1)
    # lift-off about the plate edge: preload P with lever b/2 plus the downward arm load must exceed Mh
    down = -F[:, 2]                                    # > 0 when the arm presses the base down
    P_tip = np.max((Mh - down * CLAMP_B / 2) / (CLAMP_B / 2))
    P_slide = np.max(np.linalg.norm(F[:, :2], axis=1) / MU_CLAMP - down)
    return dict(clamp_square_m=CLAMP_B, max_overturning_moment_Nm=round(float(Mh.max()), 1),
                max_horizontal_force_N=round(float(np.linalg.norm(F[:, :2], axis=1).max()), 1),
                arm_vertical_load_N=[round(float(F[:, 2].min()), 1), round(float(F[:, 2].max()), 1)],
                required_clamp_preload_N=round(float(max(P_tip, P_slide, 0.)), 0),
                preload_tip_N=round(float(P_tip), 0), preload_slide_N=round(float(P_slide), 0), samples=int(len(W)))


def main():
    out = {}
    for name in ('M1', 'M2', 'M3', 'M3a', 'M3flat'):
        W = load(name)
        if not len(W):
            continue
        m = RN.OPTIONS[name].mount
        out[name] = cart_check(m, W) if m.kind in ('M1', 'M2') else clamp_check(m, W)
        print(name, json.dumps(out[name]))
    (ROOT / 'out' / 'statics.json').write_text(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
