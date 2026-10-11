"""Task-space pose generators shared by the controller (run.py) and the mount study.

All functions return TCP targets (p, R) in world coordinates; R columns = hand x, y (finger closing
axis), z (approach, palm -> fingertips). TCP = pad tip (hand z + 0.1114 m), as in the PoCs.

Flat-plate geometry is ported 1:1 from ../poc_plate_flat/flat_poc.py. That PoC worked in a frame
where the robot stood at the origin and the counter edge faced it along -x; here the same
construction is written with an explicit horizontal unit vector `e_far` (from the plate centre away
from the counter edge), so it works for any counter/robot layout.
"""
from __future__ import annotations

import math

import numpy as np

import dishwasher as dw
import scene as S
from util import hand_frame, rot_axis, rot_y, rot_z, unit

DOWN_F = np.diag([1., -1, -1])        # FLAT PoC: hand z down, fingers close along FLAT y
E_EDGE = np.array([1., 0, 0])         # this scene: plate centre -> counter edge (+x)
E_FAR = -E_EDGE


def flat_map(e_far=E_FAR):
    """Rotation taking the FLAT PoC frame (e_far = +x_F) to the world."""
    return rot_z(math.atan2(e_far[1], e_far[0]))


# ------------------------------------------------------------------ flat plate: push to the edge
PUSH_PITCH, PUSH_S, PUSH_LAG, OVERHANG, PINCH_DEPTH, APPROACH_DEG = 30., .008, .01, .045, .012, 60.


def push_poses(origin, e_far=E_FAR, overhang=OVERHANG, pitch=PUSH_PITCH):
    """Edge-mode push (FLAT_NOTES step 2). origin = plate body origin (foot plane centre).
    Returns dict with Rpush, p_pre (finger side face 15 mm behind the far rim's outer top edge),
    drag direction and planned drag distance."""
    M = flat_map(e_far)
    c0 = origin + np.array([0, 0, S.MID_Z])
    Rpush = M @ rot_y(-math.radians(pitch)) @ DOWN_F
    hx, d = Rpush[:, 0], Rpush[:, 2]
    corner = c0 + .1009 * e_far + np.array([0, 0, -S.MID_Z + .0197])
    p_contact = corner + .0105 * hx + PUSH_S * d
    p_pre = p_contact + .015 * e_far
    edge_dist = edge_distance(origin, e_far)        # plate centre -> counter edge, along -e_far
    target_travel = edge_dist - S.R_OUT + overhang   # centre travel for the wanted overhang
    drag = target_travel + PUSH_LAG + .015
    return dict(R=Rpush, p_pre=p_pre, drag_dir=-e_far, drag=drag, target_travel=target_travel, edge_dist=edge_dist)


def edge_distance(origin, e_far=E_FAR):
    """Distance from the plate centre to the counter edge line (x = 0), along -e_far."""
    return float(-origin[0] / (-e_far[0])) if abs(e_far[0]) > .5 else float('nan')


def pinch_pose(origin, P, e_far=E_FAR, approach_deg=APPROACH_DEG, depth=PINCH_DEPTH):
    """FLAT_NOTES step 3: pinch the overhanging rim, pads aligned with the 15 deg rim face,
    approach in the rim-face plane at `approach_deg` from radial. Returns (tcp, R, pinch_point)."""
    e_r = P.T @ (-e_far)
    e_r[2] = 0
    e_r /= np.linalg.norm(e_r)
    e_t = np.cross(e_r, [0, 0, 1.])
    r_p = S.R_OUT - depth
    z_mid = .0075 + S.RIM_RISE * (r_p - S.R_DISC) / (S.R_OUT - S.R_DISC)
    ez = np.array([0, 0, 1.])
    pinch = origin + P @ np.r_[r_p * e_r[:2], z_mid]
    n_face = P @ (-math.sin(S.SLOPE) * e_r + math.cos(S.SLOPE) * ez)
    s_in = P @ (-math.cos(S.SLOPE) * e_r - math.sin(S.SLOPE) * ez)
    b = math.radians(approach_deg)
    # FLAT PoC used +60 deg; here a negative angle approaches from the other tangential side
    hz = unit(math.cos(b) * s_in + math.sin(b) * (P @ e_t))
    hy = -n_face
    R = np.column_stack([np.cross(hy, hz), hy, hz])
    return pinch + hz * .0085, R, pinch


def tilt_path(Rp0, pp0, e_far=E_FAR, n=400):
    """Plate poses for tilting it up about its far rim (FLAT_NOTES step 4): rotation about
    a = z x e_far; the pivot switches from the far foot-ring corner to the far rim's outer-bottom
    corner when the latter reaches the counter (rolling, no sliding)."""
    axis = np.cross([0, 0, 1.], e_far)
    el = Rp0.T @ e_far
    el[2] = 0
    el /= np.linalg.norm(el)
    cands = [np.r_[(S.R_FOOT + .002) * el[:2], 0.], np.r_[.1022 * el[:2], .0149]]
    piv_local = cands[0]
    piv = pp0 + Rp0 @ piv_local
    R_cur, p_cur, theta0 = Rp0, pp0, 0.
    out = []
    for th in np.linspace(0, math.pi / 2, n + 1):
        Ra = rot_axis(axis, th - theta0)
        R = Ra @ R_cur
        p = piv + Ra @ (p_cur - piv)
        z = [(p + R @ c)[2] for c in cands]
        j = int(np.argmin(z))
        if not np.allclose(cands[j], piv_local) and z[j] < (p + R @ piv_local)[2] - 1e-6:
            piv_local, piv = cands[j], p + R @ cands[j]
            R_cur, p_cur, theta0 = R, p, th
        out.append((th, R, p))
    return out


def nominal_flat_sequence(plate_xy=S.PLATE0, approach_deg=APPROACH_DEG, lift=.12):
    """Nominal hand waypoints of the whole flat pick for a plate at plate_xy (used by the mount study)."""
    origin = np.array([plate_xy[0], plate_xy[1], S.COUNTER_Z + .0003])
    P = np.eye(3)
    pu = push_poses(origin)
    wp = [('push_above', pu['p_pre'] + [0, 0, .05], pu['R']), ('push_start', pu['p_pre'], pu['R'])]
    end = pu['p_pre'] + pu['drag'] * pu['drag_dir']
    for k in (1, 2, 3):
        wp.append((f'push_drag{k}', pu['p_pre'] + k / 3 * pu['drag'] * pu['drag_dir'], pu['R']))
    origin2 = origin + pu['target_travel'] * pu['drag_dir']
    tcp, Rg, _ = pinch_pose(origin2, P, approach_deg=approach_deg)
    hz = Rg[:, 2]
    wp += [('pinch_pre', tcp - hz * .12 + [0, 0, .03], Rg), ('pinch_in', tcp - hz * .12, Rg), ('pinch', tcp, Rg)]
    path = tilt_path(P, origin2)
    Rh_p, ph_p = P.T @ Rg, P.T @ (tcp - origin2)
    for i in range(1, 7):
        _, Rpl, ppl = path[int(round(i / 6 * (len(path) - 1)))]
        wp.append((f'tilt{15 * i}', ppl + Rpl @ ph_p, Rpl @ Rh_p))
    _, Rpl, ppl = path[-1]
    wp.append(('lift', ppl + Rpl @ ph_p + [0, 0, lift], Rpl @ Rh_p))
    return wp, dict(Rpl_end=Rpl, ppl_end=ppl, Rh_p=Rh_p, ph_p=ph_p, end=end)


# ------------------------------------------------------------------ dishwasher geometry (known appliance model)
HINGE = np.array([S.DW_POS[0] + dw.X_FRONT, S.DW_POS[1], S.DW_POS[2] + dw.HINGE_Z])
HANDLE_LOCAL = np.array([dw.HANDLE_OFF, 0., dw.HANDLE_Z])


def door_R(theta):
    return rot_y(theta)


def handle_pos(theta):
    return HINGE + door_R(theta) @ HANDLE_LOCAL


def door_bar_frame(theta, kappa=.5, psi_deg=0., flip=1, alpha_override=None):
    """Bar grasp on the door pull bar at door angle theta: fingers straddle the bar.
    Approach a = R_y(alpha) (-x) with alpha = kappa*theta (0: stays horizontal, 1: rotates with
    the door, i.e. keeps pointing at the door face); psi tilts it along the bar."""
    alpha = kappa * theta if alpha_override is None else alpha_override
    a = rot_y(alpha) @ np.array([-1., 0, 0])
    c = rot_y(alpha) @ np.array([0, 0, 1.])
    a = rot_axis(c, math.radians(psi_deg)) @ a
    return hand_frame(a, flip * c)


def groove_center(theta=math.pi / 2):
    """Groove recess centre (dw local site door_groove) for door angle theta."""
    loc = np.array([-dw.DOOR_T + dw.GROOVE[2] / 2, dw.GROOVE_Y, dw.GROOVE_Z])
    return HINGE + door_R(theta) @ loc


def groove_face_z():
    """Door inner face height when the door is open 90 deg."""
    return HINGE[2] + dw.DOOR_T


def basket_handle(basket):
    return np.array([S.DW_POS[0] + dw.BASKET_X0 + dw.BASKET_HX + basket, S.DW_POS[1], S.DW_POS[2] + dw.BASKET_Z + dw.HANDLE_B_Z])


def basket_bar_frame(pitch_deg=0., psi_deg=0., flip=1):
    a = rot_y(math.radians(pitch_deg)) @ np.array([-1., 0, 0])
    c = rot_y(math.radians(pitch_deg)) @ np.array([0, 0, 1.])
    a = rot_axis(c, math.radians(psi_deg)) @ a
    return hand_frame(a, flip * c)


def plate_slot(basket=dw.BASKET_TRAVEL):
    """Seated-plate centre in the 2nd slot from the front (dw site plate_slot_target)."""
    sx = (dw.TINE_X[-2] + dw.TINE_X[-3]) / 2
    return np.array([S.DW_POS[0] + dw.BASKET_X0 + basket + sx, S.DW_POS[1] + sum(dw.PLATE_ROWS_Y) / 2,
                     S.DW_POS[2] + dw.BASKET_Z + dw.WIRE_R + dw.PLATE_R + .001])


def button_pos(name):
    i = dw.BUTTONS.index(name)
    return np.array([S.DW_POS[0] + dw.X_FRONT + 2 * dw.BTN_HALF[0], S.DW_POS[1] + dw.BTN_Y0 + dw.BTN_PITCH * i,
                     S.DW_POS[2] + dw.BTN_Z])


def press_frame(pitch_deg=0., yaw_deg=0., close='vertical', flip=1):
    """Closed fingertips pressing along -x; pitch tilts the approach downward, yaw sideways."""
    a = rot_z(math.radians(yaw_deg)) @ rot_y(math.radians(pitch_deg)) @ np.array([-1., 0, 0])
    c = np.array([0, 0, 1.]) if close == 'vertical' else np.array([0, 1., 0])
    return hand_frame(a, flip * c)


def pod_down_frame(close_axis, yaw_extra=0.):
    return hand_frame([0, 0, -1.], rot_z(yaw_extra) @ np.asarray(close_axis, float))
