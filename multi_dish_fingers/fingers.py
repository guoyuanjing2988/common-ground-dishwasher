"""Custom dish fingers for the Franka Hand: geometry (convex hexahedra), MuJoCo injection, STL export.

Finger local frame = Franka finger body frame: z along the finger (0 = mounting face on the hand slider),
y = closing axis with the inner (gripping) face at y = 0 and material at y > 0, x = finger width.
At q = 0 the two inner faces touch at the hand mid-plane (as the stock fingers).

HOOK finger  (left slider): flat TPU pad at the tip + a 3 mm SHOULDER (rim hook) 10 mm behind the tip. The dish
             rim edge seats against the shoulder, so a hanging plate / tilted bowl is held by form closure.
WEDGE finger (right slider): thin tapered blade, 2.0 mm at the tip -> 3.6 mm 22 mm back, slides into the 4.6 mm
             gap between stacked plate rims (or down the outside of a nested bowl wall) to separate the top dish.
Every piece is convex (8-vertex hexahedron); MuJoCo uses the convex hull of each mesh = the exact piece.
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

PETG_RHO, TPU_RHO = 1270., 1210.          # kg/m^3
HOOK_ON_LEFT = True                       # hook on the left slider (hand +y side), wedge on the right
STOCK_L = 0.0535                          # stock finger: mount face -> pad tip (menagerie pad box top z)
PAD_T = 0.0008                            # TPU pad thickness
SHOULDER_H, SHOULDER_D, SHOULDER_LEN = 0.003, 0.022, 0.003   # rim hook: height, distance from hook tip, length
WEDGE_SHORT = 0.012                       # wedge finger is 12 mm shorter: it reaches 10 mm past the shoulder line
TCP_IN = 0.005                            # grasp point: 5 mm inside the rim edge (from the shoulder line)


def hexa(z0, z1, w0, w1, y00, y01, y10, y11):
    """Convex piece between stations z0 and z1: half-width w, inner face y0*, outer face y1* (at z0, z1)."""
    v = []
    for z, w, yi, yo in ((z0, w0, y00, y10), (z1, w1, y01, y11)):
        for x in (-w, w):
            v += [(x, yi, z), (x, yo, z)]
    return np.array(v, float)


def finger_pieces(kind, L):
    """List of (name, vertices, material) for one finger of total length L (mount face -> tip)."""
    P = []
    root = 0.012
    P.append(('root', hexa(0, root, .0115, .0115, PAD_T, PAD_T, .014, .014), 'petg'))
    if kind == 'hook':
        zp = L - SHOULDER_D                                   # pad zone z in [zp, L]
        zs = zp - SHOULDER_LEN
        P.append(('beam', hexa(root, zs, .011, .010, PAD_T, PAD_T, .012, .0075), 'petg'))
        P.append(('shoulder', hexa(zs, zp, .010, .010, -SHOULDER_H, -SHOULDER_H, .0075, .007), 'petg'))
        P.append(('tip', hexa(zp, L, .010, .010, PAD_T, PAD_T, .007, .0055), 'petg'))
        P.append(('pad', hexa(zp + .0005, L - .0005, .0095, .0095, 0, 0, PAD_T, PAD_T), 'tpu'))
    else:
        L = L - WEDGE_SHORT
        zb = L - .022                                          # blade z in [zb, L]
        P.append(('beam', hexa(root, zb, .011, .010, PAD_T, PAD_T, .012, .0036), 'petg'))
        P.append(('blade', hexa(zb, L, .010, .009, PAD_T, PAD_T, .0036, .0020), 'petg'))
        P.append(('pad', hexa(L - .0125, L - .0005, .0085, .0085, 0, 0, PAD_T, PAD_T), 'tpu'))
    return P


def tcp_offset(L):
    """Hand frame z of the grasp point: middle of the 10 mm pad zone in front of the shoulder."""
    return 0.0584 + L - SHOULDER_D + TCP_IN


def install(spec, L, prefix='panda/'):
    """Replace the stock finger geoms by the custom pieces (hook on left_finger, wedge on right_finger)."""
    PETG_RGBA, TPU_RGBA = [.95, .55, .12, 1], [.08, .08, .09, 1]
    for body_name, kind in (('left_finger', HOOK_ON_LEFT and 'hook' or 'wedge'), ('right_finger', HOOK_ON_LEFT and 'wedge' or 'hook')):
        b = spec.body(prefix + body_name)
        b.explicitinertial = False                     # mass/inertia from the PETG/TPU pieces
        for g in list(b.geoms):
            spec.delete(g)
        for nm, v, mat in finger_pieces(kind, L):
            mname = f'{kind}_{nm}_L{int(round(L * 1000))}'
            spec.add_mesh(name=mname, uservert=v.ravel().tolist())
            kw = dict(friction=[1.0, .01, .0002], density=TPU_RHO, rgba=TPU_RGBA) if mat == 'tpu' else \
                dict(friction=[.45, .005, .0001], density=PETG_RHO, rgba=PETG_RGBA)
            b.add_geom(name=f'{kind}_{nm}', type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mname, condim=4, priority=2,
                       solref=[.004, 1], solimp=[.95, .99, .001, .5, 2], **kw)


# ------------------------------------------------------------------ STL export (ASCII, no dependencies)
QUADS = [(0, 2, 6, 4), (1, 5, 7, 3), (0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6)]


def write_stl(path, pieces, scale=1000.):
    """Each hexahedron -> 12 outward triangles (mm). Pieces overlap only on shared faces (slicer unions them)."""
    lines = [f'solid {Path(path).stem}']
    for _, v, _ in pieces:
        v = v * scale
        c = v.mean(0)
        for q in QUADS:
            for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                a, b, cc = v[list(tri)]
                n = np.cross(b - a, cc - a)
                if np.linalg.norm(n) < 1e-12:
                    continue
                if np.dot(n, a - c) < 0:              # make outward
                    b, cc = cc, b
                    n = -n
                n = n / np.linalg.norm(n)
                lines.append(f' facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n  outer loop')
                for p in (a, b, cc):
                    lines.append(f'   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}')
                lines.append('  endloop\n endfacet')
    lines.append(f'endsolid {Path(path).stem}')
    Path(path).write_text('\n'.join(lines) + '\n')


def volume(v):
    from scipy.spatial import ConvexHull
    return ConvexHull(v).volume


def mass_report(L):
    out = {}
    for kind in ('hook', 'wedge'):
        m = 0.
        for _, v, mat in finger_pieces(kind, L):
            m += volume(v) * (TPU_RHO if mat == 'tpu' else PETG_RHO)
        out[kind] = m
    return out


def tip_deflection(L, F=35., E=2.1e9):
    """Cantilever tip deflection under the grip load at the pad (PETG E ~2.1 GPa).
    70 N continuous grip = force between the jaws, ~35 N... reported for both 35 N and 70 N by the caller.
    Section: width b = 20 mm, mean thickness of the beam."""
    b, t = .020, .0095
    I = b * t ** 3 / 12
    a = L - SHOULDER_D + TCP_IN               # load point from the root
    return F * a ** 3 / (3 * E * I)


if __name__ == '__main__':
    import json
    out = Path(__file__).resolve().parent / 'fingers'
    out.mkdir(exist_ok=True)
    rep = {}
    for L in (STOCK_L, 0.080, 0.107):
        for kind in ('hook', 'wedge'):
            write_stl(out / f'{kind}_L{int(round(L * 1000))}.stl', finger_pieces(kind, L))
        rep[f'{L * 1000:.0f}'] = dict(mass_g={k: round(1000 * v, 1) for k, v in mass_report(L).items()},
                                      tip_defl_mm_70N=round(1000 * tip_deflection(L, 70.), 3))
    print(json.dumps(rep, indent=1))
