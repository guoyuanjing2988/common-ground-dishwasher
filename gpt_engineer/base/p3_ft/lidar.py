"""Simulated 2-D lidars (mj_multiRay fans from the base's two corner lidar sites) and the dock localizer.

Choice: lidar line-fit on the cabinet fronts + a 50 mm retro-reflective strip (spec.json `docking`), not a fiducial
camera:
  * the cabinet front is a long straight line the planar lidars see from anywhere along the counter run, so the same
    sensors give x (distance to the cabinets) and heading continuously while driving, not only when a tag is in view;
  * range geometry does not depend on kitchen lighting / glare, and a reflective strip is a cheap, flush fixture
    (nothing for the robot to bump into);
  * y along the counter comes from the strip's two edges, found from the high-intensity returns (ray angles, so range
    noise does not bias them); mm-level at docking range. The final mm / 0.1 deg comes from the V-blocks (mechanical).
Sensor model (EST, RPLIDAR S2 class): fan +-70 deg out of each corner cut-out, 0.25 deg resolution (561 rays; the S2's
0.12 deg is subsampled), 10 Hz, range noise sigma 10 mm, intensity = "strip / not strip", max range 4 m, rays ignore
the robot's own chassis. NOT modelled: the S2's systematic range error (spec +-30 mm), mixed pixels, motion skew.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

import scene_mobile as SM

RES, FOV, SIGMA, MAX_RANGE, MIN_RANGE = math.radians(.25), SM.LIDAR_FOV, .010, 4.0, .05
N_RAYS = int(round(2 * FOV / RES)) + 1


class Lidar:
    def __init__(self, m, which):
        self.m = m
        self.which = which
        self.site = m.site(f'lidar_{which}').id
        self.base = m.body('base').id
        self.ang = np.linspace(-FOV, FOV, N_RAYS)
        self.local = np.stack([np.cos(self.ang), np.sin(self.ang), np.zeros(N_RAYS)], 1)
        self.geomid = np.zeros(N_RAYS, np.int32)
        self.dist = np.zeros(N_RAYS)
        self.group = np.array([1, 1, 1, 1, 1, 1], np.uint8)
        self.marker = m.geom('dock_marker').id
        p, a = SM.LIDARS[which]
        self.mount = (np.array(p[:2]), a)            # pose of the lidar in the base frame

    def scan(self, d, rng):
        R = d.site_xmat[self.site].reshape(3, 3)
        vec = (self.local @ R.T).ravel()
        mujoco.mj_multiRay(self.m, d, d.site_xpos[self.site].copy(), vec, self.group, 1, self.base, self.geomid, self.dist,
                           None, N_RAYS, MAX_RANGE)
        r = self.dist.copy()
        ok = (r > MIN_RANGE) & (r < MAX_RANGE)
        r[ok] += rng.normal(0, SIGMA, ok.sum())
        r[~ok] = np.nan
        return r, (self.geomid == self.marker) & ok


def _tls_line(P):
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    t = vt[0]
    return c, t, np.array([-t[1], t[0]])


def localize(ranges, strip, mount, ang=None, rng=None):
    """Base pose (x, y, heading) in the world from one scan of a lidar mounted at `mount` = (xy, yaw) in the base frame.
    x and heading from the cabinet-front line, y from the reflective strip (if seen). Returns a dict (ok, x, th, y?)."""
    ang = np.linspace(-FOV, FOV, N_RAYS) if ang is None else ang
    ok = np.isfinite(ranges) & (ranges < 3.0)
    if ok.sum() < 20:
        return dict(ok=False, why='too few returns')
    P = np.stack([ranges * np.cos(ang), ranges * np.sin(ang)], 1)
    idx = np.nonzero(ok)[0]
    Q = P[idx]
    rng = rng or np.random.default_rng(0)
    best, bn = None, 0
    for _ in range(80):
        i, j = rng.choice(len(Q), 2, replace=False)
        t = Q[j] - Q[i]
        nt = np.linalg.norm(t)
        if nt < .05:
            continue
        t /= nt
        n = np.array([-t[1], t[0]])
        inl = np.abs((Q - Q[i]) @ n) < .03
        if inl.sum() > bn and np.ptp((Q[inl] - Q[i]) @ t) > .15:
            best, bn = inl, inl.sum()
    if best is None or bn < 25:
        return dict(ok=False, why='no line')
    c, t, n = _tls_line(Q[best])
    for _ in range(2):
        inl = np.abs((Q - c) @ n) < .025
        c, t, n = _tls_line(Q[inl])
    if n @ (-c) < 0:                                     # normal from the wall toward the lidar
        n = -n
    rho = abs(float(c @ n))
    span = float(np.ptp((Q[inl] - c) @ t))
    if span < .12:
        return dict(ok=False, why='line too short')
    psi = -math.atan2(n[1], n[0])                        # lidar heading in the world: R(psi) n = +x (cabinet normal)
    out = dict(ok=True, n_line=int(inl.sum()), span=round(span, 3), rho=round(rho, 4), marker=False)
    # strip: contiguous run of high-intensity returns; edges half-way to the neighbouring rays, on the fitted line
    si = np.nonzero(strip)[0]
    if len(si) >= 3:
        runs = np.split(si, np.nonzero(np.diff(si) > 1)[0] + 1)
        g = max(runs, key=len)
        i0, i1 = g[0], g[-1]
        a_lo = .5 * (ang[i0] + ang[max(i0 - 1, 0)])
        a_hi = .5 * (ang[i1] + ang[min(i1 + 1, len(ang) - 1)])

        def on_line(a):
            ua = np.array([math.cos(a), math.sin(a)])
            r_ = (c @ n) / (ua @ n)
            return float((r_ * ua - c) @ t)
        u = np.array([on_line(a_lo), on_line(a_hi)])
        width = float(np.ptp(u))
        if len(g) >= 3 and .6 * SM.MARKER_W < width < 1.5 * SM.MARKER_W:
            pm = c + u.mean() * t                         # strip centre, lidar frame
            cp, sp = math.cos(psi), math.sin(psi)
            pm_w = np.array([cp * pm[0] - sp * pm[1], sp * pm[0] + cp * pm[1]])
            out.update(marker=True, marker_rays=len(g), marker_width=round(width, 4), ly=SM.MARKER_Y - pm_w[1])
    lxy_b, lyaw = mount
    th = psi - lyaw                                      # base heading
    c_, s_ = math.cos(th), math.sin(th)
    off = np.array([c_ * lxy_b[0] - s_ * lxy_b[1], s_ * lxy_b[0] + c_ * lxy_b[1]])
    out['x'] = SM.CAB_X + rho - off[0]
    out['th'] = math.atan2(math.sin(th), math.cos(th))
    if out['marker']:
        out['y'] = out['ly'] - off[1]
    return out
