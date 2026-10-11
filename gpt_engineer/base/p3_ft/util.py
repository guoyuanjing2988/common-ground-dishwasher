"""Small shared helpers: rotations, renderer with the macOS CGL offline fallback (as plate_poc.py)."""
from __future__ import annotations

import ctypes
import math
import sys

import mujoco
import numpy as np


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot_axis(axis, ang):
    q = np.empty(4)
    mujoco.mju_axisAngle2Quat(q, unit(axis), ang)
    M = np.empty(9)
    mujoco.mju_quat2Mat(M, q)
    return M.reshape(3, 3)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def angle_deg(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def hand_frame(approach, close):
    """Hand rotation: z = approach (palm -> fingertips), y = finger closing axis."""
    z = unit(approach)
    y = unit(np.asarray(close, float) - np.dot(close, z) * z)
    return np.column_stack([np.cross(y, z), y, z])


def renderer(model, h, w):
    try:
        return mujoco.Renderer(model, height=h, width=w)
    except Exception:
        if sys.platform != 'darwin':
            raise
        from mujoco.cgl import cgl, GLContext
        from mujoco.rendering.classic import gl_context

        class Offline(GLContext):
            def __init__(self, w_, h_):
                self._pix, self._context = cgl.CGLPixelFormatObj(), cgl.CGLContextObj()
                attrs = (ctypes.c_int * 2)(cgl.CGLPixelFormatAttribute.CGLPFAAllowOfflineRenderers, 0)
                cnt = cgl.GLint()
                cgl.CGLChoosePixelFormat(attrs, ctypes.byref(self._pix), ctypes.byref(cnt))
                cgl.CGLCreateContext(self._pix, 0, ctypes.byref(self._context))
        orig = gl_context.GLContext
        try:
            gl_context.GLContext = Offline
            return mujoco.Renderer(model, height=h, width=w)
        finally:
            gl_context.GLContext = orig


def scene_option(hide_sites=True):
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    if hide_sites:
        for i in range(6):
            opt.sitegroup[i] = 0
    return opt


def snapshot(model, data, path, lookat, distance, azimuth, elevation, w=1280, h=720):
    from PIL import Image
    r = renderer(model, h, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    r.update_scene(data, camera=cam, scene_option=scene_option())
    Image.fromarray(r.render()).save(path)
    r.close()
