"""Offscreen rendering helper. Tries mujoco.Renderer first; on macOS falls back to a CGL
context with CGLPFAAllowOfflineRenderers (copied from ../poc_plate/plate_poc.py, class Recorder)."""
from __future__ import annotations

import ctypes
import sys

import mujoco
import numpy as np


def make_renderer(model, width=1280, height=720):
    try:
        return mujoco.Renderer(model, height=height, width=width)
    except Exception:
        if sys.platform != 'darwin':
            raise
        from mujoco.cgl import cgl, GLContext
        from mujoco.rendering.classic import gl_context

        class Offline(GLContext):
            def __init__(self, w, h):
                self._pix, self._context = cgl.CGLPixelFormatObj(), cgl.CGLContextObj()
                attrs = (ctypes.c_int * 2)(cgl.CGLPixelFormatAttribute.CGLPFAAllowOfflineRenderers, 0)
                cnt = cgl.GLint()
                cgl.CGLChoosePixelFormat(attrs, ctypes.byref(self._pix), ctypes.byref(cnt))
                cgl.CGLCreateContext(self._pix, 0, ctypes.byref(self._context))
        orig = gl_context.GLContext
        try:
            gl_context.GLContext = Offline
            return mujoco.Renderer(model, height=height, width=width)
        finally:
            gl_context.GLContext = orig


def render(renderer, data, lookat, distance, azimuth, elevation, shadows=True, hide_groups=(3,), sites=False):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    opt = mujoco.MjvOption()
    for g in hide_groups:
        opt.geomgroup[g] = 0
    if not sites:
        opt.sitegroup[:] = 0
    renderer.update_scene(data, camera=cam, scene_option=opt)
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = int(shadows)
    return np.asarray(renderer.render())
