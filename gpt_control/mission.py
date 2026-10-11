"""Scenario definitions shared by the GPT agent, the scripted check and the hard-coded baseline."""
from __future__ import annotations

import math

import numpy as np

import sim_robot as SR

GOALS = {
    'zh': '把台面上的盘子放进洗碗机，放好洗碗块，关门，快洗模式启动',
    'en': 'Put the plate from the counter into the dishwasher, put the detergent pod in, close the door and start the QUICK program.',
}

# plate starts outside the pick zone (offsets from the nominal start (-0.15, 0.40)); reachable, visible, clear of the dish
PLATE_RANDOM = [(-0.07, 0.16, 25.), (-0.10, 0.12, -15.), (-0.04, 0.13, 5.)]   # -> (-0.22,0.56) (-0.25,0.52) (-0.19,0.53)


def scenario(name, seed):
    """-> (faults dict, plate_off (dx, dy, yaw_rad))."""
    rng = np.random.default_rng(seed)
    small = rng.uniform([-.01, -.01, -math.radians(10)], [.01, .01, math.radians(10)])
    if name == 'plate_random':
        k = seed % len(PLATE_RANDOM)
        dx, dy, yaw = PLATE_RANDOM[k]
        return {}, (dx, dy, math.radians(yaw))
    if name not in SR.FAULTS:
        raise KeyError(name)
    return dict(SR.FAULTS[name]), tuple(small)


def make_robot(name, seed, video=None, run_tag='', title='', log=print):
    faults, off = scenario(name, seed)
    r = SR.SimRobot(faults=faults, plate_off=off, video=video, seed=seed, run_tag=run_tag, title=title, log=log)
    r.start()
    return r, faults, off
