"""Scenario DSL for adversarial testing of the dishwasher-loading robot (M1 docked cart + Franka, scripted Runner).

A scenario is a JSON object with optional groups; anything omitted takes the nominal value (= the scene the pipeline
was built and validated on). `validate()` rejects unknown keys, out-of-bounds values and physically impossible
combinations (interpenetrating objects, a cart bumper inside the cabinet, a plate density no real plate has, ...).
`PARAMS` is the single source of truth for bounds + documentation (rendered into the GPT prompt and GPT_ADVERSARY.md).

All lengths m, angles deg, masses kg, forces N. World frame: counter edge at x = 0 (counter is x < 0, depth 0.60 m),
+y to the right of the dishwasher when facing it, z up; counter top z = 0.90. Countertop dishwasher footprint
x in [-0.39, 0], y in [-0.215, 0.215]. Nominal plate centre (-0.15, 0.40); pod dish (-0.085, 0.655); robot arm base
(0.48, 0.42) on a cart in front of the counter.
"""
from __future__ import annotations

import copy
import json
import math

import numpy as np

# ---------------------------------------------------------------- constants (mirrors full_pipeline scene/dishwasher)
COUNTER_D = 0.60
COUNTER_Y = (-1.25, 1.25)
DW_D, DW_W, DW_H = 0.39, 0.43, 0.46
PLATE0 = (-0.15, 0.40)
DISH_C = (-0.085, 0.655)
DISH_R = 0.045
POD_SIZE = (0.032, 0.024, 0.013)
ARM_BASE = (0.48, 0.42)
PERSON_BEARING = (1.05 - 0.48, 1.00 - 0.42)          # nominal person direction from the arm base (0.81 m)
# door (dishwasher.door_spring): m 1.6 kg, COM (x -0.0159, z 0.176) from the hinge, k 1.3026 N m/rad, ref -0.0199 rad
DOOR_M, DOOR_XC, DOOR_ZC, DOOR_K, DOOR_REF = 1.6, -0.01595, 0.17602, 1.30259, -0.019914

CLUTTER_TYPES = {
    # type: (shape, dims at size_scale 1, mass kg, default rgb)  dims: cyl (r, h) | box (lx, ly, h)
    'mug': ('cyl', (0.042, 0.095), 0.33, (0.85, 0.30, 0.25)),
    'bowl': ('cyl', (0.075, 0.055), 0.35, (0.92, 0.92, 0.90)),
    'bottle': ('cyl', (0.035, 0.270), 0.80, (0.25, 0.55, 0.35)),
    'cereal_box': ('box', (0.190, 0.070, 0.280), 0.45, (0.90, 0.70, 0.20)),
    'cutting_board': ('box', (0.380, 0.250, 0.018), 0.90, (0.70, 0.52, 0.32)),
}

# name: (default, lo, hi, doc)   (scalars only; structured groups are documented in GROUP_DOCS)
PARAMS = {
    'plate.radius_m': (0.1015, 0.085, 0.135, 'outer radius (8" plate = 0.1015). The robot assumes 0.1015 everywhere'),
    'plate.mass_kg': (0.45, 0.25, 1.2, 'total mass; areal density mass/(pi r^2) must be 5-35 kg/m^2 (melamine..stoneware)'),
    'plate.mu': (0.6, 0.15, 1.0, 'dry Coulomb friction of the glaze (used for all plate contacts: counter, fingers, tines)'),
    'plate.wetness': (0.0, 0.0, 1.0, 'water film: effective mu = mu * (1 - 0.6 wetness), floor 0.1'),
    'plate.x': (PLATE0[0], -0.50, -0.02, 'start centre x (counter edge at 0); plate must lie fully inside the counter depth'),
    'plate.y': (PLATE0[1], 0.22, 0.85, 'start centre y (camera field of view, right of the dishwasher)'),
    'plate.rgb': ([0.96, 0.96, 0.93], 0.0, 1.0, 'glaze colour, 3 values 0-1 (counter is dark stone 0.27)'),
    'plate.upside_down': (False, None, None, 'true = plate lies face-down (rim on the counter)'),
    'pod.location': ('dish', None, None, '"dish" (in its dish, offset dx/dy) or "counter" (absolute x/y)'),
    'pod.dx': (0.0, -0.02, 0.02, 'pod offset inside the dish (location=dish)'),
    'pod.dy': (0.0, -0.02, 0.02, 'pod offset inside the dish (location=dish)'),
    'pod.x': (None, -0.55, -0.02, 'pod centre x on the counter (location=counter)'),
    'pod.y': (None, 0.22, 1.0, 'pod centre y on the counter (location=counter)'),
    'pod.yaw_deg': (0.0, -90.0, 90.0, 'pod yaw; nominal 0 = 32 mm side along world x'),
    'pod.size_scale': (1.0, 0.8, 1.3, 'tablet size scale (nominal 32x24x13 mm); groove is 45x35x12 mm'),
    'pod.mass_kg': (0.018, 0.010, 0.030, 'tablet mass; density must be 0.8-2.5 g/cm^3'),
    'dish.dx': (0.0, -0.08, 0.08, 'pod dish displaced from its taught spot (robot picks at the taught spot)'),
    'dish.dy': (0.0, -0.08, 0.08, ''),
    'door.hinge_friction_Nm': (0.30, 0.05, 3.0, 'hinge + gasket Coulomb friction'),
    'door.damping': (1.6, 0.3, 10.0, 'hinge viscous damping N m s/rad'),
    'door.spring_scale': (1.0, 0.6, 1.6, 'counterbalance spring stiffness scale (>1.33: door no longer rests on its open stop)'),
    'basket.rail_friction_N': (0.30, 0.05, 20.0, 'roller/rail Coulomb friction (food residue, bent rail)'),
    'basket.damping': (2.0, 0.5, 30.0, 'roller viscous drag N s/m'),
    'basket.preload_slots': ([], 0, 7, 'list of plate-slot indices 0..7 already holding an upright 8" plate (rigid with the basket). '
                                       'The robot always inserts into slot 6 (2nd from front, between tines 6 and 7)'),
    'lighting.key_scale': (1.0, 0.05, 2.0, 'main ceiling/window light intensity'),
    'lighting.ambient_scale': (1.0, 0.1, 2.0, 'ambient + camera-side fill'),
    'lighting.color_temp_K': (5500.0, 2200.0, 7500.0, 'light colour temperature (2200 = candle/sodium, 7500 = overcast sky)'),
    'lighting.key_azimuth_deg': (0.0, -180.0, 180.0, 'rotation of the key light about the vertical axis through the counter'),
    'camera_bump.roll_deg': (0.0, -3.0, 3.0, 'wall camera knocked: true orientation error (robot keeps the old calibration)'),
    'camera_bump.pitch_deg': (0.0, -3.0, 3.0, ''),
    'camera_bump.yaw_deg': (0.0, -3.0, 3.0, ''),
    'camera_bump.dx': (0.0, -0.02, 0.02, 'camera position error m'),
    'camera_bump.dy': (0.0, -0.02, 0.02, ''),
    'camera_bump.dz': (0.0, -0.02, 0.02, ''),
    'dock.dx': (0.0, 0.0, 0.03, 'cart docked short of the cabinet bumper stop (>= 0: the bumper cannot be inside the cabinet)'),
    'dock.dy': (0.0, -0.03, 0.03, 'cart lateral docking error'),
    'dock.yaw_deg': (0.0, -3.0, 3.0, 'cart yaw docking error (needs dx >= 0.30 |sin yaw|)'),
    'dishwasher.dx': (0.0, -0.05, 0.02, 'appliance pushed back (<0) or forward on the counter (robot keeps its calibrated model)'),
    'dishwasher.dy': (0.0, -0.05, 0.05, ''),
    'dishwasher.yaw_deg': (0.0, -4.0, 4.0, ''),
    'sensors.depth_noise_scale': (1.0, 0.5, 4.0, 'RGB-D depth noise multiplier (nominal sigma 1.5 mm per m range)'),
    'sensors.ft_noise_N': (0.0, 0.0, 0.5, 'wrist F/T white noise sigma per 2 ms sample (real sensor ~0.05-0.2 N)'),
    'sensors.ft_drift_N_per_min': (0.0, 0.0, 2.0, 'wrist F/T thermal drift after the start-up tare'),
    'person.at_s': (None, 0.0, 250.0, 'person appears (safety-scanner flag + visible figure) at this sim time; null = never'),
    'person.dur_s': (30.0, 3.0, 120.0, 'how long the person stays'),
    'person.distance_m': (0.81, 0.6, 2.0, 'distance from the arm base'),
    'buttons.threshold_N': (1.0, 0.3, 6.0, 'touch-pad actuation force'),
    'buttons.dwell_ms': (8.0, 2.0, 100.0, 'how long the force must stay above threshold (debounce)'),
}
GROUP_DOCS = {
    'clutter': 'list (max 4) of {"type": one of ' + '|'.join(CLUTTER_TYPES) + ', "surface": "counter"|"dishwasher_top", '
               '"x", "y" (counter: world m; dishwasher_top: offset from the appliance top centre, |x|<=0.15, |y|<=0.17), '
               '"yaw_deg", "size_scale" 0.8-1.25, optional "rgb"}. Free rigid bodies (they can be pushed / knocked over). '
               'Footprints must not overlap each other, the plate, the dish, the pod or the dishwasher. The robot does NOT '
               'perceive clutter (its collision model is the calibrated empty kitchen).',
}
KNOWN_FAULTS = [
    'door_latch: door stay does not catch, door springs back from 90 to ~68 deg (weak stay spring)',
    'pod_moved: pod not in its dish but elsewhere on the counter',
    'plate_random: plate starts at a reachable but un-validated spot (e.g. (-0.22,0.56)), outside the validated pick zone',
    'basket_jam: basket catches at 0.10 m of its 0.25 m travel',
    'button_miss: first press on the panel only wakes it up (QUICK never starts, Runner claims success)',
    'plate_slip: plate knocked out of the fingers during the tilt-up',
    'person: a person appears near the robot; the scripted Runner ignores the safety flag and moves at 0.32 m/s',
]


def defaults():
    out = {}
    for k, (dv, *_rest) in PARAMS.items():
        g, f = k.split('.')
        out.setdefault(g, {})[f] = copy.deepcopy(dv)
    out['clutter'] = []
    return out


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _footprints(s):
    """(name, x, y, r) circles of everything resting on the counter (conservative radii)."""
    fp = []
    p = s['plate']
    fp.append(('plate', p['x'], p['y'], p['radius_m']))
    dish = (DISH_C[0] + s['dish']['dx'], DISH_C[1] + s['dish']['dy'])
    fp.append(('dish', dish[0], dish[1], DISH_R + .003))
    po = s['pod']
    if po['location'] == 'counter':
        fp.append(('pod', po['x'], po['y'], .5 * math.hypot(POD_SIZE[0], POD_SIZE[1]) * po['size_scale']))
    for i, c in enumerate(s['clutter']):
        if c['surface'] != 'counter':
            continue
        shape, dims, _, _ = CLUTTER_TYPES[c['type']]
        k = c['size_scale']
        r = dims[0] * k if shape == 'cyl' else .5 * math.hypot(dims[0], dims[1]) * k
        fp.append((f'clutter{i}:{c["type"]}', c['x'], c['y'], r))
    return fp


def _dw_box(s):
    """Dishwasher footprint as an axis-aligned box (x0, x1, y0, y1), enlarged for its yaw."""
    d = s['dishwasher']
    a = math.radians(abs(d['yaw_deg']))
    ex = .5 * (DW_D * math.cos(a) + DW_W * math.sin(a))
    ey = .5 * (DW_W * math.cos(a) + DW_D * math.sin(a))
    cx, cy = -DW_D / 2 + d['dx'], d['dy']
    return cx - ex, cx + ex, cy - ey, cy + ey


def validate(scn):
    """-> (ok, errors[list of str], normalized scenario dict)."""
    errs = []
    if isinstance(scn, str):
        try:
            scn = json.loads(scn)
        except json.JSONDecodeError as e:
            return False, [f'not valid JSON: {e}'], None
    if not isinstance(scn, dict):
        return False, ['scenario must be a JSON object'], None
    s = defaults()
    for g, v in scn.items():
        if g in ('name', 'hypothesis', 'meta'):
            continue
        if g == 'clutter':
            if not isinstance(v, list):
                errs.append('clutter must be a list')
                continue
            if len(v) > 4:
                errs.append('clutter: at most 4 objects')
            for i, c in enumerate(v[:4]):
                if not isinstance(c, dict):
                    errs.append(f'clutter[{i}] must be an object')
                    continue
                unk = set(c) - {'type', 'surface', 'x', 'y', 'yaw_deg', 'size_scale', 'rgb'}
                if unk:
                    errs.append(f'clutter[{i}]: unknown keys {sorted(unk)}')
                cc = dict(type=c.get('type'), surface=c.get('surface', 'counter'), x=c.get('x'), y=c.get('y'),
                          yaw_deg=c.get('yaw_deg', 0.0), size_scale=c.get('size_scale', 1.0), rgb=c.get('rgb'))
                if cc['type'] not in CLUTTER_TYPES:
                    errs.append(f'clutter[{i}].type must be one of {list(CLUTTER_TYPES)}')
                    continue
                if cc['surface'] not in ('counter', 'dishwasher_top'):
                    errs.append(f'clutter[{i}].surface must be counter|dishwasher_top')
                    continue
                for f in ('x', 'y', 'yaw_deg', 'size_scale'):
                    if not _num(cc[f]):
                        errs.append(f'clutter[{i}].{f} must be a number')
                if any(not _num(cc[f]) for f in ('x', 'y', 'yaw_deg', 'size_scale')):
                    continue
                if not .8 <= cc['size_scale'] <= 1.25:
                    errs.append(f'clutter[{i}].size_scale {cc["size_scale"]} outside 0.8-1.25')
                if not -180 <= cc['yaw_deg'] <= 180:
                    errs.append(f'clutter[{i}].yaw_deg outside -180..180')
                if cc['rgb'] is None:
                    cc['rgb'] = list(CLUTTER_TYPES[cc['type']][3])
                elif not (isinstance(cc['rgb'], list) and len(cc['rgb']) == 3 and all(_num(x) and 0 <= x <= 1 for x in cc['rgb'])):
                    errs.append(f'clutter[{i}].rgb must be 3 numbers in 0-1')
                shape, dims, _, _ = CLUTTER_TYPES[cc['type']]
                k = cc['size_scale']
                r = dims[0] * k if shape == 'cyl' else .5 * math.hypot(dims[0], dims[1]) * k
                if cc['surface'] == 'counter':
                    if not (-COUNTER_D + r + .005 <= cc['x'] <= -r - .005 and COUNTER_Y[0] + r <= cc['y'] <= COUNTER_Y[1] - r):
                        errs.append(f'clutter[{i}] ({cc["type"]}) not fully on the counter (x in [{-COUNTER_D + r + .005:.3f}, {-r - .005:.3f}])')
                else:
                    if not (abs(cc['x']) <= .15 and abs(cc['y']) <= .17):
                        errs.append(f'clutter[{i}] on dishwasher_top must have |x|<=0.15, |y|<=0.17')
                    if cc['type'] == 'cutting_board' and k > 1.0:
                        errs.append(f'clutter[{i}] cutting board larger than the dishwasher top')
                s['clutter'].append(cc)
            continue
        if g == 'person' and v is None:
            continue
        if g not in s or not isinstance(v, dict):
            errs.append(f'unknown group "{g}"' if g not in s else f'group "{g}" must be an object')
            continue
        for f, val in v.items():
            key = f'{g}.{f}'
            if key not in PARAMS:
                errs.append(f'unknown parameter "{key}"')
                continue
            s[g][f] = val
    if errs:
        return False, errs, None
    # ---- scalar bounds
    for key, (dv, lo, hi, _) in PARAMS.items():
        g, f = key.split('.')
        v = s[g][f]
        if key == 'plate.rgb':
            if not (isinstance(v, list) and len(v) == 3 and all(_num(x) and 0 <= x <= 1 for x in v)):
                errs.append('plate.rgb must be 3 numbers in 0-1')
            continue
        if key == 'plate.upside_down':
            if not isinstance(v, bool):
                errs.append('plate.upside_down must be true/false')
            continue
        if key == 'pod.location':
            if v not in ('dish', 'counter'):
                errs.append('pod.location must be "dish" or "counter"')
            continue
        if key == 'basket.preload_slots':
            if not (isinstance(v, list) and all(isinstance(x, int) and not isinstance(x, bool) and 0 <= x <= 7 for x in v)
                    and len(set(v)) == len(v) and len(v) <= 7):
                errs.append('basket.preload_slots must be a list of distinct ints 0..7 (max 7)')
            continue
        if v is None:
            continue
        if not _num(v):
            errs.append(f'{key} must be a number')
            continue
        if not lo <= v <= hi:
            errs.append(f'{key} = {v} outside [{lo}, {hi}]')
    if errs:
        return False, errs, None
    p, po = s['plate'], s['pod']
    # ---- physical consistency
    dens = p['mass_kg'] / (math.pi * p['radius_m'] ** 2)
    if not 5 <= dens <= 35:
        errs.append(f'plate areal density {dens:.1f} kg/m^2 is not a real plate (5-35): mass {p["mass_kg"]} for radius {p["radius_m"]}')
    if p['x'] - p['radius_m'] < -COUNTER_D + .005:
        errs.append(f'plate hangs off the back of the counter (x - r = {p["x"] - p["radius_m"]:.3f} < -0.595)')
    vol = np.prod(POD_SIZE) * po['size_scale'] ** 3 * 1e6
    pd = po['mass_kg'] * 1000 / vol
    if not .8 <= pd <= 2.5:
        errs.append(f'pod density {pd:.2f} g/cm^3 not physical for a detergent tablet (0.8-2.5)')
    if po['location'] == 'counter':
        if po['x'] is None or po['y'] is None:
            errs.append('pod.location=counter needs pod.x and pod.y')
    else:
        half = .5 * math.hypot(POD_SIZE[0], POD_SIZE[1]) * po['size_scale']
        if math.hypot(po['dx'], po['dy']) + half > DISH_R - .003:
            errs.append(f'pod (half-diagonal {half*1000:.0f} mm) offset {math.hypot(po["dx"], po["dy"])*1000:.0f} mm does not fit inside '
                        f'the dish (inner radius {(DISH_R - .003)*1000:.0f} mm)')
    dk = s['dock']
    if dk['dx'] < .30 * abs(math.sin(math.radians(dk['yaw_deg']))) - 1e-9:
        errs.append(f'dock yaw {dk["yaw_deg"]} deg with dx {dk["dx"]} m puts a bumper corner inside the cabinet (need dx >= '
                    f'{.30 * abs(math.sin(math.radians(dk["yaw_deg"]))):.4f})')
    if errs:
        return False, errs, None
    if po['location'] == 'counter':
        if not (-COUNTER_D + .02 <= po['x'] <= -.02):
            errs.append('pod not on the counter')
    fps = _footprints(s)
    x0, x1, y0, y1 = _dw_box(s)
    for i, (na, xa, ya, ra) in enumerate(fps):
        if (x0 - ra < xa < x1 + ra) and (y0 - ra < ya < y1 + ra):
            errs.append(f'{na} at ({xa:.3f},{ya:.3f}) overlaps the dishwasher footprint')
        for nb, xb, yb, rb in fps[i + 1:]:
            if math.hypot(xa - xb, ya - yb) < ra + rb + .003:
                errs.append(f'{na} and {nb} overlap (centres {math.hypot(xa - xb, ya - yb)*1000:.0f} mm apart, need {(ra + rb + .003)*1000:.0f})')
    dish = (DISH_C[0] + s['dish']['dx'], DISH_C[1] + s['dish']['dy'])
    if not (-COUNTER_D + DISH_R <= dish[0] <= -DISH_R):
        errs.append('dish not on the counter')
    if s['person']['at_s'] is None:
        s['person'] = None
    return (not errs), errs, (s if not errs else None)


def diff_from_nominal(s):
    """Changed parameters only (flat dict) - used for prompts, clustering and minimisation."""
    d0 = defaults()
    out = {}
    for g, v in s.items():
        if g == 'clutter':
            if v:
                out['clutter'] = v
            continue
        if g == 'person':
            if v:
                out.update({f'person.{k}': x for k, x in v.items()})
            continue
        for f, x in v.items():
            if x != d0[g][f]:
                out[f'{g}.{f}'] = x
    return out


GROUPS = ['plate.geom', 'plate.surface', 'plate.pose', 'plate.look', 'pod', 'dish', 'door', 'basket.rail', 'basket.preload',
          'clutter', 'lighting', 'camera_bump', 'dock', 'dishwasher', 'sensors.depth', 'sensors.ft', 'person', 'buttons']
GROUP_KEYS = {'plate.geom': ['plate.radius_m', 'plate.mass_kg'], 'plate.surface': ['plate.mu', 'plate.wetness'],
              'plate.pose': ['plate.x', 'plate.y', 'plate.upside_down'], 'plate.look': ['plate.rgb'],
              'basket.rail': ['basket.rail_friction_N', 'basket.damping'], 'basket.preload': ['basket.preload_slots'],
              'sensors.depth': ['sensors.depth_noise_scale'], 'sensors.ft': ['sensors.ft_noise_N', 'sensors.ft_drift_N_per_min']}


def group_of(key):
    for g, ks in GROUP_KEYS.items():
        if key in ks:
            return g
    return key.split('.')[0]


def changed_groups(s):
    return sorted({group_of(k) for k in diff_from_nominal(s)})


def revert_groups(s, groups):
    """Copy of s with the given parameter groups set back to nominal."""
    s2 = copy.deepcopy(s)
    d0 = defaults()
    for g in groups:
        if g == 'clutter':
            s2['clutter'] = []
        elif g == 'person':
            s2['person'] = None
        elif g in GROUP_KEYS:
            for k in GROUP_KEYS[g]:
                a, b = k.split('.')
                s2[a][b] = copy.deepcopy(d0[a][b])
        else:
            s2[g] = copy.deepcopy(d0[g])
    return s2


def to_input(s):
    """Normalized scenario -> compact input JSON (only non-nominal fields), re-validatable."""
    d0 = defaults()
    out = {}
    for g, v in s.items():
        if g == 'clutter':
            if v:
                out['clutter'] = v
            continue
        if g == 'person':
            if v:
                out['person'] = dict(v)
            continue
        ch = {f: x for f, x in v.items() if x != d0[g][f]}
        if g == 'pod' and ch:
            ch['location'] = v['location']
        if ch:
            out[g] = ch
    return out


# ---------------------------------------------------------------- uniform random baseline
def sample_uniform(rng, max_tries=500):
    """Every parameter drawn uniformly within its bounds (booleans / categories uniformly; person present with p=0.5;
    0-4 clutter objects; each basket slot preloaded with p=0.25), rejection-sampled until physically valid.
    Returns (scenario_input, tries)."""
    for t in range(1, max_tries + 1):
        sc = {}
        for key, (dv, lo, hi, _) in PARAMS.items():
            g, f = key.split('.')
            if key == 'plate.rgb':
                v = [round(float(x), 3) for x in rng.uniform(0, 1, 3)]
            elif key == 'plate.upside_down':
                v = bool(rng.random() < .5)
            elif key == 'pod.location':
                v = 'dish' if rng.random() < .5 else 'counter'
            elif key == 'basket.preload_slots':
                v = [int(i) for i in range(8) if rng.random() < .25]
            elif key.startswith('person.'):
                continue
            else:
                v = round(float(rng.uniform(lo, hi)), 4)
            sc.setdefault(g, {})[f] = v
        if sc['pod']['location'] == 'dish':
            sc['pod'].pop('x'), sc['pod'].pop('y')
        if rng.random() < .5:
            sc['person'] = {f: round(float(rng.uniform(PARAMS[f'person.{f}'][1], PARAMS[f'person.{f}'][2])), 2)
                            for f in ('at_s', 'dur_s', 'distance_m')}
        cl = []
        for _ in range(int(rng.integers(0, 5))):
            ty = str(rng.choice(list(CLUTTER_TYPES)))
            if rng.random() < .8:
                c = dict(type=ty, surface='counter', x=round(float(rng.uniform(-.55, -.03)), 3), y=round(float(rng.uniform(.22, 1.0)), 3))
            else:
                c = dict(type=ty, surface='dishwasher_top', x=round(float(rng.uniform(-.15, .15)), 3), y=round(float(rng.uniform(-.17, .17)), 3))
            c.update(yaw_deg=round(float(rng.uniform(-180, 180)), 1), size_scale=round(float(rng.uniform(.8, 1.25)), 3))
            cl.append(c)
        sc['clutter'] = cl
        ok, errs, norm = validate(sc)
        if ok:
            return to_input(norm), t
    raise RuntimeError('no valid uniform sample')


SPARSE_GROUPS = ['plate', 'pod', 'dish', 'door', 'basket', 'clutter', 'lighting', 'camera_bump', 'dock', 'dishwasher', 'sensors',
                 'person', 'buttons']


def sample_sparse(rng, max_tries=500):
    """Random baseline that perturbs only 1-3 top-level groups (uniformly chosen), each drawn as in sample_uniform."""
    for t in range(1, max_tries + 1):
        full, _ = sample_uniform(rng)
        k = int(rng.integers(1, 4))
        gs = list(rng.choice(SPARSE_GROUPS, size=k, replace=False))
        if 'person' in gs and 'person' not in full:
            full['person'] = {f: round(float(rng.uniform(PARAMS[f'person.{f}'][1], PARAMS[f'person.{f}'][2])), 2)
                              for f in ('at_s', 'dur_s', 'distance_m')}
        if 'clutter' in gs and not full.get('clutter'):
            continue
        sc = {g: v for g, v in full.items() if g in gs}
        ok, errs, norm = validate(sc)
        if ok and diff_from_nominal(norm):
            return to_input(norm), t
    raise RuntimeError('no valid sparse sample')


def prompt_doc():
    lines = ['Scenario JSON = object with optional groups; omitted fields keep the nominal value. Scalar parameters '
             '(group.field: nominal [min, max] - meaning):']
    for k, (dv, lo, hi, doc) in PARAMS.items():
        rng_ = f'[{lo}, {hi}]' if lo is not None else ''
        lines.append(f'  {k}: {json.dumps(dv)} {rng_} - {doc}')
    for g, doc in GROUP_DOCS.items():
        lines.append(f'  {g}: {doc}')
    lines.append('  person: omit (never) or {"at_s", "dur_s", "distance_m"}')
    lines.append('Physical consistency is enforced: no interpenetrating objects (plate, dish, pod on counter, clutter, dishwasher), '
                 'plate areal density 5-35 kg/m^2, pod density 0.8-2.5 g/cm^3, pod must fit inside its dish, the cart bumper '
                 'cannot be inside the cabinet. Invalid scenarios are rejected (not run) and count against you.')
    return '\n'.join(lines)


if __name__ == '__main__':
    print(prompt_doc())
    rng = np.random.default_rng(0)
    tr = []
    for _ in range(20):
        s, t = sample_uniform(rng)
        tr.append(t)
    print('uniform sampler tries per valid sample:', tr)
    print(validate({'plate': {'radius_m': .13, 'mass_kg': .3}}))
    print(validate({'dock': {'yaw_deg': 2}}))
    print(validate({'clutter': [{'type': 'mug', 'x': -.15, 'y': .40}]}))
