"""Robot API: the typed interface GPT calls. One call = one JSON-schema'd action -> one structured result.

Principle: *the interface enforces safety*. Every call is validated here before anything moves:
  1. action exists and is enabled at this control level           -> INVALID_ACTION
  2. arguments match the schema (types, ranges, required, no extras) -> INVALID_ARGS
  3. safety gate: person near the robot without a <=0.25 m/s cap, speed / force caps, TCP workspace box
                                                                    -> REJECTED_SAFETY  (counted as 'unsafe blocked')
  4. task preconditions (door > 80 deg before the basket, plate in hand before insert, ...)
                                                                    -> REJECTED_PRECONDITION
  5. L2 motions: joint-path existence and a kinematic collision check against the calibrated kitchen +
     dishwasher model                                               -> NO_IK_PATH / COLLISION_PREDICTED (blocked)
Only then the controller runs (500 Hz physics / 100 Hz IK in sim; 1 kHz FCI on the real FR3). GPT never sends
joint torques or joint positions. Every result carries ok / code / message / measurements / suggested causes and a
fresh state snapshot. Ground-truth 'scoring' numbers are split off and never shown to GPT.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

import sim_robot as SR
from sim_robot import PL, S, dw, plan, StepFail

ROOT = Path(__file__).resolve().parent
COUNTER_ZONE = dict(x=(-0.40, -0.10), y=(0.30, 0.85))     # flat plate may be slid within this area (camera view, reach)
PICK_ZONE = dict(x=(-0.26, -0.12), y=(0.32, 0.49))        # pick_plate: reachable (zone_scan.py) and the edge push clears the pod dish
WORKSPACE = dict(x=(-0.55, 0.75), y=(-0.40, 1.00), z=(0.88, 1.55))   # z floor over the counter
Z_FLOOR_FRONT = 0.80                                     # z floor in front of the counter edge (x > 0.02)
V_NOMINAL, V_PERSON = 0.50, 0.25                         # m/s TCP caps (SYSTEM.md: <= 0.25 m/s near people)
F_MAX = 30.                                              # N, contact force cap (SYSTEM.md: ~2x largest task force)

# ---------------------------------------------------------------- tool registry
# arg spec: name -> dict(type=number|integer|boolean|string|vec2|vec3, min, max, enum, default, required, desc)
T = {}


def tool(name, component, levels, desc, args=None, motion=False, ros='', computer='', real=''):
    T[name] = dict(name=name, component=component, levels=set(levels), desc=desc, args=args or {}, motion=motion,
                   ros=ros, computer=computer, real=real)


L1, L2 = 'L1', 'L2'
ALL = (L1, L2, 'L2v0')
L2S = (L2, 'L2v0')
V = lambda t, **k: dict(type=t, **k)  # noqa: E731

tool('robot.get_state', 'robot', ALL, 'Fresh state snapshot (arm, gripper, wrist load, dishwasher sensors, perception memory, safety).',
     ros='service /robot/get_state (robot_state_aggregator)', computer='Main PC', real='aggregates FR3 state, Bota F/T, wall node, safety PLC status')
tool('robot.wait', 'robot', ALL, 'Hold still for a while (e.g. until a person leaves). Arm holds its pose.',
     dict(seconds=V('number', min=.5, max=60., required=True)), ros='BT Wait node (no ROS call)', computer='Main PC', real='behaviour tree')
tool('robot.set_speed_limit', 'robot', ALL, 'Cap the TCP speed for all following motions (m/s). Use <=0.25 when a person is near. '
     'Skills are time-scaled to respect it.', dict(mps=V('number', min=.05, max=V_NOMINAL, required=True)),
     ros='service /safety/set_speed_limit -> FR3 safe-limited-speed config', computer='Main PC -> FR3 Control', real='FR3 SLS (safety function)')
tool('vision.capture', 'vision', ALL, 'Take a wall-camera RGB image; it is attached to your next turn.',
     ros='service /wall_node/capture (sensor_msgs/Image)', computer='Wall node (Jetson Orin Nano + D435i)', real='RealSense D435i')
tool('vision.locate', 'vision', ALL, "Locate 'plate' (lying flat on the counter) or 'pod' (detergent tablet) with the wall RGB-D camera: "
     'Codex 2-D detection + depth geometry -> metric pose + confidence. Arm must be stowed (camera view clear). ~10-20 s.',
     dict(object=V('string', enum=['plate', 'pod'], required=True)),
     ros='action /wall_node/locate (custom LocateObject.action)', computer='Wall node: codex exec + depth fit; on-device fallback detector offline',
     real='D435i + Codex VLM')
tool('dishwasher.read_sensors', 'dishwasher', ALL, 'Door angle (deg, 0 closed, 90 fully open), basket travel (m, 0 in, 0.25 out), '
     'panel display/program/running. SIMULATED SENSOR ACCESS (truth + noise); real system: wall-camera vision / appliance API.',
     ros='service /appliance/state', computer='Wall node (vision) or appliance cloud API', real='camera estimate of door / basket / panel LEDs')
tool('arm.get_tcp', 'arm', ALL, 'TCP (finger pad tip centre) position and hand axes: approach (palm->fingertips) and closing (finger closing axis).',
     ros='topic /franka_robot_state_broadcaster/current_pose', computer='Main PC (libfranka state @1 kHz)', real='FR3 joint encoders')
tool('arm.get_ft', 'arm', ALL, 'Wrist force (N, world frame) minus the calibrated hand weight: +z = carrying a load, -z = pressing down.',
     ros='topic /bota/wrench (geometry_msgs/WrenchStamped)', computer='Main PC (Bota SensONE 800 Hz)', real='Bota SensONE F/T')
tool('arm.stow', 'arm', ALL, 'Move to the camera-clear stow pose (hand down above the counter front, right of the dishwasher). '
     'Refused while holding the plate.', motion=True,
     ros='action /arm/stow (FollowJointTrajectory to a named pose)', computer='Main PC (planner 100 Hz) -> FR3 Control 1 kHz', real='FR3')
# ---- skills (L1)
SK = dict(ros_prefix='action /skills/', computer='Main PC: behaviour-tree subtree (IK 100 Hz, FCI impedance/admittance 1 kHz) -> FR3 Control')
tool('skills.perceive_plate', 'skills', (L1,), 'Stow the arm, then vision.locate("plate"). (Pipeline step 1.)', motion=True,
     ros='action /skills/perceive_plate', computer='Main PC + wall node', real='FR3 + D435i + Codex')
tool('skills.open_door', 'skills', (L1,), 'Grasp the door pull bar at its current angle, follow the hinge arc to the 90 deg stop, hold it '
     'against the stop for hold_at_stop_s, release, back off. Verdict = door angle after release (>85 deg).', motion=True,
     args=dict(hold_at_stop_s=V('number', min=.1, max=6., default=.5)), ros='action /skills/open_door', computer=SK['computer'], real='FR3 + Franka Hand')
tool('skills.load_pod', 'skills', (L1,), 'Pick the detergent pod (default: from its dish; or at pick_xy with yaw_deg = long-side direction '
     'from vision.locate) and drop it into the door groove. Needs the door open > 80 deg and the basket in. Verdict = finger width at lift (pod in hand).',
     motion=True, args=dict(pick_xy=V('vec2'), yaw_deg=V('number', min=-180, max=180)), ros='action /skills/load_pod', computer=SK['computer'], real='FR3 + Hand')
tool('skills.pull_basket', 'skills', (L1,), 'Grasp the lower-basket pull bar and pull it out to the 0.25 m end stop, with a wrist-force guard '
     '(stops if resistance > 25 N). Needs door > 80 deg.', motion=True, ros='action /skills/pull_basket', computer=SK['computer'], real='FR3 + Hand + Bota')
tool('skills.slide_plate', 'skills', (L1,), 'Slide the flat plate across the counter to to_xy (plate centre, m) with the closed fingers on its '
     'far rim (8 N press). Needs a fresh plate perception. Leaves the perception stale.', motion=True,
     args=dict(to_xy=V('vec2', required=True)), ros='action /skills/slide_plate', computer=SK['computer'], real='FR3 + Bota admittance')
tool('skills.pick_plate', 'skills', (L1,), 'Pick the flat plate: push it to overhang the counter edge (8 N admittance), re-perceive (Codex), '
     'pinch the rim, tilt it up about its far rim, lift. Needs a fresh plate perception with the centre inside the pick zone, and the '
     'destination ready (door > 80 deg, basket out > 0.24 m). Verdict = finger width + wrist load ~4.4 N.', motion=True,
     ros='action /skills/pick_plate', computer=SK['computer'] + ' + wall node', real='FR3 + Hand + Bota + D435i/Codex')
tool('skills.insert_plate', 'skills', (L1,), 'Carry the held plate into the lower-basket plate slot, lower until the wrist F/T feels touchdown, '
     'release, retract. Needs the plate in hand, door > 80 deg, basket out.', motion=True,
     args=dict(slot=V('integer', min=1, max=1, default=1)), ros='action /skills/insert_plate', computer=SK['computer'], real='FR3 + Bota')
tool('skills.push_basket', 'skills', (L1,), 'Grasp the basket bar and push the basket fully in. Needs an empty hand.', motion=True,
     ros='action /skills/push_basket', computer=SK['computer'], real='FR3 + Hand')
tool('skills.close_door', 'skills', (L1,), 'Re-grasp the bar under the open door and push the door up along the hinge arc to closed '
     '(|angle| < 1 deg). Needs the basket in (< 0.01 m) and an empty hand.', motion=True,
     ros='action /skills/close_door', computer=SK['computer'], real='FR3 + Hand')
tool('skills.press_button', 'skills', (L1,), 'Press one control-panel pad until the wrist F/T reads 3 N, hold 0.5 s, retract. Returns the panel '
     'display afterwards. Pads: eco, intensive, quick, dry, fruit, power. (Program pads select; POWER starts the selected program.)',
     motion=True, args=dict(button=V('string', enum=list(dw.BUTTONS), required=True)), ros='action /skills/press_button',
     computer=SK['computer'], real='FR3 + Bota')
# ---- L2 primitives
POSE = dict(xyz=V('vec3', required=True, desc='target TCP position (world m), or an offset if frame=relative'),
            approach=V('vec3', desc='hand approach direction (palm->fingertips), world; omit to keep the current attitude'),
            closing=V('vec3', desc='finger closing axis, world; omit to keep / derive'),
            frame=V('string', enum=['world', 'relative'], default='world'),
            speed=V('number', min=.01, max=V_NOMINAL, default=.15, desc='peak TCP speed m/s'))
tool('arm.move_tcp', 'arm', L2S, 'Straight-line TCP move (position + optional new attitude). Checked for reach, joint path and collision '
     'with the calibrated kitchen/dishwasher model before moving.', POSE, motion=True,
     ros='action /arm/move_tcp (custom MoveTcp.action: PoseStamped goal, max speed)', computer='Main PC: IK 100 Hz + Cartesian impedance 1 kHz -> FR3 Control', real='FR3')
tool('arm.move_to_pose', 'arm', ('L2',), 'Free-space transfer to a TCP pose: IK at the goal, joint-space path (direct or via a safe '
     'configuration), collision-checked. The TCP path is NOT a straight line - use it to re-orient or travel in free space, not near contact.',
     dict(xyz=V('vec3', required=True), approach=V('vec3', required=True), closing=V('vec3', required=True),
          speed=V('number', min=.01, max=V_NOMINAL, default=.15)), motion=True,
     ros='action /arm/move_to_pose (MoveIt-style plan + execute)', computer='Main PC: IK + joint-path planner -> FR3 Control', real='FR3')
tool('arm.move_until_contact', 'arm', L2S, 'Guarded straight move along direction until the wrist F/T resistance exceeds force_N or max_dist is '
     'travelled. Fingers may touch things; links are collision-checked.', motion=True,
     args=dict(direction=V('vec3', required=True), force_N=V('number', min=1., max=F_MAX, required=True),
               max_dist=V('number', min=.003, max=.20, required=True), speed=V('number', min=.005, max=.05, default=.02)),
     ros='action /arm/guarded_move', computer='Main PC: 1 kHz admittance loop on Bota F/T', real='FR3 + Bota')
tool('arm.grip', 'arm', L2S, 'Gripper: either width_mm (position, 0-80) or force_N (close until contact, then squeeze with ~force_N per finger).',
     motion=True, args=dict(width_mm=V('number', min=0., max=80.), force_N=V('number', min=1., max=F_MAX)),
     ros='action /franka_gripper/move | /franka_gripper/grasp (franka_msgs)', computer='Main PC -> Franka Hand', real='Franka Hand')
tool('arm.release', 'arm', L2S, 'Open the gripper (default fully, 80 mm).', motion=True, args=dict(width_mm=V('number', min=10., max=80., default=80.)),
     ros='action /franka_gripper/move', computer='Main PC -> Franka Hand', real='Franka Hand')
tool('arm.follow_arc', 'arm', ('L2',), 'Move the TCP on a circular arc about an axis (axis_point, axis_dir, right-hand rule, angle_deg); '
     'the hand attitude rotates by orient_ratio*angle. For hinged doors / tilting a held plate about its rim. Joint path + collision checked '
     '(the gripped appliance part is excluded).', motion=True,
     args=dict(axis_point=V('vec3', required=True), axis_dir=V('vec3', required=True), angle_deg=V('number', min=-120., max=120., required=True),
               orient_ratio=V('number', min=0., max=1., default=1.), speed=V('number', min=.01, max=.3, default=.08)),
     ros='action /arm/follow_arc', computer='Main PC: joint-path pre-plan + Cartesian impedance', real='FR3')
tool('arm.push_along', 'arm', ('L2',), 'Slide the TCP horizontally along direction for distance while regulating the downward wrist force to press_N '
     '(0 = pure position). For pushing a flat object across the counter.', motion=True,
     args=dict(direction=V('vec3', required=True), distance=V('number', min=.005, max=.40, required=True),
               press_N=V('number', min=0., max=15., default=8.), speed=V('number', min=.005, max=.06, default=.04)),
     ros='action /arm/push_along', computer='Main PC: 1 kHz F/T admittance (Bota 800 Hz)', real='FR3 + Bota')
tool('done', 'mission', ALL, 'End the mission. success = whether YOU believe the goal is achieved; summary = what happened (and why, if not).',
     dict(success=V('boolean', required=True), summary=V('string', required=True)), ros='topic /mission/report + BT terminates',
     computer='Main PC', real='app notification to the user')

SUGGEST = {
    'DOOR_NOT_LATCHED': ['the door stay did not catch at the 90 deg stop; stays usually catch when the door is held against the stop for a few seconds',
                         'something pushes the door back (obstruction under the door)', 'hinge spring / stay fault (needs a human)'],
    'DOOR_NOT_AT_STOP': ['obstruction in the door swing', 'bar slipped out of the fingers'],
    'POD_NOT_GRASPED': ['the pod is not at the pick point (not in its dish?): stow, vision.locate("pod"), then load_pod with pick_xy / yaw_deg',
                        'if the pod cannot be found it is missing: the wash can run without it only if the user agrees'],
    'BASKET_STOPPED': ['something inside is catching the basket; pushing the basket fully back in and pulling again often frees it',
                       'basket off its rails or a heavy obstruction: stop and ask a human'],
    'PLATE_LOST': ['the plate slipped out / was knocked out of the fingers and is probably back on the counter: stow, vision.locate("plate"), pick again',
                   'if it is not on the counter it may be on the floor: stop and ask a human'],
    'PUSH_RESULT_BAD': ['plate did not end at the expected overhang: re-locate and retry'],
    'GRASP_FAILED': ['object not where expected: re-perceive'],
    'UNREACHABLE': ['target outside the reachable area for this skill: move the object (slide) or choose another target'],
    'NO_IK_PATH': ['target or attitude outside the arm reach / joint limits along a straight line: add a via point, change the attitude, '
                   'or (L2) re-orient in free space with arm.move_to_pose'],
    'COLLISION_PREDICTED': ['the straight path would hit the named part: go around it (e.g. move up / out in +x first)'],
    'PERSON_NEAR_SPEED': ['a person is within the robot safety zone: robot.set_speed_limit(mps<=0.25) to continue slowly, or robot.wait until they leave'],
    'BASKET_NOT_IN': ['basket blocked: check for an obstruction'],
    'DOOR_NOT_CLOSED': ['something blocks the door: is the basket fully in?'],
    'INSERT_UNCONFIRMED': ['the plate may not be seated: inspect / retry'],
    'TRACKING_ERROR': ['the motion was resisted: a held object constrains the hand (e.g. a hinged door must be moved along its arc), '
                       'or the hand hit something; check gripper width / wrist load / appliance sensors before continuing'],
}


# ---------------------------------------------------------------- validation
def _vec(v, n):
    return isinstance(v, (list, tuple)) and len(v) == n and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def validate(spec, args):
    if not isinstance(args, dict):
        return 'arguments must be a JSON object'
    extra = set(args) - set(spec)
    if extra:
        return f'unknown argument(s) {sorted(extra)}; allowed: {sorted(spec)}'
    for k, s in spec.items():
        if k not in args or args[k] is None:
            if s.get('required'):
                return f'missing required argument {k!r}'
            continue
        v, t = args[k], s['type']
        if t in ('number', 'integer'):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or (t == 'integer' and int(v) != v) or not math.isfinite(v):
                return f'{k} must be a {t}'
            if 'min' in s and v < s['min'] or 'max' in s and v > s['max']:
                return f'{k}={v} outside [{s.get("min")}, {s.get("max")}]'
        elif t == 'boolean' and not isinstance(v, bool):
            return f'{k} must be true/false'
        elif t == 'string':
            if not isinstance(v, str):
                return f'{k} must be a string'
            if 'enum' in s and v not in s['enum']:
                return f'{k} must be one of {s["enum"]}'
        elif t == 'vec2' and not _vec(v, 2):
            return f'{k} must be [x, y] numbers'
        elif t == 'vec3' and not _vec(v, 3):
            return f'{k} must be [x, y, z] numbers'
    return None


def r3(v):
    return [round(float(x), 4) for x in v]


class RobotAPI:
    def __init__(self, robot: SR.SimRobot, level='L1', log=print):
        self.r = robot
        self.level = level
        self.log = log
        self.stats = dict(calls=0, ok=0, invalid=0, rejected_precondition=0, rejected_safety=0, blocked_motion=0, failed=0, internal=0)
        self.history = []
        self.done = None
        self.pending_images = []

    # ------------------------------------------------------------ state
    def state(self):
        r = self.r
        p, R = r.tcp()
        ld = r.ft_load()
        perc = {}
        for k, v in r.percepts.items():
            e = v['est']
            xyz = (e['origin'] + e['P'][:, 2] * S.MID_Z) if k == 'plate' else np.array(e['xyz'])
            perc[k] = dict(centre_xyz=r3(xyz), age_s=round(r.data.time - v['t'], 1), fresh=bool(v['valid']))
            if k == 'pod':
                perc[k]['yaw_deg'] = e.get('yaw_deg')
        return dict(sim_time_s=round(float(r.data.time), 1),
                    arm=dict(tcp_xyz=r3(p), approach=r3(R[:, 2]), closing=r3(R[:, 1]), at_stow=r.at_stow(),
                             gripper_width_mm=round(r.grip_width_mm(), 1), holding=r.holding(), wrist_load_N=[round(float(x), 1) for x in ld]),
                    dishwasher_simulated_sensors=dict(door_deg=round(r.door_deg(), 1), basket_out_m=round(r.basket_m(), 3), panel=r.panel_state()),
                    perception=perc,
                    safety=dict(person_near_robot=bool(r.person['present']),
                                speed_limit_mps=r.speed_limit if r.speed_limit else f'nominal (<= {V_NOMINAL})', estop=False))

    # ------------------------------------------------------------ preconditions
    def _pre(self, name, a):
        r = self.r
        door, bsk = r.door_deg(), r.basket_m()
        empty = r.holding().startswith('nothing')
        plate_held = r.plate_in_hand()[0] and getattr(r, 'T_hp', None) is not None
        need = []
        if name in ('skills.load_pod', 'skills.pull_basket', 'skills.push_basket', 'skills.insert_plate') and door < 80:
            need.append(f'door must be open > 80 deg (now {door:.0f})')
        if name in ('skills.open_door', 'skills.load_pod', 'skills.pull_basket', 'skills.push_basket', 'skills.close_door',
                    'skills.press_button', 'skills.slide_plate', 'skills.pick_plate', 'skills.perceive_plate') and not empty and \
                not (name == 'skills.load_pod' and False):
            need.append(f'hand must be empty (now holding: {r.holding()})')
        if name == 'arm.stow' and plate_held:
            need.append('refused while holding the plate (stow attitude would drop it)')
        if name == 'skills.load_pod' and bsk > .01:
            # found by GPT (L1 basket_jam s2): with the basket out it lies over the open door and covers the groove
            need.append(f'basket must be in (now {bsk*1000:.0f} mm out): the pulled-out basket covers the door groove')
        if name == 'skills.close_door':
            if bsk > .01:
                need.append(f'basket must be in (now {bsk*1000:.0f} mm out)')
            if 1 < door < 80:
                need.append(f'door is half open ({door:.0f} deg): open it to its stop first, the close skill re-grasps under the open door')
        if name == 'skills.insert_plate':
            if not plate_held:
                need.append('no plate in hand (finger width / wrist load)')
            if bsk < .24:
                need.append(f'basket must be out > 0.24 m (now {bsk:.3f})')
        if name in ('skills.slide_plate', 'skills.pick_plate'):
            pp = r.percepts.get('plate')
            if not pp or not pp['valid']:
                need.append('needs a fresh plate perception (vision.locate("plate") / skills.perceive_plate after the plate last moved)')
            else:
                c = pp['est']['origin']
                if name == 'skills.pick_plate':
                    if not (PICK_ZONE['x'][0] <= c[0] <= PICK_ZONE['x'][1] and PICK_ZONE['y'][0] <= c[1] <= PICK_ZONE['y'][1]):
                        need.append(f'plate centre ({c[0]:.3f}, {c[1]:.3f}) outside the pick zone x {PICK_ZONE["x"]} y {PICK_ZONE["y"]}: '
                                    'slide it into the zone first')
                    if door < 80 or bsk < .24:
                        need.append('destination not ready: open the door and pull the basket out before picking (there is no place-back skill)')
                else:
                    t = np.array(a['to_xy'])
                    if not (COUNTER_ZONE['x'][0] <= t[0] <= COUNTER_ZONE['x'][1] and COUNTER_ZONE['y'][0] <= t[1] <= COUNTER_ZONE['y'][1]):
                        need.append(f'to_xy outside the counter work area x {COUNTER_ZONE["x"]} y {COUNTER_ZONE["y"]}')
                    dish = np.array(SR.OPT.layout.dish)
                    seg0, seg1 = c[:2], t
                    u = seg1 - seg0
                    # reject only a slide that brings the plate closer to the dish than the clearance (moving away is fine)
                    dmin = min(float(np.linalg.norm(seg0 + k / 40 * u - dish)) for k in range(41))
                    if dmin < S.R_OUT + S.DISH_R + .01 and dmin < float(np.linalg.norm(seg0 - dish)) - .002:
                        need.append('the slide path would push the plate into the pod dish')
                    if np.linalg.norm(t - seg0) < .01:
                        need.append('plate already there')
        if name == 'vision.locate' and not r.at_stow():
            need.append('the arm may occlude the camera: arm.stow first')
        if name == 'skills.load_pod' and a.get('pick_xy') is not None:
            t = np.array(a['pick_xy'])
            if not (-0.45 <= t[0] <= -0.02 and 0.30 <= t[1] <= 0.90):
                need.append('pick_xy outside the counter area the arm can pick from')
        return need

    # ------------------------------------------------------------ safety gate
    def _safety(self, name, a, tl):
        r = self.r
        if tl['motion'] and r.person['present'] and (not r.speed_limit or r.speed_limit > V_PERSON):
            return 'PERSON_NEAR_SPEED', 'a person is near the robot and no speed cap <= 0.25 m/s is set'
        if name == 'robot.set_speed_limit' and r.person['present'] and a['mps'] > V_PERSON:
            return 'PERSON_NEAR_SPEED', f'{a["mps"]} m/s exceeds 0.25 m/s while a person is near'
        sp = a.get('speed')
        lim = r.speed_limit or V_NOMINAL
        if sp is not None and sp > lim:
            return 'SPEED_LIMIT', f'speed {sp} > current cap {lim} m/s'
        if name in ('arm.move_tcp', 'arm.move_to_pose'):
            p0 = r.tcp()[0]
            p = (p0 + np.array(a['xyz'])) if a.get('frame') == 'relative' else np.array(a['xyz'])
            for i, ax in enumerate('xyz'):
                lo, hi = WORKSPACE[ax]
                if ax == 'z' and p[0] > .02:
                    lo = Z_FLOOR_FRONT                   # in front of the counter edge: room below (collision check covers the cart)
                if not lo <= p[i] <= hi:
                    return 'WORKSPACE', f'target {ax}={p[i]:.3f} outside workspace [{lo}, {hi}]'
        if name in ('arm.move_until_contact',):
            p = r.tcp()[0] + np.asarray(a['direction'], float) / max(1e-9, np.linalg.norm(a['direction'])) * a['max_dist']
            if p[2] < WORKSPACE['z'][0] - .02:
                return 'WORKSPACE', 'guarded move would go below the workspace floor'
        return None

    # ------------------------------------------------------------ dispatch
    def call(self, name, args):
        t0w, t0s = time.monotonic(), float(self.r.data.time)
        self.stats['calls'] += 1
        res = self._call(name, args)
        res['sim_time_s'] = round(float(self.r.data.time), 2)
        res['call_sim_s'] = round(float(self.r.data.time) - t0s, 2)
        res['call_wall_s'] = round(time.monotonic() - t0w, 2)
        if name not in ('done',):
            res['state'] = self.state()
        scoring = res.pop('_scoring_all', {})
        self.history.append(dict(action=name, args=args, result={k: v for k, v in res.items() if k != 'state'}, scoring=scoring))
        return res, scoring

    def _call(self, name, args):
        r = self.r
        tl = T.get(name)
        if tl is None or self.level not in tl['levels']:
            self.stats['invalid'] += 1
            avail = sorted(n for n, t in T.items() if self.level in t['levels'])
            return dict(ok=False, code='INVALID_ACTION', message=f'{name!r} is not available at {self.level}. Available: {avail}')
        err = validate(tl['args'], args)
        if name == 'arm.grip' and not err and (args.get('width_mm') is None) == (args.get('force_N') is None):
            err = 'give exactly one of width_mm or force_N'
        if err:
            self.stats['invalid'] += 1
            return dict(ok=False, code='INVALID_ARGS', message=err)
        a = {k: args.get(k, s.get('default')) for k, s in tl['args'].items()}
        r.update_person()
        sf = self._safety(name, a, tl)
        if sf:
            self.stats['rejected_safety'] += 1
            return dict(ok=False, code='REJECTED_SAFETY', reason=sf[0], message=sf[1], suggested_causes=SUGGEST.get(sf[0], []))
        need = self._pre(name, a)
        if need:
            self.stats['rejected_precondition'] += 1
            return dict(ok=False, code='REJECTED_PRECONDITION', message='; '.join(need))
        try:
            out = self._exec(name, a) or {}
        except StepFail as e:
            mt = dict(getattr(e, 'metrics', {}) or {})
            code = mt.pop('code', 'SKILL_FAILED')
            self.stats['failed'] += 1
            self._recover_ref()
            return dict(ok=False, code=code, message=str(e), measurements=self._pub(mt), suggested_causes=SUGGEST.get(code, []),
                        _scoring_all=self._scoring(mt))
        except Exception as e:  # noqa: BLE001
            self.stats['internal'] += 1
            self._recover_ref()
            return dict(ok=False, code='INTERNAL_ERROR', message=f'{type(e).__name__}: {e}'[:300])
        ok = out.pop('_ok', True)
        code = out.pop('code', 'OK' if ok else 'FAILED')
        msg = out.pop('_reason', '')
        if out.get('blocked'):
            self.stats['blocked_motion'] += 1
        res = dict(ok=bool(ok), code=code, message=msg, measurements=self._pub(out), _scoring_all=self._scoring(out))
        if ok:
            self.stats['ok'] += 1
        elif not out.get('blocked'):
            self.stats['failed'] += 1
        if not ok:
            res['suggested_causes'] = SUGGEST.get(code, [])
        return res

    def _recover_ref(self):
        r = self.r
        r.ref.pop('cb', None)
        r.ref['cmd'] = r.tcp()
        r.allowed = set()

    @staticmethod
    def _pub(d):
        return {k: v for k, v in d.items() if not k.startswith('_')}

    @staticmethod
    def _scoring(d):
        out = {}
        for k, v in d.items():
            if k.startswith('_scoring'):
                if isinstance(v, dict):
                    out.update(v)
                else:
                    out[k.replace('_scoring_', '')] = v
        return out

    def _exec(self, name, a):
        r = self.r
        if name == 'robot.get_state':
            return {}
        if name == 'robot.wait':
            r.hold('wait', float(a['seconds']))
            return dict(waited_s=a['seconds'])
        if name == 'robot.set_speed_limit':
            r.speed_limit = float(a['mps']) if a['mps'] < V_NOMINAL else None
            return dict(speed_limit_mps=a['mps'])
        if name == 'vision.capture':
            rgb, depth, png = r.capture(f'capture_{len(self.history)}')
            self.pending_images.append(str(png))
            return dict(image=str(png.name), note='image attached to your next turn')
        if name == 'vision.locate':
            res = r.locate(a['object'])
            if getattr(r, 'last_overlay', None):
                self.pending_images.append(str(r.last_overlay))
            if not res.get('found'):
                return dict(_ok=False, code='NOT_FOUND', _reason=res.get('error', 'not found'), **res)
            return res
        if name == 'dishwasher.read_sensors':
            return dict(door_deg=round(r.door_deg(), 1), basket_out_m=round(r.basket_m(), 3), panel=r.panel_state(),
                        note='simulated sensor access (truth + noise)')
        if name == 'arm.get_tcp':
            p, R = r.tcp()
            return dict(tcp_xyz=r3(p), approach=r3(R[:, 2]), closing=r3(R[:, 1]))
        if name == 'arm.get_ft':
            return dict(wrist_load_N=[round(float(x), 2) for x in r.ft_load()], gripper_width_mm=round(r.grip_width_mm(), 1))
        if name == 'arm.stow':
            return r.pr_stow()
        if name == 'skills.perceive_plate':
            st = r.pr_stow()
            if not st.get('_ok'):
                return st
            res = r.locate('plate')
            if getattr(r, 'last_overlay', None):
                self.pending_images.append(str(r.last_overlay))
            if not res.get('found'):
                return dict(_ok=False, code='NOT_FOUND', _reason=res.get('error', 'plate not found'), **res)
            return res
        if name == 'skills.open_door':
            return r.sk_open_door(a['hold_at_stop_s'])
        if name == 'skills.load_pod':
            return r.sk_load_pod(a.get('pick_xy'), a.get('yaw_deg'))
        if name == 'skills.pull_basket':
            return r.sk_pull_basket()
        if name == 'skills.slide_plate':
            return r.sk_slide_plate(a['to_xy'])
        if name == 'skills.pick_plate':
            return r.sk_pick_plate()
        if name == 'skills.insert_plate':
            return r.sk_insert_plate()
        if name == 'skills.push_basket':
            return r.sk_push_basket()
        if name == 'skills.close_door':
            return r.sk_close_door()
        if name == 'skills.press_button':
            return r.sk_press(a['button'])
        if name == 'arm.move_tcp':
            try:
                p, R = r._pose(a['xyz'], a.get('approach'), a.get('closing'), a.get('frame') or 'world')
            except ValueError as e:
                return dict(_ok=False, code='INVALID_POSE', _reason=str(e))
            return r.pr_move(p, R, a['speed'], exclude=r.gripped_bodies())
        if name == 'arm.move_to_pose':
            try:
                p, R = r._pose(a['xyz'], a['approach'], a['closing'], 'world')
            except ValueError as e:
                return dict(_ok=False, code='INVALID_POSE', _reason=str(e))
            return r.pr_move_to_pose(p, R, a['speed'])
        if name == 'arm.move_until_contact':
            return r.pr_until_contact(a['direction'], a['force_N'], a['max_dist'], a['speed'])
        if name == 'arm.grip':
            return r.pr_grip(a.get('width_mm'), a.get('force_N'))
        if name == 'arm.release':
            return r.pr_grip(width_mm=a['width_mm'])
        if name == 'arm.follow_arc':
            return r.pr_follow_arc(a['axis_point'], a['axis_dir'], a['angle_deg'], a['orient_ratio'], a['speed'])
        if name == 'arm.push_along':
            return r.pr_push_along(a['direction'], a['distance'], a['press_N'], a['speed'])
        if name == 'done':
            self.done = dict(success=bool(a['success']), summary=a['summary'])
            return dict(acknowledged=True)
        raise ValueError(name)


# ---------------------------------------------------------------- docs (prompt + API.md)
def _sig(t):
    parts = []
    for k, s in t['args'].items():
        ty = s['type'] + (f' {s["enum"]}' if 'enum' in s else '') + (f' [{s.get("min")}..{s.get("max")}]' if 'min' in s else '')
        parts.append(f'{k}{"" if s.get("required") else "?"}: {ty}' + (f' = {s["default"]}' if s.get('default') is not None else ''))
    return f'{t["name"]}({", ".join(parts)})'


def prompt_docs(level):
    lines = []
    for n, t in T.items():
        if level in t['levels']:
            lines.append(f'- {_sig(t)}: {t["desc"]}')
    return '\n'.join(lines)


def geometry_facts(level, hints=False):
    """Known calibrated kitchen / appliance model (what the real system has from CAD + per-dock calibration)."""
    hinge = plan.HINGE
    bar0 = plan.handle_pos(0.)
    bar90 = plan.handle_pos(math.pi / 2)
    f = [
        'World frame (m): counter front edge is the line x=0, counter top z=0.90, counter extends to x=-0.60 (wall). +x points out into the room, +y to the right seen from the room, z up.',
        'Dishwasher (countertop unit) stands on the counter centred at y=0, front face x=0.0, width y -0.215..0.215, top z=1.36. Drop-down door hinged at its bottom; it opens outward/down to 90 deg (horizontal) and rests on a stop.',
        f'Robot: Franka arm on a cart in front of the counter, base at (0.48, 0.42, 0.80). Stow pose TCP {SR.PARK.tolist()}. The wall camera looks down at the counter area right of the dishwasher (x -0.42..0, y 0.2..1.0).',
        f'Flat 8-inch plate (r 0.1015 m, 0.45 kg) starts on the counter right of the dishwasher. Pod dish centre ({SR.OPT.layout.dish[0]}, {SR.OPT.layout.dish[1]}), r 0.045. Pod 32x24x13 mm, 18 g.',
        f'pick_plate pick zone (plate centre): x {PICK_ZONE["x"]}, y {PICK_ZONE["y"]}. slide_plate target area: x {COUNTER_ZONE["x"]}, y {COUNTER_ZONE["y"]}.',
        'Panel pads (left to right): eco, intensive, quick, dry, fruit, power. Program pads select a program (display shows it); POWER starts the selected program (door must be closed).',
    ]
    if level != L1:
        gc = plan.groove_center()
        slot = plan.plate_slot(dw.BASKET_TRAVEL)
        f += [
            'TCP = centre between the finger pad tips. Hand attitude = approach (palm->fingertips) + closing axis (direction the fingers close along). Hand-down: approach [0,0,-1].',
            f'Door hinge axis: point {r3(hinge)}, direction [0,1,0] (positive rotation = door top moves out/down = opening). Door pull bar (horizontal along y, radius {dw.HANDLE_R*1000:.0f} mm) '
            f'centre {r3(bar0)} when closed, {r3(bar90)} when open 90 deg; the bar is {np.linalg.norm(bar0 - hinge):.3f} m from the hinge axis. '
            'The door is light (1.6 kg): to open, grasp the bar (fingers above and below it, approach -x) and swing it 90 deg about the hinge axis.',
            f'Door groove (pod goes in, door must be open 90 deg): centre {r3(gc)}, opening 45 mm along y x 35 mm along x, faces up (door inner face z {plan.groove_face_z():.3f}).',
            f'Lower basket pull bar (along y, radius 4 mm): centre {r3(plan.basket_handle(0.))} when the basket is in; it slides along +x to {r3(plan.basket_handle(dw.BASKET_TRAVEL))} at the 0.25 m end stop. Door must be open first.',
            f'Plate slot (basket out 0.25 m): seated plate centre {r3(slot)}; the plate must stand upright in the y-z plane (normal along x) between two tines 24 mm apart, rim resting on the basket floor wires.',
            'Panel pad centres (press along -x, ~3 N): ' + ', '.join(f'{b} {r3(plan.button_pos(b))}' for b in dw.BUTTONS) + '.',
            'Gripper: max opening 80 mm; fingers are 18 mm wide pads; the plate rim is 5 mm thick; a plate lying flat is ~20 mm tall overall.',
        ]
    if hints:
        f += L2_HINTS
    return '\n'.join('- ' + x for x in f)


# Added after the L2v0 attempt (GPT_CONTROL.md, 'what had to be added'): GPT found the edge push and the rim pinch by
# itself, but lifted the flat plate straight up by the rim, which always slipped.
L2_HINTS = [
    'Lifting a flat plate straight up by a rim pinch slips (the pinch cannot hold the plate\'s weight moment). Instead, keep its far rim '
    'on the counter and tilt the plate up 90 deg about that far-rim contact line (arm.follow_arc, orient_ratio 1, axis along y through the '
    'far rim bottom), so it stands vertical in the hand; then lift it.',
]


def write_api_md(path=ROOT / 'API.md'):
    rows = ['| API call | Levels | Real component | ROS 2 interface | Served by (computer) |', '|---|---|---|---|---|']
    for n, t in T.items():
        lv = ', '.join(sorted(x for x in t['levels'] if x != 'L2v0') + (['L2v0'] if 'L2v0' in t['levels'] else []))
        rows.append(f'| `{_sig(t)}` | {lv} | {t["real"]} | `{t["ros"]}` | {t["computer"]} |')
    codes = '\n'.join(f'- `{k}`: ' + '; '.join(v) for k, v in SUGGEST.items())
    txt = f"""# Robot API (what GPT can call)

Generated by `robot_api.py write_api_md()`; the same registry produces the prompt text, the argument validator
and this table, so they cannot drift apart.

## Contract

- **Call:** `{{"action": "<name>", "args": {{...}}}}`. The decision schema (`gpt_agent.py`) restricts `action` to the
  names enabled at the current level (an enum), so GPT can't name a non-existent call. `args` is validated
  server-side against the per-call schema below.
- **Result:** `{{ok, code, message, measurements, suggested_causes?, state, sim_time_s, call_sim_s, call_wall_s}}`.
  `state` is a fresh `robot.get_state()` snapshot. Ground-truth numbers used for scoring are stripped before the result
  reaches GPT; they are stored separately in the run log.
- **Validation order:** unknown action → `INVALID_ACTION`; schema → `INVALID_ARGS`; safety gate → `REJECTED_SAFETY`
  (person near the robot without a ≤ 0.25 m/s cap, speed above the cap, TCP target outside the workspace box
  x {WORKSPACE['x']} y {WORKSPACE['y']} z {WORKSPACE['z']} (z floor {Z_FLOOR_FRONT} in front of the counter edge), force caps ≤ {F_MAX:.0f} N by schema); preconditions →
  `REJECTED_PRECONDITION`. L2 motions only: joint path → `NO_IK_PATH`, then a collision check against the calibrated
  kitchen and dishwasher model (door and basket at their sensed state, fingers 2 mm and links 8 mm margin) → `COLLISION_PREDICTED`.
  Only then does anything move.
- **What GPT can never do:** send joint torques, joint positions, gains or controller rates. The low-level loops stay
  in the controllers: 500 Hz physics and 100 Hz IK in the sim; 1 kHz FCI and 800 Hz F/T on the real FR3.
- **Dishwasher sensors:** door angle, basket travel and the panel display are **simulated sensor access** (ground truth
  plus noise: 0.3° door, 2 mm basket). The real system would estimate them with the wall camera and read the panel LEDs,
  or use the appliance's cloud API if it has one. The wall camera in this layout does not see the door or the panel,
  so a real deployment needs a second view or the appliance API.
- **Tactile proxy:** `robot.get_state().arm.holding` and the skills' verdicts use finger width (encoders) plus the wrist
  load (F/T). In the L2 collision check, an appliance part the fingers are closed on (known from finger contacts) is
  ignored for the fingers only. The checker moves that part along with the hand: the door follows the hinge arc, the
  basket follows the pull.
- **What is checked is what runs:** L2 paths are solved by sequential IK from the *measured* joint state and densified to
  ≤ 2 cm / ≤ 5°. That exact joint path is collision-checked and then executed by joint interpolation. Near-contacts that
  already exist at the start are allowed only if the gap does not shrink. The hand is never flipped 180° while it holds
  something. A TCP more than 20 mm off target after a motion returns `TRACKING_ERROR`, not OK.
- **`arm.stow`:** a collision-checked joint path to the park configuration (direct, via the hub pose, or after a 5 cm
  retreat), or a rejection. The pipeline's canned transfer is only safe from the poses its own skills leave the arm in.

## Calls, levels and the real hardware they map to

L1 = skills level. L2 = primitives with mid-level arc/push. L2v0 = minimal primitives (first attempt).

{chr(10).join(rows)}

## Preconditions the API enforces (`REJECTED_PRECONDITION`)

| Call | Precondition | Why |
|---|---|---|
| `skills.load_pod`, `pull_basket`, `push_basket`, `insert_plate` | door > 80° | the basket rolls out over the open door; the groove is on the door |
| `skills.load_pod` | basket in (< 0.01 m) | **found by GPT (L1 basket_jam s2):** the pulled-out basket covers the door groove |
| `skills.load_pod` with `pick_xy` | pick point on the counter area the arm can reach | reach |
| hand-occupying skills | hand empty (finger width / wrist load) | the hand can do one thing at a time |
| `skills.pick_plate` | fresh plate perception; centre in the pick zone; door > 80° and basket out | there is no place-back skill, so picking with the destination closed is a dead end |
| `skills.slide_plate` | fresh perception; target in the counter area; path does not move the plate toward the pod dish | clearance to the dish |
| `skills.insert_plate` | plate in hand (finger width + 4.4 N wrist load); basket out > 0.24 m | |
| `skills.close_door` | basket in; door at its stop (or already closed) | the close skill re-grasps under the fully open door |
| `vision.locate` | arm stowed | the arm would occlude the camera |
| `arm.stow` | not holding the plate | the hand-down stow attitude would drop it |

## Error codes and the causes the API suggests

{codes}
"""
    path.write_text(txt)
    return path


if __name__ == '__main__':
    print(write_api_md())
    print(prompt_docs('L1'))
    print(geometry_facts('L2'))
