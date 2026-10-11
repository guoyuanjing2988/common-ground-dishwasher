"""Mobile robot API: ONE typed surface for the GPT / hybrid mission client. Conventions from ../gpt_control/robot_api.py
(schema check -> safety gate -> preconditions -> execute -> measured result {ok, code, message, measurements, state}),
extended with the wheeled base, power and verification calls and running on the ../mobile_pipeline simulation.

Validation order: unknown action -> INVALID_ACTION; schema -> INVALID_ARGS; safety gate -> REJECTED_SAFETY
(e-stop chain, person flagged without a <= 0.25 m/s cap, drive target outside the floor area); preconditions ->
REJECTED_PRECONDITION (mobile: arm stowed to drive, docked + braked before any arm skill, no undock with the door open or
the arm out, SoC >= 30 % to start a mission; task: door / basket / hand / perception ordering). Only then does anything
move. Every verdict comes from robot-side measurement (encoders, finger width, wrist F/T, lidar fix counts, dock /
charge contact force, BMS) or from SIMULATED appliance / verification sensors that are labelled as such. Ground truth
used for scoring is split off into `scoring` and never reaches GPT. A mission is only 'done' when done(success=true)
passes the API's own verification.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

import msim as MS
from msim import SM, dw, plan

ROOT = Path(__file__).resolve().parent
SOC_START_MIN = .30
FLOOR = dict(x=(0.35, 1.60), y=(-0.40, 2.40))           # base.drive_to target area (free floor in front of the counter)
V_NOMINAL = .50
PREDOCK = None                                          # set below


def _predock():
    x, y, yaw = SM.dock_base_pose()
    th = math.radians(yaw)
    return np.array([x + .55 * math.cos(th), y + .55 * math.sin(th)]), yaw


PREDOCK, DOCK_YAW = _predock()
DOCK_XY = np.array(SM.dock_base_pose()[:2])

# ---------------------------------------------------------------- registry
T = {}
V = lambda t, **k: dict(type=t, **k)  # noqa: E731


def tool(name, desc, args=None, job=None, kind='job', motion=False, base=False, arm=False, real='', ros='', computer='Main PC',
         timeout=240.):
    T[name] = dict(name=name, desc=desc, args=args or {}, job=job, kind=kind, motion=motion, base=base, arm=arm, real=real,
                   ros=ros, computer=computer, timeout=timeout)


# robot / mission
tool('robot.get_state', 'Fresh state snapshot (base, power, arm, appliance sensors, perception, verification, safety).', kind='local',
     real='state aggregator (FR3, BaseMCU, BMS, lidars, scanner)', ros='service /robot/get_state')
tool('robot.wait', 'Stay still for a while (e.g. until a person has left or the operator has acknowledged an e-stop). Brakes stay '
     'on, the arm holds.', dict(seconds=V('number', min=.5, max=120., required=True)), kind='local', real='behaviour tree',
     ros='BT Wait node')
tool('robot.set_speed_limit', 'Cap the arm TCP speed AND the base driving speed for all following motions (m/s). Use <= 0.25 '
     'whenever a person is flagged; 0.5 = nominal (no cap).', dict(mps=V('number', min=.05, max=V_NOMINAL, required=True)), kind='local',
     real='FR3 SLS + BaseMCU speed limit (safety functions)', ros='service /safety/set_speed_limit')
tool('robot.resume', 'After an e-stop: continue the SUSPENDED skill from where it stopped. Allowed only after the e-stop has been '
     'released AND the operator has acknowledged. Returns that skill\'s result.', kind='local', motion=True,
     real='safety PLC reset + controller resume', ros='service /safety/resume')
tool('robot.abort_skill', 'After an e-stop: cancel the SUSPENDED skill instead of resuming it (the arm stays where it is, the hand '
     'keeps its grip).', kind='local', real='behaviour tree', ros='action cancel')
tool('robot.ask_human', 'Stop and ask a human for help (ends the mission; the robot holds still and safe). Use when you cannot '
     'recover safely.', dict(question=V('string', required=True)), kind='local', real='app notification', ros='topic /mission/ask_human')
tool('done', 'End the mission. success=true is accepted ONLY if the API verifies every outcome (panel RUNNING QUICK, door closed, '
     'basket in, pod and plate verified, arm stowed, on the charger and charging); otherwise it is refused with the missing items '
     'and the mission continues. success=false ends the mission (summary = what happened, in the user\'s language).',
     dict(success=V('boolean', required=True), summary=V('string', required=True)), kind='local', real='app notification',
     ros='topic /mission/report')
# power
tool('power.get_soc', 'Battery state of charge, voltage, current, charging (BMS).', kind='local', real='BMS', ros='topic /bms/state',
     computer='BaseMCU')
tool('power.charge', 'Stay on the charging contacts until the SoC reaches target_soc or max_minutes pass (net ~+0.5 %/min while awake). '
     'Needs the robot on the charger.', dict(target_soc=V('number', min=.30, max=1.0, required=True),
                                              max_minutes=V('number', min=1., max=25., default=20.)), job='charge',
     real='charger 11.3 A + BMS', ros='action /power/charge', timeout=1600.)
# base
tool('base.get_state', 'Base pose estimate (lidar + odometry), localization quality, brakes, dock / charge contact force, speed.',
     kind='local', real='BaseMCU + lidars', ros='topic /base/state', computer='BaseMCU + Main PC')
tool('base.localize', 'Wait for fresh lidar fixes (cabinet-front line + reflective strip) and report the pose estimate.',
     job='localize', real='2 corner lidars', ros='service /base/localize', timeout=20.)
tool('base.undock_charger', 'Release brakes and drive 0.30 m forward off the charger (start of a mission). Needs SoC >= 30 %.',
     job='bm', motion=True, base=True, real='BaseMCU wheel drives', ros='action /base/undock_charger', timeout=60.)
tool('base.go_predock', 'Drive to the pre-dock pose 0.55 m in front of the work dock (straight line, then turn to face away from the '
     'cabinet).', job='bm', motion=True, base=True, real='BaseMCU + lidar localization', ros='action /base/navigate (Nav2-like)',
     timeout=120.)
tool('base.dock', 'From the pre-dock pose: reverse at 0.05 m/s into the cabinet V-blocks until the dock plate reads 100 N, brake, '
     'average 3 s of lidar fixes for the arm\'s base pose. Verdict = dock force + localization quality.', job='bm', motion=True,
     base=True, real='BaseMCU + dock plate force sensor + lidars', ros='action /base/dock', timeout=120.)
tool('base.undock', 'Release brakes and drive 0.50 m forward out of the work dock. Needs the arm stowed and the dishwasher door closed.',
     job='bm', motion=True, base=True, real='BaseMCU', ros='action /base/undock', timeout=60.)
tool('base.go_charger', 'Drive back (in reverse) to 8 cm in front of the charging contacts.', job='bm', motion=True, base=True,
     real='BaseMCU + lidars', ros='action /base/navigate', timeout=120.)
tool('base.dock_charger', 'Reverse slowly onto the charging contacts (15 N), brake. Verdict = contact force.', job='bm', motion=True,
     base=True, real='BaseMCU + charge collector', ros='action /base/dock_charger', timeout=90.)
tool('base.drive_to', 'Drive to (x, y) on the kitchen floor (straight line after turning), then turn to heading_deg if given. '
     'direction forward/reverse. For detours; the named moves above are preferred.',
     dict(x=V('number', required=True), y=V('number', required=True), heading_deg=V('number', min=-180., max=180.),
          direction=V('string', enum=['forward', 'reverse'], default='forward')), job='bm', motion=True, base=True,
     real='BaseMCU + lidars', ros='action /base/navigate', timeout=120.)
tool('base.brake', 'Engage (on=true) or release the wheel brakes (the API engages them after every base motion).',
     dict(on=V('boolean', required=True)), job='brake_cmd', real='spring brakes', ros='service /base/brake', timeout=10.)
# arm
tool('arm.unstow', 'From the compact driving pose to the camera-clear park pose over the counter, solved for the MEASURED base pose. '
     'Needs docked + brakes on.', job='g_unstow', motion=True, arm=True, real='FR3', ros='action /arm/unstow', timeout=60.)
tool('arm.stow', 'Back to the compact driving pose (needed before any base motion). Refused while holding the plate.',
     job='g_stow', motion=True, arm=True, real='FR3', ros='action /arm/stow', timeout=90.)
tool('vision.locate', "Locate the plate lying flat on the counter with the wall RGB-D camera: Codex 2-D detection + depth fit -> "
     "metric centre. Moves the arm to the camera-clear park pose first if needed. ~15-25 s.",
     dict(object=V('string', enum=['plate'], required=True)), job='g_locate', motion=True, arm=True,
     real='D435i + Codex VLM', ros='action /wall_node/locate', computer='Wall node + Main PC', timeout=400.)
tool('vision.capture', 'Take a wall-camera RGB image; it is attached to your next turn.', kind='local', real='D435i',
     ros='service /wall_node/capture', computer='Wall node')
tool('dishwasher.read_sensors', 'Door angle (deg, 0 closed, 90 open), basket travel (m, 0 in, 0.25 out) - SIMULATED SENSOR ACCESS '
     '(truth + noise) - and the panel LED readout (wall camera).', kind='local', real='camera estimate / appliance API',
     ros='service /appliance/state', computer='Wall node')
# skills (L1)
tool('skills.open_door', 'Grasp the door pull bar at its current angle, follow the hinge arc to the 90 deg stop, hold it against the '
     'stop for hold_at_stop_s, release, back off. Verdict = door angle after release (> 85 deg).',
     dict(hold_at_stop_s=V('number', min=.1, max=6., default=.5)), job='g_open_door', motion=True, arm=True,
     real='FR3 + Franka Hand', ros='action /skills/open_door')
tool('skills.load_pod', 'Pick the detergent pod from its dish on the counter and drop it into the door groove. Needs the door open '
     '> 80 deg and the basket in. Verdict = finger width at lift (pod in hand); the groove itself is NOT observed (verify.pod_in_groove).',
     job='g_load_pod', motion=True, arm=True, real='FR3 + Hand', ros='action /skills/load_pod')
tool('skills.pull_basket', 'Grasp the lower-basket bar and pull it out to the 0.25 m stop with a 25 N wrist-force guard. Needs door > 80 deg.',
     job='g_pull_basket', motion=True, arm=True, real='FR3 + Hand + Bota F/T', ros='action /skills/pull_basket')
tool('skills.pick_plate', 'Pick the flat plate: push it to overhang the counter edge (8 N admittance), re-perceive (Codex), pinch the '
     'rim, tilt it up about its far rim, lift. Needs a fresh plate perception and the destination ready (door > 80 deg, basket out '
     '> 0.24 m). Verdict = finger width + wrist load ~4.4 N.', job='g_pick_plate', motion=True, arm=True,
     real='FR3 + Hand + Bota + D435i/Codex', ros='action /skills/pick_plate', timeout=500.)
tool('skills.insert_plate', 'Carry the held plate into the lower-basket slot, lower until the wrist F/T feels touchdown, release, '
     'retract. Verdict = touchdown felt + hand unloaded; the seat is NOT observed (verify.plate_seated).', job='g_insert_plate',
     motion=True, arm=True, real='FR3 + Bota', ros='action /skills/insert_plate')
tool('skills.push_basket', 'Grasp the basket bar and push the basket fully in. Needs an empty hand.', job='g_push_basket',
     motion=True, arm=True, real='FR3 + Hand', ros='action /skills/push_basket')
tool('skills.close_door', 'Re-grasp the bar under the open door and push the door up to closed (|angle| < 1 deg). Needs the basket '
     'in and an empty hand.', job='g_close_door', motion=True, arm=True, real='FR3 + Hand', ros='action /skills/close_door')
tool('skills.press', 'Touch one control-panel pad with the conductive stylus (F/T-guided, ~3 N, hold 0.5 s), then read the panel LEDs. '
     'Verdict = the panel readout changed.', dict(button=V('string', enum=list(dw.BUTTONS), required=True)), job='press',
     motion=True, arm=True, real='FR3 + Bota + stylus; panel LEDs via the wall camera', ros='action /skills/press')
# verification (SIMULATED sensing)
tool('verify.pod_in_groove', 'Check that the pod sits in the door groove. SIMULATED SENSOR (truth + noise; stands in for a wrist camera '
     'looking into the groove). Needs the door open > 80 deg.', job='g_verify_pod', real='(simulated) wrist camera',
     ros='service /verify/pod_in_groove', timeout=10.)
tool('verify.plate_seated', 'Check that the plate stands in its basket slot (lean < 10 deg, between its own tines). SIMULATED SENSOR '
     '(truth + noise; stands in for a camera view of the basket). Needs door > 80 deg and the basket out.', job='g_verify_plate',
     real='(simulated) camera view of the basket', ros='service /verify/plate_seated', timeout=10.)

SUGGEST = {
    'LOW_BATTERY': ['SoC is below the 30 % mission-start threshold: dock onto the charger (base.dock_charger) and charge '
                    '(power.charge), or decline the mission'],
    'PATH_BLOCKED': ['protective stop: a person / obstacle is in the driving path; wait (robot.wait) until the field is clear and retry, '
                     'or drive around it with base.drive_to'],
    'PERSON_NEAR_SPEED': ['a person is flagged near the robot: robot.set_speed_limit(mps<=0.25) to continue slowly, or robot.wait until they leave'],
    'ESTOP_ACTIVE': ['the e-stop chain has stopped the robot: wait (robot.wait) until it is released and the operator has acknowledged'],
    'ESTOP': ['the skill is suspended, not finished: after release + operator acknowledgement call robot.resume() to continue it, '
              'or robot.abort_skill()'],
    'DOCK_PRELOAD_LOW': ['the base did not seat in the V-blocks: undock, go back to the pre-dock pose and dock again'],
    'CHARGER_NOT_REACHED': ['the charging contacts were not reached: go_charger again, then dock_charger'],
    'DOOR_NOT_LATCHED': ['the door stay did not catch at the 90 deg stop; stays usually catch when the door is held against the stop for a few seconds',
                         'something pushes the door back (obstruction under the door)', 'hinge spring / stay fault (needs a human)'],
    'DOOR_NOT_AT_STOP': ['obstruction in the door swing', 'bar slipped out of the fingers'],
    'POD_NOT_GRASPED': ['the pod was not at the point where the arm expected it: the pod may have been moved, or the arm is '
                        'mis-registered to the kitchen (base pose estimate)', 'if the pod is missing the wash can run without it only if the user agrees'],
    'POD_NOT_IN_GROOVE': ['the pod is not seated in the groove (no skill can pick it from the door): ask a human, or continue only if the user agrees'],
    'BASKET_STOPPED': ['something is catching the basket; pushing it fully back in and pulling again often frees it', 'heavy obstruction: ask a human'],
    'PLATE_LOST': ['the plate slipped out of the fingers and is probably back on the counter: vision.locate("plate"), pick again',
                   'if it is not on the counter: ask a human'],
    'PLATE_NOT_SEATED': ['the plate is not standing in its slot: ask a human to re-seat it'],
    'PUSH_RESULT_BAD': ['plate did not end at the expected overhang: re-locate and retry'],
    'GRASP_FAILED': ['object not where expected: re-perceive'],
    'UNREACHABLE': ['target outside the arm\'s reach from the current base pose: check the base pose / re-dock'],
    'BASKET_NOT_IN': ['basket blocked: check for an obstruction'],
    'DOOR_NOT_CLOSED': ['something blocks the door: is the basket fully in?'],
    'INSERT_UNCONFIRMED': ['the plate may not be seated: verify.plate_seated'],
    'PANEL_NO_RESPONSE': ['the panel readout did not change after the touch: the panel may have been asleep or the touch was not '
                          'registered - press again', 'START with POWER is refused while the door is open (door LED on)'],
    'PRESS_NO_CONTACT': ['the stylus did not reach the pad (no 3 N F/T contact): retry'],
    'MISSION_NOT_VERIFIED': ['complete or verify the missing items, or end with done(success=false)'],
    'NOT_FOUND': ['re-capture / retry; check that the arm is not occluding the camera'],
    'PERCEPTION_SUSPECT': ['retry vision.locate'],
}
PANEL_DOC = ('Panel logic (appliance MCU): OFF --POWER--> STANDBY --program pad--> SELECTED(program) --POWER (door closed)--> RUNNING. '
             'POWER with the door open is refused (door LED). STANDBY/SELECTED switch off after 120 s idle.')


def _vec_ok(v, n):
    return isinstance(v, (list, tuple)) and len(v) == n and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)


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
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return f'{k} must be a {t}'
            if ('min' in s and v < s['min']) or ('max' in s and v > s['max']):
                return f'{k}={v} outside [{s.get("min")}, {s.get("max")}]'
        elif t == 'boolean' and not isinstance(v, bool):
            return f'{k} must be true/false'
        elif t == 'string':
            if not isinstance(v, str):
                return f'{k} must be a string'
            if 'enum' in s and v not in s['enum']:
                return f'{k} must be one of {s["enum"]}'
    return None


def r3(v, n=3):
    return [round(float(x), n) for x in v]


# ====================================================================== the API
class MobileAPI:
    def __init__(self, sysm, yield_fn, log=print):
        self.s = sysm
        self.pl = sysm.planner
        self.sup = sysm.supervisor
        self.y = yield_fn
        self.log = log
        self.stats = dict(calls=0, ok=0, invalid=0, rejected_precondition=0, rejected_safety=0, failed=0, internal=0, estop=0)
        self.history = []
        self.ended = None                  # dict(kind=..., success=..., summary=...)
        self.claims = []                   # done(success=true) attempts (verified or not)
        self.verified = {}
        self.pending_images = []
        self.speed_cap = None
        self.mission_started = False
        self.start_soc = None
        self.suspended = None
        self.last_action = None
        self.n_img = 0

    # ------------------------------------------------------------------ sensing helpers
    @property
    def d(self):
        return self.s.d

    def soc(self):
        b = self.sup.bms
        return float(b['soc']) if b else float(self.s.battery.soc)

    def pose_est(self):
        e = self.pl.base_est
        return [round(float(e[0]), 3), round(float(e[1]), 3), round(math.degrees(float(e[2])), 1)]

    def base_st(self):
        return self.pl.base_state or {}

    def on_charger(self):
        st = self.base_st()
        return float(st.get('charge_F', 0.)) >= 10.

    def near(self, xy, r):
        return float(np.linalg.norm(np.array(self.pl.base_est[:2]) - np.asarray(xy))) < r

    def heading_err(self, deg):
        return abs((math.degrees(float(self.pl.base_est[2])) - deg + 180) % 360 - 180)

    def where(self):
        if self.pl.docked:
            return 'docked at the work dock'
        if self.on_charger():
            return 'on the charger (contacts)'
        if self.near(SM.START_NOMINAL[:2], .12):
            return 'in front of the charger (8 cm from the contacts)'
        if self.near(PREDOCK, .10) and self.heading_err(DOCK_YAW) < 8:
            return 'at the pre-dock pose'
        return 'driving area'

    def plate_held(self):
        return self.pl.T_hp is not None and self.pl.grip_width_mm() > 2.5

    def hand_empty(self):
        return self.pl.holding().startswith('nothing')

    def panel_readout(self):
        p = self.sup.panel
        if p is None:
            return None
        return dict(state=p['state'], program=p['program'], door_led=bool(p['leds'].get('door')))

    def state(self):
        pl, s = self.pl, self.s
        st = self.base_st()
        b = self.sup.bms or {}
        perc = {}
        if pl.percept is not None:
            e = pl.percept['est']
            c = e['origin'] + e['P'][:, 2] * MS.S.MID_Z
            perc['plate'] = dict(centre_xyz=r3(c), age_s=round(self.d.time - pl.percept['t'], 1), fresh=bool(pl.percept['valid']))
        ld = pl.wrist_load_ema()
        sc = s.scanner
        out = dict(
            sim_time_s=round(float(self.d.time), 1),
            base=dict(where=self.where(), pose_est=self.pose_est(), localization=pl.loc_quality(), brake=bool(st.get('brake')),
                      dock_force_N=round(float(st.get('dock_F', 0.))), charge_contact_N=round(float(st.get('charge_F', 0.)), 1),
                      speed_mps=round(abs(float(st.get('v', 0.))), 3)),
            power=dict(soc=round(self.soc(), 4), V=round(float(b.get('V', 0.)), 2), I_A=round(float(b.get('I', 0.)), 2),
                       charging=bool(b.get('charging')), mission_start_min_soc=SOC_START_MIN),
            arm=dict(pose=pl.arm_pose_name(), gripper_width_mm=round(pl.grip_width_mm(), 1), holding=pl.holding(),
                     plate_in_hand=self.plate_held(), wrist_load_N=None if ld is None else r3(ld, 1)),
            dishwasher=dict(simulated_sensors=dict(door_deg=round(pl.door_deg(), 1), basket_out_m=round(pl.basket_m(), 3)),
                            panel_led_readout=self.panel_readout()),
            perception=perc,
            verified=self.verified,
            safety=dict(person_flagged=bool(sc.near()), person_distance_m=round(sc.distance(), 2) if sc.near() else None,
                        speed_limit_mps=self.speed_cap if self.speed_cap else 'nominal 0.5 (no cap)',
                        estop=dict(pressed=self.sup.estop['pressed'], released=self.sup.estop['released'],
                                   operator_acknowledged=self.sup.estop['ack']),
                        suspended_skill=self.suspended['name'] if self.suspended else None),
            mission=dict(started=self.mission_started))
        if pl.docked:
            out['base']['pose_used_by_arm'] = r3(pl.base_pose_arm(), 4)
        return out

    # ------------------------------------------------------------------ gate + preconditions
    def _safety(self, name, a, tl):
        es = self.sup.estop
        if es['pressed'] and not (es['released'] and es['ack']) and (tl['motion'] or name in ('robot.resume',)):
            return 'ESTOP_ACTIVE', ('e-stop is pressed' if not es['released'] else 'e-stop released but not yet acknowledged by the operator')
        if tl['motion'] and self.s.scanner.near() and (not self.speed_cap or self.speed_cap > MS.V_PERSON):
            return 'PERSON_NEAR_SPEED', f'a person is flagged {self.s.scanner.distance():.2f} m from the robot and no speed cap <= 0.25 m/s is set'
        if name == 'robot.set_speed_limit' and self.s.scanner.near() and a['mps'] > MS.V_PERSON:
            return 'PERSON_NEAR_SPEED', f'{a["mps"]} m/s exceeds 0.25 m/s while a person is flagged'
        if name == 'base.drive_to':
            if not (FLOOR['x'][0] <= a['x'] <= FLOOR['x'][1] and FLOOR['y'][0] <= a['y'] <= FLOOR['y'][1]):
                return 'WORKSPACE', f'target outside the free floor area x {FLOOR["x"]} y {FLOOR["y"]}'
        return None

    def _pre(self, name, a, tl):
        pl = self.pl
        need = []
        door, bsk = pl.door_deg(), pl.basket_m()
        arm_pose = pl.arm_pose_name()
        if self.suspended and name not in ('robot.resume', 'robot.abort_skill') and (tl['motion'] or tl['kind'] == 'job'):  # planner busy
            need.append(f'skill {self.suspended["name"]} is suspended by the e-stop: robot.resume() or robot.abort_skill() first')
            return need
        if name == 'robot.resume' and not self.suspended:
            need.append('no suspended skill')
        if name == 'robot.abort_skill' and not self.suspended:
            need.append('no suspended skill')
        # ---- mobile
        if tl['base']:
            if arm_pose != 'stowed':
                need.append(f'arm must be stowed to drive (now: {arm_pose})')
            if not self.hand_empty():
                need.append(f'hand must be empty to drive (holding: {pl.holding()})')
            if pl.docked and name != 'base.undock':
                need.append('the base is docked at the work dock: base.undock first')
            if name == 'base.undock':
                if not pl.docked:
                    need.append('not docked at the work dock')
                if abs(door) > 2.:
                    need.append(f'dishwasher door must be closed before undocking (now {door:.0f} deg)')
            if name in ('base.undock_charger', 'base.go_predock', 'base.drive_to') and not self.mission_started and \
                    self.soc() < SOC_START_MIN:
                need.append(f'LOW_BATTERY: SoC {self.soc():.3f} < {SOC_START_MIN:.2f} mission-start threshold')
            if name == 'base.undock_charger' and not (self.on_charger() or self.near(SM.START_NOMINAL[:2], .15)):
                need.append('not at the charger')
            if name == 'base.dock' and not (self.near(PREDOCK, .12) and self.heading_err(DOCK_YAW) < 10):
                need.append(f'not at the pre-dock pose {r3(PREDOCK)} heading {DOCK_YAW:.0f} deg (base.go_predock first)')
            if name == 'base.dock_charger' and not (self.near(SM.START_NOMINAL[:2], .15) and self.heading_err(SM.START_NOMINAL[2]) < 15):
                need.append('not in front of the charger (base.go_charger first)')
        if tl['arm'] or name in ('verify.pod_in_groove', 'verify.plate_seated'):
            st = self.base_st()
            if tl['arm'] and not (pl.docked and st.get('brake')):
                need.append('arm motions need the base docked at the work dock with the brakes on')
            if tl['arm'] and name not in ('arm.unstow', 'arm.stow', 'vision.locate') and arm_pose == 'stowed':
                need.append('the arm is in the compact driving pose: arm.unstow first')
        if name == 'arm.unstow' and arm_pose != 'stowed':
            need.append(f'arm is already unstowed ({arm_pose})')
        if name == 'arm.stow' and self.plate_held():
            need.append('refused while holding the plate (the stow attitude would drop it)')
        if name == 'base.brake' and not a['on'] and arm_pose != 'stowed':
            need.append('brakes must stay on while the arm is out')
        if name == 'power.charge' and not self.on_charger():
            need.append('not on the charging contacts (base.dock_charger first)')
        # ---- task (from ../gpt_control)
        if name in ('skills.load_pod', 'skills.pull_basket', 'skills.push_basket', 'skills.insert_plate', 'verify.pod_in_groove',
                    'verify.plate_seated') and door < 80:
            need.append(f'door must be open > 80 deg (now {door:.0f})')
        if name in ('skills.open_door', 'skills.load_pod', 'skills.pull_basket', 'skills.push_basket', 'skills.close_door',
                    'skills.press', 'skills.pick_plate', 'vision.locate') and not self.hand_empty():
            need.append(f'hand must be empty (now holding: {pl.holding()})')
        if name == 'skills.load_pod' and bsk > .01:
            need.append(f'basket must be in (now {bsk*1000:.0f} mm out): the pulled-out basket covers the door groove')
        if name == 'skills.close_door':
            if bsk > .01:
                need.append(f'basket must be in (now {bsk*1000:.0f} mm out)')
            if 1 < door < 80:
                need.append(f'door is half open ({door:.0f} deg): open it to its stop first (the close skill re-grasps under the open door)')
        if name == 'skills.insert_plate':
            if not self.plate_held():
                need.append('no plate in hand')
            if bsk < .24:
                need.append(f'basket must be out > 0.24 m (now {bsk:.3f})')
        if name == 'skills.pick_plate':
            if pl.percept is None or not pl.percept['valid']:
                need.append('needs a fresh plate perception (vision.locate("plate") after the plate last moved)')
            if door < 80 or bsk < .24:
                need.append('destination not ready: open the door and pull the basket out before picking (there is no place-back skill)')
        if name == 'verify.plate_seated' and bsk < .2:
            need.append(f'the basket must be out to see the slot (now {bsk:.3f} m)')
        return need

    # ------------------------------------------------------------------ job dispatch (client coroutine side)
    def run_job(self, name, job, args, timeout):
        sup = self.sup
        sup.next_id += 1
        jid = sup.next_id
        sup.pending = dict(id=jid, job=job, name=name, t0=float(self.d.time))
        self.s.rec.follow = name.startswith('base.')
        self.s.mission_label = f'{name}'
        sup.send('MotionPlanner', 'Command', dict(id=jid, skill=name, job=job, args=args), info=dict(id=jid, skill=name))
        sup.set_state(('BASE_' if name.startswith('base.') else 'TASK_') + name.split('.')[-1])
        return self._wait_job(timeout)

    def _wait_job(self, timeout):
        sup = self.sup
        p = sup.pending
        cancelled = False
        while True:
            self.y()
            if p.get('done') is not None:
                sup.pending = None
                sup.set_state('IDLE')
                return p['done']
            if sup.estop['new']:
                sup.estop['new'] = False
                return 'ESTOP'
            if not cancelled and self.d.time - p['t0'] > timeout and not sup.estop['pressed']:
                sup.send('MotionPlanner', 'Cancel', {}, info={})
                cancelled = True

    # ------------------------------------------------------------------ call
    def call(self, name, args):
        t0w, t0s = time.monotonic(), float(self.d.time)
        self.stats['calls'] += 1
        self.s.scanner.trigger(self.last_action)
        res = self._call(name, args if args is not None else {})
        res['sim_time_s'] = round(float(self.d.time), 2)
        res['call_sim_s'] = round(float(self.d.time) - t0s, 2)
        if name != 'done' or not self.ended:
            res['state'] = self.state()
        scoring = res.pop('_scoring_all', {})
        self.history.append(dict(action=name, args=args, result={k: v for k, v in res.items() if k != 'state'}, scoring=scoring,
                                 t=round(t0s, 2)))
        if res.get('ok'):
            self.last_action = name
        else:
            self.last_action = None
        return res, scoring

    def _call(self, name, args):
        tl = T.get(name)
        if tl is None:
            self.stats['invalid'] += 1
            return dict(ok=False, code='INVALID_ACTION', message=f'{name!r} does not exist. Available: {sorted(T)}')
        err = validate(tl['args'], args)
        if err:
            self.stats['invalid'] += 1
            return dict(ok=False, code='INVALID_ARGS', message=err)
        a = {k: args.get(k, s.get('default')) for k, s in tl['args'].items()}
        sf = self._safety(name, a, tl)
        if sf:
            self.stats['rejected_safety'] += 1
            return dict(ok=False, code='REJECTED_SAFETY', reason=sf[0], message=sf[1], suggested_causes=SUGGEST.get(sf[0], []))
        need = self._pre(name, a, tl)
        if need:
            self.stats['rejected_precondition'] += 1
            code = 'LOW_BATTERY' if any(n.startswith('LOW_BATTERY') for n in need) else None
            r = dict(ok=False, code='REJECTED_PRECONDITION', message='; '.join(need))
            if code:
                r.update(reason=code, suggested_causes=SUGGEST[code])
            return r
        try:
            out = self._exec(name, a, tl)
        except Exception as e:  # noqa: BLE001
            import traceback
            self.stats['internal'] += 1
            return dict(ok=False, code='INTERNAL_ERROR', message=f'{type(e).__name__}: {e}'[:300], _trace=traceback.format_exc()[-800:])
        ok = bool(out.pop('_ok', True))
        code = out.pop('code', 'OK' if ok else 'FAILED')
        msg = out.pop('_reason', '')
        warns = out.pop('_warnings', [])
        sc = out.pop('_scoring', {})
        res = dict(ok=ok, code=code, message=msg, measurements={k: v for k, v in out.items() if not k.startswith('_')}, _scoring_all=sc)
        if warns:
            res['warnings'] = warns
        if ok:
            self.stats['ok'] += 1
        elif code == 'ESTOP':
            self.stats['estop'] += 1
        else:
            self.stats['failed'] += 1
        if not ok:
            res['suggested_causes'] = SUGGEST.get(code, [])
        return res

    # ------------------------------------------------------------------ execution
    def _job_result(self, name, done):
        """JobDone -> dict(_ok, code, _reason, measurements..., _scoring)."""
        m = dict(done.get('metrics') or {})
        sc = m.pop('_scoring', {}) or {}
        for k in list(m):                                 # reused mobile jobs report truth for their own logs: scoring only
            if 'true' in k or k.endswith('_err_mm_deg') or k == 'trace':
                sc[k] = m.pop(k)
        ok = bool(done['ok'])
        code = m.pop('code', None)
        out = dict(m)
        out['_scoring'] = sc
        out['_ok'] = ok
        if not ok:
            reason = done.get('reason') or ''
            if code is None:
                if 'cancelled' in reason:
                    code = 'CANCELLED'
                elif 'preload' in reason:
                    code = 'DOCK_PRELOAD_LOW'
                elif 'charging contacts' in reason:
                    code = 'CHARGER_NOT_REACHED'
                elif 'timeout' in reason:
                    code = 'TIMEOUT'
                elif 'no joint path' in reason or 'unreachable' in reason:
                    code = 'UNREACHABLE'
                elif 'bar not grasped' in reason:
                    code = 'GRASP_FAILED'
                else:
                    code = 'SKILL_FAILED'
            out['_reason'] = reason
        if code:
            out['code'] = code
        return out

    def _exec(self, name, a, tl):
        pl, sup = self.pl, self.sup
        if tl['kind'] == 'local':
            return self._local(name, a)
        job, jargs = tl['job'], {}
        if job == 'bm':
            inner = dict(**{'base.undock_charger': 'undock_charger', 'base.go_predock': 'nav_predock', 'base.dock': 'dock',
                            'base.undock': 'undock_cabinet', 'base.go_charger': 'nav_charger', 'base.dock_charger': 'dock_charger',
                            'base.drive_to': 'drive_to'})[name]
            if inner == 'dock' and not self.s.cfg.vblocks:
                inner = 'dock_pos'                       # this kitchen's dock has no V-blocks: position dock (lidar only)
            jargs = dict(inner=inner, offset=(0, 0, 0), vblocks=True)
            if name == 'base.drive_to':
                jargs.update(pose=(a['x'], a['y'], a.get('heading_deg')), direction=1 if a['direction'] == 'forward' else -1)
        elif job == 'wait':
            jargs = dict(s=a['seconds'])
        elif job == 'charge':
            jargs = dict(target_soc=a['target_soc'], max_minutes=a['max_minutes'])
        elif job == 'brake_cmd':
            jargs = dict(on=a['on'])
        elif job == 'press':
            jargs = dict(button=a['button'], tool='stylus')
        elif job == 'g_open_door':
            jargs = dict(hold_at_stop_s=a['hold_at_stop_s'])
        if tl['motion']:
            cap = self.speed_cap
            pl.tcp_cap = cap if cap and cap < V_NOMINAL else None
            pl.base_cap = cap if cap and cap < V_NOMINAL else None
        before = self.panel_readout()
        done = self.run_job(name, job, jargs, tl['timeout'])
        if done == 'ESTOP':
            self.suspended = dict(name=name, job=job, before=before, a=a, t=float(self.d.time))
            return dict(_ok=False, code='ESTOP', _reason=f'e-stop pressed during {name} (planner phase "{pl.phase}"): the arm made a Cat-1 '
                        'stop and holds, the base brakes are on. The skill is SUSPENDED, not finished.')
        return self._post(name, a, done, before)

    def _post(self, name, a, done, before):
        pl = self.pl
        out = self._job_result(name, done)
        if name in ('base.undock_charger', 'base.go_predock', 'base.drive_to') and out['_ok']:
            if not self.mission_started:
                self.start_soc = self.soc()
            self.mission_started = True
        if name == 'base.dock' and out['_ok']:
            f = float(out.get('dock_force_after_brake_N') or 0.)
            if f < 60 and self.s.cfg.vblocks:
                out.update(_ok=False, code='DOCK_PRELOAD_LOW', _reason=f'dock plate force {f:.0f} N after braking (need ~100 N)')
            q = out.get('localization', {})
            if q.get('strip_y_fixes_last_2s', 0) == 0:
                out['_warnings'] = ['LOCALIZATION_Y_UNOBSERVED: since docking the rear lidar has not seen the reflective strip, so the base y '
                                    '(along the counter) is dead-reckoned from the approach and not measured; the arm uses this estimate']
        if name == 'base.dock_charger' and out['_ok'] and float(out.get('charge_force_N') or 0.) < 10:
            out.update(_ok=False, code='CHARGER_NOT_REACHED', _reason='charge contact force below 10 N')
        if name == 'skills.press':
            out = self._press_verdict(a['button'], out, before)
        if name in ('verify.pod_in_groove', 'verify.plate_seated'):
            key = 'pod_in_groove' if 'pod' in name else 'plate_seated'
            self.verified[key] = dict(value=bool(out['_ok']), at_s=round(float(self.d.time), 1))
        if name == 'vision.locate' and getattr(pl, 'last_overlay', None) and Path(pl.last_overlay).exists():
            self.pending_images.append(str(pl.last_overlay))
        if name == 'skills.pick_plate' and getattr(pl, 'last_overlay', None) and Path(pl.last_overlay).exists():
            self.pending_images.append(str(pl.last_overlay))
        # any later change at the slot / groove makes an earlier verification stale
        if name in ('skills.load_pod',) and out['_ok']:
            self.verified.pop('pod_in_groove', None)
        if name in ('skills.insert_plate',):
            self.verified.pop('plate_seated', None)
        return out

    def _press_verdict(self, button, out, before):
        if not out.get('_ok', True) and out.get('code') not in (None,):
            return out
        t1 = float(self.d.time)
        after = self.panel_readout()
        while float(self.d.time) - t1 < 2.0:
            self.y()
            after = self.panel_readout()
            if after != before:
                break
        out.update(panel_before=before, panel_after=after)
        if not out.get('ft_stop', True):
            out.update(_ok=False, code='PRESS_NO_CONTACT', _reason='no 3 N contact felt by the wrist F/T')
        elif after == before:
            out.update(_ok=False, code='PANEL_NO_RESPONSE', _reason=f'panel readout unchanged after touching {button.upper()}: {after}')
        out['_scoring'] = dict(out.get('_scoring') or {}, panel_events=self.s.panel.events[-3:])
        for k in ('f_axis_at_stop', 'conductive_peak_N', 'nonconductive_peak_N', 'nonconductive_dwell_s'):
            if k in out:
                out['_scoring'][k] = out.pop(k)
        return out

    def _local(self, name, a):
        pl, sup = self.pl, self.sup
        if name == 'robot.get_state':
            return {}
        if name == 'robot.wait':
            t0 = float(self.d.time)
            prev = sup.state
            sup.set_state('WAIT' if prev != 'ESTOP' else prev)
            while float(self.d.time) - t0 < a['seconds']:
                self.y()
            if sup.state == 'WAIT':
                sup.set_state('IDLE')
            return dict(waited_s=a['seconds'])
        if name == 'robot.set_speed_limit':
            self.speed_cap = float(a['mps']) if a['mps'] < V_NOMINAL else None
            self.s.api_speed_cap = self.speed_cap
            return dict(speed_limit_mps=a['mps'], applies_to='arm TCP and base driving')
        if name == 'power.get_soc':
            b = sup.bms or {}
            return dict(soc=round(self.soc(), 4), V=round(float(b.get('V', 0.)), 2), I_A=round(float(b.get('I', 0.)), 2),
                        charging=bool(b.get('charging')))
        if name == 'base.get_state':
            return self.state()['base']
        if name == 'dishwasher.read_sensors':
            return dict(door_deg=round(pl.door_deg(), 1), basket_out_m=round(pl.basket_m(), 3), panel_led_readout=self.panel_readout(),
                        note='door / basket: SIMULATED sensor access (truth + noise); panel: LED readout')
        if name == 'vision.capture':
            p = self.capture_wall(f'capture{self.n_img}')
            self.pending_images.append(p)
            return dict(image=Path(p).name, note='image attached to your next turn')
        if name == 'robot.resume':
            sus = self.suspended
            self.sup.send('ArmController', 'Resume', {}, info={})
            self.sup.send('MotionPlanner', 'Resume', {}, info={})
            self.sup.estop.update(pressed=False, released=False, ack=False)
            self.sup.set_state(('BASE_' if sus['name'].startswith('base.') else 'TASK_') + sus['name'].split('.')[-1])
            done = self._wait_job(T[sus['name']]['timeout'] + 600)
            if done == 'ESTOP':
                return dict(_ok=False, code='ESTOP', _reason='e-stop pressed again; the skill is suspended')
            self.suspended = None
            out = self._post(sus['name'], sus['a'], done, sus['before'])
            out['resumed_skill'] = sus['name']
            return out
        if name == 'robot.abort_skill':
            sus = self.suspended
            self.sup.send('ArmController', 'Resume', {}, info={})
            self.sup.send('MotionPlanner', 'Resume', {}, info={})
            self.sup.send('MotionPlanner', 'Cancel', {}, info={})
            self.sup.estop.update(pressed=False, released=False, ack=False)
            done = self._wait_job(60)
            self.suspended = None
            return dict(aborted_skill=sus['name'], _scoring=dict(job_done=str(done)[:200]))
        if name == 'robot.ask_human':
            self.ended = dict(kind='ASK_HUMAN', success=False, summary=a['question'])
            return dict(acknowledged=True, note='mission paused for a human')
        if name == 'done':
            if not a['success']:
                self.ended = dict(kind='STOPPED', success=False, summary=a['summary'])
                return dict(acknowledged=True)
            missing = self.verify_mission()
            gt = self.s.mission_score(self)                      # scoring only, never returned to GPT
            self.claims.append(dict(t=round(float(self.d.time), 1), verified=not missing, missing=missing,
                                    gt_success=gt['success'], gt_criteria=gt['criteria']))
            if missing:
                return dict(_ok=False, code='MISSION_NOT_VERIFIED', _reason='done(success=true) refused: ' + '; '.join(missing),
                            missing=missing)
            self.ended = dict(kind='DONE_VERIFIED', success=True, summary=a['summary'])
            return dict(acknowledged=True, verified=True)
        raise ValueError(name)

    def verify_mission(self):
        pl = self.pl
        miss = []
        p = self.panel_readout() or {}
        if not (p.get('state') == 'RUNNING' and p.get('program') == 'quick'):
            miss.append(f'panel not RUNNING QUICK (readout {p})')
        if abs(pl.door_deg()) > 1.5:
            miss.append('door not closed')
        if pl.basket_m() > .01:
            miss.append('basket not in')
        for k in ('pod_in_groove', 'plate_seated'):
            v = self.verified.get(k)
            if not v or not v['value']:
                miss.append(f'{k} not verified (call verify.{k} while the door is open)')
        if pl.arm_pose_name() != 'stowed':
            miss.append('arm not stowed')
        b = self.sup.bms or {}
        if not (self.on_charger() and b.get('charging')):
            miss.append('robot not on the charger / not charging')
        return miss

    # ------------------------------------------------------------------ images (rendered by the main thread)
    def capture_wall(self, tag):
        s = self.s
        out = s.outdir / 'gpt_images'
        out.mkdir(parents=True, exist_ok=True)
        path = out / f'{tag}.png'
        flag = dict(done=False)

        def job():
            from PIL import Image
            if s.perception.cam is None:
                s.perception.cam = MS.PC.Camera(s.m)
            rgb, _, _ = s.perception.cam.capture(s.d, np.random.default_rng(self.n_img))
            Image.fromarray(rgb).save(path)
            flag['done'] = True
        s.main_queue.append(job)
        self.n_img += 1
        while not flag['done']:
            self.y()
        return str(path)


# ---------------------------------------------------------------- docs (prompt + API.md)
def _sig(t):
    parts = []
    for k, s in t['args'].items():
        ty = s['type'] + (f' {s["enum"]}' if 'enum' in s else '') + (f' [{s.get("min")}..{s.get("max")}]' if 'min' in s else '')
        parts.append(f'{k}{"" if s.get("required") else "?"}: {ty}' + (f' = {s["default"]}' if s.get('default') is not None else ''))
    return f'{t["name"]}({", ".join(parts)})'


def prompt_docs():
    return '\n'.join(f'- {_sig(t)}: {t["desc"]}' for t in T.values())


def facts(vblocks=True):
    x, y, yaw = SM.dock_base_pose()
    dock = ('rear in 2 V-blocks on the cabinet plinth, 100 N dock preload' if vblocks else
            'NO V-blocks in this kitchen: base.dock reverses to the dock pose using the lidar estimate only (no preload, no mechanical centring)')
    f = [
        'World frame (m): the kitchen counter front edge is the line x=0 (counter top z=0.90, cabinets below it); the robot drives on the '
        'floor at x>0. +y runs along the counter. A countertop dishwasher stands on the counter at y=0; the wall camera looks down at '
        'the counter right of the dishwasher (plate, pod dish).',
        f'Charger: base pose ({SM.CHARGER_POSE[0]:.2f}, {SM.CHARGER_POSE[1]:.2f}) heading {SM.CHARGER_POSE[2]:.0f} deg; the robot backs onto '
        f'its contacts. The robot starts ~8 cm in front of it.',
        f'Work dock (in front of the dishwasher): base pose ({x:.3f}, {y:.3f}) heading {yaw:.0f} deg; {dock}. '
        f'Pre-dock pose {r3(PREDOCK)} (0.55 m out).',
        'Localization: two corner lidars fit the cabinet-front line (x, heading) and a 50 mm reflective strip on the cabinet at y=0.42 '
        '(y). The arm plans with the MEASURED base pose; state.base.localization tells whether y is currently measured.',
        'Base: differential drive, max 0.5 m/s; spring brakes. Battery 30 Ah LFP (1.5 kWh); one mission uses ~10 Wh (~0.6 % SoC) plus '
        '~0.03 Wh per second of idling. Mission start needs SoC >= 30 %. Charging nets ~+0.5 %/min.',
        'Arm: Franka FR3 on the base. Compact driving pose = "stowed"; "park" = camera-clear pose over the counter. Skills run their own '
        'closed loops (IK 100 Hz, F/T admittance); you never command joints.',
        'Objects: flat 8-inch plate (0.45 kg) on the counter right of the dishwasher; pod (detergent tablet) in a small dish on the counter; '
        'pod goes into the groove on the inner face of the opened door; the plate goes upright into the lower-basket slot.',
        PANEL_DOC + ' Pads: ' + ', '.join(dw.BUTTONS) + '.',
        'Safety: an e-stop and a person scanner act in hardware; the API refuses motions while the e-stop is active and refuses motions '
        'without a <= 0.25 m/s cap while a person is flagged. A protective field stops the base if someone is in the driving path.',
    ]
    return '\n'.join('- ' + s for s in f)


def write_api_md(path=ROOT / 'API.md'):
    rows = ['| API call | Real component | ROS 2 interface (proposed) | Served by |', '|---|---|---|---|']
    for t in T.values():
        rows.append(f'| `{_sig(t)}` | {t["real"]} | `{t["ros"]}` | {t["computer"]} |')
    codes = '\n'.join(f'- `{k}`: ' + '; '.join(v) for k, v in SUGGEST.items())
    txt = f"""# Mobile robot API (what GPT can call)

Generated by `mobile_api.write_api_md()` from the same registry that produces the prompt text and the argument validator.

## Contract
- **Call:** `{{"action": "<name>", "args": "<JSON object>"}}`; the decision schema restricts `action` to the names below.
- **Result:** `{{ok, code, message, measurements, warnings?, suggested_causes?, state, sim_time_s, call_sim_s}}`. Ground-truth
  scoring numbers are stripped before the result reaches GPT and stored separately in the run log.
- **Validation order:** unknown action -> `INVALID_ACTION`; schema -> `INVALID_ARGS`; safety gate -> `REJECTED_SAFETY`
  (e-stop not released/acknowledged, person flagged without a <= 0.25 m/s cap, drive target outside the floor area
  x {FLOOR['x']} y {FLOOR['y']}); preconditions -> `REJECTED_PRECONDITION`. Only then does anything move.
- **Measured verdicts:** finger width + wrist F/T (pod / plate in hand, touchdown), dock-plate and charge-contact force, lidar
  fix counts (is y measured?), BMS, panel LED readout before/after each touch. Door angle / basket travel are SIMULATED
  appliance sensors (truth + 0.3 deg / 2 mm noise). `verify.*` are SIMULATED verification sensors (truth + noise).
- **Mission done:** `done(success=true)` is refused unless the API itself verifies: panel RUNNING QUICK, door closed, basket in,
  `verify.pod_in_groove` and `verify.plate_seated` passed (after the last load / insert), arm stowed, on the charger and charging.
- **E-stop:** the hardware chain stops the arm (Cat 1) and brakes the base; the running skill is SUSPENDED and the call returns
  `ESTOP`. Motions are refused until the e-stop is released and the operator has acknowledged; then `robot.resume()` continues
  the suspended skill (or `robot.abort_skill()`).

## Mobile preconditions (`REJECTED_PRECONDITION`)
| Call | Precondition |
|---|---|
| every base motion | arm stowed (driving pose), hand empty; when docked only `base.undock` |
| `base.undock` | docked, dishwasher door closed (< 2 deg), arm stowed |
| `base.undock_charger`, `base.go_predock`, `base.drive_to` before the mission has started | SoC >= {SOC_START_MIN:.2f} (`LOW_BATTERY`) |
| `base.dock` | at the pre-dock pose; `base.dock_charger`: in front of the charger |
| every arm motion and skill | docked at the work dock with the brakes on; task skills need `arm.unstow` first |
| any motion while a person is flagged | speed cap <= 0.25 m/s (`REJECTED_SAFETY`), applied to the arm TCP and the base |
| `power.charge` | on the charging contacts |
| task ordering (from ../gpt_control) | door > 80 deg for pod/basket/insert/verify; basket in for the pod; hand empty; fresh perception + destination ready for the pick; plate in hand + basket out for the insert; basket in + door at the stop for the close |

## Calls
{chr(10).join(rows)}

## Error codes and suggested causes
{codes}
"""
    path.write_text(txt)
    return path


if __name__ == '__main__':
    print(write_api_md())
    print(prompt_docs())
    print(facts())
