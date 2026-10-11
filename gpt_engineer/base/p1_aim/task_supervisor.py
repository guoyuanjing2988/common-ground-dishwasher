"""TaskSupervisor (10 Hz): explicit mission state machine. It sends jobs to the MotionPlanner, waits for JobDone,
applies per-state timeouts and retries, checks the battery before leaving the charger, verifies the dishwasher panel
through the LED readout after each touch (retry up to 2x), handles e-stop (pause -> wait for release + operator
acknowledgement -> resume or abort) and runs an abort path (safe stow -> undock -> drive home -> charger) on failure.
Every transition is logged (and sent on the bus as a StateTransition message)."""
from __future__ import annotations

import math

import skills_api as API
from nodes import Node

ARM_STEPS = ['perceive_plate', 'open_door', 'load_pod', 'pull_basket', 'pick_plate', 'insert_plate', 'push_basket', 'close_door']
BASE_JOBS = {'localize', 'undock_charger', 'nav_predock', 'dock', 'undock_cabinet', 'nav_charger', 'dock_charger',
             'localize_docked'}


def S_(state, skill, args=None, timeout=120., retries=0, expect=None, label=None):
    """One mission state = one skill call (skills_api) + client-side policy (timeout, retries, expected panel readout)."""
    if skill is None:
        return dict(state=state, skill=None, job=None, args={}, timeout=timeout, retries=retries, expect=expect, label=label or state)
    rq = API.request(skill, args)
    return dict(state=state, skill=skill, job=rq['job'], args=rq['args'], timeout=timeout, retries=retries, expect=expect,
                label=label or state)


def mission_plan(kind='full', offset=(0, 0, 0), vblocks=True, tool='stylus'):
    panel = [S_('PANEL_POWER', 'skills.press', dict(button='power', tool=tool), 60, 2, expect=('STANDBY', None), label='Panel: POWER -> STANDBY'),
             S_('PANEL_QUICK', 'skills.press', dict(button='quick', tool=tool), 60, 2, expect=('SELECTED', 'quick'), label='Panel: QUICK -> selected'),
             S_('PANEL_START', 'skills.press', dict(button='power', tool=tool), 60, 2, expect=('RUNNING', 'quick'), label='Panel: POWER -> RUNNING')]
    arm = [S_(f'TASK_{s.upper()}', 'vision.locate' if s == 'perceive_plate' else f'skills.{s}', None, 450. if s in ('perceive_plate', 'pick_plate') else 180.,
              label=f'Task: {s.replace("_", " ")}') for s in ARM_STEPS]
    if kind == 'docked':
        return ([S_('LOCALIZE_DOCKED', 'base.localize_docked', timeout=20, label='Localize at the (offset) dock'),
                 S_('UNSTOW_ARM', 'arm.unstow', timeout=30, label='Unstow arm')] + arm + panel +
                [S_('STOW_ARM', 'arm.stow', timeout=60, label='Stow arm')])
    return ([S_('LOCALIZE', 'base.localize', timeout=20, label='Localize (lidar)'),
             S_('CHECK_BATTERY', None, label='Check battery'),
             S_('UNDOCK_CHARGER', 'base.undock_charger', timeout=30, label='Undock from charger'),
             S_('NAV_PREDOCK', 'base.go_predock', dict(offset=offset), 90, label='Drive to pre-dock pose'),
             S_('DOCK', 'base.dock', dict(offset=offset, vblocks=vblocks), 90, 1, label='Dock: reverse into V-blocks, 100 N, brake'),
             S_('UNSTOW_ARM', 'arm.unstow', timeout=30, label='Unstow arm')] + arm + panel +
            [S_('STOW_ARM', 'arm.stow', timeout=60, label='Stow arm'),
             S_('UNDOCK', 'base.undock', timeout=30, label='Release brakes, undock'),
             S_('NAV_CHARGER', 'base.go_charger', timeout=90, label='Drive back to the charger'),
             S_('DOCK_CHARGER', 'base.dock_charger', timeout=60, label='Dock onto charging contacts'),
             S_('CHARGE_CHECK', 'robot.wait', dict(s=2.), 10, label='Charging?'),
             S_('FINAL_STATE', 'robot.get_state', timeout=5, label='robot.get_state'),
             S_('FINAL_SOC', 'power.get_soc', timeout=5, label='power.get_soc')])


def door_open_panel_plan():
    """Panel-logic demo: open the door, then try POWER, QUICK, POWER(start) with the door open -> START is refused."""
    return [S_('LOCALIZE_DOCKED', 'base.localize_docked', timeout=20, label='Localize at the dock'),
            S_('UNSTOW_ARM', 'arm.unstow', timeout=30, label='Unstow arm'),
            S_('TASK_OPEN_DOOR', 'skills.open_door', None, 180, label='Task: open door'),
            S_('PANEL_POWER', 'skills.press', dict(button='power'), 60, 0, expect=('STANDBY', None), label='Panel: POWER -> STANDBY'),
            S_('PANEL_QUICK', 'skills.press', dict(button='quick'), 60, 0, expect=('SELECTED', 'quick'), label='Panel: QUICK -> selected'),
            S_('PANEL_START', 'skills.press', dict(button='power'), 60, 0, expect=('RUNNING', 'quick'),
               label='Panel: POWER with the door OPEN (expect refusal)')]


class TaskSupervisor(Node):
    name, rate = 'TaskSupervisor', 10

    def __init__(self, sysm, plan, soc_min=.30, charge_hold_s=30.):
        super().__init__(sysm)
        self.plan = plan
        self.i = -1
        self.state = 'BOOT'
        self.t_state = 0.
        self.transitions = []
        self.results = []
        self.pending = None
        self.attempt = 0
        self.soc = None
        self.bms = None
        self.panel = None
        self.estop = False
        self.estop_released = False
        self.resume_to = None
        self.finished = False
        self.outcome = None
        self.soc_min = soc_min
        self.charge_hold_s = charge_hold_s
        self.abort_path = False
        self.next_id = 0
        self.wait_panel_until = None
        self.planner_status = {}
        self.estop_log = []

    # ------------------------------------------------------------------
    def go(self, t, state, why=''):
        kd = getattr(self.sys.planner, 'keyframes_dir', None) if hasattr(self.sys, 'planner') else None
        if kd is not None and self.state in ('NAV_PREDOCK', 'DOCK', 'PANEL_START', 'NAV_CHARGER', 'DOCK_CHARGER', 'ESTOP'):
            path = kd / f'base_{len(self.transitions):02d}_{self.state.lower()}_end.png'
            self.sys.main_queue.append(lambda: self.sys.rec.still(self.sys.d, path))
        self.transitions.append(dict(t=round(t, 3), frm=self.state, to=state, why=why[:160]))
        self.send('Log', 'StateTransition', None, info=dict(frm=self.state, to=state, why=why[:100]))
        self.state, self.t_state = state, t

    def dispatch(self, t, entry):
        self.next_id += 1
        self.pending = dict(id=self.next_id, entry=entry, t0=t)
        self.sys.rec.follow = entry['job'] in BASE_JOBS
        self.sys.mission_label = entry['label']
        self.send('MotionPlanner', 'Command', dict(id=self.next_id, skill=entry['skill'], job=entry['job'], args=entry['args']),
                  info=dict(id=self.next_id, skill=entry['skill'], **{k: v for k, v in entry['args'].items() if k in ('button', 'tool')}))

    def advance(self, t, why=''):
        self.i += 1
        self.attempt = 0
        if self.i >= len(self.plan):
            self.finish(t, 'ABORTED' if self.abort_path else 'DONE', why)
            return
        e = self.plan[self.i]
        self.go(t, e['state'], why)
        if e['state'] == 'CHECK_BATTERY':
            return
        self.dispatch(t, e)

    def finish(self, t, outcome, why=''):
        self.go(t, outcome, why)
        self.outcome = outcome
        self.send('MotionPlanner', 'Command', dict(id=-1, job='shutdown'), info=dict(job='shutdown'))
        self.finished = True

    def start_abort(self, t, why):
        if self.abort_path:
            self.finish(t, 'FAILED', 'abort path failed: ' + why)
            return
        self.abort_path = True
        docked = any(r['job'] == 'dock' and r['ok'] for r in self.results)
        undocked_after = any(r['job'] == 'undock_cabinet' and r['ok'] for r in self.results)
        path = [S_('ABORT_SAFE_STOW', 'arm.safe_stow', timeout=90, label='ABORT: safe stow')]
        if docked and not undocked_after:
            path.append(S_('ABORT_UNDOCK', 'base.undock', timeout=30, label='ABORT: undock'))
        if docked:
            path += [S_('ABORT_NAV_CHARGER', 'base.go_charger', timeout=90, label='ABORT: drive back'),
                     S_('ABORT_DOCK_CHARGER', 'base.dock_charger', timeout=60, label='ABORT: charger')]
        self.plan = self.plan[:self.i + 1] + path
        self.advance(t, 'abort: ' + why)

    # ------------------------------------------------------------------
    def tick(self, t):
        for msg in self.recv():
            ty = msg.typ
            if ty == 'JobDone' and self.pending and msg.data['id'] == self.pending['id']:
                self.pending['done'] = msg.data
            elif ty == 'BMS':
                self.bms = msg.data
                self.soc = msg.data['soc']
            elif ty == 'PanelReadout':
                self.panel = msg.data
            elif ty == 'PlannerStatus':
                self.planner_status = msg.data
            elif ty == 'EStop':
                if not self.estop:
                    self.estop = True
                    self.estop_log.append(dict(t=round(t, 3), event='estop', during=self.state))
                    self.resume_to = self.state
                    self.send('MotionPlanner', 'Pause', {}, info={})
                    self.go(t, 'ESTOP', 'e-stop pressed (BaseMCU)')
            elif ty == 'EStopReleased':
                self.estop_released = True
                self.estop_log.append(dict(t=round(t, 3), event='released'))
            elif ty == 'OperatorAck' and self.state == 'ESTOP' and self.estop_released:
                self.estop = self.estop_released = False
                self.estop_log.append(dict(t=round(t, 3), event=f'operator {msg.data["action"]}'))
                if msg.data['action'] == 'resume':
                    self.send('ArmController', 'Resume', {}, info={})
                    self.send('MotionPlanner', 'Resume', {}, info={})
                    self.go(t, self.resume_to, 'operator acknowledged: resume interrupted job')
                else:
                    self.send('ArmController', 'Resume', {}, info={})
                    self.send('MotionPlanner', 'Resume', {}, info={})
                    self.send('MotionPlanner', 'Cancel', {}, info={})
                    self.go(t, self.resume_to, 'operator acknowledged: abort')
        if self.finished or self.state == 'ESTOP':
            return
        if self.state == 'BOOT':
            if self.bms is not None:
                self.advance(t, 'boot complete')
            return
        if self.state == 'CHECK_BATTERY':
            if self.soc is None:
                return
            if self.soc < self.soc_min:
                self.go(t, 'LOW_BATTERY', f'SoC {self.soc:.3f} < {self.soc_min} -> go charge instead of the mission')
                self.plan = self.plan[:self.i + 1] + [S_('GO_CHARGE', 'base.dock_charger', timeout=60, label='Low battery: dock onto charger'),
                                                     S_('CHARGING', 'robot.wait', dict(s=self.charge_hold_s), self.charge_hold_s + 10,
                                                        label='Charging')]
                self.low_battery = True
            self.advance(t, f'SoC {self.soc:.3f}')
            return
        if self.pending is None:
            return
        e = self.pending['entry']
        if self.wait_panel_until is not None:
            ok = self.panel is not None and self.panel['state'] == e['expect'][0] and \
                (e['expect'][1] is None or self.panel['program'] == e['expect'][1])
            if ok:
                self.wait_panel_until = None
                self.results[-1]['panel_after'] = dict(state=self.panel['state'], program=self.panel['program'])
                self.pending = None
                self.advance(t, f'panel readout {self.panel["state"]}')
            elif t > self.wait_panel_until:
                self.wait_panel_until = None
                self.results[-1]['panel_after'] = dict(state=self.panel and self.panel['state'])
                self._retry_or_fail(t, e, f'panel did not show {e["expect"]} (readout {self.panel and self.panel["state"]})')
            return
        done = self.pending.get('done')
        if done is None:
            if t - self.pending['t0'] > e['timeout'] + (60. if self.estop_log else 0.):
                self.results.append(dict(state=e['state'], job=e['job'], ok=False, reason='timeout', t0=self.pending['t0'], t1=t))
                self.finish(t, 'FAILED', f'{e["state"]}: timeout {e["timeout"]} s')
            return
        res = API.result(done)                       # structured skill result: ok / error / measurements
        self.results.append(dict(state=e['state'], skill=e['skill'], job=e['job'], ok=res['ok'], reason=res['error'] or '',
                                 metrics=res['measurements'], t0=round(done['t0'], 3), t1=round(done['t1'], 3), attempt=self.attempt))
        if not done['ok']:
            self._retry_or_fail(t, e, done['reason'])
            return
        if e['expect'] is not None:
            self.wait_panel_until = t + 2.0                  # LED readout path: panel 100 ms + perception
            return
        if e['job'] == 'wait' and e['state'] in ('CHARGE_CHECK', 'CHARGING'):
            self.results[-1]['charging'] = bool(self.sys.charging)
        self.pending = None
        self.advance(t, f'{e["state"]} ok')

    def _retry_or_fail(self, t, e, why):
        if self.attempt < e['retries']:
            self.attempt += 1
            self.go(t, e['state'], f'retry {self.attempt}/{e["retries"]}: {why}')
            if e['job'] == 'dock':
                self.next_id += 1
                self.dispatch(t, S_('DOCK_RETRY_BACKOFF', 'base.go_predock', e['args'], 60, label='Dock retry: back to pre-dock'))
                self.pending['then'] = e
                self.pending['entry'] = dict(e, expect=None)
                self.plan.insert(self.i + 1, dict(e, retries=0))
                return
            self.dispatch(t, e)
            return
        self.pending = None
        self.start_abort(t, f'{e["state"]} failed: {why}')
