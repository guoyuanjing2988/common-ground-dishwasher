"""Skill / API layer of the mobile robot: the ONLY interface between a task-level client (today the TaskSupervisor state
machine; later a GPT / `codex exec` planner) and the real-time controllers.

Call model: name + args -> structured result
    request  : Command{id, skill, job, args}            (bus message to the MotionPlanner)
    result   : {skill, ok, error, measurements, t0, t1}  (from the planner's JobDone, see `result()`)
Calls are asynchronous on the bus (the 10 Hz client keeps running, handles e-stop, timeouts); a synchronous wrapper
for an LLM client simply sends the request and waits for the result.

Skill names follow gpt_control/API.md where they overlap (skills.*, vision.locate, robot.get_state);
base.* and power.* are the mobile additions requested for that layer. The skill sequence lives in the client
(task_supervisor.mission_plan), never inside the controllers.
"""
from __future__ import annotations

SKILLS = {
    # ---------------- base
    'base.localize':        dict(job='localize', doc='wait for lidar fixes; returns the estimated and true base pose'),
    'base.localize_docked': dict(job='localize_docked', doc='brakes on, average 3 s of lidar fixes at the current (dock) pose'),
    'base.undock_charger':  dict(job='undock_charger', doc='release brakes, drive 0.30 m forward off the charging contacts'),
    'base.drive_to':        dict(job='drive_to', doc='args pose=(x, y, heading_deg), direction=+1/-1: rotate, drive the line, rotate'),
    'base.go_predock':      dict(job='nav_predock', doc='drive to the pre-dock pose 0.55 m out of the work dock (args offset)'),
    'base.dock':            dict(job='dock', doc='reverse into the V-blocks until the dock-plate preload (100 N), brake'),
    'base.undock':          dict(job='undock_cabinet', doc='release brakes, drive 0.50 m out of the work dock'),
    'base.go_charger':      dict(job='nav_charger', doc='drive back in reverse to 8 cm in front of the charger'),
    'base.dock_charger':    dict(job='dock_charger', doc='reverse onto the charging contacts (15 N), brake'),
    'base.brake':           dict(job='brake_cmd', doc='args on=True/False: engage / release the wheel brakes'),
    'base.get_state':       dict(job='get_state', doc='base pose estimate, true pose (scoring), brake, dock / charge force'),
    'power.get_soc':        dict(job='get_soc', doc='BMS state of charge, voltage, current, charging'),
    'robot.get_state':      dict(job='get_state', doc='base + arm joint state, gripper opening, panel readout'),
    'robot.wait':           dict(job='wait', doc='args s: idle (e.g. charging)'),
    # ---------------- arm
    'arm.unstow':           dict(job='unstow', doc='solve the park pose for the MEASURED base pose, move there'),
    'arm.stow':             dict(job='stow', doc='park, then the compact driving pose'),
    'arm.safe_stow':        dict(job='safe_stow', doc='abort: open hand, back off up/out, stow'),
    # ---------------- task skills (the 9 steps of ../full_pipeline)
    'vision.locate':        dict(job='arm_step', fixed=dict(step='perceive_plate'), doc='camera + Codex + depth fit of the plate'),
    'skills.open_door':     dict(job='arm_step', fixed=dict(step='open_door'), doc='grasp the bar, follow the hinge arc to 90 deg'),
    'skills.load_pod':      dict(job='arm_step', fixed=dict(step='load_pod'), doc='pod from the dish into the door groove'),
    'skills.pull_basket':   dict(job='arm_step', fixed=dict(step='pull_basket'), doc='lower basket out to the end stop'),
    'skills.pick_plate':    dict(job='arm_step', fixed=dict(step='pick_plate'), doc='edge push, re-perceive, pinch, tilt, lift'),
    'skills.insert_plate':  dict(job='arm_step', fixed=dict(step='insert_plate'), doc='carry + insert upright (slot: 2nd from the front)'),
    'skills.push_basket':   dict(job='arm_step', fixed=dict(step='push_basket'), doc='basket back in'),
    'skills.close_door':    dict(job='arm_step', fixed=dict(step='close_door'), doc='re-grasp under the open door, close to < 1 deg'),
    'skills.press':         dict(job='press', doc='args button, tool=stylus|finger: touch a panel pad, F/T-guided, hold 0.5 s'),
}


def request(skill, args=None):
    """Command payload for a skill call (validated)."""
    if skill not in SKILLS:
        raise KeyError(f'unknown skill {skill!r}')
    s = SKILLS[skill]
    a = dict(s.get('fixed', {}))
    a.update(args or {})
    return dict(skill=skill, job=s['job'], args=a)


def result(jobdone):
    """Structured result of a skill call from the planner's JobDone message."""
    return dict(skill=jobdone.get('skill'), ok=bool(jobdone['ok']), error=jobdone.get('reason') or None,
                measurements=jobdone.get('metrics', {}), t0=jobdone.get('t0'), t1=jobdone.get('t1'))
