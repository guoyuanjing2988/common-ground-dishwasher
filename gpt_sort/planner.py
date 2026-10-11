"""GPT planner: wall-camera image (+ a context image from a room camera) + basket layout + program manual -> per-item
decisions and a wash program, in ONE structured `codex exec --output-schema` call.

GPT never sees ground truth: only the two rendered images, the goal text, the basket layout and the appliance's program
descriptions (what a user manual says). The ground-truth program RULE (thresholds) is not given to GPT.
"""
from __future__ import annotations

import json

import gpt

CATEGORIES = ['plate', 'bowl', 'mug', 'cutting_board', 'knife', 'pan', 'flask', 'other']
SLOTS = ['P1', 'P2', 'P3', 'P4', 'B1', 'B2', 'B3', 'defer', 'none']
POS = ('P1', 'P2', 'P3', 'P4', 'B1', 'B2', 'B3')
PROGRAMS = ['eco', 'intensive', 'quick', 'dry', 'fruit']

SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["items", "program", "program_reason", "summary"],
    "properties": {
        "items": {"type": "array", "items": {
            "type": "object", "additionalProperties": False,
            "required": ["id", "box", "category", "description", "dishwasher_safe", "soil", "greasy", "reason", "slot", "order"],
            "properties": {
                "id": {"type": "string", "description": "short id you assign, e.g. A, B, C"},
                "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "description": "[x_min,y_min,x_max,y_max] tight pixel box in IMAGE 1 (wall camera)"},
                "category": {"type": "string", "enum": CATEGORIES},
                "description": {"type": "string"},
                "dishwasher_safe": {"type": "boolean"},
                "soil": {"type": "string", "enum": ["light", "medium", "heavy"]},
                "greasy": {"type": "boolean"},
                "reason": {"type": "string", "description": "why safe / not safe, and why this slot"},
                "slot": {"type": "string", "enum": SLOTS},
                "order": {"type": "integer", "description": "1, 2, 3 ... loading order for items with a P/B slot; 0 otherwise"}}}},
        "program": {"type": "string", "enum": PROGRAMS},
        "program_reason": {"type": "string"},
        "summary": {"type": "string", "description": "one or two sentences for the user, in the language of the goal"}}}

GOALS = {
    'en': "Please load the dishwasher with whatever on the counter can go in it, and start a suitable wash program.",
    'zh': "请把台面上能放进洗碗机的东西装进去，然后选一个合适的洗涤程序启动。",
}

BASKET = """DISHWASHER (small countertop machine, 43 cm wide; drop-down door, one pull-out lower basket):
- Plate rack (right half of the lower basket): P1 (back), P2, P3, P4 (front). Each holds ONE dinner plate standing
  upright between the tines (every other tine gap, so plates do not touch).
- Bowl / cup area (left side of the lower basket, one row of tines leaning toward the side wall): B1, B2, B3.
  Each holds ONE bowl or ONE mug, upside down / leaning so water drains. Plates do not fit there.
- No cutlery basket; the upper tray is too low (6 cm) for any of these items.
Who loads: the robot arm can itself place a plate only in P4 (the front plate position), and it can only pick up a
plate that lies flat close to the dishwasher (its centre about 15-40 cm from the dishwasher's side wall) in the front
half of the counter, with nothing between the plate and the counter's front edge (it slides the plate to the edge to grip
its rim).
Every other item with a position is handed to the person, who puts it into the position you chose.
Rules: every position holds at most one item. A dishwasher-safe item that does not fit in this load gets slot "defer"
(it waits for the next load). An item that must not go in a dishwasher gets slot "none"."""

PROGRAMS_MANUAL = """WASH PROGRAMS (buttons on the panel, from the user manual):
- eco (节能洗): everyday dishes with normal soil; most economical, long cycle.
- intensive (强力洗): heavily soiled dishes, dried-on or burnt food, greasy dishes and pots; hottest, longest.
- quick (快速洗): lightly soiled dishes that were just used, small loads; short, lower temperature, not for grease.
- dry (烘干/除菌): drying / sterilising only, no washing.
- fruit (水果洗): fruit and vegetables only."""


def prompt(goal_lang, w, h):
    goal = GOALS[goal_lang]
    lang_note = "Write the description / reason / program_reason / summary fields in Chinese." if goal_lang == 'zh' else \
        "Write the text fields in English."
    return f"""You are the planning module of a home robot (a robot arm next to a countertop dishwasher).
User's request: "{goal}"

IMAGE 1 ({w}x{h} px, x right, y down, origin top-left) is from a camera on the back wall of the kitchen, looking over the
countertop toward the robot. In IMAGE 1 the counter's FRONT edge (where the robot stands) is the far edge, toward the TOP
of the image; the back wall is behind the camera (bottom of the image); the dishwasher is at the right edge of the image. IMAGE 2 is a context view of the same counter from a
camera in the room corner (use it only to recognise objects; give all boxes in IMAGE 1 coordinates).

{BASKET}

{PROGRAMS_MANUAL}

Tasks:
1. List EVERY loose object on the countertop (ignore the robot, the dishwasher, walls and floor). For each: a tight box in
   IMAGE 1, its category, whether it may go in a dishwasher, how dirty it looks (soil: light / medium / heavy) and whether
   it looks greasy (glossy amber film).
2. Assign each dishwasher-safe object a position (P1-P4, B1-B3) or "defer" if it does not fit, and a loading order;
   objects that must not go in a dishwasher get slot "none" and order 0. Decide which items to defer sensibly.
3. Choose ONE wash program for the items that go in this load, based on how dirty / greasy they look and how full the load is.
{lang_note}
Answer only with the JSON."""


def plan(wall_png, room_png, goal_lang, tag, w, h):
    obj, meta = gpt.codex_json(prompt(goal_lang, w, h), SCHEMA, tag, images=[wall_png, room_png], purpose=f'plan:{goal_lang}')
    return obj, meta


# ---------------------------------------------------------------- plan feasibility (structural; independent of ground truth)
def feasibility(p):
    """Checks that only need the plan: slots unique, type-compatible, non-safe items not loaded, orders consistent,
    capacity not wasted (no safe item deferred while a compatible position stays empty)."""
    issues = []
    used = {}
    for it in p['items']:
        s = it['slot']
        if s in POS:
            if s in used:
                issues.append(f"slot {s} used twice ({used[s]}, {it['id']})")
            used[s] = it['id']
            if not it['dishwasher_safe']:
                issues.append(f"{it['id']} not dishwasher-safe but given slot {s}")
            if s.startswith('P') and it['category'] != 'plate':
                issues.append(f"{it['id']} ({it['category']}) in plate position {s}")
            if s.startswith('B') and it['category'] not in ('bowl', 'mug'):
                issues.append(f"{it['id']} ({it['category']}) in bowl/mug position {s}")
            if it['order'] <= 0:
                issues.append(f"{it['id']} has slot {s} but order {it['order']}")
        else:
            if s == 'none' and it['dishwasher_safe']:
                issues.append(f"{it['id']} is dishwasher-safe but slot none")
            if s == 'defer' and not it['dishwasher_safe']:
                issues.append(f"{it['id']} not dishwasher-safe but deferred")
    orders = [it['order'] for it in p['items'] if it['slot'] in POS]
    if len(set(orders)) != len(orders):
        issues.append(f'duplicate loading orders {sorted(orders)}')
    free_p = [s for s in ('P1', 'P2', 'P3', 'P4') if s not in used]
    free_b = [s for s in ('B1', 'B2', 'B3') if s not in used]
    for it in p['items']:
        if it['slot'] == 'defer':
            if it['category'] == 'plate' and free_p:
                issues.append(f"{it['id']} (plate) deferred while {free_p} free")
            if it['category'] in ('bowl', 'mug') and free_b:
                issues.append(f"{it['id']} ({it['category']}) deferred while {free_b} free")
    return dict(feasible=not issues, issues=issues)


if __name__ == '__main__':
    print(prompt('en', 1280, 720))
    print(json.dumps(SCHEMA)[:300])
