#!/usr/bin/env python3
"""Independent re-check of the P2 (plate lean) result from the raw per-seed eval JSONs (no sim run).

Re-derives the pass/fail of every P2 mission with its own predicate (not harness.case_pass), cross-checks the frozen
scorer's lean against the pipeline's own insert-step plane_tilt_deg, and checks for collateral (steps not ok, robot
contacts, penetration, wrong tines). Writes runs/p2_verify/independent_check.json and prints a table.
"""
import json
import statistics as stt
from pathlib import Path

G = Path(__file__).resolve().parent.parent
GROUPS = {
    'baseline it0 seeds0-4': sorted((G / 'runs/p2_lean/it0/eval').glob('seed*.json')),
    'patched it1 seeds0-4': sorted((G / 'runs/p2_lean/it1/eval').glob('seed*.json')),
    'patched guard seeds5-7': sorted((G / 'runs/p2_lean/it1/guard').glob('seed*.json')),
    'patched extra seeds8-17': sorted((G / 'runs/p2_verify').glob('patched_seed*.json')),
    'baseline extra seeds8-12': sorted((G / 'runs/p2_verify').glob('base_seed*.json')),
    'patched noise3mm seeds0-4': sorted((G / 'runs/p2_verify').glob('noise3_seed*.json')),
}


def check(p):
    r = json.loads(p.read_text())
    st = r.get('states', {})
    fin, ins = st.get('final', {}), st.get('after_insert', {})
    steps = r.get('task_steps', [])
    ins_step = next((s for s in steps if s['step'] == 'insert_plate'), {})
    code_tilt = (ins_step.get('metrics', {}).get('metrics') or {}).get('plane_tilt_deg')
    max_unpl = max([s['metrics'].get('unplanned_contact_max_N') or 0 for s in steps] or [0])
    ok = (r.get('success') is True and len(steps) == 9 and all(s['ok'] for s in steps)
          and fin.get('seated_own_tines') is True and not fin.get('wrong_tines') and not fin.get('other_contacts')
          and abs(fin.get('bottom_offset_mm', 99)) < 12.0 and fin.get('lean_deg', 99) < 6.0
          and abs(fin.get('door_deg', 99)) < 1.0 and abs(fin.get('basket_m', 1)) < .005)
    return dict(file=str(p.relative_to(G)), case=r.get('case'), PASS=ok, final_lean=fin.get('lean_deg'),
                insert_lean_frozen=ins.get('lean_deg'), insert_lean_code=code_tilt,
                scorer_agrees=(code_tilt is not None and abs(code_tilt - ins.get('lean_deg', -1)) < .011),
                bottom_mm=fin.get('bottom_offset_mm'), wrong=fin.get('wrong_tines'), other=fin.get('other_contacts'),
                door=fin.get('door_deg'), basket=fin.get('basket_m'), steps_ok=sum(s['ok'] for s in steps),
                unplanned_contact_max_N=max_unpl, unexpected_robot_contacts=len(r.get('unexpected_robot_contacts') or []),
                max_pen_mm=r.get('max_penetration_mm'), sandbox=r.get('sandbox', '').split('gpt_engineer/')[-1])


out = {}
for g, files in GROUPS.items():
    rows = [check(p) for p in files]
    leans = [x['final_lean'] for x in rows if isinstance(x['final_lean'], (int, float))]
    out[g] = dict(n=len(rows), passes=sum(x['PASS'] for x in rows),
                  final_lean_min=min(leans, default=None), final_lean_max=max(leans, default=None),
                  final_lean_mean=round(stt.mean(leans), 2) if leans else None,
                  scorer_agrees_all=all(x['scorer_agrees'] for x in rows),
                  max_unplanned_N=max([x['unplanned_contact_max_N'] for x in rows] or [0]),
                  max_pen_mm=max([x['max_pen_mm'] or 0 for x in rows] or [0]), rows=rows)
    print(f"{g:28s} {out[g]['passes']}/{out[g]['n']}  final lean {out[g]['final_lean_min']}-{out[g]['final_lean_max']} "
          f"(mean {out[g]['final_lean_mean']})  scorer==code {out[g]['scorer_agrees_all']}  "
          f"max unplanned {out[g]['max_unplanned_N']} N  max pen {out[g]['max_pen_mm']} mm  sandbox {rows[0]['sandbox'] if rows else ''}")
(G / 'runs/p2_verify/independent_check.json').write_text(json.dumps(out, indent=1))
