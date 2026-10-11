"""ONE structured GPT call (codex exec, ChatGPT subscription, read-only sandbox, stdin=/dev/null) on out/counter.png.
GPT sees only the image + the prompt (no ground truth). Writes out/gpt.json (+ timing) for score.py / make_clip.py."""
from __future__ import annotations

import json
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
import os
V = os.environ.get('RC_VARIANT', '')   # '' = original scene; '_5dish' = same scene without the mug

WORK = ROOT / 'codex_work'
CATS = ['plate', 'bowl', 'mug', 'cutting_board', 'knife', 'pan', 'flask', 'soap_bottle', 'sponge', 'kettle', 'other']
SLOTS = ['P1', 'P2', 'P3', 'P4', 'B1', 'B2', 'B3']
SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["items"],
    "properties": {"items": {"type": "array", "items": {
        "type": "object", "additionalProperties": False,
        "required": ["box", "label_en", "label_zh", "category", "count", "dishwasher_safe", "reason_en", "reason_zh", "slots"],
        "properties": {
            "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                    "description": "[x_min,y_min,x_max,y_max] tight pixel box in the image (a stack: box of the whole stack)"},
            "label_en": {"type": "string", "description": "short English name, e.g. 'Dinner plates (stack)'"},
            "label_zh": {"type": "string", "description": "short Chinese name, e.g. '盘子（一摞）'"},
            "category": {"type": "string", "enum": CATS},
            "count": {"type": "integer", "description": "number of identical pieces in this entry (a stack of 3 plates = 3)"},
            "dishwasher_safe": {"type": "boolean"},
            "reason_en": {"type": "string", "description": "at most 5 words, e.g. 'wood warps', 'sharp knife, dulls', 'ceramic, safe'"},
            "reason_zh": {"type": "string", "description": "at most 8 characters, e.g. '木头会变形', '刀会变钝'"},
            "slots": {"type": "array", "items": {"type": "string", "enum": SLOTS},
                      "description": "basket positions, one per piece, for dishwasher-safe items; [] for items that stay out"}}}}}}

PROMPT = """The attached image is {w}x{h} pixels (x right, y down, origin top-left). It shows a kitchen counter next to a small
countertop dishwasher (left edge of the image, door open, basket visible). List EVERY object lying or standing on the counter
(ignore the dishwasher itself, the wall and the cabinets). Identical pieces stacked on top of each other are ONE entry with
their count. For each entry give a tight pixel bounding box, a short English and Chinese label, a category, whether it may go
in a dishwasher (with a very short reason in English and Chinese: material/damage reason if not, e.g. 'wood warps / 木头会变形'),
and for dishwasher-safe pieces the basket position(s).
DISHWASHER BASKET: plate rack P1 (back), P2, P3, P4 (front), one dinner plate each; bowl/cup area B1, B2, B3, one bowl or mug
each. Every position holds at most one piece. Items that must not go in the dishwasher get slots []. Answer only with the JSON."""


def main(tag=''):
    WORK.mkdir(exist_ok=True)
    sp, out = WORK / 'schema.json', WORK / f'out{V}{tag}.json'
    sp.write_text(json.dumps(SCHEMA))
    out.unlink(missing_ok=True)
    img = ROOT / 'out' / f'counter{V}.png'
    cmd = ['codex', 'exec', '--skip-git-repo-check', '--sandbox', 'read-only', '--output-schema', str(sp), '-o', str(out),
           f'--image={img}', PROMPT.format(w=1280, h=720)]
    t0 = time.time()
    p = subprocess.run(cmd, cwd=str(WORK), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                       timeout=600, check=False)
    sec = round(time.time() - t0, 1)
    err = p.stderr.decode(errors='replace')
    m = re.search(r'tokens used\s*\n\s*([\d,]+)', err)
    if not out.exists():
        raise SystemExit(f'codex failed rc={p.returncode} after {sec}s: {err[-600:]}')
    obj = json.loads(out.read_text())
    obj['latency_s'] = sec
    obj['tokens'] = int(m.group(1).replace(',', '')) if m else None
    obj['prompt'] = PROMPT
    (ROOT / 'out' / f'gpt{V}{tag}.json').write_text(json.dumps(obj, indent=1, ensure_ascii=False))
    print(f'latency {sec}s tokens {obj["tokens"]}')
    for it in obj['items']:
        print(it['category'], it['count'], it['label_en'], it['label_zh'], it['dishwasher_safe'], it['reason_en'], it['reason_zh'], it['slots'], it['box'])


if __name__ == '__main__':
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else '')   # '' = the run used in the clip; '_r2', '_r3' = repeat runs (consistency only)
