"""ONE structured GPT call (codex exec, ChatGPT subscription, read-only, stdin=/dev/null) on out/sink.png -> out/sink_gpt.json.
GPT sees only the image + prompt. Per item: box, EN/中文 label, dishwasher-safe + reason, and for dishes a pick order + reason."""
from __future__ import annotations

import json
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
import os
P = os.environ.get('SINK_PREFIX', 'sink')   # 'sink' (default layout) or 'sink6' (layout matching sink_refs/sink_6.png)

WORK = ROOT / 'codex_work'
CATS = ['plate', 'bowl', 'mug', 'cutting_board', 'knife', 'pan', 'flask', 'soap_bottle', 'sponge', 'dish_cloth', 'other']
SCHEMA = {"type": "object", "additionalProperties": False, "required": ["items"], "properties": {"items": {"type": "array", "items": {
    "type": "object", "additionalProperties": False,
    "required": ["box", "label_en", "label_zh", "category", "dishwasher_safe", "reason_en", "reason_zh", "pick_order", "pick_reason_en", "pick_reason_zh"],
    "properties": {
        "box": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4, "description": "[x_min,y_min,x_max,y_max] pixels"},
        "label_en": {"type": "string", "description": "short, e.g. 'Plate (leaning on right wall)'"},
        "label_zh": {"type": "string"},
        "category": {"type": "string", "enum": CATS},
        "dishwasher_safe": {"type": "boolean"},
        "reason_en": {"type": "string", "description": "<= 5 words, e.g. 'ceramic, safe', 'wood warps'"},
        "reason_zh": {"type": "string", "description": "<= 8 characters"},
        "pick_order": {"type": "integer", "description": "1, 2, 3 ... for every dish in the sink to load into the dishwasher; 0 for everything else"},
        "pick_reason_en": {"type": "string", "description": "dishes: why this position, <= 8 words, e.g. 'bowl rests on plate 2, take first'; others: ''"},
        "pick_reason_zh": {"type": "string", "description": "dishes: <= 14 characters; others: ''"}}}}}}
PROMPT = """The attached image is {w}x{h} pixels (x right, y down, origin top-left), an overhead camera view of a messy home
kitchen sink next to a countertop dishwasher (dishwasher at the far left edge). List EVERY loose object, one entry per piece
(each plate separately, even when they overlap): the dishes piled in the sink basin and the items on the counter around it.
Ignore fixtures: the sink basin, drain, faucet, dishwasher, wall tiles. For each: tight pixel bounding box of its visible part,
short English and Chinese label, category, whether it may go in a dishwasher with a very short reason (EN + 中文).
For the dishes in the sink give a PICK ORDER for a robot unloading them one at a time: an item that something else rests on
must come AFTER that thing (take the least blocked / top-most piece first), with a short reason, e.g. 'bowl rests on plate 2,
take first'. Answer only with the JSON."""


def main():
    WORK.mkdir(exist_ok=True)
    sp, out = WORK / f'{P}_schema.json', WORK / f'{P}_out.json'
    sp.write_text(json.dumps(SCHEMA))
    out.unlink(missing_ok=True)
    prompt = PROMPT.format(w=1280, h=720)
    if P == 'sink6':
        prompt = prompt.replace('an overhead camera view', 'a camera view').replace('(dishwasher at the far left edge)', '(dishwasher on the right)')
    cmd = ['codex', 'exec', '--skip-git-repo-check', '--sandbox', 'read-only', '--output-schema', str(sp), '-o', str(out),
           f"--image={ROOT / 'out' / f'{P}.png'}", prompt]
    t0 = time.time()
    p = subprocess.run(cmd, cwd=str(WORK), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=600, check=False)
    sec = round(time.time() - t0, 1)
    err = p.stderr.decode(errors='replace')
    if not out.exists():
        raise SystemExit(f'codex failed rc={p.returncode} after {sec}s: {err[-600:]}')
    m = re.search(r'tokens used\s*\n\s*([\d,]+)', err)
    obj = json.loads(out.read_text())
    obj.update(latency_s=sec, tokens=int(m.group(1).replace(',', '')) if m else None, prompt=prompt)
    (ROOT / 'out' / f'{P}_gpt.json').write_text(json.dumps(obj, indent=1, ensure_ascii=False))
    print(f'latency {sec}s tokens {obj["tokens"]}')
    for it in obj['items']:
        print(it['pick_order'], it['category'], it['label_en'], it['label_zh'], it['dishwasher_safe'], it['reason_en'], '|', it['pick_reason_en'], it['pick_reason_zh'], it['box'])


if __name__ == '__main__':
    main()
