"""The ONLY way this project talks to GPT: the `codex` CLI on the user's ChatGPT subscription.

Pattern (proven in ../full_pipeline/perception.py ask_codex):
  codex exec --skip-git-repo-check --sandbox read-only --output-schema schema.json -o out.json [--image=..] "<prompt>"
run with stdin=/dev/null from a scratch working dir, one call at a time. No API keys, no env probing.
Every call is appended to a JSONL ledger (seconds, prompt size, tokens, purpose) so per-mission cost can be counted.
"""
from __future__ import annotations

import json
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WORK = ROOT / 'codex_work'                 # scratch cwd for every codex call (empty, read-only sandbox)
LEDGER = ROOT / 'logs' / 'codex_calls.jsonl'


def codex_json(prompt: str, schema: dict, tag: str, images=(), purpose='decision', timeout=420, retries=1):
    """One structured codex call. Returns (obj | None, meta)."""
    WORK.mkdir(parents=True, exist_ok=True)
    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    sp = WORK / f'{tag}_schema.json'
    out = WORK / f'{tag}_out.json'
    sp.write_text(json.dumps(schema))
    meta = dict(tag=tag, purpose=purpose, prompt_chars=len(prompt), n_images=len(images), attempts=0)
    obj = None
    t_all = time.time()
    for attempt in range(retries + 1):
        meta['attempts'] = attempt + 1
        out.unlink(missing_ok=True)
        cmd = ['codex', 'exec', '--skip-git-repo-check', '--ephemeral', '--sandbox', 'read-only',
               '--output-schema', str(sp), '-o', str(out)] + [f'--image={p}' for p in images] + [prompt]
        try:
            p = subprocess.run(cmd, cwd=str(WORK), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE, timeout=timeout, check=False)
            err = p.stderr.decode(errors='replace')
            rc = p.returncode
        except subprocess.TimeoutExpired:
            err, rc = 'timeout', -9
        m = re.search(r'tokens used\s*\n\s*([\d,]+)', err)
        if m:
            meta['tokens'] = meta.get('tokens', 0) + int(m.group(1).replace(',', ''))
        if out.exists():
            try:
                obj = json.loads(out.read_text())
                break
            except json.JSONDecodeError as e:
                meta['error'] = f'bad json: {e}'
        else:
            meta['error'] = f'codex exit {rc}: {err[-300:]}'
    meta['seconds'] = round(time.time() - t_all, 2)
    meta['ok'] = obj is not None
    with open(LEDGER, 'a') as fh:
        fh.write(json.dumps(dict(t=time.strftime('%Y-%m-%d %H:%M:%S'), **meta)) + '\n')
    return obj, meta


VLM_SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["found", "bbox", "center"],
    "properties": {"found": {"type": "boolean"},
                   "bbox": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                            "description": "[x_min,y_min,x_max,y_max] pixels of the object"},
                   "center": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2,
                              "description": "[x,y] pixel of the object centre"}}}

VLM_PROMPTS = {
    # identical wording to ../full_pipeline/perception.py PROMPTS['flat']
    'plate': ("The attached image is {w}x{h} pixels (x right, y down, origin top-left). It was taken by a camera mounted "
              "above a kitchen counter. Locate the round dinner plate lying flat on the counter (ignore the small dish, the "
              "robot and the dishwasher). Return pixel coordinates in this image: the plate's tight bounding box and its "
              "centre. Answer only with the JSON."),
    'pod': ("The attached image is {w}x{h} pixels (x right, y down, origin top-left). It was taken by a camera mounted "
            "above a kitchen counter. Locate the small blue rectangular dishwasher detergent tablet (pod), wherever it is "
            "(it may be inside the small round dish or lying on the counter). If you cannot see it, set found=false. "
            "Return pixel coordinates in this image: its tight bounding box and its centre. Answer only with the JSON."),
}


def ask_vlm(png_path, what, w, h, tag):
    obj, meta = codex_json(VLM_PROMPTS[what].format(w=w, h=h), VLM_SCHEMA, tag, images=[png_path], purpose=f'perception:{what}')
    if obj is None:
        return dict(found=False, error=meta.get('error'), seconds=meta['seconds']), meta
    obj['seconds'] = meta['seconds']
    return obj, meta
