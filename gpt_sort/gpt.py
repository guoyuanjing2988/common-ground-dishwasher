"""The only way this task talks to GPT: `codex exec` on the ChatGPT subscription.

  codex exec --skip-git-repo-check --sandbox read-only --output-schema <schema> -o <out> --image=<png> "<prompt>"
  stdin = /dev/null, cwd = an empty scratch dir. No API keys, no environment probing.

Rate / usage limits: other processes may use codex concurrently. If a call fails with a limit-like message, back off 60 s, then 120 s,
240 s ... (max 4 back-offs) and record every back-off in the ledger, so the report can state it.
"""
from __future__ import annotations

import json
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WORK = ROOT / 'codex_work'
LEDGER = ROOT / 'logs' / 'codex_calls.jsonl'
LIMIT_RE = re.compile(r'rate.?limit|usage limit|too many requests|429|quota|try again later', re.I)


def codex_json(prompt: str, schema: dict, tag: str, images=(), purpose='plan', timeout=600, retries=1, backoffs=4):
    """One structured codex call. Returns (obj | None, meta)."""
    WORK.mkdir(parents=True, exist_ok=True)
    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    sp = WORK / f'{tag}_schema.json'
    out = WORK / f'{tag}_out.json'
    sp.write_text(json.dumps(schema))
    meta = dict(tag=tag, purpose=purpose, prompt_chars=len(prompt), n_images=len(images), attempts=0, backoffs=[])
    obj = None
    t_all = time.time()
    attempt, waits = 0, 0
    while True:
        attempt += 1
        meta['attempts'] = attempt
        out.unlink(missing_ok=True)
        cmd = ['codex', 'exec', '--skip-git-repo-check', '--ephemeral', '--sandbox', 'read-only',
               '--output-schema', str(sp), '-o', str(out)] + [f'--image={p}' for p in images] + [prompt]
        t0 = time.time()
        try:
            p = subprocess.run(cmd, cwd=str(WORK), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE, timeout=timeout, check=False)
            err = p.stderr.decode(errors='replace')
            rc = p.returncode
        except subprocess.TimeoutExpired:
            err, rc = 'timeout', -9
        meta.setdefault('call_seconds', []).append(round(time.time() - t0, 1))
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
            meta['error'] = f'codex exit {rc}: {err[-400:]}'
        if not out.exists() and LIMIT_RE.search(err) and waits < backoffs:
            w = 60 * 2 ** waits
            meta['backoffs'].append(dict(wait_s=w, msg=err[-200:]))
            waits += 1
            time.sleep(w)
            continue
        if attempt > retries + waits:
            break
    meta['seconds'] = round(time.time() - t_all, 2)
    meta['ok'] = obj is not None
    with open(LEDGER, 'a') as fh:
        fh.write(json.dumps(dict(t=time.strftime('%Y-%m-%d %H:%M:%S'), **meta)) + '\n')
    return obj, meta


BOX_SCHEMA = {
    "type": "object", "additionalProperties": False, "required": ["found", "bbox", "center"],
    "properties": {"found": {"type": "boolean"},
                   "bbox": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                            "description": "[x_min,y_min,x_max,y_max] pixels"},
                   "center": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2,
                              "description": "[x,y] pixel of the object centre"}}}


def locate(png, prompt, tag, purpose='perception'):
    obj, meta = codex_json(prompt, BOX_SCHEMA, tag, images=[png], purpose=purpose)
    if obj is None:
        return dict(found=False, error=meta.get('error'), seconds=meta['seconds']), meta
    obj['seconds'] = meta['seconds']
    return obj, meta
