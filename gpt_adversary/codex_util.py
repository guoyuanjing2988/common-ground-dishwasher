"""codex CLI calls with rate/usage-limit back-off. Wraps ../gpt_control/codex_client.codex_json (the proven pattern:
codex exec --skip-git-repo-check --sandbox read-only --output-schema S -o O, stdin=/dev/null, scratch cwd), with its
work dir and ledger redirected into gpt_adversary/. No API keys are used or looked for: the CLI runs on the user's
ChatGPT subscription login."""
from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT / 'gpt_control'))
sys.dont_write_bytecode = True
import codex_client as CC  # noqa: E402

CC.WORK = ROOT / 'codex_work'
CC.LEDGER = ROOT / 'logs' / 'codex_calls.jsonl'
_orig = CC.codex_json
LIMIT_RE = re.compile(r'rate.?limit|usage.?limit|too many requests|\b429\b|quota|try again (later|in)', re.I)
BACKOFF = (60, 120, 240, 480, 600)
LIMIT_LOG = ROOT / 'logs' / 'rate_limits.jsonl'


def codex_json(prompt, schema, tag, images=(), purpose='decision', timeout=420, retries=1):
    """CC.codex_json, but a rate / usage limit is backed off (60 s, 120 s, 240 s, ...) and logged instead of failing."""
    tot = dict(backoffs=[])
    for k in range(len(BACKOFF) + 1):
        obj, meta = _orig(prompt, schema, f'{tag}' if k == 0 else f'{tag}_b{k}', images=images, purpose=purpose,
                          timeout=timeout, retries=retries)
        if obj is not None or not LIMIT_RE.search(meta.get('error', '') or ''):
            meta['backoffs'] = tot['backoffs']
            return obj, meta
        if k == len(BACKOFF):
            break
        wait = BACKOFF[k]
        tot['backoffs'].append(wait)
        LIMIT_LOG.parent.mkdir(exist_ok=True)
        with open(LIMIT_LOG, 'a') as fh:
            fh.write(json.dumps(dict(t=time.strftime('%F %T'), tag=tag, purpose=purpose, wait_s=wait, error=meta.get('error', '')[-300:])) + '\n')
        print(f'[codex] rate/usage limit on {tag}: backing off {wait} s', flush=True)
        time.sleep(wait)
    meta['backoffs'] = tot['backoffs']
    meta['rate_limited'] = True
    return obj, meta


def install():
    """Route every codex_client call in this process (including perception inside the sim) through the back-off."""
    CC.codex_json = codex_json
