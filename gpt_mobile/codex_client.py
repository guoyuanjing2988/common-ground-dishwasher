"""The ONLY way this project talks to GPT: the `codex` CLI on the user's ChatGPT subscription (adapted from
../gpt_control/codex_client.py; non-blocking so the robot simulation keeps running while GPT thinks).

  codex exec --skip-git-repo-check --ephemeral --sandbox read-only --output-schema schema.json -o out.json [--image=..] "<prompt>"

stdin = /dev/null, cwd = an empty scratch dir, one call at a time. No API keys, no environment probing.
Every call is appended to logs/codex_calls.jsonl (seconds, prompt size, tokens, purpose).
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


class Call:
    def __init__(self, prompt, schema, tag, images=(), purpose='decision'):
        WORK.mkdir(parents=True, exist_ok=True)
        LEDGER.parent.mkdir(parents=True, exist_ok=True)
        self.sp = WORK / f'{tag}_schema.json'
        self.out = WORK / f'{tag}_out.json'
        self.err = WORK / f'{tag}_stderr.txt'
        self.sp.write_text(json.dumps(schema))
        self.prompt, self.images, self.tag = prompt, list(images), tag
        self.meta = dict(tag=tag, purpose=purpose, prompt_chars=len(prompt), n_images=len(self.images), attempts=0)
        self.t0 = time.monotonic()
        self.proc = None
        self.obj = None
        self._launch()

    def _launch(self):
        self.meta['attempts'] += 1
        self.out.unlink(missing_ok=True)
        cmd = ['codex', 'exec', '--skip-git-repo-check', '--ephemeral', '--sandbox', 'read-only',
               '--output-schema', str(self.sp), '-o', str(self.out)] + [f'--image={p}' for p in self.images] + [self.prompt]
        self.fh = open(self.err, 'w')
        self.proc = subprocess.Popen(cmd, cwd=str(WORK), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=self.fh)
        self.t_launch = time.monotonic()

    def poll(self, timeout=420, retries=1):
        """None while running; True when finished (self.obj holds the parsed JSON or None)."""
        if self.proc.poll() is None:
            if time.monotonic() - self.t_launch > timeout:
                self.proc.kill()
            return None
        self.fh.close()
        err = self.err.read_text(errors='replace')
        m = re.search(r'tokens used\s*\n\s*([\d,]+)', err)
        if m:
            self.meta['tokens'] = self.meta.get('tokens', 0) + int(m.group(1).replace(',', ''))
        if self.out.exists():
            try:
                self.obj = json.loads(self.out.read_text())
            except json.JSONDecodeError as e:
                self.meta['error'] = f'bad json: {e}'
        else:
            self.meta['error'] = f'codex exit {self.proc.returncode}: {err[-300:]}'
        if self.obj is None and self.meta['attempts'] <= retries:
            self._launch()
            return None
        self.meta['seconds'] = round(time.monotonic() - self.t0, 2)
        self.meta['ok'] = self.obj is not None
        with open(LEDGER, 'a') as fh:
            fh.write(json.dumps(dict(t=time.strftime('%Y-%m-%d %H:%M:%S'), **self.meta)) + '\n')
        return True

    def kill(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
