#!/usr/bin/env python3
"""Audit every codex transcript (logs/codex/*_stderr_*.txt): which shell commands codex ran, whether any touched a path
outside its -C sandbox copy (e.g. the original pipelines, which contain the human fixes), and whether it tried to execute
Python or the simulation. Writes logs/codex_audit.json."""
from __future__ import annotations

import json
import re
from pathlib import Path

G = Path(__file__).resolve().parent.parent
EXEC = re.compile(r'^/bin/(?:zsh|bash) -lc (.+?) in (/\S+)\s*$', re.M)
OUTSIDE = re.compile(r'(/Us[e]rs/\S+|~/\S*|\.\./\S*)')
RUNS = re.compile(r'\b(python3?|\.venv|run_mobile|run\.py|eval_mobile|eval_full|mujoco|pytest)\b')


def main():
    out = {}
    for f in sorted((G / 'logs' / 'codex').glob('*_stderr_a*.txt')):
        txt = f.read_text(errors='replace')
        cmds = EXEC.findall(txt)
        rec = dict(n_commands=len(cmds), outside_paths=[], ran_python=[], cwd=sorted({c[1] for c in cmds}))
        for cmd, cwd in cmds:
            for p in OUTSIDE.findall(cmd):
                p = p.strip('"\'')
                if not p.startswith(cwd):
                    rec['outside_paths'].append(dict(cmd=cmd[:200], path=p))
            if RUNS.search(cmd) and not re.search(r'\b(rg|grep|sed|cat|head|tail|ls)\b', cmd.split()[0] if cmd.split() else ''):
                if re.search(r"(^|['\"\s;&|])(python3?|\.venv/bin/python)\s", cmd):
                    rec['ran_python'].append(cmd[:200])
        out[f.name] = rec
    tot = dict(transcripts=len(out), commands=sum(r['n_commands'] for r in out.values()),
               with_outside_paths=[k for k, r in out.items() if r['outside_paths']],
               with_python=[k for k, r in out.items() if r['ran_python']])
    (G / 'logs' / 'codex_audit.json').write_text(json.dumps(dict(summary=tot, per_transcript=out), indent=1))
    print(json.dumps(tot, indent=1))


if __name__ == '__main__':
    main()
