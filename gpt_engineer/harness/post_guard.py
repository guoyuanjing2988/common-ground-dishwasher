#!/usr/bin/env python3
"""Post-hoc held-out nominal-mission guard for accepted patches that the harness did not guard itself.

  post_guard.py <problem_key> <code_dir> <out_dir> <case> [<case> ...]

Uses the harness's own run_case (ONE sim process at a time, under runw.sh, 3 GB RSS watchdog) and case_pass,
so the pass predicate is identical to the in-loop metric. Writes <out_dir>/summary.json.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness as H  # noqa: E402

key, code, out = sys.argv[1], Path(sys.argv[2]).resolve(), Path(sys.argv[3]).resolve()
cases = sys.argv[4:]
recs = []
for c in cases:                       # strictly sequential
    r = H.run_case(H.PROBLEMS[key], code, c, out)
    r['PASS'] = H.case_pass(key, r)
    recs.append(r)
    print(c, 'PASS' if r['PASS'] else 'FAIL', H.per_case_short(key, [r]), 'rss', r.get('watchdog_peak_rss_mb'), flush=True)
summ = dict(problem=key, code=str(code), cases=cases, score=sum(r['PASS'] for r in recs), n=len(recs),
            per_case=H.per_case_short(key, recs), peak_rss_mb=max([r.get('watchdog_peak_rss_mb') or 0 for r in recs] or [0]))
(out / 'summary.json').write_text(json.dumps(summ, indent=1))
print('GUARD', json.dumps(summ), flush=True)
