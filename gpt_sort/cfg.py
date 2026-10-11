"""Import shim: puts ../full_pipeline and ../gpt_control on sys.path (read-only reuse, nothing there is modified) and sets
this task's wall-camera resolution BEFORE perception.py reads it at import time."""
from __future__ import annotations

import sys
from pathlib import Path
import os as _os
_ROOT = Path(_os.environ.get('GYJ_WORK', Path(__file__).resolve().parents[1]))   # repo root (override with env GYJ_WORK)

ROOT = Path(__file__).resolve().parent
FP = _ROOT / 'full_pipeline'
GC = _ROOT / 'gpt_control'
for p in (str(FP), str(GC)):
    if p not in sys.path:
        sys.path.insert(0, p)

import scene as S  # noqa: E402

# wall camera for the clutter task: 1280 x 720 (the scene header's offscreen buffer is 1280 x 720)
S.CAM_W, S.CAM_H, S.CAM_FOVY = 1280, 720, 60.0
# camera under the wall cabinet on the back wall, looking forward-down over the whole counter run right of the machine
CAM_POS = (-0.56, 0.64, 1.56)
CAM_LOOK = (-0.14, 0.64, 0.88)
# second, context-only camera (room corner, no depth used): makes tall items (flask) recognisable
ROOMCAM_POS = (0.30, 1.30, 1.55)
ROOMCAM_LOOK = (-0.24, 0.66, 0.92)

import perception as PC  # noqa: E402,F401  (reads S.CAM_W / CAM_H now)
import codex_client as CC  # noqa: E402

# every codex call of this task goes to OUR ledger and scratch dir (codex_client writes relative to its module ROOT)
CC.WORK = ROOT / 'codex_work'
CC.LEDGER = ROOT / 'logs' / 'codex_calls.jsonl'
