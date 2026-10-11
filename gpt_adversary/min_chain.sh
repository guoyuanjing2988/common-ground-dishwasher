#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# sequential minimisation chain (each sim via runw.sh inside minimize.py)
cd $ROOT/gpt_adversary
export PYTHONDONTWRITEBYTECODE=1
PY=$ROOT/.venv/bin/python
run() { E=$1; B=$2; [ -f logs/minimize/$E.json ] && { echo "skip $E"; return; }; $PY minimize.py --entry $E --bisect $B; }
run G_r1_s1 3
run G_r1_s3 3
run G_r2_s2 3
run G_r6_s3 2
run G_r4_s3 2
run G_r6_s1 2
run S_06 3
run S_13 2
run S_22 2
run S_20 2
run S_15 2
run S_11 2
run G_r1_s2 0
run G_r2_s1 0
run G_r3_s1 0
run G_r2_s3 1
run R_03 1
run R_01 1
echo CHAIN DONE
