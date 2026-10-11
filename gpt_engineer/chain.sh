#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Strictly sequential: wait for the running P3 harness, then the remaining problems one after another.
cd $ROOT/gpt_engineer
PY=/opt/homebrew/bin/python3
# (P3 finished before this chain was restarted)
$PY -u harness/harness.py p2b_lean_noaim > logs/harness_p2b_lean_noaim.out 2>&1
$PY -u harness/harness.py p4_dock > logs/harness_p4_dock.out 2>&1
for r in 2 3; do
  mkdir -p runs/p1_aim_rep$r && cp -R runs/p1_aim/it0 runs/p1_aim_rep$r/ && head -1 runs/p1_aim/ledger.jsonl > runs/p1_aim_rep$r/ledger.jsonl
  $PY -u harness/harness.py p1_aim_rep$r > logs/harness_p1_aim_rep$r.out 2>&1
done
echo CHAIN_DONE >> logs/chain.out
