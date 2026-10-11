#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
cd $ROOT/gpt_adversary
for N in L1_stiff_pad L1_occupied_slot L1_sticky_rail; do
  [ -f runs/l1/$N.json ] && continue
  ./runw.sh logs/sims/$N.log run_scenario.py --scenario scenarios/l1/$N.json --out runs/l1/$N.json --commander L1
  grep -E "RESULT|END|HARNESS" logs/sims/$N.log | cut -c1-400
done
echo L1 DONE
