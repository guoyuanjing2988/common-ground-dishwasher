#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Evaluation batch: seeds 0-11, English then Chinese goal. STRICTLY one sim process at a time (runw.sh watchdog, 3 GB).
cd $ROOT/gpt_sort
for s in 0 1 2 3 4 5 6 7 8 9 10 11; do
  for l in en zh; do
    ./runw.sh logs/eval_s${s}_${l}.log run_sort.py --seed $s --lang $l
  done
done
echo BATCH_DONE >> logs/batch.done
