#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Sequential GPT evaluation runs (ONE sim process at a time, RSS watchdog via runw.sh).
# usage: ./batch.sh <listfile>   each line: level scen seed goal [extra args...]
cd $ROOT/gpt_control
while read -r LV SC SD GL EXTRA; do
  [ -z "$LV" ] && continue
  case "$LV" in \#*) continue;; esac
  TAG="${LV}_${SC}_s${SD}_${GL}"
  if [ -f "out/gpt_runs/${TAG}.json" ]; then echo "skip $TAG"; continue; fi
  echo "$(date '+%T') start $TAG"
  ./runw.sh "logs/${TAG}.log" gpt_agent.py --level "$LV" --scen "$SC" --seed "$SD" --goal "$GL" $EXTRA
  grep -h "END" "logs/${TAG}.log" | cut -c1-400
done < "$1"
echo BATCHDONE
