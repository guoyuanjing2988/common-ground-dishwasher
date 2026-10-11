#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Run jobs strictly one at a time, each under an RSS watchdog (kill above 3 GB); peak RSS -> out/peak_rss.tsv
cd $ROOT/full_pipeline
PY=$ROOT/.venv/bin/python
LIMIT_KB=3000000
job() {   # $1 = log name, rest = run.py args
  local log=$1; shift
  $PY run.py "$@" > "out/$log.log" 2>&1 &
  local P=$! peak=0
  while kill -0 $P 2>/dev/null; do
    R=$(ps -o rss= -p $P | tr -d ' '); R=${R:-0}
    [ "$R" -gt "$peak" ] && peak=$R
    if [ "$R" -gt "$LIMIT_KB" ]; then kill $P; echo "KILLED rss=$R" >> "out/$log.log"; fi
    sleep 2
  done
  wait $P; local rc=$?
  printf "%s\t%s\t%d\t%d\n" "$log" "$*" "$peak" "$rc" >> out/peak_rss.tsv
}
job M1_trials --option M1 --trials 10 --no-video
job M2_trials --option M2 --trials 10 --no-video
job M3_trials --option M3 --trials 5 --no-video
job M3a_single --option M3a --no-video
job M3flat_single --option M3flat --no-video
job M2_single --option M2
job M3_single --option M3
echo ALLDONE >> out/peak_rss.tsv
