#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Run ONE python job under an RSS watchdog (kill above 3 GB). Usage: ./guard.sh <logname> <script.py> [args...]
# Peak RSS (kB) is appended to out/peak_rss.tsv. Never run two of these at the same time.
cd $ROOT/mobile_pipeline
PY=$ROOT/.venv/bin/python
LIMIT_KB=3000000
log=$1; shift
mkdir -p out
$PY "$@" > "out/$log.log" 2>&1 &
P=$!; peak=0
while kill -0 $P 2>/dev/null; do
  R=$(ps -o rss= -p $P | tr -d ' '); R=${R:-0}
  [ "$R" -gt "$peak" ] && peak=$R
  if [ "$R" -gt "$LIMIT_KB" ]; then kill $P; echo "KILLED by watchdog rss=$R kB" >> "out/$log.log"; fi
  sleep 2
done
wait $P; rc=$?
printf "%s\t%s\t%d\t%d\n" "$log" "$*" "$peak" "$rc" >> out/peak_rss.tsv
echo "rc=$rc peak_rss_kB=$peak"
