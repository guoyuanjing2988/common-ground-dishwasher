#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# usage: runw.sh <logfile> <python args...>
# ONE sim process at a time (lock dir), RSS watchdog kills it above 3 GB, peak RSS appended to logs/peak_rss.tsv.
LOG=$1; shift
PY=$ROOT/.venv/bin/python
cd $ROOT/gpt_mobile || exit 1
LOCK=logs/.sim_lock
if ! mkdir "$LOCK" 2>/dev/null; then echo "another sim is running (lock $LOCK) - refusing to start"; exit 3; fi
trap 'rmdir "$LOCK" 2>/dev/null' EXIT
export PYTHONDONTWRITEBYTECODE=1          # never write __pycache__ into the imported (read-only) project dirs
$PY "$@" > "$LOG" 2>&1 & P=$!
PEAK=0
while kill -0 $P 2>/dev/null; do
  R=$(ps -o rss= -p $P | tr -d ' '); R=${R:-0}
  [ "$R" -gt "$PEAK" ] && PEAK=$R
  if [ "$R" -gt 3000000 ]; then echo "WATCHDOG KILL rss=$R" >> "$LOG"; kill $P; sleep 1; kill -9 $P 2>/dev/null; fi
  sleep 2
done
wait $P; RC=$?
echo "$(date '+%F %T')	$LOG	rc=$RC	peak_rss_kb=$PEAK" >> logs/peak_rss.tsv
echo "rc=$RC peak_rss_mb=$((PEAK/1024))"
