#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# usage: runw.sh <logfile> <python args...>   -- ONE sim process, RSS watchdog kills above 3 GB, records peak RSS
# (copy of ../gpt_control/runw.sh, working dir = gpt_adversary, no .pyc written into the imported projects)
LOG=$1; shift
PY=$ROOT/.venv/bin/python
cd $ROOT/gpt_adversary
export PYTHONDONTWRITEBYTECODE=1
$PY "$@" > "$LOG" 2>&1 & P=$!
PEAK=0
while kill -0 $P 2>/dev/null; do R=$(ps -o rss= -p $P | tr -d ' '); R=${R:-0}; [ "$R" -gt "$PEAK" ] && PEAK=$R; [ "$R" -gt 3000000 ] && { echo "WATCHDOG KILL rss=$R" >> "$LOG"; kill $P; }; sleep 2; done
wait $P; RC=$?
echo "$(date '+%F %T')	$LOG	rc=$RC	peak_rss_kb=$PEAK" >> logs/peak_rss.tsv
echo "rc=$RC peak_rss_mb=$((PEAK/1024))"
