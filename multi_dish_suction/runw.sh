#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# usage: runw.sh <logfile> <python args...>   -- ONE sim process, RSS watchdog kills above 3 GB, records peak RSS
LOG=$1; shift
PY=$ROOT/.venv/bin/python
cd $ROOT/multi_dish_suction
/usr/bin/time -l $PY "$@" > "$LOG" 2>&1 & P=$!
PEAK=0
while kill -0 $P 2>/dev/null; do R=0; for Q in $P $(pgrep -P $P); do X=$(ps -o rss= -p $Q | tr -d ' '); R=$((R+${X:-0})); done; [ "$R" -gt "$PEAK" ] && PEAK=$R; [ "$R" -gt 3000000 ] && { echo "WATCHDOG KILL rss=$R" >> "$LOG"; pkill -P $P; kill $P; }; sleep 1; done
wait $P; RC=$?; MAXRSS=$(grep "maximum resident" "$LOG" | awk "{print int(\$1/1024)}"); [ -n "$MAXRSS" ] && [ "$MAXRSS" -gt "$PEAK" ] && PEAK=$MAXRSS
echo "$(date '+%F %T')	$LOG	rc=$RC	peak_rss_kb=$PEAK" >> logs/peak_rss.tsv
echo "rc=$RC peak_rss_mb=$((PEAK/1024))"
