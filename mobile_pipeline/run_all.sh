#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Every reported run, strictly ONE simulation process at a time, each under the 3 GB RSS watchdog (guard.sh).
# Peak RSS of each job -> out/peak_rss.tsv. Post-processing: aggregate_mobile.py, seqdiag.py (homebrew python).
cd $ROOT/mobile_pipeline
./guard.sh brake_test run_mobile.py --brake-test
./guard.sh low_battery run_mobile.py --low-battery
./guard.sh mission_video run_mobile.py --video
./guard.sh trials run_mobile.py --trials 5
./guard.sh estop run_mobile.py --estop
./guard.sh nonconductive run_mobile.py --nonconductive
for i in 0 1 2 3 4 5 6 7 8 9 10 11; do ./guard.sh posesweep_$i run_mobile.py --pose-sweep --only $i; done
for i in 0 1 2 3 4 5 6 7 8 9 10; do ./guard.sh docksweep_$i run_mobile.py --dock-sweep --only $i; done
./guard.sh panel_door_open run_mobile.py --panel-door-open
./guard.sh tcpcap run_mobile.py --tcp-cap 0.25 --debug-truth-perception --tag tcpcap025
./guard.sh nocap_ref run_mobile.py --debug-truth-perception --tag nocap_ref
$ROOT/.venv/bin/python aggregate_mobile.py > out/aggregate.txt
/opt/homebrew/bin/python3 seqdiag.py out/mission_video_messages.jsonl.gz sequence_diagram.png
