#!/bin/bash
ROOT="${GYJ_WORK:-$(cd "$(dirname "$0")/.." && pwd)}"   # repo root
# Every reported run, strictly ONE simulation process at a time (sequential loop), each under the 3 GB RSS watchdog.
# R_full_nominal_s1_zh (with the mission video) is run first, separately:
#   ./runw.sh logs/R_full_nominal_s1_zh.log agent.py --mode full --scen nominal --seed 1 --goal zh --video out/gpt_mobile_mission.mp4 --tag R_full_nominal_s1_zh
cd $ROOT/gpt_mobile || exit 1
run() { tag=$1; shift; [ -f out/results/$tag.json ] && { echo "skip $tag"; return; }
        ./runw.sh logs/$tag.log agent.py --tag $tag "$@"; grep "END" logs/$tag.log; }
run R_full_nominal_s2_en   --mode full   --scen nominal --seed 2 --goal en
run R_full_nominal_s3_zh   --mode full   --scen nominal --seed 3 --goal zh
run R_hybrid_nominal_s1_zh --mode hybrid --scen nominal --seed 1 --goal zh
for sc in dock_y estop door_latch button_miss person_path person_arm low_battery; do
  run R_full_${sc}_s1_zh   --mode full   --scen $sc --seed 1 --goal zh --video out/videos/full_${sc}.mp4
  run R_hybrid_${sc}_s1_zh --mode hybrid --scen $sc --seed 1 --goal zh
done
echo ALL_DONE
