# GPT sorts a cluttered countertop for the dishwasher (MuJoCo)

## 中文摘要

- **任务**：MuJoCo 里一个杂乱台面，随机摆 6–9 件物品（盘、碗、杯、砧板、刀、锅、保温杯），带油渍/污渍贴图。GPT（`codex exec`，ChatGPT 订阅，`--sandbox read-only`，不用 API key）只看两张渲染图（墙上相机 + 房间角相机）和用户一句话，一次结构化调用给出：每件能不能进洗碗机、类别、脏污程度、是否油腻、篮子位置（P1–P4 / B1–B3 / defer / none）和装载顺序、洗涤程序。机器人只装它装得了的东西（**每局 1 个盘子，放 P4**），其余可洗物品**交给人**按 GPT 指定的位置放，不可洗的手洗。
- **评测**：12 个随机场景 × 中英文两种指令 = **24 局**，164 件物品，全部跑完（rc=0，峰值内存 ≤ 0.96 GB，每次只跑一个仿真）。
- **识别与分拣全对**：164/164 件检测到、能否进洗碗机 164/164、类别 164/164；24/24 个计划结构上可行（位置不冲突、不可洗物品没进篮子、没有在有空位时乱 defer）。
- **机器人装载**：24/24 局都把 GPT 指定的那个盘子装进 P4 并立住（倾斜 < 15°；23/24 < 10°，倾角 8.5–10.5°），无意外碰撞（未计划接触力 0 N），24/24 局最后按下了 GPT 选的程序并启动。
- **弱点是脏污程度判断，不是程序逻辑**：洗涤程序与真值规则一致 **16/24（67 %）**；但按 GPT **自己**标的脏污/油腻去套同一规则，一致 **22/24（92 %）**。可洗物品脏污等级只对 76/104（73 %），油腻 90/104（87 %，12 件真油腻里漏了 6 件）。8 次不一致中 6 次源于脏污标签偏差（多数是把轻度看成中度 → 选了更强的程序），2 次（种子 8，中英文都一样）是"小负载"阈值：4 件全轻度，规则要求 quick 只限 ≤ 3 件，GPT 不知道这个阈值，选了 quick。**没有一次在真值需要强力洗时选了更弱的程序。**
- **范围诚实说明**：只有盘子是机器人物理装载的（且只有 P4 一个位置可达）；碗、杯没有验证过的抓取（运动学检查 33 种姿态 0 可行），一律交给人；其余盘位 P1–P3 也交给人。水和洗涤不模拟，脏污只是视觉贴图。
- **GPT 成本（每个场景）**：2 次 codex 调用（1 次规划 + 1 次抓取前盘边重感知），中位 42 s（33–65 s），中位约 3.4 万 token（codex 报告的 "tokens used"），无限流退避、无重试；24 局合计 48 次调用、1037 s。订阅计费，无按次美元成本。
- **演示视频** `out/sort_demo.mp4`（164 s，1280×720）已检查：是正式流程（种子 10，英文指令，9 件物品），不是调试运行；全程显示 GPT 逐件决策 HUD，能看到盘子被推到台边、捏起、送入 P4 并立住，HUD 状态从 queued → loaded P4，其它物品标为 to human / deferred / hand wash，最后按 INTENSIVE + POWER。唯一瑕疵：推篮子回位时篮子停在 6 mm，严格阈值（< 6 mm）判为 BASKET_NOT_IN，之后关门到 0.06° 正常、程序正常启动（同一种子的两局评测也是这样）。

## Results (24 runs)

Source: `results.json` / `results_tables.md` (aggregate.py), per-run detail in `out/runs/<seed>_<lang>/result.json`.

| goal | runs | items found | safe / not-safe correct | category correct | soil level correct (safe items) | greasy correct | program = rule | program = rule on GPT own soil labels | plan feasible | plates loaded by robot, seated (<15 deg) | GPT calls | GPT s (median plan call) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| en | 12 | 82/82 (100 %) | 82/82 (100 %) | 82/82 (100 %) | 38/52 (73 %) | 44/52 (85 %) | 8/12 (67 %) | 11/12 (92 %) | 12/12 (100 %) | 12/12 attempted (11 also < 10 deg) | 24 | 498.6 (28.5) |
| zh | 12 | 82/82 (100 %) | 82/82 (100 %) | 82/82 (100 %) | 38/52 (73 %) | 46/52 (88 %) | 8/12 (67 %) | 11/12 (92 %) | 12/12 (100 %) | 12/12 attempted (12 also < 10 deg) | 24 | 538.4 (33.5) |
| all | 24 | 164/164 (100 %) | 164/164 (100 %) | 164/164 (100 %) | 76/104 (73 %) | 90/104 (87 %) | 16/24 (67 %) | 22/24 (92 %) | 24/24 (100 %) | 24/24 attempted (23 also < 10 deg) | 48 | 1037.0 (31.8) |

Per run:

| seed | goal | items | found | safe ok | category ok | program (GPT / rule) | plan feasible | robot loaded / seated | plate lean deg | to human | deferred | GPT calls | GPT s | sim s | wall s | max unplanned contact N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | en | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.05 | 4 | 0 | 2 | 43.7 | 176.1 | 99.7 | 0.0 |
| 1 | en | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 10.45 | 3 | 0 | 2 | 36.4 | 178.8 | 83.7 | 0.0 |
| 2 | en | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.42 | 2 | 0 | 2 | 39.6 | 177.8 | 79.6 | 0.0 |
| 3 | en | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.06 | 2 | 0 | 2 | 32.6 | 176.6 | 72.3 | 0.0 |
| 4 | en | 7 | 7 | 7 | 7 | intensive / intensive ✓ | yes | 1 / 1 | 9.08 | 4 | 0 | 2 | 42.2 | 176.7 | 99.3 | 0.0 |
| 5 | en | 8 | 8 | 8 | 8 | intensive / intensive ✓ | yes | 1 / 1 | 8.5 | 4 | 0 | 2 | 40.2 | 177.9 | 83.4 | 0.0 |
| 6 | en | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 9.05 | 4 | 0 | 2 | 53.8 | 177.4 | 111.6 | 0.0 |
| 7 | en | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.16 | 4 | 0 | 2 | 36.9 | 177.7 | 99.1 | 0.0 |
| 8 | en | 6 | 6 | 6 | 6 | quick / eco ✗ | yes | 1 / 1 | 8.68 | 3 | 0 | 2 | 36.3 | 176.0 | 75.7 | 0.0 |
| 9 | en | 6 | 6 | 6 | 6 | intensive / intensive ✓ | yes | 1 / 1 | 9.53 | 3 | 0 | 2 | 41.9 | 178.0 | 90.7 | 0.0 |
| 10 | en | 9 | 9 | 9 | 9 | intensive / intensive ✓ | yes | 1 / 1 | 8.69 | 4 | 1 | 2 | 50.4 | 179.6 | 102.6 | 0.0 |
| 11 | en | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 8.99 | 2 | 0 | 2 | 44.6 | 178.1 | 86.2 | 0.0 |
| 0 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.04 | 4 | 0 | 2 | 45.4 | 176.2 | 102.5 | 0.0 |
| 1 | zh | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 8.52 | 3 | 0 | 2 | 46.1 | 178.8 | 96.0 | 0.0 |
| 2 | zh | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.01 | 2 | 0 | 2 | 36.3 | 177.8 | 76.8 | 0.0 |
| 3 | zh | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 9.1 | 2 | 0 | 2 | 41.8 | 177.6 | 81.8 | 0.0 |
| 4 | zh | 7 | 7 | 7 | 7 | intensive / intensive ✓ | yes | 1 / 1 | 9.16 | 4 | 0 | 2 | 40.0 | 176.7 | 95.9 | 0.0 |
| 5 | zh | 8 | 8 | 8 | 8 | intensive / intensive ✓ | yes | 1 / 1 | 8.45 | 4 | 0 | 2 | 42.2 | 177.8 | 84.3 | 0.0 |
| 6 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.07 | 4 | 0 | 2 | 47.2 | 178.0 | 104.2 | 0.0 |
| 7 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.14 | 4 | 0 | 2 | 65.4 | 177.7 | 125.5 | 0.0 |
| 8 | zh | 6 | 6 | 6 | 6 | quick / eco ✗ | yes | 1 / 1 | 8.81 | 3 | 0 | 2 | 39.9 | 176.0 | 79.1 | 0.0 |
| 9 | zh | 6 | 6 | 6 | 6 | intensive / intensive ✓ | yes | 1 / 1 | 9.11 | 3 | 0 | 2 | 39.2 | 178.1 | 86.7 | 0.0 |
| 10 | zh | 9 | 9 | 9 | 9 | intensive / intensive ✓ | yes | 1 / 1 | 8.74 | 4 | 1 | 2 | 51.8 | 179.5 | 103.3 | 0.0 |
| 11 | zh | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 9.09 | 2 | 0 | 2 | 43.1 | 178.2 | 82.7 | 0.0 |

By item type (all 164 items):

| true type | items | safe/not-safe correct | category correct |
|---|---|---|---|
| board | 20 | 20/20 (100 %) | 20/20 (100 %) |
| bowl | 14 | 14/14 (100 %) | 14/14 (100 %) |
| flask | 10 | 10/10 (100 %) | 10/10 (100 %) |
| knife | 12 | 12/12 (100 %) | 12/12 (100 %) |
| mug | 34 | 34/34 (100 %) | 34/34 (100 %) |
| pan | 18 | 18/18 (100 %) | 18/18 (100 %) |
| plate | 56 | 56/56 (100 %) | 56/56 (100 %) |

### Soil / grease perception (dishwasher-safe items, all 24 runs; from `out/runs/*/result.json`)

| true \ GPT | light | medium | heavy |
|---|---|---|---|
| light (54) | **43** | 11 | 0 |
| medium (34) | 9 | **22** | 3 |
| heavy (16) | 0 | 5 | **11** |

Every soil error is one step off (no light <-> heavy confusion). Grease on the same 104 items: 84 true-negative,
6 true-positive, 8 false-positive, 6 missed (recall 6/12).

## Ground-truth program rule (as documented in code)

`truth.py::program_rule`, fixed before any GPT run and **never shown to GPT** (GPT only sees the user-manual program
descriptions in `planner.py::PROGRAMS_MANUAL`). It is applied to the items GPT actually planned into this load:

```
intensive : any item heavily soiled or greasy
quick     : every item lightly soiled and at most 3 items
eco       : otherwise (normal everyday load)
(None for an empty load)
```

"Program = rule" uses true soil/grease of the planned load. "Program = rule on GPT's own labels" applies the same rule to
GPT's own soil/grease labels, which separates perception errors from program-choice reasoning.

The manual GPT sees: eco = everyday dishes, normal soil; intensive = heavily soiled, dried-on/burnt, greasy; quick =
lightly soiled, just used, small loads, not for grease; plus dry and fruit (never correct here).

## Honest notes

1. **Program choice: 16/24 match the rule; 22/24 on GPT's own soil labels.** So the weak point is soil-level perception,
   not the program reasoning. The 8 disagreements:
   - s1 en, s1 zh, s6 en -> chose intensive, rule says eco. GPT over-read soil (light -> medium) and saw grease that is
     not there (s1: a plate; s6: a bowl medium -> heavy + greasy). Consistent with its own labels.
   - s11 en, s11 zh, s3 zh -> chose eco, rule says quick. GPT rated one item medium instead of light. Consistent with
     its own labels.
   - s8 en, s8 zh -> chose quick, rule says eco. No label errors: 4 items, all light, no grease. GPT called 4 items "a
     small load"; the rule's "at most 3 items" cutoff is ours and is not in the manual GPT sees. This is the only
     disagreement that is not a perception error, and it is arguably a rule-threshold choice, not a mistake.
   - Direction: 6/8 pick a *stronger* program than the rule, 2/8 a weaker one (quick instead of eco, no grease
     involved). GPT never chose quick or eco where the rule required intensive.
   - en and zh goals give the same program accuracy (8/12 each) and the same per-seed outcome except s6 (en wrong, zh
     right) and s3 (en right, zh wrong): the remaining variance is run-to-run sampling noise, not language.
2. **Only plates are physically loaded, and only into P4.** In every run the robot loads exactly one plate (the one GPT
   assigned to P4, which must lie flat in the front pick zone; the scene generator always places one there). The plate
   skill is edge-push + rim pinch + tilt + insert. P1-P3 are not reachable/clear: `clearance_study.py` shows the hand
   comes within 2.5 mm or penetrates (-3 to -20 mm) an already-seated neighbour plate, so plates beyond the first are not
   attempted.
3. **Everything else is handed to a human.** Bowls and mugs: no validated grasp. `bowl_mug_check.py` (kinematic, no
   physics) tried rim pinches (upside down / leaning) and mug handle grasps in B1-B3: **0 of 33 poses feasible** (IK
   error or hand/link collision with tines, rails, window). A rim pinch would leave the hand under the upside-down item.
   Handover counts over 24 runs: 46 bowls/mugs, 32 plates in P1/P2 (out of reach), total 78 items handed to the human,
   2 items deferred to the next load (s10, capacity), 60 non-dishwasher-safe items left for hand wash. The person is not
   simulated placing items; "handed to human" is a logged decision.
4. **Seated** = plate lean < 15 deg from vertical after release (24/24); 23/24 are also < 10 deg (s1 en: 10.45 deg).
5. **Basket push in s10 (en, zh, and the demo).** `push_basket` reports BASKET_NOT_IN with the basket at 6 mm; the
   success test is strict `< 6 mm`. The door then closed to 0.06 deg and the program started. Not counted as a failure in
   the tables above; flagged here so it is not hidden.
6. **Not simulated:** water, detergent, washing, soil removal. Soil and grease are visual decals; dishwasher internals
   (tine geometry, basket) are estimated.
7. **Cost per scene** (eval runs only, `logs/codex_calls.jsonl`, 48 calls, all ok on the first attempt, 0 rate-limit
   backoffs):

   | call | calls | seconds median (range) | "tokens used" median (range) |
   |---|---|---|---|
   | plan (2 images: wall + room camera, structured JSON) | 24 | 31.8 (21.8-55.3) | 18.7 k (5.7-19.2 k) |
   | plate-edge re-perception before the pick (1 image) | 24 | 10.7 (7.0-16.0) | 15.7 k (3.3-15.8 k) |
   | **per scene (2 calls)** | 24 | **42.0 (32.6-65.4)** | **34.3 k (9.4-34.9 k)** |

   Totals: 1037 s of GPT time, 692 k tokens for 24 scenes. The low token counts are the number codex prints
   ("tokens used"); they vary with what codex counts as cached, so treat tokens as approximate. ChatGPT subscription,
   no per-call dollar cost. Sim wall time per run 72-126 s plus GPT time; peak RSS 0.68-0.96 GB per process.

## Demo video check (`out/sort_demo.mp4`)

- 164 s, 1280x720, 30 fps, h264, 79 MB. Rendered at 11:47 by `run_sort.py --seed 10 --lang en` with
  `--video out/sort_demo.mp4 --keyframes frames/` under `runw.sh` (rc 0, peak 0.96 GB); log `logs/demo.log`.
- Nominal run (seed 10, 9 items, English goal), not a debug run: title "GPT sorts a cluttered counter (seed 10, goal:
  en)", sim-paused GPT plan overlay at the start (green = dishwasher-safe, red = not), then the per-item HUD (item, safe,
  slot, status, program + reason) for the whole video.
- Plate B is pushed to the counter edge, pinched, lifted, carried, inserted into P4 and released; HUD changes from
  queued -> "loaded P4" (green). A plate -> to human (P1), D/C bowls and F mug -> to human (B1-B3), E mug deferred,
  cutting board / knife / flask -> hand wash. Then basket in, door closed, INTENSIVE + POWER pressed (rule on the planned
  load: intensive, match).
- Cosmetic: the program reason line in the HUD is cut off after ~45 characters; the basket-push 6 mm flag (note 5).
- Contact sheet: `out/check/contact_sheet.png` (20 frames, every 8.2 s).
- `out/slot_7_5.mp4` is an earlier development clip (`slot_test.py`, no GPT, plate boxes from ground truth, labelled
  DEBUG) from the slot / clearance study, not a study result.

## How it works / reproduce

- `sort_scene.py` - seeded scene: kitchen + countertop dishwasher + Panda on a cart, 6-9 items with soil decals, wall
  and room cameras. Ground truth is returned for scoring only.
- `planner.py` - the single GPT planning call (prompt, JSON schema, basket layout, program manual) and the structural
  plan-feasibility check.
- `gpt.py` - the only path to GPT: `codex exec --skip-git-repo-check --ephemeral --sandbox read-only --output-schema
  ... --image=...`, stdin from /dev/null, ledger in `logs/codex_calls.jsonl`.
- `run_sort.py` - one trial: plan -> execute (plate skill / hand to human / defer / hand wash) -> close up -> press
  program + POWER -> score against `truth.py`. Writes `out/runs/<tag>/result.json`.
- `aggregate.py` -> `results.json`, `results_tables.md`.
- `batch.sh` - seeds 0-11 x {en, zh}, strictly one sim at a time under `runw.sh` (3 GB RSS watchdog).

```
./batch.sh                                   # 24 runs, ~45 min
./runw.sh logs/demo.log run_sort.py --seed 10 --lang en --video out/sort_demo.mp4 --keyframes frames/
python aggregate.py
```
