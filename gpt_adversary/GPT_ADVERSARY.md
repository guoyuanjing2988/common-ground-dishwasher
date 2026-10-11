# GPT as an adversarial tester of the dishwasher-loading robot

## 中文摘要

**一句话结论：** 让 GPT 当"出难题的测试员"是有用的，但它和随机测试各有所长，不能互相替代。
- GPT 找到的机制数最多（9 个，全部是新机制）。
- 而且只有 GPT 找到了**最危险的几类**：机器人自称成功、其实没做成；碰掉东西；把盘子撞出夹爪。
- 稀疏随机（每次只改 1–3 组参数）找到了 GPT 完全没碰过的 3 个机制：
  - 碗篮导轨发涩；
  - 门铰链发紧；
  - 洗碗块碟子被挪动。
- 全参数均匀随机虽然 23/23 全部失败，但都死在前两步，只暴露出 3 种浅层模式，而且原因混在一起，必须再做最小化才能看清。

**主要数字**（MuJoCo，脚本化 Runner，Codex 视觉；每种方法 23 个场景）：

| | GPT 对抗 | 均匀随机 | 稀疏随机 |
|---|---|---|---|
| 失败率 | 10/23 | 23/23 | 22/23 |
| 不同失败模式（人工归并） | **9**（全为新机制） | 3 | 9（6 个新机制，3 个属已知故障或其变体） |
| 严重度 ≥3 的新模式（机器人自称成功、碰掉物体、撞伤盘子） | **4**（M1–M4） | 0 | 0 |
| 每个场景改动的参数个数 | 1.1 | 44 | 8 |
| 额外成本 | 6 次 GPT 调用，共 1054 s，15.7 万 token | 0 | 0 |

**按"严重度 × 合理性"排序的失败模式**（每个都已最小化，并用第二个随机种子复现）：
1. **M1 按键只是"一碰"，且从不确认面板。**
   - 按压在腕部力传感器读到 3 N 时就停。触控键实际超过 1 N 的时间只有 12 ms（判定需要 8 ms），所谓"保持"阶段只有 0.07 N。
   - 只要按键阈值 ≥1.375 N（标称 1.0 N），快洗就不会启动，而 Runner 照样报成功。
2. **M2 洗碗块在碟里转了 ≥34°**：夹歪了，落不进凹槽。
   - 转 28° 时更糟：洗碗块后来掉到内胆底部，Runner 仍报成功。
3. **M3 目标槽位已经有盘子**：搬运中撞上，机器人和盘子之间出现 200–320 N 的接触力，盘子被撞出夹爪。
4. **M4 台边有瓶子**：靠近台边、在机械臂扫掠范围内的瓶子被第 6 关节碰下台面，Runner 毫无察觉。
5. **M5 小角度偏转**：小车偏 1°，或洗碗机转 1.9°，开门照常成功，但取放洗碗块失手。
6. **M6 位置偏移**：洗碗机被推后 11 mm，或小车横向停偏约 1.4 cm，门把手就抓空。
7. **M7 碗篮导轨发涩**：阻力 ≥5 N 时，5 N 的轻夹就会打滑，碗篮拉不出来。
8. **M8 门铰链发紧**：约 2 N·m 摩擦加阻尼时，夹爪从把手上滑脱，门打不开。
9. **M9 盘子不是 8 寸**：9 寸及以上被视觉尺寸门限拒绝，这是安全拒绝。8.6 寸能通过门限，但夹持几何不对，掌心以 71 N 压住盘沿。
10. **M10–M12 其余三类**：
    - 洗碗块碟子被移动（属已知故障的变体）；
    - 盘子背面朝上；
    - 墙装相机被碰歪约 2°。

**诚实声明（模拟器假象和判据问题）：**
- **放置判据问题：** 盘子正常放好时就斜 7.5–9.5°，而原放置判据要求 <10°，微小扰动就会被判"没放好"。这是判据边缘的假象，不算失败模式；本研究改用 <15°（与 GPT_CONTROL 的"seated"一致）。
- **M1 的余量依赖模拟器：** 余量取决于接触刚度，但"不保持力、不看面板"这一弱点是真实的。
- **M3 的力被高估：** 预置的盘子与碗篮刚性相连，接触力偏大。
- **用了真值的检查：** Runner 自带的部分检查用的是真值（洗碗块是否进槽、盘子是否放好）。真机没有这些检查，所以 M2、M3 在真机上更可能是"默默失败"。

**GPT-L1 抽查：**
- 同样 2.5 N 的硬按键，L1 的按键技能会保持约 3 N 0.5 s，成功。这说明修复方向是对的。
- 目标槽位有盘子时，GPT 发现异常接触力后主动停止，并如实报告未完成，没有谎报成功。

**内存：** 共 190 次仿真进程，严格串行，峰值 RSS 944 MB（带视频录制的运行；不录视频最高 686 MB），看门狗从未触发；全程没有遇到限流。

---

## 1. Failure-mode catalogue (ranked by severity × plausibility)

Severity:
- 4 = unsafe or damage;
- 3 = silent failure (the robot believes it succeeded), or damage risk;
- 2 = detected failure, safe stop.

Plausibility is my judgement of how likely the scenario is in a real home: 3 = everyday, 2 = occasional, 1 = contrived.

"Threshold" comes from magnitude bisection toward nominal. "Repro" means the minimal scenario was re-run with seed 2, which gives new sensor noise and a fresh Codex perception call.

| # | Failure mode | Minimal reproducing scenario | Threshold found | Sev × Plaus | Class | Real robot would notice? | Found by (first) | Repro |
|---|---|---|---|---|---|---|---|---|
| M1 | Panel press is an impact tap, never verified → QUICK silently not started | `{"buttons":{"threshold_N":1.375}}` | 1.0 N passes; 1.375 N fails (GPT proposed 4 N) | 3×3 | robot weakness; margin sim-dependent | no (Runner claims success) | GPT r1 | ✓ |
| M2 | Pod rotated in its dish → bridges the groove; at 28° it later falls to the tub floor | `{"pod":{"location":"dish","yaw_deg":33.75}}` | 22.5° ok; 28° silent; 34° fails | 3×3 | robot weakness | no (pod-in-groove not sensed) | GPT r1 | ✓ |
| M3 | Target slot already occupied → carried plate strikes it, 200–320 N robot–plate force, plate knocked out | `{"basket":{"preload_slots":[6]}}` | structural | 3×3 | robot weakness; forces inflated (rigid pre-load) | partly (force is measurable, no guard) | GPT r1 | ✓ |
| M4 | Clutter in the arm sweep knocked off the counter, Runner unaware | bottle at (−0.035, 0.47), scale 0.8 | structural | 4×2 | robot weakness | no (12 N contact) | GPT r2 | ✓ |
| M5 | Small cart/appliance rotation → bar grasp still works, pod pick/placement misses | `{"dock":{"dx":0.0055,"yaw_deg":-1.0}}`; also `{"dishwasher":{"yaw_deg":1.875}}` | cart 0.5° → silent QUICK failure; 1° → pod missed | 2×3 | robot weakness | pick miss yes, groove miss no | GPT r4 / sparse | ✓ |
| M6 | Appliance nudged ~1 cm or cart docked ~1.4 cm off → pull-bar grasp misses | `{"dishwasher":{"dx":-0.01125}}` | 7.5 mm → pads missed instead; 11 mm → bar missed | 2×3 | robot weakness | yes (fingers close to 0 mm) | GPT r2 / random / sparse | ✓ |
| M7 | Sticky basket rail → 5 N bar squeeze slips | `{"basket":{"rail_friction_N":7.24,"damping":4.5}}` | 4.9 N stops at 0.237 m; 7.2 N not moved (nominal 0.3) | 2×3 | robot weakness | yes | **sparse only** | ✓ |
| M8 | Stiff/damped door hinge → bar slips, door stays shut | `{"door":{"hinge_friction_Nm":2.569,"damping":5.94,"spring_scale":1.14}}` | 1.43 N·m + 3.7 ok; 2.0 + 4.9 fails (GPT tried 2.0 with nominal damping: passed) | 2×3 | robot weakness | yes | **random/sparse only** | ✓ |
| M9 | Plate is not 8": ≥9" or ≤7.4" refused by the 95–110 mm radius gate; 8.6" accepted but 71 N palm–rim squeeze | `{"plate":{"radius_m":0.1143}}` | deterministic gate | 2×3 | robot weakness (scope limit by design) | yes (safe refusal) | GPT r2 / random / sparse | ✓ |
| M10 | Pod not at the taught spot (dish moved ≥2.6 cm) | `{"dish":{"dx":-0.017,"dy":-0.019}}` | smallest tried | 2×3 | robot weakness; **variant of known `pod_moved`** | yes | **sparse only** | ✓ |
| M11 | Plate face-down → edge push rides over the inverted rim (12 mm overhang instead of 45) | `{"plate":{"upside_down":true}}` | structural | 2×2 | robot weakness (rim profile simplified) | partly | GPT r3 | ✓ |
| M12 | Wall camera knocked ~2° pitch + 2 cm → 4.4 mm / 2° pose error, push under-travel; smaller bumps → 47–71 N rim squeeze | `{"camera_bump":{"pitch_deg":-2,"dx":0.02}}` | 1.5° + 15 mm passes | 2×2 | robot weakness | partly | GPT r6 / sparse | ✓ |

**Known faults that were re-found** (not counted as new): the person flag being ignored (S_21, S_23) and the pod lying on the counter (S_05). Both were found by sparse random only; GPT had been told to avoid them.

**Not catalogued:**
- **R_15:** a perception mis-fit (radius 155 mm) in an 18-group random scenario, not minimised.
- **S_22:** an insertion 12.6 mm off the slot, with a camera bump, a stiff pad and F/T noise. It did not reproduce with seed 2.
- **Sim/predicate artefact:** the 10° seating predicate against a nominal seated lean of 7.5–9.5°. This study counts "seated" as < 15°; the 10° verdict is kept in every run file as `plate_slot.in_slot_10deg`.

**Fix directions** (details per mode in `triage_modes.json`):
- **Force-held, verified actions (M1, M7, M8).**
  - Hold a regulated force instead of stopping at a threshold.
  - Read the panel display and retry.
  - Use a firmer, form-closed grip on the bars, with stall and slip detection.
- **Re-localise every run (M5, M6, M12).** Fiducials on the appliance and dock, or a bar touch-off, and a camera extrinsics self-check; correct the taught poses.
- **Perceive what the plan assumes (M2, M3, M4, M10).**
  - Measure the pod pose and grasp along its yaw.
  - Check basket slot occupancy.
  - Build a counter occupancy map for the swept-volume check.
- **Plate-class-aware recipes (M9, M11).** Radius-parameterised push/pinch/slot geometry and a face-up/face-down check.
- **Measured verdicts.** The Runner's "success" must come from sensors, not from its own command having been sent: panel display, pod and slot camera view.

Key frames are in `frames/M*/` (one still per step end plus `zz_final.png`, and the Codex overlay for perception modes). The montage is `out/failures.mp4` (1 min 59 s), with one title card and clip per mode, M1–M12 except M10. The M6 clip, run exactly at the 11 mm threshold, stopped with "door only at x deg" (partial bar grasp) rather than a clean bar miss: the outcome at the boundary varies between the two.

## 2. GPT adversary vs random baselines (same budget: 23 valid scenarios each)

| | GPT adversary | uniform random | sparse random (1–3 groups) |
|---|---|---|---|
| scenarios proposed / sampled | 23 | 23 | 23 |
| rejected by the validator | 0 | 0 (rejection sampling: mean 14.4 draws per valid sample) | 0 (mean 1.1 draws) |
| failures (severity ≥ 2) | 10 (43 %) | 23 (100 %) | 22 (96 %) |
| distinct automatic signatures | 8 | 3 | 9 |
| distinct failure modes (triaged) | **9** | 3 | 9 |
| of which new mechanisms (not the 7 known faults or variants) | **9** | 3 | 6 |
| new modes of severity ≥ 3 (silent / unsafe / damage) | **4** (M1–M4) | 0 | 0 |
| modes found only by this method | M1, M2, M3, M4, M11 | — | M7, M10, K-person, K-pod (M8 also by uniform) |
| parameters changed per scenario (mean) | 1.1 | 44 | 8.1 |
| steps reached before failing | all 9 steps (perception → buttons) | only steps 1–2 | steps 1–6 |
| GPT proposal calls / time / tokens | 6 / 1054 s / 157 k | — | — |
| Codex perception calls inside the sims | 39 | 23 | 27 |
| sim wall time | 891 s | 266 s | 458 s |

![discovery curve](out/discovery.png)

What the numbers mean:
- **Uniform random** perturbs about 17 groups at once. Every run dies at perception (plate radius outside the gate, 11/12) or at the door bar (appliance or cart offset, 11/11). So it never reaches the deep steps where the silent failures live, and each failure has many candidate causes. The two minimisations (R_01, R_03) took 20 sim runs each to attribute.
- **Sparse random** is a much stronger baseline than uniform. It fails often because uniform draws within the bounds are frequently large: a 3–5 cm appliance shift or 7–19 N rail friction. It found three mechanisms GPT never probed: basket rail friction, door damping, dish position.
- **GPT** fails less often. It deliberately chose moderate, realistic single-parameter changes and spent rounds 3–5 confirming tolerances: F/T drift up to 2 N/min, noise 0.4 N, heavy plate, big pod, mug next to the pod and box on the appliance top all passed.
  - Every one of its failures is single-cause, so no minimisation was needed to attribute it.
  - It is the only method that produced structured scenarios random sampling essentially never hits: a plate in exactly slot 6, a bottle in the push sweep, a pad threshold just above the robot's tap, a pod rotated inside its dish.
  - These are also the most severe modes.
- **GPT's blind spots** were the appliance mechanics (rails, hinge damping). It tried a hinge friction of 2.0 N·m with nominal damping, which turned out to sit just below the failure threshold, and then dropped the direction.
- **Honest bottom line:** for *count* of mechanisms, sparse random matched GPT at zero proposal cost. For *severity, plausibility and interpretability*, GPT was clearly better. The two are complementary; the best practical loop would seed GPT with sparse-random results.

## 3. Cost (ChatGPT subscription via `codex exec`, no API keys)

| item | calls | time | tokens |
|---|---|---|---|
| GPT scenario proposals (6 rounds, 11.8 → 33.7 k-char prompts) | 6 | 1054 s (rounds 1–3: 267–317 s; rounds 4–6: 56–70 s) | 157 k |
| Codex plate perception inside all sims (search + baselines + minimisation + video + L1) | 238 | — | 1.67 M |
| GPT-L1 commander spot checks (2 runs) | 29 decisions | 256 s | — |
| all codex calls this study (ledger `logs/codex_calls.jsonl`) | 282 | 3532 s | 2.31 M |

- No call failed.
- No rate or usage limit was hit; the 60 s → 600 s back-off in `codex_util.py` was never needed (`logs/rate_limits.jsonl` absent).
- Simulations: 190 processes, strictly one at a time, each under the 3 GB RSS watchdog (`runw.sh`). Peak RSS was 944 MB (video-recording runs; 686 MB without video), median 577 MB, and the watchdog never fired.

## 4. GPT-L1 commander on adversarial scenarios (optional, budgeted to 2 runs)

| scenario | scripted Runner | GPT-L1 (gpt_control stack, unchanged) |
|---|---|---|
| stiff pad, 2.5 N (M1 class) | silent failure (QUICK not running, claims success) | **success**, 11 decisions, 87 s GPT. The L1 press skill holds ~3 N for 0.5 s and GPT reads the panel |
| target slot occupied (M3) | plate knocked out, stops on a ground-truth check | **failure, reported honestly**: GPT saw abnormal contact force on insertion, re-perceived, then stopped with "未完成…插入时出现异常接触力…未启动快洗"; 18 decisions |
| sticky rail 7.2 N (M7) | basket not pulled | not run (deadline) |

## 5. Method

**Scenario DSL** (`dsl.py`, rendered verbatim into the GPT prompt by `dsl.prompt_doc()`):
- **What it covers.** 48 bounded parameters plus structured groups:
  - plate radius, mass, μ, wetness, pose, colour and face-down;
  - pod location, offset, yaw, size and mass; dish offset;
  - door hinge friction, damping and spring; basket rail friction, damping and pre-loaded slots;
  - up to 4 clutter objects (mug, bowl, bottle, cereal box, cutting board) on the counter or the appliance top;
  - lighting (key and ambient scale, colour temperature, key direction);
  - camera bump (3 rotations and 3 translations); dock error; appliance pose;
  - depth noise, F/T noise and drift; person timing and distance; button threshold and debounce.
- **What it rejects** (all of these are checked):
  - unknown keys and out-of-bounds values;
  - interpenetrating footprints and items off the counter;
  - implausible plate areal density (5–35 kg/m²) or pod density (0.8–2.5 g/cm³);
  - a pod that does not fit inside its dish;
  - a cart bumper inside the cabinet (dx ≥ 0.30 |sin yaw|).

**Harness** (`adv_robot.py`):
- Imports `full_pipeline` and `gpt_control` unchanged and builds the scene from the scenario. Clutter objects are free bodies; the bumped camera renders while the robot keeps its old calibration.
- The robot's own model stays nominal: IK and collision model, `plan.*` constants and camera calibration. That gap is exactly what is being tested.
- The scripted Runner is `gpt_control/scripted.py`'s `run_hardcoded` sequence. All Codex artefacts and logs stay inside `gpt_adversary/`.

**Search loop** (`search.py`):
- Each round's prompt contains the system description, the DSL, the 7 known faults, all previous scenarios with compact outcomes and the distinct signatures found so far.
- GPT returns 3–5 scenarios, each with a hypothesis, an expected failure and a real-world story (strict output schema).
- The sims run sequentially and the outcomes are fed back. 6 rounds produced 23 scenarios.
- Baselines: the same count of uniform samples (every parameter, rejection-sampled until valid), and sparse samples (1–3 random groups).

**Outcome classes** (`classify.py`, ground truth, scoring only):
- 4 unsafe: person-near overspeed, a robot collision above 50 N with anything other than the handled plate, or an object off the counter;
- 3 silent: the Runner claims success but the mission failed;
- 2 detected: the Runner stops;
- 1 degraded: success with a 20–50 N contact, clutter pushed, or a robot–plate squeeze above 50 N;
- 0 success.

A failure is severity ≥ 2.

**Triage:**
1. Automatic signatures (stopping step | reason | GT criteria failed | damage flags) are only the first cut. They over-merge (face-down plate and camera bump share "push left # mm overhang") and under-merge (the same bar-miss arises from appliance and from cart offset).
2. I assigned modes by mechanism after reading the step logs (`triage_modes.json`).
3. Minimisation (`minimize.py`):
   - greedy one-group-at-a-time revert to nominal, keeping a revert only if the same mode persists (same stopping step, GT criteria and damage classes);
   - then 1–3 halvings toward nominal;
   - then a seed-2 reproduction.

   In total, 18 entries and 101 sim runs.

**Caveats** (be careful with these):
- **Ground-truth checks in the Runner.** The scripted Runner's own step checks partly read ground truth (pod in groove, plate seated, overhang): this is inherited from `full_pipeline`. "Detected" therefore overstates what a real robot would notice; the catalogue's "Real robot would notice?" column corrects for this.
- **One seed per scenario in the search.** Codex perception is stochastic and minimal scenarios were reproduced only once more. Thresholds are bracketed by 1–3 bisection steps, so they are coarse.
- **Sim-dependent magnitudes.**
  - The M1 margin is a contact transient.
  - M3 forces come from rigid pre-loaded plates.
  - M7/M8 slip thresholds depend on the finger–bar friction model.
  - The door latch force of a real dishwasher (20–50 N) is not modelled, so M8 is if anything optimistic.
- **Manual clustering.** The mode clustering and the plausibility scores are my judgement; the evidence for each is in the run files listed in `results.json`.
- **Unrealistic scenarios.** The uniform-random scenarios change about 17 groups at once. Each value is in bounds, but the combinations are not realistic homes, and none of the catalogued modes depends on such a combination.

## 6. Files

- `dsl.py`: DSL, bounds, validator and samplers.
- `adv_robot.py`: scene builder, robot subclass, scripted run and streamed recorder.
- `run_scenario.py`: one sim, scripted or `--commander L1`.
- `runw.sh`: RSS watchdog.
- `search.py`: GPT and random searches, and re-classification.
- `classify.py`, `minimize.py`, `render_modes.py`, `report.py`, `codex_util.py`: classification, minimisation, rendering, reporting and rate-limit back-off.
- `scenarios/`: every scenario JSON:
  - `G_*` (GPT), `R_*`, `S_*`;
  - `min/` (minimisation runs) and `min_<entry>.json` (minimal results);
  - `modes/M*.json` (the clip scenarios);
  - `l1/`.
- `runs/`: the full result JSON of every sim. `logs/search_{G,R,S}.json` holds the search state with GPT hypotheses and the per-round analysis; GPT prompts are in `logs/prompts/`; minimisation traces are in `logs/minimize/`.
- `results.json`: all entries, modes, metrics, minimisation, memory and costs. `triage_modes.json` is the catalogue source.
- `frames/M*/`: key frames. `out/clips/`: per-mode clips. `out/failures.mp4`: the montage. `out/discovery.png`: the discovery curve.
