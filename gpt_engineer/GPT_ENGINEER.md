# GPT as the engineer: can it close real sim problems on its own? (2026-10-11)

## 中文摘要

**一句话结论：** 在这三个真实问题上，GPT（`codex exec`，ChatGPT 订阅，只读沙箱）**都是第一轮就修好了**，补丁小（2–8 行）、物理上说得通、独立审查判为 clean、在留出的随机种子上也成立。P1 又独立重复了 2 次，也都是第一轮修好（8 mm、9 mm）。
- 但三个问题都有一个共同点：**答案就是一个现成的旋钮，而且代码注释或日志已经把方向指出来了**。
- 把这个旋钮冻结以后（P2b），GPT 用完 8 轮预算，**0/5**，平均倾角只从 9.8° 降到 9.1°。它 8 轮都在"松手/夹持/下降速度"这一类上打转，没有换过思路。
- **总体判断：** 在有冻结评分器、规则检查和留出回归的框架里，GPT 能当"读代码、调一个旋钮"的工程师，5 个被接受的补丁都可信。但要它想出新机制，这次没做到。样本只有 4 个问题、一个代码库，只能算案例，不能当基准。

**逐题：**
- **P1 复现历史修复（插盘瞄准偏置 8 mm → 退回 3 mm）：**
  - 基线 2/4（−5 mm 和 −0.5° 两个工况盘子掉进隔壁一格，倾 20°）。
  - GPT 第 1 轮就改回 **8 mm**。理由：最低的盘沿在法向 +6.8 mm，要把它对到槽中间并给停靠误差留余量。
  - 结果 4/4；另外 3 次完整任务和 3 个位姿工况 6/6 通过；作弊审查 clean。
  - GPT 用时：1 次调用 144 s。当时人工调试用了约 10 min、6 次仿真，先试了 0 mm 和 −3 mm（方向反了），才到 6 mm、8 mm。
  - 另做了 2 次独立重复（同样的提示，重新调 codex），都是第 1 轮 4/4：一次又是 8 mm（结果和第一次逐位相同），一次是 9 mm（倾角在 3/4 工况里更小）。两次审查都 clean。所以 P1 是稳定的，不是运气。见 §1b。
- **P2 盘子倾斜（原来的开放问题，静态推车）：**
  - 基线 0/5，最终倾角 8.8–10.9°。
  - GPT 第 1 轮把插入瞄准偏置 3 mm 改成 **10 mm**。结果 5/5，最终倾角 1.4–2.4°，盘底仍在自己的两根齿之间（+1.9…+2.8 mm）。
  - **因为"一轮就修好"很意外，我单独复核了：**
    - 冻结评分器和代码自带判据数值完全一致（7.93°→1.48°）；
    - 渲染图肉眼可见盘子竖直；
    - 另外 10 个留出种子 10/10（1.6–2.3°），同种子基线 8.9–10.8°；
    - 感知噪声加大到 3 mm，5/5（1.6–2.4°）。
    - 加上 3 个守卫种子和 5 个测试种子，补丁后合计 23/23 次完整任务通过。
  - **为什么这么容易：** 原代码注释里就写着"aim 3 mm toward the convex side … this shortens the lean"。GPT_CONTROL 当初把倾斜归因于收尾动作（推篮、关门），**这个归因不对**：倾斜的大头（7.3–8.0°）在插入那一刻就定了。
- **P2b（P2 加约束：瞄准偏置冻结）：**
  - 8 轮 **0/5**，最好一轮平均最终倾角 9.07°（基线 9.78°）。
  - 第 2 轮夹持力调太小，盘子掉到隔壁一格（22°）。
  - 这说明 GPT 能读懂代码、提出可检验的假设，但**物理直觉弱**。连续 8 次都没跳出"松手/接触瞬态"这一个假设族；一个能动盘子位姿的办法（例如松手后用指尖把盘子推正）一次都没提。我也没验证这类办法能不能做到 < 6°。
- **P3 腕部 F/T 误触发（把滤波退回）：**
  - 在当前代码里，只有"完整任务 + V 形块预压"时按键会误触发：手指离面板还有 7–10 mm 就停了，面板没反应，监督器重按。
  - 基线 1/3，共 3 次误触发。
  - GPT 第 1 轮把 F/T 信号改成 **50 ms 滑动平均**，在信号源处统一滤波，所有力判据共用，阈值没动。
  - 结果 3/3；另外 2 次完整任务 + 2 个停靠工况 4/4，0 次误触发；作弊审查 clean。
  - 代价：滤波有约 25 ms 滞后，按键冲击峰值 28–52 N（基线成功按键 23–42 N）。
- **P4（停靠 y −4 cm 的传感布置）：** 因截止时间放弃，没有结果。

**后续复核（12:00 后）：**
- 逐个核对了每个被接受的补丁（P1、P1 重复 2、P1 重复 3、P2、P3）：
  - **作弊审查都做了**，都是单独一次 codex 调用，结论都是 clean。规则检查 0 个 flag。我自己也读了 diff：没有瞬移、焊接、读真值，也没有放宽阈值。
  - **标称任务回归检查（≥ 3 次）：** P1、P2 原本就做了。重复 2 的补丁和 P1 完全相同（8 mm），直接继承 P1 的 6/6。重复 3（9 mm）和 P3 当初没做够（重复跳过了守卫；P3 的留出集里只有 2 次完整任务），**这次补跑了**：P1 重复 3 的 3 次留出完整任务 **3/3**（插入后倾 1.8–2.4°）；P3 多跑 1 次留出完整任务 **1/1**（按 3 次，无误触发，面板 RUNNING）。
- **P2 独立复核**（`harness/verify_p2_independent.py`，自己写的判据，不用 harness 的 `case_pass`，直接读原始 JSON）：
  - 补丁后 5/5 + 3/3 + 10/10 + 噪声 3 mm 5/5，**共 23/23**；同种子基线 0/10。
  - 冻结评分器给的插入后倾角，和代码自己算的 `plane_tilt_deg` 在全部 33 次任务里一致。
  - 评分器和评估器的文件时间早于基线运行，没有改过，所以基线和补丁用的是同一个判据。
  - 补丁版代码和原始 `full_pipeline/` 比，只差这一行。基线就是原始代码，没有被故意削弱。
  - 渲染图肉眼可见：基线倾 9.7°，补丁后 2.1°。
  - **P2 一轮就过，原因不是作弊**：要调的旋钮和方向，原代码注释里已经写了。
- **安全：**
  - 所有仿真串行、都在 3 GB RSS 看门狗下跑，共 127 个进程，峰值 RSS 0.69 GB（进程内 ≤ 0.83 GB），看门狗一次没触发。
  - Codex 调用全部 `--sandbox read-only`，stdin 为 /dev/null。审计了 codex 执行过的全部命令：没有运行过 Python 或仿真；越出沙箱的只有一次，是搜 AGENTS.md，没有读到原流水线里的人类答案。
  - 没碰到限流。

---

## 0. Summary table

"Metric by iteration" lists test cases passing, from iteration 0 (the baseline) to the last iteration. Diff size counts the changed lines in the accepted (or best) cumulative patch. GPT s is the engineer calls only; the review call is listed separately.

| problem | metric by iteration | diff | GPT s (engineer + review) | generalisation (held-out nominal) | cheating verdict |
|---|---|---|---|---|---|
| P1 insert aim, run 1 | 2/4 → **4/4** | 4 lines, `planner.py` (3 → 8 mm) | 144 + 56 | 6/6 (3 full missions, 3 poses) | clean (codex review + rule check 0 flags) |
| P1 replication 2 | 2/4 → **4/4** | 2 lines (3 → 8 mm, same value as run 1) | 48 + 18 | inherits run 1's 6/6 (identical patch; per-case states identical too) | clean |
| P1 replication 3 | 2/4 → **4/4** | 2 lines (3 → **9 mm**) | 46 + 19 | **3/3** full missions, run post hoc (`runs/p1_aim_rep3/it1/guard_post/`; insert lean 1.8–2.4°) | clean |
| P2 plate lean | 0/5 → **5/5** | 6 lines, `pipeline.py` (3 → 10 mm) | 62 + 19 | 3/3 in loop, plus 10/10 extra seeds and 5/5 at 3 mm perception noise; **independently re-derived as 23/23** (baseline 0/10) | clean |
| P2b lean, aim frozen | 0/5 for all 8 iterations (mean lean 9.78 → best 9.07°) | best: 6 lines | 8 calls, 469 | n/a (nothing accepted) | n/a; rule check 0 flags |
| P3 F/T false triggers | 1/3 → **3/3** | 8 lines, `planner.py` (50 ms moving average) | 64 + 32 | 4/4 in loop (2 full missions, dock0, pose3), plus mission5 post hoc **1/1** (3 presses, DONE, panel RUNNING) | clean |
| P4 dock sensing | not run (dropped) | – | – | – | – |

**Did every accepted patch get both checks?** (re-audited at hand-over)
- **Cheating review:** yes, all 5 accepted patches. Each was a separate read-only codex call on the cumulative diff (`review` in each `ledger.jsonl`), and every verdict was `clean`. I also read all 5 diffs myself.
  - None has a teleport, weld, `qpos`/`qvel` write, model edit, ground-truth read or a seed/case branch.
  - None loosens a threshold: the P3 press threshold stays 3 N, and the < 6° and in-slot predicates live in the frozen harness.
- **Nominal regression (≥ 3 missions):**
  - P1 run 1 (3 missions + 3 poses) and P2 (3 + 10 + 5 seeds) were already covered.
  - The loop skipped guards for the replications. Replication 2's patch is byte-identical to run 1's, and its 4 test cases reproduced run 1's states exactly, so it inherits run 1's guard. Replication 3 uses a different value (9 mm), so I ran 3 held-out full missions for it.
  - P3's in-loop guard had only 2 full missions (plus dock0 and pose3). Its 3 test cases are themselves full missions, but held-out, so I added mission5.
- **P2 metric is computed the same way as the baseline:**
  - `score.py` and `eval_full.py` are dated 09:49 and 09:52, before the baseline ran at 10:25. Neither was modified afterwards.
  - The baseline sandbox is byte-identical to `../full_pipeline/`, and the patched sandbox differs from it only in the aim line.
  - The frozen scorer's lean after insert equals the pipeline's own `plane_tilt_deg` in all 33 P2 missions.
  - My re-derivation uses a stricter predicate than the loop: also all 9 steps ok, no wrong-tine or other contacts, and absolute values on door and basket. It reproduces every pass and fail.

## 1. Results per problem

**Harness, in one paragraph**
- Each iteration is one `codex exec --sandbox read-only -C <current-best sandbox copy> --output-schema …`, with stdin from /dev/null.
  - The prompt holds the problem statement, the rules, the metric history, the cumulative diff, and the last run's frozen-scorer states and step logs.
  - Up to 4 key frames go in as `--image`.
  - It returns `{diagnosis, hypothesis, patch, expected_effect, test_plan}`.
- The harness, not codex, applies `patch` to a fresh copy. It rejects edits to frozen files, scoring functions or physics (rule check), runs each test case as one sim process under `runw.sh` (3 GB watchdog), and scores with `harness/score.py`.
  - `score.py` is outside the sandbox: a frozen copy of the original `plate_slot_state`, applied after insert, after push, after close and at the end.
- When the target is met:
  - held-out nominal missions (the guard);
  - a separate codex cheating review of the cumulative diff (working dir = the original code).
- Budget: 8 iterations.

### P1: replay of a known historical fix (mobile base, insert aim 8 mm reverted to 3 mm)

| iter | metric (of 4) | per case after insert: lean °, bottom offset mm | diff | GPT s | sim wall s |
|---|---|---|---|---|---|
| 0 (3 mm) | 2 | pose3 **20.6 / −18.6 (one tine over)**, pose11 **11.3 / −7.7 → ABORT**, pose2 3.4 / +0.9, dock0 7.9 / −4.1 | – | – | 373 |
| 1 | **4 ✓** | pose3 7.0 / −2.9, pose11 6.6 / −2.6, pose2 1.2 / +3.2, dock0 3.5 / +0.6 (all DONE) | 4 lines, planner.py | 144 (78 k tokens) | 429 |

**Generalisation and review**
- **Guard 6/6:** 3 full charger → dock → task → charger missions (random plate and start pose) and pose sweeps x −2 mm, y −5 mm, yaw +0.5°.
- **Cheating review: clean.**

**Final diff (`diffs/p1_aim_FINAL.diff`):**
```diff
-        self.insert_aim = .003                              # insert aim offset toward -plate normal (m)
+        # The lowest rim is 6.8 mm toward +normal from the plate centre.
+        # Centre that rim in the gap with clearance for docking error.
+        self.insert_aim = .008                              # insert aim offset toward -plate normal (m)
```

**Against the original (manual) fix**, reconstructed from `../mobile_pipeline/out/dev/`:
- First insert failure seen at 23:41 (pose sweep x −5 mm).
- The dedicated aim experiments ran 00:17:50–00:27:50: **~10 min wall, 6 sim runs**.
  - aim 0 mm: fail, −18.3 mm;
  - touchdown filter: fail;
  - −3 mm: nominal fail;
  - 6 mm: ok, 5.4°;
  - 8 mm at x −5 mm: ok;
  - 8 mm at x +5 mm: ok.
- It tried the **wrong direction first**. GPT went straight to the same value, 8 mm, with a geometric argument, in one 144 s call and one 7 min test round.

**Fairness caveats**
- The sandbox comment that said "re-tuned … (was 3 mm)" was scrubbed.
- But `pipeline.py` still says the leading lip sits +6.8 mm along the normal, and that is exactly what GPT used.
- The codex audit shows it never read `../mobile_pipeline`.

**Replications:** see §1b.

### P2: plate lean, an open problem (static M1 cart, 5 seeds, truth perception)

| iter | metric (of 5) | final lean ° per seed (after insert → push → close → final, seed 0) | diff | GPT s | sim wall s |
|---|---|---|---|---|---|
| 0 | 0 | 9.43, 8.79, 10.94, 8.94, 10.81 (seed 0: 7.93 → 8.36 → 8.83 → 9.43) | – | – | 115 |
| 1 | **5 ✓** | 2.28, 1.55, 2.35, 1.41, 2.20 (seed 0: 1.48 → 1.78 → 2.00 → 2.28) | 6 lines, pipeline.py | 62 (50 k tokens) | 115 |

**Generalisation and review**
- **Guard 3/3:** held-out seeds 5–7 at 1.9–2.4°.
- **Cheating review: clean.** Its one caveat: 10 mm leaves limited clearance if the grasp estimate is off.

**Final diff (`diffs/p2_lean_FINAL.diff`):** `org = slot - .003 * R_slot[:, 2] …` → `org = slot - .010 * R_slot[:, 2] …`

**Extra verification (`runs/p2_verify/`)**

| check | n | pass | final lean ° | bottom offset mm |
|---|---|---|---|---|
| more held-out seeds 8–17, patched | 10 | **10/10** | 1.56–2.29 | +1.9…+2.6 |
| same seeds 8–12, original code | 5 | 0/5 | 8.92–10.82 | −5.1…−7.1 |
| perception noise 1 → 3 mm, seeds 0–4, patched | 5 | **5/5** | 1.62–2.39 | +1.8…+2.6 |

**What else was checked**
- **Scorer agreement:** the frozen scorer agrees exactly with the in-code predicate (insert seed 0: 7.93° → 1.48° in both).
- **Frames:** the edge-on renders show the plate visibly vertical (`runs/p2_lean/it1/eval/frames/seed2_after_close.png` vs `it0/…`).
- **Collateral:**
  - all 18 patched missions completed all 9 steps;
  - no wrong-tine or other contacts;
  - door closed, basket in;
  - unplanned robot contacts were unchanged vs the baseline (hand–rim ≤ 8 N in seed 1 in both).

**Why one iteration was enough**
- The original line was commented "aim 3 mm toward the convex side: a seated plate leans that way onto its own tine, so this shortens the lean". The knob and its direction were documented in the code.
- The baseline data GPT was given also showed that 7.3–8.0° of the 8.8–10.9° is already there right after insert. Push and close add only 1–3°.
- So the earlier GPT_CONTROL explanation ("a property of the close-out skills") was **wrong about the main component**.
- The growth during push/close is still there after the fix (+0.6–0.9°), just from a smaller starting lean.

**Not tested**
- The 10 mm aim was not tested on the mobile base, and not with real Codex perception, only truth + 1 or 3 mm noise.
- The bottom now sits +2–3 mm on the other side of centre. Margin to the far tine is about 9 mm.

### P2b: plate lean with the aim offset FROZEN (rule-enforced; a stress test of GPT beyond the obvious knob)

| iter | metric | mean final lean ° | hypothesis (GPT, abridged) | diff lines | GPT s |
|---|---|---|---|---|---|
| 0 | 0/5 | 9.78 | – | – | – |
| 1 | 0/5 | 9.26 | add a lightly constrained settling phase before opening (kept) | 5 | 66 |
| 2 | 0/5 | **21.99** | reduce the pinch to ~5 N before descent → **plate slipped and dropped one tine over** (discarded) | 9 | 67 |
| 3 | 0/5 | 9.42 | open faster so both pads disengage before rotation (discarded) | 2 | 47 |
| 4 | 0/5 | 9.26 | hold the firm grasp at touchdown before unloading (kept, tie) | 2 | 51 |
| 5 | 0/5 | **9.07** | slow the descent to reduce contact impulse (kept, best) | 6 | 56 |
| 6 | 0/5 | 9.08 | require sustained weight support before stopping descent | 6 | 65 |
| 7 | 0/5 | 9.33 | bounded integral correction of servo error at the hand target | 12 | 65 |
| 8 | 0/5 | 9.33 | open fully before retracting | 4 | 53 |

**Outcome**
- 8 iterations, about 8 min of GPT time and 16 min of sim. **Not solved.**
- Best: −0.7° mean (9.78 → 9.07), mostly from less growth during push/close.
- No rule violations and no rejected patches.

**What GPT never tried**
- Changing anything geometric that the rule allowed: a corrective nudge after release, or inserting with a slight pre-tilt the other way.
- It also never questioned its own hypothesis family.
- Whether < 6° is reachable without moving the aim is **unknown**; I did not test it.

**Harness note**
- A first P2b attempt was restarted after 1 iteration (`runs/_aborted_p2b_tiebreak_v1/`; it tested release unloading, 0/5, lean unchanged).
- The pass-count metric alone kept any 0/5 patch as "best", so I added a continuous tie-break (mean final lean).

### P3: wrist F/T false triggers (mobile; the filters reverted in a copy)

**Revert used**
- Insert touchdown back to EMA α 0.2, > 0.5 N for 5 samples, ±40 mm window.
- Panel press stop back to "raw F/T along the approach axis > 3 N", one sample.
- The original pre-fix code was not preserved, so this is a **reconstruction** from the MOBILE_RESULTS text.

**Reproduction**
- With the brake/noslip fixes in place, the revert does **not** fail in docked runs (no V-blocks, 2 runs checked).
- It fails only in full missions with the 100 N V-block preload. The press stops 7–10 mm short of the pad at 3.1 N, the panel stays OFF, and the supervisor retries.
- Insert false touchdowns did not reproduce.

**Metric:** a mission passes only with exactly 3 press attempts (counted by the frozen supervisor), DONE, plate in slot, door closed, basket in and panel RUNNING.

| iter | metric (of 3) | press attempts per mission | stop force N / TCP short of pad at stop | GPT s | sim wall s |
|---|---|---|---|---|---|
| 0 | 1 | 5, 4, 3 (3 false triggers, all at 3.1 N, 7–10 mm short) | 3.1–15.2 N | – | 405 |
| 1 | **3 ✓** | 3, 3, 3 | 3.0–3.7 N, 1.4–2.6 mm (pad contact) | 64 (46 k tokens) | 405 |

**Generalisation and review**
- **Guard 4/4:** missions 3–4, dock0, pose x −5 mm, all with 3 presses.
- **Cheating review: clean.**

**Final diff (`diffs/p3_ft_FINAL.diff`, 8 lines)**
- A `deque(maxlen=25)` (50 ms at 500 Hz) of world-frame wrist forces, filled from every ArmState sample.
- `hand_force_world()` returns the window mean, so every force decision uses the filtered signal: press stop, press-and-hold, insert touchdown, door/basket guards.
- Thresholds unchanged.

**Against the original fix**
- The original used per-consumer EMAs with hold counts and narrower windows: press > 2.5 N for 10 ms near the pad; touchdown > 2 N for 20 ms.
- GPT's version is simpler and global. It adds about 25 ms of lag everywhere.
- Press impact peaks are 28–52 N, vs 23–42 N for the baseline presses that did reach the pad, and 22–49 N reported for the original fix. Comparable, but the 52 N is the highest seen.
- The number of iterations the original fix took cannot be separated from the dev logs.

### P4: dock y −4 cm sensing (optional)
Set up (`base/p4_dock`, `harness/p4_dock_statement.txt`, sensor-placement whitelist), but **dropped at 11:32** for the time deadline. The baseline was interrupted mid-run (`runs/_dropped_p4_dock/`). No result.

### 1b. P1 replications (same prompt, fresh codex calls)
Two more runs of P1 with the identical prompt and baseline evidence, each a fresh `codex exec` call. The loop skips guards for replications; replication 3's guard was run post hoc (see §0).

| run | GPT value | GPT s / tokens | after-insert lean °, bottom mm: pose3 / pose11 / pose2 / dock0 | metric | review |
|---|---|---|---|---|---|
| 1 | 8 mm | 144 / 78 k | 7.0 −2.9 / 6.6 −2.6 / 1.2 +3.2 / 3.5 +0.6 | 4/4 | clean |
| 2 | 8 mm ("increase by 5 mm") | 48 / 44 k | 6.96 −2.9 / 6.61 −2.6 / 1.15 +3.2 / 3.5 +0.6 (identical to run 1: same patch, deterministic sim) | 4/4 | clean |
| 3 | **9 mm** ("shift +6 mm along x") | 46 / 49 k | 6.03 −2.0 / 5.69 −1.6 / 1.29 +3.0 / 2.53 +1.7 | 4/4 | clean |

**What the replications show**
- 3/3 runs fixed P1 in one iteration, with the right direction and a value in the 8–9 mm band. The human-like history found 6–8 mm worked.
- All three diagnoses cite the same evidence: the two failures detect touchdown but sit −7.7 to −18.6 mm off, and the basket is pushed about 13 mm during insert. All three conclude the aim leaves too little clearance from the facing tine.
- Run 1 is the only one that derived the value from the +6.8 mm rim geometry. Runs 2 and 3 extrapolated from the per-pose trend in the logs (+5 mm dock x error seats best).
- So the outcome is stable across GPT samples. The size of the step (+5 or +6 mm) varies a little, and 9 mm leans 0.9–1.0° less than 8 mm in 3 of 4 cases (pose2 is 0.1° worse).
- The replications are not independent of the prompt. All three saw the same logs and the same `pipeline.py` comment about the +6.8 mm lip.

## 2. Cheating / overfitting checks

**Rule check (automatic, before any sim run)**
- Edits are allowed only in the task-code files.
- Scoring helpers must be AST-identical: `plate_slot_state`, `pod_state`, `touching`, `plate_c`, `pR`.
- Rejected outright: `qpos`/`qvel` writes, model or `opt` edits, welds, mocap.
- Flagged: ground-truth reads.
- P2b also froze the aim line and `plan.plate_slot`.
- Result: 0 rejections and 0 flags in 13 applied patches (P1 ×3, P2, P3, P2b ×8).

**Frozen metric**
- `harness/score.py` lives outside the sandbox and is applied through method hooks.
- So in-code predicates could not move the score. GPT never tried to change them.

**Codex cheating review (separate call, working dir = original code)**
- All 5 accepted patches (P1 ×3, P2, P3): **all `clean`**. The reviewers flagged only robustness risks of tuned constants.

**Codex transcript audit (`logs/codex_audit.json`)**
- 19 transcripts, 93 shell commands, all read-only (rg/sed/cat/ls).
- 0 Python or sim executions.
- 1 command touched a path outside its sandbox: `rg --files …/gpt_engineer -g AGENTS.md`, which found nothing.
- It never opened `../mobile_pipeline` or `../full_pipeline`, which contain the human fixes.

**Generalisation**
- Every accepted patch passed held-out nominal runs: P1 6/6 (replication 2 identical), P1 replication 3 3/3 (post hoc), P2 3/3 + 15/15 extra, P3 4/4 + 1/1 (post hoc).
- Each P2 and P3 test or guard run is itself a full nominal mission.

## 3. Honest verdict: what GPT-as-engineer can and can't do here

**Overall:** inside this harness, GPT is a reliable engineer for **well-instrumented, single-knob failures**: 5/5 accepted patches across 3 problems and 3 P1 samples, all in one iteration, all clean, all generalising. It is **not** shown to be an engineer for problems that need a new mechanism: P2b went 0/5 over 8 iterations, with no change of strategy. The one-iteration P2 result was re-verified independently and is real. But it is explained by the code comment that named the knob and its direction, not by deep physical insight. All the evidence comes from 4 problems on one robot codebase, so treat this as a case study, not a benchmark.

**Can**
- Given good failure telemetry, it reads an unfamiliar ~3 k-line robotics codebase read-only.
  - The telemetry it had: frozen per-step plate states, bottom offsets, F/T value at stop and distance to the pad.
- It localises the cause and writes a small, correct, minimal patch first time.
  - It did this in **3/3 problems where the fix is a parameter or a local signal-processing change** and the code or logs point at it.
- Its diagnoses were quantitatively right:
  - "lowest rim at +6.8 mm → aim 8 mm";
  - "stops at 3.1 N with the TCP 7–10 mm short and zero conductive force → transient, not contact".
- It was faster than the logged human-like process for P1 (1 call vs ~6 runs, and no wrong-direction tries).
- It never cheated, even with frozen files and functions within reach. Its patches were clean and commented.

**Can't, or not shown**
- **When the documented knob is taken away (P2b), it did not find another mechanism in 8 tries.**
  - It cycled through one hypothesis family (release and contact transients) with ≤ 0.7° effect.
  - One of its patches made things much worse (22°).
  - It did not change strategy after 5 null results, and it never proposed an experiment designed to discriminate between causes. The `test_plan` fields were generic.
- **The successes are easy cases.** One knob, a direction hinted in code comments, and a metric that responds monotonically.
  - These show GPT is a good *code-reading tuner*.
  - They do not show it can do open-ended physical problem solving. The one test of that, P2b, failed.
- **N is tiny.** 3 solved problems, 1 unsolved, 2 P1 replications (both also one-shot: 8 mm and 9 mm). Physics is deterministic per seed, but GPT is not. The replications show only small sampling variance on the easy problem; nobody replicated P2b.
- **It needs the harness.** The frozen scorer, rule check, key frames, guard runs and a separate reviewer are what make its patches trustworthy. Without them, an accepted patch's "success" would rest on in-code predicates that GPT could edit.
- **P3 is a reconstructed revert.** It reproduced only in full missions; insert touchdown false triggers did not reproduce. So it tests one symptom of the original bug.

**Practical recommendation**
- Use GPT as a first-pass engineer on well-instrumented failures. A one-shot patch plus a sim check costs about 1–2 min of GPT and one test round.
- Keep a human (or a stronger search loop) for problems where the fix needs a new mechanism. Have it propose discriminating experiments rather than successive patches.

## 4. Files

| path | content |
|---|---|
| `GPT_ENGINEER.md` | this report |
| `results.json` | per problem: every iteration (status, metric, tie-break, diff size, GPT s, tokens, sim wall s, peak RSS, guard, review) |
| `metric_vs_iteration.png` | pass count vs iteration (P1, P2, P3) + mean final lean vs iteration (P2, P2b) |
| `diffs/` | per iteration: `*_proposed.diff` (GPT's text), `*_applied.diff`, `*_cumulative.diff`; `*_FINAL.diff` per problem |
| `harness/harness.py` | the loop (codex call, patch, rule check, eval, guard, review) |
| `harness/score.py` | frozen scorer + key-frame renderer |
| `harness/eval_mobile.py`, `harness/eval_full.py` | per-case evaluators (frozen case definitions) |
| `harness/aggregate.py`, `harness/audit_codex.py` | results/plot; codex transcript audit |
| `harness/*_statement.txt` | problem statements for P3, P2b, P4 (P1/P2 are inline in `harness.py`) |
| `base/p1_aim`, `base/p2_lean`, `base/p3_ft`, `base/p4_dock` | the broken sandbox sources (copies; originals untouched) |
| `runs/<problem>/it<k>/` | `proposal.json`, `code/` (patched copy), `eval/` (per case JSON + log + frames), `guard/`; `ledger.jsonl`, `final.json` |
| `runs/p2_verify/` | extra P2 verification (10 seeds, baseline pairs, 3 mm perception noise); `independent_check.json` = hand-over re-derivation |
| `harness/verify_p2_independent.py` | hand-over: re-derives every P2 pass/fail from the raw JSONs with its own predicate, and checks frozen-scorer vs in-code lean |
| `harness/post_guard.py` | hand-over: held-out nominal-mission guard for accepted patches that the loop did not guard (uses `run_case`/`runw.sh`) |
| `runs/p1_aim_rep2/`, `runs/p1_aim_rep3/` | P1 replications; `runs/p1_aim_rep3/it1/guard_post/` = post-hoc guard |
| `runs/p3_ft/it1/guard_extra/` | post-hoc extra held-out full mission for P3 |
| `logs/post_guard_*.out` | post-hoc guard logs |
| `logs/codex/` | every codex prompt, schema, output and transcript; `logs/codex_calls.jsonl` (seconds, tokens, backoffs) |
| `logs/peak_rss.tsv` | watchdog peak RSS per sim process |
| `runw.sh` | the RSS watchdog (copied from `../gpt_control/runw.sh`) |

**Run**
- `/opt/homebrew/bin/python3 harness/harness.py <p1_aim|p2_lean|p3_ft|p2b_lean_noaim|p4_dock>`. It is one process; sims are strictly sequential under `runw.sh`.
- Then `/opt/homebrew/bin/python3 harness/aggregate.py`.

**Resources**
- 127 sim processes logged in `logs/peak_rss.tsv`, all sequential. That includes the 2 replications and the 4 post-hoc guard missions run at hand-over.
- Peak RSS: 0.69 GB (706,608 kB, watchdog; 0.82 GB in-process `ru_maxrss`). No watchdog kills (no `WATCHDOG KILL` in any log). The hand-over runs peaked at 0.59 GB.
- 20 codex calls (including 1 smoke test and 5 reviews), 1049 s of GPT, 0.98 M tokens, 0 rate-limit backoffs.
