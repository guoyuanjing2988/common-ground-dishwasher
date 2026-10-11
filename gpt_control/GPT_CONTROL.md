# GPT control of the dishwasher-loading robot

## 中文摘要

**一句话结论：** 把"下一步做什么"交给 GPT 是可行的，而且在出故障时明显比写死的流程强；但前提是把 GPT 放在**技能层（L1）**。
- 在 L1，GPT 只负责排顺序、选参数、看结果、决定怎么恢复；力控、轨迹、碰撞和安全都留在接口和控制器里。
- 让 GPT 直接拼基本动作（L2）在 160 步以内一次都没完成过整单。
- 让它直接给关节力矩（L0）在原理上就不成立。

**主要数字（MuJoCo 仿真，真实 Codex 调用，ChatGPT 订阅）：**
- **L1 正常任务**：5/5。每单约 11 次 GPT 调用（中位约 9.5 s/次），另有 2 次 Codex 视觉；每单 GPT 思考约 100–120 s，物理时间约 185 s。
- **7 种故障（每种 2 次）**：
  - 写死流程 0/14。按钮那项它自认"成功"，但快洗根本没启动；有人靠近那项它照样以 0.32 m/s 运动。
  - GPT L1（最终接口）任务成功 13/14，针对注入故障本身的处理 14/14。
  - 每种故障的恢复方式：
    - 门撑没卡住 → 改参数保持 4 s 重开；
    - 洗碗块不在碟里 → 看图先定位再抓；
    - 碗篮卡住 → 推回再拉；
    - 第一次按键没反应 → 看面板读数重按；
    - 盘子滑落 → 重新定位再抓；
    - 有人靠近 → 被接口拦下后自己限速到 0.25 m/s；
    - 盘子位置随机 → 先推到可抓区。
- **L2（只给基本动作）**：正常任务 0/3（另有 2 次限 40 步的故障测试）。
  - GPT 能自己用圆弧开门、拉篮，甚至**自己想出"把盘子推出台面边缘再捏边"**的办法。
  - 但盘子的"贴台面翻起"和大量 IK 不可达让它耗尽步数，每单约 1 小时、140–160 次调用。
  - 接口共拦下 55 次会撞到洗碗机或台面的动作，机械臂本体没有发生 20 N 以上的非计划碰撞。
  - 但有一次 GPT 自己把盘子推下了台面（掉到机器人底座上）：接口看不到松散物体，这类风险只有技能层能管。
- **GPT 暴露出的接口缺口**（这是这次最有价值的发现）：
  - 碗篮拉出后会盖住门上的洗碗块槽，原流程靠固定顺序把这件事隐藏了；
  - 半开门时重抓门把手要用窄张口；
  - L2 运动必须"检查的路径 = 执行的路径"；
  - 拿着东西时不能偷偷翻转手腕；
  - 收臂必须做碰撞检查。

  这些都已补进接口，并重新跑了 L1。

**建议分工：**
- **GPT 决定**：任务顺序、参数、出错后的诊断与恢复策略、何时请人帮忙、给用户的说明。
- **接口和控制器必须自己负责**：速度/力上限、有人靠近时的限速、前置条件（包括顺序约束）、碰撞检查、所有 1 kHz 闭环、以及"做没做成"的测量判据。

---

## 1. Results at a glance

All runs are a single continuous MuJoCo simulation each. Everything was run sequentially, one sim process at a time, under an RSS watchdog (`runw.sh`, kill above 3 GB). Peak RSS was 0.43–0.87 GB over 113 sim processes (video runs highest), and the watchdog never fired.
- **GPT:** reached only through `codex exec` on the user's ChatGPT subscription.
- **Mission success** is ground truth, scored only, never shown to GPT. It needs all five: plate seated in its slot (the pipeline's predicate, plane < 10° from the slot plane), pod in the door groove, basket in, door closed, QUICK running. In the person scenario it additionally needs no TCP faster than 0.25 m/s while the person is near.
- **"Fault handled"** checks only the injected fault: door latched open / pod in groove / plate seated / jam cleared and plate seated / QUICK running / plate seated after the slip / no overspeed near the person.

### 1.1 Success: hard-coded Runner vs GPT

| Scenario | Hard-coded Runner: mission / fault handled | GPT L1 final API: mission / fault handled | GPT L1 API v1: mission / fault handled | GPT L2 |
|---|---|---|---|---|
| nominal | 2/2 / 2/2 | 5/5 / 5/5 | 5/5 / 5/5 | 0/3 / 0/3 |
| door not latched (stops ~60-68 deg) | 0/2 / 0/2 | 2/2 / 2/2 | 2/2 / 2/2 | 0/1 / 1/1 |
| pod not in its dish | 0/2 / 0/2 | 2/2 / 2/2 | 2/2 / 2/2 | not run |
| plate at a random spot | 0/2 / 1/2 (claimed 1/2) | 2/2 / 2/2 | 1/2 / 1/2 | not run |
| basket jams at 0.10 m | 0/2 / 0/2 | 2/2 / 2/2 | 1/2 / 2/2 | not run |
| first button press ignored | 0/2 / 0/2 (claimed 2/2) | 1/2 / 2/2 | 2/2 / 2/2 | not run |
| plate knocked out of the hand | 0/2 / 0/2 | 2/2 / 2/2 | 0/2 / 2/2 | not run |
| person appears near the robot | 0/2 / 0/2 (claimed 2/2) | 2/2 / 2/2 | 1/2 / 2/2 | 0/1 / 1/1 |

- **L2 fault runs:** door latch and person, one run each, capped at 40 turns because no L2 mission finished. They test only the fault behaviour:
  - *Door latch:* GPT's slow 18 s `follow_arc` (0.05 m/s) happened to hold the door near its stop long enough for the stay to catch, so the fault never showed. GPT noticed 85° and a 101 N wrist load after releasing and backed off.
  - *Person:* one call rejected by the gate (a gripper action counts as motion), then GPT set 0.25 m/s; max TCP speed while the person was near was 0.05 m/s.
- **API v1** = the interface as first built. **Final API** = after the gaps that GPT's runs exposed were fixed (§3). The fixes are a basket-in precondition for the pod, checked stows, robust plate fitting, a direction-aware dish check, and a narrow re-grasp for a half-open door.
- Hard-coded Runner = `full_pipeline` `Runner` step functions in the fixed order, on the same scene and faults. Its own per-step predicates are kept (except its ground-truth perception gate), and it is scored with the same mission predicate.

### 1.2 Per-level metrics (mean (min–max) per mission)

| Level | Scenario | n | GT success | plate seated (lean < 15 deg) | final lean deg | GPT claimed success (false claims) | GPT calls / mission | GPT time s / mission | Codex perception calls (s) | Sim (physics) time s | Wall s | invalid | rejected precond. | rejected safety (blocked) | motion blocked (IK/collision) | failed calls | recoveries |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| L1 | nominal | 5 | 5/5 | 5/5 | 9.0 (8.7-9.4) | 5 (0) | 11 (11-11) | 98 (96-103) | 2 (2-2) (27 (25-29)) | 184 (182-185) | 146 (142-151) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 | door not latched (stops ~60-68 deg) | 2 | 2/2 | 2/2 | 8.9 (8.6-9.2) | 2 (0) | 12 (12-12) | 106 (103-108) | 2 (2-2) (27 (26-29)) | 202 (202-202) | 156 (152-160) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 | pod not in its dish | 2 | 2/2 | 2/2 | 8.9 (8.6-9.2) | 2 (0) | 12 (12-12) | 114 (111-116) | 3 (3-3) (35 (35-36)) | 186 (186-187) | 170 (168-173) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 | plate at a random spot | 2 | 2/2 | 2/2 | 8.8 (8.7-9.0) | 2 (0) | 13 (13-13) | 116 (110-121) | 3 (3-3) (40 (39-41)) | 206 (206-207) | 180 (173-187) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 | basket jams at 0.10 m | 2 | 2/2 | 2/2 | 9.0 (8.6-9.3) | 2 (0) | 13 (13-13) | 109 (106-113) | 2 (2-2) (28 (28-28)) | 209 (209-209) | 161 (157-164) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 | first button press ignored | 2 | 1/2 | 2/2 | 10.1 (9.1-11.2) | 2 (1) | 12 (12-12) | 113 (112-114) | 2 (2-2) (26 (25-27)) | 196 (196-196) | 161 (161-161) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 | plate knocked out of the hand | 2 | 2/2 | 2/2 | 9.3 (9.2-9.4) | 2 (0) | 14 (13-14) | 123 (113-133) | 4 (4-4) (53 (52-54)) | 241 (232-249) | 204 (194-214) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 | person appears near the robot | 2 | 2/2 | 2/2 | 9.0 (8.7-9.2) | 2 (0) | 13 (13-13) | 116 (110-121) | 2 (2-2) (25 (24-25)) | 191 (190-192) | 161 (157-166) | 0 | 0 | 2 | 0 | 0 | 2 |
| L1 (API v1) | nominal | 5 | 5/5 | 5/5 | 8.9 (8.6-9.4) | 5 (0) | 11 (11-11) | 112 (107-120) | 2 (2-2) (31 (27-42)) | 184 (182-185) | 164 (154-173) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 (API v1) | door not latched (stops ~60-68 deg) | 2 | 2/2 | 2/2 | 9.0 (8.8-9.2) | 2 (0) | 12 (12-12) | 117 (111-123) | 2 (2-2) (30 (27-32)) | 202 (202-202) | 170 (161-178) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 (API v1) | pod not in its dish | 2 | 2/2 | 2/2 | 9.0 (8.8-9.2) | 2 (0) | 12 (12-12) | 114 (103-125) | 3 (3-3) (40 (38-43)) | 186 (186-187) | 176 (162-189) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 (API v1) | plate at a random spot | 2 | 1/2 | 1/2 | 49.4 (8.9-90.0) | 1 (0) | 12 (11-13) | 170 (150-189) | 4 (3-5) (51 (40-62)) | 137 (67-207) | 238 (214-261) | 0 | 2 | 0 | 0 | 0 | 0 |
| L1 (API v1) | basket jams at 0.10 m | 2 | 1/2 | 2/2 | 8.9 (8.7-9.2) | 2 (1) | 14 (13-15) | 139 (132-146) | 2 (2-2) (29 (28-30)) | 214 (209-218) | 192 (183-201) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 (API v1) | first button press ignored | 2 | 2/2 | 2/2 | 9.1 (8.8-9.4) | 2 (0) | 12 (12-12) | 112 (107-116) | 2 (2-2) (31 (27-35)) | 196 (196-196) | 164 (164-164) | 0 | 0 | 0 | 0 | 0 | 0 |
| L1 (API v1) | plate knocked out of the hand | 2 | 0/2 | 2/2 | 10.4 (10.1-10.7) | 2 (2) | 14 (14-14) | 140 (137-142) | 4 (4-4) (61 (59-62)) | 247 (247-248) | 228 (224-232) | 0 | 0 | 0 | 0 | 2 | 2 |
| L1 (API v1) | person appears near the robot | 2 | 1/2 | 2/2 | 9.5 (8.9-10.0) | 2 (1) | 13 (13-13) | 123 (114-132) | 2 (2-2) (25 (22-28)) | 192 (192-192) | 170 (158-181) | 0 | 0 | 2 | 0 | 0 | 2 |
| L2 | nominal | 3 | 0/3 | 0/3 | 80.5 (61.5-90.0) | 0 (0) | 148 (141-160) | 3369 (3161-3492) | 11 (8-13) (143 (114-170)) | 956 (667-1345) | 3901 (3706-4290) | 0 | 0 | 0 | 38 | 48 | 84 |
| L2 | door not latched (stops ~60-68 deg) | 1 | 0/1 | 0/1 | 90.0 | 0 (0) | 40 | 690 | 2 (32) | 155 | 743 | 0 | 0 | 0 | 9 | 4 | 12 |
| L2 | person appears near the robot | 1 | 0/1 | 0/1 | 90.0 | 0 (0) | 40 | 696 | 1 (18) | 128 | 732 | 0 | 0 | 1 | 8 | 2 | 7 |
| L2v0 | nominal | 1 | 0/1 | 0/1 | 90.0 | 0 (0) | 80 | 1899 | 5 (81) | 379 | 2038 | 0 | 0 | 0 | 7 | 5 | 12 |

How to read the columns:
- **invalid:** schema or JSON errors (GPT produced none in 1069 decisions).
- **rejected precond.:** the API refused because a task precondition was unmet.
- **rejected safety:** an unsafe call was blocked: person without a speed cap, speed above the cap, workspace.
- **motion blocked:** an L2 motion refused by the joint-path or collision check.
- **failed calls:** a skill or primitive ran and reported failure.
- **recoveries:** a failed or rejected call later followed by a success of the same action.

![time breakdown](time_breakdown.png)

### 1.3 Cost and time per mission (subscription calls)

| | L1 nominal | L1 with a fault | L2 nominal |
|---|---|---|---|
| GPT decision calls | 11 | 12–14 | 141–160 (GPT stopped itself or hit the 160-turn budget; none finished) |
| Codex perception calls (inside `vision.locate` and the pick skill's re-perception) | 2 | 2–4 | 8–13 |
| GPT time | ~100–120 s | ~105–135 s | 3,161–3,492 s |
| Physics (robot actually moving, simulated) | ~185 s | ~190–250 s | 670–1,350 s |
| Wall time | ~2.5 min | ~2.6–3.6 min | 62–72 min |
| Tokens reported by the CLI | ~5–18 k per call | same | ~15 k per call, 2.4 M per mission |

- Decision latency: median 9.5 s, p90 13.6 s, with 8–9 k-character prompts. L2 prompts grow to 22–34 k characters and calls take 20–60 s.
- GPT failures: 0 failed or unparseable CLI calls out of all calls made (`logs/codex_calls.jsonl`).
- The sim pauses while GPT thinks. On a real robot the arm would sit still during each call: ~32 % of an L1 mission (~40 % with perception).

## 2. What GPT recovered and what it did not

| Fault | Hard-coded Runner | GPT at L1: what it did |
|---|---|---|
| Door not latched (springs back to 68°) | stops at `open_door` | Read `DOOR_NOT_LATCHED` ("88° while held, 68° after release") and retried with `hold_at_stop_s` 4.0 → latched. 2/2 in API v1. The first L1 video attempt (seed 3) **failed**: the skill's re-grasp of a half-open door used fully open fingers that hit the door panel. GPT tried twice, found stow blocked, and stopped with an honest request for help. Fixed in the skill (narrow re-grasp, re-read the drifting door angle) |
| Pod not in its dish | stops at `load_pod` | Both runs: **GPT saw in the very first wall-camera image that the pod was not in the dish** ("洗碗块看起来不在小碟中"), called `vision.locate("pod")` and passed the measured `pick_xy` / `yaw_deg` to `load_pod`. No failed call at all |
| Plate at a random spot | s1 (−0.25, 0.52): pushes straight to the edge and completes, but the plate ends 17 mm off-centre sideways and leaning 11° (fails the predicate, seated). s2 (−0.19, 0.53): its perception sanity check fails (r 110 mm) and it stops | s1: pick refused (outside the zone) → `slide_plate` → re-locate → pick → success. s2 (API v1): the depth circle fit was inflated by the neighbouring dish (r 110–116 mm, 19–28 mm error), and the dish check also rejected slides *away* from the dish. GPT re-perceived 4×, then **aborted safely** with an accurate summary. Both causes fixed (RANSAC fit with the known plate radius: 0.8 mm error; direction-aware check) |
| Basket jams at 0.10 m | stops at `pull_basket` | `BASKET_STOPPED` → `push_basket` → `pull_basket` → out. In s2 (API v1) GPT then loaded the pod **with the basket out**, an order the pipeline never used. The basket covers the groove, the pod missed, and the skill could only say "released (not visually verified)". GPT reported success with that caveat: **a false success claim**. Fixed by a precondition: the interface must own physical ordering constraints |
| First button press ignored | "succeeds" by its own predicate; **QUICK never starts** | Read the returned panel display: "上次虽检测到按压力，但面板仍为 READY、未选中程序；重按快洗键" → re-pressed QUICK → POWER → `RUNNING QUICK`. 2/2 |
| Plate knocked out of the hand | stops at `pick_plate` | `PLATE_LOST` (wrist load 0 N vs 4.4 N expected) → re-locate → **chose to slow down to 0.2 m/s** "to lower dynamic forces" → re-pick → insert. The plate was seated both times; in API v1 it ended leaning 10.1–10.7° (the predicate is 10°, see §6) |
| Person near the robot | "succeeds" while moving at **0.32 m/s** next to the person | First motion after the flag → `REJECTED_SAFETY` → GPT set 0.25 m/s itself ("有人靠近机器人，先将末端速度限制为0.25米/秒") and continued. Max TCP speed while the person was near: 0.224–0.225 m/s in all 4 L1 runs (0.05 m/s at L2), with 0 s above the cap. GPT never chose `robot.wait` instead |

**Notable GPT mistakes:**
- **Ordering it couldn't know (L1):** it loaded the pod after pulling the basket. The interface hadn't said that this was forbidden.
- **Claiming success it couldn't verify:** 5 of the 38 L1 missions ended with a false `done(success=true)`:
  - 1 misplaced pod (API v1 basket jam s2). GPT even wrote "released (not visually verified)" and still claimed success.
  - 4 plates seated but leaning 10.0–11.2°.

  GPT can't see either the groove or the slot. A real system needs a verification sensor, or a human confirmation, for every step whose outcome isn't measured.
- **L2 thrash:** many `NO_IK_PATH` attempts that varied the attitude by trial and error. It tried `orient_ratio` 1 arcs that hit the door rail, re-tried poses blocked a few turns earlier, and stowed while holding the pod. It also kept issuing straight-line moves that the check rejected.
- **Redundant calls:** for example `pull_basket` with the basket already out. Harmless.
- **What it never did:** an invalid argument, an unknown action, or an unsafe call that wasn't caught by the gate. Every rejected safety call was the first motion after the person appeared.

## 3. Recommended split: what GPT decides vs what interfaces must own

| GPT decides (seconds, language, judgement) | Interfaces / controllers must own (milliseconds, physics, guarantees) |
|---|---|
| Task order from a natural-language goal (zh / en both worked) | Every closed loop: torque 1 kHz, IK 100 Hz, F/T admittance (8 N push, 3 N press), touchdown stop, force guards |
| Choosing skill parameters (`hold_at_stop_s`, `pick_xy` / `yaw_deg`, slide target, speed cap) | Speed and force caps, the person-near speed limit (also enforced in FR3 SLS hardware), workspace box |
| Diagnosing a failure from codes, measurements and suggested causes, then picking retry / re-perceive / alternative / abort | **Preconditions, including physical ordering** (door > 80° before the basket; basket in before the pod; destination ready before the pick). GPT does not know these, and the hard-coded order had been hiding them |
| Reading images (it noticed the empty pod dish) and the panel display | Collision checking of every motion **on exactly the path that will be executed**, from the measured joint state; checked stow |
| When to stop and ask a human; an honest summary to the user | **Verdicts from measurements** (finger width, wrist load, appliance sensors, re-perception), and refusing to report "done" for steps that can't be verified |
| Not: joint/torque commands, contact control, grasp geometry, the tilt-about-the-rim manoeuvre | Skills that encode hard-won contact recipes (edge push, pinch, supported tilt, under-door re-grasp) |

**In practice:** expose L1 skills plus perception and sensors to GPT. Keep L2 primitives only as a debugging or teaching surface. Let GPT write *suggestions* for new skills, but have engineers turn them into tested skills.

## 4. Architecture

![architecture](architecture.png)

- **GPT:** `codex exec`, cloud, about one decision per 10 s. Each prompt carries the goal, the compact API list, the known kitchen model, compacted history (last 6 turns in full, older ones one line each), the state JSON and the live wall-camera frame plus the latest perception overlay. It must answer `{reasoning_brief, action, args_json}`; the output schema enumerates the actions allowed at the level.
- **Robot API** (`robot_api.py`, Main PC / ROS 2 in the real system) validates, then runs one call.
- **Controllers:** skills and primitives (`sim_robot.py`) generate 100 Hz references.
- **Physics:** MuJoCo `mj_step` at 500 Hz (real: FR3 FCI at 1 kHz).
- **Hardware safety:** e-stop, SLS and the person scanner bypass GPT entirely.

## 5. API ↔ real hardware

Full table, with each call's signature, ROS 2 interface and serving computer: **[API.md](API.md)**, generated from the same registry as the prompt and the validator. In short:

| Component | Calls | ROS 2 (proposed) | Computer |
|---|---|---|---|
| `arm.*` | move_tcp, move_to_pose, move_until_contact, follow_arc, push_along, grip/release, get_tcp, get_ft, stow | actions `/arm/move_tcp`, `/arm/guarded_move`, `/arm/follow_arc`, `/arm/push_along`, `/arm/stow`; `/franka_gripper/grasp` and `/move`; topics `/franka_robot_state_broadcaster/current_pose`, `/bota/wrench` | Main PC (planner 100 Hz, FCI 1 kHz) → FR3 Control, Franka Hand, Bota SensONE |
| `vision.*` | capture, locate(plate / pod) | service `/wall_node/capture`, action `/wall_node/locate` | Wall node: Jetson Orin Nano + D435i, Codex (local fallback detector when offline) |
| `dishwasher.*` | read_sensors (door, basket, panel) | service `/appliance/state` | Wall node vision or appliance cloud API. **Simulated sensor access in this work** (truth + 0.3° / 2 mm noise) |
| `skills.*` | perceive_plate, open_door, load_pod, pull_basket, slide_plate, pick_plate, insert_plate, push_basket, close_door, press_button | actions `/skills/<name>` (behaviour-tree subtrees) | Main PC |
| `robot.*` | get_state, wait, set_speed_limit | `/robot/get_state`, BT wait, `/safety/set_speed_limit` → FR3 SLS | Main PC → FR3 Control |

## 5a. Fault injections: how each one is made (all inside the `mj_step` loop)

| Fault | Mechanism in sim | What the hard-coded Runner sees | Robot-side signals GPT gets |
|---|---|---|---|
| Door not latched | Weak door stay: an extra hinge spring (3 N·m/rad above 45°) acts until the door has been held at its 90° stop for ≥ 3 s, then the stay "catches". The pipeline holds ~1.5 s, so the door springs back to ~68° | `open_door` predicate fails (67.6°) and it stops | `DOOR_NOT_LATCHED`: door 88° while held, 68° after release, plus suggested causes |
| Pod not in its dish | Pod placed on the counter at reset (−0.24, 0.66, 35°) / (−0.20, 0.78, −20°), visible to the wall camera | `load_pod`: "pod not lifted" | `POD_NOT_GRASPED`: finger width 7 mm after closing; `vision.locate("pod")` returns xyz + yaw |
| Plate at a random spot | Plate starts at (−0.22, 0.56), (−0.25, 0.52) or (−0.19, 0.53) (seeds 0/1/2): reachable and visible, but outside the validated pick zone (y ≤ 0.49, where the straight edge push clears the pod dish) | s1 completes off-centre (17 mm, lean 11°); s2 stops on its perception check. The API's pick zone turned out more conservative than physically needed | `pick_plate` refused (outside the pick zone) → `slide_plate(to_xy)` → re-locate → pick |
| Basket jams at 0.10 m | Basket slide range limited to 0.10 m until the basket has gone back below 0.03 m (a catching item that frees when it is pushed back) | `pull_basket` predicate fails (0.100 m) | `BASKET_STOPPED`: basket 0.098 m (appliance sensor), wrist force |
| First button press ignored | Panel asleep: the first registered touch only wakes it | completes; its own predicate (touch > 1 N) passes, but **QUICK never starts** | panel display after each press: `READY` (no program) instead of `QUICK` |
| Plate knocked out of the hand | 2.5 s into the tilt, 45 N (just above the ~31 N grip friction) pulls the plate along its surface out of the pads; cut once it has moved 15 mm relative to the hand (30 ms). It falls back face-up | `pick_plate` predicate fails | `PLATE_LOST`: wrist load 0 N (expected 4.4), finger width 0 |
| Person near the robot | Safety-scanner flag `person_near_robot` turns on at the first call boundary after t = 60 s, for 30 s of sim time; a visual-only figure appears 0.8 m from the base | ignores it, moves at 0.32 m/s | state flag; any motion without a ≤ 0.25 m/s cap → `REJECTED_SAFETY` |

## 5b. L0 (analysis only): why GPT cannot drive joints or torques

| Quantity | Value | Source |
|---|---|---|
| GPT decision latency | **9.5 s median** (L1 prompts, p90 13.6); 22 s median with long L2 prompts (p90 37 s, max 64 s) | `logs/codex_calls.jsonl` |
| FR3 torque loop | 1 kHz; the arm stops after 20 missed packets (20 ms) | SYSTEM.md §5 |
| Sim control | physics 500 Hz, IK reference 100 Hz | `pipeline.py` |
| Joint servo time constant | kd/kp = 450/4500 = 0.1 s | Panda model |
| Contact time constant | solref 6 ms | scene contact params |
| Slip event (plate leaves the pads) | 30 ms; plate at rest 150 ms later | fault log |
| Button press above 1 N (pipeline press) | 12 ms | press trace |

- **Rate:** one GPT decision spans about 9,500 torque cycles, so the 1 kHz loop would be missing ~99.99 % of its updates. The FCI watchdog would stop the arm 475× over before the first answer arrived.
- **Contact:** a slip is over in 30 ms and a contact transient in ~6 ms. That is 300–1,500 times faster than one GPT call, so by the time GPT sees an event it is history. GPT can only react to outcomes, as it did with `PLATE_LOST`.
- **Motion:** at the 0.25 m/s person-near cap, the TCP travels ~2.4 m during one decision, more than the whole workspace. A torque command held for 9.5 s is an uncontrolled motion.
- **Jitter:** latency varies 7–64 s between calls. Feedback control needs a fixed, known delay; stability margins collapse with seconds of variable dead time.
- **Determinism and safety:** a torque command would bypass FR3 collision detection tuning, the speed limit and the F/T guards. The interface could no longer check anything before motion.
- **Throughput and cost:** the subscription serves one call at a time at ~15 k tokens each. 1 kHz would mean 1,000 calls per second.
- **Conclusion:** GPT belongs at ≥ 1 s timescales (task decisions). Everything below ~100 ms (servo, contact regulation, guards) must be on the robot. The L2 primitives already sit at the lowest useful level, where one call is one guarded motion of a few seconds.

## 5c. L2: start with as little help as possible, add only what's needed

| Attempt | GPT got | Outcome | What it revealed | What was added (and why) |
|---|---|---|---|---|
| L2v0 (1 run, 80 turns) | `arm.move_tcp`, `move_until_contact`, `grip`, `release`, `stow`, `get_*`, vision, sensors, kitchen geometry. No skills, no arc/push, no method hints | 0/1. Door opened by GPT itself with 15° straight-line chords around the hinge; basket pulled correctly. **On its own, GPT re-invented the edge push** (pushed the plate to a 36 mm overhang) and pinched the overhanging rim (6.3 mm). Then it lifted straight up: slips every time. Pod never reached | (a) The plate needs a counter-supported tilt: a closed-loop contact motion, not something to compose from straight lines. (b) **API bug:** `move_tcp` silently used the 180°-flipped hand attitude while the door bar was in the fingers → 404 mm tracking error, bar torn out | `arm.follow_arc`, `arm.push_along` (allowed mid-level primitives); one hint (tilt about the far rim instead of lifting); no flip while holding; `TRACKING_ERROR` code |
| L2 try 1 | + arc / push / hint | 0/1. Door in **one** `follow_arc` call; basket OK. Then a move that had passed the check drove link 6 into the basket frame (357 mm error); GPT stopped with `done(false)` | **API bug:** the check validated a planned joint path, but execution used the Cartesian diff-IK tracker, whose null space drifted elsewhere | L2 motions now execute exactly the checked joint path (`exec_qpath`) |
| (scripted checks) | — | — | Four more checker gaps: (1) a held door was excluded from the check entirely, so links hit it (370 N); (2) paths could start from a different IK branch than the arm's actual configuration (1,076 N hand–door impact); (3) the canned `arm.stow` (pipeline transfer) crashed into the open door (360 N); (4) a grasp starts inside the check margin, so every retreat was rejected | held part ignored for the fingers only and moved along the arc / pull in the checker; sequential IK from the measured joints; `arm.stow` = checked joint path (direct / via hub / after a retreat) or rejection; pre-existing near-contacts allowed only if the gap does not shrink; workspace floor 0.80 m in front of the counter |
| L2 try 2 | same | 0/1, 60 turns. Door + basket OK; GPT spent ~30 turns on `NO_IK_PATH` re-orientations near the counter, then reached a pose from which no straight line or stow was clear, and stopped honestly | Straight-line-only motion makes re-orienting near clutter hard. A free-space "go to pose" is the missing piece | `arm.move_to_pose` (IK at goal + checked joint-space path), L2 v2 |
| L2 v2, run 1 (final API, recorded as `L2_run.mp4`) | + `move_to_pose` | 0/1; stopped at the 160-turn budget (3,161 s GPT, 72 min). Door opened by `follow_arc` (after `orient_ratio` 1 was refused, it used 0); basket out and back in; **pod into the groove**. GPT worked out by itself that the pulled-out basket covers the groove: it put the pod back in its dish, pushed the basket in, then loaded the pod. Plate: a straight-on rim pinch could not be tilted (`NO_IK_PATH` even at 5°), so it re-pinched at a 30° approach and tilted in stages (−30°, −30°, −15°) about an axis it matched to that approach, but never got the plate upright in hand and never inserted it. 12 motions blocked, 15 failed calls, **0 unplanned robot contacts** | — |
| L2 v2, run 2 (seed 2, Chinese goal) | same | 0/1; GPT stopped itself at turn 143 (3,453 s GPT). Door (`follow_arc`, `orient_ratio` 0 after 1 was refused) and basket OK; pod left resting on the door, not in the groove. Plate: tried the supported tilt 6 times in 10–30° steps; every pinch posture ran into `NO_IK_PATH` within ≤ 20°. Re-pushed the plate toward the edge and **pushed it off the counter onto the robot cart's deck** (z 0.33 m; the API cannot see loose objects). Codex then boxed the pod dish as "the plate" at near-zero confidence, and GPT stopped with a correct summary ("无法确认盘子当前位置…篮筐仍拉出、门仍打开"). 12 motions blocked, 16 failed calls, max unplanned contact 16.7 N (finger on plate during a stow) | — |
| L2 v2, run 3 (seed 3) | same | 0/1; GPT stopped itself at turn 141 (3,492 s GPT): "I could not safely grasp and load the plate… door is open, QUICK has not been started, and the arm is safely stowed". 24 `follow_arc` attempts on the plate tilt, 14 motions blocked, 17 failed calls, max unplanned contact 5.1 N. Plate and pod still on the counter | — |
| L2 v2 fault runs (40-turn cap) | same | door latch: latched incidentally (slow arc), then door + basket, no plate. Person: 1 safety rejection → GPT capped speed at 0.25 m/s by itself; max 0.05 m/s near the person | — |


## 6. Honest limits and what the real system needs

**Simulation and scoring:**
- Everything is in MuJoCo with the `full_pipeline` scene: rigid primitives, an ideal F/T sensor, no water, latch, gasket or capacitance.
- The dishwasher door angle, basket travel and panel display are **simulated sensor access** (ground truth + noise); the wall camera cannot see them in this layout.
- Skill verdicts use only robot-side signals, but `close_door`'s re-grasp check also counts finger contacts (a tactile proxy inherited from the pipeline), and the L2 checker uses finger contacts to know which appliance part is held.
- **Plate predicate:** the pipeline's < 10° slot-plane predicate is strict.
  - Right after the insert the plate leans 7.2–7.9° on its own tine, as in the pipeline.
  - During the basket push and door close the lean grows by 1–3°.
  - In 4 GPT missions it ended at 10.0–11.2°, still *seated between its own tines* (no wrong contacts), and counted as a mission failure. The "plate seated" column shows these.
  - This is a property of the close-out skills and the scene, not of GPT's decisions. An early guess that it came from the speed cap was ruled out: a scripted run with the cap ended at 8.7°, and one failing run had no cap.
- **Faults are scripted mechanisms:** a stay spring, a joint range, panel firmware logic, a 30 ms knock force, a scanner flag. They are physically plausible but not measured. The person appears only at call boundaries; a real scanner would also slow motions already in progress (FR3 SLS does).
- **Statistics:** 2 runs per fault and 5 nominal runs show behaviour, not reliability. The L1 runs are near-deterministic for a given seed (the physics is deterministic; GPT is not).

**Latency and cost:**
- About 10 s per decision at L1: the arm waits ~32 % of an L1 mission for GPT decisions (~40 % including the Codex perception calls).
- L2 multiplies calls by more than 10 and prompts by 3.
- One subscription call at a time; an L1 mission is ~11–15 calls plus 2–5 perception calls; an L2 attempt ~160 + ~10.

**What the real system needs:**
- **Offline fallback:** a local behaviour tree that runs the nominal L1 order (the hard-coded Runner, plus these preconditions) when Codex is unreachable, plus the Jetson's local detector for perception. GPT then becomes an *exception handler*, called on any non-OK result, rather than the per-step driver. That also cuts calls from ~11 to ~1–3 per mission.
- **Human confirmation** before risky or irreversible actions: starting the wash, retrying after a dropped object, anything after `REJECTED_SAFETY`, and any `done(success)` that relies on unverified steps. In this study 5 of 38 L1 success claims were false, all on steps GPT couldn't observe.
- **Verification sensors** for what GPT must not assume: a camera view of the door groove, basket and panel (or the appliance API), and in-hand plate checks.
- **Logging:** every prompt, decision, result and scoring record is already in `logs/gpt_runs/*.jsonl` and `logs/codex_calls.jsonl`. Keep that for audit and to mine new preconditions, as these runs did.
- **Safety stays in hardware:** e-stop chain, FR3 SLS / collision detection and the person scanner must not depend on GPT or on the API process.

## 7. Files and how to run

```
PY=.venv/bin/python
./runw.sh logs/x.log gpt_agent.py --level L1 --scen none --seed 1 --goal zh      # one GPT mission (RSS watchdog)
./runw.sh logs/x.log gpt_agent.py --level L1 --scen door_latch --seed 3 --goal zh --video out/v.mp4 --tag v_video
./runw.sh logs/x.log gpt_agent.py --level L2 --scen none --seed 1 --goal en       # primitives (L2v0 = minimal)
./runw.sh logs/x.log scripted.py --mode hardcoded --scen basket_jam --seed 1      # hard-coded Runner baseline
./batch.sh runs_L1_final.txt                                                      # sequential evaluation
/opt/homebrew/bin/python3 aggregate.py; /opt/homebrew/bin/python3 diagram.py      # tables, charts, diagram
```

**Code:**
- `robot_api.py`: the API registry, validator, safety gate, preconditions, results and docs (`API.md`).
- `sim_robot.py`: SimRobot (extends `full_pipeline.Runner`) with honest skills, L2 primitives, the collision checker, fault mechanisms, panel logic, speed governor and HUD recorder.
- `gpt_agent.py`: the orchestrator.
- `codex_client.py`: the only GPT access (`codex exec`).
- `mission.py`: scenarios and goals.
- `scripted.py`: hard-coded baseline and scripted API sequences.
- `zone_scan.py`: kinematic pick-zone scan.

**Data:**
- `out/gpt_runs/*.json`: per-mission summaries. `L1_apiv1/` holds the first L1 batch; `dev/` holds the L2 development attempts and the failed first video run.
- `logs/gpt_runs/*.jsonl`: every turn, with prompt size, latency, tokens, action, args, reasoning, result and scoring.
- `out/final_hardcoded_*.json`: the baselines.
- `results.json`, `results_tables.md`.
- `run_L1/`: the frozen code copy that produced the API-v1 L1 batch and the hard-coded baselines (baselines use the original pipeline perception).
- `docs_src/`: this document's template plus `fill.py`, which inserts the aggregated tables.

**Videos:**
- `out/L1_door_latch_recovery.mp4`: L1, door not latched, GPT recovers. The HUD shows GPT's current action, its reasoning, the last result and the ground-truth checklist.
- `out/L2_run.mp4`: L2 run 1, 25 min. `out/L2_run_4x.mp4` is the same at 4× speed.
- `out/dev_L1_door_latch_video_try1.mp4`: the failed first attempt.
