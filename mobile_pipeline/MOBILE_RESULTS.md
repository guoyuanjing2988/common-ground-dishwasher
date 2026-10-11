# Mobile dishwasher-loading robot: wheels, battery, multi-controller architecture (2026-10-11)

## 中文摘要

- **整体能跑通。** 机器人从充电桩出发，靠激光雷达定位，开到柜子前倒车进 V 形块，压紧约 100 N 后抱闸。
  - 装盘 8 步 + 面板 3 次触摸（POWER → QUICK → POWER）都做完后收臂、松闸，开回充电桩并对上充电触点。
  - 真 Codex 视觉的完整任务 **6/6 成功**（5 次随机试验 + 1 次录像）。
- **定位与停靠。** 停靠后机械臂基座相对设计位姿的误差是 x −1.0…−1.6 mm、y ±0.8 mm、yaw ≤ 0.09°。
  - 臂用的是**实测**基座位姿：雷达拟合柜门线 + 反光条，误差 ≤ 0.4 mm / 1.2 mm / 0.08°。
  - 抱闸干活期间底盘蠕动 ≤ 0.9 mm / 0.04°。
- **容差。**
  - 按 spec 要求的 ±2/±5 mm、±0.2/±0.5°（**不补偿**，机器人以为在名义位置）：**12/12 通过**。
  - 用户要求的 ±2/±4 cm、±3/±6°（臂用实测位姿补偿）：**10/11 通过**。唯一失败是 y −4 cm：反光条出了后雷达视野，y 退回名义值，偏差 40 mm，抓不到洗涤块。
- **电。**
  - 每趟任务（充电桩 → 装盘 → 回桩，约 270 s）耗电 **9.9–10.2 Wh**，SoC 降 0.58–0.60 %，峰值电流 4.2 A。
  - 其中臂约 57 %，常开负载约 41 %，轮子不到 0.5 %。
  - 一次充满可跑约 **120 趟**；把 spec 里 90 s 唤醒（2.7 Wh，本仿真未跑）算进去约 **95 趟**。
  - 低电量（SoC 22 % < 30 %）时直接去充电，到阈值约需 15.5 min。
- **急停。** 端盘途中拍急停：臂 0.25 s 停住，TCP 多走 2.4 mm，盘在手里只滑了 0.04 mm，门没动（90.03°）。松开并确认后接着做完，任务成功。
- **面板。**
  - 只有导电触头（≥ 0.5 N 且 ≥ 80 ms）才算按下。普通手指压 0.4–0.5 s、33–38 N 也**不响应**，重试 2 次后安全中止并回桩。
  - 门开着按 START 会被拒（3 声提示）。
- **改成 spec v1.0 以后，有三处要改代码才能过。**
  - FR3 关节限位 + 末端 F/T 传感器：插盘落点偏了约 4 mm，插盘对准偏置从 3 mm 改为 8 mm 才稳。
  - 底盘带悬挂，腕部 F/T 有 ±2 N 振动噪声，触底/按键判定都要加滤波。
  - 自由底盘的轮胎摩擦是软约束，会慢慢蠕动，开了 `noslip`。
- **硬急停（Cat 0）。** 0.5 m/s 下 51 mm 刹停，后两个万向轮短暂卸载到 0 N，抬起 ≤ 0.11 mm，没有倾翻。
  - 先电机减速再抱闸（Cat 1，1.5 m/s²）的话不会卸载，刹停距离 91 mm。

## Headline tables

### Full missions (real Codex; charger → dock → 9 steps → charger)
Plate pose randomised ±10 mm / ±10°; start pose randomised ±5 cm / ±5° around a point 8 cm in front of the charging contacts; seed 20261011.

| run | outcome | all 9 steps | sim s | Codex s | dock err x / y / yaw (mm, mm, °) | base-estimate err (mm, mm, °) | creep during task mm / ° | min wheel / caster N | CoP margin m | brake torque max N m | SoC drop % | energy Wh | I peak A | charger pose err x / y / yaw (mm, mm, °) | peak RSS MB |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| mission_trial0 | DONE | yes | 269.0 | 16.3, 14.4 | −1.4, 0.0, −0.001 | −0.29, 0.19, 0.057 | 0.86 / 0.029 | 349 / 0 | 0.153 | 14.1 | 0.589 | 10.04 | 3.8 | −0.3, 3.8, −0.7 | 600 |
| mission_trial1 | DONE | yes | 272.3 | 14.4, 19.9 | −1.6, −0.26, −0.020 | −0.41, 0.25, 0.077 | 0.85 / 0.042 | 349 / 0 | 0.148 | 15.8 | 0.595 | 10.15 | 4.0 | 0.0, 3.8, −0.8 | 691 |
| mission_trial2 | DONE | yes | 273.4 | 14.5, 21.1 | −1.2, 0.63, 0.042 | −0.13, −1.23, 0.022 | 0.88 / 0.021 | 350 / 0 | 0.153 | 13.8 | 0.599 | 10.19 | 4.2 | −0.4, 3.3, −1.0 | 799 |
| mission_trial3 | DONE | yes | 272.2 | 20.1, 14.1 | −1.0, 0.34, 0.014 | −0.25, −1.15, 0.046 | 0.90 / 0.019 | 350 / 0 | 0.153 | 13.2 | 0.596 | 10.15 | 3.9 | 0.1, 3.2, −1.2 | 906 |
| mission_trial4 | DONE | yes | 265.5 | 14.1, 12.8 | −1.1, 0.75, 0.083 | −0.19, −0.59, 0.076 | 0.90 / 0.032 | 350 / 0 | 0.153 | 13.5 | 0.581 | 9.92 | 3.8 | −0.4, 3.6, −1.0 | 1016 * |
| mission_video | DONE | yes | 266.0 | 15.1, 12.7 | −1.2, 0.44, 0.012 | −0.32, −0.71, 0.039 | 0.89 / 0.017 | 350 / 0 | 0.153 | 13.9 | 0.582 | 9.93 | 4.0 | −0.3, 3.1, −1.3 | 787 |
| **success** | **6/6** | **6/6** | | | | | | | | | | | | | |

**Notes on the table**
- **Dock error** is the arm base (link0) vs the M1 design pose (0.48, 0.42, yaw 195°), from ground truth.
  - The −1.2 mm bias in x: the brake catches the cart after the preload has overshot to 130–150 N, against the 100 N the V-block seat is designed for.
- **Base-estimate error** is the lidar-measured pose that the arm IK uses, minus truth.
- **Energy** is the battery-side load energy of the whole mission.
- **Peak RSS:** all 5 trials ran inside one process, so RSS grew about 100 MB per trial (*). Single missions peak at 590–790 MB.
- **Perception:** centre error 0.69 mm at the start and 0.75 mm at the counter edge in the video run. Insert landing (bottom offset from the slot centre): +0.7 to +2.0 mm; the plate leans 2.2–3.5°.

### Per step with the FR3 joint limits + Bota F/T (6 real-Codex missions; minimum over runs, peak over runs)

| state / skill | ok | mean s | peak TCP m/s | peak elbow m/s | min (q6 − q6_min) ° | min (q4_max − q4) ° | min margin, any joint ° (joint) | peak joint torque / limit |
|---|---|---|---|---|---|---|---|---|
| arm.unstow | 6/6 | 2.3 | 0.23 | 0.09 | 82.8 | 138.1 | 21.6 (j4) | 0.30 |
| vision.locate (perceive) | 6/6 | 18.4 | 0.11 | 0.10 | 79.1 | 138.1 | 26.7 (j4) | 0.34 |
| skills.open_door | 6/6 | 22.0 | 0.30 | 0.30 | 45.3 | 94.8 | 10.6 (j2) | 0.68 |
| skills.load_pod | 6/6 | 24.0 | 0.30 | 0.36 | 55.8 | 82.1 | 35.9 (j4) | 0.60 |
| skills.pull_basket | 6/6 | 14.8 | 0.30 | 0.18 | 81.8 | 82.1 | 18.6 (j2) | 0.63 |
| skills.pick_plate | 6/6 | 54.0 | 0.30 | 0.32 | 44.4 | 68.0 | **3.5 (j4)** | 0.71 |
| skills.insert_plate | 6/6 | 21.4 | 0.16 | 0.20 | **31.0** | **19.1** | 10.5 (j7) | 0.76 |
| skills.push_basket | 6/6 | 15.6 | 0.29 | 0.31 | 71.8 | 121.8 | 17.7 (j7) | 0.52 |
| skills.close_door | 6/6 | 19.7 | 0.30 | 0.34 | 51.5 | 91.4 | 5.7 (j2) | **0.89** |
| skills.press(power) → STANDBY | 6/6 | 10.7 | 0.25 | 0.16 | 56.0 | 131.1 | 23.6 (j4) | 0.38 |
| skills.press(quick) → SELECTED | 6/6 | 8.4 | 0.25 | 0.25 | 86.0 | 113.4 | 24.8 (j4) | 0.49 |
| skills.press(power) → RUNNING | 6/6 | 8.5 | 0.25 | 0.23 | 86.0 | 117.9 | 25.3 (j4) | 0.43 |
| arm.stow | 6/6 | 7.1 | 0.30 | 0.16 | 68.6 | 138.6 | 13.4 (j4) | 0.42 |

**FR3 limits**
- The smallest q6 margin is 31° (insert) and the smallest q4 margin is 19° (insert). No path needed re-planning for the FR3 limits; the IK and joint-path planner already plan inside them.
- The tightest joint overall is j4 at **3.5°** from its limit, during the pick tilt (near-straight elbow). It is safe, but it is the first thing to watch on a real FR3.

**TCP speed cap (0.25 m/s, spec SLS)**
- The unmodified planner peaks at 0.30 m/s (transfers).
- With `--tcp-cap 0.25` the mission still succeeds. Peak TCP is 0.257 m/s (quintic/IK overshoot ≤ 3 %) and arm-task time goes from 226.9 to 231.6 s (**+4.7 s, +2 %**). Both runs used truth perception (labelled DEBUG).

### Dock-offset tolerance (arm uses the MEASURED base pose; truth perception, labelled DEBUG)
Each case starts already at the offset dock pose with brakes on, no V-blocks (they would remove the offset), and runs localize → unstow → 8 steps → panel → stow.

| offset (dx, dy, dyaw) | result | first failure | lidar base-estimate err (mm, mm, °) |
|---|---|---|---|
| 0, 0, 0 | PASS | | −0.4, 0.0, 0.10 |
| y +2 cm | PASS | | −0.3, 0.7, 0.06 |
| y −2 cm | PASS | | −0.1, **4.2**, 0.05 |
| y +4 cm | PASS | | −0.2, −0.3, 0.05 |
| y −4 cm | **FAIL** | load_pod: pod not lifted | 0.1, **40.0**, 0.01 |
| x +2 cm | PASS | | −0.3, −0.8, 0.05 |
| x +4 cm | PASS | | −0.2, −1.0, 0.05 |
| yaw +3° (+ x 8.5 mm †) | PASS | | −0.1, −1.7, 0.02 |
| yaw −3° (+ x 8.5 mm †) | PASS | | −0.1, −1.4, 0.00 |
| yaw +6° (+ x 22 mm †) | PASS | | −0.1, −1.0, 0.01 |
| yaw −6° (+ x 22 mm †) | PASS | | −0.1, −0.6, 0.04 |

**Notes on the dock sweep**
- **x offsets** are away from the cabinet only; the cabinet blocks the other direction.
- **† Yaw cases:** a yawed base touches the cabinet doors with one corner first, so the start is shifted out until that corner clears the doors by 0.5 mm.
- **Conclusion:** with measured-pose compensation the task tolerates x +4 cm, yaw ±6°, y −2…+4 cm.
- **Why y −4 cm fails:** the y limit is not reach. The reflective strip leaves the field of view of the rear-RIGHT corner lidar when the base is 4 cm toward −y. The y estimate then stays at nominal (40 mm error) and the pod pick misses. At −2 cm the strip is already partly cut off (4.2 mm error).
- **Fix:** a strip in front of each rear corner, or the lidar on the centreline.

### Spec recheck: base-pose tolerance WITHOUT compensation (robot believes the nominal pose; truth perception)
Perception is re-expressed in the robot's believed frame, emulating the per-dock camera→robot re-calibration. The dishwasher is NOT re-measured.

| perturbation | ±2 mm x | ±5 mm x | ±2 mm y | ±5 mm y | ±0.2° yaw | ±0.5° yaw |
|---|---|---|---|---|---|---|
| result | PASS / PASS | PASS / PASS | PASS / PASS | PASS / PASS | PASS / PASS | PASS / PASS |

**12/12 pass**, so the spec's design target (≤ 1 mm / 0.1°) has margin. Before the insert re-tuning (see "What changed"), −5 mm x and −0.5° yaw failed at the insert.

### E-stop while carrying the plate (real Codex)
**Sequence**
- The e-stop button is pressed 0.5 s into `carry_across` (t = 158.5 s).
- BaseMCU sends EStop to the ArmController (1 ms), MotionPlanner and TaskSupervisor.
- Brakes were already on.
- The ArmController does a Cat-1 stop: it decelerates the commanded motion to zero in 0.2 s, then holds.
- Release follows at +3 s, operator "resume" at +1.5 s; then the ArmController blends for 0.5 s and the planner continues the same trajectory.

| arm stop time | TCP travel after e-stop | plate slip in hand during the stop | plate rotation | finger contacts at stop / at resume | door angle at stop / resume | outcome |
|---|---|---|---|---|---|---|
| 0.25 s | 2.4 mm | 0.04 mm | 0.02° | 14 / 10 | 90.03° / 90.03° | insert seated, mission DONE |

The Franka Hand is assumed to keep its grasp when stopped. That is an OPEN item in the spec: real Hand power-loss behaviour is unverified.

### Dishwasher panel (appliance-MCU emulation)

| test | presses | contact force / dwell | panel result | supervisor |
|---|---|---|---|---|
| normal (stylus), 6 missions | POWER, QUICK, POWER | conductive 22–49 N peak (impact), held 0.5 s at ~3 N F/T | OFF → STANDBY → SELECTED[quick] → RUNNING[quick] | readout verified after each touch |
| plain finger pad (`--nonconductive`, real Codex) | POWER ×3 (2 retries) | non-conductive 33–38 N, 0.41–0.49 s ≥ 0.5 N | stays **OFF**, no event | panel not responding → ABORT: safe stow, undock, drive back, charger |
| door open (`--panel-door-open`, DEBUG) | POWER, QUICK, POWER | conductive | STANDBY, SELECTED, then **start_refused** ×3 (door_closed = false), stays SELECTED | aborted |

Bounce on impact sometimes registers a second touch about 0.3 s later, e.g. `touch_quick` twice. The panel logic ignores it; the 100 ms release debounce is a modelled value.

### Energy / battery

| quantity | value |
|---|---|
| energy per mission, charger → load → charger (battery side) | **9.9–10.2 Wh** over 266–273 s |
| breakdown (video mission) | arm 5.69 Wh (57 %: 45 W control electronics + joint power, through the inverter); fixed 24/48 V loads 4.09 Wh; brake coils released 0.11 Wh; wheels **0.043 Wh** |
| by phase (video mission) | drive out 0.54 Wh / 13.9 s; dock 0.31 Wh / 8.5 s; arm task + panel 8.34 Wh / 224 s (of which 0.95 Wh / 27.8 s waiting for Codex); drive back + charger 0.73 Wh / 19.1 s |
| peak bus power / peak current | 223 W / **4.2 A**. The BMS limit is ≥ 60 A, so current is not a constraint |
| SoC drop per mission | 0.58–0.60 % of 30 Ah |
| cycles per charge (usable 80 % × 1536 Wh = 1229 Wh) | **≈ 120** missions simulated as is; **≈ 95** adding the spec's 90 s wake/FR3-boot phase (2.67 Wh, not simulated). The spec's own estimate is 16.2 Wh / 76 cycles |
| low battery (`--low-battery`, SoC 0.22 < 0.30) | CHECK_BATTERY → LOW_BATTERY → GO_CHARGE (`base.dock_charger`) → CHARGING. Net +9.0 A while awake on the 11.3 A charger, so ≈ **15.5 min** to reach 30 %. The mission is not started |

### Braking from max speed, arm stowed (`--brake-test`, open floor, μ 0.5)

| spec | direction | stop | stop distance | min wheel / front-caster / rear-caster N (10 ms avg) | caster lift | casters unloaded |
|---|---|---|---|---|---|---|
| v1.0 | forward | Cat 0: brakes bite, 30 ms ramp | **51 mm** (spec calc 46) | 232 / 129 / **0** | 0.11 mm | 0.13 s |
| v1.0 | reverse | Cat 0 | 51 mm | 225 / **0** / 82 | 0.02 mm | 0.01 s |
| v1.0 | forward / reverse | Cat 1: motors 1.5 m/s², then brake | 91 mm | 351 / 149 / 41 and 351 / 84 / 108 | 0 | 0 s |
| provisional (90 kg, μ 1.0, r 0.0825) | forward | Cat 0 | 36 mm | 121 / 0 / 0 | **4.1 mm** | 0.29 s |

**Reading the braking results**
- Under the v1.0 spec, a Cat 0 stop at 0.5 m/s unloads the trailing casters to 0 N for 0.13 s but lifts them only 0.11 mm.
- The CoP stays inside the polygon of loaded contacts (wheels + front casters), so there is **no tip**.
- **This contradicts the spec's "no caster lift-off".** The cause is the sprung drive wheels: they carry a fixed spring force, so the whole pitching moment is taken by the casters.
- A Cat 1 stop has no unloading.

### Base loads during the docked task (video mission; per contact, 10 ms average)
- **Drive wheels:** 349–354 N (spring-defined).
- **Front casters (aisle side):** 6–394 N.
- **Rear casters (cabinet side):** 0–294 N.
  - The base rocks diagonally onto 3 casters when the arm reaches over the counter or door: rigid 4-caster rectangle plus contact compliance.
  - CoP margin to the support polygon stays ≥ 0.148 m. The spec's docked statics gives 0.172 m.
- **Brakes:** at most 13–16 N m used of 27 N m (49–58 %; the spec estimates 41 %).
- **Dock plate preload during the task:** 126–149 N.
- **Base speed during the task:** ≤ 5 mm/s (vibration).
- **Arm wrench on the base:** vertical −120…−213 N, horizontal ≤ 77 N, overturning moment ≤ 74 N m. M1 static cart: 68 N m.

## What was built

| file | content |
|---|---|
| `scene_mobile.py` | free-body wheeled base, spec v1.0. 2 sprung driven wheels (sphere, r 0.10) with 55 N m-peak torque-limited motors. 4 ball casters (r 0.05). Wheel-locking brake actuators with a 27 N m holding limit. Rear sprung dock plate with 2 cone pins (force sensor, k 20 kN/m). Sprung charge collector. 2 corner lidars. IMU and truth pose sensors. Kitchen extended to y 2.65 with a charging station, plinth V-blocks and a reflective strip. FR3 joint limits; Bota F/T (28 mm, 0.21 kg); conductive 8 mm stylus on the outer side of the left finger |
| `bus.py` | message bus with per-link latency. Every message is logged to `out/<run>_messages.jsonl.gz` |
| `nodes.py` | ArmController, BaseMCU, DishwasherPanel, Perception, Operator |
| `planner.py` | MotionPlanner. The `../full_pipeline` task code (`pipeline.Runner`, reused) runs in a lock-step coroutine and never touches MuJoCo controls. Also base navigation and docking jobs |
| `task_supervisor.py` | mission state machine (timeouts, retries, LED-verified panel, battery check, e-stop, abort path) |
| `skills_api.py` | **skill/API layer**: name + args → {ok, error, measurements}. See "Skill API" below |
| `lidar.py` | mj_multiRay lidar and the line + strip localizer |
| `power.py` | electrical model and LFP battery |
| `system.py` | scheduler, monitors, HUD video |
| `run_mobile.py` | CLI: `--video`, `--trials N`, `--dock-sweep`, `--pose-sweep`, `--estop`, `--low-battery`, `--nonconductive`, `--panel-door-open`, `--brake-test`, `--tcp-cap`, `--debug-truth-perception`, `--base-pose lidar\|truth` |
| `run_all.sh` | every reported run, one process at a time under `guard.sh` (3 GB RSS watchdog) |
| `aggregate_mobile.py` | builds `results_mobile.json` and `out/aggregate.txt` |
| `seqdiag.py` | builds `sequence_diagram.png` |
| `check_base.py` | development physics checks |

**Outputs**
- `mission.mp4`: one full real-Codex mission, 226 s, 1280×720. HUD shows the supervisor state, planner job/phase, ArmController mode, BaseMCU brake/speed/e-stop, SoC/V/I, dock force and charging, and the dishwasher MCU state with its 6 LEDs. Codex waits are fast-forwarded 4×, with a still card of the Codex box and depth fit.
- `frames/`: 8 task key frames plus 5 base key frames (pre-dock, docked, panel RUNNING, back at the charger, charging).
- `sequence_diagram.png`: the video mission. 66 event messages drawn as arrows; the 9 periodic streams are summarised with their rates.
- `mission_messages_lowrate.jsonl`: the video mission log without the 500/100/20 Hz streams.
- `results_mobile.json`, `out/*.json` (per run), `out/peak_rss.tsv`.

### Controller architecture ("which chip does what")

| module (real part) | rate | receives | sends | does |
|---|---|---|---|---|
| ArmController (FR3 control box) | 500 Hz | ArmSetpoint, EStop, Resume | ArmState 500 Hz, ArmStatus 10 Hz | setpoint interpolation, FR3 speed/range limits, gravity torque on the joints, torque monitor, Cat-1 stop and hold, 0.5 s resume blend |
| MotionPlanner (main PC, RT) | 100 Hz | ArmState, BaseState, LidarFix, Command, PerceiveResult, Pause/Resume/Cancel | ArmSetpoint, BaseCmd 50 Hz, Brake, PerceiveRequest, JobDone | IK + joint paths at the MEASURED base pose, TCP = FK(encoders), odometry + lidar fusion, path following, V-block and charger docking |
| TaskSupervisor (task state machine) | 10 Hz | JobDone, BMS, PanelReadout, EStop*, OperatorAck | Command (skill calls), Pause/Resume/Cancel | mission, timeouts, retries, abort path, e-stop, low battery |
| Perception (PC + wall node) | 10 Hz | PerceiveRequest, PanelLEDs | LidarFix ×2 lidars, PerceiveResult, PanelReadout | lidar localization; asynchronous `codex exec` (sim time throttled to wall time, so the arm really waits the real latency); LED readout |
| BaseMCU (STM32 + drivers + BMS) | 500 Hz | BaseCmd, Brake, EStopButton | BaseState 100 Hz, BMS 10 Hz, EStop/EStopReleased | wheel velocity PI, accel / yaw-rate limits, 100 ms watchdog, thermal derating, brakes, dock / charge force, IMU tilt, odometry, e-stop chain |
| DishwasherPanel (appliance MCU) | 500 Hz | (contacts, door sensor) | PanelLEDs, PanelEvent | capacitive scan, state machine, START interlock |
| Operator | 10 Hz | | EStopButton, OperatorAck | scripted human |

**Latencies:** arm box↔PC 1 ms, MCU↔PC 2 ms, e-stop chain 1 ms, supervisor↔planner 1 ms, lidar→planner 5 ms, panel LEDs→perception 100 ms (optical), Codex = its real wall latency (12.7–21.1 s).

**Message volume per mission:** about 133 k ArmState, 26.6 k BaseState, 19.5 k ArmSetpoint, 5.3 k LidarFix, 1.7 k BaseCmd, 2.7 k each of the 10 Hz status streams, and about 66 event messages.

**Lock-step planner**
- The planner's sequential task code runs in a thread in strict lock-step with the scheduler. It computes only at its 100 Hz ticks, and nothing runs concurrently.
- Every 500 Hz ArmState sample is replayed through the step's F/T filters and stop conditions, so guarded moves are decided at most 10 ms late.

### Skill API (for the GPT / `codex exec` layer in `../gpt_control`)

`skills_api.request(name, args)` → `Command{id, skill, job, args}` on the bus; the planner's JobDone → `skills_api.result()` = `{skill, ok, error, measurements, t0, t1}`. The mission sequence lives only in the client (`task_supervisor.mission_plan`). The controllers expose skills, not a hard-wired sequence.

**Base and power**
- `base.localize`, `base.localize_docked`, `base.undock_charger`
- `base.drive_to(pose=(x, y, heading_deg), direction)`
- `base.go_predock(offset)`, `base.dock(offset, vblocks)`, `base.undock`
- `base.go_charger`, `base.dock_charger`
- `base.brake(on)`, `base.get_state`, `power.get_soc`

**Robot and arm**
- `robot.get_state`, `robot.wait(s)`
- `arm.unstow`, `arm.stow`, `arm.safe_stow`

**Task skills**
- `vision.locate`
- `skills.open_door`, `skills.load_pod`, `skills.pull_basket`, `skills.pick_plate`, `skills.insert_plate`, `skills.push_basket`, `skills.close_door`
- `skills.press(button, tool=stylus|finger)`

Every mission ends with `robot.get_state` and `power.get_soc`. Example result on the charger: `{soc 0.894, V 53.97, I −9.0 A, charging true}`.

`arm.move_tcp` and `arm.move_until_contact` from the planned `gpt_control` API are not exposed yet.

### Docking method and why
- **Coarse localization: lidar line-fit on the cabinet fronts plus a 50 mm retro-reflective strip.**
  - The spec chose it.
  - The cabinet line is visible all along the counter, so heading and x are corrected continuously while driving.
  - The strip edges are taken from intensity transitions at the ray angles, which are insensitive to range noise.
- **Fine: mechanical.** Reverse at 0.05 m/s into two plinth V-blocks with cone pins, until the sprung dock plate reads 100 N, then brake.
- **Simulated localization accuracy (σ 10 mm range noise, 0.25° rays):**
  - 0.3 mm / 0.3 mm / 0.02° at the 0.55 m pre-dock pose.
  - 0.65 mm / 2.4 mm / 0.15° per scan when docked (the lidar is 8 cm from the doors). This is averaged over 30 scans for the arm.

## What changed when spec.json v1.0 replaced the provisional numbers
- **Base**
  - Wheel r 0.0825 → **0.10**; motors 15/30 → **27.4/55 N m**; brakes 20 → **27 N m**.
  - Floor μ 1.0 → **0.5**.
  - Mass 90 + arm → **112.9 kg base + arm** (sim total 130.4 kg vs spec 133.2: the MJCF Panda is 0.9 kg lighter than the FR3, and some small parts are not modelled).
  - CoG z 0.35 → **0.33** (the spec's base-only CoG is used directly).
  - Driven wheels carry **55 %** (sprung).
- **Battery:** 48 V → **51.2 V (16S)**, BMS ≥ 60 A, 11.3 A charger.
- **Docking** changed from a front bumper + lidar marker to **rear dock side**: reverse into **V-blocks**, **100 N** preload, cone pins and charge collector in the plinth recess. Two corner lidars.
  - The protruding marker block was replaced by a **flush reflective strip**; the block hit the chassis when docked.
- **Arm**
  - **FR3 joint limits** in the model, the IK and the ArmController (q6 ≥ +0.44 rad, q4 ≤ −0.117 rad).
  - **Bota F/T** (+28 mm, +0.21 kg) between flange and hand.
  - FR3 speed limits in the ArmController.
- **Fixed loads** now come from `power.components`.

**Consequences found in simulation** (each was a failure before the fix)
1. **Insert landing shift.** With the FR3 limits and the F/T in the tool stack, the plate landed about 4 mm toward −x. 3 of the first runs dropped it one tine over and it leaned 20°.
   - Fix: the insert aim offset was re-tuned from 3 mm to **8 mm**.
   - Landing is now −2.9…+3.2 mm across the ±5 mm sweep.
2. **Wrist F/T noise.** The sprung free base gives **±2 N** of noise at 500 Hz; the static cart had none.
   - The original 0.5 N insert-touchdown and 3 N button thresholds fired with no contact.
   - Fix: 20–40 ms EMA filters plus hold counts. Button stop is > 2.5 N for 10 ms; touchdown is > 2 N for 20 ms.
3. **Tyre creep.** Soft-constraint tyre friction let the braked base creep about 0.07 mm/s under the dock-spring preload (4.4 mm per task).
   - Fix: `noslip_iterations = 3`, and the brake was modelled as a position-hold actuator with a torque limit instead of dry friction.
   - Creep is now ≤ 0.9 mm per task, which is the static brake compliance.
4. **Panel touch.** The spec's "hold ≥ 0.5 s" press is implemented as F/T admittance at about 3 N.
5. **Dock seat.** The V-block seat is placed so the base sits at the design pose when the plate is compressed by the 100 N preload. Without that, the chassis rear rested on the cabinet doors and the wheels slipped.

**Open item:** the arm in the simulation is still the Panda MJCF, with FR3 limits applied; FR3 link masses and inertias are not used.

## Physically simulated vs modelled vs abstracted

**Physically simulated** (MuJoCo, `mj_step` only, no welds / mocap / teleports)
- The free base on wheel, caster and floor contacts, with suspension springs, brakes as torque-limited wheel locks, and motors as torque actuators.
- The V-block dock, sprung dock plate and charge collector.
- The arm on the moving base, with gravity torque at the joints; the reaction reaches the base.
- Every task contact; the plate, pod, door and basket.
- Stylus and finger contact with the pads.
- Lidar rays (`mj_multiRay`).
- Wall camera RGB-D.

**Modelled** (equations / parameters, EST unless stated)
- **Drives:** motor and drive electrics as η plus copper-loss constants; no inductance, PWM or temperature (only a 10 s RMS derating).
- **Battery:** LFP OCV table, 60 mΩ, coulomb counting; no chemistry, ageing or CV phase.
- **Loads and sensors:** inverter efficiency and fixed loads from the spec; lidar noise σ 10 mm with no bias (the S2's ±30 mm systematic error is NOT modelled); IMU tilt noise 0.02°.
- **Bus:** message latencies are fixed. No jitter, packet loss or Wi-Fi dropouts; Wi-Fi is a fixed 40 ms. The Codex latency is real.
- **Panel MCU:** capacitive sensing is "conductive geom + force + dwell", not a capacitance model. The OFF/STANDBY/SELECTED/RUNNING logic and START-with-POWER are **chosen**; the real E804 manual was not available. The water tank is always OK.
- **Thresholds:** brake engage/release ramps and the 100 ms command watchdog.

**Abstracted / not modelled**
- Global localization beyond the cabinet line and strip (no map, no other obstacles, no people).
- The charger is a separate station 2 m away, as requested. The spec puts the charger on the work dock.
- Casters are ball casters (no swivel offset or flutter).
- The spec's per-dock AprilTag camera recalibration: the pose sweep emulates it with ground truth.
- The FR3 wake/boot phase (spec 90 s, 2.67 Wh).
- Hand power-loss behaviour in the e-stop: assumed to keep its grip.
- No water or washing.

**Statistics:** 6 real-Codex missions and 23 sweep runs show feasibility on this test set. They are not a reliability estimate.

## Run

```bash
PY=.venv/bin/python
./guard.sh mission_video run_mobile.py --video    # always one process at a time, under the RSS watchdog
./run_all.sh                                      # everything reported here, sequential, about 2.5 h
```

**Peak RSS** (in-process `ru_maxrss`; watchdog values in `out/peak_rss.tsv`)
- Single missions: 590–600 MB.
- Video mission: 787 MB.
- 5 trials in one process: 1016 MB.
- Sweep runs: 550–610 MB.
- Low battery / door-open runs: 390 MB.
- Brake test: 388 MB.

Nothing came near the 3 GB kill limit. SVDs use `full_matrices=False`, and video frames are streamed to ffmpeg.
