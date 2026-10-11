# Dishwasher-loading robot: real hardware, power and compute design

## 中文摘要

- **机械臂：保留 Franka（买现款 FR3 + Franka Hand）。**
  - FR3 的运动学与仿真里的 Panda 相同，臂展 0.855 m，所以 10/10 的仿真结果大部分仍可用。
  - **关节限位不同。** 最大的差别是 J6 下限从 −1° 变成 +25°，这一点必须让仿真重跑核对。
  - FR3 控制柜只吃 100–240 V 交流电，约 80 W。车上加一台 48 V→230 V 正弦波逆变器（Victron Phoenix 48/800），每个循环只多耗约 2 Wh。
- **底盘：差速驱动。**
  - 两个驱动轮，用 Oriental Motor BLV-R 200 W 无刷电机：自带断电抱闸，经 50:1 减速，配 200 mm 轮子。另有 4 个万向脚轮。
  - 尺寸与仿真相同（0.65×0.60 m）。所有设备都塞进仿真里那个 0.26 m 高的底盘箱和 0.18 m 方立柱里。
  - 整车 133 kg（底盘 113 kg），收臂时重心高 0.33 m。
  - 0.5 m/s² 加减速时，倾翻安全系数 >10。急停时轮胎打滑（约 2.7 m/s²）先于倾翻（约 5.9 m/s²）。
  - 停靠：用激光雷达拟合柜门线，倒车进 V 形块，压紧 100 N 后抱闸。目标重复精度 ≤1 mm / 0.1°。另外每次停靠后，墙上相机看手上的 AprilTag，重标一次外参。
- **电源：51.2 V 30 Ah 磷酸铁锂（1.54 kWh）。**
  - 每个循环（约 6.6 min，含开机和来回开车）耗 **16 Wh**。保守估算按 FR3 平均 250 W 算，是 26 Wh。
  - 一次充满可做 **76 个循环**，保守 47 个。
  - 充电桩用 Mean Well NPB-750-48（11.3 A），20→100 % 约 2.5 h，每个循环只需补电约 2 min。
  - 机器人本身**不接市电**。需要 3 个插座：洗碗机原有插座、充电桩插座、墙上视觉节点插座。
  - 拖线方案（不用电池）更便宜，但有绊人、压线和断电停机的问题，列为 B 方案。
- **芯片分工：**
  - FR3 控制柜负责 1 kHz 关节力矩环和安全功能。
  - 主机（x86 + PREEMPT_RT）跑 1 kHz FCI 阻抗/导纳、100 Hz IK 规划、状态机和停靠控制。
  - STM32H723 负责底盘：通过 RS-485 下发轮速，管理电池 BMS（CAN）、防撞条和看门狗。
  - 墙上的 Jetson Orin Nano Super + D435i 负责拍照、调 Codex，并在断网时跑本地兜底检测。
  - 急停链路是纯硬件：PILZ 安全继电器同时切断机械臂和轮子。
- **电容触摸键：** 用导电硅胶指尖（Ø8–10 mm），通过导线接到底盘大块金属上，按住 ≥0.5 s。E804 面板的具体感应方式无法核实，所以先用几十块钱做一次实物测试。
- **BOM：** 约 **US$53k（≈S$69k，含 10 % 预留）**，其中机械臂、手爪和力传感器占 US$36k。
- **三大未决风险：**
  1. FR3 的 J6 限位
  2. 触摸键能否被识别
  3. 停靠精度 / 过道宽度 ≥1.0 m

Tags used throughout: **V** = verified with a source (see §11). **R** = from a reseller listing only. **E** = my estimate or calculation. **OPEN** = must be measured or quoted.

Machine-readable numbers are in `spec.json`. Calculations are in `calc.py` (→ `calc_out.json`), `docked_statics.py` (→ `docked_statics.json`) and `bom.py` (→ `bom.json`, `bom_table.md`). Diagrams are made by `diagrams.py`.

---

## 1. Decisions at a glance

| Subsystem | Decision | Key numbers |
|---|---|---|
| Arm | **Franka Research 3 + Franka Hand** (keep Franka) | 7 DoF, reach 0.855 m, payload 3 kg, 18.3 kg (V). 100–240 VAC, ~80 W (V). FCI 1 kHz torque (V) |
| Arm power | 48 V → 230 V pure-sine inverter, **Victron Phoenix 48/800** | 650 W continuous / 1500 W peak, 9.5 W no-load, 91 % (V) |
| Base | Differential drive: 2 driven wheels on the centre axle + 4 corner casters, sprung drive arms | 0.65 × 0.60 m (same as sim), ground clearance 60 mm |
| Drive | **2 × Oriental Motor BLMR6200SKM-GFV-F + GFV6G50 + BLVD-KRD** | 200 W, 24/48 VDC. 27.4 N·m rated / 55 N·m peak at the gearhead (V). Power-off brake (V); ≈27 N·m holding at the wheel (E). 60–80 rpm → 0.84 m/s capability with r = 0.10 m |
| Battery | 51.2 V (16S) **30 Ah LiFePO4**, 2 × 8S modules, one BMS with CAN, IEC 62619 / UN38.3 | 1.54 kWh, ~16 kg (E) |
| Charging | Floor/plinth dock with spring contacts, **Mean Well NPB-750-48** | 11.3 A, CC/CV, CAN (V); 20→100 % ≈ 2.5 h (E) |
| Main computer | **OnLogic Helix 500** (x86, 12–24 V DC in), Ubuntu 22.04 PREEMPT_RT, ROS 2 | FCI client 1 kHz, IK 100 Hz, BT 10 Hz |
| Base MCU | **STM32H723** (Nucleo-H723ZG), FreeRTOS + micro-ROS | wheel commands 50 Hz over RS-485 Modbus; BMS over CAN; watchdog 100 ms |
| Perception | Fixed **wall node**: Jetson Orin Nano Super + RealSense D435i, runs `codex exec` plus an on-device fallback | Codex 11–19 s per call (sim-measured) |
| Docking | 2 × RPLIDAR S2 line-fit on the cabinet fronts → reverse into V-blocks, push 100 N, brake. Then the wall camera re-calibrates from an AprilTag on the hand | target ≤ 1 mm / 0.1° (E) |
| Safety | Hardware e-stop chain: PILZ PNOZ s3 → FR3 X3 e-stop input + drive contactor K2 (fail-safe wheel brakes). Safety edges on all 4 sides | FR3 PL d Cat 3 (V); base ≤ 0.3 m/s near people (E policy) |
| Touch panel | Conductive silicone fingertip Ø8–10 mm, wired to chassis ground; press and hold ≥ 0.5 s | OPEN, cheap test in §7 |
| Cost | **US$53.3k incl. 10 % contingency** (≈ S$69k) | arm + hand + F/T US$36.4k |

---

## 2. Arm choice

### 2.1 Candidates (all verified unless tagged)

| | **Franka FR3** | Kinova Gen3 7-DoF | UR5e + DC OEM box | UFactory xArm 7 (DC box) | Kassow KR810 EDGE |
|---|---|---|---|---|---|
| DoF / reach / payload | 7 / **0.855 m** / 3 kg | 7 / 0.902 m / 4 kg mid-range, 2 kg at full reach | **6** / 0.85 m / 5 kg | 7 / **0.70 m** / 3.5 kg | 7 / 0.85 m / 10 kg (R) |
| Arm mass | 18.3 kg | 8.2 kg | 20.7 kg | 14.3 kg | ~24–26 kg (R) |
| Power input | **100–240 VAC only** | **18–30 VDC** | **19–72 VDC** (OEM DC box) | 24–72 VDC (DC box) | 42–58 VDC (R) |
| Typical / peak power | ~80 W / 450 W (R, UNVERIFIED) | 36 W avg / 155 W peak (2019 sheet) | — / 570 W max | ~200 W / 500 W (line-wide) | ~200 / 400–600 W (R) |
| Controller | 19" box 483×355×89 mm, ~7 kg, must lie flat | in the base | 451×150×168 mm, 4.3 kg | 262×160×76 mm, 2.6 kg | in the base |
| Low-level interface | **FCI 1 kHz torque/joint/Cartesian** over Ethernet | Kortex 1 kHz (pos/vel/current/torque) | RTDE ≤ 500 Hz, position/velocity only | ~250 Hz servo (UNVERIFIED) | — |
| Joint torque sensing | link-side sensors in all 7 joints | in every actuator | estimated from current + flange F/T | current estimate | — |
| Safety | PL d Cat 3; e-stop / enable / 2 safe inputs. "Certification ongoing" | no PL rating published | PL d Cat 3, 23 safety functions | UNVERIFIED | — |
| Price (2026) | **≈US$29.2k** (R) | ≈US$33.5k (R) | ≈US$35–48k (R) | ≈US$11.9–13.1k (R) | quote |
| What the sim needs | **joint limits only** (§2.3) | full re-plan (different kinematics, continuous joints) | full re-plan; the pinch-tilt-insert path with 6 DoF is unproven | **fails reach** (0.70 < 0.855 m) | full re-plan |

### 2.2 Decision: keep Franka and buy the FR3, with an inverter

1. **The simulation stays valid.** The FR3 has the Panda's link geometry and reach (0.855 m). The Panda itself is no longer sold new. Every other arm invalidates the 10/10 result and the mount study.
2. **AC input costs little.** The inverter adds 9.5 W no-load plus about 10 % of ~90 W, which is about 19 W while awake and **≈ 2 Wh per cycle**. The battery has 1.2 kWh usable. The Phoenix 48/800 (650 W continuous, 1500 W peak) covers the 450 W FR3 peak quoted by a reseller with 44 % headroom. Mass 5.5 kg; it sits in the lower bay as ballast.
3. **1 kHz torque control.** The sim's F/T-regulated pushes (8 N edge push, 3 N button stop, 11–14 N insertion touchdown) map directly onto FCI Cartesian impedance or admittance. UR's RTDE is position/velocity only at ≤ 500 Hz.
4. **Plan B is Kinova Gen3.** It is native 24 V, 36 W average, 8.2 kg and has no controller box. Choose it only if the FR3 joint-limit re-check (§2.3) fails, and accept a full re-simulation.

### 2.3 What the simulation must re-check for FR3 (in `spec.json → arm_choice.sim_recheck_required`)

| Joint | Panda limits used in the sim (rad) | FR3 limits (rad) | Effect |
|---|---|---|---|
| J1 | ±2.8973 | ±2.9007 | same |
| J2 | ±1.7628 | ±1.8361 | FR3 wider |
| J3 | ±2.8973 | ±2.9007 | same |
| J4 | −3.0718 / −0.0698 | −3.0770 / **−0.1169** | upper limit 2.7° tighter |
| J5 | ±2.8973 | **±2.8763** | 1.2° tighter |
| **J6** | **−0.0175** / 3.7525 | **+0.4398** / 4.6216 | **lower limit 26.2° tighter**, upper 50° wider |
| J7 | ±2.8973 | ±3.0508 | FR3 wider |

- **How bad it could be:** the sim's worst joint margin was 12.9° (insert) and 15.0° (door). If either of those margins is on J6's lower side, **that step is infeasible on an FR3** until it is re-planned.
- **Why I can't settle it here:** the sim logs do not store joint trajectories, so I could not check this offline.
- **Requested from the simulation work:** re-plan with the FR3 limits and report min q6 per step.
- **Also on the sim's list:**
  - The Bota F/T adds 0.21 kg and its height between the flange and the hand (TCP offset).
  - FR3 velocity limits are higher than Panda's, so they are not a concern.

---

## 3. Mobile base

### 3.1 Geometry and packaging (the sim's cart is the envelope)

The sim cart is a **chassis box at z 0.06–0.32 m over the full 0.65 × 0.60 m footprint**, plus a **0.18 × 0.18 m column** up to the arm flange at z 0.80. The open dishwasher door overhangs the cart at z 0.91, and the arm works above the deck.

So **everything must fit inside the chassis box and the column** (`spec.json → envelope`):

| Bay | Contents |
|---|---|
| Lower bay (z 0.06–0.20) | Centre axle: the 2 drive units (motor + brake + gearhead inboard of each 200 mm wheel, ~0.2 m each). Fore and aft of the axle: one 8S battery module each, inverter, ballast plate |
| Upper bay (z 0.20–0.32) | FR3 Control lying flat (89 mm high, as Franka requires), main PC, DC-DC, PNOZ, MCU, switch |
| Column | Cable riser (arm cable, F/T USB, ground lead), e-stop button on top-side face |
| Corners at z ≈ 0.25 | 2 × RPLIDAR S2 (front-left, rear-right) in cut-outs |
| Perimeter | ASO safety edges. Rear (dock side): 2 cone pins + charge collector at z ≈ 0.05, inside the 75 mm plinth recess |

**Mass budget** (base frame: x = 0 at the footprint centre, + toward the aisle; `calc.py`):

| Part | kg | x (m) | z CoG (m) | Tag |
|---|---|---|---|---|
| FR3 arm, stowed | 18.30 | +0.125 | 1.05 | V mass / E CoG |
| Franka Hand + fingertip | 0.75 | +0.075 | 1.25 | V |
| Bota SensONE F/T | 0.21 | +0.075 | 1.20 | V |
| Column 0.18 × 0.18 × 0.48 m steel + flanges | 14.0 | +0.175 | 0.56 | E |
| FR3 Control | 7.0 | −0.08 | 0.27 | V |
| Inverter Phoenix 48/800 | 5.5 | −0.18 | 0.26 | V |
| Battery 51.2 V 30 Ah (2 × 8S) + BMS | 16.0 | −0.02 | 0.13 | E (a 30 Ah 16S pack is ~16 kg, Leoch, R) |
| 2 × drive unit + wheel + bearing block + spring arm | 13.0 | 0 | 0.10 | E |
| 4 × caster 100 mm | 4.4 | 0 | 0.07 | E |
| Chassis frame, deck, covers | 28.0 | 0 | 0.18 | E |
| Compute, DC-DC, safety, wiring | 6.0 | −0.15 | 0.25 | E |
| Edges, lidars, dock pins, collector | 5.0 | 0 | 0.20 | E |
| Ballast plate | 15.0 | +0.12 | 0.08 | E (design choice: lowers the CoG; position tunable) |
| **Total** | **133.2** | **+0.029** | **0.330** | base without arm/hand/F/T: **112.9 kg**, CoG z 0.208 |

### 3.2 Drive sizing (`calc_out.json → drive`)

| Quantity | Value | Basis |
|---|---|---|
| Wheel radius / track | 0.10 m / 0.50 m | E design |
| Driven wheels' share of weight | 55 % (sprung arms) | E design |
| Rolling resistance (C_rr 0.02) | 26 N → 1.3 N·m per wheel | E |
| Accelerate at 0.5 m/s² | 4.6 N·m per wheel | calc |
| Dock push 100 N | 6.3 N·m per wheel | calc |
| 5 mm step (100 mm casters lead) | 15.5 N·m per wheel; traction-limited max step ≈ **6.5 mm** | calc (μ 0.5) |
| Motor capability | **27.4 N·m rated, 55 N·m peak** per wheel (274 / 550 N at the tyre) | V (Oriental) |
| Speed | 0.5 m/s = 48 wheel rpm = 2390 motor rpm (range 1–4000). Capability 0.84 m/s | V + calc |
| Commanded limits | 0.5 m/s max, **0.3 m/s near people**, 0.05 m/s final dock approach, 0.5 m/s², 0.8 rad/s yaw | E policy |
| Mechanical power | 46 W peak accel, 13 W cruise (rated 2 × 200 W) | calc |
| Odometry | BLV-R hall feedback × 50 → about 0.4 mm of travel per pulse (E). Fine for a 2 m trip; the final pose comes from the dock | E |

- **Torque margin:** the 200 W/50:1 unit has about 5× torque margin. The 400 W version is 48 V-only and rated for 30 minutes, so it was not chosen.
- **Why not hub motors:** I found no hub motor with a confirmed spring-applied power-off brake (ZLTECH's "brake" is dynamic braking). That is why a geared unit with a power-off brake was chosen over the cheaper hub motors.

### 3.3 Tipping while driving and braking (arm stowed)

| Check | Value |
|---|---|
| CoG height (total) | 0.33 m |
| Distance CoG → caster tip line | front 0.236, rear 0.294, side 0.238 m; minus 0.035 m caster swivel offset (worst case) → **0.20 m** |
| Deceleration needed to tip | 7.0 m/s² nominal / **≈ 5.9 m/s²** with the swivel offset |
| Commanded acceleration | 0.5 m/s² → **safety factor > 10** |
| E-stop with brakes biting | brake torque limit 540 N; **tyre friction limit 359 N** (μ 0.5 on the 55 % carried by driven wheels) → **2.7 m/s²**. Stop distance 46 mm from 0.5 m/s, 17 mm from 0.3 m/s |
| Can braking tip it? | **No.** The wheels skid at ≤ 4.9 m/s² (μ g), below the ~5.9 m/s² needed to tip. Margin factor 2.2 at the actual 2.7 m/s² |
| Static tilt to tip (ramp) | 35° |

### 3.4 Fail-safe brakes and floor

- **Brakes:** the BLV-R brake is **"power off activated, automatically controlled by the driver"** (V). Static torque is 0.637 N·m at the motor (V), ×50 ≈ 32 N·m. I cap it at the 27.4 N·m gearhead rating, giving **≈ 27 N·m per wheel**.
  - **OPEN:** confirm the output holding figure and the gearhead overhung-load rating with Oriental. My design uses a separate axle bearing block, so the gearhead sees torque only.
  - **When it applies:** on any K2 opening, BMS cut-off, driver fault or MCU watchdog stop.
- **Docked hold** (`docked_statics.json`, recorded sim wrenches, 204 k samples):
  - The brakes must hold 100 N dock preload + 39 N arm pull-away + 58 N side.
  - That is **41 % of available tyre friction**.
  - Worst centre-of-pressure margin: **0.172 m** (sim reference 0.183 m with the assumed 135 kg cart).
  - Dock contact force: ≤ 155 N (100 N preload + 55 N arm push).
- **Floor:** the route must be **flat tile**.
  - Max step 5 mm (traction limit about 6.5 mm with 100 mm casters).
  - Max gap 15 mm (E).
  - AgileX Tracer, for comparison, is rated for a 10 mm step / 25 mm gap (V).
  - Kitchen thresholds or rugs on the route are not allowed. **OPEN:** measure the real floor.
  - Wet floor: μ may fall to about 0.3 (E). Brake hold is then about 215 N, still above the 151 N needed when docked (100 N preload + 39 N pull-away, combined with 58 N side load).

### 3.5 Docking: localisation, precision, force

1. **Coarse (from the charge dock):** odometry + 2 × RPLIDAR S2 (±30 mm accuracy, 0.12°, 10 Hz, V).
   - The lidars fit the cabinet-front / plinth line (sub-degree heading) and its end, or a 50 mm reflective strip on the plinth (y position).
   - This gets the cart to within ±20 mm / ±2° of the dock, 0.15 m out.
2. **Fine (mechanical):** reverse at 0.05 m/s into **two floor- or plinth-mounted V-blocks**, with two cone pins on the cart's rear at z ≈ 0.05 (inside the 75 mm plinth recess).
   - Capture about ±30 mm / ±3° (E).
   - The drive pushes 100 N, then the brakes apply. Seating is confirmed by both dock micro-switches plus a motor-current rise.
   - Expected repeatability of a two-pin V-block dock: **≤ 1 mm / 0.1°** (E, typical of kinematic couplings; OPEN, measure 20 docks).
3. **Per-dock calibration (removes base error from perception):**
   - The arm shows an AprilTag on the hand at 3 poses to the wall camera. The wall node re-estimates T(camera→robot), taking about 15 s.
   - The plate pose then reaches the robot already in the robot frame. The sim's plate errors were 0.4–1.75 mm, and the M3 rack pick failed at 2.1–2.3 mm, so this matters.
   - The dishwasher-frame targets (door bar, basket handle, buttons, pod groove) still see the base error directly.
   - Optional extra: an F/T "touch-off" on 3 points of the counter edge (about 20 s) measures the real base pose to ≈ 1 mm.
4. **How much base error the task tolerates is OPEN.** The joint margins (≥ 12.9°) do not translate into position tolerance.
   - A sweep was requested from the simulation work: ±2/±5 mm and ±0.2/±0.5° with the dishwasher model not re-measured (`spec.json → sim_rechecks_requested`).
   - A 0.1° yaw error moves the hand 1.4 mm at 0.8 m reach.
   - The door bar has a 22 mm finger gap and the buttons are F/T-guided, so a few millimetres are probably fine (E).

### 3.6 Alternatives considered for the base

| Option | Why not chosen |
|---|---|
| ZLTECH hub motors + ZLAC8015D (CANopen) | Cheapest and compact, but no verified **spring-applied** brake (only dynamic braking). 150 kg-per-pair rating is marginal at 133 kg. Usable only if a separate fail-safe parking brake is added (none found off-the-shelf) |
| maxon MW 500 wheel drive (160 mm, 23 N·m, ≥ 64 N·m brake, CANopen) | Very good fit, but the datasheet is "Preliminary", the brake's fail-safe type is not stated and there is no price. Worth a quote |
| Commercial AMR base: AgileX Tracer 2.0 (702×610 mm, ~€9k R, 150 kg payload), Ranger Mini 3.0 (€17.9k R), MiR250, Robotnik RB-Kairos | They don't fit the sim envelope: the deck is 169–358 mm high with no room for FR3 Control + inverter + column inside 0.26 m, or the footprint is ~0.8–1.0 m. No fail-safe-brake data for Tracer. Tracer 2.0 is the best "buy" fallback if a custom chassis is unwanted; the column and equipment would then sit on top, so re-check the door clearance |

---

## 4. Power system

### 4.1 Architecture (`power_flow.png`)

**Main path:** battery 51.2 V 16S LFP (BMS: CAN, ≥ 60 A continuous) → **F0** Class T 100 A at the pack → **S0** lockable manual disconnect (≥ 60 VDC rated; a 48 V-rated switch is not enough at 58.4 V full) → **K1** main contactor (Albright SW80, 48 V coil).
- K1 closes only after **pre-charge** through 68 Ω / 25 W. The MCU closes K1 once the bus reaches 90 % of pack voltage.
- K1 drops on key-off, BMS fault or 44 V cut-off.

**48 V bus branches:**

| Fuse | Branch |
|---|---|
| F1 40 A | Inverter Phoenix 48/800 → 230 V → FR3 Control |
| F2 25 A | **K2 safety contactor** (PNOZ output) → 2 × BLVD-KRD drivers → motors and brakes |
| F3 10 A | Mean Well RSD-300C-24 (33.6–62.4 V in, isolated, 300 W, 92 %) → 24 V rail: PC, switch, lidars (5 V buck), MCU, PNOZ, edges relay, F/T |
| F4 20 A | Charge inlet: spring collector → ideal-diode → BMS charge FET |

All DC fuses and switches must be rated **≥ 60 VDC**. Ordinary 32 V blade fuses are not acceptable.

**Peak current:** all peaks coincident ≈ 1.08 kW = **23 A at 48 V** (calc). The inverter's 1.5 kW surge adds about 38 A for a few seconds, which is within F0 and the BMS.

### 4.2 Power budget (`calc_out.json`; bus-side W including inverter/DC-DC losses)

| Load | Path | Idle W | Active W | Peak W | Tag |
|---|---|---|---|---|---|
| FR3 arm + Control | AC via inverter (η 0.90) | 60 → 66.7 | 80 → 88.9 | 450 | V ~80 W; idle E; peak R |
| Inverter self-consumption | — | 9.5 | 9.5 | — | V |
| Main PC Helix 500 | 24 V (η 0.92) | 12 → 13.0 | 30 → 32.6 | 60 | E (60 W under stress, AnandTech) |
| 2 × RPLIDAR S2 | 24 V | 0.5 (stopped) | 5.4 | 6 | V/R |
| GbE switch | 24 V | 4.3 | 4.3 | 5 | E |
| MCU + transceivers | 24 V | 1.1 | 1.1 | 1.5 | E |
| PNOZ + edge relay | 24 V | 2.7 | 2.7 | 2.5 | E |
| Bota F/T | 24 V | 1.1 | 1.1 | 1 | E |
| 2 × BLVD-KRD standby | 48 V | 8 | 8 | 8 | E |
| 2 × BLV-R driving | 48 V | 0 | 45 | 470 | calc |
| 2 × brake coils (released) | 48 V | 0 | 12 | 12 | E |
| **Awake, idle** | | **107 W** | | | |

**Energy per cycle:**

| Phase | s | Bus W | Wh |
|---|---|---|---|
| Wake: inverter on, FR3 boot + self-test (boot time OPEN) | 90 | 107 | 2.7 |
| Undock, drive ~2.2 m, dock, calibrate | 45 | 188 | 2.4 |
| Arm task (sim 185 s) | 185 | 149 | 7.7 |
| 2 Codex waits (arm holding) | 32 | 127 | 1.1 |
| Drive back + dock on charger | 45 | 188 | 2.4 |
| **Cycle (6.6 min)** | 397 | | **16.2 Wh** (pessimistic, FR3 averaging 250 W: **25.9 Wh**) |

**What that buys:**

| Metric | Value |
|---|---|
| Battery usable (80 % of 1536 Wh) | 1229 Wh |
| Cycles per charge | **76** (pessimistic 47) |
| Awake-idle time off the charger | 11.5 h |
| Recharge per cycle | ≈ 1.7 min at 11.3 A |
| Full charge 20 → 100 % | ≈ 2.5 h (E) |
| Asleep on the dock (inverter, FR3, drivers off; PC suspended, wake-on-LAN) | ≈ 6 W (E), supplied by the charger |

**Battery sizing:** a home runs 1–3 cycles a day, so 30 Ah is oversized for energy. It is kept for C-rate: 1500 W surge = 1 C. It also preserves BMS current headroom and lets the robot stay awake all day off the charger. A 20 Ah pack would also work.

### 4.3 Charging dock

- **Charger:** Mean Well NPB-750-48 (V: 90–264 VAC, 11.3 A, 4 preset curves + programmable CC/CV, CAN, "battery full" signal), set to 57.6 V absorption / 54.4 V float for 16S LFP (E).
- **Contacts:** 2 spring-loaded contacts on the dock plate plus a collector on the cart rear at z ≈ 0.05, in the plinth recess. Examples: RoboPad 100 A collector (V); Conductix/Stäubli AGV contacts tolerate ±4–5 mm (V, quote).
- **Live only when docked:** the contacts are energised only when the dock's presence switch closes **and** the charger sees a battery voltage. A dock relay does the switching.
- **Input:** ≈ 0.75 kW from socket **S2**, well within a 13 A BS1363 socket.

### 4.4 Battery vs tethered (mains cable on a reel)

| | **Battery (chosen)** | Tether (plan B) |
|---|---|---|
| Hardware | + LFP pack, BMS, inverter, charger, contacts (≈ US$2.1k) | + 230 V cord reel (no 230 V robot reel found off-the-shelf; the Reelcraft spring reel is 125 V, US$953, R), + a 48 V PSU for the drives. − battery/inverter/charger. Net ≈ −US$1k |
| Mass | +16 kg pack + 5.5 kg inverter, both useful as ballast | lighter, but ballast is needed anyway |
| Cable vs the dishwasher door | none | The door opens downward to a horizontal slab at z 0.91–0.98 and does not sweep the floor, so a floor cable is safe from the door. **But:** the dock face is against the plinth, so a cable along the plinth gets pinched in the dock. A cable from the aisle side is a trip hazard across the 1 m aisle. The turn-in-place can run over it. An overhead feed would hang through the arm's workspace and under the wall cabinets / camera |
| Mains outage mid-task | robot unaffected; finishes or parks | robot loses power mid-task: arm brakes freeze it with the door open or the plate in hand |
| Battery hazards | LFP (most thermally stable Li chemistry), BMS, Class T fuse, steel compartment, IEC 62619 pack | none |
| Verdict | Chosen for **no cable on the floor or in the workspace** and **ride-through** | Use if a battery is unwanted: put the reel at the park spot, run the cord to the cart's aisle-side top through a 0.3 m swivel boom, and limit the route to a straight line |

### 4.5 Sockets needed (all 230 V / 50 Hz BS1363)

| Socket | Where | What |
|---|---|---|
| **S1** | Backsplash next to the dishwasher (existing) | Dishwasher E804 only. The robot never powers it |
| **S2** | Low wall or inside the cabinet at the park/charge dock | NPB-750-48 charger (≤ 0.75 kW), DC cable through the toe-kick to the dock plate |
| **S3** | Under the wall cabinet near the camera | Wall node: Jetson Orin Nano Super (7–25 W) + D435i |
| — | Work dock | **no socket needed** |

Layout: `kitchen_topview.png`.

---

## 5. Compute: which chip controls what (`block_diagram.png`)

### 5.1 Processors

| Processor | Where | OS | Role | Loop rate | Interfaces |
|---|---|---|---|---|---|
| **FR3 Control** (Franka) | Robot upper bay, 230 V | Franka proprietary RT | Joint torque control, joint brakes, collision detection, safety functions (SLS/SLP/SLD), Franka Hand | 1 kHz (V) | GbE to the PC (FCI), X3 e-stop/enable inputs, arm cable |
| **Main PC**: OnLogic Helix 500 (x86, 12–24 V DC) | Robot upper bay | Ubuntu 22.04 + PREEMPT_RT, ROS 2 Humble, libfranka | FCI client (Cartesian impedance + F/T admittance), IK and joint-path planner (mink, as in the sim), behaviour-tree task state machine, docking controller, lidar line-fit, SOC policy | **1 kHz** RT thread (round trip < 1 ms; 20 missed packets stop the arm, V); IK 100 Hz; BT 10 Hz; base 50 Hz | Dedicated GbE to FR3 Control (**no switch**, Franka requirement, V); 2nd GbE to the switch; USB/serial to the Bota (800 Hz); Wi-Fi 6E |
| **Base MCU**: STM32H723 (Nucleo-H723ZG) | Robot | FreeRTOS + micro-ROS (UDP) | Wheel velocity → drivers; command watchdog (100 ms → zero speed + brake); BMS (CAN); K1 pre-charge sequencing; dock switches, charge detect; edge/e-stop/K2 status (read only) | 50 Hz wheel, 1 kHz I/O scan | Ethernet (micro-ROS), RS-485 Modbus RTU to BLVD-KRD, CAN to BMS, GPIO |
| **2 × BLVD-KRD** (Oriental) | Robot | firmware | Motor commutation, speed loop, **brake control** | internal | RS-485 Modbus RTU (V); CANopen per a secondary summary (verify) |
| **BMS** | in the pack | firmware | Cell V/T, SOC, charge/discharge FETs | — | CAN |
| **Bota SensONE** | wrist | firmware | 6-axis F/T | 800 Hz serial (1 kHz on EtherCAT) (V) | USB/serial |
| **Wall node**: Jetson Orin Nano Super + RealSense D435i | Under the wall cabinet (z 1.46) | JetPack 6.2 (Ubuntu 22.04) | RGB-D capture; depth plane + circle fit (as in `perception.py`); `codex exec` calls; **on-device fallback detector** (Hough circle + depth RANSAC on the known 8" plate, optional small YOLO); AprilTag extrinsic calibration per dock; **hosts the robot's private Wi-Fi AP** | 30 fps camera; 1 result per request | USB 3 to the camera; Ethernet uplink to the home router; Wi-Fi AP to the robot |
| Codex VLM | Cloud | — | 2-D plate detection | 11–19 s per call (sim-measured) | HTTPS |

**Why the perception computer is on the wall, not the robot:**
- The camera is fixed to the wall, so a USB cable to a moving robot is impossible.
- It keeps about 15 W off the battery.
- With the Jetson as the robot's own AP, a home Wi-Fi outage does not cut the robot off. An internet outage only removes Codex, and the local detector takes over.

### 5.2 Task steps → processors → messages

| Step | Wall node | Main PC | FR3 Control | MCU / drivers |
|---|---|---|---|---|
| 0. Start | — | User or app request (or a schedule) → wake from suspend (WoL from the MCU). Checks SOC ≥ 30 %, closes K1 via the MCU, inverter on, FR3 boot | boot, self-test, unlock brakes | pre-charge → K1. BMS SOC → PC |
| 0b. Drive to the work dock | — | Plan: out 0.165 m, turn, 1.83 m, turn, back in. Lidar line-fit 10 Hz → twist 50 Hz → MCU | arm stowed, brakes on | drivers follow wheel speeds; watchdog; brakes release while moving, apply at the dock |
| 0c. Dock + calibrate | Detects the AprilTag on the hand at 3 poses → T_cam→robot | moves the arm to 3 poses | joint control | brakes holding |
| 1. Perceive plate | Capture → **Codex** (≈ 15 s) → depth fit → plate pose in the robot frame → JSON to PC | waits (arm still) | holding | — |
| 2. Open door | — | Joint-path pre-plan + Cartesian impedance; F/T watches the bar force (13 N peak in the sim) | 1 kHz torque; Hand grasp | — |
| 3. Pod → groove | — | pick from the known dish, place | 1 kHz | — |
| 4. Pull basket | — | impedance pull 0.25 m; position from proprioception, clamped to the known travel | 1 kHz | — |
| 5. Plate: push, re-perceive, pinch, tilt, lift | 2nd capture → **Codex** → edge pose | F/T admittance 8 N push (Bota 800 Hz); grasp; in-hand transform recorded | 1 kHz; Hand force-limited grasp | — |
| 6. Insert | — | carry with the measured in-hand transform; stop on the F/T touchdown (11–14 N) | 1 kHz | — |
| 7–8. Basket in, door close | — | impedance paths | 1 kHz | — |
| 9. QUICK, then POWER | — | approach until F/T 3 N, **hold 0.5 s**, retract (×2) | 1 kHz | — |
| 10. Return + charge | — | stow, undock, drive back, dock on the contacts; inverter off → sleep | power down | charge detect; BMS charges |

### 5.3 Network and latency budget

| Link | Latency | What happens if it fails |
|---|---|---|
| PC ↔ FR3 Control (direct GbE) | < 1 ms round trip (hard requirement) | 20 lost packets → the arm stops (V) |
| PC ↔ MCU (switch, UDP) | < 1 ms | MCU watchdog 100 ms → stop + brake |
| Robot ↔ wall node (Jetson AP, Wi-Fi) | 2–20 ms (E); 1 message per perception | The task waits. No control loop crosses Wi-Fi |
| Wall node ↔ Codex | RTT 20–60 ms (V, FCC/AWS) + 11–19 s inference (sim-measured) | Local fallback detector; same depth fit, so mm accuracy is unchanged (E) |

---

## 6. Safety

| Hazard | Measure | Basis |
|---|---|---|
| Arm contact with a person | FR3 power and force limiting; collision thresholds set about 2× the largest task force (door 13 N, insertion 14 N → **~30 N** Cartesian). TCP speed ≤ 0.25 m/s via FR3 SLS when a person may be present (impact on cycle time OPEN) | ISO/TS 15066 quasi-static limits (V): hand 140 N, forearm 160 N, chest 140 N, face 65 N, skull 130 N. The 250 mm/s reduced speed is the T1-mode limit (V), used here as a conservative cap |
| Base hitting a person | ≤ 0.3 m/s near people (E policy). Safety edges on all 4 sides → ELMON relay (Cat 3 / PL e, V) → PNOZ → K2 opens → brakes. Stop 17 mm from 0.3 m/s; kinetic energy 6 J | **OPEN:** choose an edge with overtravel > 17 mm braking + ~15 mm reaction travel. SICK nanoScan3 (PL d, ≈ £2.1–2.6k, V/R) is the upgrade for a protective field |
| E-stop | 2 mushroom buttons on the cart + wireless e-stop → **PILZ PNOZ s3** (dual channel) → FR3 X3 e-stop input (stop Cat 0/1 → joint brakes) **and** K2 (drives unpowered, fail-safe brakes on). Aux contact → MCU (status only). No software in the chain | IEC 60204-1: an e-stop is Cat 0 or 1 (V). FR3 PL d Cat 3, e-stop input (V) |
| Battery | LFP, BMS (cell OV/UV/OT, charge/discharge FETs), Class T fuse at the pack, steel compartment, IEC 62619 (covers AGVs, V) and UN38.3 certificates required from the supplier. Charge only through the dock's CC/CV charger; contacts dead unless docked | V standards |
| Water | The open door is above the chassis at y 0.12–0.215 m, so drips land on the deck: IPX4 top cover, cable glands. Drives and lidars sit low and inboard | E |
| Software / runaway | MCU watchdog; FCI drop → arm stops; BT never starts below 30 % SOC | — |
| Standards scope | A home mobile manipulator fits **ISO 13482** (personal care robots, V scope). ISO 10218-1/-2:2025 now contains the old ISO/TS 15066 body table (Annex M) (V). IEC 60335-1 applicability not researched | **OPEN** for any product use |

### 6.1 Power loss or stop mid-task

| Event | Arm | Base | Door / plate | Recovery |
|---|---|---|---|---|
| E-stop / safety edge | stops; joint brakes engage | K2 opens; brakes apply | Plate stays pinched if the Franka Hand holds without power (**OPEN**: Hand power-loss behaviour unverified). Door held where it is, or released | Operator resets PNOZ. BT resumes from the last checkpoint after a fresh perception (plate or door state re-checked) |
| **Mains outage** | none: battery | none | none | Dishwasher S1 loses power; the robot just presses buttons again later |
| Home Wi-Fi / internet outage | none | none | Codex unavailable → local detector | — |
| PC crash | FCI drop → stop, brakes | MCU watchdog → stop + brake | as e-stop | reboot, resume |
| Battery cut-off | power lost → joint brakes, Hand **OPEN** | brakes apply (power-off) | The plate may fall onto the counter or open door (≤ 0.3 m drop). E804 door behaviour when released half-open is **OPEN** (sim: free hinge to its stop) | Prevented by SOC policy: no cycle below 30 %, 44 V hard cut-off, plus 76 cycles of margin |

---

## 7. Pressing the capacitive touch panel

- **How it senses (V):** appliance touch keys mostly use **self-capacitance**. A finger adds a capacitance to ground through the body, typically **1–10 pF** (TI); Renesas puts a finger at about 10 pF against 10–50 pF parasitic.
  - Holtek's BS83B series, a common appliance touch MCU, is self-capacitance with frequency-shift sensing; the threshold and debounce are set in firmware (V).
  - Mutual-capacitance keys see a < 1 pF *decrease* (R).
- **Why a bare robot finger fails:** a plastic finger is a dielectric and adds almost nothing. A small metal tip that is not connected to anything has little capacitance to earth.
- **Fingertip spec:**
  - **Conductive silicone or rubber tip, Ø 8–10 mm** (keys are designed for ≥ 10 mm fingers, Cypress AN2292, V).
  - It sits on a brass insert in a 3D-printed pad on one Franka finger.
  - **Braided lead along the arm to the chassis frame / battery negative.**
  - Press until the F/T reads 3 N, as in the sim, and **hold ≥ 0.5 s**. Firmware debounce is typically a few scans; 0.3–0.5 s is my recommendation (E).
  - Off-the-shelf tips: 6 mm silicone stylus tips, 40 for US$12 (V listing); 8 mm Extylus tip (V listing).
- **Grounding on a battery robot (E):**
  - The robot is not earthed. Its 0.65 × 0.60 m metal chassis 60 mm above the floor has roughly ε₀A/d ≈ 60 pF to the floor, about 100 pF with fringing (my estimate). That is the same order as a human body.
  - **Optional:** a PE contact in the work-dock V-block, earthed through S1's earth, makes the coupling unambiguous.
- **E804's actual sensing method cannot be verified** (no teardown found). **OPEN risk, with a cheap test plan (~US$20, 1 h):**
  1. On the real E804, try a conductive tip on an insulating stick in 3 states: (a) floating, (b) wired to a 0.5 m² aluminium sheet on the floor, (c) wired to the appliance's PE.
  2. Use tip Ø 6 / 8 / 10 mm and hold 0.1 / 0.3 / 0.5 / 1.0 s, 20 presses each. Record the success rate and any double-triggers (QUICK must register exactly once).
  3. Repeat with water droplets on the panel.
  4. **Accept** if ≥ 20/20 in configuration (b) at 0.5 s.
  5. **Otherwise** use (c) with the dock-PE contact. As a last resort, ask for an appliance with mechanical buttons.

---

## 8. Bill of materials

Full table with sources: `bom_table.md` and `bom.json`.

| Group | US$ | Main items |
|---|---|---|
| Arm | 36,450 | FR3 + Control 29,218 (R), Franka Hand 1,777 (R), Bota SensONE 5,430 (R), fingertip 25 |
| Drive | 3,306 | 2 × Oriental BLV-R 200 W brake set **1,373 each (V, Oriental web price)**, wheels and bearings 400 (E), casters 160 (E) |
| Structure | 1,790 | chassis, deck, column, covers 1,500 (E), ballast, 2 dock V-block sets |
| Power | 2,490 | LFP 30 Ah pack 900 (E, quote), Phoenix 48/800 260 (R), RSD-300C-24 130 (R), 2 contactors 180 (R), fuses/disconnect/cabling 450 (E), NPB-750-48 220 (R), charge contacts 350 (E) |
| Compute | 1,340 | Helix 500 1,000 (R/E), AX210 35, Nucleo-H723ZG + carrier 110, EKI-2725 switch 195 (R) |
| Sensors | 800 | 2 × RPLIDAR S2 (E) |
| Wall node | 999 | Jetson Orin Nano Super 399 (after the July 2026 price rise, R), D435i + mount 600 (R) |
| Safety | 1,260 | PNOZ s3 170 (R), e-stops + key 120, wireless e-stop 170 (R, generic, not safety-rated), safety edges + ELMON relay 800 (R/E) |
| **Subtotal** | **48,435** | |
| Contingency 10 % | 4,844 | |
| **Total** | **≈ US$53.3k ≈ S$69k** (rate 1.30, E) | Everything except arm + hand: ≈ US$22.3k |

Not included: labour, design or fabrication engineering, the dishwasher, tooling, certification, shipping and GST.

---

## 9. Failure modes and honest limits

### 9.1 VERIFIED (source in §11)

- **FR3:** 0.855 m reach, 3 kg payload, 18.3 kg, 100–240 VAC, ~80 W, 19" controller ~7 kg, FCI 1 kHz, PL d Cat 3 with e-stop input. FR3 joint limits and their difference from Panda.
- **Kinova Gen3, UR5e DC OEM box, xArm 7 DC box:** input ranges, reach and power figures as tabled.
- **Oriental BLV-R 200 W / 50:1:** 27.4 / 55 N·m, 24–48 VDC, power-off brake 0.637 N·m at the motor, US$1,373.
- **Power parts:** Victron Phoenix 48/800 (650 W continuous, 1500 W peak, 9.5 W no-load, 5.5 kg); Mean Well RSD-300C-24 input range and efficiency; NPB-750-48 (11.3 A, CAN).
- **Sensors and compute:** RPLIDAR S2 (±30 mm, 0.12°); RealSense D435i (spun out of Intel in 2025, still sold); Bota SensONE (800 Hz serial); Jetson price rise (July 2026).
- **Safety and touch:** ISO/TS 15066 quasi-static force table; capacitive touch principles and the Holtek BS83 sensing method.

### 9.2 ESTIMATED (my numbers; calculation in `calc.py`)

- **Masses:** all structural and battery masses; the 133 kg total and CoG z 0.33 m.
- **Drive physics:** C_rr 0.02, μ 0.5, the 55 % driven-wheel load split, ≈ 27 N·m brake hold at the output, step capability ~6.5 mm.
- **Power:** FR3 idle 60 W, PC 12–30 W, driver standby and brake-coil power, FR3 boot 90 s → 16 Wh per cycle (26 Wh pessimistic).
- **Docking:** repeatability ≤ 1 mm / 0.1°, capture ±30 mm / ±3°.
- **Prices:** battery, chassis, wheels, safety edges and lidar.
- **Touch coupling:** chassis-to-floor capacitance ~100 pF.

### 9.3 OPEN (measure or quote before building)

1. **FR3 joint limits in the sim** (J6 lower limit +25°). This is the highest risk.
2. **Capacitive panel registration** with a grounded conductive tip (test plan in §7).
3. **Base-pose tolerance** of the task (sim sweep), plus measured dock repeatability.
4. **Kitchen geometry:**
   - aisle ≥ 1.0 m for the turn-in-place;
   - a flat floor with no step over 5 mm;
   - cabinet **door handles**: the sim assumed flush fronts at x = −0.02, and handles protrude about 30 mm into the dock face. Use V-blocks on the plinth and keep the cart's rear face clear of handles.
5. **FR3 real power draw** (only "~80 W" is published; the 450 W peak is a reseller figure) and boot time.
6. **Oriental output brake torque** and gearhead overhung-load rating.
7. **Franka Hand** grip retention on power loss.
8. **Battery pack** supplier quote (16S 30 Ah, CAN BMS, IEC 62619) that fits 2 × ~0.19 × 0.45 × 0.13 m modules.
9. **FR3 certification** status ("certification process ongoing" in its datasheet). For a home product this, plus ISO 13482 and IEC 60335 applicability, is unresolved.
10. **Safety-edge overtravel vs stopping distance**, and whether a safety lidar is needed near people.
11. **Arm speed cap** of 0.25 m/s near people: its effect on cycle time is unknown (the sim does not report TCP speeds).

---

## 10. Files

| File | What |
|---|---|
| `SYSTEM.md` | this document |
| `spec.json` | numbers for the simulation (versioned, with a `changelog`) |
| `calc.py` → `calc_out.json` | mass/CoG, tip, e-stop, drive torque, step, power and energy |
| `docked_statics.py` → `docked_statics.json` | M1 statics re-run with the real base and recorded sim wrenches (numpy only; no MuJoCo run) |
| `bom.py` → `bom.json`, `bom_table.md` | BOM with sources |
| `make_spec.py` | builds `spec.json` from the above |
| `diagrams.py` → `block_diagram.png`, `power_flow.png`, `kitchen_topview.png` | diagrams |

---

## 11. Sources (accessed 2026-10-10)

**Arms**
- FR3 datasheet R02212 v2.6: https://franka.de/hubfs/Datasheet%20Franka%20Research%203_R02212_2.6_EN_July%202026.pdf
- FR3 joint limits: https://raw.githubusercontent.com/frankaemika/franka_description/main/robots/fr3/joint_limits.yaml
- Panda joint limits: https://raw.githubusercontent.com/frankaemika/franka_description/main/robots/fer/joint_limits.yaml
- FCI requirements: https://support.franka.de/docs/requirements.html and https://support.franka.de/docs/troubleshooting.html
- FR3 price (R): https://vention.io/parts/franka-robotics-fr3-4193
- Franka Hand (R): https://vention.io/parts/franka-robotics-fr3-franka-hand-4422
- Franka TMR / Mobile FR3 Duo prototype: https://franka.de/hubfs/Prototype%20Manual%20and%20Datasheets%20_Mobile%20FR3%20Duo_R04245_EN.pdf
- Kinova Gen3: https://www.kinovarobotics.com/uploads/Kinova_Onepager_Gen3_2026_EN.pdf, https://www.robotshop.com/products/kinova-gen3-robot-7dof-l53
- UR5e: https://www.universal-robots.com/manuals/latest/en/datasheets/ur5e/
- UR OEM DC box: https://www.universal-robots.com/manuals/EN/HTML/SW10_13/Content/prod_oem_polyx/oempsx_cb_all_specs_root.htm
- xArm 7: https://www.ufactory.cc/xarm-collaborative-robot/
- Kassow EDGE (R): https://www.albeco.com.pl/en/catalog/preview/Broszura_Kassow_Robots_Edge_EN.pdf

**Drive and base**
- Oriental BLV-R brochure: https://www.orientalmotor.com/products/pdfs/2018-2019/569%20BLV-R%20Brochure.pdf
- Oriental 200 W/50:1 set: https://catalog.orientalmotor.com/item/blv-r-series-brushless-dc-motor-speed-control/shop-blv-r-series-brushless-dc-motors/blmr6200skm-gfv-f-gfv6g50-blvd-krd
- maxon MW 500: https://www.maxongroup.com/medias/sys_master/root/8998075039774/Flyer-maxon-wheel-drive-system-EN-2022.pdf
- ZLTECH hub motors: https://www.robotmotor.com/zltech-6-5inch-24-48vdc-350w-wheel-hub-motor-for-robot-product/ and https://www.zlingkj.com/robot-hub-servo-motor-series/901228
- AgileX Tracer: https://global.agilex.ai/products/tracer
- Tente caster: https://cdn.tente.com/datasheets/1044UAP100R26_28S45RAL9002BH139_en.pdf
- RPLIDAR S2: https://bucket-download.slamtec.com/4a5f962943b5140f97ae3b7fab7ed9bdf2adee11/SLAMTEC_rplidar_datasheet_S2_v2.0_en.pdf
- SICK nanoScan3: https://www.sick.com/media/pdf/9/79/979/dataSheet_NANS3-AAAZ30AN1_1100333_en.pdf
- ASO ELMON relay: https://cdn.automationdirect.com/static/archive/specs/asosafrly-2025-04-25--11-09-27.pdf

**Power**
- Victron Phoenix: https://www.victronenergy.com/upload/documents/Datasheet-Phoenix-Inverter-VE.Direct-250VA-1200VA-EN.pdf
- Mean Well RSD-300: https://www.meanwell.com/Upload/PDF/RSD-300/RSD-300-SPEC.PDF
- Mean Well NPB-750: https://www.meanwell.com/Upload/PDF/NPB-750/NPB-750-SPEC.PDF
- Albright SW80 (R): https://ph.fridayparts.com/48v-100a-aftermarket-albright-sw80-10-single-acting-solenoid-contactor-for-industrial-automation-equipment
- Blue Sea Class T block: https://www.bluesea.com/products/5007100/Class_T_Fuse_Block_with_Insulating_Cover_-_110_to_200A
- IEC 62619: https://webstore.iec.ch/en/publication/64073
- RoboPad collector: https://uk.robotshop.com/products/robopad-extendable-collector-contacts-90mm-wide-100a
- Conductix AGV contacts: https://conductix.ch/en/node/13242
- Reelcraft cord reel (R): https://www.motion.com/products/sku/02971371
- SS 145-2 sockets: https://www.singaporestandardseshop.sg/Product/SSPdtDetail/9f82ea37-8284-4fe4-b19e-01ef95fba061

**Compute and sensors**
- Jetson Orin Nano Super: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/
- Jetson price rise: https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/
- OnLogic Helix 500: https://pdf.directindustry.com/pdf/onlogic/helix-500/240361-1044909.html, https://www.anandtech.com/show/16950/onlogic-helix-hx500-industrial-minipc-review-35w-comet-lake-goes-fanless/6
- RealSense D435i: https://www.realsenseai.com/products/depth-camera-d435i/
- RealSense spin-out: https://www.intelcapital.com/realsense-completes-spin-out-from-intel-raises-50-million-to-accelerate-ai-powered-vision-for-robotics-and-biometrics/
- Bota SensONE: https://uploads.unchainedrobotics.de/media/file_upload/BFT-SENS-ECAT-M8-SEN-datasheet_927296b3.pdf
- NUCLEO-H723ZG: https://www.st.com/en/evaluation-tools/nucleo-h723zg.html
- Advantech EKI-2725: https://uk.rs-online.com/web/p/network-hubs/2835848
- PILZ PNOZ s3: https://elektrobode.nl/products/pilz-pnozsigma-pnoz-s3-24vdc-relais-vr-veiligheidsstroomcircuits-750103
- FORT wireless e-stop: https://store.clearpathrobotics.com/products/wireless-e-stop
- Home RTT: https://aws.amazon.com/blogs/networking-and-content-delivery/measuring-network-latency-to-aws-region-before-deployment/

**Safety and touch**
- ISO/TS 15066 body table (reproduced): https://arxiv.org/pdf/2311.13814
- ISO 10218:2025 FAQ: https://www.automate.org/robotics/blogs/updated-iso-10218-faq
- T1 250 mm/s: https://dguv.de/fb-holzundmetall/sg/sg_maf/robotik/faq/index.jsp
- ISO 13482 scope: https://www.bsigroup.com/globalassets/localfiles/en-gb/consumer-guides/resources/bsi-consumer-brochure-personal-care-robots-uk-en.pdf
- Stop categories: https://www.controldesign.com/articles/2021/standards-and-options-for-e-stops/
- TI capacitive touch SLAY048: https://ti.com/lit/pdf/slay048
- Microchip AN2934: https://circuits.mit.edu/_static/S25/labs/rc/touchsensor_design.pdf
- Renesas layout guide: https://renesas.com/en/document/apn/capacitive-touch-hardware-design-and-layout-guidelines-synergy-rx200-and-rx100-application-note
- Holtek BS83B datasheet: https://www.endrich.com/Datenbl%C3%A4tter/Aktive%20Komponenten/BS83B08C_12C_16Cv151.pdf
- Grounded artificial finger patent: https://patents.google.com/patent/US9652077
- Stylus tips: https://www.newegg.com/p/0SA-00ZV-000U8
