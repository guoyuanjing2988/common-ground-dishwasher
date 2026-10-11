# multi_dish_suction: destacking with a Franka Cobot Pump (suction)

The Franka Panda carries a **Franka Cobot Pump** (Schmalz) on its flange instead of the Franka Hand. It takes two
nested bowls and then three stacked plates from piles on the counter and loads them into the countertop dishwasher
basket. Bowls go on their side, tilted, in the bowl section. Plates go upright between the plate tines.
Everything runs in one continuous `mj_step` simulation. Suction is a contact force capped at a realistic value.
There are no welds, no attached bodies and no teleports.

- Video: `out/suction_demo.mp4`. H.264, 1280×720, 30 fps, **51.1 s**, **5.1 MB**. The 128.6 s of simulation is played at ×2.5. Frames were streamed straight to ffmpeg.
- Key frames (8): `frames/00_start`, `01_bowl1_sealed`, `02_bowl1_place`, `04_bowl2_place`, `05_plate2_sealed`,
  `06_plate1_insert`, `10_plate3_insert`, `11_final` (`.png`). Four more frames are in `frames/extra/`.
- Raw results: `out/result.json`. Run log: `logs/full_video.log`.
- Peak memory: **674 MB RSS** for the video run and ~380 MB without video. Every run used `runw.sh` (3 GB watchdog, one process at a time).
- Wall time: 47 s with video and 15 s without.
- Reproduce: `./runw.sh logs/x.log suction_demo.py` (video) or add `--no-video`.

## Results (final poses measured from physics after everything settled, robot parked)

| Item | Target | Final pose error | Tilt / lean | At rest | Peak suction force (cap) | Peak lip contact force | Seal losses | OK |
|---|---|---|---|---|---|---|---|---|
| Bowl 1 (top of nest) | bowl spot 1 | 1.9 mm (origin) | axis 46.9° from vertical, opening azimuth 88° | 0.04 mm/s | 44.0 N (44.0) | 57.9 N | 0 | ✓ |
| Bowl 2 (bottom of nest) | bowl spot 2 | 9.2 mm | 37.1°, azimuth 81° | 0.01 mm/s | 44.0 N (44.0) | 57.7 N | 0 | ✓ |
| Plate 1 (top of pile) | slot 5 | rim-in-slot dx −0.8 mm, dy −4.1 mm, rim −0.2 mm vs wires | lean 10.2°, yaw 0.8° | 0.04 mm/s | 44.0 N (44.0) | 64.9 N | 0 | ✓ |
| Plate 2 | slot 6 | dx −2.1 mm, dy −3.9 mm, rim −0.2 mm | lean 9.1°, yaw 0.8° | 0.04 mm/s | 44.0 N (44.0) | 65.5 N | 0 | ✓ |
| Plate 3 (bottom of pile) | slot 7 | dx −3.7 mm, dy −4.7 mm, rim 0.0 mm | lean 7.6°, yaw 0.9° | 0.03 mm/s | 44.0 N (44.0) | 66.1 N | 0 | ✓ |

**5/5 placed. 0 seal losses. 0 unplanned robot contacts** (any robot or tool geom touching anything other than the
item under its cup). Maximum penetration was 0.87 mm and MuJoCo raised no warnings.

**Pass criteria.**
- Plate: the lowest point of the plate's mid-thickness circle is within ±12 mm of the slot centre line (slot pitch is 28 mm), |dy| < 20 mm, lean < 15°, yaw < 10°, the rim is on the wires, the plate is at rest (< 5 mm/s), and it is not touching the robot.
- Bowl: within 20 mm of the nominal rest pose, axis tilt 40° ± 10°, opening azimuth 90° ± 15°, at rest, and not touching the robot.

**Reading the force columns.**
- The suction force is the adhesion actuator force, `vacuum level × F_cap`. It saturates at the cap once the seal is made, so the peak always equals the cap.
- The lip contact force is the summed normal force on the 8 lip contacts. Its peak is cap + robot press-down during the pick (the cup is driven 3 mm past the surface).
- Payloads are a plate at 0.45 kg (4.4 N) and a bowl at 0.18 kg (1.8 N). The static safety factor is ~10× for a plate and ~25× for a bowl.

**Honesty tests** (same code; environment switches used only for these tests):

| Test | Result |
|---|---|
| `SD_FCAP=3` (cap 3 N, below the plate's 4.4 N weight) | seal made; the plate stays on the pile on lift; seal lost (lip gap 1.04 mm); `dropped` failure |
| `SD_PICK_TILT=10` (cup 10° off the plate normal) | no seal; pick fails |
| `SD_PICK_TILT=20` | no seal; pick fails |

Logs are `logs/test_fcap3.log`, `logs/test_tilt10.log` and `logs/test_tilt20.log`.

## Suction model (`suction_demo.py`, top of file)

| Parameter | Value | Source |
|---|---|---|
| End effector | Franka Cobot Pump: ISO 9409-1-A50 flange, powered through the flange, ≤ 750 mbar, 12 L/min, 810 g, no compressed air | product spec |
| Housing | cylinder Ø80 × 95 mm, 0.73 kg | EST |
| Stem / adapter | Ø20 × 35 mm. Lets the housing clear a bowl wall: 9 mm radial clearance at the inside-bottom pick | EST |
| Cup | Ø40 mm bellows, 18 mm high. Flange to lip = **148 mm** | EST (typical Schmalz Ø40 bellows cup) |
| Collision geometry | housing and stem cylinders, plus a lip ring of 8 compliant spheres (r = 3 mm, solref 0.015) | honest tool length; nothing passes through the basket |
| Working vacuum | 700 mbar (0.70 bar) | spec max 750 mbar |
| Theoretical force | area 1.257e-3 m² × 70 kPa = **88 N** | |
| Efficiency | **0.50**, for a curved/wet glazed surface, lip leakage and a safety margin | assumption |
| **Force cap F_cap** | **44 N** = adhesion actuator gain; ctrl ∈ [0, 1] is the vacuum level | |
| Vacuum build-up | first order, τ = 0.15 s once sealed; drops to 0 immediately when the seal breaks or the valve vents | EST |
| Lip friction | μ = 0.8 (rubber on glazed ceramic, dry) | EST |

**How the force acts.** The adhesion actuator (`trntype=body`, target `cup`) injects its force at the lip contacts,
along the contact normals, pulling the item onto the cup. Holding against gravity and inertia therefore comes from
two things only:
- the normal pull, minus the payload;
- the friction that this pull creates, which takes shear when a plate hangs upright.

No body is ever attached to the hand. Too weak a cap drops the item (see the honesty test).

**Seal check, evaluated every 2 ms physics step.**
1. **Acquisition.** All 8 lip spheres must be in contact (dist ≤ 0) with geoms of the *same* object. Every contact normal must be within **15°** of the cup axis. Only then does the vacuum start to build.
   - Because the modelled lip ring is rigid, a cup tilted by ≥ 10° cannot touch all 8 points. In practice this check is stricter than the 15° rule, and acquisition needs a near-normal approach. A real bellows cup conforms more.
2. **Maintenance.** While holding, every lip must stay within **1.0 mm** of the geom it sealed on (`mj_geomDistance`; models a bellows lip following small gaps), and the cup axis must stay within 15° of the surface normal (stored in the object frame at acquisition).
3. **Loss.** If maintenance fails, the vacuum goes to 0 at once, the force disappears, and the event is logged as a seal loss. The controller aborts the item when this happens during a carry.

## Strategy

**Plates.**
- Pick the top plate of the stack straight down, cup on the flat disc 40 mm off-centre. No push to the counter edge is needed.
- Rotate the plate upright, with the cup on the face high above the plate centre and the tool pointing −x into the basket.
- Lower the plate vertically into its slot, vent, back off 30 mm along the tool axis, then lift.

**Order and clearance (the sideways tool).** When a plate stands upright, the tool sticks out of its face toward the door. Three measures handle this:
1. **Fill back to front** (slots 5 → 6 → 7). The tool is always in front of the plate being placed, so it never passes over a plate already in the basket.
2. **Cup high on the face** (40 mm above the plate centre; the flat disc ends at r = 65 mm) and the plate held 3° leaning back. The tool axis rises toward the front, so the Ø80 housing passes over the basket front rail.
3. **Use the front-most slots.** The rail sits under the short housing rather than under the bulky wrist (link 7). `clear_test.py` sweeps tool roll (0–315°) and lean (8°/12°) against the basket geometry. With the original pull bar, every combination collided with it (−8 to −30 mm). That is why the handle change listed below was made. After the change, a 90° tool roll clears the rail.
- **Insert aim:** the plate's mid-thickness at its lowest point is aimed 8 mm toward the back tine. This follows the gpt_engineer p2_lean finding (a larger aim offset reduces the seated lean). With a 0 mm aim the plates slid and settled at ~20° lean; with 8 mm they settle at 7.6–10.2°.

**Bowls (placed first).**
- Pick the top bowl of the nest by its inside bottom, cup pointing straight down into the bowl.
- Carry it and tilt it 40°, opening toward the (still empty) plate section and up. Set it down on the floor wires against the bowl prop wire, then vent.
- Retreat along the tool axis, then away from the dishwasher face before rising (this avoids the control band).
- Bowls go first because, in this pose, the tool leans over the plate section. Once plates are in, that space is taken.
- An **inverted** bowl cannot be placed this way: the cup is inside the bowl and would be trapped under it. Tilted on its side is the honest option for an inside-bottom suction grasp.

**Robot.**
- M1 docked floor cart (`scene.py`, copied). The base is at (0.55, 0.30, 0.90), yaw 180°.
- That is 0.10 m further out than full_pipeline's default (0.45, 0.30, 0.90), because at 0.45 the horizontal-tool insert poses fold joint 4 to −3.0 rad and joint 6 to 3.6 rad. There, joint 5/6 hit their 12 N·m torque limit and the tool sagged into the basket rail.
- Motion: mink IK (daqp) on a robot-only kinematic model. Cartesian lines are sampled every 1 cm. Joint moves use a min-jerk time law. The joints are menagerie position servos with gravity compensation, which stands in for Franka's payload model.

## Simplifications (stated honestly)

- **Ground-truth object poses** instead of perception, for the pile tops and the basket position.
- **Door already open, basket already out** (0.33 m travel, as in multi_dish_b). No pod and no buttons.
- **Small piles:** 3 plates and 2 nested bowls (primitives from `poc_plate_flat` and `poc_bowl_grasp`).
- **Basket changes in my copy of `dishwasher.py` (documented in the file):**
  1. The 35° leaning bowl row is replaced by a low **bowl prop wire** along x (y = −0.101, 12 mm above the floor wires).
  2. The **basket pull bar** is lowered from 40 mm to 12 mm above the top rail and moved from the centre to in front of the bowl section (y = −0.11). As modelled originally (an EST, not a measured part), it blocked the horizontal suction tool at the plate section.
- **Sped-up video** (×2.5).
- **Plates and bowls are rigid primitives.** No water, food residue or wetness; the cup friction is the dry value.
- **The pump's own dynamics are not modelled:** valve timing is collapsed into τ = 0.15 s, and there is no vacuum sensor noise. The seal decision uses simulator geometry (contacts and distances), which stands in for the pump's vacuum-level sensor.
- **Cup geometry is an estimate:** the 8 rigid lip spheres approximate a soft bellows lip. Pump housing dimensions are also estimates.
- **Bowl rest poses differ** (46.9° vs 37.1°) because each bowl settles against the prop on its own. Both are within the tolerance.

## Suction vs parallel gripper for this task

**Pros of suction**
- **Destacking is direct.** The top plate of a stack and the top bowl of a nest are picked straight down by the face or inside bottom. multi_dish_b and full_pipeline instead push each flat plate ~10 cm to the counter edge, pinch the overhang, then tilt. Here each plate pick takes about 5 s from above the pile to lifted, with no push and no edge-overhang perception.
- **Bowls become feasible.** The GPT_SORT study found 0 of 33 bowl/mug grasp poses feasible with the bulky parallel gripper (rim pinches, upside down or leaning, in B1–B3). The inside-bottom suction pick plus a tilted placement loaded 2/2 bowls here.
- **One grasp type covers flat, curved and nested items.** There is no finger clearance to plan around the tines, and no grasp-force regulation.

**Cons and real-world caveats**
- **The tool sticks out of the face.** Upright plate insertion forces a horizontal tool, and that tool has to clear the basket front rail and handle. This works only in front slots, back-to-front, with the cup high on the face and the basket handle out of the way. In a full-size dishwasher with tall front tines, rear slots would be hard to reach; a gripper inserts from above without this problem.
- **Inverted bowls are impossible** with a cup inside the bowl. A regrasp (set down, pick from the outside bottom) or a second cup orientation would be needed. Bowls end up tilted with the opening up, which collects water. That is acceptable for the basket model but not ideal for drying.
- **Food residue on the cup lip or the dish face** (grease, rice grains, sauce) breaks the seal or contaminates the next item. Pre-rinsed dishes are realistic for loading; scraped-only dishes are not. The cup also needs periodic cleaning; a silicone or food-grade NBR lip is advisable.
- **Wet surfaces lower friction** (μ maybe 0.3–0.5). Upright plates hang in shear, so the margin falls from ~10× to ~3–5×. It is still adequate for 0.45 kg plates, not for heavy stoneware platters.
- **Textured, porous or unglazed areas leak.** Unglazed foot rings, matte or porous stoneware, crazed glaze, embossed rims, and wicker or wood items leak or never seal. The 12 L/min pump compensates for small leaks but not porous bisque. Our model assumes a smooth glazed face.
- **Small flat area.** A Ø40 cup needs about 45 mm of flat or gently curved surface. Saucers with a raised centre, deep soup plates, and plates with heavy decoration may offer no valid spot. The seal angle (≤ 15°) also limits picks on curved bowl bottoms to the flat centre.
- **Holding force is modest** (44 N after the 50% derating) compared with a gripper's ~70 N clamp. It is plenty for tableware, but a jolt or collision during transport peels the cup. A seal loss is immediate and complete, which is why the controller aborts on loss.
- **Practical upside:** no compressed air is needed and the Cobot Pump runs from the flange, so it is a drop-in on the same docked cart.

**Bottom line.** For this destacking task, suction is the better end effector for *picking*: plates and bowls straight off a pile, and bowls the gripper could not grasp at all. *Placement* is its weak point. The tool's sideways protrusion constrains slot order and reachable slots, and it rules out inverted bowls. A tool changer, or a cup on a short swivel, would remove most of those limits.

## Files

| File | Purpose |
|---|---|
| `suction_demo.py` | scene build (copied kitchen/cart + Panda without hand + Cobot Pump), suction model, IK, controller, checks, HUD and ffmpeg streaming |
| `scene.py` | copied from multi_dish_b/full_pipeline (kitchen, M1 cart, plate primitives); one fix: unique freejoint names |
| `dishwasher.py` | copied from full_pipeline; travel 0.33, bowl prop wire, relocated low pull bar (all marked `multi_dish_suction`) |
| `runw.sh` | watchdog (from gpt_control): RSS of the process tree, kill above 3 GB, exact max RSS via `/usr/bin/time -l` |
| `clear_test.py`, `reach_test.py`, `dbg_*.py` | clearance and reachability sweeps used to choose the tool roll, lean, slots and base pose |

---

# Cluttered-counter version: GPT recognition + depth perception (`kitchen_demo.py`)

- Video: `out/kitchen_suction_demo.mp4`. H.264, 1280×720, 30 fps, **65.5 s**, **5.6 MB**, frames streamed to ffmpeg.
- Sequence:
  1. Opening on the cluttered counter.
  2. Recognition card: GPT's own boxes (green = load, red = leave), EN + 中文 labels and reasons, and a ">> GPT call took 38 s, fast-forwarded" banner.
  3. Loading, with the caption naming the pose source and its error for every pick, and the checklist.
  4. Checklist overlay.
  5. End card: the loaded basket, plus what was left on the counter and why.
- Key frames (8): `frames/kitchen/00_k00_opening`, `k01_recognition`, `01_bowl1_sealed`, `02_bowl1_place`, `05_plate2_sealed`,
  `06_plate1_insert`, `11_k98_checklist`, `k99_end`. More are in `frames/kitchen/extra/`.
- Results: `out/kitchen/result.json`. GPT output: `out/kitchen/plan.json`. Inputs: `out/kitchen/wall.png`, `depth.npy`.
- Peak memory: 896 MB (`runw.sh`, one sim at a time). Wall time 65 s. Re-rendered at 12:51 SGT; the video is 65.5 s and 5.6 MB.

**Pipeline.** Each stage is a separate process, and only one sim runs at a time.
1. `capture`: settle the scene (sim), then save the back-wall RGB + depth (1280×720, fovy 60°, depth noise 1.5 mm/m as in full_pipeline).
2. `plan`: one `codex exec --skip-git-repo-check --ephemeral --sandbox read-only --output-schema … --image=wall.png` call. stdin is `/dev/null`, it runs on the ChatGPT subscription, and no API keys are used. Took 38 s and 17.2k tokens.
3. `run`: the video.
   - Before **every** pick, the wall camera re-captures depth.
   - Points inside GPT's box for the pile are back-projected. This uses the full_pipeline `perception.py` method, copied: RANSAC counter plane on a ring around the box, outline circle fit.
   - Height: the plate-stack cup point is the 85th percentile height on the disc (r < 45 mm). The bowl cup point is the 15th percentile height of the inside bottom, seen through the opening.
   - A perceived pose is accepted if it is within 12 mm (xy) and 6 mm (z) of truth; otherwise it falls back to labelled GT. **No fallback was needed.**

**Clutter.** Eight items, all left untouched:
- From `gpt_sort/items.py` (copied as `kitchen_items.py`): chef's knife, wooden cutting board, frying pan, vacuum flask, mug.
- Added as primitives in `kitchen.py` (EST dimensions): dish-soap bottle, sponge, oil bottle.
- All of them sit behind or beside the piles, outside the arm's sweep.
- Measured over the whole run, from scene start: the largest displacement is **0.85 mm** (initial settling of the pan) and the largest tilt change is **0.25°**. **0 robot contacts with any clutter item. Nothing knocked over.**

## Results (all picks used PERCEIVED poses)

| Item | Target | Perceived vs true cup point (xy / z) | Final pose | Peak suction (cap) | Seal losses | OK |
|---|---|---|---|---|---|---|
| Bowl 1 (top of nest) | bowl spot 1 | 0.3 mm / −1.0 mm | 3.8 mm, tilt 46.6° | 44 N (44) | 0 | ✓ |
| Bowl 2 | bowl spot 2 | 0.3 mm / −1.0 mm | 4.7 mm, tilt 46.6° | 44 N (44) | 0 | ✓ |
| Plate 1 (top) | slot 5 | 1.3 mm / +1.0 mm | dx −1.2 mm, lean 9.9° | 44 N (44) | 0 | ✓ |
| Plate 2 | slot 6 | 1.0 mm / +1.0 mm | dx −3.4 mm, lean 7.8° | 44 N (44) | 0 | ✓ |
| Plate 3 | slot 7 | 1.1 mm / +1.0 mm | dx −5.1 mm, lean 6.3° | 44 N (44) | 0 | ✓ |

The fitted rim radius was 101.6–102.6 mm for plates (true 101.5) and 57.0–57.2 mm for bowls (true 57.5).
Perception errors are tiny compared with what suction tolerates: the cup needs ~45 mm of flat face, and the descent is guarded (it stops when the seal is made).

## GPT recognition: decisions vs truth

| # | GPT label (EN / 中文) | Truth | Dishwasher-safe | GPT target | Correct? |
|---|---|---|---|---|---|
| 1 | Kitchen knife / 菜刀 | chef's knife | no | leave: hand-wash | ✓ |
| 2 | Insulated flask / 保温瓶 | vacuum flask | no | leave: hand-wash | ✓ |
| 3 | Cleaning sponge / 清洁海绵 | sponge | no | leave: not a dish | ✓ |
| 4 | Bowl / 碗 (count 1) | **2 nested bowls** | yes | bowl area | ✓ decision. ✗ count: the nest looks like one bowl from above. The robot re-perceives after each pick, so it still found and loaded both. |
| 5 | Stack of white plates ×3 / 白色盘子叠 | 3 plates | yes | plate rack | ✓ |
| 6 | Red mug / 红色马克杯 | mug | yes | bowl area | ✓ decision, but **not executable by this tool** (below) |
| 7 | Frying pan / 平底煎锅 ("apparent nonstick") | cast-iron pan | no | leave: hand-wash | ✓ decision. Material guess wrong. |
| 8 | Green capped carton / 绿色带盖纸盒 ("food packaging") | **dish-soap bottle** | no | leave: not a dish | ✓ decision. ✗ label. |
| 9 | Yellow bottle / 黄色瓶子 | oil bottle | no | leave: not a dish | ✓ (generic label) |
| 10 | Wooden cutting board / 木制砧板 | cutting board | no | leave: hand-wash | ✓ |
| 11–12 | Food pieces in the pan ×8 / on the board ×7 | visual soil decals | no | leave | extra entries (soil listed as items) |

Every real object got the right load/leave decision. Two labels were wrong (soap bottle called a carton, cast iron called nonstick). One count was wrong (the bowl nest). The food decals were listed as items.

**Mug.** GPT correctly marks it dishwasher-safe → bowl area. The suction tool cannot load it: the Ø80 mm pump housing does not fit the Ø72 mm mug opening, so the 53 mm stem + cup cannot reach the inside bottom 88 mm down, and the outside is curved and has a handle. It is left on the counter and flagged on the end card as "hand to a person". This is a concrete instance of the suction caveat above.

**Ground truth is not used for control in the kitchen run** (fixed after a review at ~12:50 SGT).
- The first render's caption at the pile approach still said "(ground-truth pose)". That was stale text: the approach already targeted the perceived cup point.
- However, two hidden GT uses were found in `suction_pick`, and both are now removed for the kitchen run:
  1. The object-in-cup transform used to compute every placement pose was taken from the simulator's object pose after the lift. It now comes from the **perceived** object pose at the moment of sealing (plate origin = perceived disc top − 10 mm; bowl origin = perceived inside bottom − 4 mm; yaw is irrelevant for round items).
  2. The "did it lift" check used the simulator's object height. It now uses the **vacuum seal state**, standing in for the pump's vacuum sensor.
- The video was re-rendered: still 5/5 OK; final poses changed slightly, and the table above shows the re-rendered values. Captions now say "(perceived pose)". The plain `suction_demo.py` run (GT poses, as stated in its own section) is unchanged.
- The seal decision itself still uses simulator contacts and distances as a stand-in for the pump's vacuum sensor (see the suction model above).

**Additional simplifications in this version.**
- Perception and GPT use rendered images, not a real camera.
- The camera pose is known.
- GPT's boxes for the plate stack and bowl nest come from the first image and are re-used for every later depth capture (the piles do not move).
- The "accept if within 12 mm / 6 mm of truth" gate uses ground truth only as a safety and scoring check. It never triggered.

---

# Messy-sink attempt (`sink.py`): status at 13:32 SGT, INCOMPLETE

**Built and working**
- A sink scene matching `sink_refs/sink_6.png`:
  - basin (−0.30…−0.05, 0.33…0.68; 100 mm deep) cut into the counter;
  - faucet, collidable splash-back, soap, sponge, checked-colour dish cloth, cup with chopsticks, a cutting board leaning on the wall, and the earlier clutter;
  - 4 plates in a loose, offset stack leaning sideways, with 2 bowls wedged against the wall. Seeded like that, then settled by physics; all 6 stay in the sink, at < 0.4 mm/s after 8 s.
- **Robot base moved** to (0.45, 0.48, 0.90) so the sink is in reach (`sink_reach.py`).
- **GPT pick order** (`codex exec`, 53 s): it found all 6 dishes and ordered them top plate → blue bowl → orange bowl → plates 2–4. The board and oil bottle were misread as one "long utensil".
- **Depth patch finder:** finds the flat disc of the top plate at the true 23° tilt, and the bowls' inside bottoms.
- **In-hand classification:** after a pick, the item is shown to the wall camera and its rim circle is fitted: 97–99 mm → plate, 56.7 mm → bowl.

**Not yet working: holding through the lift**
- Seals on tilted dishes in the pile are acquired, then lost during the lift, with a 1.5 mm lip gap at 3.5–5°.
- The modelled cup is a rigid ring of 8 lip points. On a tilted, loose plate it seals only partially (6/8 lips), and with the 44 N cap the seal then peels.
- In one run (13:25) a plate reached the basket but was not seated correctly. In another, a dropped plate fell to the floor; that run's checks detected and reported it.
- A spring-loaded cup gimbal (bellows compliance) was tried and reverted: it did not fix the peeling.

**Blocked at 13:31.** The shared interpreter `common_ground_dishwasher/.venv` and `mujoco_menagerie/` (Panda model) were deleted by another process. That folder became a git repo, and both are git-ignored. No other Python with mujoco or mink exists on the machine, and installs are forbidden, so no further sims can run. The last `out/sink_suction_demo.mp4` (13:30) shows only failed and dropped picks. It is NOT a deliverable.
