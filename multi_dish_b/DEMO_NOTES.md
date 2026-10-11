# multi_dish_b: Franka loads a countertop dishwasher (MuJoCo)

## Sink, full job from a CLOSED dishwasher: `out/sink_full_open_demo.mp4`

**Start state: the door is closed and the basket is in.** The full sequence is:
1. open_door (full_pipeline: follow the hinge arc), reaching 90.02°; peak finger force 11.4 N;
2. pull_basket out to 0.33 m;
3. the 5-dish sink loop: 3 leaning plates and 2 inverted bowls, all loaded;
4. push basket in (0.0 mm);
5. close door to −0.009°, with the `door_closed` sensor true;
6. press QUICK, then POWER. Touch-pad peaks are 4.59 N and 4.84 N. The panel shows RUNNING (QUICK).

The layout and seed are the same as `sink_full` (seed 3). **GPT outputs are replayed from the logged run.** The first-look answer from `result_sink_full.json` is saved as `out/sink/replay_sink_full_look1.json` and reused for every look, filtered by the items already attempted. No codex call was made in this run, and the on-screen banners say "GPT output replayed from the logged run". The camera image is not exactly the same, because the robot arm sits elsewhere after opening the door; the dishes in the sink are unchanged.

Poses: perceived poses (depth re-captured before every pick), the same GT audit as below. Perceived pose errors were 1.4–2.7 mm.

| Item | Target | Final result |
|---|---|---|
| plate 0 | slot 6 | lowest rim +2.2 mm, lean 2.0° |
| plate 1 | slot 5 | +2.1 mm, 2.1° |
| plate 2 | slot 4 | +2.8 mm, 1.35° |
| bowl 0 | bowl spot 1 | foot-ring error 8.8 mm, tilt 16.8° |
| bowl 1 | bowl spot 2 | 4.5 mm, 18.3° |

All rows OK. No dish left the sink unnoticed and the counter items did not move. No unplanned robot contact above 2 N, and 0 warnings.

- **Pod checks:** the pod checks in pull_basket and push_basket are "not applicable" (this scene has no pod).
- **Run:** `sink_demo.py --full --closed --seed 3 --cache out/sink/replay_sink_full_look1.json --replay out/sink/replay_sink_full_look1.json`.
- **Data:** `out/result_sink_full_open.json`; key frames in `keyframes_sink_full_open/`.

---

## Sink, fanned plate STACK attempt (sink_6.png look): `out/sink_stack_demo.mp4` (honest failure)

**Result: 1 of 5 dishes loaded. The finishing steps were not completed.** Only one inverted bowl reached the basket, by the foot-ring grasp.
- **Stack:** layout "stack", seed 4, run `sink_demo.py --full --stack --seed 4`. 3 plates in a fanned stack along the front wall, each lying on the one below, 60 mm sideways and 21 mm outward. They settled at leans of 54°, 41° and 34° from vertical, with 2 upside-down bowls beside them.
- **Seed 3 collapsed:** the stack fell nearly flat (11–23° from horizontal) into an ungraspable pile.
- **Plates:** the rim pinch on the stacked plates failed. The fingers made no contact at the rim point 25° from the top toward the exposed side; the cause has not been investigated. One pinch had no joint path at all. A plate was disturbed during the attempts (max penetration 47 mm during recovery).
- **Bowls:** one bowl was loaded (foot error 1.8 mm at placement). The bowl next to the side wall y = 0.95 failed: with the fingers closing along y, the hand most likely hits that wall.
- **Basket push:** the basket was pushed fully in (0.0 mm), but the step's plate-in-slot check failed because no plate had been loaded, so the run stopped there. The door was not closed and no button was pressed.
- **End card bug:** it marks the loaded bowl ✗ because the bowl target was still referenced to the pulled-out basket after the failed push step (a scoring bookkeeping bug). The bowl is in the basket, on the bowl tines.
- **Ran out of time:** I did not iterate further before the 14:50 hard stop.

**Delivered successful version:** use `out/sink_full_demo.mp4` (separate leaning plates, all 5 dishes loaded, basket pushed, door closed, QUICK + POWER pressed, panel RUNNING).

---

## Sink, full job: `out/sink_full_demo.mp4`

**All 5 dishes go from the messy sink into the dishwasher. The robot then pushes the basket in, closes the door and presses QUICK, then POWER. The panel ends showing RUNNING.**

All 5 grasps use perceived poses: GPT box plus depth. GPT looked before every pick: 5 looks of 21.6–37.9 s, fast-forwarded on screen. GT fallback never fired. The final poses below were measured after the door was closed and the buttons pressed.

**This layout has NO flat plate and NO upright bowl. This is stated on screen too.** Every dish is placed so that the proven skills can grasp it:
- 3 plates slid down against the front wall, their rims up over its edge, at random yaw and spacing;
- 2 bowls upside down, wedged behind them.

The scene is seeded and settled by physics. The flat-plate and upright-bowl skills are being attempted separately by another agent. The mixed layout with those two items is the previous section (`out/sink_demo.mp4`, 4 of 6 loaded).

**Files**
- Video: `out/sink_full_demo.mp4`. H.264 yuv420p, 1280×720, **135.3 s, 37.3 MB**. Written as `.tmp.mp4`, then moved into place.
- Data: `out/result_sink_full.json`.
- Key frames: `keyframes_sink_full/`.
- Run: `./runw.sh logs/x.log sink_demo.py --full --tag _x`. Wall time about 300 s, peak RSS 1.07 GB.

| Look | GPT item (EN / 中文) | Perceived pose error | Pose used | Final, after door closed + program started |
|---|---|---|---|---|
| 1 | Right blue bowl / 右侧蓝碗 | 1.80 mm | perceived | ✓ bowl spot 1: foot error 13.6 mm, tilt 13.8° from inverted |
| 2 | Blue bowl / 蓝色碗 | 1.45 mm | perceived | ✓ bowl spot 2: foot error 4.4 mm, tilt 18.3° |
| 3 | Light blue plate / 浅蓝色盘子 | 2.69 mm | perceived | ✓ slot 6: lowest rim +0.8 mm, lean 3.3° |
| 4 | White plate / 白色盘子 | 2.24 mm | perceived | ✓ slot 5: lowest rim +2.7 mm, lean 1.6° |
| 5 | Cream plate / 米色盘子 | 2.45 mm | perceived | ✓ slot 4: lowest rim +2.8 mm, lean 1.4° |

The foot error of bowl 1 was 8.0 mm right after placement. The basket push moved it to 13.6 mm, still under the 15 mm limit.

**Finishing steps.** These are the full_pipeline skills, unchanged. The rail setpoints are the M2 ones.

| Step | Result |
|---|---|
| Push basket in | basket at 0.0 mm |
| Close door | −0.009°; `door_closed` sensor true |
| Press QUICK, then POWER | wrist F/T stop at 3 N; touch-pad peaks 4.59 N / 4.84 N (threshold 1 N) |

**Panel:** RUNNING (QUICK).

**Physics and contacts**
- No dish left the sink unnoticed.
- The 8 counter items stayed put (≤ 0.85 mm).
- No unplanned robot contact above 2 N.
- MuJoCo warnings: 0.
- Max penetration: 5.6 mm, plate on basin wall during the initial drop.

**Honest notes**
- **No real panel display:** the dishwasher model has no display. "RUNNING" is shown in the caption bar, and the QUICK and POWER pads are recoloured green. That is a visual change only, made after both touch pads registered more than 1 N.
- **Pod check skipped:** the push-basket step's pod-in-groove check does not apply, because this scene has no pod. It is returned as "not applicable".
- **Bowl targets after the push:** they now refer to the pushed-in basket (known back stop). The first take scored them against the pulled-out basket and wrongly showed ✗. That take is kept in `out/dev/`.
- **Slip gate:** the pinch-lift slip gate is 30 mm, as in the sink section. Actual slip: 5.5–7.5 mm.
- **GT audit:** the same as the sink section below.
  - Identity matching, scoring and the fallback switch read true poses; the fallback switch never fired.
  - The motion uses only GPT boxes, depth, encoders, wrist F/T and the known dishwasher model.
  - The finishing steps use the known dishwasher model plus wrist F/T, exactly as in full_pipeline.

---

## Messy sink (latest): `out/sink_demo.mp4`

**4 of 6 dishes are loaded; 2 are handed to a person.** All 4 grasps use perceived poses: GPT boxes plus depth. Final poses were measured after everything settled:
- 2 plates: slot 6 and slot 5;
- 2 bowls, upside down and tilted: bowl spots 1 and 2.

The 2 handed over are an **upright bowl sitting on a plate** (opening up; flipping it needs a regrasp this gripper cannot do) and the **plate under it**, which lies flat on the basin floor with no exposed rim to pinch. Nothing left the sink unnoticed: no dish ended outside the basin unless it was loaded. The 8 counter items stayed put (≤ 0.85 mm).

**Video:** `out/sink_demo.mp4`. H.264 yuv420p, 1280×720, **114.6 s, 26.5 MB**. It was written as `.tmp.mp4`, then moved into place. The video shows:
1. The dump: 6 dishes dropped into the sink, settling under `mj_step`, at real-time speed.
2. A wide shot.
3. The "GPT is recognising" banner (59.0 s, fast-forwarded) and the recognition card. Boxes are numbered with the GPT pick order; the legend gives EN + 中文 labels and poses; the not-loaded list gives reasons.
4. A short **"Look #n"** card before every next pick. This is a new image and a **new GPT call** on what is left (28–44 s each, fast-forwarded): 7 looks in total.
5. The picks and placements, with the caption `Pick #k <label> → slot/spot [perceived pose]`.
6. Handover captions.
7. The loaded-basket close-up and an end card: ✓/✗ list plus "Left on the counter / for a person" with the reasons.

Key frames are in `keyframes_sink/`. The images and answers of every GPT look are in `out/sink/`. All data is in `out/result_sink.json`. A rough first take is kept in `out/dev/sink_demo_rough_v1.mp4`: its bowl-orientation test wrongly handed over both upside-down bowls (2 of 6 loaded).

**Run:** `./runw.sh logs/sink.log sink_demo.py --tag _x` → `out/sink_demo_x.tmp.mp4`. Wall time is about 390 s, including 7 GPT calls. Peak RSS: 1.06 GB.

### Scene (`sink_scene.py`)
The sink6 reference look is approximated where it stays graspable.
- **Basin:** a recessed steel basin cut into the counter right of the dishwasher. Interior x −0.36…−0.06, y 0.30…0.95; floor 12 cm below the counter; 4 walls and a drain. The counter and the cabinet are split around it.
- **Fixtures and clutter:** a faucet with spout and lever, soap bottle, sponge, checked dish cloth, a cutting board against the wall, and a cup with chopsticks. The left-side clutter from the kitchen version is kept.
- **Dishes:** dropped from perturbed poses (seed 3) and settled by physics:
  - 2 plates slid down against the front wall, rims sticking up over its edge;
  - a 3rd plate flat at the back with an upright bowl tilted on it;
  - 2 bowls upside down, wedged between the plates and the back wall.
- **Not the sink6 fanned stack.** I tried a fanned, nested 4-plate stack. It settled into nested plates whose rims had no room for a finger, and every grasp failed. So the final layout is less chaotic than sink6. I did not use `recognition_card/sink_scene.py` sink6, for the same reason and for lack of time.
- **Overhead camera:** `sinkcam` is mounted under the wall cabinet behind the sink, looking down into the basin, so it sees the faces of the leaning plates. Depth noise σ = 1.5 mm × range.
- **Mount:** the same cart and 1.10 m linear axis. The base follows the perceived item y for picks and returns to y = 0.36 for loading.

### Loop (`sink_demo.py`)
1. **GPT, first look.** `codex exec` (read-only, stdin /dev/null) receives the sinkcam image only. It returns per item: box, EN/中文 label, in-sink, dishwasher-safe + reason, pose, and **pick order** (top-most / least-blocked first).
2. **Pick the first remaining item** in GPT order. Items already attempted are skipped; they are identified by the box centre projected onto the basin, within 6 cm.
3. **Re-capture depth right before the pick**, then:
   - **Plate:** RANSAC plane on the depth points in the GPT box. A plate within 25° of horizontal is flat → handover. Otherwise:
     - in-plane up axis;
     - top rim point = 99.5th percentile along that axis;
     - centre = top − R;
     - concave side from the rim-vs-disc offset;
     - origin = visible-face offset.
     Then **pinch the top rim** (pipeline `pick_rack`: pads on the rim face, approach down along the plate plane) and lift 0.17 m along the plane. Carry and insert use the unchanged `step_insert` with the measured-at-grasp in-hand transform; touchdown is detected by the wrist F/T.
   - **Bowl:** the top 6 mm band of the depth points within 6.5 cm of the box centre. Upside down = a compact top (foot ring, mean radius < 34 mm) and a centre less than 25 mm below the top. An upright bowl shows its cavity, about 40 mm deep → handover. For an upside-down bowl: foot-ring grasp with the fingers along y (narrow in x, clear of the basin walls), then the hand turns about the bowl axis to the placement frame. Placement is as before.
4. **Look again:** re-capture. If the depth image still shows something above the basin floor, call GPT again on the new image, show the "Look #n" card, and continue. The loop stops when GPT lists no new pickable dish.

### Results (`out/result_sink.json`)
**Recognition.** GPT found all 6 dishes and gave sensible safe / not-safe calls for the rest. Mistakes:
- it reported a "possible pan beneath the white plate" (most likely the dark drain);
- it called a blue cloth a "cutting board";
- the pick order changed between looks: look 1 put the flat plate 2nd; look 2 put it last.

| Look | GPT item (pick # in that look) | Perceived pose error | Pose used | Outcome (final pose after everything settled) |
|---|---|---|---|---|
| 1 | Upright blue bowl 正放蓝碗, #1 | (blob mis-centred, 73 mm) | – | **handed to a person**: opening up on a plate, needs a flip |
| 2 | Beige plate 米色盘子, #1 | 2.87 mm | perceived | ✓ slot 6: lowest rim +1.7 mm, lean 2.5° |
| 3 | Blue inverted bowl 倒扣蓝碗, #2 (#1 already handed over) | 1.71 mm | perceived | ✓ bowl spot 1: foot error 4.9 mm, tilt 17.9° |
| 4 | Inverted blue bowl, #2 | 1.82 mm | perceived | ✓ bowl spot 2: foot error 2.5 mm, tilt 19.0° |
| 5 | White plate 白色盘子, #1 | 2.40 mm | perceived | ✓ slot 5: lowest rim +3.0 mm, lean 1.3° |
| 6 | White plate (flat, under the bowl), #2 | – | – | **handed to a person**: flat on the basin floor, no exposed rim |

**Measurements**
- Plate lift slip: 2.0 / 4.1 mm. Bowl lift slip: 0.35 / 0.34 mm.
- No unplanned robot contact above 2 N.

**Physics warning from the dump:** max penetration 24 mm, between the faucet base and the sponge. The sponge was seeded overlapping the faucet base and was pushed out at t = 0. This is a seeding error, not a robot contact.

### Ground-truth audit (sink)

**Controller inputs (drive motion): perceived or sensed**
- The GPT boxes.
- Depth (re-captured before every pick) → plate plane and rim, bowl foot ring and orientation class.
- Plate in-hand transform: encoders relative to the perceived pose. Bowl in-hand offset: perceived grasp point plus encoders.
- Wrist-F/T touchdown.
- Basket and slot targets: the known dishwasher model + the known travel.
- The look-again stop test: depth above the basin floor.

**Ground truth used for decisions (stated here)**
- *Identity:* the perceived item is named by its nearest simulator body. This is for the table and the slot bookkeeping only; the robot moves to the perceived pose.
- *GT-fallback switch:* error > 6 mm → use the true pose (labelled on screen). It **did not fire** for any loaded item.
- *Step success predicates:* lift rise / slip / finger contacts, `plate_slot_state` and `bowl_check`. They only abort an item; on failure the hand opens and the item is reported as "grasp failed, left in the sink for a person". No item failed in the final run.

**Scoring / HUD only:** slip tracking, the final table, the dishes-left-the-sink check, the clutter-moved check.

**Reset / seeding only:** dish drop poses and arm / door / basket at t = 0. After that, only `mj_step` moves anything.

### Sink simplifications (honest)
- **Layout:** chosen so rims are exposed: 2 plates leaning on the wall edge. The fully nested sink6 stack was not graspable with this gripper in the time available. The pile is seeded and then settled by physics, not a random dump.
- **GPT:** sees clean renders. Its latency (28–59 s per look) is real but fast-forwarded in the video.
- **Handover:** the 2 handed-over items are reported, not actually handed to a simulated person.
- **Pinch-lift slip gate:** 30 mm (rack PoC: 5 mm), because the plate slides along the wall edge while lifting. The actual slips were 2–4 mm.
- **Earlier simplifications still in force** (sections below): door open, basket out 0.33 m, the 1.10 m linear axis, the 10 mm insert aim, 3× video speed.
- **Shared-environment outage:** the shared venv was deleted around 13:30 and restored at 13:36. `kitchen.py` and `pipeline.py` now fall back to a pure-python blob labeller and any available QP solver if scipy or daqp are missing; neither fallback was used in the reported runs.

---

## Kitchen upgrade (final submission video): `out/kitchen_demo.mp4`

**5/5 dishes are placed correctly, and every grasp uses a PERCEIVED pose.** No GT fallback was needed. The poses come from a live GPT recognition call (`codex exec`) plus depth back-projection. All 8 clutter items stayed where they were: at most 0.85 mm of drift, at most 0.25° of tilt.

- **Video:** `out/kitchen_demo.mp4`. H.264 yuv420p, 1280×720, 30 fps, **125.9 s, 39.5 MB**. It shows 331 s of simulation at 3× speed. Frames were streamed to ffmpeg (imageio-ffmpeg / libx264) during the run.
- **Video structure:**
  1. Opening: wide shot of the cluttered counter.
  2. Overhead camera image with the banner "GPT is recognising … 29.3 s, fast-forwarded". The simulation is paused during the call, which is shown as a still for 2.5 s.
  3. Recognition card, 8 s: numbered boxes. Green = load, with its slot; amber = dishwasher-safe but not in this load; red = hand-wash. Two legend panels give English + 中文 labels, the slot or the reason.
  4. Pose card, 5 s: the depth-fitted centres and the perceived-vs-true error per dish.
  5. Loading: the caption shows `Plate k/3 → slot n [perceived pose (GPT box + depth)]` plus the current step, with a running checklist.
  6. Loaded-basket close-up.
  7. End card: ✓ list with the measured errors, plus "Left on the counter (hand-wash / not this load) 留在台面" with GPT's reasons.
- **Key frames:** `keyframes_kitchen/`. The cards are also in `out/recog/` (`counter.png` is the image GPT saw; `card.png`, `card_poses.png`).
- **Data:** `out/result_kitchen.json` (everything) and `out/recog/recognition.json` (GPT answer, matches, errors).
- **Run:** `./runw.sh logs/k.log multi.py --tag _k --frames frames_kitchen`. The run makes one live GPT call. `--cache <gpt json>` reuses a saved answer and was used for development only. `--no-video` runs in about 80 s wall plus the GPT call.
- **Peak RSS:** 1.13 GB for the video run (two 720p renderers: the recorder and the counter camera); 0.78 GB without video. One process at a time, under the `runw.sh` 3 GB watchdog.

### Scene
**Dishes:** 3 flat plates at y 0.40 / 0.63 / 0.86 and 2 inverted bowls at y 1.04 / 1.18. The gaps were widened to about 27 mm so the depth blobs separate.

**Clutter:** 8 items left of the dishwasher (y < −0.27), outside every arm path of the sequence:
- **from `gpt_sort/items.py` (copied as `items.py`):** a wooden cutting board, a chef's knife, a cast-iron pan with grease decals, a vacuum flask and a mug;
- **new primitives:** a dish-soap bottle, a sponge and an oil bottle.

All collision geometry is boxes and cylinders.

**Overhead camera `countercam`:** (0.42, 0.04, 2.20) looking at (−0.26, 0.04, 0.90), fovy 56°, 1280×720. Depth noise is σ = 1.5 mm × range, the same as `perception.py`.

### Recognition results (live GPT call in the video run, 29.3 s; GPT sees only the camera image + prompt)

| # | GPT label (EN / 中文) | Dishwasher-safe | GPT reason | GPT target | Matched item | Perceived pose error |
|---|---|---|---|---|---|---|
| 1 | Frying pan / 平底锅 | no | Hand-wash to protect nonstick coating / 手洗保护不粘涂层 | leave / hand-wash | pan (cast iron) | – |
| 2 | Orange mug / 橙色马克杯 | yes | Ceramic; no suitable space this cycle / 陶瓷杯，本轮无位置 | not loaded this cycle | mug | – |
| 3 | Oil bottle / 食用油瓶 | no | Contains oil; leave on counter / 装有油，留在台面 | leave / hand-wash | oil | – |
| 4 | Insulated flask / 保温瓶 | no | Hand-wash to protect insulation and seals / 手洗保护保温和密封 | leave / hand-wash | flask | – |
| 5 | Wooden cutting board / 木砧板 | no | Wood can warp and crack / 木材易变形开裂 | leave / hand-wash | board | – |
| 6 | Kitchen knife / 菜刀 | no | Hand-wash to protect blade and handle / 手洗保护刀刃刀柄 | leave / hand-wash | knife | – |
| 7 | Dish soap bottle / 洗洁精瓶 | no | Contains dish soap; not a washing item / 装有洗洁精，不入机 | leave / hand-wash | soap | – |
| 8 | Green and yellow sponge / 绿黄色海绵 | no | Cleaning sponge; keep out of this load / 清洁海绵，本轮不入机 | leave / hand-wash | sponge | – |
| 9 | White dinner plate, left / 左侧白餐盘 | yes | Plain ceramic plate is dishwasher safe | plate slot 4 | plate 1 | **0.93 mm** (xy centre) |
| 10 | White dinner plate, middle / 中间白餐盘 | yes | same | plate slot 5 | plate 2 | **0.91 mm** |
| 11 | White dinner plate, right / 右侧白餐盘 | yes | same | plate slot 6 | plate 3 | **0.73 mm** |
| 12 | Blue bowl, left / 左侧蓝碗 | yes | Ceramic bowl; load upside down and tilted | bowl spot 1 | bowl 1 | **1.19 mm** (3-D foot-ring centre) |
| 13 | Blue bowl, right / 右侧蓝碗 | yes | same | bowl spot 2 | bowl 2 | **0.98 mm** |

**What GPT got right**
- 13/13 items found, and every dishwasher-safe / not-safe decision is sensible.
- The plate slots (4, 5, 6) and bowl spots (1, 2) form valid assignments, so the robot used GPT's slots; the loading order is front slot first.
- It called the cast-iron pan "nonstick". The decision (hand-wash) is right; the label is wrong.

**Re-perception at the counter edge** (depth only, no second GPT call): plate centre error 0.95 / 0.58 / 0.69 mm. GT fallback threshold: 4 mm for plates, 6 mm for bowls. Not triggered.

### Results (video run, final poses after everything was loaded and the robot parked)

| Item | Target (from GPT) | Pose used for the grasp | Final pose error | Tilt | OK |
|---|---|---|---|---|---|
| Plate 3 (right) | slot 6 | perceived (0.73 mm) | lowest rim +0.5 mm from slot centre | lean 3.7° | ✓ |
| Plate 2 (middle) | slot 5 | perceived (0.91 mm) | +1.3 mm | 2.9° | ✓ |
| Plate 1 (left) | slot 4 | perceived (0.93 mm) | +2.9 mm | 1.4° | ✓ |
| Bowl 1 | bowl spot 1 | perceived (1.19 mm) | foot-ring 4.1 mm | 18.4° from inverted | ✓ |
| Bowl 2 | bowl spot 2 | perceived (0.98 mm) | 6.9 mm | 17.8° from inverted | ✓ |

The predicates are the same as below. Other measurements:
- Plate in-hand slip during the tilt: 0.41–0.49 mm. Bowl lift slip: 0.12 / 0.60 mm.
- Max penetration 0.85 mm; MuJoCo warnings 0.
- The only unplanned robot contacts are the finger brushing the neighbour plate on release, at ≤ 1.5 N.

### Ground-truth audit: every read of simulator object state, classified

**Controller inputs (drive motion), all perceived or derived from sensing**

*Plate pick*
- Initial pose: `self.est` = GPT box → depth blob → circle fit (`kitchen.estimate_plate`).
- Push distance: from that estimate.
- Re-perception at the edge: depth blob nearest the predicted position (`Multi.perceive`).
- Pinch pose: from the re-perceived pose.
- In-hand transform `T_hp`: encoders relative to the re-perceived pose.
- Insert attitude and position: tilt path of the perceived pose + `T_hp`. Touchdown: wrist F/T.

*Bowl pick*
- Grasp point: the perceived foot-ring centre. xy comes from the top of the depth blob; z is the counter plane plus the known bowl model height (+64.3 mm). The raw depth maximum reads about 4 mm high from noise, which made the fingers miss the ring in the first test.
- In-hand offset `rel` for the placement: from the perceived grasp point and the encoders. Fixed in this audit: it previously read the true foot pose.
- Placement touchdown: wrist F/T.

*Rail setpoints and slot targets*
- Rail setpoints: from the perceived y.
- Plate slot and bowl-spot targets: the known dishwasher model + the known basket travel (end stop). Fixed in this audit: the bowl target previously read the basket body pose from the simulator. Same value, 0.125 / 0 / 1.021.

**Decisions that use ground truth (stated on screen and here)**
- **GT-fallback switch:** per dish, the perceived-vs-true error is compared with 4 mm (plates) / 6 mm (bowls) to decide whether to use the perceived pose. This is the allowed "fall back to GT only if perception would break the grasp" rule. It never fired.
- **Item identity:** each GPT box is matched to a simulator body by the nearest true position. This only names the item for scoring and the table; the robot moves to the perceived position.
- **Step success predicates:** these read true poses and contacts, and on failure the step aborts and the robot moves on to the next item. They have no effect on motion when the step passes, which is every step in the reported runs. They are:
  - pick_flat: overhang window, finger contact after the pinch, held/upright after lift;
  - pick_bowl: rise / slip / contact;
  - `plate_slot_state` and `bowl_check` after insertion.

**Scoring / HUD only**
- Slip tracking, centre and tilt errors, the clutter-moved check, contact logs, the final table.

**Reset only (`qpos` writes)**
- Arm, rail, door 90° and basket 0.33 m, set once at t = 0.

### Kitchen-upgrade simplifications (honest list)
**GPT and perception**
- GPT runs on one rendered, clean, synthetic overhead image (no glare, no depth holes).
- The depth noise model is Gaussian. Poses use known models: the 8″ plate radius only as a sanity value in the fit, and the bowl height for z.
- GPT's recognition happens once at the start. Its latency is real (29.3 s) but fast-forwarded in the video.
- The robot does not handle the clutter. It is placed out of the arm's sweep by layout, not by planning around it.
- The mug is recognised as dishwasher-safe but deliberately not loaded (no mug space in this load); GPT chose "not loaded this cycle" itself.

**Earlier simplifications, still in force** (see the earlier-version section below)
- Door already open, basket already out (0.33 m), no pod or buttons.
- Linear axis lengthened to 1.10 m.
- Insert aim of 10 mm (gpt_engineer p2_lean fix) and the small release opening.
- Plates side by side rather than stacked; video at 3× speed.

**Reproducing the earlier video:** the earlier plates+bowls-only video `out/multi_dish_demo.mp4` was made before the dishes were re-spaced and the clutter added. Its exact code is kept in `debug/multi_v1_backup.py`.

---

## Earlier version (plates + bowls only, ground-truth poses): `out/multi_dish_demo.mp4`

This is the independent backup demo, built with the simplest robust approach. **5/5 dishes are placed correctly** in the video run, judged from the measured final poses. All 5 are also placed in each of 3 extra runs with perturbed start poses.

- Video: `out/multi_dish_demo.mp4`. H.264 (yuv420p), 1280×720, 30 fps, **106.6 s**, **37.9 MB**, 3,199 frames. It shows 320 s of simulation at 3× speed. Frames were streamed to ffmpeg (imageio-ffmpeg / libx264) during the simulation.
- Key frames: `keyframes/` (8 frames). `frames/` has one still per step, plus `98_loaded_basket.png` and `99_final.png`.
- Data: `out/result_final.json` (video run) and `out/result_s{1,2,3}.json` (perturbed runs). Logs are in `logs/`; peak RSS is in `logs/peak_rss.tsv`.

## Run

```
cd multi_dish_b
./runw.sh logs/video.log multi.py                        # video run -> out/raw_demo.mp4 + frames/ + out/result.json
./runw.sh logs/nv.log    multi.py --no-video             # ~45 s wall
./runw.sh logs/s1.log    multi.py --no-video --seed 1    # +-5 mm x/y on every item, +-10 deg plate yaw
```

`runw.sh` runs one process under an RSS watchdog that kills it above 3 GB. It uses the full_pipeline venv (`common_ground_dishwasher/.venv`).

## What happens (one continuous `mj_step` simulation)

**Start state**
- The door is open (90°) and the basket is pulled out (0.33 m).
- On the counter, right of the machine: 3 flat 8″ plates side by side at y = 0.40 / 0.615 / 0.83, and 2 inverted 115 mm bowls side by side at y = 1.00 / 1.155.

**Plates 1→3 → slots 6, 5, 4.** Slots are numbered 1 (back) to 8 (front); the front slot is used first. Each plate runs the **unchanged full_pipeline FLAT skill**:
1. Drag the plate to the counter edge with an 8 N wrist-F/T admittance press (≈49 mm overhang).
2. Pinch the rim face.
3. Tilt up about the far rim; the rim stays on the counter.
4. Lift and hold 1 s.
5. Carry and insert between the tines, stopping on wrist-F/T touchdown.
6. Release and retract.

**Bowls 1→2 → bowl spots 1, 2.** The bowl PoC method is used:
1. Friction grasp on the foot ring from above (no scooping).
2. Lift 0.10 m.
3. Carry over the bowl section.
4. Rotate 15° about x and lower onto the existing 35°-leaning bowl tine row, with the basket side wall as the second support. Stop on wrist-F/T touchdown.
5. Release and retract.

**End:** the robot parks, the camera shows a close-up of the loaded basket, then a final checklist overlay.

**Caption bar** (bottom): the current item and target (e.g. `Plate 2/3 → slot 5`) and the current step. The top-right checklist ticks off each item as it finishes; the final overlay lists each item ✓ with its measured error.

## Results (video run, final poses measured after *all* items were loaded and the robot parked)

| Item | Target | Final pose error | Tilt | OK |
|---|---|---|---|---|
| Plate 1 | slot 6 (between tines 6/7) | lowest rim point +0.2 mm from slot centre (gap half-width 12 mm) | lean 3.9° from slot plane | ✓ |
| Plate 2 | slot 5 | +1.2 mm | 3.0° | ✓ |
| Plate 3 | slot 4 | +2.2 mm | 2.0° | ✓ |
| Bowl 1 | bowl spot 1 (tine 6 of bowl row) | foot-ring centre 9.9 mm from target | 16.1° from inverted (opening·z = −0.96) | ✓ |
| Bowl 2 | bowl spot 2 (tine 1 of bowl row) | 1.6 mm | 19.6° from inverted (−0.94) | ✓ |

**Plate OK** = the full_pipeline predicate:
- The lowest rim point is between the plate's own two tines, within 30 mm of the slot centre in y.
- The plane is < 10° from the slot plane.
- The plate touches only its own tines and the floor/frame wires.
- Speed < 5 mm/s, and the robot is not touching it.

The plate *centre* sits 12.6–13.9 mm forward of the slot centre. That is the geometric result of the lean: a plate in a 24 mm gap rests on one of its tines, so the centre offset is reported but not scored, as in full_pipeline.

**Bowl OK** = the bowl PoC thresholds:
- Foot-ring error < 15 mm.
- Opening·z < −0.9.
- Speed < 5 mm/s.
- Resting on bowl tines, touching no other dish, not touching the robot.

**At the moment of each insert** (`out/result_final.json` → steps):
- Plates: lowest-rim offset 3.3 / 3.4 / 3.3 mm; lean 0.92 / 0.83 / 0.95°.
- Bowls: foot error 8.7 / 1.5 mm; tilt 16.9 / 19.7°.
- The plate leans grow to 2–4° later. On release, the finger brushes the previously loaded neighbour plate at 1.4–1.9 N; these are the only unplanned robot contacts in the run, all < 2 N.

**Grasp quality**
- Plate in-hand slip over the 90° tilt: 0.24–0.27 mm; hold slip: 0.06–0.09 mm.
- Bowl lift slip: 0.12 / 0.58 mm.

**Physics health**
- Max penetration: 0.76 mm (bowl on a floor wire at release).
- MuJoCo warnings: 0.

**Perturbed start poses** (±5 mm x/y on every item, ±10° plate yaw; seeds 1, 2, 3): **3/3 runs place 5/5**.
- Plate leans: 1.8–8.0°. The worst is seed 3, plate 1, at 8.0°, still < 10°.
- Lowest-rim offsets: ≤ 4.1 mm.
- Bowl foot errors: 2.0–7.2 mm; bowl tilts 17.9–19.3°.

**Peak memory**
- Video run: 650 MB.
- No-video runs: 390–413 MB.
- Highest of all runs: 702 MB (the first video run).
- One sim process at a time throughout.

**Timing:** 320 s simulated. Wall time is 109 s with video and ~42 s without.

## Simplifications and deviations (honest list)

**Perception and start state**
- **Ground-truth object poses** replace camera + Codex perception. Both full_pipeline perception calls are replaced by the true body pose; the bowl grasp uses the true foot-ring pose. The insert/placement then uses only the in-hand transform measured at grasp (encoders) plus the known basket model.
- **The door starts open and the basket starts pulled out.** These are set once at reset with `dw.set_state`; no pod and no buttons.

**Mount: the M2-style cart with a longer linear axis, not plain M1**
- Three plates and two bowls side by side span 0.8 m of counter, which plain M1 cannot reach. So I used the proven M2 configuration: docked cart, base x = 0.40, z = 0.80, yaw 195°, with a linear axis along the counter front. The axis is lengthened from 0.40 m to **1.10 m** (base y 0.15–1.25).
- Every plate pick and every insert runs at the **same base-relative geometry as the proven M2 run**: base 0.04 m left of the plate for picks, base y = 0.36 for all inserts. The axis moves while the hand holds a lifted plate or bowl.

**Basket changes (my copy of `dishwasher.py`)**
- `BASKET_TRAVEL` 0.25 → **0.33 m**, so both bowl spots and slots 4–6 sit in front of the control band. The slide joint holds the basket at its end stop, as in a dishwasher whose rails extend onto the door.
- The bowl section is the model's existing one, unchanged: one row leaning 35° plus the side wall, from the bowl PoC. Bowl spots are centred on bowl-row tines 6 and 1, 0.14 m apart.

**Plate skill changes (my copy of `pipeline.py`)**
- **Insert aim offset 3 mm → 10 mm** toward the convex side. Credit: the `gpt_engineer` p2_lean fix (`gpt_engineer/diffs/p2_lean_FINAL.diff`). In this setup it brought the seated lean at insert from 5.0–7.7° to 0.8–1.1°.
- **Release opening** 200 ctrl → `grip_ctrl(0.006)` (≈6 mm per finger), to limit the finger's push on the neighbouring plate. Neighbour contact at release went from 3.4–4.1 N (opening 200) to 2.5–2.6 N (12 mm) to 1.4–1.9 N (6 mm). The final leans of the earlier plates went from 7.1 / 5.7° (12 mm) to 3.9 / 3.0° (6 mm).
- The plate body and site names and the slot index are now parameters (several plates, any slot). The predicate itself is unchanged.

**Order and approach**
- Slots are loaded front to back: seated plates lean toward the front, so loading back to front put the next hand into the previous plate.
- The back bowl spot is approached from in front (via-point 7 cm forward). With the fingers closing along x, the hand body otherwise hits the control band.

**Starting piles:** "piles" here means items **side by side** (allowed); there is no stack or destacking.

**Video:** 3× speed-up.

**Mechanics: what is not used, and what the gripper is**
- No welds, connects, mocap, teleports or applied forces on the objects. Object qpos is written only at reset.
- Grasps are friction grasps with the force-limited Franka gripper approximation from full_pipeline: 800 N/m, 70 N limit. Bowls are held with a ≈20 N tendon force on the foot ring.
- Gravity compensation is on the Panda only.

**Other limits, unchanged from full_pipeline**
- Rigid primitive dishes.
- The IK has no collision constraint; collision-freedom is shown by the contact log.
- The control layout (slot choice, approach via-points) is hand-tuned for this layout, and only 4 runs were made, so this is a demonstration, not a reliability estimate.

## Files

**Code**
- `multi.py`: scene (3 named plates + 2 PoC bowls), runner, bowl skill, HUD, results.
- `pipeline.py`, `scene.py`, `plan.py`, `dishwasher.py`: copies of full_pipeline with the edits listed above.
- `util.py`, `perception.py`: unchanged copies; `perception.py` is imported but not used.

**Tools:** `runw.sh` (the watchdog) and `debug/` (tracing scripts used during development; run them with `PYTHONPATH=..`).
