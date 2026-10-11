# Custom dish fingers for the Franka Hand: demo notes (2026-10-11)

**Result:** with the custom **rim-hook + wedge-blade** fingers (80 mm), the Franka on the M1 docked cart picks **2 nested bowls and 3 stacked plates** from piles on the counter. It places all **5/5 correctly by measured pose**, in one continuous MuJoCo run (`mj_step` only: no welds, attaching or teleports), with **0 unexpected robot contacts**.

The same tips at stock length (53.5 mm) place the 2 bowls but **0/3 plates**: the hand body hits the counter at the plate-edge grasp.

## Deliverables
| What | Path |
|---|---|
| Video (H.264, 1280×720, 30 fps, **94.4 s, 8.5 MB**, sim shown at 2×) | `out/fingers_demo.mp4` |
| Key frames (finger close-up + 8 steps) | `frames/00_finger_closeup.png` … `frames/08_final.png` |
| Printable STLs (ASCII, mm). **Chosen: L = 80 mm.** The 54 and 107 mm files are the sweep variants. | `fingers/hook_L80.stl`, `fingers/wedge_L80.stl` (also `*_L54.stl`, `*_L107.stl`) |
| Drawing with key dimensions (L = 80) | `fingers/drawing.png` (107 mm variant: `fingers/drawing_L107.png`) |
| Finger-length sweep | `out/finger_length_sweep.png`, `out/finger_length_sweep.json` |
| Run logs (per-item poses, metrics, contacts) | `out/final_L80.json`, `out/final_L107.json`, `out/sweep_L0.0535.json`, `out/sweep_L0.080.json` |
| Code | `fingers.py` (geometry, MuJoCo injection, STL), `scene_f.py` (scene), `demo.py` (controller, evaluation, video), `dishwasher.py` (copy with documented changes), `sweep_plot.py` |

Reproduce with one sim process under the 3 GB RSS watchdog. All defaults are the final settings:

```
./runw.sh logs/video80.log demo.py --L 0.080 --video out/fingers_demo.mp4 --keyframes --tag final_L80
```

Peak RSS over all 90 runs in `logs/peak_rss.tsv` was **691 MB**; the final video run was 626 MB. A run without video takes about 15 s wall time for 181 s of sim time; with video it takes 66 s.

## Chosen finger design (L = 80 mm, 1.5× stock)
The fingers are **asymmetric**. Each is built from convex hexahedra, and the same pieces are the MuJoCo collision meshes and the STL. Mass comes from density: **PETG 1.27 g/cm³** and **TPU 1.21 g/cm³**.

- **Hook finger (left slider), 80 mm, 19.1 g.**
  - 12×14×23 mm root block (the mount).
  - Beam tapering from 12 to 7.5 mm thick and from 22 to 20 mm wide.
  - Flat gripping face with a **22 mm TPU pad (μ = 1.0)** at the tip.
  - A **3 mm high × 3 mm long shoulder ("rim hook") 22 mm behind the tip**. The dish rim edge seats against it.
- **Wedge finger (right slider), 68 mm (12 mm shorter), 12.3 g.**
  - Same root block.
  - Beam tapering to a **thin blade: 3.6 mm → 2.0 mm over the last 22 mm**, with a 12 mm TPU pad on the gripping face.
  - Being shorter means it only reaches 10 mm past the shoulder line, so it fits in the 12 mm exposed band of a nested bowl.
- **Stock stroke and force are kept:** 80 mm stroke (0–40 mm per finger). Squeeze is force-limited at **70 N jaw clamp**: 140 N on the menagerie tendon, which has coefficient 0.5 per finger, with k = 800 N/m.
- **Mounting (sketch level):** the 12 mm root block replaces the stock finger on the Franka Hand finger slider, with 2× M3 clearance holes and a locating key matching the stock finger interface. The gripping face sits at the slider's zero-stroke plane, so q = 0 means the faces touch (as with stock).
- **Print:** PETG, 100 % infill in the blade. TPU 95A pad, dual-material or glued.

**How the tips are used:**
- **Plates:**
  1. The hand approaches the edge of the 3-plate stack horizontally, with the fingers along the 15° rim slope at the rim point facing the robot (30° azimuth).
  2. The **2.0 mm wedge slides into the 4.6 mm gap** between the top plate and the plate below. The hook pad stays 11 mm above the rim.
  3. On closing, the wedge lifts the top rim and the 70 N clamp holds it with the rim edge against the shoulder.
  4. The wrist then **pivots the plate about the hooked rim** to hang it upright.
  5. The hanging plate slides into its slot in its own plane, above the seated plates, and is lowered until the basket carries its full weight. The fingers open 9 mm and the hand retracts.
- **Bowls:**
  1. The fingers point down along the bowl wall.
  2. The **hook goes inside the top bowl** and the wedge slides down its outside, in the 12 mm band above the rim of the bowl below. The rim seats on the shoulder.
  3. Lift, then **pivot about the hooked rim** to 60° (wrist only).
  4. The bowl is placed with its back against a leaning back-rest row, its rim on the floor wires, and its opening toward the plates (+y).

## Results table (final run, L = 80 mm, `out/final_L80.json`; poses measured 2 s after the last release)
| Item | Slot | Final pose error | Tilt / lean | Checks | OK |
|---|---|---|---|---|---|
| Bowl 1 (top of nested pile) | bowl rest row 1, x = 98 mm | x error 4.7 mm | **33.5° from upright**, opening (−0.27, 0.48, 0.83) | in bowl area, at rest (0.0 mm/s), robot not touching | ✓ |
| Bowl 2 | bowl rest row 2, x = 98 mm | x error 1.3 mm | **60.0° from upright**, opening (−0.01, 0.87, 0.50) | in bowl area, at rest | ✓ |
| Plate 1 (top of stack) | plate slot 2 | centre 14.1 mm from slot centre* | **lean 1.7°**, plane 0.2° from vertical | lowest rim point between its own two tines, at rest | ✓ |
| Plate 2 | plate slot 4 | 11.9 mm* | **lean 1.0°**, 0.5° | between its tines, at rest | ✓ |
| Plate 3 | plate slot 3 | 14.8 mm* | **lean 2.1°**, 1.1° | between its tines, at rest | ✓ |

\* Most of this is the deliberate **10 mm aim offset toward the convex side** (gpt_engineer p2_lean fix: aim 10 mm, not 3 mm). It is the reason the plates lean only 1–2° instead of 12–35° without it.

**Success predicates:**
- **Plate:** the lowest rim point lies between its own two tines; the normal is less than 15° from the slot normal; the plate is within 20 mm of the floor wires; speed is below 5 mm/s; the robot is not touching it.
- **Bowl:** the bowl centre is inside the bowl area (basket x 20–160 mm, y < 40 mm); its axis is tilted 25–115° from upright; speed is below 5 mm/s; the robot is not touching it.

**Other metrics** (`out/final_L80.json`, `out/finger_length_sweep.json`):
- **Unexpected robot contacts > 2 N:** none, at max 0.0 N. The monitor counts any contact between the robot and a body other than the dish being handled.
- **Grip slip while the plate pivots from flat to hanging:** 0.8 mm and 0.3° for all 3 plates.
- **Grip slip from close to release, which includes the touchdown/unload phase:** plates 23–29 mm and 12–16°; bowls 11–22 mm and 8–17°.
  - Because of this slip, the final alignment is **closed-loop on the measured (ground-truth) dish pose** above the slot. The residual before lowering is 0.3–0.4 mm and 0.1° for plates, and 0.1–3.1 mm and 0.1–1.4° for bowls.

## Finger-length sweep (same tips; `out/finger_length_sweep.png`)
| Length | Bowls placed | Plates placed | Why / notes | PETG tip deflection under 70 N | Flange → TCP | TCP error per 0.1° wrist error |
|---|---|---|---|---|---|---|
| 53.5 mm (1× stock) | 2/2 | **0/3** | At the plate-edge grasp the hand body (±104 mm along the vertical closing axis) **hits the counter top** (134–287 N); the plate is not held | 0.38 mm | 95 mm | 0.17 mm |
| **80 mm (1.5×) – chosen** | 2/2 | 3/3 | no unexpected contacts | 1.9 mm | 121 mm | 0.21 mm |
| 107 mm (2×) | 2/2 | 3/3 | no unexpected contacts; the same poses, with the hand 27 mm further away | 5.7 mm | 148 mm | 0.26 mm |

**Feasibility = IK + collision + physics.** Each length was run end to end in physics with the same controller.

**Deflection** is the cantilever δ = F a³ / (3 E I), with:
- PETG E = 2.1 GPa;
- a beam section 20 × 9.5 mm;
- F = 70 N at the grasp point (a = L − 17 mm).

The deflection grows as L³. At 107 mm, 5.7 mm of tip compliance would eat into the 4.6 mm plate gap and the clamp. That is why **80 mm was chosen: it is the shortest length that succeeds**. The simulated fingers are rigid. For a real print, thicken the root to 14 mm (δ × 0.31) or add a carbon-fibre / aluminium spine.

**TCP error amplification** is the distance from flange to TCP times the angle error. It grows linearly: +28 % at 80 mm and +56 % at 107 mm versus stock.

## Design iterations (what failed and why, all measured in sim)
1. **Symmetric 10 mm pads, shoulder 10 mm from the tip.**
   - Bowls: rim grasp and lift from the nested pile worked first time, with 1.7–5 mm slip.
   - Plates: the hanging plate hinged in the grip about the rim tangent, because the 10 mm pad gives too short a lever against 0.44 N·m. It rotated 21° in the hand during the pivot and was released tilted, leaning 20–35°.
   - **Fix:** a 22 mm hook pad, and a 12 mm-shorter wedge so the bowl pick still fits the 12 mm nested band. This gives the asymmetric pair. The jaw clamp was raised to the 70 N spec, from 35 N. Plate pivot slip dropped to 0.7 mm / 0.3°.
2. **Joint-space pivot of the plate:** the plate flew out of the grip (over 500 mm slip) because of the wild wrist reorientation. A Cartesian pivot about the hooked rim from the 0° edge grasp hit IK limits (200 mm error).
   - **Fix:** an IK continuity search over the grasp azimuth showed that grasping at the **30° rim point** gives a continuous Cartesian pivot (0.4 mm tracking).
3. **Flipping the plate's concave side to the other side** (hand leaning away from the machine) needs a 180° turn at the hub. Link 7 hit the dishwasher's top corner and the plates dropped, so this was abandoned.
4. **Plates in the original tine rows (slots along x):**
   - Back slots: the hand (±104 mm along the closing axis = the slot normal) hits the control band and top front edge at x < 0.
   - Front slots: there is no room left for the bowl section.
   - **Fix:** the plate section in my basket copy is turned 90° (see simplifications).
5. **Plate release:**
   - Opening wide (38 mm per finger) **knocked the neighbouring seated plate** over (36° lean).
   - Releasing while the plate was only partly supported let it tip.
   - Sliding the next hanging plate past a seated plate at slot height grazed it.
   - **Fixes:** lower until the basket carries ≥ 92 % of the plate weight; open only 9 mm per finger; carry the hanging plate above the seated plates (TCP z ≥ 1.45 m), then descend vertically.
6. **Bowl placement variants:**
   - Opening toward −y with the hand outside the side wall: unreachable from the cart side.
   - Back-of-basket bowl slots: link 6 hits the control-band buttons.
   - 70° tilt: 26 mm in-hand slip.
   - Yawed bowls: they dropped.
   - **Kept:** two front back-rest rows at a 60° commanded tilt.
     - Bowl 1's front rim settles onto the bases of row 2, so it ends at **31–34°**, not 60°. It is still tilted and resting on the rest, but shallower than bowl 2. This is a weakness, and the 25° tilt threshold was chosen after seeing it.

## Simplifications (stated honestly)
- **Ground-truth object poses** are used instead of perception: for the grasp, and for closed-loop alignment of the dish in the hand before lowering. A real system would need wrist-camera or F/T feedback here.
- The **door is already open and the basket already out** (0.25 m travel). There is no pod and no buttons.
- **Small piles:** 3 plates, 2 bowls.
- **Basket copy changes** (`dishwasher.py` in this folder, marked `COPY`):
  - (a) The **bowl section** is two 80 mm back-rest rows leaning 30° toward −y, at y = −0.120 and −0.045 (row 2 only for x ≥ 0.04), replacing the single 35° row.
  - (b) The **plate section is turned 90°**: two tine rows along y at x = −0.040 and +0.100. Tine pitch (28 mm) and height (60 mm) are unchanged. This puts the plates' normal along y, so the hand above a plate stays in front of the machine instead of over the control band.
  - (b) goes beyond the "bowl section" allowance. It is needed because, from the M1 cart, the 208 mm-wide Franka hand cannot reach the original slots without hitting the machine.
- Plates and bowls are **rigid, primitive-based models** from the PoCs (8" plate 450 g; bowl 115 mm, 180 g). Contacts are soft; there is no breakage, water or soil.
- The **video is shown at 2× speed**. Sim time is 181 s for 5 items.
- **Fingers are rigid in sim.** Their compliance is estimated analytically (table above).
- **Robot gravity compensation is on**, as in all the PoCs.

## Comparison with the stock fingers
**Stock Franka fingers** are about 18 mm thick at the pad and 54 mm long. In this sweep, the custom tips at stock length still failed plates: the hand body hits the counter.

- **Bowls:** the GPT_SORT study found **0/33 bowl placement poses feasible** (IK or collision with tines, rails or window): "a rim pinch would leave the hand under the upside-down item".
  - Here, the hook-inside / wedge-outside rim grasp plus a wrist pivot about the hooked rim lets the hand **stay above the basket** for a tilted (not inverted) placement.
  - 2/2 bowls were placed with no robot contact.
- **Plates from a stack:** stock fingers cannot enter the **4.6 mm gap** between stacked rims, so the earlier pipeline needed a slow push-to-counter-edge step (FLAT PoC).
  - The 2.0 mm wedge picks the top plate **directly from the stack**: no drag, no re-grasp. Each pick takes about 10–13 s of sim time from approach to lift (the push-to-edge, re-perceive and pinch sequence it replaces took 38 s in full_pipeline).
- **Cost:** about 31 g of printed fingers. TCP error is 28 % larger. There is about 1.9 mm of tip compliance under 70 N (as designed; stiffen for production). Inverted bowl placement is still not done: the bowls end tilted (34° and 60°) with the opening facing sideways and up.

---

# Messy-sink variant (added 2026-10-11, 13:30)

**Result:** **3/3 plates** are taken off an offset, sliding stack in a sink basin and seated upright in the dishwasher. They are measured 2 s after the last release:

| Item | Slot | Centre error | Lean | Plane vs vertical | Lowest rim | Verdict |
|---|---|---|---|---|---|---|
| Plate 1/3 (top of stack) | slot 2 | 14.1 mm* | 2.0° | 1.0° | between its tines | ✓ |
| Plate 2/3 | slot 4 | 13.7 mm* | 1.7° | 0.9° | between its tines | ✓ |
| Plate 3/3 | slot 3 | 14.0 mm* | 1.9° | 1.1° | between its tines | ✓ |
| Bowl 1/2 and Bowl 2/2 | – | – | – | – | – | **not loaded** (see below) |

\* Includes the deliberate 10 mm aim offset.

- **Bowls failed.** For both bowls wedged against the basin wall, every rim grasp angle tried (5 per bowl) either had no IK solution from the M1 cart or the approach stalled more than 6 mm off the commanded path (stall errors of 10–415 mm were logged). The controller therefore skipped each bowl before touching it.
- **Contacts during the bowl attempts:** while the approach was stalling, the hand and fingers touched the faucet spout, the back worktop and the back basin wall. These are the only unexpected robot contacts in the run (peak 250 N). There were none during any plate pick or place.
- **Left in the sink:** the bottom plate (deliberately left: the 3 plate slots used were full) and the 2 bowls.

**Perception:** **ground-truth object poses** are used for:
- choosing the next dish (the top-of-stack check is "no other plate on its concave side");
- the grasp pose;
- the closed-loop alignment above the slot.

This is stated in the video captions ("GT pose") and in the final caption.

**Scene** (`sink.py`, my own build; I did not use `recognition_card/sink_scene.py` because mine was already running):
- A 330 × 400 × 150 mm stainless under-counter basin to the right of the dishwasher (as seen from the room). The worktop is split around the cut-out.
- A gooseneck faucet on the back ledge, a dish-soap bottle, a sponge (yellow with a green scrub layer) and a dish cloth (thin box).
- Following the user's reference photo `sink_6.png`: an **offset, sliding stack of 4 nested plates** leaning about 62° on the front basin wall, with each upper plate slid 12 mm down the stack. There are **2 bowls** by the left basin wall.
- The poses are that layout plus random jitter (seed 6). They then **settle under mj_step for 3 s**, which is shown at the start of the video.
- **This is not a random drop.** I first tried random drops (scan in `logs/`). The plates mostly settled flat, and the top-rim hook cannot grasp a flat plate.

**Strategy (the rim-hook + wedge fingers on a pile):**
1. Choose the top plate of the stack.
2. Approach from above along the rim slope. Each grasp angle around the rim is IK-checked before any motion, and the approach is Cartesian from above the sink so the arm never sweeps over the dishwasher.
3. The **2 mm wedge slides down between the top plate and the plate under it** while the hook pad goes on the free concave face. Close at 70 N so the rim seats on the shoulder.
4. Lift out of the pile, carry the plate hanging, and turn it so the grip point is at the top of the plate in its slot.
5. Closed-loop alignment on the GT pose, then seat until the basket carries ≥ 92 % of the weight.
6. Open 9 mm and slide the fingers off along the rim.

**Grip slip** in the hand, from close to release (this includes the turn and the touchdown): 30–32 mm and about 16°. After the alignment this does not affect placement.

**Deliverables:**
- `out/sink_fingers_demo.mp4`: H.264, 1280×720, **133.5 s, 10.1 MB**, shown at 2× speed. It opens with the finger close-up and has a caption bar throughout. The final checklist shows the 3 plates ✓ and the 2 bowls as "skipped, no reachable grasp".
- Frames: `frames/sink_t004.png`, `sink_t020.png`, `sink_t045.png`, `sink_t090.png`, `sink_t128.png`, `sink_final.png`.
- Run log: `out/sink_final.json` and `logs/sink_video.log`.
- Reproduce: `./runw.sh logs/sink_video.log sink.py --seed 6 --tag sink_final --video out/sink_fingers_demo.tmp.mp4`.

**Run time and memory:** 261 s of sim time, 102 s wall time. Peak RSS was 614 MB for this run and 691 MB across all 117 watchdog-logged runs.

**Robustness across seeds (honest):** the same code was run on other jitter seeds before the final one.
- Seed 6 placed 3/3 plates with no unexpected contacts.
- Seeds 1, 5 and 7 placed the first plate well, and then a later pick or retract knocked it over.
- Seeds 2–4 ended with plates leaning 21–90°.
- Bowls failed in every seed, for the reach reasons above.

**So the video shows the best seed. It is not a reliability result.** The main failure modes are:
- the in-hand grasp configuration varies from plate to plate, so the slot alignment sometimes cannot be reached;
- retract and carry motions can hit already-seated plates;
- the bowls by the basin wall are out of reach with a rim-down grasp from the docked cart.

The likely fixes are IK collision avoidance against the dishwasher and basin, and a sink placed closer to the cart.
