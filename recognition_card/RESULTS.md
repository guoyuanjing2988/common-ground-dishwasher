# Recognition card: GPT recognises a messy kitchen counter

**Deliverables**
- `out/recognition_card.mp4`: 1280×720, H.264 (yuv420p, CRF 18), 30 fps, **27.3 s**, 1.45 MB. Frames are streamed to ffmpeg.
- `out/recognition_card.png`: still of the final annotated frame (summary panel).
- Key frames: `out/k_look.png` (GPT banner), `out/k_boxes.png` (all boxes), `out/k_fit.png` (depth fit).
- `out/counter.png`: the exact image GPT saw. `out/gpt.json` holds GPT's answer (the run used in the clip), `out/gt.json` the ground truth, and `out/score.json` the scoring and depth fits.

**Clip sequence:** raw counter view (0–2 s) → "GPT is looking… / GPT 识别中" banner with a timer that runs to the **real 33.3 s** latency in 3 s, labelled "shown 11× faster" (2–5 s) → 11 boxes appear one by one, green with basket slot or red with reason (5–15 s) → depth-fit circles on the plate stack and bowls, each with "pose ±x mm" (15–20 s) → summary panel "6 into the dishwasher · 7 stay on the counter" with the reasons list (20–27 s).

## Scene
- Kitchen, counter and countertop dishwasher come from `../full_pipeline`, read-only. There is no robot.
- Items:
  - **Dishes:** 3 plates in a slightly offset pile, 2 bowls, 1 mug. Soil decals come from `../gpt_sort/items.py`.
  - **Other items:** chef's knife lying on a wooden cutting board (overlap), cast-iron frying pan with its handle over the counter edge, vacuum flask, and three items new in `scene_rc.py`: dish-soap bottle, sponge and stovetop kettle.
- 13 objects, settled for 5 s with `mj_step`. The pile plates slid 6–9 mm and tilt 2.5°, which is natural. Nothing else moved. Max |qvel| after the settle is 1e-3.
- One RGB-D render from a room-side camera (1280×720, fovy 44°). Depth noise σ = 1.5 mm at 1 m, as in `full_pipeline/perception.py`.

## GPT recognition
One `codex exec --skip-git-repo-check --sandbox read-only --output-schema … --image=counter.png` call, with stdin from /dev/null, on the ChatGPT subscription; no API key was used. GPT sees only the image and the prompt. A stack counts as one entry with a `count`.

Accuracy was scored against ground truth, with boxes matched greedily by IoU ≥ 0.3. The GT boxes are the visible-pixel boxes from the segmentation render.

| run | latency | tokens | entries found | label/category | count (plates 3) | dishwasher-safe | slots valid* | mean box IoU |
|---|---|---|---|---|---|---|---|---|
| **clip run** | **33.3 s** | 17 083 | **11/11** (13 objects) | 11/11 | 11/11 | 11/11 | yes | 0.93 |
| repeat 2 | 26.3 s | 16 992 | 11/11 | 11/11 | 11/11 | 11/11 | yes | 0.94 |
| repeat 3 | 29.6 s | 17 010 | 11/11 | 11/11 | 11/11 | 11/11 | yes | 0.93 |

\*Slots are valid when they are unique, plates are in P1–P4, bowls and the mug are in B1–B3, and non-safe items have no slot. In all 3 runs GPT gave plates P1 P2 P3, bowls B1 and B2, and the mug B3. It made no false positives.

Honest caveats:
- **The pan's material was misread.** Ground truth is a cast-iron skillet. GPT called it a "nonstick frying pan" in all 3 runs, with the reason "coating may degrade / 涂层易损". The category and the verdict (not dishwasher-safe) are right, but the reason is for the wrong material. The black primitive pan does not look like cast iron.
- **The kettle's material is a guess.** GPT said "aluminum may corrode/discolor". The ground truth is just "stovetop kettle, not dishwasher-safe" with no material, so this reason was not scored.
- **The free-text reasons vary between runs.** For example, the soap bottle got "hand wash", then "plastic may deform", then "not dishware". Only the boolean verdict is scored.
- **The reasons are GPT's own words, not a curated list.** The clip shows them unedited.

## Depth pose fit (plates and bowls)
Each fit is seeded only by GPT's box. The steps are adapted from `full_pipeline/perception.py`, and its `ransac_plane` and `circle_fit` are reused:
1. Back-project the depth image.
2. Fit the counter plane by RANSAC on the ring around the box.
3. Select points by height band. For the plate stack this is the top 12 mm of the stack, which is the top plate. For bowls it is the upper wall, 30–75 mm, which drops the flat board next to them.
4. Take the top-view outline as the farthest point per 5° bin.
5. Fit a RANSAC circle with 3 mm inliers and r between 4 and 13 cm, then refit by least squares.

Error is the horizontal distance between the fitted circle centre and the settled ground-truth body position.

| target | centre error (clip run) | repeats | fitted radius (true) |
|---|---|---|---|
| top plate of the stack | **1.6 mm** | 1.5, 1.5 | 102.6 mm (101.5) |
| bowl (left, B1) | **1.1 mm** | 1.1, 1.1 | 66.8 mm (65.0) |
| bowl (middle, B2) | **0.8 mm** | 0.8, 0.9 | 66.8 mm (65.0) |

The bowl radius is about 1.8 mm high because the fit traces the outer lip of the 4 mm wall, while the nominal 65 mm is the shell's centre radius.

## Simplifications
- Simulation only (MuJoCo primitives). Soil and grease are visual decals. The scene is hand-laid, not randomised, and there is one camera view.
- The pile is scored as one entry with count 3. Its pose is the top plate's only, because the lower plates are mostly hidden.
- Basket slots come from GPT, given the slot names of the `../gpt_sort` basket (P1–P4 plates, B1–B3 bowls/cups). They are not checked for physical fit here.
- The summary says **6** into the dishwasher (3 plates, 2 bowls and the mug) and 7 stay on the counter. The mug was asked for in the scene and is dishwasher-safe, so it is counted in. The count comes from GPT's answer and was not hard-coded.
- Memory: there was one sim process at a time, under the copied `runw.sh` watchdog with a 3 GB kill. The scene run finishes in under 2 s, faster than the watchdog's 2 s sampling. A separate probe of the same build plus 2500 steps, also under the watchdog, measured `ru_maxrss` at **60 MB**.

## Reproduce
```
./runw.sh logs/scene.log scene_rc.py                                   # scene + RGB-D + GT  (sim, watchdogged)
/opt/homebrew/bin/python3 recog.py                                     # GPT call -> out/gpt.json   (recog.py _r2 for repeats)
.venv/bin/python fit_score.py    # depth fit + scoring -> out/score.json
.venv/bin/python make_clip.py    # clip + still
```

## Variant without the mug: `out/recognition_card_5dish.mp4` (+ `.png`)
- **Clip:** 1280×720, H.264, 30 fps, **26.4 s**, 1.45 MB. The summary reads **"5 into the dishwasher · 7 stay on the counter"**.
- **Build:** `RC_VARIANT=_5dish` runs the same four scripts and writes files with a `_5dish` suffix. The original outputs are untouched.
- **Scene:** the same scene minus the mug. The mug is built into a throwaway spec, so the decal RNG stream and every other item's pose are identical (checked: the other 12 settled poses match exactly).
- **GPT:** a fresh call, 29.0 s and 16 923 tokens.
  - Found 10/10 entries (12 objects), with all labels 10/10, plate count 3/3, dishwasher-safe 10/10, valid slots (plates P1–P3, bowls B1 and B2) and mean box IoU 0.94.
  - This run read the pan as **"cast iron frying pan — iron rusts, seasoning strips / 生锈损伤油膜"**, which is correct, unlike the 3 runs with the mug.
- **Pose error:** top plate 1.5 mm, bowls 1.1 mm and 0.8 mm.

---

# Messy kitchen sink: `out/sink_recognition.mp4`

**Deliverables**
- `out/sink_recognition.mp4`: 1280×720, H.264 (yuv420p, CRF 18), 30 fps, **26.5 s**, 1.71 MB. It is written to `.tmp`, then moved into place.
- `out/sink_recognition.png`: still of the final frame.
- Key frames: `out/sink_k_look.png`, `out/sink_k_boxes.png`, `out/sink_k_fit.png`.
- Reusable scene module: `sink_scene.py`, which provides `build(seed)` and `run(seed)`.
- Pipeline: `sink_recog.py` (the GPT call), `sink_fit_score.py` (scoring and surface fits), `sink_clip.py`.
- Data: `out/sink.png` (the image GPT saw), `out/sink_gpt.json`, `out/sink_gt.json`, `out/sink_score.json`.

**Clip sequence:**
1. Raw overhead view of the sink (0–2 s).
2. "GPT is looking… / GPT 识别中" banner with a timer that runs to the **real 60.9 s** (2–5 s).
3. 14 boxes in pick order (5–16 s). Each dish gets a numbered badge ①…⑦ and its pick reason. Red boxes mark items that stay out, with GPT's reason.
4. Depth-fit markers on exposed dish surfaces (16–20 s): the fitted patch, the normal arrow, and "tilt X° (true Y°)".
5. Summary panel (20–26.5 s): **"7 into the dishwasher · 7 stay out"**, the pick order with reasons, and why each item stays out.

## Scene (`sink_scene.py`, seed 3)
- **Counter and sink.** The counter is built in this module with a 38 × 56 cm sink cut-out, because the full_pipeline counter is one solid box. The 20 cm deep stainless basin has a floor, 4 walls, a lip and a drain grate.
- **Fixtures.** A gooseneck faucet with lever, a tiled splash-back, and a collision wall that the board leans on. The full_pipeline countertop dishwasher (read-only) sits at the left edge.
- **Dishes.** 4 plates, 2 bowls and 1 mug, with soil and grease decals from `../gpt_sort/items.py`. They are placed with seeded perturbation (±8 mm, ±15° yaw, ±3° tilt), dropped 1 cm into rough pile poses and settled 6 s with `mj_step` (dt 1 ms).
- **Result after settling.**
  - One plate lies flat on the floor, and a second plate rests on it at 15° tilt.
  - Plates lean on the back wall (52°) and the right wall (36°). A bowl and that right-wall plate lean on each other.
  - The second bowl is tipped onto the plate pile, the mug lies on its side, and every dish ended inside the basin.
  - Max |qvel| after the settle is 6e-4.
- **Around the sink:** dish-soap bottle, sponge, striped dish cloth with a fold, cutting board leaning on the wall, chef's knife, cast-iron pan, vacuum flask.
- **Ground truth support test (physical).** Each dish in turn is teleported out of the sink, and the rest are stepped 0.6 s. A dish that moves more than 5 mm was resting on the removed one. Result:
  - plate0 (floor) supports plate1 (21.9 mm) and bowl5 (7.4 mm).
  - plate1 supports bowl5 (6.7 mm).
  - plate2 and bowl4 support each other (39.8 mm / 18.5 mm), a mutual lean.
  - plate3, bowl5 and the mug support nothing.
- **Memory:** one sim process under the `runw.sh` watchdog, peak RSS 138 MB.

## GPT (one `codex exec` call; subscription, read-only, stdin /dev/null; image and prompt only)
**Latency 60.9 s, 18 076 tokens.** Each item is matched to ground truth by IoU, with a bonus for the same category.

| metric | result |
|---|---|
| objects found (one entry per piece, incl. 4 overlapping plates) | **14/14**, no false positives |
| category / label | **14/14** |
| dishwasher-safe verdict | **14/14** (7 dishes in, 7 others out) |
| mean box IoU vs visible-pixel GT box | 0.86. Lowest are the mug (0.55; GPT boxed the handle region as well) and the floor plate (0.49; only its rim is visible) |
| pick order: first pick really free? | **yes**. Step 1 is the bowl that leans on the right-wall plate. It is half of a mutual lean, which counts as free. |
| pick order steps physically free when picked | **6/7** |

Each step was checked against the physics support test. Step 2 is wrong: GPT takes the plate on top of the floor plate ("overlaps rear left plate, take first"). The tipped bowl was still leaning on that plate, and in the support test it moved **6.7 mm** when the plate was removed. That is a light lean, just over the 5 mm threshold, but it fails the rule. Every other step is valid. The floor plate comes 6th, after both things resting on it, and the right-wall plate comes last, after its bowl.

## Depth markers (exposed dish surfaces)
Each marker is computed from GPT's box only:
1. Back-project the depth image.
2. Keep points above the basin floor plus 6 mm. The sink geometry is assumed known.
3. Fit a RANSAC plane (3 mm) to the largest flat visible patch.

The marker's quality is judged by segmentation purity (the share of patch pixels that lie on the matched dish) and the normal error against the dish's true axis. The clip shows markers only where purity is at least 0.8.

| dish | purity | fitted tilt | true tilt | normal error |
|---|---|---|---|---|
| plate on the pile (pick ②) | 0.97 | 15.0° | 15.0° | **0.5°** |
| tipped bowl (③) | 0.89 | 15.3° | 11.8° | **4.3°** |
| plate on back wall (⑤) | 0.96 | 52.5° | 52.1° | **1.7°** |
| plate on right wall (⑦) | 0.87 | 36.3° | 35.5° | **1.4°** |
| bowl leaning on plate (①) | 0.53 | — | 28.2° | patch mostly on the plate under it; not shown |
| floor plate (⑥) | 0.18 | — | 0.0° | buried; patch lands on the plate above; not shown |
| mug on its side (④) | 0.75 | — | — | curved, no plane; not shown |

## Simplifications and honesty notes
- Single seed and single GPT run, so there is no run-to-run variance for the sink.
- The support-test threshold (5 mm) is a choice. The one pick-order violation is a light lean (6.7 mm).
- GPT's free-text reasons are shown unedited. Some are weak:
  - sponge: "material compatibility unknown"
  - pan: "coating compatibility unknown". The pan is cast iron in ground truth.
- No robot and no grasp feasibility. "Pick order" means support order only.

```
./runw.sh logs/sink_scene.log sink_scene.py 3          # scene + RGB-D + seg + support test (sim, watchdogged)
/opt/homebrew/bin/python3 sink_recog.py                 # GPT call
<venv>/python sink_fit_score.py && <venv>/python sink_clip.py
```

---

# Sink variant matching `sink_refs/sink_6.png`: `out/sink6_recognition.mp4`

**Deliverables**
- `out/sink6_recognition.mp4`: 1280×720, H.264, 30 fps, **25.0 s**, 1.54 MB. It is written to `.tmp`, then moved into place.
- `out/sink6_recognition.png`: still of the final frame.
- Key frames: `out/sink6_k_*.png`.
- Data: `out/sink6.png` (the image GPT saw), `out/sink6_gt.json`, `out/sink6_gpt.json`, `out/sink6_score.json`.

**Reuse the exact layout**
- `sink_scene.build(6, layout="sink6")` or `sink_scene.run(6, layout="sink6")` rebuilds it. From the shell: `./runw.sh <log> sink_scene.py 6 sink6`.
- The downstream scripts take `SINK_PREFIX=sink6`.
- Layout parameters live in `sink_scene.LAYOUTS['sink6']`: sink cut-out, camera, dish seed poses, counter items, board, faucet and settle time.
- The `'default'` layout keeps the earlier scene.

**Scene**
- Single stainless basin, x ∈ [-0.44, -0.08] and y ∈ [-0.80, -0.30], **left of the countertop dishwasher** as in the photo.
- Back ledge with gooseneck faucet, soap bottle and sponge. The checked blue-and-white tea towel, with a fold, lies on the left counter.
- Cutting board leaning on the wall, plus pan, flask and knife on the counter. A front-oblique camera from above, like the photo.
- **Dish seeding:**
  - 4 plates seeded as a loose, offset stack: tilted 34° against the right basin wall, each shifted 1–2.4 cm sideways.
  - 2 bowls between the stack and the left wall. One of them is seeded at the foot of the stack as a chock.
- **Settling:** 20 s of `mj_step` (dt 1 ms). The stack slid sideways into a fanned pile, tilted 6–23°, propped by the chock bowl. The far bowl sits upright by the left wall.
- **Support test.** Each dish is removed in turn and the rest settle 0.6 s. Drift in a control run with nothing removed is subtracted (≤ 1.1 mm).
  - The 3 lower plates hold each other up, a mutual cycle.
  - **The chock bowl props up all 4 plates.** Removing it makes them slide 13–21 mm.
  - The top plate and the far bowl support nothing.
- Sim peak RSS 137 MB under the watchdog.

**GPT** (one call, 47.7 s, 18 872 tokens; the prompt says "dishwasher on the right"):

| metric | result |
|---|---|
| objects found | **11/13**. GPT **missed 2 of the 4 plates**, the hidden middle ones (291 and 642 visible px). It added **1 extra entry**: the tea towel's fold was listed as a second "white dish cloth". |
| labels / dishwasher-safe on matched objects | 11/11 and 11/11 |
| mean box IoU | 0.89 |
| first pick free | **yes**: the top plate of the stack |
| pick-order steps physically free | **3/4** |

The pick-order failure is step 3. GPT takes the centre bowl "after the front plate", but that bowl is the chock propping up the remaining plates, which slide up to 21 mm when it is removed. Steps 1 (top plate), 2 (far bowl) and 4 (rear plate, part of the mutual cycle) are valid. GPT's summary therefore says **4 into the dishwasher** while ground truth is 6.

**Depth markers:**

| dish | purity | normal error |
|---|---|---|
| top plate | 0.96 | 1.8° |
| far bowl | 0.91 | 2.2° |
| chock bowl | 1.00 | 1.8° |
| rear plate | 0.07 | not shown (its box mostly covers the plate above) |

**Note for reuse:** `sink_scene.py` gained layout parameters, a drift-corrected support test and an apron used only by `sink6`. The `'default'` outputs (`out/sink_*`) were produced before the drift correction and were not regenerated, and their support test had no drift control. That scene's max |qvel| after settling was 6e-4, so drift is expected to be small, but it was not measured.
