# Robot mounting: M1 vs M2 vs M3 (and why no M4 was built)

**Recommendation: M1.** A heavy cart docked against the cabinet fronts and braked, with a 0.80 m pedestal. The arm base sits 0.48 m in front of the counter edge and 0.42 m right of the dishwasher centre, yawed 195°.

- **M1:** passes every reach/collision target and ran the full task **10/10** with real camera + Codex perception. No unplanned contact above 20 N.
- **M2:** M1 plus a 0.40 m linear axis. Also 10/10, but it buys nothing the task needs: about 4 s slower, a wider cart and an extra servo axis.
- **M3:** clamped on the counter. It cannot do the flat-plate pick from any placement that also reaches the door. Even with the plate pre-placed upright in a drying rack, it fails to close the door: **0/5 end-to-end**.

All figures come from `mount_study.py`, `m3_scan.py`, `rail_scan.py`, `mount_table.py` (pose study) and `statics.py`, plus the end-to-end runs in `out/`. Raw data: `out/mount_table.json`, `out/m3_scan.json`, `out/rail_scan.json`, `out/statics.json`, `results.json`.

## Kitchen layout (all options, EST unless noted)

**Counter and cabinets**
- Counter top z = 0.90, depth 0.60 (spec). The front edge is the line x = 0 and the counter runs y = −1.25 … 1.25. Worktop thickness 0.04.
- Cabinet doors at x = −0.02 (20 mm overhang). Plinth 0.10 high, recessed 75 mm.

**Dishwasher**
- `dishwasher.py`, copied unchanged from `../dishwasher_model`. Centred at y = 0, front flush with the counter edge (spec).
- The open door is a 0.43 × 0.39 m slab lying at z 0.91–0.98 in front of the counter (x 0…0.39). The basket pulled out sits above it.

**Plate, pod and camera**

| Item | M1 / M2 | M3 |
|---|---|---|
| Plate start (flat) | centre (−0.15, 0.40): 0.15 m behind the edge, 0.08 m right of the dishwasher side | (−0.15, 0.80) |
| Pod dish | (−0.085, 0.655), further right | (−0.47, 0.58) |
| Wall camera | (−0.27, 0.42, 1.46), under a wall cabinet, looking down at the plate area | (−0.56, 0.80, 1.36) on the backsplash, looking at the rack |

- The plate start area is on the right of the dishwasher, the side with the buttons and the plate section of the basket.
- On that side the 60° tangential pinch approach of FLAT_NOTES puts the wrist away from the open door. On the left side it would sit next to the door: a hand clearance of about 35 mm.

## Options

| | M1 docked cart | M2 docked cart + linear axis | M3 counter clamp |
|---|---|---|---|
| Base (x, y, z), yaw | (0.48, 0.42, 0.80), 195° | (0.40, 0.40 ± 0.20, 0.80), 195° | (−0.10, 0.38, 0.90), 315° |
| Footprint | cart 0.65 × 0.60 m, rear bumper on the cabinet doors | cart 0.57 × 1.00 m, rail 0.40 m along y | 0.2 × 0.2 m clamp plate on the worktop |
| How chosen | Grid of 27 base poses (`mount_study.py --grid`): 11 pass all targets. Chosen for the best worst-case joint margin (15° at z = 0.80; z = 1.00 fails the door) | `rail_scan.py`: best base y per task. Setpoints: door / pod / perceive 0.40, basket / buttons 0.48, pick / insert 0.36 | `m3_scan.py`: 6 placements × 8 yaws × {flat, rack} = 96 configurations. None passes all 7 targets; this is the best rack placement |
| Plate start | flat (FLAT method) | flat | upright in a drying rack (PLATE method), because the flat pick is unreachable |
| IK collision avoidance | not needed | not needed | needed: the base is 0.17 m from the machine's side wall, and the elbow went through it without it |

Implementation notes:
- **M2 rail:** a slide joint on the cart with a position servo. Speed ≤ 0.3 m/s, travel 0.40 m, carriage gravcomp = 1.
- **M3 extras:** a counter-clearance height of 1.22 m (the plate in the rack stands 1.11 m high), a 2 cm pre-grasp under the open door, and a 19 mm re-grasp opening.

## Reach / collision table (pose study, `out/mount_table.json`)

**Method**
- mink IK on the full scene. The door and basket are set to the state of each step, and the open door, counter, cabinet and cart are all in the collision set.
- Pass criteria: < 2 mm and < 1°, joint margin ≥ 2°, no penetration (`mj_geomDistance`, all pairs).
- Paths are solved waypoint by waypoint. A joint jump > 0.6 rad inside a contact path fails.
- **Margin** = worst joint-limit margin along the path.
- **Clearance** includes the intended finger-to-bar gaps (2 mm by design) and the fingertip 0.5 mm off a button pad.

| Target / path | M1 | M2 (at its rail setpoint) | M3 rack | M3 flat | M3a yaw 45 (rack) | M3 "back" x = −0.30 yaw 135 (flat) |
|---|---|---|---|---|---|---|
| Door: grasp bar (closed) → follow arc to 90° → back (open + close) | PASS 15.0° | PASS 15.3° | **FAIL**: finger 5.7 mm into the door top rail at 90° | **FAIL** (same) | PASS 2.5° | **FAIL**: hand 12 mm into the door at 60° |
| Pod: dish → door groove (door open) | PASS 46.6° | PASS 48.2° | PASS 11.8° | PASS 11.8° | PASS 20.5° | **FAIL**: joint jump 0.71 rad |
| Basket handle, 0 ↔ 0.25 m (door open) | PASS 28.9° | PASS 38.8° | PASS 35.0° | PASS 35.0° | PASS 35.5° | **FAIL**: hand 47 mm into the basket |
| Flat plate: drag to edge, pinch, tilt, lift (door open, basket out) | PASS 30.1° | PASS 2.5° | n/a | **FAIL**: pinch approach 38.8 mm short | n/a | PASS 7.4° |
| Rack pick (M3 fallback) | n/a | n/a | PASS 25.2° | n/a | PASS 33.0° | n/a |
| Carry + insert into plate slot (basket out) | PASS 12.9° | PASS 12.9° | PASS 22.7° | PASS 36.5° | **FAIL**: link6 1.4 mm into the control band | PASS 33.2° |
| btn_quick | PASS 55.0° | PASS 59.0° | PASS 46.9° | PASS 46.9° | PASS 56.2° | PASS 24.1° |
| btn_power | PASS 32.9° | PASS 29.0° | PASS 34.2° | PASS 34.2° | PASS 37.2° | PASS 25.9° |

**M3, all 96 configurations (`out/m3_scan.json`)**
- No counter placement passes everything.
- Flat pick, first failing waypoint over the 48 flat-layout configurations: pinch approach 26, tilt 45–90° 18, lift 2, pass 2.
- The only 2 flat passes are at x = −0.30, near the back of the counter, and those fail the door, pod and basket.
- **The trade-off is structural:**
  - Near the front edge, the counter-height arm cannot get the hand under and around the overhanging rim with a continuous wrist motion. In simulation every individual pinch and tilt pose is reachable, but no continuous joint path exists: the tilt needs a > 0.6 rad wrist flip while the plate is pinched.
  - Near the back, it cannot reach the door arc or the basket.

**M2's rail (`out/rail_scan.json`)**
- Base positions in front of the machine (y ≤ 0.16) **fail** the door, pod, basket and plate: the open door and basket occupy that space.
- So a rail along the counter front can only slide within the region where M1 already works. It raises some margins (basket 26° → 38°) and allows a 0.08 m shorter cart, nothing else.

## End-to-end results (real camera + Codex perception every run)

Single continuous `mj_step` simulation per run. Plate start randomised (seed 20261010):
- Flat plate: ±10 mm x/y, ±10° yaw.
- Rack (M3): ±3 mm, ±3°, the poc_plate randomisation. ±10 mm in a 35 mm-pitch rack puts the plate on a prong.

| | M1 | M2 | M3 (rack fallback) |
|---|---|---|---|
| Trials | 10 | 10 | 5 |
| Full success / "clean" (no unplanned robot contact > 20 N) | **10/10 / 10/10** | **10/10 / 10/10** | **0/5 / 0/5** |
| Per step: perceive · door open · pod · basket out · pick · insert · basket in · door close · buttons | 10 · 10 · 10 · 10 · 10 · 10 · 10 · 10 · 10 | 10 · 10 · 10 · 10 · 10 · 10 · 10 · 10 · 10 | 5 · 5 · 5 · 5 · **3** · 3/3 · 3/3 · **0/3** · – |
| Failure modes | none | none | Close door: fingers cannot reach between the bar and the door face under the open door; the upper finger lands on the door's free end (3/5, key frame `frames/7_M3_failure_close_door.png`). Rack grasp missed (2/5): the same offsets succeed with the true pose, so the miss is sensitive to the 2.1–2.3 mm perception error. Door opening also has a 66 N unplanned hand–door contact |
| Cycle time, sim (successful runs) | 185.3–186.0 s | 189.7–191.4 s | – (fails at 142 s) |
| Plus Codex (2 calls, sim paused) | 212–220 s total | 216–223 s total | – |
| Peak RSS of the run.py process | 620 MB (trials), see RESULTS.md | 620 MB (trials), 894 MB (video run) | 584 MB (trials), 842 MB (video run) |

Documented single runs:
- **M3a** (yaw 45): fails `open_door`. The door swings into the forearm: link5 hits the door top rail at 321 N during the arc and 530 N while backing off, and the door stops at 55°.
- **M3flat** (M3 attempting the flat plate): door, pod and basket succeed, and the edge push succeeds (overhang 49.8 mm). Then: *"no joint path for pinch + tilt + lift"*.

## Statics (`statics.py`, worst sample over every M1/M2/M3 run, about 200 k samples per cart option)

**Input**
- The MuJoCo force/torque sensor at the Panda link0 mount, negated to give the arm's action on the base.
- It includes the arm weight. Gravity compensation is an internal joint torque in the model, so the sensor still sees the 171 N.
- It also includes the dynamics and every contact reaction: door, basket, plate push, buttons.

**Assumptions (rough):**
- Cart 120 kg (steel + ballast) at its footprint centre, plus a 15 kg column at the base. Castors 60 mm inside the footprint.
- Braked-castor friction μ = 0.5.
- The cabinet doors can only push the cart away from the counter. So tipping or sliding toward the counter is blocked by the dock, but motion away from it is not.
- M2: carriage at the worse end of its travel for every sample.
- M3: 0.20 m square clamp pattern, μ = 0.3 on stone.

| | M1 | M2 | M3 |
|---|---|---|---|
| Arm vertical load on the base | 144–217 N down | 125–212 N down | 75–222 N down |
| Peak overturning moment at the mount | 68 N·m | 54 N·m | 134 N·m |
| Centre-of-pressure margin to the nearest free castor line (front / left / right) | **0.183 m** (front 0.191) | **0.141 m** (front) | n/a |
| Cart mass that still keeps a 20 mm margin | 13 kg | 29 kg | n/a |
| Dock bumper force needed (arm pushing the cart toward the counter) | **55 N** | **69 N** | n/a |
| Force pulling the cart away from the counter (brakes must hold) / side force | 39 N / 58 N; brake friction used 7.7 % | 58 N / 48 N; 7.8 % | n/a |
| Clamp: required preload | n/a | n/a | **1.1 kN** (tip-over about the plate edge 1114 N, slide 489 N). Peak horizontal 189 N, largely the unplanned hand–door contact |

**Conclusion**
- A 120 kg docked cart has a large margin. Even a 30 kg cart would not tip from the loads this task produces.
- The dock bumper carries at most about 70 N, and the brakes use under 8 % of their friction.
- These are quasi-static numbers. Collision impulses outside the modelled task (someone leaning on the arm, an e-stop with the arm extended) are not covered.

## Cost / complexity in a real home (rough, labelled estimates; the Panda itself is common to all)

| | M1 | M2 | M3 |
|---|---|---|---|
| Extra hardware | Steel cart, ballast, 4 braked castors, dock bumper + a locating stop on the plinth. **Roughly US$1–3k** | M1 + 0.4 m servo linear axis, drive, cable chain, safety integration as an 8th axis. **Roughly +US$4–10k** | Clamp plate / C-clamps. **Roughly US$0.2–0.5k** |
| Floor space while working | about 0.65 × 0.60 m, docked under the counter overhang | about 0.57 × 1.0 m | none, but 0.25 × 0.25 m of worktop |
| Setup | Roll in, dock, brake. The pose repeats to a few mm with a locating stop; a calibration check is still needed | Same, plus homing the axis | 1.1 kN clamp preload on the worktop edge (stone or laminate must take it). Re-calibrate after every re-clamp |
| Task coverage here | all 9 steps | all 9 steps | flat pick impossible; door re-grasp under the open door impossible |

## Hybrids considered (M4): none built, because none is better than M1

- **Mobile base driving between two docked poses / longer rail.** `rail_scan.py` shows that every base position from which the dishwasher steps work is also a position from which the flat pick works: y 0.24–0.48 at x 0.48. M1's single dock already lies in that band.
  - A second pose would only add travel time and docking error.
  - Positions in front of the machine are blocked by the open door.
- **M3 + a front-to-back counter rail.** The flat pick needs the clamp near the back (x = −0.30). The dishwasher steps need it near the front (x = −0.10). They also need different yaws: 135° vs 315°/45°.
  - The door arc fails at every M3 yaw except 45°, and at 45° the forearm collides with the swinging door in simulation (M3a).
  - A rail cannot fix the door, so this hybrid would still fail the task.

## Recommendation

**Use M1**, with a locating stop so that the docked pose repeats.
- It is the simplest mount that passes everything: one base pose, no extra axis, no collision-aware IK needed.
- It completes 10/10 end-to-end with a single fixed dock, has the largest statics margin, and needs about 55 N of dock bumper force.
- M2 adds cost and an axis for no task benefit.
- M3 saves floor space but cannot do this task: no flat pick, no door re-grasp from the counter, and it needs 1.1 kN of clamp preload on the worktop.
