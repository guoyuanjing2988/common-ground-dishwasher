# Countertop dishwasher model — notes (2026-10-10)

Everything lives in `dishwasher_model/`. Run all scripts with
`.venv/bin/python` (MuJoCo 3.15.0, mink 1.3.0).

| File | Purpose |
|---|---|
| `dishwasher.py` | `add_dishwasher(spec, pos, yaw=0.0, prefix='dw/')`. Uses primitives only. Also has the helpers `door_closed()` and `set_state()`. |
| `scene_demo.py` | `build_scene(mount)`: floor, counter (top z=0.90, depth 0.60), dishwasher flush with the counter edge, and the Panda attached with `MjSpec.attach` (prefix `panda/`, gravcomp=1). |
| `tools/reach_check.py` | mink IK reach check (`--search` runs the mount grid search) → `tools/reach_results.json` |
| `tests.py` | Plain-assert tests → `test_results.json` |
| `render.py`, `render_util.py` | 4 PNGs at 1280×720 → `renders/`. `mujoco.Renderer` worked directly; the CGL `AllowOfflineRenderers` fallback is in place but was not needed. |

Sources: **listing** = Taobao listing; **seller** = seller answer of 2026-10-10; **PoC** = `poc_bowl_grasp` / `poc_plate` notes; **spec** = this task. Everything else is **EST** and is marked `# EST:` in the code.

## Coordinate frame
- **Dishwasher local frame:** the origin is the footprint centre at the bottom of the feet. +x points out of the front (the door opening direction), +y is to the viewer's right when standing in front of the machine, and +z is up. `pos`/`yaw` place this frame in the world.
- **Scene:** the counter front edge is the line x=0 and the counter occupies −0.60 ≤ x ≤ 0. The dishwasher is at (−0.195, 0, 0.90) with yaw 0, so its front face is at x=0.

## Dimensions and sources
| Item | Value (m) | Source |
|---|---|---|
| Outer W × D × H | 0.430 × 0.390 × 0.460 | listing |
| Net mass | 13.0 kg (model total 13.000 kg) | listing |
| Wall thickness | 0.015 | spec "~0.015" (EST) |
| Feet | r 0.015, h 0.015 | EST, typical rubber feet |
| Door thickness | 0.035 | EST, photo proportion (door with window) |
| Door hinge axis height | 0.048 above the support surface | EST. Raised from 0.030 so that the open door's pull bar clears the surface by 10 mm; at 0.030 the bar hit the floor at 88.6° (see "Changes made") |
| Door height / half-width | 0.352 (top edge z=0.400) / 0.213 | EST, photo proportion |
| Control band | z 0.405–0.460, full width, 6 pads | listing (touch buttons); pad size 24×16×3 mm, positions and order EST |
| Window | 0.32 × 0.182, one 4 mm pane, rgba (.42,.52,.62,.35) | listing ("large window"); size EST |
| Pull bar ("handle/lip at the top edge") | capsule r 0.008, length 0.20, centre 0.030 in front of the door face, 0.035 below the top edge | EST (stand-off chosen so the Panda fingers fit) |
| Door groove | interior 0.045 × 0.035 × 0.012; 2 mm walls and floor; centre at door-local (y −0.10, z 0.055) | spec dims; seller says the groove exists; position EST |
| Tub interior | x −0.180…0.159, y ±0.200, z 0.075 (floor)…0.445 | derived from the shell; floor height EST (pump/sump below) |
| Lower basket | 0.32 × 0.37 × 0.10, floor wires at z 0.121, 4 rollers r 0.010 on side rails | listing (wire basket on rollers); dims EST |
| Basket travel | 0.25 | spec |
| Floor-wire pitch | 0.014 (lengthwise), 0.04 (cross) | PoC (0.014); cross EST |
| Plate tines | r 0.002, h 0.060, pitch 0.028, two rows 0.14 apart, 9 tines/row = 8 slots | PoC-proven (PLATE_NOTES); 8 slots match the "8 plates" claim |
| Bowl tines | 1 row of 9, r 0.002, h 0.060, leaning 35° toward −y; the basket side wall 0.124 away is the second support (same layout as the bowl PoC) | PoC |
| Upper tray | wire tray 0.32 × 0.37 × 0.035 at z 0.350, on ledges | listing (removable shallow tray); fixed here; dims EST |
| Spray arm | hub r 0.018 + bar 0.30 × 0.022 × 0.010 at z 0.102; fixed | listing; dims EST |
| Filter | cylinder r 0.030, 0.020 proud of the floor, at (0.080, −0.100) | listing (cylindrical filter); size and position EST |
| Pod floor zone | site `floor_pod_zone` at (0.080, +0.110) on the tub floor, under the basket and away from the filter | seller (pod on tub floor under rack); position EST |
| Masses | door 1.6, basket 0.9, tray 0.3, spray arm 0.09, pads 6×0.002, shell 4.5, hidden machinery 5.598 | all EST; sum set to 13 kg |

**Basket layout.** The plate section is y −0.005…0.135 (rows), with plates standing facing the front and slots stacked front to back. The bowl section is y −0.185…−0.061.

**Capacity.** The 8 plate slots match the claim. The bowl section holds about 2–3 inverted rice bowls (Ø0.115) along the 0.32 m row, **not 8**. The listing's "8 bowls" was not reproduced and is probably not simultaneous with 8 plates.

**Sites** (all prefixed `dw/`): `door_handle`, `door_groove` (centre of the recess volume), `basket_handle`, `plate_slot_target`, `bowl_slot_target`, `filter_zone`, `floor_pod_zone`, and `btn_{eco,intensive,quick,dry,fruit,power}`.
- `plate_slot_target` is the centre of a seated 8" plate (r 0.1015) in the **2nd slot from the front**, at floor wire + 0.002 + 0.1015 + 0.001. The front slot sits directly behind the basket pull bar, where the hand collided with the bar.
- `bowl_slot_target` is the foot-ring centre of an inverted bowl resting on the 35° row, at (+0.0787 z, −0.0786 y) from the row base, copied from the bowl PoC.

**Sensors:** `dw/door_angle` (jointpos), `dw/basket_pos` (jointpos), and six `dw/btn_*_touch` (touch sensors on box sites enclosing each pad's outer face).

## Joint parameters
**Door hinge** (`dw/door_hinge`, axis local +y, range 0…90°). The door is 1.6 kg with its COM at door-local (x −0.016, z 0.176), so G = m·g·z_com = 2.763 N·m.
- **Counterbalance spring:** k = 1.303 N·m/rad, springref = −1.14°. These were chosen analytically (`door_spring()`) so that the net torque is −0.10 G at 0° and +0.25 G at 90°:
  - At 0°, the net torque holds the door shut against its closed stop. This "spring latch" is the closed-state notion; closed = |door_angle| < 1°.
  - At 90°, the net torque presses the door onto its open stop.
- **Net torque** (gravity + spring): 0° −0.276; 15° +0.106; 30° +0.457; 45° +0.728; 60° +0.877; 75° +0.873; 90° +0.691 N·m.
- **Other parameters:** damping 1.6 N·m·s/rad (tuned so the 45° release reaches the stop in about 2.4 s), frictionloss 0.30 N·m (EST hinge + gasket), armature 0.005 kg·m² (EST).
- **Consequence:** below about 20° the net torque is less than the friction, so a door left slightly ajar stays ajar. It does not self-close, and a real latch would.

**Basket slide** (`dw/basket_slide`, axis local +x, range 0…0.25 m): damping 2.0 N·s/m and frictionloss 0.30 N (both EST, rollers), armature 0. The rollers ride 1.5 mm above the rails, so the joint carries the weight and there are no roller contacts.

**Contact parameters** on all dishwasher geoms: friction (0.6, 0.005, 0.0001), solref (0.006, 1), solimp (0.95, 0.99, 0.001, 0.5, 2), priority 0.
- Dish/object geoms should use priority 1, as in the PoC notes.
- Scene options: implicitfast, 2 ms timestep, elliptic cone, impratio 10, 100 iterations, tolerance 1e-10.

## Test results (`tests.py`, all pass)
1. **Compile:** 13 bodies, 214 geoms, 3 joints (incl. the probe freejoint), 8 sensors.
2. **Mass:** total 13.000 kg. Door 1.6, basket 0.9, tray 0.3, arm 0.09, pads 0.012, static base 10.098 (shell 4.5 + machinery 5.598). The shell is world-attached, so its mass is documentation only.
3. **Geometric clearance** (`mj_geomDistance`, including static pairs):
   - Door closed vs everything: ≥ 0.50 mm (door bottom vs kick strip).
   - Basket in vs fixed parts: ≥ 1.50 mm (roller vs rail).
   - Door swept 0→90° in 5° steps: min 0.50 mm.
   - Basket swept 0→0.25 m with the door open: min 1.50 mm.
4. **Rest, 2 s, door closed and basket in:** 0 dishwasher contacts. Max penetration in the whole world was 0.189 mm, which is the test probe resting on the floor. The door stayed at |angle| ≤ 0.009° (closed = true) and the basket drifted 0.000 mm.
5. **Door released at 45°** (t in s : angle in °):
   - 0:45.0, 0.5:52.6, 1.0:62.5, 1.5:73.1, 2.0:83.0, 2.5:90.0, then 90.02 to t=8 s.
   - At 0.3 s it is at 49.1°, it reaches 89.5° at 2.39 s, and peak speed is 0.37 rad/s (21°/s). That gives a door-tip speed of about 0.13 m/s at the stop: no slam.
6. **Fully open:** set at 90°, the door stays at 90.02° with the joint limit active and a stop reaction of 0.690 N·m. Released at 80°, it is at 90.02° after 4 s.
7. **Basket, pushed with xfrc_applied 2 N** (door open):
   - Out to 0.250 m in 0.634 s (peak 0.64 m/s), then back in to 0.000 m in 0.634 s.
   - 0 basket contacts and penetration ≤ 0.19 mm, which is the probe on the floor.
   - Left at 0.12 m with no force, it stays at 0.1200 m.
8. **Touch:** a 20 g, 12 mm cube was "dropped" onto each vertical pad by pointing gravity into the panel, because the pads are on the vertical band face.
   - All 6 sensors read 0 before and 0.1962 N after (= m·g exactly).
   - Every other pad read 0.000 N.

## Reach check (`tools/reach_check.py`)
**Method:**
- mink IK runs on the full scene model, with the door, basket and finger DOFs frozen through `DofFreezingTask` constraints.
- Candidate gripper orientations come from the target geometry and are tried in order of preference.
  - Bar grasps: fingers straddle the bar, with the approach rotated about the bar (φ) and tilted along it (ψ).
  - Basket slots: from above, with tilts.
  - Buttons: closed fingertips press along the panel normal, with optional pitch and yaw.
- Each candidate uses up to 11 seeds.
- **Pass** means position error < 2 mm, orientation error < 1°, joint-limit margin ≥ 2°, and no robot–environment penetration. Penetration is checked with `mj_geomDistance` over all robot/environment geom pairs, so it also catches the static base link0; link0 vs counter_top is excluded because that is the mount.

**Mount search** (`--search`, 24 mounts, x ∈ {−0.08, −0.12, −0.16, −0.22}, y ∈ {−0.38, −0.44, −0.50}, yaw ∈ {0, 45}°):
- 20 of the 24 mounts pass all 9 targets. The 4 failures are (−0.16, −0.50, 0°), (−0.22, −0.44, 0°) and (−0.22, −0.50, 0°/45°). All 4 fail plate_slot_target, where the arm hits the control band or basket; the last two also fail btn_power. Mounts further back (x ≤ −0.25) failed several targets in earlier checks.
- **Chosen mount: (x −0.08, y −0.50, yaw 45°)** on the counter top, 0.29 m to the viewer's left of the dishwasher centre. It has the best worst-case joint margin, 9.9°.
- The link0 footprint spans x −0.223…−0.008, so it is fully on the counter, and y −0.643…−0.428.

**Results at the chosen mount: 9/9 pass**

| Target (state) | TCP target / grasp | Pos err mm | Rot err ° | Min joint margin | Collisions | Min clearance |
|---|---|---|---|---|---|---|
| door_handle (door closed) | bar grasp from the front, fingers vertical | 0.010 | 0.00 | 20.7° (j7) | none | 4.2 mm (finger–bar) |
| door_handle (door open 90°) | bar under the door's free end, from below, tilted 40° along the bar | 0.010 | 0.00 | 9.9° (j2) | none | 4.0 mm (finger–bar) |
| door_groove (door open) | pod release pose: TCP 18 mm above the groove centre (12 mm above the face), approach tilted 15° | 0.017 | 0.00 | 28.5° (j4) | none | 2.0 mm (finger–groove wall) |
| basket_handle (in, door open) | bar grasp from the front | 0.010 | 0.00 | 16.8° (j7) | none | 4.2 mm |
| basket_handle (out 0.25) | bar grasp from the front | 0.006 | 0.00 | 10.4° (j7) | none | 4.2 mm |
| plate_slot_target (out) | TCP at the seated-plate centre, from above (tilt −15° about the plate normal), fingers close across the plate | 0.010 | 0.00 | 34.1° (j7) | none | **0.7 mm** (hand–basket pull bar) |
| bowl_slot_target (out) | TCP at the bowl foot-ring centre, vertical from above, fingers close along x | 0.016 | 0.00 | 28.1° (j4) | none | 13.6 mm |
| btn_quick (door closed) | closed fingertips 0.6 mm off the pad, along the normal | 0.009 | 0.00 | 21.2° (j7) | none | 1.5 mm |
| btn_power (door closed) | same | 0.004 | 0.00 | 21.3° (j7) | none | 1.5 mm |

These solutions are kinematic poses only, not trajectories:
- **Plate slot:** the 0.7 mm clearance between the hand and the basket pull bar is tight. A controller should hold the plate higher, at its rim (≈ +0.09 m), which is what a real placement does.
- **Open-door handle:** it needs a from-below approach tilted 40° along the bar. It is reachable, but no motion plan into that pose was checked.
- **Groove:** the "groove" target is a release pose 18 mm above the groove centre, not the recess centre. With the TCP at the recess centre, the fingertips (8.9 mm past the TCP) would hit the groove floor, and the opened fingers (~50 mm) are wider than the 45 mm groove.

## Changes made during the work (honest log)
- **Hinge height.** With the hinge at 0.030, the door released at 45° stopped at **88.6°**: the 38 mm pull bar hit the support surface. I raised the hinge to 0.048 and moved the tub floor, rails, basket and spray arm up to keep all clearances.
- **Plate slot target.** At first it was in the front slot. IK put the fingers 14 mm into the basket pull bar, so I moved it one slot back.
- **Reach check method.** A quick-mode grid search (12 candidates × 2 seeds) was too restrictive to judge mounts, so I replaced it with the full-mode search. In the first full check at mount (−0.25, −0.50, 45°), 3 targets failed: open-door handle, plate slot and power button.

## Known simplifications
- **Spray arm and upper tray are fixed.** In reality the spray arm rotates and the tray is removable. To make the tray removable, give `dw/upper_tray` a freejoint.
- **Buttons are rigid pads with touch sensors.** Capacitive sensing is not simulated (a real robot needs a conductive fingertip), and the pads don't move.
- **No latch mechanism.** "Closed" is a spring preload (−0.28 N·m) against the 0° joint limit plus the |angle| < 1° sensor rule. A door opened past about 20° does not self-close, and below 20° friction holds it.
- **No door gasket, water, heater, pump, inlet hose or bucket.** The machinery mass is a hidden, non-colliding box.
- **Basket rollers don't touch the rails.** The basket is carried by an ideal slide joint (frictionloss + damping), and when it is out it is cantilevered from the joint. A real basket rolls onto rails on the door.
- **Wires and tines are rigid capsules.** Real tines flex, which matters for tight plate fits (see PLATE_NOTES).
- **The front is flush with the counter edge** (spec), so the open door overhangs the edge. On a flat surface the open door's pull bar clears it by 10 mm.
- **Capacity:** 8 plate slots, but only about 2–3 bowls in the bowl section; the listing's 8 bowls was not reproduced.
- **Unknown real geometry:** door/window proportions, button order and positions, groove position, filter position and all internal heights are estimates from typical designs, not measured from the product.
