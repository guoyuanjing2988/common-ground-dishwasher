# End-to-end dishwasher loading: results (2026-10-10)

**Headline:** a Franka Panda on a docked floor cart (**M1**, see MOUNTING.md) completes the full sequence in one continuous MuJoCo simulation, **10/10 randomised trials**, with **real camera + Codex perception** in every trial. M2 (cart + linear axis) also 10/10. M3 (counter clamp) 0/5. The sequence:

1. perceive the flat plate
2. open the door along its hinge arc
3. pod into the door groove
4. pull the basket out
5. drag / pinch / tilt / lift the flat plate
6. insert it upright between the tines
7. push the basket in
8. close the door
9. press QUICK, then POWER

## Run

```
PY=.venv/bin/python
$PY run.py                                  # option M1, one run with video -> out/M1_run.mp4, out/M1_single.json
$PY run.py --option M1 --trials 10 --no-video --seed 20261010
$PY run.py --option M2|M3|M3a|M3flat ...    # other mountings (MOUNTING.md)
./run_seq.sh                                # every production job, ONE process at a time, RSS watchdog (kill > 3 GB)
$PY statics.py; $PY aggregate.py            # -> out/statics.json, out/results.json (copied to results.json)
```

`--debug-truth-perception` replaces Codex by the true pose and is labelled DEBUG in its outputs. It was used only while developing; **no reported number uses it**.

## Deliverables

**Code**
- `run.py`: CLI and option definitions.
- `pipeline.py`: controller, steps, success predicates, video HUD.
- `scene.py`: scene builder.
- `plan.py`: pose generators shared with the reach study.
- `perception.py`: camera, Codex, depth fit.
- `dishwasher.py`: unchanged copy.
- Studies: `mount_study.py`, `m3_scan.py`, `rail_scan.py`, `mount_table.py`, `statics.py`, `aggregate.py`.

**Videos and frames**
- `M1_run.mp4`: the full run, 1280×720, 166 s. HUD with step name, ✓/✗ checklist and the required footer. Codex calls appear as still cards with the camera image, the Codex box and the depth-fit circle.
- `M2_run.mp4`: M2, 170 s.
- `M3_failure_run.mp4`: M3, 117 s, failing at the door re-grasp.
- `frames/`: 8 key frames.

**Data**
- `results.json`: aggregated numbers.
- `out/*_trials.json`, `out/*_single.json`: per-run, per-step logs with time, metrics and failure reason.
- `out/codex/`: every image sent to Codex, its JSON answer and an overlay.

## Per-step results (success predicate → outcome)

Ground truth (true object poses, dishwasher joint sensors, contacts) is used only in these predicates and logs, never for control.
- **Control inputs:** joint encoders (TCP pose), a wrist F/T sensor (MuJoCo force sensor at the hand flange), the camera, and the known, calibrated dishwasher model.
- **Clean:** no unplanned robot contact above 20 N.

| # | Step | Predicate | M1 (n = 10) | M2 (n = 10) | M3 rack (n = 5) | Sim time M1, mean |
|---|---|---|---|---|---|---|
| 1 | Perceive plate | Codex found the plate; radius 95–110 mm; plane ≤ 5° off; *scored* centre error < 5 mm | 10/10 | 10/10 | 5/5 | 2.5 s (+ Codex) |
| 2 | Open door | door angle > 85° after release | 10/10 | 10/10 | 5/5 (not clean: 66 N hand–door contact) | 22.2 s |
| 3 | Pod into groove | pod centre inside the groove recess, below the inner face, speed < 5 mm/s | 10/10 | 10/10 | 5/5 | 24.1 s |
| 4 | Pull basket out | basket ≥ 0.245 m and pod still in the groove | 10/10 | 10/10 | 5/5 | 15.9 s |
| 5 | Pick plate (flat: drag, re-perceive, pinch, tilt, lift; rack: rim pinch, lift) | held by fingers only, off the counter, plane < 10° from vertical, slip during 1 s hold < 5 mm (rack: rise ≥ 160 mm, slip < 5 mm) | 10/10 | 10/10 | **3/5** | 38.0 s |
| 6 | Insert plate | lowest rim point between its own two tines, < 10° from the slot plane, touching only its own tines + floor wires, robot not touching, speed < 5 mm/s | 10/10 | 10/10 | 3/3 | 21.4 s |
| 7 | Push basket in | basket < 5 mm, plate still in slot, pod still in groove | 10/10 | 10/10 | 3/3 | 17.3 s |
| 8 | Close door | \|angle\| < 1° after release and 1 s settle | 10/10 | 10/10 | **0/3** | 19.8 s |
| 9 | Press QUICK then POWER | touch sensor > 1 N on each pad | 10/10 | 10/10 | – | 17.7 s |
| | **All steps** | | **10/10 (10 clean)** | **10/10 (10 clean)** | **0/5** | **185.3–186.0 s** |

**Timing**
- Cycle time: 185–186 s sim (M1) and 190–191 s (M2), plus 2 Codex calls of **10.8–19.3 s (mean 15.8 s)** each. The simulation is paused during the calls. Total 212–220 s (M1).
- Wall time per trial without video: 47–55 s.
- The video run takes about 300 s wall.

## Key numbers (M1, 10 trials; min / mean / max)

**Perception (Codex + depth)**
- At start: centre error **0.44 / 0.88 / 1.75 mm**.
- Re-perception at the counter edge: **0.74 / 0.95 / 1.65 mm**. Codex centre pixel error is 3.6–7.4 px; the depth circle fit gives a radius of 101.6–102.3 mm (true 101.5).

**Edge push** (FLAT method: finger side face at 30° on the far rim)
- The drag distance is open-loop from perception #1.
- Overhang reached **48.4 / 48.7 / 49.1 mm** (design 45 mm).
- Mean press 9.1–10.9 N, regulated to 8 N with wrist-F/T admittance. Plate tilt during the push ≤ 1.7°.

**Pinch, tilt, lift**
- In-hand slip over the 90° tilt: **0.60 / 0.72 / 0.92 mm**.
- Slip during the 1 s hold: 0.06–0.15 mm.
- Plate plane 0.02–0.11° from vertical.

**Insert**
- Carry and insert use the **measured-at-grasp in-hand transform**: hand pose from the encoders relative to the **re-perceived** plate pose at closing. No re-perception in hand.
- The lowest rim point lands **3.3–4.0 mm** from the slot centre. The plate leans **7.2–7.9°** onto its own tine, as a real plate in a 24 mm gap does.
- Touchdown is detected by the wrist F/T: a peak of 11–14 N before the stop.

**Pod and basket**
- The basket-path check found the groove free: **34.5 mm** minimum pod–basket clearance during the pull. So the pod went into the groove, not onto the tub floor.
- The pod is 32 × 24 × 13 mm, 18 g. It drops about 15 mm into the 45 × 35 × 12 mm groove and settles 5.5 mm below the inner face.

**Door and buttons**
- Opened to 90.02° (on its stop). Peak finger force on the bar: 13.3 N.
- Closed to −0.01° (`door_closed` sensor true).
- Button press: advance until the wrist F/T reads 3 N, giving touch-sensor peaks of **5.4 N (QUICK) / 6.6 N (POWER)**.

**Physics health**
- Max penetration 1.15 mm, MuJoCo warnings **0**, unplanned robot–environment contacts **none** (M1).
- M2: max penetration 0.70 mm; one 10.6 N hand–plate-rim touch during the pinch, below the 20 N clean threshold.

**Variation across trials**
- The door, pod, basket and button steps are identical in every trial: only the plate pose is randomised.
- Randomisation: ±10 mm x/y, ±10° yaw, seed 20261010.

## Failure modes seen (with numbers)

**In the final runs:** none for M1 or M2.

**M3 (counter clamp)**
- 3/5 fail `close_door`: under the open door the upper finger lands on the door's free end. It cannot get into the 22 mm gap between bar and door face, so the fingers stay open at 22.5 mm each. The pose study shows the same thing: finger 5.7 mm into the door top rail at 90°.
- 2/5 miss the rack grasp (0 finger contacts, 24 mm "slip") at Codex + depth centre errors of 2.1–2.3 mm. The same plate offset with the true pose succeeds.
- **M3a:** the forearm hits the door (321–530 N) and the door stops at 55°.
- **M3flat:** no continuous joint path through the pinch + tilt.

**Found and fixed during development** (each would have failed the run):
- Basket grasp: the fully opened lower finger landed on the top frame wire 40 mm below the bar.
- Fingers locked into each other after a bar slipped out. Bar-grasp squeeze targets are now clamped at "fully closed".
- Joint-limit drift: q5/q7 reached ±2.9 rad after the sideways-tilted insertion pose, and q1 reached 2.9 rad mid door-arc in M2, pulling the bar out of the grasp. Fixed by planning every contact path as a joint path first: multi-seed sequential IK with the best worst-case margin. The online IK is then biased along it.
- Transfers that cut through the dishwasher side wall (747 N) or the control band (900 N). Fixed by routing: back off from the machine front, then a hub point, then descend from above into the counter area.
- Button presses at 215–350 N, from an F/T sign error.
- A rack built with the plate on a prong: it slid to 33° lean.
- A 3.7 mm bias in the robot's basket-position estimate (it was commanded 4 mm past the end stop). The estimate is now clamped to the known travel.
- In development, parallel runs wrote the Codex image to the same file, so Codex was shown the wrong scene (17–24 mm "errors"). File names are now unique per run.

## Peak memory (RSS of the run.py process, from `run_seq.sh` watchdog sampling every 2 s; `out/peak_rss.tsv`)

| Job | Peak RSS |
|---|---|
| M1 × 10 trials, no video | 620 MB |
| M2 × 10 trials, no video | 620 MB |
| M3 × 5 trials, no video | 584 MB |
| M3a single, no video | 559 MB |
| M3flat single, no video | 628 MB |
| M2 single with video (1280×720 H.264 streamed to ffmpeg) | 894 MB |
| M3 single with video | 842 MB |
| M1 single with video | not measured: it ran before the watchdog existed. An M1 no-video probe peaked at 622 MB |

The earlier about 20 GB per process came from `np.linalg.svd(P - c)` in the RANSAC plane refit, which built an N×N U matrix. It is now `full_matrices=False`; fixed. All final runs were executed strictly one process at a time.

## Honest limits

**Perception**
- Clean synthetic renders, Gaussian depth noise (σ = 1.5 mm × range in metres), no glare or depth holes.
- Codex supplies the 2-D detection. The millimetre pose comes from depth geometry plus a known 8″ plate model (radius used for sanity only).
- The plate's yaw is unobservable and irrelevant (it is round).
- The pod's pose is not perceived. It sits in a fixed dish; the dish location is known.
- The dishwasher pose and geometry are a calibrated model. The basket position is estimated from proprioception, clamped to its known travel.

**Push and insertion control**
- The edge-push distance is open-loop from the first perception. The pinch uses a second perception at the edge.
- The insertion uses the in-hand transform measured at the grasp. The in-hand slip during the tilt (≤ 0.9 mm) is not re-observed.

**Contact and actuation models**
- Rigid primitives everywhere: the plate is 36 rim boxes, the tines rigid capsules.
- The gripper is the PoCs' force-limited servo approximation of the Franka grasp mode (unverified).
- The wrist F/T is an ideal MuJoCo force sensor (no noise, no bias drift).
- Button "presses" are touch-sensor forces on rigid pads; capacitance is not simulated.
- No latch, water, washing, or door gasket.

**Controller**
- A mink differential-IK reference with joint-path pre-planning and routed transfers. It is not a general collision-aware planner. For M1/M2 the IK has no collision constraint; collision-freedom is verified only through the contact log of these runs.
- M3 uses static-shell collision avoidance; 0 infeasible-QP fallbacks occurred in the reported M3 run, 1 in M3a.

**Statistics and scope**
- 10 trials per option show feasibility on this test set, not a reliability estimate.
- The door, basket and button steps see no randomisation: the dishwasher pose is fixed.
- One plate, one pod, one slot, no other dishes.
- The cart is static once docked: its tipping is checked offline from the recorded wrench, not simulated as a free body.
- The cost figures in MOUNTING.md are rough estimates, not quotes.
