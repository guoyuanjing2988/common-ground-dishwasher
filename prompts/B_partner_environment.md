# Track B — partner: environment + checks + video

You own: scene/counter.xml, scene/dishwasher.xml, scene/props.xml, task/checks.py, render/, tools/view.py, README.md.
After each check passes: commit, `git pull --rebase`, push — Track A is waiting on B1 (~11:15) and B3 (~12:00).
Paste the spec table below wherever a prompt says [paste spec table].

**B1 — counter + dishwasher shell + door (10:30)**
> Read AGENTS.md. Fill in scene/counter.xml and scene/dishwasher.xml (they are already included by the frozen scene/world.xml — do not edit world.xml or scene/build.py), using the spec table below. Build the dishwasher as a hollow shell from box geoms (no meshes). Add the front door as a body with a hinge joint at its bottom edge (range 0–90°, opening outward/downward), joint damping, and a counterbalance spring (springref so it rests ~half-closed-resisting rather than slamming). Add sites door_handle (outer top edge centre) and door_groove (a recess on the inner face built from 4 thin box walls + floor). Add a "closed" latch: a sensor (jointpos) we can threshold at < 1°. Add tools/view.py that opens the scene via scene.build.load_model() in mujoco.viewer (run with `.venv/bin/mjpython tools/view.py`; on Windows plain python).
> [paste spec table]
> Check: `.venv/bin/python -c "from scene.build import load_model;m,d=load_model();print(m.njnt,[m.site(i).name for i in range(m.nsite)])"` prints the sites; in the viewer, dragging the door (double-click + ctrl-drag) opens it smoothly and it doesn't explode.

**B2 — basket, tines, filter, buttons (11:15)**
> Add the lower basket to dishwasher.xml (see PoC lessons in AGENTS.md: give one tine row near slot_target an inward lean of ~35°, marked # EST): a body on a slide joint along -y (travel 0.25 out of the front, low friction, small damping, joint limits), wire frame from thin capsules, rows of tines (capsule r 0.002, h 0.06, pitch 0.028). Add site basket_handle (front top centre), site slot_target (between two tine rows where an upside-down bowl should rest), filter_zone (box site over the filter on the tub floor). Add btn_quick and btn_power as small cylinders on the control band, each with a touch sensor (site-based). Check: script that steps 2 s with gravity — basket stays put, nothing drifts > 1 mm; in the viewer the basket slides out to its limit and back.

**B3 — props (11:45)**
> Create scene/props.xml: (1) rice bowl as one free body built from geoms (no mesh): rim ring + wall as a ring of ~24 thin box segments, a bottom disc, a foot ring of ~16 segments; total mass 0.18 kg; condim 4, friction 0.6, priority=1 and the PoC solref/solimp from AGENTS.md; site bowl_foot at the foot ring centre. Start upside-down on a small drip tray on the counter. (2) pod: free box, 0.015 kg, in a small dish next to the bowl. (3) bucket (hollow cylinder from segments) on the counter right of the robot (spec table), site bucket_inside (box site), and a hose_end free body (small capsule) resting inside it. Check: 3 s of mj_step — bowl and pod settle, max speed < 1 mm/s at the end, no penetration warnings (print d.warning).

**B4 — checks + HUD + video (12:30)**
> Create task/checks.py with predicates evaluated from MjData only: door_closed (angle < 1°), basket_in (slide < 2 mm), pod_in_groove (pod pos inside door_groove volume, not touching filter_zone), bowl_in_slot (bowl_foot within 15 mm of slot_target, upside-down: bowl up-axis · world z < -0.9, speed < 5 mm/s for 0.5 s), hose_in_bucket, buttons_pressed (touch > 2 N, order quick → power). Plus telemetry: max finger contact force, each joint |torque| / limit, min clearance robot↔dishwasher. Create render/video.py: offscreen mujoco.Renderer at 1280×720 (if it fails with CGLError on macOS, use the CGLPFAAllowOfflineRenderers context per AGENTS.md), render every N steps, overlay HUD with PIL (step name, checklist ✓/✗, telemetry, and a fixed footer "Simulation (MuJoCo). Water, washing and touch-screen are not physically simulated. Dishwasher internals estimated from product photos."), write mp4 with imageio. Check: render 3 s of the static scene to out/test.mp4 with the HUD visible.

---

## Spec table (paste into prompts that need numbers)

| Item | Value | Source |
|---|---|---|
| Counter | top at z=0.90, depth 0.60 (y from 0 front edge to 0.60 back), width 1.6 | standard worktop (EST) |
| Dishwasher outer | 0.430 W × 0.390 D × 0.460 H, 13 kg, sits on counter, front face flush with counter front edge, centre x=0 | listing spec plate |
| Shell wall | 0.015 thick | EST |
| Control band | top 0.06 of front face; buttons on it | listing photos (EST) |
| Door | front, hinge along bottom front edge, ~0.40 W × 0.36 H × 0.03 thick, opens 0→90° downward to horizontal; damping + counterbalance spring; closed-latch = angle < 1° | listing photos (EST) |
| Door groove | recess on inner door face, 0.045 × 0.035 × 0.012 deep, centred, 0.10 from door top edge | seller reply 2026-10-10 (location); size EST |
| Lower basket | 0.36 W × 0.32 D × 0.12 H wire basket on a slide joint along -y (out the front), travel 0.25, roller friction low | listing photos (EST) |
| Tines | capsules r=0.002, h=0.06, rows along x, pitch 0.028 | EST |
| Filter | disc r=0.04 on tub floor centre-back | listing photos (EST) |
| Buttons | btn_quick and btn_power on control band, 0.012 dia, right side | listing panel 快速洗 / 电源·暂停启动 |
| Rice bowl | Ø0.115 rim, h 0.060, wall 0.004, foot ring Ø0.050 × 0.008, 0.18 kg, ceramic μ≈0.6; starts **upside-down** on a drip tray right of the robot | typical (EST) |
| Pod | 0.025 × 0.025 × 0.015, 0.015 kg, starts in a small dish next to the bowl | typical (EST) |
| Bucket | hollow cylinder Ø0.25 × 0.25 **on the counter right of the robot** (≈x 0.80, y 0.20; left of the dishwasher would be ~0.85 m from the base, beyond Panda reach); hose end body starts inside it | listing "自吸" mode (EST placement) |
| Panda base | on counter, right of dishwasher, x≈0.45, y≈0.25, facing -x | EST; verify reach in prompt A1 |
