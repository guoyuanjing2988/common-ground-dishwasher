# Track A — you: robot + control + full run

You own: scene/build.py, ctrl/, task/run.py, tools/reach_check.py, tools/trials.py.
`git pull --rebase` at ~11:15 (door from B1) and ~12:00 (basket/props from B2–B3); then switch from ph_ placeholders to real sites.
You also need task/checks.py and render/video.py from Track B (~12:30); until then print simple numbers yourself.

**A1 — robot in the scene + reach check (10:30)**
> Read AGENTS.md and ./mujoco_menagerie/franka_emika_panda/panda.xml. The Panda is already attached in scene/build.py at site robot_mount. Do not edit world.xml or Track B files; if the base pose must move, change only how build.py positions the attachment (e.g. move/override the frame in MjSpec) and tell the partner. In build.py, add placeholder sites ph_<name> (via MjSpec) for every interface site at its spec position. Write ctrl/ik.py using mink (current installed API): solve for a target pose of the hand/TCP (add a TCP site between the fingertips if Menagerie lacks one — in our scene file, not Menagerie). Then write tools/reach_check.py: for each interface site (door_handle, door_groove with door open, basket_handle at both slide limits, bowl_foot, slot_target, btn_quick, btn_power, hose_end), solve IK with a sensible approach orientation and report position error (mm), joint-limit margin, and collisions. Check: every site reachable with error < 2 mm and no robot–environment collision; if not, propose a new base pose and re-run.
> (Use the real site if it exists in the model, otherwise its ph_ placeholder. When Track B pushes real sites, delete the matching placeholders.)

**A2 — skills with real control (11:15)**
> Create ctrl/skills.py: a controller that converts IK joint targets into Panda position-actuator ctrl and advances with mj_step only. Skills: move_to(pose, duration) with smooth interpolation; grasp(width_target, force_check) using the gripper actuator; release(); pull(site, direction, distance) and push(...) as straight-line Cartesian moves with the gripper closed on/against a handle; press(site, depth) that stops on touch sensor > threshold. Each skill returns success/fail with reason. Check: script opens the door to 90° by hooking door_handle and pulling down along the arc (Cartesian waypoints on the door's arc), and pulls the basket out to its limit. Video to out/a2.mp4 via render/video.py if ready, else plain frames.

**A3 — pod into groove (12:00)**
> Using skills.py: pick the pod from its dish (top-down pinch), carry it over door_groove, lower to 10 mm above and release. Success = checks.pod_in_groove. Tune only contact params on our own geoms (friction, condim, solref) — never weld. Check: 5 consecutive successes from reset.

**A4 — bowl into basket (12:30 → freeze 13:30)**
> Pick the upside-down bowl by its foot ring (vertical approach, fingers pinch the foot ring outer diameter 0.050), lift 0.10, carry to above slot_target keeping it upside-down, lower until contact with tines, release, retract. Success = checks.bowl_in_slot. If the bowl jitters between tines, report the contacts and widen tine pitch within the EST range (≤ 0.035). Then: push basket in, close door (push door_handle up along the arc until door_closed), press btn_quick then btn_power. Write task/run.py as a state machine running the full sequence and saving out/run.json (per-step result, telemetry) + out/run.mp4. Check: one full successful run end-to-end.

**A5 — robustness (13:30)**
> tools/trials.py: run the full sequence N=20 times with bowl start pose randomised ±10 mm / ±5° yaw and pod ±5 mm; record per-step success and failure reasons; print a table and save out/trials.json. Do not change randomisation to inflate success. Check: table printed; success rate reported honestly.

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
