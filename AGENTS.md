# Project: Robot loads a countertop dishwasher (MuJoCo physics sim)

## Goal
A Franka Panda (MuJoCo Menagerie) loads one rice bowl into a countertop dishwasher and starts it, in true dynamic physics.
Sequence: open door → put 1 detergent pod into the inner-door groove → pull lower basket out → pick an upside-down bowl by its foot ring → place it upside-down between basket tines → push basket in → close door → verify inlet hose end is inside the water bucket → press Quick-wash, then Power/Start.

## Hard rules
- Physics must advance with mujoco.mj_step only. Never set object qpos/qvel during a run (only at reset). Grasps hold by friction: no weld/connect/attach constraints on the bowl or pod. If a fallback "assisted grasp" is ever added it must be behind a flag and printed on every rendered frame.
- Real dimensions in metres (SI units everywhere). Every number that comes from an estimate is marked `# EST` in XML/py with its source.
- Not simulated (state this in README and HUD): water flow, washing, capacitive touch (buttons are touch sensors; a real robot needs a conductive fingertip), detergent dissolving.
- Python 3.12 venv at ./.venv (mujoco 3.15.0, mink 1.3.0, imageio[ffmpeg], pillow). Do not upgrade or add packages without asking. Read the installed APIs; don't assume older MuJoCo/mink APIs.
- Robot model: ./mujoco_menagerie/franka_emika_panda/panda.xml, attached with prefix "panda/" (read it for actuator names, ctrl ranges, keyframe "home", body/site names). Do not edit Menagerie files.
- Keep changes small; one task per prompt; run the check command before finishing.

## Lessons from the 10 Oct PoC (bowl grasp, 10/10 randomized passes) — apply these, don't rediscover them
- Panda bodies need `gravcomp=1` (set via MjSpec on the attached bodies). Without it the position servos sag ~6 mm and the hand hits the object.
- Stock Menagerie gripper servo (100 N/m) cannot hold a 180 g bowl at friction 0.6 (capacity ≈1.47 N < 1.77 N weight). Use 600 N/m: gainprm[0]=600*0.04/255, biasprm=(0,-600,-10); keep ctrl range 0–255 (255 = open) and force limit ±100 N. Gives ~15 N actuator force.
- Object geoms need `priority=1`, otherwise contact friction mixes to the Panda's 1.0 and your 0.6 is silently ignored.
- Contact params that worked: condim 4, friction (0.6, 0.005, 0.0001), solref (0.006, 1), solimp (0.95, 0.99, 0.001, 0.5, 2), solver iterations 100, tolerance 1e-10, noslip 0.
- A tilted inverted bowl flattens between two vertical tine rows; one row leaning inward ~35° holds it at 17–19°. Level placement also works (1.8 mm error). Tine lean is our estimate, so label it.
- Lowering: stop when the object gets >0.5 N upward support for 10 ms, then open. This rebounds a few mm before release, so refine it if there's time.
- macOS offscreen render can fail inside Codex with `CGLError: invalid CoreGraphics connection`. The fix is a CGL context with `CGLPFAAllowOfflineRenderers`.
- The PoC used the object's true initial pose (no perception). Say so in the HUD/README.

## Two people work in parallel — FILE OWNERSHIP (never edit the other person's files)
| Track A (robot) | Track B (environment/product) |
|---|---|
| scene/build.py (robot attach + ph_ placeholders), ctrl/, task/run.py, tools/reach_check.py, tools/trials.py | scene/counter.xml, scene/dishwasher.xml, scene/props.xml, task/checks.py, render/, tools/view.py, README.md |
scene/world.xml is shared and frozen after setup: <option>, floor, light, robot_mount site, and includes of the three Track B scene files. The robot is attached in Python (scene/build.py); always load via scene.build.load_model().
If you need something in the other person's file, stop and write the request in NOTES.md instead of editing it.
Git: commit + `git pull --rebase` + push after each passing check, straight to main.

## Layout
scene/   world.xml (shared, frozen), build.py (A), counter.xml, dishwasher.xml, props.xml (B)
ctrl/    ik.py, skills.py (A)
task/    run.py (A), checks.py (B)
render/  video.py (B)
tools/   view.py (B), reach_check.py, trials.py (A)
out/     generated videos/json (gitignored)

## Interface (named sites — both tracks rely on these exact names; defined by Track B)
door_handle, door_groove, basket_handle, bowl_foot, slot_target, btn_quick, btn_power, hose_end, bucket_inside, filter_zone
Until Track B defines them, Track A adds placeholders in scene/build.py (via MjSpec) named ph_<name> (e.g. ph_door_handle) at the spec positions, and deletes them once the real sites exist.
