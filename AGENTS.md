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
