# Shared — start here (both people)

Files: this one (both), `A_you_robot.md` (Track A, robot), `B_partner_environment.md` (Track B, environment). Root: AGENTS.md (Codex rules), SETUP.md (install, do tonight).

## Step 0 — together, one laptop, 10:00–10:20

The repo, AGENTS.md and these prompts already exist (pushed 10 Oct). Both people: finish SETUP.md tonight.
At 10:00, ONE person gives Codex this prompt, then commits + pushes; the other runs `git pull`:

> Read AGENTS.md. Create the skeleton only: scene/world.xml with <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>, a floor, a light, a site `robot_mount` at pos="0.45 0.25 0.90" euler="0 0 3.14159", and <include> of scene/counter.xml, scene/dishwasher.xml, scene/props.xml (each a valid `<mujocoinclude>` with an empty worldbody). Create scene/build.py with `load_model()` that loads world.xml as a mujoco.MjSpec, attaches ./mujoco_menagerie/franka_emika_panda/panda.xml at site robot_mount with prefix "panda/" (MjSpec.attach), compiles and returns (model, data). Everything (viewer, run, checks, video) loads the scene ONLY through scene.build.load_model(). Create empty folders ctrl/ task/ render/ tools/ with __init__.py where Python needs it, .gitignore (.venv, mujoco_menagerie, out, __pycache__), and an empty NOTES.md. Check: `.venv/bin/python -c "from scene.build import load_model;m,d=load_model();print(m.nu)"` prints 8.

(Verified 2026-10-10 with mujoco 3.15: MjSpec.attach of Menagerie panda.xml at a site compiles, meshes resolve, 8 actuators named panda/actuator1…8. Matching integrator=implicitfast avoids an attach warning.)

AGENTS.md lives in the repo root (Codex reads it automatically).

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

---

## Final — README (partner writes it at 14:30, using your trial numbers)
> Write README.md: what it is (1 paragraph), video link/file, how to run (`.venv/bin/mjpython tools/view.py`, `.venv/bin/python task/run.py`, `tools/trials.py`), the honest-limits list from AGENTS.md, the spec table with sources (listing, seller reply, EST), trial success rate, credits (MuJoCo, Menagerie Panda — Apache-2.0, mink).
