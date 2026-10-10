# Setup (each Mac, do tonight) / 环境安装（今晚各自装好）

Tested 2026-10-10 on Apple M5 Pro, macOS, Python 3.12.

```bash
# 1. clone
git clone https://github.com/guoyuanjing2988/common-ground-dishwasher.git
cd common-ground-dishwasher

# 2. Python 3.12 venv (needs python3.12: `brew install python@3.12` if missing)
python3.12 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install mujoco==3.15.0 mink==1.3.0 "imageio[ffmpeg]" pillow
#   (on a company network whose pip mirror lacks mujoco, prefix the install with PIP_INDEX_URL=https://pypi.org/simple)

# 3. MuJoCo Menagerie — Panda only (sparse checkout, ~43 MB)
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
git -C mujoco_menagerie sparse-checkout set franka_emika_panda

# 4. verify
.venv/bin/python -c "import mujoco,mink;m=mujoco.MjModel.from_xml_path('mujoco_menagerie/franka_emika_panda/scene.xml');r=mujoco.Renderer(m,240,320);d=mujoco.MjData(m);r.update_scene(d);print('ok',mujoco.__version__,m.nu,r.render().shape)"
#   expect: ok 3.15.0 8 (240, 320, 3)
.venv/bin/mjpython -c "import mujoco,mujoco.viewer,time;m=mujoco.MjModel.from_xml_path('mujoco_menagerie/franka_emika_panda/scene.xml');d=mujoco.MjData(m);v=mujoco.viewer.launch_passive(m,d);time.sleep(3);v.close();print('viewer ok')"
#   a window opens for 3 s (macOS needs mjpython for the viewer)
```

Also: log in to Codex (ChatGPT account) on your Mac, and open this folder in Codex.
Tomorrow: read `prompts/00_shared_start_here.md` first, then your own track file.
