"""Architecture diagram: GPT -> API layer -> controllers -> physics, with rates. Run with /opt/homebrew/bin/python3."""
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager as _fm
_fm.fontManager.addfont('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
matplotlib.rcParams['font.family'] = 'Arial Unicode MS'
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

INK, MUTED, BG = '#1f2933', '#52606d', '#ffffff'
C = dict(gpt='#d9e8fb', api='#fde8c8', ctl='#dff3e4', hw='#eceff3', phys='#f3e1f1', safe='#fbd9d3')

fig, ax = plt.subplots(figsize=(15, 10.4), dpi=150)
ax.set_xlim(0, 15)
ax.set_ylim(0, 10.4)
ax.axis('off')
fig.patch.set_facecolor(BG)


def box(x, y, w, h, title, lines, col, fs=10.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12', fc=col, ec=INK, lw=1.2))
    ax.text(x + .18, y + h - .25, title, fontsize=fs + 1.5, weight='bold', color=INK, va='top')
    for i, ln in enumerate(lines):
        ax.text(x + .18, y + h - .75 - .34 * i, ln, fontsize=fs - .5, color=INK, va='top')


def arrow(x0, y0, x1, y1, label='', col=INK, ls='-', lx=None, ly=None, ha='left'):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>', mutation_scale=16, lw=1.6, color=col, linestyle=ls))
    if label:
        ax.text((x0 + x1) / 2 if lx is None else lx, (y0 + y1) / 2 if ly is None else ly, label, fontsize=9.5, color=col, ha=ha, va='center')


ax.text(.3, 10.1, 'GPT control of the dishwasher-loading robot: who decides what, at which rate', fontsize=15, weight='bold', color=INK)
# row 1
box(.3, 7.3, 4.4, 2.3, 'User goal (natural language)', ['"把台面上的盘子放进洗碗机，', '放好洗碗块，关门，快洗模式启动"', 'or the same goal in English'], C['hw'])
box(5.5, 7.0, 5.0, 2.6, 'GPT  (codex exec, ChatGPT subscription)', ['cloud; one decision per call', '8-20 s per call  ->  ~0.05-0.1 Hz',
    'in: goal + API docs + history', '     + state JSON + wall-camera image', 'out: {reasoning_brief, action, args}'], C['gpt'])
box(11.3, 3.6, 3.4, 6.0, 'Hardware safety', ['no software and no GPT', 'in this chain:', '', 'e-stop -> PILZ PNOZ', '  -> FR3 X3 input', '  + K2 -> wheel brakes', '',
    'FR3 safe-limited speed', '  <= 0.25 m/s near people', '', 'person flag from the', 'safety scanner / edges', '  -> API gate + SLS'], C['safe'])
# row 2
box(.3, 3.6, 4.4, 2.75, 'Skills + primitives (controllers)', ['L1 skills: open_door, load_pod,', '   pick_plate, insert_plate ...', 'L2 primitives: move_tcp, follow_arc,',
    '   push_along, move_until_contact, grip', 'behaviour tree 10 Hz, IK 100 Hz', 'F/T admittance (8 N push, 3 N press)'], C['ctl'])
box(5.5, 3.6, 5.0, 2.75, 'Robot API layer  (Main PC, ROS 2)', ['schema check -> INVALID_ARGS', 'safety gate: person / speed / force /', '   workspace -> REJECTED_SAFETY',
    'preconditions -> REJECTED_PRECONDITION', 'L2: IK path + collision check', 'result: ok / code / measurements / state'], C['api'])
# row 3
box(.3, .5, 4.4, 2.4, 'FR3 Control  /  sim physics', ['real: FCI torque loop 1 kHz,', '   collision detection, SLS / SLP', 'sim: mj_step 500 Hz (dt 2 ms),', '   position servos + gravcomp', 'Franka Hand, Bota F/T 800 Hz'], C['phys'])
box(5.5, .5, 5.0, 2.4, 'Kitchen + dishwasher', ['door (hinge), basket (slide), touch panel,', 'plate, pod', 'appliance sensors: SIMULATED access here', '(real: vision / appliance API)'], C['hw'])
box(11.3, .5, 3.4, 2.4, 'Wall node (Jetson + D435i)', ['RGB-D 30 fps', 'vision.locate: Codex box', '   + depth fit -> pose (mm)', '10-20 s per call; offline:', '   fallback detector'], C['hw'])

arrow(4.7, 8.45, 5.5, 8.45)
arrow(7.4, 7.0, 7.4, 6.35, 'action + args', INK, lx=7.3, ha='right')
arrow(8.6, 6.35, 8.6, 7.0, 'result + state', '#2f6fb3', lx=8.7)
arrow(5.5, 5.3, 4.7, 5.3, 'call', INK, lx=5.1, ly=5.5, ha='center')
arrow(4.7, 4.3, 5.5, 4.3, 'result', '#2f6fb3', lx=5.1, ly=4.08, ha='center')
arrow(1.6, 3.6, 1.6, 2.9, 'refs 100 Hz', INK, lx=1.5, ha='right')
arrow(3.4, 2.9, 3.4, 3.6, 'encoders, F/T 1 kHz', '#2f6fb3', lx=3.5)
arrow(4.7, 1.7, 5.5, 1.7)
arrow(11.3, 1.7, 10.5, 1.7, '', '#2f6fb3')
arrow(12.0, 2.9, 10.5, 3.9, 'poses, images', '#2f6fb3', lx=11.9, ly=3.35, ha='left')
arrow(11.3, 5.0, 10.5, 5.0, 'person flag', '#c0392b', lx=10.9, ly=5.25, ha='center')
arrow(11.35, 3.63, 4.6, 2.93, 'stop / speed cap (direct, bypasses GPT and API)', '#c0392b', ls='--', lx=6.0, ly=3.05)
ax.text(.3, .1, 'Rates: GPT ~0.05-0.1 Hz  |  API / behaviour tree 10 Hz  |  IK 100 Hz  |  torque loop 1 kHz (real) / mj_step 500 Hz (sim). '
        'GPT never sends joint torques or joint positions.', fontsize=10, color=MUTED)
fig.savefig('architecture.png', bbox_inches='tight', facecolor=BG)
print('architecture.png')
