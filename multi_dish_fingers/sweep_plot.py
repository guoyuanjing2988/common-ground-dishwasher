"""Finger-length sweep summary -> out/finger_length_sweep.png + out/finger_length_sweep.json (homebrew python3)."""
import json, re, math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

R = Path(__file__).resolve().parent
runs = {53.5: ('sweep_L0.0535', 'logs/sweep_0.0535.log'), 80.0: ('sweep_L0.080', 'logs/sweep_0.080.log'), 107.0: ('final_L107', 'logs/video.log')}
E, T, B, F = 2.1e9, .0095, .020, 70.          # PETG modulus, beam thickness, width, jaw clamp force
rows = []
for L, (tag, logf) in runs.items():
    d = json.loads((R / 'out' / f'{tag}.json').read_text())
    fin = d['final']
    bowls = [f for f in fin if f['item'].startswith('bowl')]
    plates = [f for f in fin if f['item'].startswith('plate')]
    log = (R / logf).read_text()
    piv = [(float(a), float(b)) for a, b in re.findall(r"pivot: tcp err .*?'dp': ([0-9.]+), 'da': ([0-9.]+)", log)]
    a = (L - 22 + 5) / 1000                       # load point from the root (TCP: 5 mm inside the shoulder line)
    defl = F * a ** 3 / (3 * E * B * T ** 3 / 12) * 1000
    tcp = 0.0584 + L / 1000 - .017               # flange -> TCP distance (m)
    rows.append(dict(L_mm=L, bowls_ok=sum(f['ok'] for f in bowls), plates_ok=sum(f['ok'] for f in plates),
                     plates_attempted=3, fail=d['fail'], max_unexpected_contact_N=d['max_unexpected_contact_N'],
                     first_unexpected=d['unexpected_robot_contacts'][0][:6] if d['unexpected_robot_contacts'] else None,
                     bowl_inhand_rot_deg=d['metrics'].get('bowl_rot_deg'), plate_inhand_rot_deg=d['metrics'].get('plate_rot_deg'),
                     plate_pivot_slip=[dict(dp_mm=round(p, 2), da_deg=round(q, 2)) for p, q in piv],
                     tip_deflection_mm_70N=round(defl, 2), flange_to_tcp_mm=round(tcp * 1000, 1),
                     tcp_err_mm_per_0p1deg=round(tcp * math.radians(.1) * 1000, 3)))
(R / 'out' / 'finger_length_sweep.json').write_text(json.dumps(rows, indent=1))
Ls = [r['L_mm'] for r in rows]
lab = [f"{r['L_mm']:.0f} mm\n({r['L_mm'] / 53.5:.1f}x stock)" for r in rows]
fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
x = range(len(rows))
ax[0].bar([i - .2 for i in x], [r['bowls_ok'] for r in rows], .4, label='bowls placed (of 2)', color='#d98a4a')
ax[0].bar([i + .2 for i in x], [r['plates_ok'] for r in rows], .4, label='plates placed (of 3)', color='#5b8fd1')
ax[0].set_xticks(list(x), lab); ax[0].set_ylim(0, 3.6); ax[0].set_title('Placement success (physics run)')
ax[0].legend(loc='upper left', frameon=False)
for i, r in enumerate(rows):
    if r['fail']:
        ax[0].text(i, 2.6, 'hand body hits\ncounter at the\nplate-edge grasp', ha='center', fontsize=8, color='#a33')
ax[1].plot(Ls, [r['tip_deflection_mm_70N'] for r in rows], 'o-', color='#c0392b', label='PETG tip deflection @ 70 N (mm)')
ax[1].plot(Ls, [r['tcp_err_mm_per_0p1deg'] * 10 for r in rows], 's--', color='#555', label='TCP error per 0.1 deg wrist error (x10, mm)')
ax[1].set_xlabel('finger length (mm)'); ax[1].set_title('Compliance and error amplification'); ax[1].legend(frameon=False, fontsize=8)
for i, r in enumerate(rows):
    pv = r['plate_pivot_slip']
    if pv:
        ax[2].bar(i - .2, max(p['da_deg'] for p in pv), .4, color='#5b8fd1', label='plate: rotation in hand during pivot' if i == 2 else None)
    if r['bowl_inhand_rot_deg']:
        ax[2].bar(i + .2, max(r['bowl_inhand_rot_deg']), .4, color='#d98a4a', label='bowl: max rotation in hand, close->release' if i == 2 else None)
ax[2].set_xticks(list(x), lab); ax[2].set_ylabel('deg'); ax[2].set_title('Grip slip (max over items)'); ax[2].legend(frameon=False, fontsize=8)
fig.suptitle('Finger length sweep - same rim-hook + wedge tips, M1 cart, MuJoCo', fontsize=12)
fig.tight_layout()
fig.savefig(R / 'out' / 'finger_length_sweep.png', dpi=110)
print(json.dumps(rows, indent=1))
