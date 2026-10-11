"""Sink recognition clip: out/sink_recognition.mp4 (1280x720, 30 fps, H.264; streamed to ffmpeg into a .tmp, then mv) + .png.
Inputs: out/sink.png (the frame GPT saw), out/sink_gpt.json, out/sink_score.json."""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'
import os
P = os.environ.get('SINK_PREFIX', 'sink')   # 'sink' (default layout) or 'sink6' (layout matching sink_refs/sink_6.png)

W, H, FPS = 1280, 720, 30
FONT = '/System/Library/Fonts/Hiragino Sans GB.ttc'
GREEN, RED, CYAN, AMBER = (46, 204, 113), (231, 76, 60), (52, 214, 235), (255, 196, 0)
INK, PANEL = (245, 246, 248), (18, 20, 24)
ZH = dict(plate='盘子', bowl='碗', mug='马克杯', cutting_board='砧板', knife='刀', pan='煎锅', flask='保温瓶', soap_bottle='洗洁精',
          sponge='海绵', dish_cloth='抹布', other='其他')
EN = dict(plate='Plate', bowl='Bowl', mug='Mug', cutting_board='Cutting board', knife='Knife', pan='Pan', flask='Flask',
          soap_bottle='Soap bottle', sponge='Sponge', dish_cloth='Dish cloth', other='Other')


def F(sz, bold=True):
    return ImageFont.truetype(FONT, sz, index=1 if bold else 0)


def ease(t):
    t = min(max(t, 0.), 1.)
    return t * t * (3 - 2 * t)


base = Image.open(OUT / f'{P}.png').convert('RGB')
g = json.loads((OUT / f'{P}_gpt.json').read_text())
sc = json.loads((OUT / f'{P}_score.json').read_text())
items, LAT, summ = g['items'], g['latency_s'], sc['summary']
dishes = sorted([i for i in range(len(items)) if items[i]['pick_order'] > 0], key=lambda i: items[i]['pick_order'])
others = sorted([i for i in range(len(items)) if items[i]['pick_order'] <= 0], key=lambda i: items[i]['box'][0])
order = dishes + others
fits = [f for f in sc['fits'] if f['purity'] >= .8]          # markers only where the fitted patch really is the dish surface
dummy = ImageDraw.Draw(base)


def tsize(t, f):
    b = dummy.textbbox((0, 0), t, font=f)
    return b[2] - b[0], b[3] - b[1]


def tag_lines(i):
    it = items[i]
    c = it['category']
    if it['pick_order'] > 0:
        return f"{EN[c]} · {ZH[c]}", f"{it['pick_reason_zh']}"
    return f"{EN[c]} · {ZH[c]}", f"{it['reason_en']} / {it['reason_zh']}"


f1, f2 = F(17), F(14, False)
placed, anchor = {}, {}
TH = 42
groups = (('bot', dishes, (H - 30 - 8 - 2 * TH - 6, H - 30 - 8 - TH)), ('top', others, (6, 6 + TH + 6)))
for side, ids, rows_y in groups:
    ids = sorted(ids, key=lambda i: (items[i]['box'][0] + items[i]['box'][2]) / 2)
    edge = {0: 8, 1: 8}
    for k, i in enumerate(ids):
        l1, l2 = tag_lines(i)
        w = max(tsize(l1, f1)[0] + (34 if side == 'bot' else 0), tsize(l2, f2)[0]) + 16
        cx = (items[i]['box'][0] + items[i]['box'][2]) / 2
        row = k % 2
        x = min(max(cx - w / 2, edge[row] + 8), W - w - 8)
        placed[i] = (x, rows_y[row], x + w, rows_y[row] + TH)
        edge[row] = x + w
        b = items[i]['box']
        anchor[i] = (cx, b[3] if side == 'bot' else b[1])


def fit_text(t, f, wmax):
    if tsize(t, f)[0] <= wmax:
        return t
    while t and tsize(t + '…', f)[0] > wmax:
        t = t[:-1]
    return t + '…'


def badge(d, x, y, n, a, r=13):
    d.ellipse((x - r, y - r, x + r, y + r), fill=(255, 255, 255, a), outline=GREEN + (a,), width=2)
    t = str(n)
    tw, th = tsize(t, F(17))
    d.text((x - tw / 2, y - th / 2 - 4), t, font=F(17), fill=(10, 60, 30, a))


def draw_item(ov, i, a, dim=1.):
    it = items[i]
    col = GREEN if it['dishwasher_safe'] else RED
    al = int(255 * a * dim)
    d = ImageDraw.Draw(ov)
    x0, y0, x1, y1 = it['box']
    s = 1 + .12 * (1 - ease(a * 1.6))
    cx, cy, hw, hh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2 * s, (y1 - y0) / 2 * s
    d.rounded_rectangle((cx - hw, cy - hh, cx + hw, cy + hh), radius=6, outline=col + (al,), width=3)
    r = placed[i]
    ax, ay = anchor[i]
    ty = r[1] if ay < r[1] else r[3]
    d.line((min(max(ax, r[0] + 10), r[2] - 10), ty, ax, ay), fill=col + (int(210 * a * dim),), width=2)
    d.ellipse((ax - 4, ay - 4, ax + 4, ay + 4), fill=col + (al,))
    d.rounded_rectangle(r, radius=7, fill=(16, 18, 22, int(215 * a * dim)), outline=col + (al,), width=2)
    l1, l2 = tag_lines(i)
    off = 0
    if it['pick_order'] > 0:
        badge(d, r[0] + 20, r[1] + 13, it['pick_order'], al, 11)
        badge(d, x0 + 4, y0 + 4, it['pick_order'], al, 13)
        off = 32
    d.text((r[0] + 8 + off, r[1] + 3), l1, font=f1, fill=INK + (al,))
    d.text((r[0] + 8, r[1] + 23), l2, font=f2, fill=(col if it['dishwasher_safe'] else (255, 150, 140)) + (al,))


def banner(ov, t1, t2, a, col=AMBER, y=None):
    d = ImageDraw.Draw(ov)
    w = max(tsize(t1, F(30))[0], tsize(t2, F(18, False))[0]) + 48
    x = (W - w) // 2
    y = 110 if y is None else y
    d.rounded_rectangle((x, y, x + w, y + 84), radius=14, fill=(14, 16, 20, int(225 * a)), outline=col + (int(255 * a),), width=2)
    d.text((x + 24, y + 10), t1, font=F(30), fill=INK + (int(255 * a),))
    d.text((x + 24, y + 52), t2, font=F(18, False), fill=(200, 205, 212, int(255 * a)))


def footer(img):
    d = ImageDraw.Draw(img)
    d.rectangle((0, H - 30, W, H), fill=(10, 11, 14, 170))
    d.text((14, H - 25), 'MuJoCo simulation · dishes dropped & settled with physics · GPT = codex exec (ChatGPT subscription) on this one image',
           font=F(15, False), fill=(210, 214, 220, 255))


def fit_layer(ov, t):
    d = ImageDraw.Draw(ov)
    for k, f in enumerate(fits):
        p = ease((t - .3 * k) / .8)
        if p <= 0:
            continue
        a = int(255 * p)
        for (u, v) in f['patch_px']:
            d.point((u, v), fill=CYAN + (int(150 * p),))
        (cx, cy), (tx, ty) = f['center_px'], f['tip_px']
        d.ellipse((cx - 6, cy - 6, cx + 6, cy + 6), fill=CYAN + (a,), outline=(0, 0, 0, a))
        d.line((cx, cy, cx + (tx - cx) * 2.2, cy + (ty - cy) * 2.2), fill=CYAN + (a,), width=3)
        txt = f"tilt {f['tilt_deg']:.0f}° (true {f['gt_tilt_deg']:.0f}°)"
        tw, th = tsize(txt, F(15))
        lx, ly = cx + 12, cy + 8
        d.rounded_rectangle((lx, ly, lx + tw + 12, ly + th + 12), radius=6, fill=(8, 30, 36, int(220 * p)), outline=CYAN + (a,), width=1)
        d.text((lx + 6, ly + 2), txt, font=F(15), fill=(220, 250, 255, a))


T_RAW, T_LOOK, T_EACH, T_FIT, T_SUM = 2.0, 3.0, .75, 4.0, 6.5
t_look0, t_box0 = T_RAW, T_RAW + T_LOOK
t_fit0 = t_box0 + T_EACH * len(order) + .5
t_sum0 = t_fit0 + T_FIT
T_END = t_sum0 + T_SUM
n_in = len(dishes)
n_stay = len(others)


def summary(img, t):
    p = ease(t / .7)
    pw = 640
    x0 = int(W - pw * p)
    ov = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.rectangle((x0, 0, W, H), fill=PANEL + (240,))
    x = x0 + 26
    d.text((x, 14), f'{n_in} into the dishwasher · {n_stay} stay out', font=F(25), fill=INK)
    d.text((x, 50), f'{n_in} 件进洗碗机（按取放顺序）· {n_stay} 件不进', font=F(19), fill=(190, 196, 204))
    y = 88
    d.text((x, y), 'PICK ORDER  取放顺序', font=F(15), fill=GREEN)
    y += 22
    for i in dishes:
        it = items[i]
        badge(d, x + 11, y + 11, it['pick_order'], 255, 11)
        d.text((x + 30, y), f"{EN[it['category']]} · {ZH[it['category']]}", font=F(16), fill=INK)
        d.text((x + 170, y + 1), fit_text(f"{it['pick_reason_zh']} / {it['pick_reason_en']}", F(13, False), W - x - 170 - 12), font=F(13, False), fill=GREEN)
        y += 26
    y += 8
    d.text((x, y), 'STAYS OUT — why  不进洗碗机 — 原因', font=F(15), fill=(255, 130, 118))
    y += 22
    for i in others:
        it = items[i]
        d.ellipse((x + 4, y + 6, x + 14, y + 16), fill=RED)
        d.text((x + 24, y), f"{EN[it['category']]} · {ZH[it['category']]}", font=F(16), fill=INK)
        d.text((x + 190, y + 1), fit_text(f"{it['reason_zh']} / {it['reason_en']}", F(13, False), W - x - 190 - 12), font=F(13, False), fill=(255, 160, 150))
        y += 25
    st = sc['steps']
    bad = [s for s in st if not s['valid']]
    lines = [f"GPT vs ground truth: {summ['matched']}/{summ['gt_objects']} objects found · labels {summ['category_ok']}/{summ['gt_objects']} · dishwasher-safe {summ['safe_ok']}/{summ['gt_objects']}",
             f"pick order vs physics (remove-and-settle test): {summ['pick_steps_valid']}/{len(st)} steps free when picked"
             + ((f" (step {bad[0]['step']}: a bowl still leaned on that {bad[0]['gt'][:-1]}, moved {list(bad[0]['moves_mm'].values())[0]:.1f} mm)" if P == 'sink' else
                  f" (step {bad[0]['step']}: that {bad[0]['gt'][:-1]} still props up the plate stack; it slides {max(bad[0]['moves_mm'].values()):.0f} mm)") if bad else ''),
             f"GPT latency {LAT:.1f} s (one call) · surface tilt error {', '.join(str(f['normal_err_deg']) for f in fits)}° on exposed dishes"]
    miss = summ['dishes_gt'] - summ['dishes_ordered']
    if miss > 0:
        lines.insert(1, f"missed: {miss} of {summ['dishes_gt']} dishes (hidden plates inside the stack) · {summ['gpt_entries'] - summ['matched']} extra entry (cloth fold seen as a 2nd cloth)" if P == 'sink6' else f"missed {miss} dishes")
    for k, ln in enumerate(lines):
        d.text((x, H - 30 - 8 - 20 * (len(lines) - k)), fit_text(ln, F(12, False), W - x - 10), font=F(12, False), fill=(170, 176, 186))
    img.alpha_composite(ov)


def frame(t):
    img = base.copy().convert('RGBA')
    ov = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    if t < t_look0:
        d = ImageDraw.Draw(ov)
        a = ease((t - .3) / .5)
        d.rounded_rectangle((18, 18, 470, 62), radius=10, fill=(14, 16, 20, int(200 * a)))
        d.text((32, 25), ('Overhead camera · RGB-D  水槽俯视相机' if P == 'sink' else 'Sink camera · RGB-D  水槽相机'), font=F(21), fill=INK + (int(255 * a),))
        img.alpha_composite(ov)
        fa = ease(t / .4)
        if fa < 1:
            img = Image.blend(Image.new('RGBA', (W, H), (0, 0, 0, 255)), img, fa)
        footer(img)
        return img
    if t < t_box0:
        u = (t - t_look0) / T_LOOK
        img.alpha_composite(Image.new('RGBA', (W, H), (0, 0, 0, int(70 * ease(u * 4)))))
        d = ImageDraw.Draw(ov)
        sx = int((u * 2.2 % 1.0) * W)
        for k in range(24):
            d.line((sx - k * 2, 0, sx - k * 2, H), fill=AMBER + (max(0, 110 - k * 5),), width=2)
        dots = '.' * (1 + int(t * 3) % 3)
        banner(ov, f'GPT is looking{dots}   GPT 识别中   {LAT * ease(u * 1.05):5.1f} s',
               f'real latency {LAT:.1f} s for one call · shown {LAT / T_LOOK:.0f}× faster', ease(u * 5), y=300)
        img.alpha_composite(ov)
        footer(img)
        return img
    tb = t - t_box0
    fitting = t >= t_fit0
    for n, i in enumerate(order):
        a = ease((tb - n * T_EACH) / .35)
        if a > 0:
            draw_item(ov, i, a, 1 - .55 * ease((t - t_fit0) / .5) if fitting else 1.)
    if fitting:
        fit_layer(ov, t - t_fit0)
        if t < t_sum0:
            banner(ov, 'Depth fit on exposed dish surfaces   深度拟合', 'GPT box → depth back-projection → RANSAC plane on the visible surface → tilt vs ground truth',
                   ease((t - t_fit0) / .4) * (1 - ease((t - t_sum0 + .3) / .3)), col=CYAN, y=478)
    img.alpha_composite(ov)
    if t >= t_sum0:
        summary(img, t - t_sum0)
    footer(img)
    return img


def main():
    n = int(round(T_END * FPS))
    tmp = OUT / f'{P}_recognition.mp4.tmp'
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-f', 'mp4', str(tmp)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    last = None
    for k in range(n):
        last = frame(k / FPS).convert('RGB')
        p.stdin.write(last.tobytes())
    p.stdin.close()
    rc = p.wait()
    if rc == 0:
        os.replace(tmp, OUT / f'{P}_recognition.mp4')
    last.save(OUT / f'{P}_recognition.png')
    for nm, tt in ((f'{P}_k_look', t_look0 + 1.5), (f'{P}_k_boxes', t_fit0 - .2), (f'{P}_k_fit', t_sum0 - .3)):
        frame(tt).convert('RGB').save(OUT / f'{nm}.png')
    print(f'frames {n} duration {T_END:.1f}s rc={rc}')


if __name__ == '__main__':
    main()
