"""Recognition card clip: out/recognition_card.mp4 (1280x720, 30 fps, H.264, frames streamed to ffmpeg) + out/recognition_card.png.
Inputs: out/counter.png (the exact frame GPT saw), out/gpt.json (GPT answer + measured latency), out/score.json (depth fits)."""
from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'out'
import os
V = os.environ.get('RC_VARIANT', '')   # '' = original scene; '_5dish' = same scene without the mug

W, H, FPS = 1280, 720, 30
FONT = '/System/Library/Fonts/Hiragino Sans GB.ttc'
GREEN, RED, CYAN, AMBER = (46, 204, 113), (231, 76, 60), (52, 214, 235), (255, 196, 0)
INK, PANEL = (245, 246, 248), (18, 20, 24)


def F(sz, bold=True):
    return ImageFont.truetype(FONT, sz, index=1 if bold else 0)


def ease(t):
    t = min(max(t, 0.), 1.)
    return t * t * (3 - 2 * t)


base = Image.open(OUT / f'counter{V}.png').convert('RGB')
g = json.loads((OUT / f'gpt{V}.json').read_text())
sc = json.loads((OUT / f'score{V}.json').read_text())
items = g['items']
LAT = g['latency_s']
# show order: safe items first (plates, bowls, mug), then the rest left -> right
order = sorted(range(len(items)), key=lambda i: (not items[i]['dishwasher_safe'], items[i]['box'][0]))
n_safe = sum(it['count'] for it in items if it['dishwasher_safe'])
n_stay = sum(it['count'] for it in items if not it['dishwasher_safe'])
fits = {f['gpt_idx']: f for f in sc['fits']}
summ = sc['summary']

T_RAW, T_LOOK, T_BOX_EACH, T_FIT, T_SUM = 2.0, 3.0, 0.85, 4.8, 7.5
t_look0 = T_RAW
t_box0 = t_look0 + T_LOOK
t_fit0 = t_box0 + T_BOX_EACH * len(items) + .6
t_sum0 = t_fit0 + T_FIT
T_END = t_sum0 + T_SUM


def tag_text(it):
    lab = f"{it['label_en']} · {it['label_zh']}"
    if it['dishwasher_safe']:
        return lab, '→ ' + ' '.join(it['slots'])
    return lab, f"{it['reason_en']} / {it['reason_zh']}"


# ---- tag layout (computed once): tags live in the empty wall strip (top) and cabinet strip (bottom), two staggered
# rows each, ordered by x so leader lines do not cross; each tag points at its box with a leader line
f_tag, f_tag2 = F(18), F(15, False)
dummy = ImageDraw.Draw(base)


def tsize(txt, f):
    b = dummy.textbbox((0, 0), txt, font=f)
    return b[2] - b[0], b[3] - b[1]


placed, anchor = {}, {}
TH = 46
for side in ('top', 'bot'):
    ids = [i for i in range(len(items)) if ((items[i]['box'][1] + items[i]['box'][3]) / 2 < 330) == (side == 'top')]
    ids.sort(key=lambda i: (items[i]['box'][0] + items[i]['box'][2]) / 2)
    rows_y = (6, 6 + TH + 6) if side == 'top' else (H - 30 - 8 - 2 * TH - 6, H - 30 - 8 - TH)
    right_edge = {0: 300 if side == 'top' else 8, 1: 300 if side == 'top' else 8}
    for k, i in enumerate(ids):
        it = items[i]
        l1, l2 = (f"{it['label_en']} · {it['label_zh']}",
                  ('→ ' + ' '.join(it['slots'])) if it['dishwasher_safe'] else f"{it['reason_en']} / {it['reason_zh']}")
        w = max(tsize(l1, f_tag)[0], tsize(l2, f_tag2)[0]) + 16
        cx = (it['box'][0] + it['box'][2]) / 2
        row = k % 2
        x = max(cx - w / 2, right_edge[row] + 8)
        x = min(x, W - w - 8)
        y = rows_y[row]
        placed[i] = (x, y, x + w, y + TH)
        right_edge[row] = x + w
        anchor[i] = (cx, it['box'][1] if side == 'top' else it['box'][3])


def tag_text(it):
    lab = f"{it['label_en']} · {it['label_zh']}"
    if it['dishwasher_safe']:
        return lab, '→ ' + ' '.join(it['slots'])
    return lab, f"{it['reason_en']} / {it['reason_zh']}"


def draw_box(ov, i, a, dim=1.):
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
    ty = r[3] if ay > r[3] else r[1]
    tx = min(max(ax, r[0] + 10), r[2] - 10)
    d.line((tx, ty, ax, ay), fill=col + (int(220 * a * dim),), width=2)
    d.ellipse((ax - 4, ay - 4, ax + 4, ay + 4), fill=col + (al,))
    d.rounded_rectangle(r, radius=7, fill=(16, 18, 22, int(215 * a * dim)), outline=col + (al,), width=2)
    l1, l2 = tag_text(it)
    d.text((r[0] + 8, r[1] + 4), l1, font=f_tag, fill=INK + (al,))
    d.text((r[0] + 8, r[1] + 25), l2, font=f_tag2, fill=(col if it['dishwasher_safe'] else (255, 150, 140)) + (al,))


def banner(ov, txt1, txt2, a, y=18, col=AMBER):
    d = ImageDraw.Draw(ov)
    f1, f2 = F(30), F(18, False)
    w = max(tsize(txt1, f1)[0], tsize(txt2, f2)[0]) + 48
    x = (W - w) // 2
    d.rounded_rectangle((x, y, x + w, y + 84), radius=14, fill=(14, 16, 20, int(225 * a)), outline=col + (int(255 * a),), width=2)
    d.text((x + 24, y + 10), txt1, font=f1, fill=INK + (int(255 * a),))
    d.text((x + 24, y + 52), txt2, font=f2, fill=(200, 205, 212, int(255 * a)))


def footer(ov, txt, a=1.):
    d = ImageDraw.Draw(ov)
    f = F(15, False)
    d.rectangle((0, H - 30, W, H), fill=(10, 11, 14, int(170 * a)))
    d.text((14, H - 25), txt, font=f, fill=(210, 214, 220, int(255 * a)))


FOOT = 'MuJoCo simulation · GPT = codex exec (ChatGPT subscription) on this single RGB image · poses from the depth channel'
counts_y = 250


def counter_chip(ov, k_safe, k_stay, a=1.):
    d = ImageDraw.Draw(ov)
    f = F(20)
    t1, t2 = f'dishwasher {k_safe}', f'stay out {k_stay}'
    w1, w2 = tsize(t1, f)[0] + 24, tsize(t2, f)[0] + 24
    x = 14
    d.rounded_rectangle((x, counts_y, x + w1, counts_y + 36), radius=8, fill=GREEN + (int(230 * a),))
    d.text((x + 12, counts_y + 5), t1, font=f, fill=(10, 30, 15, int(255 * a)))
    d.rounded_rectangle((x + w1 + 8, counts_y, x + w1 + 8 + w2, counts_y + 36), radius=8, fill=RED + (int(230 * a),))
    d.text((x + w1 + 20, counts_y + 5), t2, font=f, fill=(40, 8, 5, int(255 * a)))


def fit_layer(ov, t):
    d = ImageDraw.Draw(ov)
    fl = F(20)
    for k, (i, f) in enumerate(fits.items()):
        p = ease((t - .35 * k) / 1.1)
        if p <= 0:
            continue
        ring = [tuple(q) for q in f['ring_px']]
        m = max(2, int(len(ring) * p))
        d.line(ring[:m], fill=CYAN + (255,), width=4)
        cx, cy = f['center_px']
        d.line((cx - 9, cy, cx + 9, cy), fill=CYAN + (255,), width=3)
        d.line((cx, cy - 9, cx, cy + 9), fill=CYAN + (255,), width=3)
        if p > .6:
            a = int(255 * ease((p - .6) / .4))
            txt = f"pose ±{f['err_mm']:.1f} mm"
            tw, th = tsize(txt, fl)
            xs = [q[0] for q in ring]
            ys = [q[1] for q in ring]
            lx, ly = max(xs) + 8, (min(ys) + max(ys)) / 2 - 16
            if f['kind'] == 'plate' or lx + tw + 16 > W:
                lx = min(xs) - tw - 24
            d.rounded_rectangle((lx, ly, lx + tw + 16, ly + th + 14), radius=7, fill=(8, 30, 36, a), outline=CYAN + (a,), width=2)
            d.text((lx + 8, ly + 4), txt, font=fl, fill=(220, 250, 255, a))


def summary_panel(img, t):
    p = ease(t / .7)
    pw = 640
    x0 = int(W - pw * p)
    ov = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.rectangle((x0, 0, W, H), fill=PANEL + (240,))
    x = x0 + 28
    d.text((x, 18), f'{n_safe} into the dishwasher · {n_stay} stay on the counter', font=F(25), fill=INK)
    d.text((x, 56), f'{n_safe} 件进洗碗机 · {n_stay} 件留在台面', font=F(21), fill=(190, 196, 204))
    y = 100
    d.text((x, y), 'INTO THE DISHWASHER  进洗碗机', font=F(15), fill=GREEN)
    y += 24
    fr, fz = F(18), F(15, False)
    for i in order:
        it = items[i]
        if not it['dishwasher_safe']:
            continue
        cnt = f" ×{it['count']}" if it['count'] > 1 else ''
        d.ellipse((x, y + 7, x + 10, y + 17), fill=GREEN)
        d.text((x + 20, y), f"{it['label_en']}{cnt} · {it['label_zh']}", font=fr, fill=INK)
        d.text((x + 440, y + 2), ' '.join(it['slots']), font=fz, fill=GREEN)
        y += 28
    y += 12
    d.text((x, y), 'STAYS ON THE COUNTER  留在台面 — why / 原因', font=F(15), fill=(255, 130, 118))
    y += 26
    for i in order:
        it = items[i]
        if it['dishwasher_safe']:
            continue
        d.ellipse((x, y + 7, x + 10, y + 17), fill=RED)
        d.text((x + 20, y), f"{it['label_en']} · {it['label_zh']}", font=fr, fill=INK)
        d.text((x + 20, y + 21), f"{it['reason_en']} / {it['reason_zh']}", font=fz, fill=(255, 160, 150))
        y += 44
    fs = F(14, False)
    _pr = next(r for r in sc['rows'] if r['gt'] == 'plate')
    _pc = f"{items[_pr['gpt_idx']]['count'] if 'gpt_idx' in _pr else 0}/{_pr['gt_count']}"
    errs = ' / '.join(f"{f['err_mm']:.1f}" for f in fits.values())
    lines = [f"GPT vs ground truth: {summ['detected']}/{summ['gt_entries']} entries found ({summ['gt_objects']} objects, plate count {_pc})",
             f"labels {summ['category_ok']}/{summ['gt_entries']} · dishwasher-safe {summ['safe_ok']}/{summ['gt_entries']} · mean box IoU {summ['mean_iou']:.2f}",
             f"GPT latency {LAT:.1f} s (one call) · plate / bowl / bowl pose error {errs} mm"]
    y = max(y + 6, H - 30 - 8 - 20 * len(lines))
    for k, ln in enumerate(lines):
        d.text((x, y + 20 * k), ln, font=fs, fill=(170, 176, 186))
    img.alpha_composite(ov)


def frame(t):
    img = base.copy().convert('RGBA')
    ov = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    if t < t_look0:
        a = ease(t / .4)
        d = ImageDraw.Draw(ov)
        d.rounded_rectangle((18, 18, 418, 62), radius=10, fill=(14, 16, 20, int(200 * ease((t - .3) / .5))))
        d.text((32, 25), 'Counter camera · RGB-D  台面相机', font=F(21), fill=INK + (int(255 * ease((t - .3) / .5)),))
        img.alpha_composite(ov)
        if a < 1:
            img = Image.blend(Image.new('RGBA', (W, H), (0, 0, 0, 255)), img, a)
        footer(img, FOOT)
        return img
    if t < t_box0:
        u = (t - t_look0) / T_LOOK
        dim = Image.new('RGBA', (W, H), (0, 0, 0, int(70 * ease(u * 4))))
        img.alpha_composite(dim)
        d = ImageDraw.Draw(ov)
        sx = int((u * 2.2 % 1.0) * W)                      # scanning line
        for k in range(24):
            d.line((sx - k * 2, 0, sx - k * 2, H), fill=AMBER + (max(0, 110 - k * 5),), width=2)
        el = LAT * ease(u * 1.05)
        dots = '.' * (1 + int(t * 3) % 3)
        banner(ov, f'GPT is looking{dots}   GPT 识别中   {el:5.1f} s',
               f'real latency {LAT:.1f} s for one call · shown {LAT / T_LOOK:.0f}× faster', ease(u * 5))
        img.alpha_composite(ov)
        footer(img, FOOT)
        return img
    # boxes phase and later
    tb = t - t_box0
    k_shown = [i for n, i in enumerate(order) if tb >= n * T_BOX_EACH]
    fitting = t >= t_fit0
    for n, i in enumerate(order):
        a = ease((tb - n * T_BOX_EACH) / .35)
        if a <= 0:
            continue
        dimf = 1.
        if fitting and i not in fits:
            dimf = 1 - .6 * ease((t - t_fit0) / .5)
        draw_box(ov, i, a, dimf)
    ks = sum(items[i]['count'] for i in k_shown if items[i]['dishwasher_safe'])
    kn = sum(items[i]['count'] for i in k_shown if not items[i]['dishwasher_safe'])
    if t < t_sum0:
        counter_chip(ov, ks, kn)
    if fitting:
        fit_layer(ov, t - t_fit0)
        if t < t_sum0:
            banner(ov, 'Depth fit on plates & bowls   深度拟合',
                   'GPT box → depth back-projection → counter plane (RANSAC) → circle fit · error vs ground truth',
                   ease((t - t_fit0) / .4) * (1 - ease((t - t_sum0 + .3) / .3)), col=CYAN)
    img.alpha_composite(ov)
    if t >= t_sum0:
        summary_panel(img, t - t_sum0)
    footer(img, FOOT)
    return img


def main():
    n = int(round(T_END * FPS))
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
           str(OUT / f'recognition_card{V}.mp4')]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    last = None
    for k in range(n):
        im = frame(k / FPS).convert('RGB')
        p.stdin.write(im.tobytes())
        last = im
    p.stdin.close()
    p.wait()
    last.save(OUT / f'recognition_card{V}.png')
    for nm, tt in (('k_look', t_look0 + 1.5), ('k_boxes', t_fit0 - .2), ('k_fit', t_sum0 - .3)):
        frame(tt).convert('RGB').save(OUT / f'{nm}{V}.png')
    print(f'frames {n}  duration {T_END:.1f} s  rc={p.returncode}')


if __name__ == '__main__':
    main()
