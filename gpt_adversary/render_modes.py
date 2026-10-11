#!/usr/bin/env python3
"""Key frames + a short clip per failure mode (re-run of its minimal scenario with a streamed-to-ffmpeg recorder), then
the montage out/failures.mp4 (title card + clip per mode, ffmpeg concat). One sim process at a time, via runw.sh.

  python render_modes.py            # all modes in triage_modes.json that have no clip yet
  python render_modes.py --montage  # only (re)build out/failures.mp4 from the existing clips
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT))
import search as SE  # noqa: E402

CLIPS = ROOT / 'out' / 'clips'
FONT = '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'


def title_card(path, mode, seconds=3.0):
    img = Image.new('RGB', (960, 540), (16, 20, 26))
    d = ImageDraw.Draw(img)
    fb, fm, fs = ImageFont.truetype(FONT, 34), ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 18)
    d.text((40, 40), f'{mode["id"]}  -  {mode["title"]}', fill=(255, 220, 140), font=fb)
    y = 110
    for ln in textwrap.wrap(mode['mechanism'], 78)[:6]:
        d.text((40, y), ln, fill=(230, 235, 240), font=fm)
        y += 32
    y += 10
    meta = (f'severity {mode["severity"]}/4   plausibility {mode["plausibility"]}/3   class: {mode["classification"]}   '
            f'({mode.get("novelty", "")})')
    d.text((40, y), meta, fill=(150, 200, 255), font=fs)
    y += 34
    for ln in textwrap.wrap('minimal scenario: ' + json.dumps(mode.get('minimal_params', {})), 95)[:5]:
        d.text((40, y), ln, fill=(190, 195, 200), font=fs)
        y += 26
    png = path.with_suffix('.png')
    img.save(png)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-loop', '1', '-i', str(png), '-t', str(seconds), '-r', '30',
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '26', str(path)], check=True)


def render(mode):
    CLIPS.mkdir(parents=True, exist_ok=True)
    fdir = ROOT / 'frames' / mode['id']
    fdir.mkdir(parents=True, exist_ok=True)
    scn = ROOT / 'scenarios' / 'modes' / f'{mode["id"]}.json'
    scn.parent.mkdir(parents=True, exist_ok=True)
    scn.write_text(json.dumps(dict(name=mode['id'], title=mode['title'], params=mode['minimal_params']), indent=1))
    out = ROOT / 'runs' / 'modes' / f'{mode["id"]}_video.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    clip = CLIPS / f'{mode["id"]}.mp4'
    cap = f'{mode["id"]}: {mode["title"]}  |  minimal scenario: {json.dumps(mode["minimal_params"])}'
    ref = json.loads((ROOT / mode['ref_run']).read_text())
    st = next((x for x in ref['steps'] if x['step'] == mode['clip_step']), ref['steps'][-1])
    rf = max(0., (st.get('t_start') or 0.) - 1.5)
    ru = rf + mode.get('clip_len_s', 1e9)
    extra = ['--video', str(clip), '--caption', cap[:330], '--record-from', str(rf), '--record-until', str(ru),
             '--speed', str(mode.get('speed', 4.)), '--keyframes', str(fdir)]
    res, rss, wall = SE.run_sim(scn, out, seed=mode.get('seed', 1), extra=extra)
    # hold the final state a moment is done inside run_scripted; perception images (what Codex saw) for perception-related modes
    for p in sorted((ROOT / 'runs' / f'{mode["id"]}_video' / 'codex').glob('*overlay.png'))[:2]:
        shutil.copy(p, fdir / f'codex_{p.name}')
    import classify as C
    cls = C.classify(res)
    print(f'{mode["id"]}: video run {cls["label"]} {cls.get("signature")} [{rss}, {wall}s] -> {clip}', flush=True)
    return dict(id=mode['id'], label=cls['label'], signature=cls.get('signature'), rss=rss, wall_s=wall)


def montage(modes):
    parts = []
    for m in modes:
        clip = CLIPS / f'{m["id"]}.mp4'
        if not clip.exists():
            continue
        tc = CLIPS / f'{m["id"]}_title.mp4'
        title_card(tc, m)
        parts += [tc, clip]
    lst = CLIPS / 'concat.txt'
    lst.write_text(''.join(f"file '{p}'\n" for p in parts))
    out = ROOT / 'out' / 'failures.mp4'
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', str(lst), '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p', '-crf', '27', '-r', '30', str(out)], check=True)
    print('montage ->', out, flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--montage', action='store_true')
    ap.add_argument('--only', nargs='*')
    a = ap.parse_args()
    modes = json.loads((ROOT / 'triage_modes.json').read_text())['modes']
    if not a.montage:
        log = []
        for m in modes:
            if a.only and m['id'] not in a.only:
                continue
            if (CLIPS / f'{m["id"]}.mp4').exists() and not a.only:
                continue
            log.append(render(m))
        (ROOT / 'logs' / 'render_modes.json').write_text(json.dumps(log, indent=1))
    montage([m for m in modes if m.get('in_montage', True)])
