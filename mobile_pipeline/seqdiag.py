#!/opt/homebrew/bin/python3
"""Sequence diagram of one mission from its message log (out/<run>_messages.jsonl.gz).

Event messages (commands, job results, brake, perception requests / results, panel events, e-stop chain) are drawn as
arrows between the controller lanes, one row per message (time printed on the left; rows are NOT to time scale).
Periodic streams (ArmState 500 Hz, ArmSetpoint 100 Hz, BaseState 100 Hz, BaseCmd 50 Hz, LidarFix 2 x 10 Hz, ...) are
summarised in the side table with their message counts and mean rates. Supervisor state transitions are drawn as
labelled marks on the TaskSupervisor lane.

  /opt/homebrew/bin/python3 seqdiag.py out/mission_video_messages.jsonl.gz sequence_diagram.png
"""
import gzip
import json
import sys
from collections import Counter

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LANES = ['Operator', 'TaskSupervisor', 'MotionPlanner', 'ArmController', 'BaseMCU', 'Perception', 'DishwasherPanel']
STREAMS = {'ArmState', 'ArmSetpoint', 'BaseCmd', 'BaseState', 'LidarFix', 'BMS', 'ArmStatus', 'PanelLEDs', 'PlannerStatus'}
COL = dict(Command='#1f5fa8', JobDone='#2a8a3e', Brake='#8a5a00', PerceiveRequest='#7a3fa8', PerceiveResult='#7a3fa8',
           PanelEvent='#c06000', PanelReadout='#c06000', EStop='#c0392b', EStopButton='#c0392b', EStopReleased='#c0392b',
           OperatorAck='#c0392b', Pause='#c0392b', Resume='#c0392b', Cancel='#c0392b')


def label(r):
    i = r.get('info') or {}
    t = r['type']
    if t == 'Command':
        return f"call {i.get('skill') or i.get('job', '')}({i.get('button', '')})".replace('()', '')
    if t == 'JobDone':
        return f"result {i.get('skill') or i.get('job', '')}: {'ok' if i.get('ok') else 'FAIL'}"
    if t == 'Brake':
        return f"Brake {'engage' if i.get('engage') else 'release'}"
    if t == 'PerceiveRequest':
        return f"PerceiveRequest {i.get('tag', '')}"
    if t == 'PerceiveResult':
        return f"PerceiveResult ({i.get('codex_s')} s)"
    if t == 'PanelEvent':
        return f"PanelEvent {i.get('event', '')} -> {i.get('state', '')}"
    if t == 'PanelReadout':
        return f"LED readout {i.get('state', '')} {i.get('program') or ''}".strip()
    if t == 'EStopButton':
        return 'E-stop button ' + ('PRESSED' if i.get('pressed') else 'released')
    if t == 'OperatorAck':
        return f"OperatorAck {i.get('action', '')}"
    return t


def main(path, out):
    events, transitions, cnt, t_end = [], [], Counter(), 0.
    with gzip.open(path, 'rt') as fh:
        for line in fh:
            r = json.loads(line)
            t_end = max(t_end, r['t'])
            if r['type'] in STREAMS:
                cnt[(r['from'], r['to'], r['type'])] += 1
                continue
            if r['type'] == 'StateTransition':
                transitions.append(r)
                continue
            if r['type'] == 'PanelReadout' and not r.get('info'):
                continue
            if r['from'] not in LANES or r['to'] not in LANES:
                continue
            events.append(r)
    rows = sorted([('ev', e['t'], e) for e in events] + [('st', s['t'], s) for s in transitions], key=lambda x: (x[1], x[0] == 'ev'))
    n = len(rows)
    fig = plt.figure(figsize=(17, max(10, .155 * n + 2)))
    ax = fig.add_axes([.10, .02, .62, .95])
    xs = {l: i for i, l in enumerate(LANES)}
    for l, x in xs.items():
        ax.plot([x, x], [-1, n], color='#bbbbbb', lw=1, zorder=0)
        ax.text(x, -1.5, l, ha='center', va='bottom', fontsize=9, fontweight='bold')
    for k, (kind, t, r) in enumerate(rows):
        ax.text(-.65, k, f'{t:7.2f} s', ha='right', va='center', fontsize=6.5, family='monospace', color='#444444')
        if kind == 'st':
            i = r.get('info', {})
            ax.plot([xs['TaskSupervisor']], [k], marker='s', ms=4, color='#333333')
            ax.text(xs['TaskSupervisor'] - .08, k, f"{i.get('to', '')}", ha='right', va='center', fontsize=6.5, color='#333333',
                    fontweight='bold')
            continue
        a, b = xs[r['from']], xs[r['to']]
        c = COL.get(r['type'], '#555555')
        ax.annotate('', xy=(b, k), xytext=(a, k), arrowprops=dict(arrowstyle='->', color=c, lw=1.1))
        lat = (r['td'] - r['t']) * 1000
        ax.text((a + b) / 2, k - .18, f"{label(r)}  [{lat:.0f} ms]", ha='center', va='bottom', fontsize=6.2, color=c)
    ax.set_xlim(-1.3, len(LANES) - .4)
    ax.set_ylim(n + .5, -2.5)
    ax.axis('off')
    tx = fig.add_axes([.74, .02, .25, .95])
    tx.axis('off')
    lines = ['PERIODIC STREAMS (not drawn as arrows)', f'mission length {t_end:.1f} s sim', '']
    for (s, d, ty), c in sorted(cnt.items(), key=lambda x: -x[1]):
        lines.append(f'{ty:13s} {s} -> {d}')
        lines.append(f'      {c:7d} msgs  ~{c / max(t_end, 1e-9):6.1f} Hz')
    lines += ['', f'event messages drawn: {len(events)}', f'supervisor transitions: {len(transitions)}', '',
              'Arrow label: message type + link latency.', 'Rows are ordered in time, not to scale.']
    tx.text(0, 1, '\n'.join(lines), va='top', ha='left', fontsize=8, family='monospace')
    fig.suptitle(f'Message sequence of one mission ({path.split("/")[-1]})', fontsize=11, y=.995)
    fig.savefig(out, dpi=110)
    print('wrote', out, 'rows', n)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
