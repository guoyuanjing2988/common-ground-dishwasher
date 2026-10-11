"""Message bus between the controller modules ("chips"). Every message has a sender, a receiver, a type and a simulated
link latency; it is delivered at the first scheduler step at or after t_send + latency. Every message is logged (one
JSON line: time, delivery time, from -> to, type, sequence number, optional short info) - the log is streamed to disk
(gzip), nothing is kept in memory except the in-flight queue and per-link counters.

Link latencies (all EST, stated in MOBILE_RESULTS.md):
  ArmController <-> MotionPlanner    1 ms   (arm control box <-> main computer, 1 kHz real-time Ethernet, FCI-like)
  BaseMCU      <-> MotionPlanner     2 ms   (CAN / real-time Ethernet)
  BaseMCU       -> ArmController     1 ms   (hard-wired safety chain: e-stop)
  TaskSupervisor <-> MotionPlanner   1 ms   (same computer, IPC)
  Perception    -> MotionPlanner     5 ms   (lidar driver + scan processing)
  Perception    -> TaskSupervisor    5 ms
  DishwasherPanel -> Perception    100 ms   (panel LEDs read optically by a camera: frame + processing)
  wall camera   -> Perception       40 ms   (kitchen camera over Wi-Fi) - Codex adds its real wall-clock latency
  default                            2 ms
"""
from __future__ import annotations

import gzip
import heapq
import itertools
import json
from collections import Counter, defaultdict

LATENCY = {
    ('ArmController', 'MotionPlanner'): .001, ('MotionPlanner', 'ArmController'): .001,
    ('BaseMCU', 'MotionPlanner'): .002, ('MotionPlanner', 'BaseMCU'): .002,
    ('BaseMCU', 'ArmController'): .001,
    ('TaskSupervisor', 'MotionPlanner'): .001, ('MotionPlanner', 'TaskSupervisor'): .001,
    ('Perception', 'MotionPlanner'): .005, ('MotionPlanner', 'Perception'): .002,
    ('Perception', 'TaskSupervisor'): .005,
    ('DishwasherPanel', 'Perception'): .100,
    ('WallCamera', 'Perception'): .040,
}
DEFAULT_LATENCY = .002
STREAMS = {'ArmState', 'ArmSetpoint', 'BaseCmd', 'Odom', 'BaseState', 'LidarFix', 'BMS', 'ArmStatus', 'PanelLEDs',
           'PlannerStatus'}   # periodic streams (shown as bands in the sequence diagram)


class Msg:
    __slots__ = ('t', 'td', 'src', 'dst', 'typ', 'data', 'seq')

    def __init__(self, t, td, src, dst, typ, data, seq):
        self.t, self.td, self.src, self.dst, self.typ, self.data, self.seq = t, td, src, dst, typ, data, seq


class Bus:
    def __init__(self, log_path=None):
        self.q = []
        self.inbox = defaultdict(list)
        self.seq = itertools.count()
        self.counts = Counter()
        self.fh = None
        if log_path is not None:
            self.fh = gzip.open(log_path, 'wt', compresslevel=5) if str(log_path).endswith('.gz') else open(log_path, 'w')

    def send(self, t, src, dst, typ, data=None, info=None, lat=None):
        lat = LATENCY.get((src, dst), DEFAULT_LATENCY) if lat is None else lat
        s = next(self.seq)
        m = Msg(t, t + lat, src, dst, typ, data, s)
        heapq.heappush(self.q, (m.td, s, m))
        self.counts[(src, dst, typ)] += 1
        if self.fh is not None:
            extra = '' if info is None else ',"info":' + json.dumps(info, default=str, separators=(',', ':'))
            self.fh.write(f'{{"t":{t:.4f},"td":{t + lat:.4f},"from":"{src}","to":"{dst}","type":"{typ}","seq":{s}{extra}}}\n')
        return m

    def deliver(self, t):
        q = self.q
        while q and q[0][0] <= t + 1e-9:
            m = heapq.heappop(q)[2]
            self.inbox[m.dst].append(m)

    def recv(self, dst):
        msgs = self.inbox[dst]
        if msgs:
            self.inbox[dst] = []
        return msgs

    def summary(self):
        out = defaultdict(int)
        for (s, d, t), n in self.counts.items():
            out[f'{s} -> {d}: {t}'] += n
        return dict(sorted(out.items()))

    def close(self):
        if self.fh is not None:
            self.fh.close()
            self.fh = None
