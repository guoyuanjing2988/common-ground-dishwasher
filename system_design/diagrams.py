"""Diagrams for SYSTEM.md (matplotlib only). Run: /opt/homebrew/bin/python3 diagrams.py"""
import json, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, FancyArrowPatch, Polygon

import logging; logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
plt.rcParams["font.family"] = ["Hiragino Sans GB", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
HERE = pathlib.Path(__file__).parent

INK = "#1f2933"; MUTED = "#6b7280"; GRID = "#d1d5db"
C_COUNTER = "#e5e7eb"; C_DW = "#93c5fd"; C_CART = "#fbbf24"; C_DOCK = "#34d399"
C_AC = "#dc2626"; C_DC48 = "#2563eb"; C_DC24 = "#7c3aed"; C_LV = "#059669"; C_SAFE = "#ea580c"


def box(ax, x, y, w, h, text, fc="#ffffff", ec=INK, fs=8.5, lw=1.2, bold_first=True):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.008,rounding_size=0.015",
                                fc=fc, ec=ec, lw=lw))
    lines = text.split("\n")
    if bold_first:
        ax.text(x + w / 2, y + h - 0.012, lines[0], ha="center", va="top", fontsize=fs + 0.5,
                weight="bold", color=INK)
        if len(lines) > 1:
            ax.text(x + w / 2, y + h - 0.042, "\n".join(lines[1:]), ha="center", va="top",
                    fontsize=fs - 0.5, color=INK, linespacing=1.25)
    else:
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, color=INK)


def link(ax, p, q, label="", color=INK, lw=1.6, ls="-", style="-|>", fs=7.5, off=(0, 0.008),
         rad=0.0):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=10, color=color, lw=lw,
                                 linestyle=ls, connectionstyle=f"arc3,rad={rad}"))
    if label:
        mx, my = (p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1]
        ax.text(mx, my, label, ha="center", va="bottom", fontsize=fs, color=color,
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))


# ------------------------------------------------------------------ kitchen
def kitchen():
    fig, ax = plt.subplots(figsize=(11, 7.2))
    # world frame: x = out from the counter (aisle), y = along the counter
    # plot horizontal = y, vertical = x (counter at top -> flip x)
    def R(x0, y0, w_y, d_x, **kw):
        ax.add_patch(Rectangle((y0, -x0 - d_x), w_y, d_x, **kw))

    def P(x, y):
        return (y, -x)

    # counter run (extends past the sim's ±1.25 m to the park spot)
    R(-0.60, -1.25, 3.95, 0.60, fc=C_COUNTER, ec=INK, lw=1.2)
    ax.text(*P(-0.52, -1.20), "Counter 台面 z=0.90, depth 0.60\ncabinet fronts x=-0.02, plinth recess 75 mm",
            fontsize=8, color=MUTED, va="top")
    # back wall
    ax.plot([-1.25, 2.70], [0.60, 0.60], color=INK, lw=3)
    ax.text(0.9, 0.63, "back wall / backsplash", fontsize=8, color=MUTED, ha="center")
    # opposite side of the galley
    ax.add_patch(Rectangle((-1.25, -1.30), 3.95, 0.30, fc="#f3f4f6", ec=MUTED, hatch="//", lw=0.8))
    ax.text(0.7, -1.17, "opposite cabinets / wall — aisle ≥ 1.0 m needed for the turn (OPEN: measure the real kitchen)",
            fontsize=8, color=MUTED, ha="center", va="center")
    # dishwasher
    R(-0.39, -0.215, 0.43, 0.39, fc=C_DW, ec=INK, lw=1.2)
    ax.text(*P(-0.30, 0.0), "Dishwasher\nBREMED E804\n430×390", ha="center", va="center", fontsize=8)
    # open door (dashed)
    R(0.0, -0.215, 0.43, 0.39, fc="none", ec="#1d4ed8", ls="--", lw=1.2)
    ax.text(*P(0.20, 0.0), "door open\n(z 0.91-0.98,\nabove floor)", ha="center", va="center",
            fontsize=7, color="#1d4ed8")
    # water tank (assumed left of the machine)
    R(-0.45, -0.60, 0.28, 0.30, fc="#bae6fd", ec=INK, lw=0.8)
    ax.text(*P(-0.30, -0.46), "water\ntank", ha="center", va="center", fontsize=7)
    # plate + pod dish
    ax.add_patch(Circle(P(-0.15, 0.40), 0.10, fc="white", ec=INK, lw=1))
    ax.text(*P(-0.15, 0.40), "plate", ha="center", va="center", fontsize=7)
    ax.add_patch(Circle(P(-0.085, 0.655), 0.03, fc="#fde68a", ec=INK, lw=0.8))
    ax.text(*P(-0.04, 0.70), "pod", fontsize=7)
    # wall camera
    ax.plot(*P(-0.27, 0.42), marker="v", ms=11, color="#7c3aed")
    ax.text(*P(-0.36, 0.48), "wall node: RGB-D cam + Jetson\n(under wall cabinet, z 1.46)", fontsize=7.5,
            color="#7c3aed")
    # sockets
    def sock(x, y, label):
        ax.plot(*P(x, y), marker="s", ms=9, color=C_AC, mec=INK)
        ax.text(P(x, y)[0] + 0.05, P(x, y)[1] + 0.02, label, fontsize=7.5, color=C_AC)
    sock(-0.58, -0.32, "S1 dishwasher 13 A\n(backsplash, existing)")
    sock(-0.58, 0.90, "S3 wall node\n(USB-C PSU)")
    sock(-0.58, 2.62, "S2 charger\n(wall, low)")
    # work dock (cart M1)
    cart_c = (0.305, 0.42)
    R(cart_c[0] - 0.325, cart_c[1] - 0.30, 0.60, 0.65, fc=C_CART, ec=INK, lw=1.4, alpha=0.85)
    ax.plot(*P(0.48, 0.42), marker="o", ms=9, color=INK)
    ax.text(*P(0.70, 0.42), "WORK DOCK\narm base (0.48, 0.42, 0.80)\nyaw 195°", ha="center",
            va="top", fontsize=8, weight="bold")
    # dock plate on the plinth (locating V-blocks)
    for yy in (0.22, 0.62):
        ax.add_patch(Polygon([P(-0.06, yy - 0.03), P(-0.06, yy + 0.03), P(0.0, yy)], fc=C_DOCK, ec=INK))
    ax.text(*P(-0.10, 0.75), "dock V-blocks\non plinth", fontsize=7, color="#047857")
    # park / charge dock
    park_c = (0.305, 2.25)
    R(park_c[0] - 0.325, park_c[1] - 0.30, 0.60, 0.65, fc="#fde68a", ec=INK, lw=1.2, ls="--")
    for yy in (2.05, 2.45):
        ax.add_patch(Polygon([P(-0.06, yy - 0.03), P(-0.06, yy + 0.03), P(0.0, yy)], fc=C_DOCK, ec=INK))
    ax.add_patch(Rectangle((2.17, 0.0), 0.16, 0.025, fc="#111827"))
    ax.text(*P(0.62, 2.25), "PARK + CHARGE DOCK\nspring contacts + V-blocks", ha="center",
            va="top", fontsize=8, weight="bold")
    # drive path
    path = [(0.305, 0.42), (0.47, 0.42), (0.47, 2.25), (0.305, 2.25)]
    xs = [p[1] for p in path]; ys = [-p[0] for p in path]
    ax.plot(xs, ys, color=INK, lw=2, ls=(0, (5, 3)))
    for a, b in zip(path[:-1], path[1:]):
        ax.add_patch(FancyArrowPatch(P(*a), P(*b), arrowstyle="-|>", mutation_scale=14, color=INK,
                                     lw=0, shrinkA=30, shrinkB=30))
    ax.text(-1.24, -0.42, "Route (≈2.2 m each way):\n1. pull out 0.165 m\n2. turn 90° in place\n3. drive 1.83 m, ≤ 0.3 m/s\n"
            "4. turn 90°\n5. reverse 0.165 m, 0.05 m/s,\n   into V-blocks; push 100 N;\n   brake", fontsize=7.5,
            ha="left", va="top", bbox=dict(fc="white", ec=GRID))
    # rotation clearance circle
    import math
    rr = math.hypot(0.325, 0.30)
    ax.add_patch(Circle(P(0.47, 2.25), rr, fc="none", ec=MUTED, ls=":", lw=1))
    ax.add_patch(Circle(P(0.47, 0.42), rr, fc="none", ec=MUTED, ls=":", lw=1))
    ax.text(*P(0.97, 0.42), "turn-in-place sweep r=0.44 m (+arm stowed inside)", fontsize=7, color=MUTED, ha="center")
    # cable routes
    ax.plot([2.62, 2.62, 2.30], [0.58, 0.0, 0.0], color=C_AC, lw=2)
    ax.text(2.66, 0.25, "NPB-750-48 charger in\ncabinet / toe-kick;\nDC to plinth contacts", fontsize=7, color=C_AC)
    ax.plot([-0.32, -0.32], [0.58, 0.39], color=C_AC, lw=2)
    ax.plot([0.90, 0.42], [0.58, 0.27], color=C_AC, lw=1.5, ls="--")
    # Wi-Fi
    ax.text(2.75, -0.92, "Wi-Fi AP (home router)\nrobot ⇄ wall node ⇄ cloud (Codex)", fontsize=7.5,
            color="#7c3aed", ha="left")
    # axes
    ax.annotate("", xy=(3.0, -0.75), xytext=(3.0, -0.45),
                arrowprops=dict(arrowstyle="-|>", color=INK))
    ax.text(3.03, -0.68, "+x (aisle)", fontsize=8)
    ax.annotate("", xy=(3.3, -0.45), xytext=(3.0, -0.45),
                arrowprops=dict(arrowstyle="-|>", color=INK))
    ax.text(3.2, -0.41, "+y", fontsize=8)
    ax.set_xlim(-1.3, 3.45); ax.set_ylim(-1.35, 0.72)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("Kitchen top view 厨房俯视图 — park/charge, drive path, work dock, sockets (S1–S3), cables",
                 fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(HERE / "kitchen_topview.png", dpi=150)
    plt.close(fig)


DRIVE = ("2 × Oriental BLVD-KRD drivers (own DSP)\nspeed loop in driver, hall feedback\n"
         "• BLV-R 200 W motor + power-off brake + 50:1\n• brake auto-applies at stop / power loss\n"
         "• Modbus RTU (RS-485) 115.2 kbit/s, 50 Hz cmds")


def blocks():
    fig, ax = plt.subplots(figsize=(19, 12))
    ax.set_xlim(0, 1.4); ax.set_ylim(0, 0.92); ax.axis("off")
    # regions
    ax.add_patch(Rectangle((0.005, 0.005), 1.05, 0.80, fc="#fffbeb", ec="#d97706", lw=1.2, ls="--"))
    ax.text(0.015, 0.79, "ON THE ROBOT (battery 51.2 V)", fontsize=10, weight="bold", color="#b45309", va="top")
    ax.add_patch(Rectangle((1.07, 0.30), 0.325, 0.505, fc="#f5f3ff", ec="#7c3aed", lw=1.2, ls="--"))
    ax.text(1.08, 0.79, "FIXED IN KITCHEN (mains)", fontsize=10, weight="bold", color="#6d28d9", va="top")
    ax.add_patch(Rectangle((1.07, 0.005), 0.325, 0.27, fc="#f0f9ff", ec="#0284c7", lw=1.2, ls="--"))
    ax.text(1.08, 0.265, "CLOUD", fontsize=10, weight="bold", color="#0369a1", va="top")
    ax.text(0.0, 0.905, "Compute & bus block diagram 计算与总线框图 — which chip controls what (rates = loop rates)",
            fontsize=12, weight="bold", va="top")

    box(ax, 0.36, 0.55, 0.30, 0.22,
        "Main PC — OnLogic Helix 500 (x86)\nUbuntu 22.04 + PREEMPT_RT, ROS 2 Humble\n"
        "• libfranka FCI client: Cartesian impedance /\n  admittance 1 kHz (RT thread)\n"
        "• IK + joint-path planner 100 Hz (mink)\n• Task state machine (behaviour tree) 10 Hz\n"
        "• Docking / base path controller 50 Hz\n• Lidar scan matching 10 Hz",
        fc="#fef3c7", fs=8.5)
    box(ax, 0.02, 0.55, 0.27, 0.22,
        "FR3 Control (Franka, 19in box)\nproprietary RT OS, 230 V AC via inverter\n"
        "• joint torque loops 1 kHz (+ drives in joints)\n• safety: SLS/SLP, collision detect,\n  e-stop X3 input, PL d Cat 3\n"
        "• Franka Hand gripper control",
        fc="#fee2e2", fs=8.5)
    box(ax, 0.02, 0.35, 0.27, 0.15,
        "FR3 arm (7 joint drives + torque sensors)\n+ Franka Hand + conductive fingertip\n"
        "+ Bota SensONE wrist F/T 800 Hz",
        fc="#fee2e2", fs=8.5)
    box(ax, 0.36, 0.30, 0.30, 0.18,
        "Base MCU — STM32H723 (Nucleo-H723ZG)\nFreeRTOS + micro-ROS (UDP)\n"
        "• wheel velocity cmds 50 Hz → drivers (Modbus RTU)\n• cmd watchdog 100 ms → stop + brake\n"
        "• BMS (CAN), bumpers, brake & contactor status,\n  charge-contact detect, e-stop chain monitor",
        fc="#dcfce7", fs=8.5)
    box(ax, 0.72, 0.30, 0.31, 0.18, DRIVE, fc="#dcfce7", fs=8.5)
    box(ax, 0.72, 0.55, 0.31, 0.10,
        "2 × 2D lidar (RPLIDAR S2, 10 Hz)\ncabinet-edge line fit for docking, obstacle stop", fc="#ecfeff", fs=8.5)
    box(ax, 0.72, 0.67, 0.31, 0.10,
        "GbE switch (Advantech EKI-2725, 12-48 V)\n+ Wi-Fi 6E (Intel AX210 in main PC)", fc="#ecfeff", fs=8.5)
    box(ax, 0.72, 0.10, 0.31, 0.15,
        "BMS (in 16S LFP pack)\ncell V/T, SOC, charge/discharge FETs\nCAN 500 kbit/s", fc="#dbeafe", fs=8.5)
    box(ax, 0.02, 0.05, 0.62, 0.24,
        "HARDWARE SAFETY CHAIN (no software in the loop)\n"
        "e-stop buttons ×2 on cart + wireless e-stop receiver + safety edges (ASO SENTIR → ELMON relay)\n"
        "→ PILZ PNOZ s3 dual-channel safety relay\n"
        "   ├─ FR3 X3 e-stop input  → arm stop (Cat 1 → joint brakes)\n"
        "   ├─ drive contactor K2 opens → motor drivers unpowered, fail-safe wheel brakes apply\n"
        "   └─ aux contact → MCU (status only)",
        fc="#ffedd5", ec=C_SAFE, fs=8.5)
    box(ax, 1.085, 0.50, 0.295, 0.24,
        "Wall perception node\nJetson Orin Nano Super (JetPack 6.2)\n+ RealSense D435i (USB 3, 30 fps)\n"
        "• capture RGB-D, depth plane + circle fit\n• calls Codex (codex exec), ~15 s\n"
        "• fallback detector on-device\n  (Hough circle + depth RANSAC / small YOLO)\n• cam→robot extrinsic check per dock",
        fc="#ede9fe", fs=8.3)
    box(ax, 1.085, 0.32, 0.295, 0.13,
        "Charge dock\nMean Well NPB-750-48 (CC/CV 11.3 A)\n→ spring contacts on plinth", fc="#ede9fe", fs=8.3)
    box(ax, 1.085, 0.06, 0.295, 0.15,
        "Codex VLM (OpenAI cloud)\n2 calls / cycle, 11-19 s each\nRTT 20-60 ms + inference", fc="#e0f2fe", fs=8.3)

    link(ax, (0.36, 0.66), (0.29, 0.66), "GbE direct, FCI 1 kHz UDP\n(no switch, < 1 ms round trip)", color="#b91c1c", fs=7.2, off=(0, 0.01))
    link(ax, (0.155, 0.55), (0.155, 0.50), "arm cable", color="#b91c1c", style="<|-|>", fs=7)
    link(ax, (0.29, 0.43), (0.36, 0.63), "USB/serial F/T 800 Hz", color=INK, fs=7, off=(-0.03, -0.03))
    link(ax, (0.51, 0.55), (0.51, 0.48), "UDP/micro-ROS 100 Hz (via switch)", color=INK, style="<|-|>", fs=7.2, off=(0.0, -0.01))
    link(ax, (0.66, 0.39), (0.72, 0.39), "RS-485\nModbus RTU", color=C_LV, style="<|-|>", fs=7)
    link(ax, (0.60, 0.30), (0.75, 0.25), "CAN", color=C_LV, style="<|-|>", fs=7)
    link(ax, (0.72, 0.60), (0.66, 0.62), "Ethernet / USB", color=INK, fs=7)
    link(ax, (0.72, 0.72), (0.66, 0.72), "GbE", color=INK, style="<|-|>", fs=7)
    link(ax, (1.03, 0.73), (1.085, 0.66), "Wi-Fi (home AP)\nplate pose JSON", color="#7c3aed", style="<|-|>", fs=7, off=(0.0, 0.02))
    link(ax, (1.23, 0.50), (1.23, 0.21), "HTTPS (Wi-Fi/fibre)", color="#0369a1", style="<|-|>", fs=7)
    link(ax, (0.33, 0.29), (0.155, 0.35), "", color=C_SAFE, lw=2)
    link(ax, (0.55, 0.29), (0.60, 0.30), "", color=C_SAFE, lw=2)
    link(ax, (0.64, 0.20), (0.80, 0.30), "K2", color=C_SAFE, lw=2, fs=7)
    fig.savefig(HERE / "block_diagram.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def power():
    fig, ax = plt.subplots(figsize=(19, 11))
    ax.set_xlim(0, 1.4); ax.set_ylim(0, 0.86); ax.axis("off")
    ax.text(0.0, 0.855, "Power flow & wiring 电源与接线 — 51.2 V LiFePO4 bus; AC only for the FR3 Control", fontsize=12,
            weight="bold", va="top")
    b = lambda *a, **k: box(ax, *a, **k)
    b(0.01, 0.50, 0.15, 0.22, "BATTERY\n51.2 V 30 Ah LFP 16S\n(2 × 8S modules)\n1.54 kWh, 16 kg\nBMS: CAN, 60 A cont.,\nchg/dsg FETs, T sensors", fc="#dbeafe", fs=8.3)
    b(0.20, 0.58, 0.11, 0.12, "F0 main fuse\nClass T 100 A\n≥ 20 kA AIC", fc="#fff", fs=8)
    b(0.34, 0.58, 0.11, 0.12, "S0 manual\ndisconnect\n≥ 60 V DC, lockable", fc="#fff", fs=8)
    b(0.48, 0.58, 0.14, 0.12, "K1 main contactor\nAlbright SW80 (48 V coil)\n+ pre-charge 68 Ω/25 W\nvia MCU + key switch", fc="#fff", fs=7.8)
    ax.add_patch(Rectangle((0.65, 0.12), 0.012, 0.62, fc=C_DC48, ec="none"))
    ax.text(0.656, 0.755, "48 V BUS\n44-58.4 V", ha="center", fontsize=8.5, weight="bold", color=C_DC48)
    for a_, b_ in (((0.16, 0.64), (0.20, 0.64)), ((0.31, 0.64), (0.34, 0.64)), ((0.45, 0.64), (0.48, 0.64)), ((0.62, 0.64), (0.65, 0.64))):
        link(ax, a_, b_, color=C_DC48, lw=2.5, style="-")
    # branches
    rows = [(0.66, "F1 40 A", "INVERTER Victron Phoenix 48/800\n650 W cont / 1500 W peak, 9.5 W no-load", "230 V AC 50 Hz →\nFR3 Control (~80 W, peak ≤ 450 W)\n→ FR3 arm (joint brakes hold on power loss)", C_AC),
            (0.48, "F2 25 A", "K2 SAFETY CONTACTOR\n(PILZ PNOZ s3 output)", "2 × BLVD-KRD → BLV-R 200 W motors\nbrakes apply when K2 opens (fail-safe)", C_SAFE),
            (0.30, "F3 10 A", "DC-DC Mean Well RSD-300C-24\n33.6-62.4 V in, 300 W, isolated", "24 V rail: main PC 12-60 W, GbE switch,\nlidars (5 V buck), MCU, PNOZ, safety\nedges, Bota F/T, e-stop chain", C_DC24),
            (0.14, "F4 20 A", "CHARGE INLET\nspring contacts + ideal-diode\n+ BMS charge FET", "← dock: NPB-750-48 (230 V → 57.6 V, 11.3 A)\ncontacts live only when the dock's\npresence switch + charger see a battery", C_LV)]
    for y, fuse, mid, load, col in rows:
        link(ax, (0.662, y + 0.05), (0.70, y + 0.05), color=C_DC48, lw=2.2, style="-")
        b(0.70, y, 0.07, 0.10, fuse, fc="#fff", fs=8, bold_first=False)
        link(ax, (0.77, y + 0.05), (0.80, y + 0.05), color=col, lw=2.2, style="-")
        b(0.80, y, 0.24, 0.10, mid, fc="#fff", ec=col, fs=8)
        link(ax, (1.04, y + 0.05), (1.07, y + 0.05), color=col, lw=2.2)
        b(1.07, y, 0.32, 0.10, load, fc="#fff", ec=col, fs=7.8, bold_first=False)
    # mains side
    b(0.01, 0.12, 0.30, 0.25, "MAINS SOCKETS (230 V / 50 Hz, BS1363 13 A)\nS1 dishwasher (existing, own socket — robot never powers it)\n"
      "S2 charge dock: NPB-750-48 charger (≈ 0.75 kW max)\nS3 wall node: Jetson Orin Nano Super + D435i\n    (USB-C / 19 V brick, ≤ 25 W)\n\n"
      "Robot itself has NO mains cable (battery).\nPlan B (tether): 3-core 1.5 mm² cord on a\nspring reel → 230 V straight to FR3 + a\n48 V PSU; battery removed (see SYSTEM.md).", fc="#fef2f2", ec=C_AC, fs=8)
    b(0.34, 0.12, 0.27, 0.25, "SAFETY CHAIN (hardware)\nE-stop ×2 (cart) + wireless e-stop\n+ safety edges (ELMON relay)\n→ PILZ PNOZ s3 dual channel\n  ├ FR3 X3 e-stop input (arm stops, brakes)\n  ├ K2 opens (drives off, wheel brakes on)\n  └ aux → MCU\nK1 drops on: BMS fault, key-off,\nlow-voltage cutoff (44 V)", fc="#ffedd5", ec=C_SAFE, fs=8)
    link(ax, (0.61, 0.25), (0.80, 0.53), "", color=C_SAFE, lw=1.5, ls="--")
    ax.text(0.085, 0.47, "BMS ⇄ MCU over CAN", ha="center", fontsize=7.5, color=C_LV)
    fig.savefig(HERE / "power_flow.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    power()
    blocks()
    kitchen()
    print("ok")
