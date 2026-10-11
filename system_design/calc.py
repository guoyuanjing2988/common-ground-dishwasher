"""Design calculations for the dishwasher-loading mobile manipulator.

Pure arithmetic (no simulation). Inputs are the component numbers in
COMPONENTS / BASE below; every value is tagged V (verified, see SYSTEM.md
sources), E (estimated) or S (sim-derived). Writes calc_out.json, which
SYSTEM.md and spec.json quote.

Run: /opt/homebrew/bin/python3 calc.py
"""
import json, math, pathlib

G = 9.81
HERE = pathlib.Path(__file__).parent

# ---------------------------------------------------------------- geometry
# Base frame: origin on the floor at the footprint centre, +x = towards the
# aisle when docked (the cabinet side is -x), +y along the counter.
FOOT_X, FOOT_Y = 0.65, 0.60            # S (sim cart footprint)
WHEEL_R = 0.10                         # E 200 mm PU drive wheel on the gearhead (design choice)
TRACK = 0.50                           # E (wheel centre to centre)
# Drive wheels on the y-axis through the centre; 4 casters 60 mm inside the
# footprint corners (same castor inset as statics.py).
CASTER_INSET = 0.06
CASTER_X = FOOT_X / 2 - CASTER_INSET   # 0.265
CASTER_Y = FOOT_Y / 2 - CASTER_INSET   # 0.24

# ---------------------------------------------------------------- masses
# (name, kg, x, y, z of the part's own CoG in the base frame, tag)
# Arm mount: sim puts the arm base 0.175 m towards the aisle from the
# footprint centre (base at x=0.48 world, footprint centre at x=0.305 world).
ARM_MOUNT_X = 0.175
MASSES = [
    # Equipment envelope = sim cart: chassis box z 0.06-0.32 over the whole footprint + 0.18x0.18 column to 0.80.
    ("FR3 arm, stowed (CoG ~0.25 m above flange, over the deck)", 18.3, ARM_MOUNT_X - 0.05, 0.0, 0.80 + 0.25, "V mass (Franka datasheet) / E CoG"),
    ("Franka Hand + conductive fingertip", 0.75, ARM_MOUNT_X - 0.10, 0.0, 1.25, "V 0.73 kg"),
    ("Wrist F/T Bota SensONE (serial/USB)", 0.21, ARM_MOUNT_X - 0.10, 0.0, 1.20, "V"),
    ("Column 0.18x0.18x0.48 m steel box + flanges", 14.0, ARM_MOUNT_X, 0.0, 0.56, "E"),
    ("FR3 Control (19in, 483x355x89, upper bay)", 7.0, -0.08, 0.0, 0.27, "V"),
    ("Inverter Victron Phoenix 48/800 230V", 5.5, -0.18, -0.17, 0.26, "V"),
    ("Battery 51.2 V 30 Ah LFP, 2 x 8S modules + BMS (lower bay)", 16.0, -0.02, 0.0, 0.13, "E (Leoch 30 Ah ~16 kg)"),
    ("2 x drive module (BLV-R 200 W + brake + 50:1 GFV + 200 mm wheel + bearing block + spring arm)", 2 * 6.5, 0.0, 0.0, 0.10, "E"),
    ("4 x caster Tente 100 mm", 4 * 1.1, 0.0, 0.0, 0.07, "E"),
    ("Chassis: welded steel tube frame + Al deck + covers", 28.0, 0.0, 0.0, 0.18, "E"),
    ("Compute, DC-DC, safety, switch, wiring", 6.0, -0.15, 0.12, 0.25, "E"),
    ("Bumpers/safety edges, 2 lidars, dock pins, charge collector", 5.0, 0.0, 0.0, 0.20, "E"),
    ("Ballast steel plate (lower bay, aisle half)", 15.0, 0.12, 0.0, 0.08, "E design choice"),
]


def mass_props(masses):
    m = sum(p[1] for p in masses)
    cx = sum(p[1] * p[2] for p in masses) / m
    cy = sum(p[1] * p[3] for p in masses) / m
    cz = sum(p[1] * p[4] for p in masses) / m
    return m, cx, cy, cz


# ---------------------------------------------------------------- drive
V_MAX = 0.50          # m/s (spec) — software limit 0.30 near people
A_MAX = 0.50          # m/s^2 commanded
CRR = 0.02            # E rolling resistance, PU tyres + casters on tile
MU_FLOOR = 0.5        # E rubber/PU on dry ceramic tile (wet: ~0.3)
DOCK_PRELOAD = 100.0  # N, design: drive pushes into the V-blocks, then brakes apply (> 2.5x the 39 N pull-away)
STEP_H = 0.005        # m, design threshold / tile lip (route must be flat; max by traction reported)
CASTER_R = 0.05       # m (100 mm caster)
MOTOR_T_RATED = 27.4  # V N*m at gearhead output, Oriental BLMR6200SKM + GFV6G50 (continuous)
MOTOR_T_PEAK = 55.0   # V N*m peak at output
MOTOR_RPM_RATED = 60.0  # V output rpm at 3000 motor rpm (80 at 4000)
GEAR = 50.0
BRAKE_T = 27.0        # calc: 0.637 N*m motor brake (V) x 50, capped at gearhead rating (verify with Oriental)

# ---------------------------------------------------------------- power
# Loads in W at the load; path efficiency applied below. idle = powered, not moving.
INV_EFF, INV_NOLOAD = 0.90, 9.5     # V Phoenix 48/800: max eff 91 %, no-load 9.5 W
DCDC_EFF = 0.92                     # V Mean Well RSD-300C-24 ~92 %
LOADS = {  # name: (path, idle_W, active_W, peak_W, tag)
    "FR3 arm + Control (AC)":          ("ac",   60.0,  80.0, 450.0, "V ~80 W datasheet; idle E; peak 450 W reseller (UNVERIFIED)"),
    "Main PC OnLogic Helix 500":       ("24",   12.0,  30.0,  60.0, "E (60 W stress, AnandTech)"),
    "2 x RPLIDAR S2 (spun down docked)": ("24",  0.5,   5.0,   6.0, "V/R >2 W each"),
    "GbE switch EKI-2725":             ("24",    4.0,   4.0,   5.0, "E"),
    "Base MCU STM32H723 + RS-485/CAN": ("24",    1.0,   1.0,   1.5, "E"),
    "PILZ PNOZ s3 + safety-edge relay":("24",    2.5,   2.5,   2.5, "E"),
    "Bota SensONE F/T":                ("24",    1.0,   1.0,   1.0, "E"),
    "2 x BLVD-KRD drivers (standby)":  ("48",    8.0,   8.0,   8.0, "E"),
    "2 x BLV-R drive (driving)":       ("48",    0.0,  45.0, 2*200.0/0.85, "calc: 46 W mech peak accel, 13 W cruise; 200 W rated each"),
    "2 x motor brake coils (released)":("48",    0.0,  12.0,  12.0, "E"),
}
BATT_V, BATT_AH = 51.2, 30.0
DOD = 0.80
# Cycle phases (s) and which loads are 'active' in each. Idle-but-powered otherwise.
PHASES = [
    ("wake: inverter on, FR3 boot + self-test", 90.0, []),
    ("undock + drive out + dock", 45.0, ["2 x RPLIDAR", "2 x BLV-R", "brake coils", "Main PC"]),
    ("arm task (185 s sim)", 185.0, ["FR3", "Main PC", "Bota"]),
    ("2 Codex waits (arm holding)", 32.0, ["Main PC"]),
    ("undock + drive back + dock", 45.0, ["2 x RPLIDAR", "2 x BLV-R", "brake coils", "Main PC"]),
]
SLEEP_W = 6.0   # E: on the dock, inverter + FR3 + drivers off; MCU, BMS, PC suspended (wake-on-LAN)


def bus_w(load, which):
    path, idle, act, peak, _ = LOADS[load]
    w = {"idle": idle, "active": act, "peak": peak}[which]
    if path == "ac":
        return w / INV_EFF
    if path == "24":
        return w / DCDC_EFF
    return w


def phase_power(active):
    tot = INV_NOLOAD
    for k in LOADS:
        tot += bus_w(k, "active" if any(a in k for a in active) else "idle")
    return tot


def main():
    out = {}
    m, cx, cy, cz = mass_props(MASSES)
    out["mass_total_kg"] = round(m, 1)
    out["cog_m"] = [round(cx, 3), round(cy, 3), round(cz, 3)]
    m_base = m - sum(p[1] for p in MASSES[:3])
    out["mass_base_excl_arm_hand_ft_kg"] = round(m_base, 1)

    # tipping during driving (arm stowed). Tip edges: caster lines.
    # x-direction (fore/aft) when the cart drives along x: edge at +-CASTER_X
    edge_fwd = CASTER_X - cx      # braking while driving +x: tip about front casters
    edge_back = CASTER_X + cx
    a_tip_fwd = G * edge_fwd / cz
    a_tip_back = G * edge_back / cz
    a_skid = MU_FLOOR * G
    out["tip"] = {
        "edge_distance_front_m": round(edge_fwd, 3),
        "edge_distance_rear_m": round(edge_back, 3),
        "edge_distance_side_m": round(CASTER_Y - abs(cy), 3),
        "a_to_tip_fwd_m_s2": round(a_tip_fwd, 2),
        "a_to_tip_rear_m_s2": round(a_tip_back, 2),
        "a_to_tip_side_m_s2": round(G * (CASTER_Y - abs(cy)) / cz, 2),
        "a_wheel_skid_limit_m_s2": round(a_skid, 2),
        "commanded_a_m_s2": A_MAX,
        "safety_factor_vs_commanded": round(min(a_tip_fwd, a_tip_back) / A_MAX, 1),
        "tip_possible_by_braking": min(a_tip_fwd, a_tip_back) < a_skid,
        "static_tilt_to_tip_deg": round(math.degrees(math.atan(min(edge_fwd, edge_back) / cz)), 1),
    }
    # e-stop with brakes biting: decel limited by min(brake torque, tyre friction)
    # Only the 2 driven wheels brake; their share of the weight ~ fraction below.
    w_drive_frac = 0.55  # E (driven wheels sprung so they carry ~55 %)
    n_drive = m * G * w_drive_frac
    f_brake_tyre = MU_FLOOR * n_drive
    f_brake_torque = 2 * BRAKE_T / WHEEL_R
    f_brake = min(f_brake_tyre, f_brake_torque)
    a_estop = f_brake / m
    out["estop"] = {
        "driven_wheel_normal_load_N": round(n_drive),
        "brake_force_tyre_limit_N": round(f_brake_tyre),
        "brake_force_torque_limit_N": round(f_brake_torque),
        "decel_m_s2": round(a_estop, 2),
        "stop_distance_from_0.5_m_s_mm": round(1000 * V_MAX**2 / (2 * a_estop), 0),
        "stop_distance_from_0.3_m_s_mm": round(1000 * 0.3**2 / (2 * a_estop), 0),
        "tip_margin_factor": round(min(a_tip_fwd, a_tip_back) / a_estop, 2),
    }
    # holding while docked: sim needs 39 N pull-away + 58 N side (statics.json)
    out["dock_hold"] = {
        "required_pull_away_N": 39.3, "required_side_N": 58.4,
        "brake_capacity_N": round(f_brake),
        "utilisation": round(math.hypot(39.3 + DOCK_PRELOAD, 58.4) / f_brake, 2),
    }

    # drive torque per wheel
    f_roll = CRR * m * G
    f_acc = m * A_MAX
    t_cruise = f_roll * WHEEL_R / 2
    t_acc = (f_roll + f_acc) * WHEEL_R / 2
    t_dock = (f_roll + DOCK_PRELOAD) * WHEEL_R / 2
    # step climb: horizontal force to lift a caster of radius r over h
    n_caster = m * G * (1 - w_drive_frac) / 2  # two leading casters share
    k = math.sqrt(2 * CASTER_R * STEP_H - STEP_H**2) / (CASTER_R - STEP_H)
    f_step = 2 * n_caster * k
    t_step = (f_step + f_roll) * WHEEL_R / 2
    traction = MU_FLOOR * n_drive
    wheel_rpm = V_MAX / WHEEL_R * 60 / (2 * math.pi)
    out["drive"] = {
        "rolling_force_N": round(f_roll, 1),
        "torque_cruise_Nm_per_wheel": round(t_cruise, 2),
        "torque_accel_Nm_per_wheel": round(t_acc, 2),
        "torque_dock_preload_Nm_per_wheel": round(t_dock, 2),
        "step_height_m": STEP_H,
        "step_force_both_leading_casters_N": round(f_step),
        "torque_step_Nm_per_wheel": round(t_step, 2),
        "traction_limit_N": round(traction),
        "step_climb_traction_ok": f_step + f_roll < traction,
        "max_step_by_traction_m": None,
        "wheel_rpm_at_vmax": round(wheel_rpm, 1),
        "gear_ratio": GEAR,
        "motor_rpm_at_vmax": round(wheel_rpm * GEAR),
        "wheel_force_rated_N": round(MOTOR_T_RATED / WHEEL_R), "wheel_force_peak_N": round(MOTOR_T_PEAK / WHEEL_R),
        "vmax_capability_m_s": round(80 * 2 * math.pi * WHEEL_R / 60, 2),
        "mech_power_accel_at_vmax_W": round((f_roll + f_acc) * V_MAX, 1),
        "mech_power_cruise_W": round(f_roll * V_MAX, 1),
    }
    # largest step the traction allows
    lo, hi = 0.0, CASTER_R
    for _ in range(60):
        h = (lo + hi) / 2
        kk = math.sqrt(2 * CASTER_R * h - h * h) / (CASTER_R - h)
        if 2 * n_caster * kk + f_roll < traction:
            lo = h
        else:
            hi = h
    out["drive"]["max_step_by_traction_m"] = round(lo, 4)

    # energy per cycle
    rows, e = [], 0.0
    for name, t, act in PHASES:
        pw = phase_power(act)
        e += pw * t / 3600
        rows.append({"phase": name, "s": t, "bus_W": round(pw, 1), "Wh": round(pw * t / 3600, 2)})
    e_batt = BATT_V * BATT_AH * DOD
    peak = INV_NOLOAD + sum(bus_w(k, "peak") for k in LOADS)
    idle = phase_power([])
    out["power_table"] = [{"load": k, "path": v[0], "idle_W": v[1], "active_W": v[2], "peak_W": round(v[3], 1),
                           "bus_idle_W": round(bus_w(k, "idle"), 1), "bus_active_W": round(bus_w(k, "active"), 1),
                           "tag": v[4]} for k, v in LOADS.items()]
    out["energy"] = {
        "phases": rows,
        "cycle_time_s": sum(r["s"] for r in rows),
        "energy_per_cycle_Wh": round(e, 1),
        "bus_idle_awake_W": round(idle, 1),
        "bus_peak_W_all_peaks_coincident": round(peak),
        "bus_peak_A_at_48V": round(peak / 48.0, 1),
        "battery_nominal_Wh": round(BATT_V * BATT_AH),
        "battery_usable_Wh": round(e_batt),
        "cycles_per_charge": int(e_batt // e),
        "hours_awake_idle_off_charger": round(e_batt / idle, 1),
        "charger_A": 11.3, "charge_time_20_to_100pct_h": round(0.8 * BATT_AH / 11.3 + 0.4, 1),
        "recharge_per_cycle_min": round(e / (11.3 * 54.0 * 0.93) * 60, 1),
        "sleep_on_dock_W": SLEEP_W,
    }
    # pessimistic: FR3 draws 250 W average while moving (datasheet only says ~80 W)
    extra = (250.0 - LOADS["FR3 arm + Control (AC)"][2]) / INV_EFF * 185.0 / 3600
    out["energy"]["energy_per_cycle_Wh_pessimistic_FR3_250W"] = round(e + extra, 1)
    out["energy"]["cycles_per_charge_pessimistic"] = int(e_batt // (e + extra))
    (HERE / "calc_out.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
