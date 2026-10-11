"""BOM with sources. USD. tag: V = vendor/manufacturer price, R = reseller listing, E = estimate/quote needed.
Run: /opt/homebrew/bin/python3 bom.py -> bom.json + markdown table on stdout."""
import json, pathlib
H = pathlib.Path(__file__).parent
USD_SGD = 1.30  # E, approximate exchange rate
B = [
 # group, item, qty, unit USD, tag, source
 ("Arm", "Franka Research 3 arm + FR3 Control (FCI incl.)", 1, 29218, "R", "https://vention.io/parts/franka-robotics-fr3-4193"),
 ("Arm", "Franka Hand gripper", 1, 1777, "R", "https://vention.io/parts/franka-robotics-fr3-franka-hand-4422"),
 ("Arm", "Bota SensONE wrist F/T (serial/USB)", 1, 5430, "R", "https://www.roboticscenter.ai/store/product/bota-sensone"),
 ("Arm", "Conductive silicone fingertip (6-8 mm) + braided ground lead", 1, 25, "R/E", "https://www.newegg.com/p/0SA-00ZV-000U8"),
 ("Drive", "Oriental BLMR6200SKM-GFV-F + GFV6G50 + BLVD-KRD (200 W, power-off brake)", 2, 1373, "V", "https://catalog.orientalmotor.com/item/blv-r-series-brushless-dc-motor-speed-control/shop-blv-r-series-brushless-dc-motors/blmr6200skm-gfv-f-gfv6g50-blvd-krd"),
 ("Drive", "200 mm PU drive wheel, axle bearing block, coupling, sprung swing arm", 2, 200, "E", "-"),
 ("Drive", "Swivel caster 100 mm PU, 100 kg (Tente 1044UAP100R26 class)", 4, 40, "E", "https://cdn.tente.com/datasheets/1044UAP100R26_28S45RAL9002BH139_en.pdf"),
 ("Structure", "Welded steel chassis, Al deck, covers (IPX4 top), 0.80 m column, arm flange", 1, 1500, "E", "-"),
 ("Structure", "Ballast plate 15 kg", 1, 50, "E", "-"),
 ("Structure", "Dock V-blocks + cone pins (work dock + charge dock)", 2, 120, "E", "-"),
 ("Power", "Battery 51.2 V 30 Ah LFP (2 x 8S), BMS with CAN, IEC 62619 / UN38.3", 1, 900, "E (quote)", "https://www.leoch.com/uploads/ueditor/file/20231111/6383531340936257313889595.pdf"),
 ("Power", "Victron Phoenix 48/800 230V VE.Direct UK", 1, 260, "R", "https://www.victronenergy.com/upload/documents/Datasheet-Phoenix-Inverter-VE.Direct-250VA-1200VA-EN.pdf"),
 ("Power", "Mean Well RSD-300C-24 DC-DC", 1, 130, "R", "https://my.rs-online.com/web/p/dc-dc-converters/7704087"),
 ("Power", "Albright SW80 48 V contactor (K1) + safety contactor K2", 2, 90, "R", "https://ph.fridayparts.com/48v-100a-aftermarket-albright-sw80-10-single-acting-solenoid-contactor-for-industrial-automation-equipment"),
 ("Power", "Class T 100 A fuse + block, branch fuses (>=60 VDC), disconnect, pre-charge, cable, lugs", 1, 450, "E", "https://www.bluesea.com/products/5007100/Class_T_Fuse_Block_with_Insulating_Cover_-_110_to_200A"),
 ("Power", "Charge dock: Mean Well NPB-750-48 charger", 1, 220, "R", "https://www.jameco.com/z/NPB-750-48-MEAN-WELL-759-4W-55-2V-11-3A-Ultra-Wide-Output-Range-Intelligent-Battery-Charger_2334711.html"),
 ("Power", "Charge contacts (collector + dock plate, presence switch, dock relay)", 1, 350, "E (quote)", "https://uk.robotshop.com/products/robopad-extendable-collector-contacts-90mm-wide-100a"),
 ("Compute", "Main PC OnLogic Helix 500 (12-24 V DC), 16 GB, SSD", 1, 1000, "R/E", "https://www.phoronix.com/review/onlogic-helix-500"),
 ("Compute", "Intel AX210 Wi-Fi 6E card + antennas", 1, 35, "R", "https://vetco.net/products/intel-wifi-6e-ax210-wifi-bluetooth-5-2-m-2-card"),
 ("Compute", "STM32 NUCLEO-H723ZG + carrier (RS-485, CAN-FD, I/O)", 1, 110, "R/E", "https://futureelectronics.com/p/development-tools--microcontroller-microprocessor--32-bit-eval-board/nucleo-h723zg-stmicroelectronics-4152938"),
 ("Compute", "Advantech EKI-2725 GbE switch (12/48 V)", 1, 195, "R", "https://uk.rs-online.com/web/p/network-hubs/2835848"),
 ("Sensors", "RPLIDAR S2 2D lidar", 2, 400, "E", "https://bucket-download.slamtec.com/4a5f962943b5140f97ae3b7fab7ed9bdf2adee11/SLAMTEC_rplidar_datasheet_S2_v2.0_en.pdf"),
 ("Wall node", "Jetson Orin Nano Super dev kit", 1, 399, "R (news, after 2026-07 price rise)", "https://hwbusters.com/news/nvidia-jetson-prices-jump-up-to-101-the-249-orin-nano-super-is-now-399/"),
 ("Wall node", "RealSense D435i + mount + 2 m USB-C cable", 1, 600, "R", "https://www.realsenseai.com/products/depth-camera-d435i/"),
 ("Safety", "PILZ PNOZ s3 24 VDC", 1, 170, "R", "https://elektrobode.nl/products/pilz-pnozsigma-pnoz-s3-24vdc-relais-vr-veiligheidsstroomcircuits-750103"),
 ("Safety", "E-stop mushroom buttons (2) + key switch", 1, 120, "E", "-"),
 ("Safety", "Wireless e-stop (generic; FORT PL d is quote-only)", 1, 170, "R", "https://store.clearpathrobotics.com/products/wireless-e-stop"),
 ("Safety", "ASO SENTIR safety edges (4 sides) + ELMON rail 41-822 relay", 1, 800, "R/E", "https://es.misumi-ec.com/vona2/detail/221304237161"),
]
rows, tot, groups = [], 0, {}
for g, item, q, u, tag, src in B:
    ext = q * u
    tot += ext
    groups[g] = groups.get(g, 0) + ext
    rows.append(dict(group=g, item=item, qty=q, unit_usd=u, ext_usd=ext, tag=tag, source=src))
cont = round(0.10 * tot)
out = dict(rows=rows, subtotal_usd=tot, contingency_10pct_usd=cont, total_usd=tot + cont,
           total_sgd=round((tot + cont) * USD_SGD), usd_sgd_rate_assumed=USD_SGD, by_group_usd=groups,
           total_excl_arm_usd=tot + cont - 29218 - 1777)
(H / "bom.json").write_text(json.dumps(out, indent=1))
print("| # | Group | Item | Qty | Unit USD | Ext USD | Tag | Source |\n|---|---|---|---|---|---|---|---|")
for i, r in enumerate(rows, 1):
    s = r["source"]
    s = f"[link]({s})" if s.startswith("http") else s
    print(f"| {i} | {r['group']} | {r['item']} | {r['qty']} | {r['unit_usd']:,} | {r['ext_usd']:,} | {r['tag']} | {s} |")
print(f"| | | **Subtotal** | | | **{tot:,}** | | |\n| | | Contingency 10 % | | | {cont:,} | E | |\n| | | **Total** | | | **{tot+cont:,}** (≈ SGD {round((tot+cont)*USD_SGD):,}) | | |")
print(groups, out["total_excl_arm_usd"])
