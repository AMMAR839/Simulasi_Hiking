"""
Analisis error simulasi Hiking LoRa — skrip STANDALONE (tidak butuh ROS).

Hardware referensi proyek:
  - LoRa  : EByte E220-900T22D (LLCC68), TX max 22 dBm, f=915 MHz
             Ref: EByte E220-900T22D Datasheet v1.0, 2021
  - Antena: SMA female 3 dBi (portable node & relay node)
  - GPS   : u-blox NEO-6M V2 (GY-GPS6MV2), CEP 2.5 m (open-sky)
             Ref: u-blox NEO-6M Product Summary, UBX-09003295-R08

Jalankan setelah simulasi selesai:
  python tools/analysis_error.py --pos sim_logs/sim_XXX/positions.csv
                                  --net sim_logs/sim_XXX/network_events.csv

Dependensi: pip install matplotlib numpy
"""
import argparse
import csv
import math
import os
from collections import defaultdict

# ── Konstanta hardware proyek ─────────────────────────────────────────────────
# Harus konsisten dengan parameter di launch file / ROS nodes

REF_LAT               = -6.89148    # reference_lat (hiker_agent & scenario.py)
REF_LON               = 107.61066   # reference_lon
METERS_PER_WORLD_UNIT = 35.0        # meters_per_world_unit (scenario.py DEFAULT)
FREQ_MHZ              = 915.0       # frekuensi operasi LoRa (E220-900T22D: 850-930 MHz)

# EByte E220-900T22D: TX power maksimum 22 dBm (Ref: EByte datasheet §3.1)
TX_POWER_DBM          = 22.0

# Antena SMA female 3 dBi (spesifikasi komponen user)
# Kedua ujung link (TX dan RX) menggunakan antena yang sama → gain total = 2 × 3 = 6 dBi
# Ref: Friis transmission equation, IEEE Std 149-2021
ANTENNA_GAIN_DB       = 3.0

# Path loss exponent (n) untuk terrain pegunungan — log-distance empirical model
# Ref: Petäjäjärvi et al., "Evaluation of LoRa LPWAN Technology", ISWCS 2015 (n=2.7–3.5 rural)
#      Amatya et al., "LoRa Performance in Mountain Environment", IEEE Access 2019 (n≈3.0–3.5)
#      Georgiou & Raza, "Low Power Wide Area Network Analysis", IEEE Commun. Lett. 2017
_N_MOUNTAIN           = 3.5

# ─────────────────────────────────────────────────────────────────────────────


# ── Fungsi konversi & model propagasi ────────────────────────────────────────

def local_to_gps(x_wu: float, y_wu: float) -> tuple:
    """
    Konversi posisi world-unit → GPS lat/lon (ground truth).

    Formula identik dengan scenario.py::local_to_gps() yang dipakai ROS:
        north_m = y  * meters_per_world_unit      ← KALI (bukan bagi)
        lat     = ref_lat + north_m / 111_320.0   ← konstanta ITU-R P.1561

    Bug umum: menggunakan y / METERS_PER_WORLD_UNIT (membagi) → error ~1200×.
    """
    north_m = y_wu * METERS_PER_WORLD_UNIT
    east_m  = x_wu * METERS_PER_WORLD_UNIT
    lat = REF_LAT + north_m / 111_320.0
    lon = REF_LON + east_m  / (111_320.0 * math.cos(math.radians(REF_LAT)))
    return lat, lon


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Jarak (meter) antara dua titik GPS — formula Haversine."""
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 2.0 * R * math.asin(math.sqrt(max(0.0, a)))


def fspl_rx_dbm(distance_m: float) -> float:
    """
    RSSI teoritis free-space (FSPL) — batas bawah teoretis propagasi.

    EIRP = P_tx + G_tx + G_rx  (Friis, IEEE Std 149)
    Kedua antena diperhitungkan → +2 × ANTENNA_GAIN_DB.
    """
    if distance_m <= 0.1:
        return TX_POWER_DBM + 2.0 * ANTENNA_GAIN_DB
    d_km = distance_m / 1000.0
    fspl = 32.44 + 20.0 * math.log10(d_km) + 20.0 * math.log10(FREQ_MHZ)
    return TX_POWER_DBM + 2.0 * ANTENNA_GAIN_DB - fspl


def log_distance_rx_dbm(distance_m: float, n: float = _N_MOUNTAIN) -> float:
    """
    Model log-distance empiris untuk terrain pegunungan (ITU-R P.1546 inspired).

    PL(d) = PL(d0) + 10·n·log10(d/d0)
    d0 = 1000 m (referensi 1 km, standard ITU-R P.1546)
    n  = 3.5 (path loss exponent pegunungan, literatur di atas)

    Model ini TIDAK termasuk shadowing (σ ~ 8–12 dB untuk gunung),
    sehingga mewakili median propagasi — bukan worst-case.
    """
    if distance_m <= 0.1:
        return fspl_rx_dbm(0.1)
    d0_m   = 1000.0
    pl_d0  = 32.44 + 20.0 * math.log10(d0_m / 1000.0) + 20.0 * math.log10(FREQ_MHZ)
    pl     = pl_d0 + 10.0 * n * math.log10(max(1.0, distance_m) / d0_m)
    return TX_POWER_DBM + 2.0 * ANTENNA_GAIN_DB - pl


def rmse(values: list) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v ** 2 for v in values) / len(values))


def mae(values: list) -> float:
    if not values:
        return 0.0
    return sum(abs(v) for v in values) / len(values)


def percentile(sorted_vals: list, p: float) -> float:
    if not sorted_vals:
        return 0.0
    idx = max(0, min(len(sorted_vals) - 1, int(len(sorted_vals) * p)))
    return sorted_vals[idx]


def wilson_ci(n_success: int, n_total: int, z: float = 1.96) -> tuple:
    """
    Wilson score interval untuk proporsi — lebih akurat dari normal approx.
    Ref: Brown et al., "Interval Estimation for a Binomial Proportion",
         Statistical Science 2001.
    Return: (ci_low_pct, ci_high_pct)
    """
    if n_total == 0:
        return (0.0, 100.0)
    p      = n_success / n_total
    denom  = 1.0 + z * z / n_total
    center = (p + z * z / (2 * n_total)) / denom
    half   = (z / denom) * math.sqrt(
        p * (1 - p) / n_total + z * z / (4 * n_total * n_total)
    )
    return (max(0.0, (center - half) * 100),
            min(100.0, (center + half) * 100))


# ── Load CSV ───────────────────────────────────────────────────────────────────

def load_positions(path: str) -> list:
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "hiker_id":    r["hiker_id"],
                    "weather":     r.get("weather", ""),
                    "true_x_wu":   float(r["true_x_wu"]),
                    "true_y_wu":   float(r["true_y_wu"]),
                    "gps_lat":     float(r["gps_lat"]),
                    "gps_lon":     float(r["gps_lon"]),
                    "gps_error_m": float(r["gps_error_m"]),
                    "dop":         float(r["dop"]),
                })
            except (ValueError, KeyError):
                continue
    return rows


def load_network(path: str) -> list:
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                d  = float(r.get("distance_m") or 0)
                rx = float(r.get("rx_dbm") or 0)
                if d <= 0 or rx == 0:
                    continue
                rows.append({
                    "hiker_id":    r["hiker_id"],
                    "delivered":   r["delivered"].strip().lower() == "true",
                    "drop_reason": r.get("drop_reason", "").strip(),
                    "true_x_wu":   float(r.get("true_x_wu") or 0),
                    "true_y_wu":   float(r.get("true_y_wu") or 0),
                    "gps_lat":     float(r.get("gps_lat") or 0),
                    "gps_lon":     float(r.get("gps_lon") or 0),
                    "distance_m":  d,
                    "rx_dbm":      rx,
                    "snr_db":      float(r.get("snr_db") or 0),
                    "margin_db":   float(r.get("margin_db") or 0),
                    "sf":          int(float(r.get("spreading_factor") or 9)),
                    "weather":     r.get("weather", ""),
                    "obstacle_loss_db":   float(r.get("obstacle_loss_db") or 0),
                    "terrain_shadow_db":  float(r.get("terrain_shadow_db") or 0),
                    "diffraction_db":     float(r.get("diffraction_db") or 0),
                    "terrain_scatter_db": float(r.get("terrain_scatter_db") or 0),
                    "fading_db":          float(r.get("fading_db") or 0),
                })
            except (ValueError, KeyError):
                continue
    return rows


# ── Hitung metrik ──────────────────────────────────────────────────────────────

def compute_metrics(pos_rows: list, net_rows: list) -> dict:
    results = {}

    # 1. GPS error per-tick ────────────────────────────────────────────────────
    tick_errors = [r["gps_error_m"] for r in pos_rows if r["gps_error_m"] > 0]
    if tick_errors:
        tick_sorted = sorted(tick_errors)
        results["gps_tick"] = {
            "n":      len(tick_errors),
            "rmse":   rmse(tick_errors),
            "mae":    mae(tick_errors),
            "p50":    percentile(tick_sorted, 0.50),
            "p95":    percentile(tick_sorted, 0.95),
            "max":    max(tick_errors),
            "values": tick_sorted,
        }

    # 2. GPS error per-paket (haversine true world-unit → GPS payload) ─────────
    pkt_errors = []
    for r in net_rows:
        if r["true_x_wu"] != 0.0 and r["gps_lat"] != 0.0:
            true_lat, true_lon = local_to_gps(r["true_x_wu"], r["true_y_wu"])
            pkt_errors.append(haversine_m(true_lat, true_lon, r["gps_lat"], r["gps_lon"]))
    if pkt_errors:
        pkt_sorted = sorted(pkt_errors)
        results["gps_packet"] = {
            "n":      len(pkt_errors),
            "rmse":   rmse(pkt_errors),
            "p95":    percentile(pkt_sorted, 0.95),
            "values": pkt_sorted,
        }

    # 3. RSSI vs FSPL dan vs model empiris pegunungan ─────────────────────────
    fspl_deltas = []
    emp_deltas  = []
    for r in net_rows:
        fspl_ref = fspl_rx_dbm(r["distance_m"])
        emp_ref  = log_distance_rx_dbm(r["distance_m"])
        fspl_deltas.append(r["rx_dbm"] - fspl_ref)
        emp_deltas.append(r["rx_dbm"]  - emp_ref)

    if fspl_deltas:
        results["rssi"] = {
            "n":         len(fspl_deltas),
            "mae_fspl":  mae(fspl_deltas),
            "rmse_fspl": rmse(fspl_deltas),
            "bias_fspl": sum(fspl_deltas) / len(fspl_deltas),
            "mae_emp":   mae(emp_deltas),
            "rmse_emp":  rmse(emp_deltas),
            "bias_emp":  sum(emp_deltas)  / len(emp_deltas),
            "fspl_values": fspl_deltas,
            "emp_values":  emp_deltas,
            "distances": [r["distance_m"] for r in net_rows],
            "rx_dbm":    [r["rx_dbm"]     for r in net_rows],
            "delivered": [r["delivered"]  for r in net_rows],
        }

    # 4. PDR per Spreading Factor ──────────────────────────────────────────────
    sf_stats: dict = defaultdict(lambda: {"delivered": 0, "total": 0})
    for r in net_rows:
        sf_stats[r["sf"]]["total"] += 1
        if r["delivered"]:
            sf_stats[r["sf"]]["delivered"] += 1
    results["sf"] = dict(sf_stats)

    # 5. Drop reason ───────────────────────────────────────────────────────────
    drops: dict = defaultdict(int)
    for r in net_rows:
        if not r["delivered"] and r["drop_reason"]:
            drops[r["drop_reason"]] += 1
    results["drops"] = dict(drops)

    # 6. Kontribusi atenuasi (rata-rata) ──────────────────────────────────────
    if net_rows:
        obs  = [r["obstacle_loss_db"]   for r in net_rows]
        terr = [r["terrain_shadow_db"]  for r in net_rows]
        diff = [r["diffraction_db"]     for r in net_rows]
        scat = [r["terrain_scatter_db"] for r in net_rows]
        fade = [r["fading_db"]          for r in net_rows]
        results["attenuation"] = {
            "obstacle_mean":    sum(obs)  / len(obs),
            "terrain_mean":     sum(terr) / len(terr),
            "diffraction_mean": sum(diff) / len(diff),
            "scatter_mean":     sum(scat) / len(scat),
            "fading_mean":      sum(fade) / len(fade),
            "total_logged":     (sum(obs) + sum(terr) + sum(diff) + sum(scat) + sum(fade)) / len(obs),
        }

    return results


# ── Cetak ringkasan ────────────────────────────────────────────────────────────

def print_summary(m: dict) -> None:
    SEP  = "=" * 70
    SEP2 = "-" * 70
    print(f"\n{SEP}")
    print("  LAPORAN EVALUASI ERROR — SIMULASI HIKING LoRa")
    print(f"  Hardware: EByte E220-900T22D (22 dBm) + 3 dBi SMA | GPS: NEO-6M V2")
    print(SEP)

    # ── [1] GPS tick error ────────────────────────────────────────────────────
    if "gps_tick" in m:
        g = m["gps_tick"]
        print(f"\n[1] GPS Position Error — per-tick  ({g['n']} sampel)")
        print(f"    RMSE     : {g['rmse']:.2f} m")
        print(f"    MAE      : {g['mae']:.2f} m")
        print(f"    Median   : {g['p50']:.2f} m   (≈ CEP jika distribusi Rayleigh)")
        print(f"    95th pct : {g['p95']:.2f} m")
        print(f"    Max      : {g['max']:.2f} m")
        print(f"    {SEP2}")
        print(f"    Referensi u-blox NEO-6M V2 (UBX-09003295-R08):")
        print(f"      Open-sky (DOP≈1.0) : CEP=2.5m, RMSE≈3.0m, 95th pct≈5.2m")
        print(f"      Pegunungan (DOP×2) : RMSE≈5–7m, 95th pct≈8–12m  ← lebih realistis")
        print(f"    Noise model (σ=2.0m × DOP) sesuai dengan CEP=2.5m spec NEO-6M")
        status = "OK" if g['p95'] <= 12.0 else "PERLU DIKALIBRASI"
        print(f"    Status: {status}")

    # ── [2] GPS packet (haversine) ────────────────────────────────────────────
    if "gps_packet" in m:
        gp = m["gps_packet"]
        print(f"\n[2] GPS Position Error — per-paket LoRa  ({gp['n']} paket)")
        print(f"    RMSE haversine  : {gp['rmse']:.2f} m")
        print(f"    95th pct        : {gp['p95']:.2f} m")
        if gp["rmse"] < 30.0:
            print(f"    Status: OK — koordinat GPS dalam paket LoRa valid untuk SAR")
        elif gp["rmse"] < 200.0:
            print(f"    Status: CUKUP — error masih dalam 1 radius pencarian SAR")
        else:
            print(f"    !! KRITIS: RMSE > 200 m — cek formula local_to_gps() dan "
                  f"METERS_PER_WORLD_UNIT={METERS_PER_WORLD_UNIT}")

    # ── [3] RSSI vs model ────────────────────────────────────────────────────
    if "rssi" in m:
        r = m["rssi"]
        print(f"\n[3] RSSI vs Model Propagasi  ({r['n']} link)")
        print(f"    ─── vs FSPL (free-space, batas bawah teori) ───")
        print(f"    Bias : {r['bias_fspl']:+.2f} dB  |  MAE {r['mae_fspl']:.2f}  |  RMSE {r['rmse_fspl']:.2f} dB")
        print(f"    Catatan: FSPL adalah minimum teoritis. Medan pegunungan NLOS")
        print(f"             wajar 20–45 dB di bawah FSPL (ITU-R P.452).")
        print(f"    ─── vs Log-distance n={_N_MOUNTAIN} (pegunungan empiris, ITU-R P.1546) ───")
        print(f"    Bias : {r['bias_emp']:+.2f} dB  |  MAE {r['mae_emp']:.2f}  |  RMSE {r['rmse_emp']:.2f} dB")
        abs_bias_emp = abs(r["bias_emp"])
        if abs_bias_emp < 5:
            status_emp = "Sangat baik (|bias| < 5 dB)"
        elif abs_bias_emp < 12:
            status_emp = "Baik (|bias| 5–12 dB, wajar untuk model deterministik)"
        elif abs_bias_emp < 20:
            status_emp = "Acceptable (|bias| 12–20 dB)"
        else:
            status_emp = "Perlu review — kemungkinan stacking losses berlebihan"
        print(f"    Status empiris: {status_emp}")

    # ── [4] PDR ───────────────────────────────────────────────────────────────
    if "sf" in m:
        print(f"\n[4] Packet Delivery Rate per Spreading Factor")
        print(f"    (Wilson 95% CI — n minimum 1000 untuk publikasi IEEE)")
        for sf in sorted(m["sf"]):
            s = m["sf"][sf]
            n_ok, n_tot = s["delivered"], s["total"]
            pdr       = n_ok / n_tot * 100 if n_tot > 0 else 0
            ci_lo, ci_hi = wilson_ci(n_ok, n_tot)
            bar    = "█" * int(pdr / 5)
            status = "OK" if pdr >= 80 else ("WARN" if pdr >= 50 else "FAIL")
            print(f"    SF{sf:2d}  {pdr:5.1f}%  {bar:<20s}  ({n_ok}/{n_tot})  [{status}]")
            print(f"          95% CI: [{ci_lo:.1f}%, {ci_hi:.1f}%]"
                  f"  ← {'cukup' if n_tot >= 200 else 'terlalu sedikit sampel'}")

    # ── [5] Drop reason ───────────────────────────────────────────────────────
    if "drops" in m and m["drops"]:
        total = sum(m["drops"].values())
        print(f"\n[5] Penyebab Packet Loss  ({total} drop)")
        for reason, cnt in sorted(m["drops"].items(), key=lambda x: -x[1]):
            print(f"    {reason:20s}: {cnt:4d}  ({cnt/total*100:.1f}%)")

    # ── [6] Atenuasi ──────────────────────────────────────────────────────────
    if "attenuation" in m:
        a = m["attenuation"]
        print(f"\n[6] Kontribusi Atenuasi per Komponen (rata-rata per link)")
        print(f"    Obstacle (vegetasi)   : {a['obstacle_mean']:.1f} dB  "
              f"(dikurangi korelasi terrain, ITU-R P.833)")
        print(f"    Terrain shadow        : {a['terrain_mean']:.1f} dB")
        print(f"    Diffraction excess    : {a['diffraction_mean']:.1f} dB  "
              f"(=max(0, raw_diff - shadow))")
        print(f"    Terrain scatter (LOS) : {a['scatter_mean']:.1f} dB  "
              f"(ITU-R P.452, 1.0 dB/km, hanya LOS)")
        print(f"    Fading                : {a['fading_mean']:.1f} dB  (Rician/Rayleigh)")
        print(f"    ─────────────────────────────────────────────────")
        print(f"    Total excess          : {a['total_logged']:.1f} dB  "
              f"(Ref: NLOS gunung ITU-R P.452 ≈ 20–45 dB)")

    print(f"\n{SEP}\n")


# ── Plot ───────────────────────────────────────────────────────────────────────

def plot_all(m: dict, save_path: str) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib tidak tersedia. Install: pip install matplotlib")
        return

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(
        "Evaluasi Error Simulasi Hiking LoRa\n"
        "Hardware: EByte E220-900T22D (22 dBm, LLCC68)  |  GPS: u-blox NEO-6M V2  "
        f"|  f={FREQ_MHZ:.0f} MHz  |  Antena: {ANTENNA_GAIN_DB:.0f} dBi",
        fontsize=12, fontweight="bold"
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

    # ─ Plot 1: CDF GPS error ──────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    if "gps_tick" in m:
        vals = m["gps_tick"]["values"]
        cdf  = [i / len(vals) for i in range(len(vals))]
        ax1.plot(vals, cdf, "b-", linewidth=2, label="GPS error per-tick (σ=2m×DOP)")
        p95  = m["gps_tick"]["p95"]
        ax1.axvline(p95, color="red", linestyle="--",
                    label=f"95th pct simulasi = {p95:.1f} m")
        ax1.axhline(0.95, color="red", linestyle=":", alpha=0.35)
        ax1.axvline(5.2, color="green", linestyle=":",
                    label="NEO-6M open-sky 95th ≈ 5.2 m", alpha=0.85)
        ax1.axvspan(8.0, 12.0, alpha=0.12, color="orange",
                    label="NEO-6M gunung (DOP×2) 95th: 8–12 m")
        if "gps_packet" in m:
            vp  = m["gps_packet"]["values"]
            cdp = [i / len(vp) for i in range(len(vp))]
            ax1.plot(vp, cdp, "g--", linewidth=1.5, alpha=0.8,
                     label=f"Haversine per-paket LoRa  (n={len(vp)})")
    ax1.set_xlabel("Position Error (m)")
    ax1.set_ylabel("CDF")
    ax1.set_title("CDF GPS Position Error")
    ax1.legend(fontsize=7.5, loc="lower right")
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 1.05)

    # ─ Plot 2: RSSI vs jarak + FSPL + empirical ──────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    if "rssi" in m:
        dist = m["rssi"]["distances"]
        rx   = m["rssi"]["rx_dbm"]
        delv = m["rssi"]["delivered"]
        ax2.scatter(
            [d for d, ok in zip(dist, delv) if ok],
            [r for r, ok in zip(rx,   delv) if ok],
            s=10, alpha=0.45, c="#2ecc71", label="Terkirim",
        )
        ax2.scatter(
            [d for d, ok in zip(dist, delv) if not ok],
            [r for r, ok in zip(rx,   delv) if not ok],
            s=18, alpha=0.80, c="#e74c3c", marker="x", linewidths=1.5, label="Drop",
        )
        if dist:
            d_lo  = max(10,  int(min(dist)))
            d_hi  = int(max(dist)) + 500
            step  = max(1, (d_hi - d_lo) // 300)
            d_rng = list(range(d_lo, d_hi, step))
            ax2.plot(d_rng, [fspl_rx_dbm(d)         for d in d_rng],
                     "k--", linewidth=1.8, label="FSPL (free-space, teoritis)")
            ax2.plot(d_rng, [log_distance_rx_dbm(d) for d in d_rng],
                     "b-",  linewidth=1.8,
                     label=f"Log-dist n={_N_MOUNTAIN} (gunung empiris, ITU-R P.1546)")
    ax2.set_xlabel("Jarak (m)")
    ax2.set_ylabel("RSSI (dBm)")
    ax2.set_title("RSSI vs Jarak: Simulasi vs Model Propagasi")
    ax2.legend(fontsize=7.5)
    ax2.grid(True, alpha=0.3)

    # ─ Plot 3: PDR per SF + Wilson CI ────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    if "sf" in m:
        sf_list = sorted(m["sf"].keys())
        pdrs    = []
        ci_los  = []
        ci_his  = []
        for sf in sf_list:
            s       = m["sf"][sf]
            n_ok    = s["delivered"]
            n_tot   = s["total"]
            pdr     = n_ok / n_tot * 100 if n_tot > 0 else 0
            lo, hi  = wilson_ci(n_ok, n_tot)
            pdrs.append(pdr)
            ci_los.append(pdr - lo)
            ci_his.append(hi  - pdr)
        colors = ["#2ecc71" if p >= 80 else "#f39c12" if p >= 50 else "#e74c3c"
                  for p in pdrs]
        x      = list(range(len(sf_list)))
        bars   = ax3.bar(x, pdrs, color=colors, edgecolor="white", width=0.55)
        ax3.errorbar(x, pdrs, yerr=[ci_los, ci_his], fmt="none",
                     color="black", capsize=6, linewidth=1.8, label="95% CI (Wilson)")
        ax3.axhline(80, color="orange", linestyle="--", alpha=0.65, label="Target PDR 80%")
        ax3.set_xticks(x)
        ax3.set_xticklabels([f"SF{sf}" for sf in sf_list])
        ax3.set_ylabel("Packet Delivery Rate (%)")
        ax3.set_title("PDR per Spreading Factor (dengan Confidence Interval)")
        ax3.set_ylim(0, 118)
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3, axis="y")
        for bar, pdr, sf in zip(bars, pdrs, sf_list):
            n_tot = m["sf"][sf]["total"]
            ax3.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 4.0,
                f"{pdr:.1f}%\n(n={n_tot})",
                ha="center", va="bottom", fontsize=8,
            )

    # ─ Plot 4: Distribusi error vs FSPL dan vs empirical ─────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    if "rssi" in m:
        fd = m["rssi"]["fspl_values"]
        ed = m["rssi"]["emp_values"]
        bf = m["rssi"]["bias_fspl"]
        be = m["rssi"]["bias_emp"]
        bins = 40
        ax4.hist(fd, bins=bins, color="steelblue", alpha=0.60,
                 label="vs FSPL", edgecolor="white")
        ax4.hist(ed, bins=bins, color="darkorange", alpha=0.60,
                 label=f"vs Log-dist n={_N_MOUNTAIN} (gunung)", edgecolor="white")
        ax4.axvline(0,  color="black",      linewidth=1.8, label="Referensi Δ=0")
        ax4.axvline(bf, color="steelblue",  linestyle="--", linewidth=1.5,
                    label=f"Bias FSPL = {bf:+.1f} dB")
        ax4.axvline(be, color="darkorange", linestyle="--", linewidth=1.5,
                    label=f"Bias empiris = {be:+.1f} dB")
        # Rentang wajar NLOS gunung (shade)
        ax4.axvspan(-35, -15, alpha=0.08, color="green",
                    label="NLOS gunung wajar: −15 s/d −35 dB (ITU-R P.452)")
        ax4.set_xlabel("RSSI − Model (dB)  [negatif = lebih lemah dari referensi]")
        ax4.set_ylabel("Jumlah link")
        ax4.set_title("Distribusi Error Model Propagasi")
        ax4.legend(fontsize=7.5)
        ax4.grid(True, alpha=0.3)

    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot tersimpan → {save_path}")
    plt.show()


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analisis error simulasi Hiking LoRa (standalone, tidak butuh ROS)"
    )
    parser.add_argument("--pos", required=True,
                        help="Path ke positions.csv dari data_logger_node")
    parser.add_argument("--net", required=True,
                        help="Path ke network_events.csv dari data_logger_node")
    parser.add_argument("--out", default="analysis_result.png",
                        help="Output PNG (default: analysis_result.png)")
    args = parser.parse_args()

    for fp in (args.pos, args.net):
        if not os.path.exists(fp):
            print(f"ERROR: File tidak ditemukan: {fp}")
            return

    print(f"Memuat data dari:\n  {args.pos}\n  {args.net}")
    pos_rows = load_positions(args.pos)
    net_rows = load_network(args.net)
    print(f"  → {len(pos_rows)} baris posisi, {len(net_rows)} event jaringan")

    if not pos_rows and not net_rows:
        print("Tidak ada data yang bisa dianalisis.")
        return

    metrics = compute_metrics(pos_rows, net_rows)
    print_summary(metrics)
    plot_all(metrics, save_path=args.out)


if __name__ == "__main__":
    main()
