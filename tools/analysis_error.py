"""
Analisis error simulasi Hiking LoRa — skrip STANDALONE (tidak butuh ROS).

Jalankan setelah simulasi selesai:
  python tools/analysis_error.py --pos ~/sim_logs/sim_XXX/positions.csv
                                  --net ~/sim_logs/sim_XXX/network_events.csv

Output:
  - Ringkasan statistik di terminal
  - File PNG: analysis_result.png (4 plot)

Dependensi: pip install matplotlib numpy
"""
import argparse
import csv
import math
import os
from collections import defaultdict

# ── Konstanta simulasi (harus sesuai parameter di launch file) ────────────
REF_LAT              = -6.89148    # reference_lat di hiker_agent
REF_LON              = 107.61066   # reference_lon di hiker_agent
METERS_PER_WORLD_UNIT = 35.0       # meters_per_world_unit
FREQ_MHZ             = 915.0       # frekuensi LoRa
TX_POWER_DBM         = 17.0        # tx power default
ANTENNA_GAIN_DB      = 2.0         # antenna gain default
# ─────────────────────────────────────────────────────────────────────────


# ── Fungsi konversi & geometri ─────────────────────────────────────────────
def local_to_gps(x_wu: float, y_wu: float) -> tuple:
    """Konversi posisi world-unit ke GPS lat/lon (ground truth)."""
    lat = REF_LAT + (y_wu / METERS_PER_WORLD_UNIT) / 111_111.0
    lon = REF_LON + (x_wu / METERS_PER_WORLD_UNIT) / (
        111_111.0 * math.cos(math.radians(REF_LAT))
    )
    return lat, lon


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Jarak (meter) antara dua titik GPS menggunakan formula Haversine."""
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + (
        math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2) ** 2
    )
    return 2.0 * R * math.asin(math.sqrt(max(0.0, a)))


def fspl_rx_dbm(distance_m: float) -> float:
    """RSSI teoritis free-space (FSPL) pada jarak tertentu."""
    if distance_m <= 0.1:
        return TX_POWER_DBM + ANTENNA_GAIN_DB
    d_km = distance_m / 1000.0
    fspl = 32.44 + 20.0 * math.log10(d_km) + 20.0 * math.log10(FREQ_MHZ)
    return TX_POWER_DBM + ANTENNA_GAIN_DB - fspl


def rmse(values: list) -> float:
    return math.sqrt(sum(v ** 2 for v in values) / len(values))


def mae(values: list) -> float:
    return sum(abs(v) for v in values) / len(values)


def percentile(sorted_vals: list, p: float) -> float:
    idx = max(0, min(len(sorted_vals) - 1, int(len(sorted_vals) * p)))
    return sorted_vals[idx]


# ── Load CSV ───────────────────────────────────────────────────────────────
def load_positions(path: str) -> list:
    """
    Baca positions.csv.
    Kembalikan list dict dengan field numerik sudah di-cast.
    """
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
    """
    Baca network_events.csv.
    Hanya baris dengan distance_m > 0 dan rx_dbm tersedia.
    """
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                d = float(r.get("distance_m") or 0)
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
                    "obstacle_loss_db":  float(r.get("obstacle_loss_db") or 0),
                    "terrain_shadow_db": float(r.get("terrain_shadow_db") or 0),
                    "diffraction_db":    float(r.get("diffraction_db") or 0),
                    "fading_db":         float(r.get("fading_db") or 0),
                })
            except (ValueError, KeyError):
                continue
    return rows


# ── Hitung metrik ──────────────────────────────────────────────────────────
def compute_metrics(pos_rows: list, net_rows: list) -> dict:
    results = {}

    # ── 1. GPS Position Error (dari positions.csv, per-tick) ──────────────
    tick_errors = [r["gps_error_m"] for r in pos_rows if r["gps_error_m"] > 0]
    if tick_errors:
        tick_sorted = sorted(tick_errors)
        results["gps_tick"] = {
            "n":    len(tick_errors),
            "rmse": rmse(tick_errors),
            "mae":  mae(tick_errors),
            "p50":  percentile(tick_sorted, 0.50),
            "p95":  percentile(tick_sorted, 0.95),
            "max":  max(tick_errors),
            "values": tick_sorted,
        }

    # ── 2. GPS Position Error (dari network_events, haversine true vs GPS) ─
    pkt_errors = []
    for r in net_rows:
        if r["true_x_wu"] and r["gps_lat"]:
            true_lat, true_lon = local_to_gps(r["true_x_wu"], r["true_y_wu"])
            err = haversine_m(true_lat, true_lon, r["gps_lat"], r["gps_lon"])
            pkt_errors.append(err)
    if pkt_errors:
        pkt_sorted = sorted(pkt_errors)
        results["gps_packet"] = {
            "n":    len(pkt_errors),
            "rmse": rmse(pkt_errors),
            "p95":  percentile(pkt_sorted, 0.95),
            "values": pkt_sorted,
        }

    # ── 3. RSSI vs FSPL (error model propagasi) ────────────────────────────
    rssi_deltas = []   # positif = lebih baik dari FSPL, negatif = lebih buruk
    for r in net_rows:
        expected = fspl_rx_dbm(r["distance_m"])
        rssi_deltas.append(r["rx_dbm"] - expected)
    if rssi_deltas:
        results["rssi"] = {
            "n":    len(rssi_deltas),
            "mae":  mae(rssi_deltas),
            "rmse": rmse(rssi_deltas),
            "bias": sum(rssi_deltas) / len(rssi_deltas),
            "values": rssi_deltas,
            "distances": [r["distance_m"] for r in net_rows],
            "rx_dbm":    [r["rx_dbm"]     for r in net_rows],
            "delivered": [r["delivered"]  for r in net_rows],
        }

    # ── 4. Packet Delivery Rate per Spreading Factor ───────────────────────
    sf_stats: dict = defaultdict(lambda: {"delivered": 0, "total": 0})
    for r in net_rows:
        sf_stats[r["sf"]]["total"] += 1
        if r["delivered"]:
            sf_stats[r["sf"]]["delivered"] += 1
    results["sf"] = dict(sf_stats)

    # ── 5. Drop reason breakdown ───────────────────────────────────────────
    drops: dict = defaultdict(int)
    for r in net_rows:
        if not r["delivered"] and r["drop_reason"]:
            drops[r["drop_reason"]] += 1
    results["drops"] = dict(drops)

    # ── 6. Kontribusi atenuasi per-komponen (rata-rata) ────────────────────
    if net_rows:
        results["attenuation"] = {
            "obstacle_mean":  sum(r["obstacle_loss_db"] for r in net_rows) / len(net_rows),
            "terrain_mean":   sum(r["terrain_shadow_db"] for r in net_rows) / len(net_rows),
            "diffraction_mean": sum(r["diffraction_db"] for r in net_rows) / len(net_rows),
            "fading_mean":    sum(r["fading_db"] for r in net_rows) / len(net_rows),
        }

    return results


# ── Cetak ringkasan ────────────────────────────────────────────────────────
def print_summary(m: dict) -> None:
    SEP = "=" * 64
    print(f"\n{SEP}")
    print("  LAPORAN EVALUASI ERROR SIMULASI HIKING LoRa")
    print(SEP)

    if "gps_tick" in m:
        g = m["gps_tick"]
        print(f"\n[1] GPS Position Error  (per-tick, {g['n']} sampel)")
        print(f"    RMSE       : {g['rmse']:.2f} m")
        print(f"    MAE        : {g['mae']:.2f} m")
        print(f"    50th pct   : {g['p50']:.2f} m")
        print(f"    95th pct   : {g['p95']:.2f} m  ← standar akurasi GPS")
        print(f"    Max        : {g['max']:.2f} m")
        print(f"    Ref. U-blox M8 outdoor: ~2.5 m CEP, ~5 m 95th pct")

    if "gps_packet" in m:
        gp = m["gps_packet"]
        print(f"\n[2] GPS Position Error  (haversine true→GPS, {gp['n']} paket)")
        print(f"    RMSE : {gp['rmse']:.2f} m  |  95th pct : {gp['p95']:.2f} m")

    if "rssi" in m:
        r = m["rssi"]
        print(f"\n[3] RSSI vs Free-Space Path Loss  ({r['n']} link)")
        print(f"    MAE        : {r['mae']:.2f} dB")
        print(f"    RMSE       : {r['rmse']:.2f} dB")
        print(f"    Bias       : {r['bias']:+.2f} dB  "
              f"({'lebih lemah' if r['bias'] < 0 else 'lebih kuat'} dari FSPL)")
        print(f"    Catatan    : MAE 5–15 dB wajar untuk medan pegunungan")
        print(f"                 (obstacle + terrain + fading ≠ free-space)")

    if "sf" in m:
        print(f"\n[4] Packet Delivery Rate per Spreading Factor")
        for sf in sorted(m["sf"]):
            s = m["sf"][sf]
            pdr = s["delivered"] / s["total"] * 100 if s["total"] > 0 else 0
            bar = "█" * int(pdr / 5)
            status = "OK" if pdr >= 80 else ("WARN" if pdr >= 50 else "FAIL")
            print(f"    SF{sf:2d}  {pdr:5.1f}%  {bar:<20s}  "
                  f"({s['delivered']}/{s['total']})  [{status}]")

    if "drops" in m and m["drops"]:
        total = sum(m["drops"].values())
        print(f"\n[5] Penyebab Packet Loss  ({total} drop)")
        for reason, cnt in sorted(m["drops"].items(), key=lambda x: -x[1]):
            print(f"    {reason:15s}: {cnt:4d}  ({cnt/total*100:.1f}%)")

    if "attenuation" in m:
        a = m["attenuation"]
        print(f"\n[6] Rata-rata Kontribusi Atenuasi per Komponen")
        print(f"    Obstacle   : {a['obstacle_mean']:.1f} dB")
        print(f"    Terrain    : {a['terrain_mean']:.1f} dB")
        print(f"    Diffraction: {a['diffraction_mean']:.1f} dB")
        print(f"    Fading     : {a['fading_mean']:.1f} dB")

    print(f"\n{SEP}\n")


# ── Plot ───────────────────────────────────────────────────────────────────
def plot_all(m: dict, save_path: str) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib tidak tersedia. Install: pip install matplotlib")
        return

    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Evaluasi Error Simulasi Hiking LoRa", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.38, wspace=0.32)

    # ─ Plot 1: CDF GPS Position Error ─────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    if "gps_tick" in m:
        vals = m["gps_tick"]["values"]
        cdf = [i / len(vals) for i in range(len(vals))]
        ax1.plot(vals, cdf, "b-", linewidth=2, label="GPS error (tick)")
        p95 = m["gps_tick"]["p95"]
        ax1.axvline(p95, color="r", linestyle="--", label=f"95th pct = {p95:.1f} m")
        ax1.axhline(0.95, color="r", linestyle=":", alpha=0.4)
        if "gps_packet" in m:
            vp = m["gps_packet"]["values"]
            cdp = [i / len(vp) for i in range(len(vp))]
            ax1.plot(vp, cdp, "g--", linewidth=1.5, alpha=0.7, label="Haversine (paket)")
    ax1.set_xlabel("Position Error (m)")
    ax1.set_ylabel("CDF")
    ax1.set_title("CDF GPS Position Error")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 1.05)

    # ─ Plot 2: RSSI vs Distance + kurva FSPL ──────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    if "rssi" in m:
        dist = m["rssi"]["distances"]
        rx   = m["rssi"]["rx_dbm"]
        delv = m["rssi"]["delivered"]
        ax2.scatter(
            [d for d, ok in zip(dist, delv) if ok],
            [r for r, ok in zip(rx, delv) if ok],
            s=6, alpha=0.35, c="#2ecc71", label="Terkirim",
        )
        ax2.scatter(
            [d for d, ok in zip(dist, delv) if not ok],
            [r for r, ok in zip(rx, delv) if not ok],
            s=6, alpha=0.35, c="#e74c3c", label="Drop",
        )
        if dist:
            d_min, d_max = max(1, int(min(dist))), int(max(dist)) + 10
            d_range = list(range(d_min, d_max, max(1, (d_max - d_min) // 200)))
            fspl_line = [fspl_rx_dbm(d) for d in d_range]
            ax2.plot(d_range, fspl_line, "k--", linewidth=2, label="FSPL (free-space)")
    ax2.set_xlabel("Jarak (m)")
    ax2.set_ylabel("RSSI (dBm)")
    ax2.set_title("RSSI vs Jarak: Simulasi vs Free-Space")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ─ Plot 3: PDR per SF ─────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    if "sf" in m:
        sf_list = sorted(m["sf"].keys())
        pdrs = [
            m["sf"][sf]["delivered"] / m["sf"][sf]["total"] * 100
            if m["sf"][sf]["total"] > 0 else 0
            for sf in sf_list
        ]
        colors = [
            "#2ecc71" if p >= 80 else "#f39c12" if p >= 50 else "#e74c3c"
            for p in pdrs
        ]
        bars = ax3.bar(
            [f"SF{sf}" for sf in sf_list], pdrs,
            color=colors, edgecolor="white", width=0.55,
        )
        ax3.axhline(80, color="orange", linestyle="--", alpha=0.6, label="Target 80%")
        ax3.set_ylabel("Packet Delivery Rate (%)")
        ax3.set_title("PDR per Spreading Factor")
        ax3.set_ylim(0, 112)
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3, axis="y")
        for bar, pdr in zip(bars, pdrs):
            ax3.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 1.5,
                f"{pdr:.1f}%",
                ha="center", va="bottom", fontsize=9,
            )

    # ─ Plot 4: Distribusi error RSSI (simulasi − FSPL) ────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    if "rssi" in m:
        deltas = m["rssi"]["values"]
        ax4.hist(deltas, bins=45, color="steelblue", edgecolor="white", alpha=0.85)
        bias = m["rssi"]["bias"]
        ax4.axvline(0,    color="black",  linewidth=1.5, label="FSPL referensi (Δ=0)")
        ax4.axvline(bias, color="red",    linestyle="--",
                    label=f"Bias = {bias:+.1f} dB")
        ax4.set_xlabel("RSSI − FSPL (dB)  [negatif = lebih lemah dari free-space]")
        ax4.set_ylabel("Jumlah link")
        ax4.set_title("Distribusi Error Model Propagasi")
        ax4.legend(fontsize=8)
        ax4.grid(True, alpha=0.3)

    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Plot tersimpan → {save_path}")
    plt.show()


# ── Entry point ────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analisis error simulasi Hiking LoRa (standalone, tidak butuh ROS)"
    )
    parser.add_argument("--pos", required=True,
                        help="Path ke positions.csv dari data_logger_node")
    parser.add_argument("--net", required=True,
                        help="Path ke network_events.csv dari data_logger_node")
    parser.add_argument("--out", default="analysis_result.png",
                        help="Nama file output plot PNG (default: analysis_result.png)")
    args = parser.parse_args()

    if not os.path.exists(args.pos):
        print(f"ERROR: File tidak ditemukan: {args.pos}")
        return
    if not os.path.exists(args.net):
        print(f"ERROR: File tidak ditemukan: {args.net}")
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
