"""
Gambar tambahan Bab 10 — siLacak
fig12: Profil Cuaca dan Dampak Komunikasi LoRa (Persamaan 4.7)
fig13: Model Redaman Vegetasi Weissberger / ITU-R P.833 (Persamaan 4.11)
fig14: Analisis RSSI Sepanjang Ridge Route & Pergantian Relay Dijkstra
fig15: Snapshot 3D Jaringan Multi-Pendaki (Gazebo-like)
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from matplotlib.patches import Circle, FancyBboxPatch, FancyArrowPatch
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as mticker
import warnings
warnings.filterwarnings('ignore')

sys.path.insert(0, '/home/ammar/Documents/Simulasi_Hiking/src/hiking_lora_sim')
from hiking_lora_sim.scenario import (
    terrain_height_world, LORA_NODES, BASE_STATION,
    RADIO_OBSTACLES, TRAILS, DEFAULT_METERS_PER_WORLD_UNIT
)

OUT = "/home/ammar/Documents/Simulasi_Hiking/figures_bab10"
DPI = 180
BG  = "#fafafa"
M   = DEFAULT_METERS_PER_WORLD_UNIT   # 35 m / world-unit

C_GREEN  = "#2e7d32"
C_YELLOW = "#f9a825"
C_RED    = "#c62828"
C_BLUE   = "#1565c0"
C_GRAY   = "#546e7a"
C_ORANGE = "#e65100"
C_PURPLE = "#6a1b9a"
C_TEAL   = "#00695c"
C_BS     = "#c62828"
C_RELAY  = "#1565c0"
C_HIKER  = "#e65100"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.facecolor": BG,
    "axes.facecolor": BG,
})

# ─── Data profil cuaca (dari lora_network.py) ───────────────────────────────────
WEATHERS = [
    ("Cerah\n(Clear)",       0.00, 0.0,  0.00, 1.00, 1.00, "#4caf50"),
    ("Berkabut\n(Fog)",      0.00, 0.5,  0.00, 1.15, 0.85, "#90a4ae"),
    ("Hujan Ringan\n(Light Rain)", 0.01, 1.0,  0.00, 1.40, 0.80, "#42a5f5"),
    ("Hujan Lebat\n(Heavy Rain)",  0.05, 3.0,  0.05, 1.70, 0.65, "#1565c0"),
    ("Badai Petir\n(Thunderstorm)", 0.10, 8.0, 0.15, 2.00, 0.50, "#6a1b9a"),
]
# kolom: label, attn_db_km, noise_db, extra_drop_prob, wet_foliage, speed_factor, color

_SF_SENSITIVITY = {7: -123, 8: -126, 9: -129, 10: -132, 11: -134.5, 12: -136}


# ══════════════════════════════════════════════════════════════════════════════
# Utilitas
# ══════════════════════════════════════════════════════════════════════════════

def fspl_db(d_m, f_mhz=923.0):
    d_m = np.maximum(d_m, 1.0)
    return 20 * np.log10(d_m / 1000) + 20 * np.log10(f_mhz) + 32.44


def weissberger(d_v_m, f_ghz=0.923):
    """Redaman vegetasi model Weissberger / ITU-R P.833 (Persamaan 4.11)."""
    if d_v_m <= 0:
        return 0.0
    if d_v_m <= 14:
        return 0.45 * f_ghz ** 0.284 * d_v_m
    else:
        return 0.2 * f_ghz ** 0.3 * d_v_m ** 0.6


def chord_through_circle(px, py, nx, ny, cx, cy, r):
    """Panjang korda lintasan (px,py)→(nx,ny) yang melewati lingkaran (cx,cy,r)."""
    dx, dy = nx - px, ny - py
    fx, fy = px - cx, py - cy
    a = dx * dx + dy * dy
    if a < 1e-12:
        return 0.0
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4 * a * c
    if disc < 0:
        return 0.0
    t1 = max(0.0, min(1.0, (-b - disc ** 0.5) / (2 * a)))
    t2 = max(0.0, min(1.0, (-b + disc ** 0.5) / (2 * a)))
    if t2 <= t1:
        return 0.0
    return ((dx * (t2 - t1)) ** 2 + (dy * (t2 - t1)) ** 2) ** 0.5 * M


def check_los(hx, hy, hz_wu, nx, ny, nz_wu, n=24):
    """True jika tidak ada terrain yang menembus garis lurus antar dua titik."""
    for t in np.linspace(0.06, 0.94, n):
        xi = hx + t * (nx - hx)
        yi = hy + t * (ny - hy)
        zi = hz_wu + t * (nz_wu - hz_wu)
        if terrain_height_world(xi, yi) > zi + 0.4:
            return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 12 — Profil Cuaca dan Dampak Sinyal LoRa
# ══════════════════════════════════════════════════════════════════════════════
def fig12_weather_impact():
    fig = plt.figure(figsize=(15, 10), facecolor=BG)
    fig.suptitle(
        "Gambar 10.14 — Profil Cuaca dan Dampak terhadap Komunikasi LoRa siLacak\n"
        "Parameter cuaca memengaruhi atenuasi, noise floor, drop paket, vegetasi basah, "
        "dan kecepatan pendaki",
        fontsize=12, fontweight='bold'
    )
    gs = GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.38,
                  left=0.07, right=0.97, top=0.90, bottom=0.08)

    labels    = [w[0] for w in WEATHERS]
    attn      = np.array([w[1] for w in WEATHERS])
    noise     = np.array([w[2] for w in WEATHERS])
    drop_prob = np.array([w[3] for w in WEATHERS]) * 100  # %
    wet_foil  = np.array([w[4] for w in WEATHERS])
    speed     = np.array([w[5] for w in WEATHERS]) * 100  # %
    colors    = [w[6] for w in WEATHERS]
    short     = ["Cerah", "Berkabut", "Hujan\nRingan", "Hujan\nLebat", "Badai\nPetir"]

    x = np.arange(len(WEATHERS))

    # ── (a) Radar chart ─────────────────────────────────────────────────────
    ax_radar = fig.add_subplot(gs[0, 0], projection='polar')
    ax_radar.set_facecolor(BG)

    cat_labels = [
        "Atenuasi\n(×10 dB/km)", "Noise\nTambahan\n(dB / 8)",
        "P(drop)\nEkstra\n(% / 15)", "Faktor\nBasah\nVegetasi",
        "Kecepatan\nPendaki\n(× 100%)"
    ]
    n_cat = len(cat_labels)
    angles = np.linspace(0, 2 * np.pi, n_cat, endpoint=False).tolist()
    angles += angles[:1]

    ax_radar.set_theta_offset(np.pi / 2)
    ax_radar.set_theta_direction(-1)
    ax_radar.set_thetagrids(np.degrees(angles[:-1]), cat_labels, fontsize=7.5)
    ax_radar.set_ylim(0, 1)
    ax_radar.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax_radar.set_yticklabels(['', '0.5', '', '1.0'], fontsize=7)
    ax_radar.grid(color='gray', alpha=0.35)

    for i, (lbl, col) in enumerate(zip(short, colors)):
        vals = [
            attn[i] / 0.10,
            noise[i] / 8.0,
            drop_prob[i] / 15.0,
            (wet_foil[i] - 1.0) / 1.0,
            speed[i] / 100.0,
        ]
        vals += vals[:1]
        ax_radar.plot(angles, vals, color=col, lw=2.0, label=lbl)
        ax_radar.fill(angles, vals, color=col, alpha=0.12)

    ax_radar.set_title("(a) Radar Parameter\n5 Profil Cuaca",
                        fontsize=9.5, fontweight='bold', pad=20)
    ax_radar.legend(loc='lower right', bbox_to_anchor=(1.35, -0.12),
                    fontsize=7.5, framealpha=0.9)

    # ── (b) RSSI degradation vs Jarak ──────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_facecolor(BG)
    d_km = np.linspace(0.1, 5.0, 300)
    fspl_curve = fspl_db(d_km * 1000)

    for lbl, attn_v, noise_v, _, _, _, col in WEATHERS:
        rssi = 17 + 4 - fspl_curve - attn_v * d_km - noise_v
        ax2.plot(d_km, rssi, color=col, lw=2.3, label=lbl.replace('\n', ' '))

    for sf, sens in [(9, -129), (11, -134.5), (12, -136)]:
        ax2.axhline(sens, color=C_GRAY, lw=1.0, ls='--', alpha=0.6)
        ax2.text(4.85, sens + 0.8, f"SF{sf}={sens}dBm",
                 fontsize=7.5, color=C_GRAY, ha='right', alpha=0.85)

    ax2.set_xlabel("Jarak link (km)", fontsize=9.5)
    ax2.set_ylabel("RSSI efektif (dBm)", fontsize=9.5)
    ax2.set_xlim(0.1, 5.0)
    ax2.set_title("(b) RSSI vs Jarak per Profil Cuaca\n"
                  "(TX 17 dBm, G=4 dB, LOS, 923 MHz)",
                  fontsize=9.5, fontweight='bold')
    ax2.legend(fontsize=7.5, loc='upper right', framealpha=0.9, ncol=1)
    ax2.grid(True, alpha=0.2)

    # ── (c) Penurunan RSSI kuantitas pada jarak tetap 1.5 km ───────────────
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.set_facecolor(BG)
    d_ref = 1.5  # km
    fspl_ref = fspl_db(d_ref * 1000)
    base_rssi = 17 + 4 - fspl_ref

    rssi_w = []
    for _, attn_v, noise_v, _, _, _, col in WEATHERS:
        rssi_w.append(base_rssi - attn_v * d_ref - noise_v)

    bars = ax3.barh(short, [base_rssi - r for r in rssi_w],
                    color=colors, alpha=0.85, edgecolor='white', lw=1.2)
    for bar, r in zip(bars, rssi_w):
        ax3.text(bar.get_width() + 0.05,
                 bar.get_y() + bar.get_height() / 2,
                 f"RSSI = {r:.1f} dBm",
                 va='center', fontsize=8.5, color=C_GRAY)

    ax3.set_xlabel("Penurunan RSSI dibanding 'Cerah' (dB)", fontsize=9)
    ax3.set_title(f"(c) Degradasi RSSI pada d = {d_ref} km\n"
                   "RSSI dasar (Cerah) = "
                   f"{base_rssi:.1f} dBm",
                  fontsize=9.5, fontweight='bold')
    ax3.grid(True, axis='x', alpha=0.2)
    ax3.set_xlim(0, max(base_rssi - r for r in rssi_w) * 1.4 + 0.1)

    # ── (d) Faktor basah vegetasi → L_veg aktual ───────────────────────────
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.set_facecolor(BG)
    # Redaman Weissberger di d_v = 100 m (chord obstacle hutan lebat)
    d_chord_m = 100.0
    base_veg = weissberger(d_chord_m)
    veg_wet = [base_veg * w[4] for w in WEATHERS]

    bars4 = ax4.bar(short, veg_wet, color=colors, alpha=0.85,
                    edgecolor='white', lw=1.2, width=0.6)
    for bar, v in zip(bars4, veg_wet):
        ax4.text(bar.get_x() + bar.get_width() / 2, v + 0.05,
                 f"{v:.2f} dB", ha='center', fontsize=8.5, color=C_GRAY,
                 fontweight='bold')

    ax4.axhline(base_veg, color=C_GREEN, lw=1.8, ls='--',
                label=f"L_veg dasar (cerah) = {base_veg:.2f} dB")
    ax4.set_ylabel("L_veg efektif (dB)", fontsize=9.5)
    ax4.set_title(f"(d) Redaman Vegetasi Weissberger × Faktor Basah\n"
                   f"Panjang korda d_v = {d_chord_m:.0f} m, f = 0,923 GHz",
                  fontsize=9.5, fontweight='bold')
    ax4.legend(fontsize=8.5, framealpha=0.9)
    ax4.grid(True, axis='y', alpha=0.2)
    ax4.set_ylim(0, max(veg_wet) * 1.25)

    # ── (e) Kecepatan pendaki + probabilitas drop ekstra ───────────────────
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.set_facecolor(BG)
    w_pos = np.arange(len(WEATHERS))
    bars5a = ax5.bar(w_pos - 0.2, speed, 0.35, color=colors, alpha=0.85,
                     label="Kecepatan pendaki (%)", edgecolor='white', lw=1.2)
    for bar, s in zip(bars5a, speed):
        ax5.text(bar.get_x() + bar.get_width() / 2, s + 0.8,
                 f"{s:.0f}%", ha='center', fontsize=8.5, color=C_GRAY)

    ax5_r = ax5.twinx()
    ax5_r.spines['top'].set_visible(False)
    bars5b = ax5_r.bar(w_pos + 0.2, drop_prob, 0.35, color=colors, alpha=0.50,
                       hatch='//', label="P(drop) ekstra (%)", edgecolor='white', lw=1.2)
    for bar, d in zip(bars5b, drop_prob):
        if d > 0:
            ax5_r.text(bar.get_x() + bar.get_width() / 2, d + 0.2,
                       f"{d:.0f}%", ha='center', fontsize=8.5, color=C_GRAY)

    ax5.set_xticks(w_pos)
    ax5.set_xticklabels(short, fontsize=8.5)
    ax5.set_ylabel("Kecepatan pendaki (%)", fontsize=9.5)
    ax5_r.set_ylabel("Probabilitas drop ekstra (%)", fontsize=9.5)
    ax5.set_ylim(0, 130)
    ax5_r.set_ylim(0, 25)
    ax5.set_title("(e) Kecepatan Pendaki & Probabilitas\nDrop Paket Ekstra per Profil Cuaca",
                  fontsize=9.5, fontweight='bold')
    h1, l1 = ax5.get_legend_handles_labels()
    h2, l2 = ax5_r.get_legend_handles_labels()
    ax5.legend(h1 + h2, l1 + l2, fontsize=8, loc='lower left', framealpha=0.9)
    ax5.grid(True, axis='y', alpha=0.2)

    # ── (f) Tabel ringkasan ────────────────────────────────────────────────
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis('off')
    rows = []
    for w in WEATHERS:
        lbl = w[0].replace('\n', ' ')
        rows.append([lbl,
                     f"{w[1]:.2f}",
                     f"{w[2]:.1f}",
                     f"{w[3]*100:.0f}",
                     f"×{w[4]:.2f}",
                     f"{w[5]*100:.0f}%"])
    col_labels = ["Profil\nCuaca", "Atenuasi\n(dB/km)", "Noise\n(dB)",
                  "P_drop\nekstra (%)", "Faktor\nBasah", "Kecepatan\nPendaki"]
    tbl = ax6.table(cellText=rows, colLabels=col_labels,
                    loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1.0, 1.55)
    for j in range(len(col_labels)):
        tbl[0, j].set_facecolor("#455a64")
        tbl[0, j].set_text_props(color='white', fontweight='bold')
    for i, (_, _, _, _, _, _, col) in enumerate(WEATHERS):
        for j in range(len(col_labels)):
            tbl[i + 1, j].set_facecolor(col + "30")
    ax6.set_title("(f) Tabel Parameter Profil Cuaca\n(nilai diimplementasikan dalam simulasi)",
                  fontsize=9.5, fontweight='bold', pad=8)

    plt.savefig(f"{OUT}/fig12_weather_impact.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig12_weather_impact.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 13 — Model Redaman Vegetasi Weissberger (Persamaan 4.11)
# ══════════════════════════════════════════════════════════════════════════════
def fig13_weissberger():
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor=BG)
    fig.suptitle(
        "Gambar 10.15 — Model Redaman Vegetasi Weissberger / ITU-R P.833\n"
        "Persamaan 4.11a dan 4.11b: dua rezim berdasarkan kedalaman penetrasi d_v",
        fontsize=12, fontweight='bold'
    )

    f_ghz = 0.923

    # ── (a) L_veg vs d_v — dua rezim ───────────────────────────────────────
    ax = axes[0]
    ax.set_facecolor(BG)

    d_short = np.linspace(0.1, 14, 100)
    d_long  = np.linspace(14, 400, 300)

    lveg_short = np.array([weissberger(d, f_ghz) for d in d_short])
    lveg_long  = np.array([weissberger(d, f_ghz) for d in d_long])
    lv_max     = weissberger(400, f_ghz)

    ax.plot(d_short, lveg_short, color=C_BLUE, lw=2.8,
            label=r"$L_{veg} = 0{,}45 \cdot f^{0.284} \cdot d_v$"
                  r"\n(rezim pendek, $d_v \leq 14$ m)")
    ax.plot(d_long, lveg_long, color=C_GREEN, lw=2.8,
            label=r"$L_{veg} = 0{,}2 \cdot f^{0.3} \cdot d_v^{0.6}$"
                  "\n(rezim panjang, $d_v > 14$ m)")

    # Breakpoint
    lveg_break = weissberger(14, f_ghz)
    ax.plot(14, lveg_break, 'o', color=C_RED, ms=12, zorder=6,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.axvline(14, color=C_RED, lw=1.5, ls=':', alpha=0.7)
    ax.annotate(f"Batas rezim: d_v = 14 m\nL_veg = {lveg_break:.2f} dB",
                xy=(14, lveg_break), xytext=(55, lveg_break - 1.8),
                fontsize=9, color=C_RED, fontweight='bold',
                arrowprops=dict(arrowstyle='->', color=C_RED, lw=1.3),
                bbox=dict(fc='white', ec=C_RED, boxstyle='round,pad=0.25'))

    # Titik referensi praktis sepanjang kurva
    refs = [(50, C_TEAL), (100, C_ORANGE), (200, C_PURPLE), (400, C_GRAY)]
    for d_ref, rcol in refs:
        lv_ref = weissberger(d_ref, f_ghz)
        ax.plot(d_ref, lv_ref, 's', color=rcol, ms=9, zorder=6,
                markeredgecolor='white', markeredgewidth=1.0)
        ax.text(d_ref + 5, lv_ref - 0.35,
                f"d_v={d_ref}m\n{lv_ref:.1f}dB",
                fontsize=7.5, color=rcol, fontweight='bold')

    # Catatan obstacle simulasi — sebagai text box, bukan scatter di kurva
    # (chord aktual 700–1540 m >> 400 m; model extrapolated)
    obs_info = "\n".join([
        f"Obstacle nyata dalam simulasi (korda maks ≈ 2×r×35 m):",
        f"• lower_dense_forest: r=22wu → korda≈1540m (loss_db=14 dB)",
        f"• valley_forest:      r=20wu → korda≈1400m (loss_db=11 dB)",
        f"",
        f"Korda > 400m → ekstrapolasi formula Pers. 4.11b",
    ])
    ax.text(0.98, 0.05, obs_info, transform=ax.transAxes,
            fontsize=7.5, va='bottom', ha='right', color=C_GRAY,
            bbox=dict(fc='white', ec=C_GRAY, alpha=0.88,
                      boxstyle='round,pad=0.35'))

    # Garis referensi loss obstacle di sisi kanan Y
    for loss_ref, lbl, lcol in [(14.0, "loss_db hutan lebat\n(14 dB)", C_TEAL),
                                  (11.0, "loss_db hutan valley\n(11 dB)", C_ORANGE)]:
        ax.axhline(loss_ref, color=lcol, lw=1.5, ls='--', alpha=0.65)
        ax.text(415, loss_ref, lbl, fontsize=7.5, color=lcol,
                va='center', clip_on=False)

    ax.set_xlabel("Kedalaman penetrasi vegetasi d_v (m)", fontsize=11)
    ax.set_ylabel("Redaman vegetasi L_veg (dB)", fontsize=11)
    ax.set_title("(a) Kurva Weissberger di 923 MHz — dua rezim\n"
                 "Titik ■ = referensi praktis; garis -- = loss_db obstacle simulasi",
                 fontsize=10, fontweight='bold')
    ax.legend(fontsize=9, framealpha=0.9, loc='upper left')
    ax.grid(True, alpha=0.25)
    ax.set_xlim(0, 410)
    ax.set_ylim(0, max(lv_max, 14.5) * 1.15)

    # ── (b) Efek faktor basah per profil cuaca, referensi d_v nyata ────────
    ax2 = axes[1]
    ax2.set_facecolor(BG)

    d_plot = np.linspace(0.1, 400, 600)
    lveg_dry = np.array([weissberger(d, f_ghz) for d in d_plot])

    weather_wet = [
        ("Cerah",        1.00, "#4caf50", '-'),
        ("Berkabut",     1.15, "#90a4ae", '--'),
        ("Hujan Ringan", 1.40, "#42a5f5", '-.'),
        ("Hujan Lebat",  1.70, "#1565c0", ':'),
        ("Badai Petir",  2.00, "#6a1b9a", (0, (3, 1, 1, 1))),
    ]
    for wname, wfactor, wcol, wls in weather_wet:
        ax2.plot(d_plot, lveg_dry * wfactor, color=wcol, lw=2.3,
                 linestyle=wls, label=f"{wname} (×{wfactor:.2f})")

    # Garis horizontal loss_db obstacle nyata
    ref_losses = [
        (14.0, C_TEAL,   "Hutan lebat\n(loss=14 dB)"),
        (11.0, C_ORANGE, "Hutan valley\n(loss=11 dB)"),
    ]
    for loss_v, lcol, llbl in ref_losses:
        ax2.axhline(loss_v, color=lcol, lw=1.8, ls='--', alpha=0.75)
        ax2.text(d_plot[-1] * 0.98, loss_v + 0.2, llbl,
                 fontsize=8, color=lcol, ha='right', fontweight='bold',
                 bbox=dict(fc='white', ec=lcol, alpha=0.8,
                           boxstyle='round,pad=0.15'))

    # Shaded region: d_v di mana kondisi cerah mencapai masing-masing loss_db
    for loss_v, lcol, _ in ref_losses:
        # Cari d_v dry yang memberikan loss_v
        mask = np.abs(lveg_dry - loss_v) < 0.15
        if mask.any():
            d_cross = d_plot[mask][0]
            ax2.axvline(d_cross, color=lcol, lw=1.2, ls=':', alpha=0.55)
            ax2.text(d_cross + 3, loss_v * 0.35,
                     f"d_v≈{d_cross:.0f}m\n(cerah)", fontsize=7.5,
                     color=lcol, va='center', alpha=0.85)

    ax2.set_xlabel("Kedalaman penetrasi vegetasi d_v (m)", fontsize=11)
    ax2.set_ylabel("L_veg efektif (dB) — Weissberger × faktor basah", fontsize=11)
    ax2.set_title("(b) Pengaruh Kondisi Cuaca pada L_veg\n"
                  "Garis -- = loss_db obstacle nyata dalam simulasi siLacak",
                  fontsize=10, fontweight='bold')
    ax2.legend(fontsize=9, framealpha=0.9, loc='upper left')
    ax2.grid(True, alpha=0.25)
    ax2.set_xlim(0, 410)
    ax2.set_ylim(0, lveg_dry.max() * 2.1 * 1.05)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig13_weissberger.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig13_weissberger.png")


# ══════════════════════════════════════════════════════════════════════════════
# Fungsi bantu: hitung RSSI sepanjang rute (dari data world nyata)
# ══════════════════════════════════════════════════════════════════════════════
def compute_rssi_along_trail(trail_name='ridge_route', n_pts=100):
    pts = TRAILS[trail_name]
    t0 = np.linspace(0, 1, len(pts))
    tf = np.linspace(0, 1, n_pts)
    xs = np.interp(tf, t0, [p[0] for p in pts])
    ys = np.interp(tf, t0, [p[1] for p in pts])

    # Jarak kumulatif dalam km
    dx = np.diff(xs) * M
    dy = np.diff(ys) * M
    dist_km = np.concatenate([[0], np.cumsum((dx ** 2 + dy ** 2) ** 0.5)]) / 1000

    MAST_WU  = 10.0 / M    # tinggi mast relay dalam world-units
    HIKER_WU = 1.5  / M    # tinggi antena pendaki

    all_nodes = [BASE_STATION] + LORA_NODES
    n_nodes   = len(all_nodes)

    rssi_mat  = np.zeros((n_pts, n_nodes))
    los_mat   = np.zeros((n_pts, n_nodes), dtype=bool)
    veg_mat   = np.zeros((n_pts, n_nodes))

    for i, (hx, hy) in enumerate(zip(xs, ys)):
        hz = terrain_height_world(hx, hy) + HIKER_WU

        for j, node in enumerate(all_nodes):
            nz = terrain_height_world(node.x, node.y) + MAST_WU
            d3d_wu = ((hx - node.x) ** 2 + (hy - node.y) ** 2 +
                      (hz - nz) ** 2) ** 0.5
            d3d_m  = d3d_wu * M

            # FSPL
            fspl = fspl_db(d3d_m)

            # Redaman vegetasi (hanya obstacle jenis 'trees')
            total_veg = 0.0
            for obs in RADIO_OBSTACLES:
                if obs.kind == 'trees':
                    chord_m = chord_through_circle(
                        hx, hy, node.x, node.y,
                        obs.x, obs.y, obs.radius
                    )
                    total_veg += weissberger(chord_m)
            veg_mat[i, j] = total_veg

            # LOS check → n=3.5 untuk NLOS
            is_los = check_los(hx, hy, hz, node.x, node.y, nz)
            los_mat[i, j] = is_los
            nlos_extra = 0.0
            if not is_los:
                nlos_extra = max(0, 10 * 1.5 * np.log10(max(d3d_m, 100) / 100))

            rssi_mat[i, j] = 17 + 4 - fspl - total_veg - nlos_extra

    return dist_km, rssi_mat, los_mat, veg_mat, all_nodes, xs, ys


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 14 — RSSI Sepanjang Ridge Route & Pergantian Relay Dijkstra
# ══════════════════════════════════════════════════════════════════════════════
def fig14_rssi_trail():
    print("  Menghitung RSSI sepanjang ridge_route...", flush=True)
    dist_km, rssi_mat, los_mat, veg_mat, all_nodes, xs, ys = \
        compute_rssi_along_trail('ridge_route', n_pts=100)

    all_names = [n.name for n in all_nodes]
    node_colors = {
        'base_station':       C_RED,
        'node_basecamp_gate': '#ef6c00',
        'node_valley_watch':  '#558b2f',
        'node_forest_pass':   '#00838f',
        'node_ridge_mid':     C_BLUE,
        'node_crater_edge':   C_PURPLE,
        'node_upper_traverse':'#ad1457',
        'node_north_saddle':  '#4527a0',
        'node_summit_view':   '#00695c',
    }

    # Best relay index di setiap titik (relay yang memberi RSSI tertinggi)
    best_idx    = np.argmax(rssi_mat, axis=1)
    best_rssi   = rssi_mat[np.arange(len(dist_km)), best_idx]
    sensitivity_sf9 = -129.0
    link_margin = best_rssi - sensitivity_sf9

    # Elevasi rute dalam meter nyata
    elev_m = np.array([terrain_height_world(x, y) * M
                       for x, y in zip(xs, ys)])
    elev_km = elev_m / 1000

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), facecolor=BG,
                              gridspec_kw={'height_ratios': [2.5, 2.5, 1.2]})
    fig.suptitle(
        "Gambar 10.16 — RSSI Sepanjang Ridge Route dan Pergantian Relay Dijkstra\n"
        "Dihitung dari koordinat nyata terrain_height_world() — data aktual simulasi",
        fontsize=12, fontweight='bold'
    )

    # ── Panel atas: RSSI per node ──────────────────────────────────────────
    ax1 = axes[0]
    ax1.set_facecolor(BG)

    # Sensitivitas SF
    sf_lines = [(9, -129, '-.'), (11, -134.5, ':'), (12, -136, '--')]
    for sf, sens, ls in sf_lines:
        ax1.axhline(sens, color=C_GRAY, lw=1.0, ls=ls, alpha=0.55)
        ax1.text(dist_km[-1] * 1.005, sens, f"SF{sf}={sens}dBm",
                 fontsize=7.5, color=C_GRAY, va='center', alpha=0.85)

    # RSSI per relay
    for j, node in enumerate(all_nodes):
        col = node_colors.get(node.name, C_GRAY)
        short = node.name.replace('node_', '').replace('_', ' ').title()
        ax1.plot(dist_km, rssi_mat[:, j], color=col, lw=1.8, alpha=0.7,
                 label=short)

    # Best relay bold
    ax1.plot(dist_km, best_rssi, color='black', lw=3.0, ls='-', alpha=0.9,
             label='Best relay (Dijkstra)', zorder=5)

    ax1.set_ylabel("RSSI (dBm)", fontsize=10)
    ax1.set_title("(a) RSSI ke Setiap Relay Node — Data Terrain Nyata",
                  fontsize=10, fontweight='bold', loc='left')
    ax1.legend(fontsize=7.5, loc='lower right', framealpha=0.92,
               ncol=3, title="Node", title_fontsize=8)
    ax1.set_xlim(0, dist_km[-1])
    ax1.set_ylim(-160, -50)
    ax1.fill_between(dist_km, -160, sensitivity_sf9,
                     alpha=0.06, color=C_RED, label='_noleg')
    ax1.text(dist_km[-1] * 0.5, -153,
             "Zone tidak terjangkau SF9 (di bawah sensitivitas)",
             ha='center', fontsize=8.5, color=C_RED, alpha=0.75)
    ax1.grid(True, alpha=0.2)
    ax1.set_xticklabels([])

    # ── Panel tengah: link margin + LOS/NLOS + relay handoff ──────────────
    ax2 = axes[1]
    ax2.set_facecolor(BG)

    # LOS/NLOS background menggunakan best relay LOS status
    best_los = los_mat[np.arange(len(dist_km)), best_idx]
    for i in range(len(dist_km) - 1):
        col_bg = "#e8f5e9" if best_los[i] else "#ffebee"
        ax2.axvspan(dist_km[i], dist_km[i + 1], alpha=0.6, color=col_bg, lw=0)

    # Relay handoff boundaries — only label if sufficiently spaced
    prev_best = best_idx[0]
    last_label_km = -999.0
    min_label_gap = 0.75  # km
    label_y_positions = [link_margin.max() * 0.97, link_margin.max() * 0.70,
                         link_margin.max() * 0.43]
    label_y_cycle = 0
    for i, bidx in enumerate(best_idx):
        if bidx != prev_best:
            ax2.axvline(dist_km[i], color=C_ORANGE, lw=1.8, ls='-', alpha=0.7, zorder=4)
            if dist_km[i] - last_label_km >= min_label_gap:
                old_name = all_nodes[prev_best].name.replace('node_','').replace('_','\n').title()
                new_name = all_nodes[bidx].name.replace('node_','').replace('_','\n').title()
                y_pos = label_y_positions[label_y_cycle % len(label_y_positions)]
                ax2.text(dist_km[i] + 0.04, y_pos,
                         f"{old_name}\n→{new_name}",
                         fontsize=6.5, color=C_ORANGE, va='top',
                         bbox=dict(fc='white', ec=C_ORANGE, alpha=0.88,
                                   boxstyle='round,pad=0.12'))
                last_label_km = dist_km[i]
                label_y_cycle += 1
            prev_best = bidx

    # Link margin
    ax2.plot(dist_km, link_margin, color=C_BLUE, lw=2.5, label='Link Margin SF9 (dB)')
    ax2.fill_between(dist_km, 0, link_margin,
                     where=(link_margin >= 0), alpha=0.18, color=C_GREEN)
    ax2.fill_between(dist_km, link_margin, 0,
                     where=(link_margin < 0), alpha=0.25, color=C_RED)
    ax2.axhline(0, color=C_RED, lw=1.8, ls='--', alpha=0.7, label='Margin = 0 dB (batas layak)')
    ax2.axhline(10, color=C_GREEN, lw=1.3, ls=':', alpha=0.6, label='Target margin ≥ 10 dB')

    # Custom legend untuk LOS/NLOS background
    los_patch  = mpatches.Patch(color='#e8f5e9', label='LOS (tidak ada terrain menghalangi)')
    nlos_patch = mpatches.Patch(color='#ffebee', label='NLOS (terrain menghalangi + n=3,5)')
    ho_line    = Line2D([0], [0], color=C_ORANGE, lw=2.0,
                        label='Pergantian relay (handoff)')

    ax2.set_ylabel("Link Margin terhadap SF9 (dB)", fontsize=10)
    ax2.set_title("(b) Link Margin Best-Relay (SF9) + Kondisi LOS/NLOS + Relay Handoff",
                  fontsize=10, fontweight='bold', loc='left')
    handles, lbls = ax2.get_legend_handles_labels()
    ax2.legend(handles + [los_patch, nlos_patch, ho_line],
               lbls + [los_patch.get_label(), nlos_patch.get_label(), ho_line.get_label()],
               fontsize=7.5, loc='upper right', framealpha=0.92, ncol=2)
    ax2.set_xlim(0, dist_km[-1])
    ax2.grid(True, alpha=0.2)
    ax2.set_xticklabels([])

    # ── Panel bawah: elevasi rute ──────────────────────────────────────────
    ax3 = axes[2]
    ax3.set_facecolor(BG)
    ax3.fill_between(dist_km, 0, elev_m, color='#795548', alpha=0.4)
    ax3.plot(dist_km, elev_m, color='#4e342e', lw=1.8)

    # Relay nodes di sepanjang rute ridge
    MAST_WU = 10.0 / M
    for node in all_nodes:
        col = node_colors.get(node.name, C_GRAY)
        nz_m = terrain_height_world(node.x, node.y) * M
        dists_to_n = [((node.x - x) ** 2 + (node.y - y) ** 2) ** 0.5
                      for x, y in zip(xs, ys)]
        idx_c = int(np.argmin(dists_to_n))
        if dists_to_n[idx_c] < 18:
            nd_km = dist_km[idx_c]
            ax3.plot(nd_km, nz_m + 40, '^', color=col, ms=10,
                     markeredgecolor='white', zorder=5)
            short = node.name.replace('node_', '').replace('_', '\n')
            ax3.text(nd_km, nz_m + 95, short, ha='center',
                     fontsize=6.5, color=col, fontweight='bold',
                     bbox=dict(fc='white', ec=col, alpha=0.85,
                               boxstyle='round,pad=0.1'))

    ax3.set_xlabel("Jarak dari basecamp sepanjang ridge_route (km)", fontsize=10)
    ax3.set_ylabel("Ketinggian\n(m nyata)", fontsize=9)
    ax3.set_title("(c) Profil Elevasi Ridge Route (terrain_height_world × 35 m/wu)",
                  fontsize=10, fontweight='bold', loc='left')
    ax3.set_xlim(0, dist_km[-1])
    ax3.set_ylim(bottom=0)
    ax3.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig14_rssi_trail.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig14_rssi_trail.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 15 — Snapshot 3D Multi-Pendaki (Gazebo-like)
# ══════════════════════════════════════════════════════════════════════════════
def fig15_network_snapshot_3d():
    fig = plt.figure(figsize=(16, 10), facecolor='#0d1117')
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#0d1117')

    # ── Terrain surface (resolusi lebih kasar untuk kecepatan) ────────────
    N3 = 90
    X3 = np.linspace(-120, 100, N3)
    Y3 = np.linspace(-110, 100, N3)
    XX3, YY3 = np.meshgrid(X3, Y3)
    ZZ3 = np.vectorize(terrain_height_world)(XX3, YY3)

    norm3d = mcolors.Normalize(vmin=ZZ3.min(), vmax=ZZ3.max())
    # Warna terrain: stone/earth tones dengan alpha rendah
    from matplotlib import cm
    terrain_colors = cm.terrain(norm3d(ZZ3))
    terrain_colors[..., 3] = 0.70  # alpha lebih rendah → link lebih terlihat

    ax.plot_surface(XX3, YY3, ZZ3, facecolors=terrain_colors,
                    rstride=2, cstride=2, linewidth=0,
                    antialiased=False, zorder=1)

    # ── Kontur di dasar (bayangan terrain) ────────────────────────────────
    z_base = ZZ3.min() - 2
    ax.contourf(XX3, YY3, ZZ3, levels=15, cmap='terrain',
                alpha=0.25, zorder=0, offset=z_base)

    # ── Rute pendakian ──────────────────────────────────────────────────
    route_cfg = {
        'ridge_route':  ('#76ff03', 2.5, '-'),
        'valley_route': ('#40c4ff', 2.0, '--'),
        'crater_route': ('#ea80fc', 2.0, ':'),
    }
    for rname, pts in TRAILS.items():
        col, lw, ls = route_cfg[rname]
        rxs = [p[0] for p in pts]
        rys = [p[1] for p in pts]
        rzs = [terrain_height_world(p[0], p[1]) + 0.8 for p in pts]
        ax.plot(rxs, rys, rzs, color=col, lw=lw, linestyle=ls,
                zorder=5, alpha=0.85)

    # ── Hiker positions (3 pendaki di 3 rute berbeda) ────────────────────
    hikers = [
        ('ridge_route',  4, '#ff9800', 'Pendaki 1\nridge_route'),
        ('valley_route', 3, '#00e5ff', 'Pendaki 2\nvalley_route'),
        ('crater_route', 6, '#ce93d8', 'Pendaki 3\ncrater_route'),
    ]
    hiker_positions = []
    for rname, wp_idx, hcol, hlbl in hikers:
        wp = TRAILS[rname][wp_idx]
        hx, hy = wp
        hz = terrain_height_world(hx, hy) + 1.2
        ax.scatter([hx], [hy], [hz], s=200, c=[hcol], marker='D',
                   zorder=8, edgecolors='white', linewidth=1.5,
                   depthshade=False)
        ax.text(hx, hy, hz + 4.5, hlbl, color=hcol, fontsize=8,
                fontweight='bold', ha='center',
                bbox=dict(fc='#0d1117', ec=hcol, alpha=0.75,
                          boxstyle='round,pad=0.2'))
        hiker_positions.append((hx, hy, hz))

    # ── Base station ───────────────────────────────────────────────────────
    bx, by = BASE_STATION.x, BASE_STATION.y
    bz = terrain_height_world(bx, by) + 2.0
    ax.scatter([bx], [by], [bz], s=350, c=['#ff1744'], marker='*',
               zorder=9, edgecolors='white', linewidth=1.5, depthshade=False)
    ax.text(bx, by, bz + 6, 'Base\nStation', color='#ff1744', fontsize=9,
            fontweight='bold', ha='center')

    # ── Relay nodes dengan mast vertikal ─────────────────────────────────
    MAST_WU = 10.0 / M
    relay_labels = {
        'node_basecamp_gate': 'BCamp\nGate',
        'node_valley_watch':  'Valley\nWatch',
        'node_forest_pass':   'Forest\nPass',
        'node_ridge_mid':     'Ridge\nMid',
        'node_crater_edge':   'Crater\nEdge',
        'node_upper_traverse':'Upper\nTraverse',
        'node_north_saddle':  'North\nSaddle',
        'node_summit_view':   'Summit\nView',
    }
    relay_pos = {}
    for node in LORA_NODES:
        nz_base = terrain_height_world(node.x, node.y)
        nz_top  = nz_base + MAST_WU
        ax.plot([node.x, node.x], [node.y, node.y], [nz_base, nz_top],
                color='#90caf9', lw=2.5, zorder=7, alpha=0.9)
        ax.scatter([node.x], [node.y], [nz_top], s=140, c=['#90caf9'],
                   marker='^', zorder=8, edgecolors='white',
                   linewidth=1.2, depthshade=False)
        short = relay_labels.get(node.name, node.name)
        ax.text(node.x, node.y, nz_top + 3.5, short,
                color='#90caf9', fontsize=7, ha='center', fontweight='bold')
        relay_pos[node.name] = (node.x, node.y, nz_top)

    relay_pos['base_station'] = (bx, by, bz)

    # ── Link radio aktif — elevated above terrain for visibility ──────────
    LINK_CLEARANCE = 9.0  # world-units di atas terrain

    def draw_elevated_link(ax, p1, p2, quality, n_seg=18):
        """Link yang selalu berada di atas terrain surface."""
        colors_q = {'good': '#00e676', 'marginal': '#ffea00', 'weak': '#ff1744'}
        lw_q     = {'good': 4.5, 'marginal': 3.5, 'weak': 2.5}
        col = colors_q[quality]
        lw  = lw_q[quality]
        ts  = np.linspace(0, 1, n_seg)
        lxs = p1[0] + (p2[0] - p1[0]) * ts
        lys = p1[1] + (p2[1] - p1[1]) * ts
        lzs_line = p1[2] + (p2[2] - p1[2]) * ts
        lzs_terr = np.array([terrain_height_world(x, y) + LINK_CLEARANCE
                              for x, y in zip(lxs, lys)])
        lzs = np.maximum(lzs_line, lzs_terr)
        ax.plot(lxs, lys, lzs, color=col, lw=lw, alpha=0.95, zorder=8,
                solid_capstyle='round')
        # Label margin di midpoint
        mi = n_seg // 2
        d_m = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5 * M
        margin = round(17 + 4 - (20*np.log10(max(d_m,1)/1000) +
                                  20*np.log10(923) + 32.44) - (-129))
        label = f"+{margin}dB" if margin >= 0 else f"{margin}dB"
        ax.text(lxs[mi], lys[mi], lzs[mi] + 3.5, label,
                color=col, fontsize=8, ha='center', fontweight='bold',
                bbox=dict(fc='#0d1117', ec=col, alpha=0.7,
                          boxstyle='round,pad=0.15'))

    hx1, hy1, hz1 = hiker_positions[0]
    hx2, hy2, hz2 = hiker_positions[1]
    hx3, hy3, hz3 = hiker_positions[2]

    # Pendaki 1 (ridge) → ridge_mid → basecamp_gate → base_station
    rm  = relay_pos['node_ridge_mid']
    bcg = relay_pos['node_basecamp_gate']
    bs  = relay_pos['base_station']
    draw_elevated_link(ax, (hx1, hy1, hz1), rm,  'good')
    draw_elevated_link(ax, rm,  bcg, 'good')
    draw_elevated_link(ax, bcg, bs,  'good')

    # Pendaki 2 (valley) → valley_watch → ridge_mid
    vw = relay_pos['node_valley_watch']
    draw_elevated_link(ax, (hx2, hy2, hz2), vw,  'marginal')
    draw_elevated_link(ax, vw,  rm,  'good')

    # Pendaki 3 (crater) → upper_traverse → north_saddle
    ut = relay_pos['node_upper_traverse']
    ns = relay_pos['node_north_saddle']
    sv = relay_pos['node_summit_view']
    draw_elevated_link(ax, (hx3, hy3, hz3), ut,  'good')
    draw_elevated_link(ax, ut,  ns,  'good')
    draw_elevated_link(ax, ns,  sv,  'marginal')

    # ── Legend ─────────────────────────────────────────────────────────────
    legend_els = [
        Line2D([0],[0], color='#76ff03', lw=2.5, label='Ridge Route'),
        Line2D([0],[0], color='#40c4ff', lw=2.0, ls='--', label='Valley Route'),
        Line2D([0],[0], color='#ea80fc', lw=2.0, ls=':', label='Crater Route'),
        Line2D([0],[0], marker='D', color='w', markerfacecolor='#ff9800',
               ms=9, label='Pendaki (Portable Node)'),
        Line2D([0],[0], marker='^', color='w', markerfacecolor='#90caf9',
               ms=9, label='Relay Node'),
        Line2D([0],[0], marker='*', color='w', markerfacecolor='#ff1744',
               ms=12, label='Base Station'),
        Line2D([0],[0], color='#69f0ae', lw=3, label='Link kuat (margin tinggi)'),
        Line2D([0],[0], color='#ffeb3b', lw=2, label='Link marginal'),
    ]
    leg = ax.legend(handles=legend_els, loc='upper left', fontsize=8,
                    framealpha=0.25, facecolor='#0d1117',
                    labelcolor='white', edgecolor='#37474f', ncol=2)

    # ── Styling ───────────────────────────────────────────────────────────
    ax.set_xlabel("X (World Units)", fontsize=9, color='#90a4ae', labelpad=8)
    ax.set_ylabel("Y (World Units)", fontsize=9, color='#90a4ae', labelpad=8)
    ax.set_zlabel("Ketinggian (wu)", fontsize=9, color='#90a4ae', labelpad=8)
    ax.tick_params(colors='#546e7a', labelsize=7)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#263238')
    ax.yaxis.pane.set_edgecolor('#263238')
    ax.zaxis.pane.set_edgecolor('#263238')
    ax.grid(True, color='#263238', lw=0.5, alpha=0.5)
    ax.view_init(elev=32, azim=-115)

    fig.suptitle(
        "Gambar 10.17 — Snapshot Jaringan LoRa Multi-Pendaki (Perspektif Gazebo)\n"
        "3 pendaki aktif pada rute berbeda, routing Dijkstra ke Base Station\n"
        "Warna link: hijau = margin tinggi, kuning = marginal",
        fontsize=11, fontweight='bold', color='white', y=0.97
    )

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig15_network_snapshot_3d.png", dpi=DPI,
                bbox_inches='tight', facecolor='#0d1117')
    plt.close()
    print("✓ fig15_network_snapshot_3d.png")


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Membuat gambar tambahan Bab 10 siLacak...\n")
    fig12_weather_impact()
    fig13_weissberger()
    fig14_rssi_trail()
    fig15_network_snapshot_3d()
    print(f"\nSelesai! Gambar tersimpan di:\n{OUT}/")
