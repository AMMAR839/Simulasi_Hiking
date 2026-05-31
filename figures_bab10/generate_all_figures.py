"""
Generator semua gambar Bab 10 - Pemodelan Simulasi siLacak
Menghasilkan 11 gambar untuk subbab 10.3.1 - 10.3.4
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import FancyArrowPatch, Circle, FancyBboxPatch, Ellipse
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
import matplotlib.ticker as mticker
import networkx as nx
import warnings
warnings.filterwarnings('ignore')

OUT = "/home/ammar/Documents/Simulasi_Hiking/figures_bab10"
DPI = 180

# ─── Palette ───────────────────────────────────────────────────────────────────
C_GREEN  = "#2e7d32"
C_YELLOW = "#f9a825"
C_RED    = "#c62828"
C_BLUE   = "#1565c0"
C_GRAY   = "#546e7a"
C_ORANGE = "#e65100"
C_PURPLE = "#6a1b9a"
C_TEAL   = "#00695c"
BG       = "#fafafa"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.facecolor": BG,
    "axes.facecolor": BG,
})

# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 1 — Penampang Terrain + Zona Fresnel (10.3.1)
# ══════════════════════════════════════════════════════════════════════════════
def fig1_terrain_fresnel():
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), facecolor=BG)
    fig.suptitle("Gambar 10.3 — Kondisi LOS, Terrain Shadow, dan Difraksi Knife-Edge\n"
                 "pada Lintasan Radio di Medan Pegunungan (Persamaan 4.12–4.14)",
                 fontsize=13, fontweight='bold', y=0.98)

    x = np.linspace(0, 10, 500)

    def terrain(x):
        base = 0.3 + 0.05 * np.sin(2 * x) + 0.02 * np.sin(7 * x)
        hill = 1.8 * np.exp(-((x - 5) ** 2) / 1.5)
        return base + hill

    terrain_y = terrain(x)

    titles = [
        "(a) Kondisi LOS — Lintasan Bebas, Zona Fresnel Tidak Terhalang",
        "(b) Terrain Shadow — Terrain Menembus Zona Fresnel Pertama",
        "(c) Difraksi Knife-Edge — Sinyal Membelok di Tepi Punggungan",
    ]

    for idx, (ax, title) in enumerate(zip(axes, titles)):
        ax.set_facecolor(BG)
        ax.fill_between(x, 0, terrain_y, color="#795548", alpha=0.55, label="Terrain")
        ax.fill_between(x, 0, np.minimum(terrain_y, 0.35), color="#4caf50", alpha=0.4)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 3.2)
        ax.set_xlabel("Jarak horizontal →", fontsize=9)
        ax.set_ylabel("Ketinggian (satuan relatif)", fontsize=9)
        ax.set_title(title, fontsize=10, fontweight='bold', loc='left', pad=4)

        tx_x, tx_y = 0.4, terrain(np.array([0.4]))[0] + 0.15
        rx_x, rx_y = 9.6, terrain(np.array([9.6]))[0] + 0.15

        # antena
        for px, py, lbl in [(tx_x, tx_y, "TX\n(Pendaki)"), (rx_x, rx_y, "RX\n(Relay/BS)")]:
            ax.plot([px, px], [terrain(np.array([px]))[0], py], color=C_BLUE, lw=2.5)
            ax.plot(px, py, 'o', color=C_BLUE, ms=8, zorder=5)
            ax.annotate(lbl, (px, py + 0.08), ha='center', fontsize=8, color=C_BLUE, fontweight='bold')

        if idx == 0:
            # LOS bersih — garis lurus
            ax.annotate("", xy=(rx_x, rx_y), xytext=(tx_x, tx_y),
                        arrowprops=dict(arrowstyle="-|>", color=C_GREEN, lw=2.2))
            # zona Fresnel (elips)
            mid_x = (tx_x + rx_x) / 2
            mid_y = (tx_y + rx_y) / 2 + 0.05
            ell = Ellipse((mid_x, mid_y), width=9.0, height=0.55,
                          angle=0, fill=False, edgecolor=C_GREEN, lw=1.5,
                          linestyle='--', alpha=0.7)
            ax.add_patch(ell)
            ax.text(mid_x, mid_y + 0.35, "Zona Fresnel ke-1\n(tidak terhalang → LOS)",
                    ha='center', fontsize=8.5, color=C_GREEN,
                    bbox=dict(fc=BG, ec=C_GREEN, alpha=0.7, boxstyle='round,pad=0.2'))
            ax.text(5.0, 0.15, "✓  Link Margin positif\n    L_ter = 0 dB",
                    ha='center', fontsize=9, color=C_GREEN,
                    bbox=dict(fc="white", ec=C_GREEN, boxstyle='round,pad=0.3'))

        elif idx == 1:
            # Terrain shadow
            top_hill = np.argmax(terrain_y)
            hill_x = x[top_hill]
            hill_y = terrain_y[top_hill]
            ax.annotate("", xy=(rx_x, rx_y), xytext=(tx_x, tx_y),
                        arrowprops=dict(arrowstyle="-|>", color=C_RED, lw=2.2,
                                        linestyle='dashed'))
            # Fresnel terhalang
            ell2 = Ellipse(((tx_x + rx_x) / 2, (tx_y + rx_y) / 2 + 0.05),
                           width=9.0, height=0.55, fill=False,
                           edgecolor=C_RED, lw=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(ell2)
            ax.annotate(f"Terrain menembus\nzona Fresnel (h={hill_y-1.1:.1f} rel.)\n→ Terrain Shadow",
                        xy=(hill_x, hill_y + 0.05), xytext=(hill_x - 1.5, 2.4),
                        fontsize=8.5, color=C_RED,
                        arrowprops=dict(arrowstyle="->", color=C_RED),
                        bbox=dict(fc="white", ec=C_RED, boxstyle='round,pad=0.2'))
            ax.text(5.0, 0.15, "✗  NLOS — L_ter dihitung\n    Pers. 4.13–4.14",
                    ha='center', fontsize=9, color=C_RED,
                    bbox=dict(fc="white", ec=C_RED, boxstyle='round,pad=0.3'))

        else:
            # Difraksi
            top_hill = np.argmax(terrain_y)
            hill_x = x[top_hill]
            hill_y = terrain_y[top_hill]
            ax.plot([tx_x, hill_x, rx_x], [tx_y, hill_y + 0.05, rx_y],
                    color=C_ORANGE, lw=2.2, linestyle='--', label="Lintasan difraksi")
            ax.plot(hill_x, hill_y + 0.05, 'v', color=C_ORANGE, ms=10, zorder=6)
            ax.annotate("Tepi knife-edge\n(titik difraksi)",
                        xy=(hill_x, hill_y + 0.08), xytext=(hill_x + 1.2, 2.6),
                        fontsize=8.5, color=C_ORANGE,
                        arrowprops=dict(arrowstyle="->", color=C_ORANGE),
                        bbox=dict(fc="white", ec=C_ORANGE, boxstyle='round,pad=0.2'))
            ax.text(5.0, 0.15, "⚠  L_ter dari Pers. 4.14\n    (loss difraksi Fresnel-Kirchhoff)",
                    ha='center', fontsize=9, color=C_ORANGE,
                    bbox=dict(fc="white", ec=C_ORANGE, boxstyle='round,pad=0.3'))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(f"{OUT}/fig1_terrain_fresnel.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig1_terrain_fresnel.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 2 — Peta 2D Zona Obstacle Radio (10.3.1)
# ══════════════════════════════════════════════════════════════════════════════
def fig2_obstacle_map():
    fig, ax = plt.subplots(figsize=(11, 9), facecolor=BG)
    ax.set_facecolor("#e8f5e9")

    # terrain gradient
    terrain_img = np.zeros((200, 200))
    for i in range(200):
        for j in range(200):
            xi, yj = j / 200 * 200 - 110, i / 200 * 200 - 100
            terrain_img[i, j] = (
                np.exp(-((xi - 74) ** 2 + (yj - 70) ** 2) / 800) * 36
                + np.exp(-((xi - 38) ** 2 + (yj - 38) ** 2) / 400) * 17
                + max(0, -xi * 0.05 - yj * 0.02) * 5
            )
    ax.imshow(terrain_img, extent=[-110, 90, -100, 100], cmap='YlOrBr',
              alpha=0.35, origin='lower', zorder=0)

    # Relay nodes
    relay_nodes = {
        "base_station":       (-106, -96, "Base Station\n(Basecamp)"),
        "node_basecamp_gate": (-91,  -78, "Basecamp\nGate"),
        "node_valley_watch":  (-66,  -23, "Valley\nWatch"),
        "node_forest_pass":   (-38,  -23, "Forest\nPass"),
        "node_ridge_mid":     (4,    -3,  "Ridge\nMid"),
        "node_crater_edge":   (54,   45,  "Crater\nEdge"),
        "node_upper_traverse":(45,   55,  "Upper\nTraverse"),
        "node_north_saddle":  (70,   64,  "North\nSaddle"),
        "node_summit_view":   (91,   81,  "Summit\nView"),
    }

    # Obstacle zones
    obstacles = [
        (-72, -50, 22, "#388e3c", 0.25, "Hutan Lebat\n(L_veg = 14 dB, Pers. 4.11)"),
        (-38, -10, 14, "#388e3c", 0.20, "Forest Pass\n(L_veg = 8-12 dB)"),
        (54,   45, 16, "#bf360c", 0.20, "Kawah\n(L_obs = 10 dB)"),
        (-10,  10, 10, "#5d4037", 0.18, "Batuan\n(L_obs = 12 dB)"),
    ]

    for ox, oy, r, color, alpha, label in obstacles:
        circ = Circle((ox, oy), r, color=color, alpha=alpha, zorder=1)
        ax.add_patch(circ)
        circ2 = Circle((ox, oy), r, fill=False, edgecolor=color, lw=1.8,
                        linestyle='--', zorder=2)
        ax.add_patch(circ2)
        ax.text(ox, oy, label, ha='center', va='center', fontsize=7.5,
                color='white', fontweight='bold', zorder=3,
                bbox=dict(fc=color, alpha=0.6, boxstyle='round,pad=0.15'))

    # Koneksi relay
    edges = [
        ("base_station", "node_basecamp_gate"),
        ("node_basecamp_gate", "node_valley_watch"),
        ("node_basecamp_gate", "node_forest_pass"),
        ("node_valley_watch", "node_ridge_mid"),
        ("node_forest_pass", "node_ridge_mid"),
        ("node_ridge_mid", "node_crater_edge"),
        ("node_ridge_mid", "node_upper_traverse"),
        ("node_crater_edge", "node_north_saddle"),
        ("node_upper_traverse", "node_north_saddle"),
        ("node_north_saddle", "node_summit_view"),
    ]
    for a, b in edges:
        x1, y1 = relay_nodes[a][:2]
        x2, y2 = relay_nodes[b][:2]
        ax.plot([x1, x2], [y1, y2], color=C_BLUE, lw=1.2, alpha=0.5,
                linestyle=':', zorder=3)

    # Plot nodes
    for name, (nx_, ny_, lbl) in relay_nodes.items():
        color = C_RED if name == "base_station" else C_BLUE
        marker = '*' if name == "base_station" else '^'
        ms = 16 if name == "base_station" else 11
        ax.plot(nx_, ny_, marker, color=color, ms=ms, zorder=5,
                markeredgecolor='white', markeredgewidth=0.8)
        va = 'bottom' if ny_ > 0 else 'top'
        ax.annotate(lbl, (nx_, ny_), xytext=(nx_ + 2, ny_ + 4),
                    fontsize=7.5, color=color, fontweight='bold',
                    bbox=dict(fc="white", ec=color, alpha=0.8,
                              boxstyle='round,pad=0.15'))

    # Pendaki contoh
    hiker_x, hiker_y = 15, 20
    ax.plot(hiker_x, hiker_y, 'o', color=C_ORANGE, ms=11, zorder=5,
            markeredgecolor='white')
    ax.annotate("Pendaki\n(Portable Node)", (hiker_x, hiker_y),
                xytext=(hiker_x + 10, hiker_y - 15),
                fontsize=8, color=C_ORANGE, fontweight='bold',
                arrowprops=dict(arrowstyle="->", color=C_ORANGE),
                bbox=dict(fc="white", ec=C_ORANGE, alpha=0.85, boxstyle='round,pad=0.2'))

    # Radius annotation
    circ_ann = Circle((-72, -50), 22, fill=False, edgecolor="#388e3c",
                       lw=0, zorder=6)
    ax.annotate("", xy=(-72, -50 + 22), xytext=(-72, -50),
                arrowprops=dict(arrowstyle="<->", color="#388e3c", lw=1.5))
    ax.text(-68, -50 + 11, "r = 22 wu", fontsize=8, color="#388e3c")

    # Legend
    legend_elements = [
        mpatches.Patch(color="#388e3c", alpha=0.5, label="Zona vegetasi lebat"),
        mpatches.Patch(color="#bf360c", alpha=0.5, label="Zona kawah"),
        mpatches.Patch(color="#5d4037", alpha=0.5, label="Zona batuan"),
        Line2D([0], [0], marker='^', color='w', markerfacecolor=C_BLUE,
               markersize=9, label="Relay Node"),
        Line2D([0], [0], marker='*', color='w', markerfacecolor=C_RED,
               markersize=12, label="Base Station"),
        Line2D([0], [0], marker='o', color='w', markerfacecolor=C_ORANGE,
               markersize=9, label="Portable Node (Pendaki)"),
    ]
    ax.legend(handles=legend_elements, loc='lower right', fontsize=8,
              framealpha=0.9, title="Legenda", title_fontsize=9)

    ax.set_xlim(-115, 105)
    ax.set_ylim(-105, 105)
    ax.set_xlabel("Koordinat X (satuan dunia Gazebo, 1 wu ≈ 35 m)", fontsize=9)
    ax.set_ylabel("Koordinat Y (satuan dunia Gazebo)", fontsize=9)
    ax.set_title("Gambar 10.4 — Peta 2D Penempatan Node dan Zona Obstacle Radio\n"
                 "Redaman vegetasi dihitung dengan model Weissberger (Persamaan 4.11)",
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.2, lw=0.6)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig2_obstacle_map.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig2_obstacle_map.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 3 — Alur GPS Sintetis (10.3.2)
# ══════════════════════════════════════════════════════════════════════════════
def fig3_gps_flowchart():
    fig, ax = plt.subplots(figsize=(10, 12), facecolor=BG)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 14)
    ax.axis('off')
    ax.set_title("Gambar 10.5 — Alur Pembentukan Koordinat GPS Sintetis\n"
                 "dengan Model Gangguan TTFF, DOP, Noise, dan Multipath (Persamaan 4.1–4.5)",
                 fontsize=11, fontweight='bold', pad=10)

    def box(ax, x, y, w, h, text, color, textcolor='white', fontsize=9, style='round,pad=0.3'):
        bbox = FancyBboxPatch((x - w/2, y - h/2), w, h,
                               boxstyle=style, fc=color, ec='white',
                               lw=1.5, zorder=3)
        ax.add_patch(bbox)
        ax.text(x, y, text, ha='center', va='center',
                fontsize=fontsize, color=textcolor,
                fontweight='bold', zorder=4, wrap=True,
                multialignment='center')

    def arrow(ax, x1, y1, x2, y2, label='', color='#455a64'):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=color,
                                    lw=1.8, mutation_scale=14),
                    zorder=2)
        if label:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(mx + 0.15, my, label, fontsize=8.5, color=color,
                    fontweight='bold')

    # Nodes
    box(ax, 5, 13.2, 4.5, 0.8, "Posisi sejati (x, y, z) dari Gazebo World", C_BLUE)
    box(ax, 5, 12.0, 4.5, 0.8, "Konversi → lat, lon\n(Proyeksi Equirectangular, Pers. 4.1)", C_TEAL)
    box(ax, 5, 10.8, 4.0, 0.8, "TTFF selesai?\n(Cold Start = 8 detik pertama)", C_PURPLE, style='round,pad=0.3')
    box(ax, 2.0, 9.5, 2.8, 0.8, "Status: NO_FIX\nGunakan last known\nlocation", C_GRAY)
    box(ax, 7.5, 9.5, 2.5, 0.8, "Status: FIX\nMulai terapkan\nmodel gangguan", C_GREEN)
    box(ax, 5, 8.3, 4.5, 0.8, "Tambahkan Gaussian Noise\nN(0, σ_GPS) ke komponen E, N, Alt", C_ORANGE)
    box(ax, 5, 7.0, 4.5, 0.9, "Hitung DOP Dinamis\n(Pers. 4.4: N_sat ≥ 4, HDOP ≤ 5)\nDOP ∈ [1.0, 4.5]", C_RED)
    box(ax, 5, 5.6, 4.5, 0.9, "noise_efektif = σ_GPS × DOP + δ_multipath\n(Pers. 4.5)\nKomponen Alt × 0.5", "#6a1b9a")
    box(ax, 2.0, 4.3, 2.6, 0.8, "Hitung δ_multipath\nfungsi jarak ke\nobstacle terdekat", C_GRAY)
    box(ax, 5, 3.1, 4.5, 0.8, "Validasi: ε_pos = √((Δx)²+(Δy)²)\nTarget: ≤10 m (LOS), ≤25 m (NLOS)", C_BLUE)
    box(ax, 5, 1.9, 4.5, 0.8, "Publikasi koordinat GPS berderau\n→ Node Jaringan LoRa & Data Logger", C_TEAL)

    # Arrows
    arrow(ax, 5, 12.8, 5, 12.4)
    arrow(ax, 5, 11.6, 5, 11.2)
    arrow(ax, 5, 10.4, 2.0, 9.9, "Belum", C_RED)
    arrow(ax, 5, 10.4, 7.5, 9.9, "Sudah", C_GREEN)
    arrow(ax, 7.5, 9.1, 5, 8.7)
    arrow(ax, 2.0, 9.1, 2.0, 4.7)
    arrow(ax, 5, 7.9, 5, 7.45)
    arrow(ax, 5, 6.55, 5, 6.05)
    arrow(ax, 2.0, 4.7, 3.5, 5.2)
    arrow(ax, 5, 5.15, 5, 3.5)
    arrow(ax, 5, 2.7, 5, 2.3)

    # Kotak keterangan DOP
    kotak_txt = ("Koefisien DOP tambahan:\n"
                 "• Vegetasi lebat: +1.8 × kedalaman\n"
                 "• Terrain curam: +1.2 × kedalaman\n"
                 "• Kawah:  +0.8 × kedalaman\n"
                 "• Batuan: +0.5 × kedalaman")
    ax.text(8.8, 6.9, kotak_txt, fontsize=7.5, va='center', ha='left',
            color=C_RED,
            bbox=dict(fc="white", ec=C_RED, alpha=0.85, boxstyle='round,pad=0.3'))
    ax.annotate("", xy=(7.3, 7.0), xytext=(8.75, 7.0),
                arrowprops=dict(arrowstyle="->", color=C_RED, lw=1.3))

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig3_gps_flowchart.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig3_gps_flowchart.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 4 — Kurva Baterai: Tegangan & TX Power vs SoC (10.3.2)
# ══════════════════════════════════════════════════════════════════════════════
def fig4_battery_curve():
    fig, ax1 = plt.subplots(figsize=(10, 6), facecolor=BG)
    ax2 = ax1.twinx()
    ax1.set_facecolor(BG)

    soc = np.linspace(0, 100, 500)

    # Tegangan: V = 4.20 - (1-SoC/100)*1.20
    voltage = 4.20 - (1 - soc / 100) * 1.20

    # TX Power: konstan 17 dBm saat SoC >= 30%, turun linear ke 11 dBm saat SoC=0
    tx_power = np.where(soc >= 30, 17.0,
               np.where(soc >= 0, 17 - (1 - soc / 30) * 6.0, 11.0))

    l1, = ax1.plot(soc, voltage, color=C_BLUE, lw=2.8, label="Tegangan Baterai (V)")
    l2, = ax2.plot(soc, tx_power, color=C_RED, lw=2.8, linestyle='--',
                   label="TX Power efektif (dBm)")

    # Zona
    ax1.axvspan(0, 20, alpha=0.12, color=C_RED, zorder=0)
    ax1.axvspan(20, 30, alpha=0.10, color=C_YELLOW, zorder=0)
    ax1.axvspan(30, 100, alpha=0.07, color=C_GREEN, zorder=0)

    ax1.axvline(30, color=C_RED, lw=1.5, linestyle=':', alpha=0.7)
    ax1.axvline(20, color=C_ORANGE, lw=1.5, linestyle=':', alpha=0.7)

    ax1.text(60, 3.05, "Zona Normal\nTX Power = 17 dBm", ha='center', fontsize=9,
             color=C_GREEN, fontweight='bold')
    ax1.text(25, 3.05, "Zona\nReduksi\nTX", ha='center', fontsize=8.5,
             color=C_YELLOW, fontweight='bold')
    ax1.text(10, 3.05, "Low-Power\nMode", ha='center', fontsize=8.5,
             color=C_RED, fontweight='bold')

    # Arrows annotation
    ax2.annotate("TX Power mulai\nturun (SoC < 30%)",
                 xy=(30, 17), xytext=(42, 14.5),
                 fontsize=9, color=C_RED, fontweight='bold',
                 arrowprops=dict(arrowstyle="->", color=C_RED),
                 bbox=dict(fc="white", ec=C_RED, boxstyle='round,pad=0.2'))
    ax1.annotate("Low-Power Mode aktif\n(SoC < 20%)\nKecepatan pendaki × 0.85",
                 xy=(20, 3.44), xytext=(32, 3.38),
                 fontsize=9, color=C_ORANGE, fontweight='bold',
                 arrowprops=dict(arrowstyle="->", color=C_ORANGE),
                 bbox=dict(fc="white", ec=C_ORANGE, boxstyle='round,pad=0.2'))

    ax1.set_xlabel("State of Charge — SoC (%)", fontsize=11)
    ax1.set_ylabel("Tegangan Baterai (V)", fontsize=11, color=C_BLUE)
    ax2.set_ylabel("TX Power Efektif (dBm)", fontsize=11, color=C_RED)
    ax1.tick_params(axis='y', labelcolor=C_BLUE)
    ax2.tick_params(axis='y', labelcolor=C_RED)

    ax1.set_xlim(0, 100)
    ax1.set_ylim(2.85, 4.35)
    ax2.set_ylim(9, 19)

    lines = [l1, l2]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left', fontsize=9, framealpha=0.9)

    ax1.set_title("Gambar 10.6 — Model Baterai Samsung 30Q: Tegangan dan TX Power vs SoC\n"
                  "Penurunan TX Power berdampak langsung pada Link Margin (Persamaan 4.8, 4.21–4.22)",
                  fontsize=11, fontweight='bold')
    ax1.grid(True, alpha=0.25, lw=0.7)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig4_battery_curve.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig4_battery_curve.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 5 — Waterfall Link Budget (10.3.3)
# ══════════════════════════════════════════════════════════════════════════════
def fig5_link_budget_waterfall():
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor=BG)
    fig.suptitle("Gambar 10.7 — Waterfall Diagram Komponen Link Budget LoRa\n"
                 "Persamaan 4.8: RSSI = P_tx + G_tx + G_rx − PL_total",
                 fontsize=12, fontweight='bold')

    scenarios = [
        {
            "title": "(a) Kondisi LOS — ridge_route, SF9, Jarak 800 m",
            "components": [
                ("TX Power\n(E220, +17 dBm)",    17,   C_GREEN,  True),
                ("Gain Antena TX",                 2,   C_TEAL,   True),
                ("Gain Antena RX",                 2,   C_TEAL,   True),
                ("FSPL (800 m, 923 MHz)",        -87,   C_GRAY,   False),
                ("Obstacle Loss\n(vegetasi tipis)", -5, C_GREEN,  False),
                ("Terrain Shadow",                 0,   C_BLUE,   False),
                ("Diffraction Loss",               0,   C_BLUE,   False),
                ("Weather Loss\n(cerah)",          0,   C_GREEN,  False),
                ("Fading Loss\n(Rician K=10 dB)", -3,   C_ORANGE, False),
            ],
            "sensitivity": -129,  # SF9
        },
        {
            "title": "(b) Kondisi NLOS — valley_route, SF11, Jarak 1200 m",
            "components": [
                ("TX Power\n(E220, +17 dBm)",    17,   C_GREEN,  True),
                ("Gain Antena TX",                 2,   C_TEAL,   True),
                ("Gain Antena RX",                 2,   C_TEAL,   True),
                ("FSPL (1200 m, 923 MHz)",       -92,   C_GRAY,   False),
                ("Obstacle Loss\n(hutan lebat)", -14,   C_RED,    False),
                ("Terrain Shadow\nLoss",          -8,   C_RED,    False),
                ("Diffraction Loss",              -6,   C_ORANGE, False),
                ("Weather Loss\n(hujan ringan)",  -3,   C_ORANGE, False),
                ("Fading Loss\n(Rayleigh NLOS)", -8,   C_RED,    False),
            ],
            "sensitivity": -134.5,  # SF11
        },
    ]

    for ax, sc in zip(axes, scenarios):
        ax.set_facecolor(BG)
        ax.set_title(sc["title"], fontsize=10, fontweight='bold', pad=8)

        running = 0
        yticks, ylabels = [], []
        bars = []

        for i, (label, val, color, is_gain) in enumerate(sc["components"]):
            bottom = running if is_gain else running + val
            bar_val = abs(val)
            b = ax.barh(i, bar_val, left=min(running, running + val),
                        color=color, alpha=0.82, height=0.65,
                        edgecolor='white', linewidth=1.2)
            bars.append(b)
            running += val
            sign = "+" if val > 0 else ""
            ax.text(min(running, running - val) + bar_val / 2 if bar_val > 3 else running + 1.5,
                    i, f"{sign}{val} dB", ha='center' if bar_val > 6 else 'left',
                    va='center', fontsize=8.5, color='white' if bar_val > 6 else color,
                    fontweight='bold')
            yticks.append(i)
            ylabels.append(label)

        # RSSI result line
        ax.axvline(running, color=C_BLUE, lw=2.5, linestyle='-',
                   label=f"RSSI = {running:.0f} dBm", zorder=5)
        ax.axvline(sc["sensitivity"], color=C_RED, lw=2.0, linestyle='--',
                   label=f"Sensitivitas = {sc['sensitivity']} dBm", zorder=5)

        margin = running - sc["sensitivity"]
        color_margin = C_GREEN if margin >= 10 else C_YELLOW if margin >= 0 else C_RED
        ax.fill_betweenx([-0.5, len(sc["components"]) - 0.5],
                          sc["sensitivity"], running,
                          alpha=0.12, color=color_margin)
        ax.text((running + sc["sensitivity"]) / 2, -1.2,
                f"Link Margin = {margin:.0f} dB", ha='center', fontsize=10,
                color=color_margin, fontweight='bold',
                bbox=dict(fc="white", ec=color_margin, boxstyle='round,pad=0.25'))

        ax.set_yticks(yticks)
        ax.set_yticklabels(ylabels, fontsize=8.5)
        ax.set_xlabel("Level Daya (dBm)", fontsize=9)
        ax.set_xlim(sc["sensitivity"] - 10, 25)
        ax.legend(loc='lower right', fontsize=8.5, framealpha=0.9)
        ax.grid(True, axis='x', alpha=0.25, lw=0.7)
        ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig5_link_budget_waterfall.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig5_link_budget_waterfall.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 6 — Graf Konektivitas + Dijkstra (10.3.3)
# ══════════════════════════════════════════════════════════════════════════════
def fig6_routing_graph():
    fig, axes = plt.subplots(1, 2, figsize=(15, 8), facecolor=BG)
    fig.suptitle("Gambar 10.8 — Graf Konektivitas LoRa dan Jalur Routing Dijkstra\n"
                 "Bobot tepi = 100 + 0.1 × jarak (meter), Pers. 4.18",
                 fontsize=12, fontweight='bold')

    G = nx.DiGraph()

    nodes = {
        "Pendaki\n(hiker_3)":      (0.0, 4.5),
        "node_valley\n_watch":     (1.5, 2.5),
        "node_forest\n_pass":      (1.5, 6.5),
        "node_ridge\n_mid":        (3.5, 4.5),
        "node_crater\n_edge":      (5.5, 6.0),
        "node_upper\n_traverse":   (5.5, 3.0),
        "node_north\n_saddle":     (7.0, 4.5),
        "node_summit\n_view":      (8.5, 5.5),
        "node_basecamp\n_gate":    (3.5, 1.0),
        "Base\nStation":           (5.5, 0.5),
    }

    edges_all = [
        ("Pendaki\n(hiker_3)", "node_valley\n_watch",  "+38 dB", True),
        ("Pendaki\n(hiker_3)", "node_forest\n_pass",   "+22 dB", True),
        ("Pendaki\n(hiker_3)", "node_ridge\n_mid",     "+8 dB",  True),
        ("node_valley\n_watch", "node_ridge\n_mid",    "+31 dB", True),
        ("node_forest\n_pass", "node_ridge\n_mid",     "+27 dB", True),
        ("node_ridge\n_mid", "node_crater\n_edge",     "+15 dB", True),
        ("node_ridge\n_mid", "node_upper\n_traverse",  "+19 dB", True),
        ("node_ridge\n_mid", "node_basecamp\n_gate",   "+24 dB", True),
        ("node_crater\n_edge", "node_north\n_saddle",  "+12 dB", True),
        ("node_upper\n_traverse", "node_north\n_saddle","+17 dB",True),
        ("node_north\n_saddle", "node_summit\n_view",  "+9 dB",  True),
        ("node_basecamp\n_gate", "Base\nStation",      "+41 dB", True),
        ("node_upper\n_traverse", "node_basecamp\n_gate","+4 dB",False),
        ("Pendaki\n(hiker_3)", "Base\nStation",        "−12 dB", False),
    ]

    optimal_path = [
        "Pendaki\n(hiker_3)",
        "node_ridge\n_mid",
        "node_basecamp\n_gate",
        "Base\nStation"
    ]

    for ax_idx, ax in enumerate(axes):
        ax.set_facecolor(BG)
        ax.set_xlim(-0.5, 9.5)
        ax.set_ylim(-0.5, 8.0)
        ax.axis('off')

        # Draw edges
        for u, v, margin, ok in edges_all:
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            on_path = (u in optimal_path and v in optimal_path and
                       optimal_path.index(v) == optimal_path.index(u) + 1)

            if on_path and ax_idx == 1:
                color, lw, ls, alpha = C_GREEN, 3.5, '-', 1.0
            elif ok:
                color, lw, ls, alpha = C_BLUE, 1.5, '-', 0.5
            else:
                color, lw, ls, alpha = C_RED, 1.3, '--', 0.45

            ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                        arrowprops=dict(arrowstyle="-|>", color=color,
                                        lw=lw, alpha=alpha,
                                        connectionstyle="arc3,rad=0.1"),
                        zorder=2)
            mx, my = (x1 + x2) / 2 + 0.1, (y1 + y2) / 2 + 0.15
            ax.text(mx, my, margin, fontsize=7.5, color=color,
                    ha='center', alpha=min(1.0, alpha + 0.2), fontweight='bold')

        # Draw nodes
        for name, (nx_, ny_) in nodes.items():
            is_bs = "Base" in name
            is_hiker = "Pendaki" in name
            color = C_RED if is_bs else (C_ORANGE if is_hiker else C_BLUE)
            marker = '*' if is_bs else ('o' if is_hiker else 's')
            ms = 18 if is_bs else (14 if is_hiker else 12)
            on_path = name in optimal_path

            if on_path and ax_idx == 1:
                ax.plot(nx_, ny_, marker, color=C_GREEN, ms=ms + 3,
                        zorder=5, markeredgecolor='white', markeredgewidth=1.5)
            ax.plot(nx_, ny_, marker, color=color, ms=ms,
                    zorder=6 if not (on_path and ax_idx == 1) else 5,
                    markeredgecolor='white', markeredgewidth=1.2,
                    alpha=1.0 if on_path or ax_idx == 0 else 0.65)
            ax.text(nx_, ny_ - 0.4, name, ha='center', va='top',
                    fontsize=7.5, color=color, fontweight='bold',
                    bbox=dict(fc="white", ec=color, alpha=0.85,
                              boxstyle='round,pad=0.15'))

        subtitle = ("(a) Graf Konektivitas Penuh\n"
                    "(tepi terbentuk jika margin ≥ 0 dB)" if ax_idx == 0 else
                    "(b) Jalur Optimal Dipilih Dijkstra\n"
                    "(hijau = rute aktif, bobot minimum)")
        ax.set_title(subtitle, fontsize=10, fontweight='bold', pad=6)

        # Legend
        leg = [
            Line2D([0], [0], color=C_BLUE, lw=2, label="Link margin ≥ 0 dB (layak)"),
            Line2D([0], [0], color=C_RED, lw=1.5, ls='--', label="Link margin < 0 dB (tidak layak)"),
        ]
        if ax_idx == 1:
            leg.append(Line2D([0], [0], color=C_GREEN, lw=3,
                               label="Jalur terpilih Dijkstra"))
        ax.legend(handles=leg, loc='upper right', fontsize=8, framealpha=0.9)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig6_routing_graph.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig6_routing_graph.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 7 — Trade-off SF (10.3.3)
# ══════════════════════════════════════════════════════════════════════════════
def fig7_sf_tradeoff():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9), facecolor=BG,
                                    sharex=True)
    fig.suptitle("Gambar 10.9 — Trade-off Spreading Factor LoRa\n"
                 "Sensitivitas vs Time on Air vs Duty Cycle (Persamaan 4.10)",
                 fontsize=12, fontweight='bold')

    sf = np.array([7, 8, 9, 10, 11, 12])
    sensitivity = np.array([-123, -126, -129, -132, -134.5, -136])
    data_rate = np.array([5469, 3125, 1758, 879, 477, 250])
    toa_ms = np.array([15.5, 26.7, 49.2, 102.2, 185.3, 370.6])

    # ── ax1: Sensitivitas + Data Rate ──
    ax1.set_facecolor(BG)
    c1 = C_BLUE
    c2 = C_GREEN
    l1 = ax1.plot(sf, sensitivity, 'o-', color=c1, lw=2.5, ms=8,
                  label="Sensitivitas penerima (dBm)", markeredgecolor='white')
    ax1b = ax1.twinx()
    l2 = ax1b.plot(sf, data_rate, 's--', color=c2, lw=2.2, ms=8,
                   label="Data Rate (bps)", markeredgecolor='white')

    for i, s in enumerate(sf):
        ax1.annotate(f"{sensitivity[i]} dBm", (s, sensitivity[i]),
                     xytext=(0, -14), textcoords='offset points',
                     ha='center', fontsize=8, color=c1)
        ax1b.annotate(f"{data_rate[i]:,}", (s, data_rate[i]),
                      xytext=(0, 8), textcoords='offset points',
                      ha='center', fontsize=8, color=c2)

    ax1.set_ylabel("Sensitivitas Penerima (dBm)", color=c1, fontsize=10)
    ax1b.set_ylabel("Data Rate (bps)", color=c2, fontsize=10)
    ax1.tick_params(axis='y', labelcolor=c1)
    ax1b.tick_params(axis='y', labelcolor=c2)
    ax1.set_ylim(-140, -118)

    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='lower left', fontsize=9, framealpha=0.9)
    ax1.set_title("(a) Sensitivitas meningkat, Data Rate menurun seiring SF naik",
                  fontsize=10, pad=5)
    ax1.grid(True, alpha=0.25)

    # ── ax2: ToA + Duty Cycle risk ──
    ax2.set_facecolor(BG)
    interval_s = 30
    n_hikers = np.array([5, 10, 20])
    colors_hiker = [C_GREEN, C_ORANGE, C_RED]

    for n, col in zip(n_hikers, colors_hiker):
        duty_pct = toa_ms * n / (interval_s * 1000) * 100
        ax2.plot(sf, duty_pct, 'o-', color=col, lw=2.2, ms=7,
                 label=f"{n} pendaki aktif", markeredgecolor='white')

    ax2.axhline(1.0, color=C_RED, lw=2.0, linestyle='--',
                label="Batas duty cycle 1% (regulasi SDPPI)")
    ax2.fill_between(sf, 1.0, ax2.get_ylim()[1] if ax2.get_ylim()[1] > 1 else 3,
                     alpha=0.08, color=C_RED)

    ax2b = ax2.twinx()
    ax2b.bar(sf, toa_ms, color=C_BLUE, alpha=0.25, width=0.4,
             label="Time on Air (ms)")
    ax2b.set_ylabel("Time on Air (ms)", color=C_BLUE, fontsize=10)
    ax2b.tick_params(axis='y', labelcolor=C_BLUE)

    for i, s in enumerate(sf):
        ax2.annotate(f"{toa_ms[i]:.0f} ms", (s, 0),
                     xytext=(0, 3), textcoords='offset points',
                     ha='center', fontsize=7.5, color=C_BLUE, alpha=0.8)

    ax2.set_xlabel("Spreading Factor (SF)", fontsize=11)
    ax2.set_ylabel("Penggunaan Duty Cycle (%)", fontsize=10)
    ax2.set_xticks(sf)
    ax2.set_xticklabels([f"SF{s}" for s in sf])
    ax2.legend(loc='upper left', fontsize=8.5, framealpha=0.9)
    ax2.set_title("(b) Risiko Pelanggaran Duty Cycle Meningkat pada SF Tinggi dengan Banyak Pendaki",
                  fontsize=10, pad=5)
    ax2.grid(True, alpha=0.25)
    ax2.set_ylim(0, ax2.get_ylim()[1] * 1.15 if ax2.get_ylim()[1] > 0.5 else 3)

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig7_sf_tradeoff.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig7_sf_tradeoff.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 8 — Kurva Sigmoid PER (10.3.4)
# ══════════════════════════════════════════════════════════════════════════════
def fig8_per_sigmoid():
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), facecolor=BG)
    fig.suptitle("Gambar 10.10 — Model Probabilitas Packet Error Rate (PER)\n"
                 "Persamaan 4.16: PER = 1 / (1 + exp(0.8 × (RSSI − S_rx)))",
                 fontsize=12, fontweight='bold')

    margin = np.linspace(-15, 25, 600)
    k = 0.8
    per = 1 / (1 + np.exp(k * (margin - 2)))

    # ── ax[0]: kurva utama ──
    ax = axes[0]
    ax.set_facecolor(BG)
    ax.plot(margin, per, color=C_BLUE, lw=3.0, label="PER (Pers. 4.16)")
    ax.fill_between(margin, per, 1, where=(margin < 0), alpha=0.15, color=C_RED)
    ax.fill_between(margin, 0, per, where=(margin > 5), alpha=0.12, color=C_GREEN)

    # Annotasi titik kunci
    points = [
        (-10, "PER ≈ 99%\n(link hampir pasti gagal)", C_RED, 'left'),
        (2,   "PER = 50%\n(titik transisi)", C_ORANGE, 'left'),
        (10,  "PER ≈ 3%\n(link sangat andal)", C_GREEN, 'right'),
    ]
    for m_pt, lbl, col, ha in points:
        per_pt = 1 / (1 + np.exp(k * (m_pt - 2)))
        ax.plot(m_pt, per_pt, 'o', color=col, ms=10, zorder=5,
                markeredgecolor='white', markeredgewidth=1.2)
        offset_x = 2 if ha == 'left' else -2
        ax.annotate(lbl, (m_pt, per_pt), xytext=(m_pt + offset_x, per_pt + 0.12),
                    fontsize=8.5, color=col, ha=ha, fontweight='bold',
                    bbox=dict(fc="white", ec=col, alpha=0.85, boxstyle='round,pad=0.2'),
                    arrowprops=dict(arrowstyle="->", color=col, lw=1.3))

    ax.axvline(0, color=C_GRAY, lw=1.3, ls=':', alpha=0.7,
               label="Margin = 0 dB (ambang layak)")
    ax.axvline(10, color=C_GREEN, lw=1.3, ls=':', alpha=0.7,
               label="Margin = 10 dB (target operasional)")
    ax.axhline(0.1, color=C_ORANGE, lw=1.0, ls='--', alpha=0.6,
               label="PER = 10% (batas wajar)")

    ax.set_xlabel("Link Margin (dB)", fontsize=11)
    ax.set_ylabel("Packet Error Rate (PER)", fontsize=11)
    ax.set_ylim(-0.05, 1.10)
    ax.set_xlim(-15, 25)
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    ax.set_title("(a) Kurva PER vs Link Margin", fontsize=10, pad=5)
    ax.grid(True, alpha=0.25)
    ax.yaxis.set_major_formatter(mticker.PercentFormatter(xmax=1))

    # ── ax[1]: PDR multi-hop kumulatif ──
    ax2 = axes[1]
    ax2.set_facecolor(BG)

    per_values = np.linspace(0.0, 0.8, 300)
    pdr_1hop = (1 - per_values) * 100
    pdr_2hop = (1 - per_values) ** 2 * 100
    pdr_3hop = (1 - per_values) ** 3 * 100

    ax2.plot(per_values * 100, pdr_1hop, color=C_GREEN, lw=2.5,
             label="1 hop (direct)")
    ax2.plot(per_values * 100, pdr_2hop, color=C_ORANGE, lw=2.5,
             label="2 hop (via 1 relay)")
    ax2.plot(per_values * 100, pdr_3hop, color=C_RED, lw=2.5,
             label="3 hop (via 2 relay)")
    ax2.axhline(90, color=C_BLUE, lw=1.8, ls='--', alpha=0.7,
                label="Target PDR ≥ 90% (LOS)")
    ax2.axhline(80, color=C_PURPLE, lw=1.5, ls='--', alpha=0.6,
                label="Target PDR ≥ 80% (NLOS)")

    ax2.fill_between(per_values * 100, 90, 100, alpha=0.08, color=C_GREEN)

    ax2.set_xlabel("PER per Tautan (%)", fontsize=11)
    ax2.set_ylabel("PDR End-to-End (%)", fontsize=11)
    ax2.set_xlim(0, 80)
    ax2.set_ylim(0, 105)
    ax2.legend(fontsize=8.5, loc='upper right', framealpha=0.9)
    ax2.set_title("(b) Akumulasi PDR End-to-End per Jumlah Hop\n(Persamaan 4.18)",
                  fontsize=10, pad=5)
    ax2.grid(True, alpha=0.25)
    ax2.yaxis.set_major_formatter(mticker.PercentFormatter())

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig8_per_pdr.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig8_per_pdr.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 9 — Pohon Keputusan Drop Reason (10.3.4)
# ══════════════════════════════════════════════════════════════════════════════
def fig9_drop_reason_tree():
    fig, ax = plt.subplots(figsize=(14, 9), facecolor=BG)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 10)
    ax.axis('off')
    ax.set_facecolor(BG)
    ax.set_title("Gambar 10.11 — Pohon Keputusan Mekanisme Kegagalan Paket\n"
                 "Enam drop_reason yang diimplementasikan dalam Simulasi siLacak",
                 fontsize=12, fontweight='bold', pad=10)

    def rbox(ax, x, y, w, h, text, fc, ec='white', fs=8.5, tc='white', style='round,pad=0.25'):
        p = FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle=style,
                            fc=fc, ec=ec, lw=1.8, zorder=3)
        ax.add_patch(p)
        ax.text(x, y, text, ha='center', va='center', fontsize=fs,
                color=tc, fontweight='bold', zorder=4, multialignment='center')

    def arr(ax, x1, y1, x2, y2, lbl='', lbl_side='right', col='#455a64'):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8,
                                    mutation_scale=12), zorder=2)
        if lbl:
            mx = (x1 + x2) / 2 + (0.15 if lbl_side == 'right' else -0.15)
            my = (y1 + y2) / 2
            ax.text(mx, my, lbl, fontsize=8.5, color=col, fontweight='bold',
                    ha='left' if lbl_side == 'right' else 'right')

    # Mulai
    rbox(ax, 7, 9.3, 3.0, 0.65, "Paket siap dikirim\n(Portable Node)", C_BLUE, fs=9)

    # 1. Route check
    rbox(ax, 7, 8.3, 3.2, 0.65, "❶ Ada route valid\nke Base Station?", C_PURPLE, fs=8.5)
    arr(ax, 7, 8.98, 7, 8.63)

    # No route → drop
    rbox(ax, 11.5, 8.3, 2.2, 0.6, "drop_reason:\nno_route", C_RED, fs=8.5)
    arr(ax, 8.6, 8.3, 10.4, 8.3, "Tidak", col=C_RED)

    # 2. Duty cycle
    rbox(ax, 7, 7.2, 3.2, 0.65, "❷ Duty cycle\nmasih < 1%?", C_PURPLE, fs=8.5)
    arr(ax, 7, 7.97, 7, 7.52, "Ya", col=C_GREEN)
    rbox(ax, 11.5, 7.2, 2.2, 0.6, "drop_reason:\nduty_cycle", C_RED, fs=8.5)
    arr(ax, 8.6, 7.2, 10.4, 7.2, "Tidak", col=C_RED)

    # 3. PER sigmoid
    rbox(ax, 7, 6.1, 3.4, 0.75, "❸ Random < PER?\n(Sigmoid, Pers. 4.16)\nPER = f(RSSI − S_rx)", C_PURPLE, fs=8)
    arr(ax, 7, 6.87, 7, 6.47, "Ya", col=C_GREEN)
    rbox(ax, 11.5, 6.1, 2.2, 0.6, "drop_reason:\nper_model", C_RED, fs=8.5)
    arr(ax, 8.7, 6.1, 10.4, 6.1, "Tidak (gagal)", col=C_RED)

    # 4. Collision
    rbox(ax, 7, 5.0, 3.2, 0.65, "❹ Collision\ndengan node lain?", C_PURPLE, fs=8.5)
    arr(ax, 7, 5.73, 7, 5.32, "Ya", col=C_GREEN)
    rbox(ax, 11.5, 5.0, 2.2, 0.6, "drop_reason:\ncollision", C_RED, fs=8.5)
    arr(ax, 8.6, 5.0, 10.4, 5.0, "Ya (collision)", col=C_RED)

    # 5. Weather
    rbox(ax, 7, 3.9, 3.2, 0.65, "❺ Weather drop\naktif?", C_PURPLE, fs=8.5)
    arr(ax, 7, 4.67, 7, 4.22, "Tidak", col=C_GREEN)
    rbox(ax, 11.5, 3.9, 2.2, 0.6, "drop_reason:\nweather", C_RED, fs=8.5)
    arr(ax, 8.6, 3.9, 10.4, 3.9, "Ya", col=C_RED)

    # 6. Protocol
    rbox(ax, 7, 2.8, 3.2, 0.65, "❻ CRC error /\nTTL habis?", C_PURPLE, fs=8.5)
    arr(ax, 7, 3.57, 7, 3.12, "Tidak", col=C_GREEN)
    rbox(ax, 11.5, 2.8, 2.2, 0.6, "drop_reason:\nprotocol", C_RED, fs=8.5)
    arr(ax, 8.6, 2.8, 10.4, 2.8, "Ya", col=C_RED)

    # Delivered
    rbox(ax, 7, 1.6, 3.5, 0.75,
         "✓  delivered = True\nPaket diterima Base Station\nPDR_e2e (Pers. 4.17–4.18)",
         C_GREEN, fs=9)
    arr(ax, 7, 2.47, 7, 1.97, "Tidak", col=C_GREEN)

    # Keterangan kiri
    note = ("Catatan:\n"
            "• Semua evaluasi berjalan\n"
            "  secara sekuensial per paket\n"
            "• Fading (Rician/Rayleigh)\n"
            "  sudah termasuk dalam RSSI\n"
            "  yang dimasukkan ke Pers. 4.16\n"
            "• SOS mendapat k=3 retry\n"
            "  sebelum dinyatakan gagal\n"
            "  (Pers. 4.20)")
    ax.text(0.4, 5.5, note, fontsize=8.5, va='center', ha='left',
            color=C_GRAY,
            bbox=dict(fc="white", ec=C_GRAY, alpha=0.9, boxstyle='round,pad=0.4'))

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig9_drop_reason_tree.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig9_drop_reason_tree.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 10 — Kurva SOS Retry (10.3.4)
# ══════════════════════════════════════════════════════════════════════════════
def fig10_sos_retry():
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), facecolor=BG)
    fig.suptitle("Gambar 10.12 — Probabilitas Keberhasilan SOS vs Jumlah Retry\n"
                 "Persamaan 4.20: P_SOS = 1 − PER^k",
                 fontsize=12, fontweight='bold')

    # ── ax[0]: P_SOS vs k untuk berbagai PER ──
    ax = axes[0]
    ax.set_facecolor(BG)
    per_vals = [0.2, 0.3, 0.4, 0.5, 0.6]
    colors = [C_GREEN, C_TEAL, C_BLUE, C_ORANGE, C_RED]
    k_range = np.arange(1, 6)

    for per, col in zip(per_vals, colors):
        p_sos = (1 - per ** k_range) * 100
        ax.plot(k_range, p_sos, 'o-', color=col, lw=2.5, ms=9,
                markeredgecolor='white', markeredgewidth=1.2,
                label=f"PER per kirim = {int(per*100)}%")
        for k, p in zip(k_range, p_sos):
            if k <= 3:
                ax.annotate(f"{p:.1f}%", (k, p),
                            xytext=(0, 8), textcoords='offset points',
                            ha='center', fontsize=7.5, color=col)

    ax.axhline(90, color=C_RED, lw=2.0, ls='--', label="Target ≥ 90% (Spesifikasi)")
    ax.fill_between(k_range, 90, 100, alpha=0.08, color=C_GREEN)
    ax.set_xlabel("Jumlah Retry (k)", fontsize=11)
    ax.set_ylabel("Probabilitas Keberhasilan SOS (%)", fontsize=11)
    ax.set_xticks(k_range)
    ax.set_xticklabels([f"k={k}" for k in k_range])
    ax.set_ylim(20, 105)
    ax.set_xlim(0.5, 5.5)
    ax.legend(fontsize=8.5, loc='lower right', framealpha=0.9)
    ax.set_title("(a) P_SOS vs Jumlah Retry untuk Berbagai PER Awal", fontsize=10)
    ax.grid(True, alpha=0.25)

    # Contoh spesifik k=3, PER=0.4
    p_example = (1 - 0.4 ** 3) * 100
    ax.annotate(f"PER=40%, k=3\nP_SOS = {p_example:.1f}% ✓",
                xy=(3, p_example), xytext=(3.8, 88),
                fontsize=9, color=C_BLUE, fontweight='bold',
                arrowprops=dict(arrowstyle="->", color=C_BLUE),
                bbox=dict(fc="white", ec=C_BLUE, boxstyle='round,pad=0.25'))

    # ── ax[1]: Visualisasi probabilitas kumulatif ──
    ax2 = axes[1]
    ax2.set_facecolor(BG)
    per_range = np.linspace(0, 0.9, 400)
    for k_val, col, ls in [(1, C_RED, '-'), (2, C_ORANGE, '--'), (3, C_GREEN, '-.')]:
        ax2.plot(per_range * 100, (1 - per_range ** k_val) * 100,
                 color=col, lw=2.5, ls=ls, label=f"k={k_val} percobaan")

    ax2.axhline(90, color=C_BLUE, lw=1.8, ls=':', alpha=0.7,
                label="Target 90%")
    ax2.fill_between(per_range * 100, 90, 100, alpha=0.07, color=C_GREEN)

    # Batas PER max untuk memenuhi target per k
    for k_val, col in [(1, C_RED), (2, C_ORANGE), (3, C_GREEN)]:
        # 1 - PER^k = 0.9 → PER^k = 0.1 → PER = 0.1^(1/k)
        per_max = 0.1 ** (1 / k_val) * 100
        ax2.axvline(per_max, color=col, lw=1.3, ls=':', alpha=0.6)
        ax2.text(per_max + 0.5, 5 + k_val * 8,
                 f"k={k_val}: PER maks\n= {per_max:.0f}%",
                 fontsize=8, color=col, fontweight='bold')

    ax2.set_xlabel("PER per Pengiriman Tunggal (%)", fontsize=11)
    ax2.set_ylabel("Probabilitas Keberhasilan SOS (%)", fontsize=11)
    ax2.set_xlim(0, 90)
    ax2.set_ylim(0, 105)
    ax2.legend(fontsize=9, loc='upper right', framealpha=0.9)
    ax2.set_title("(b) Batas PER Maksimum agar P_SOS ≥ 90%\nper Jumlah Retry", fontsize=10)
    ax2.grid(True, alpha=0.25)
    ax2.yaxis.set_major_formatter(mticker.PercentFormatter())

    plt.tight_layout()
    plt.savefig(f"{OUT}/fig10_sos_retry.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig10_sos_retry.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR 11 — Ringkasan Metrik Evaluasi (10.3.4)
# ══════════════════════════════════════════════════════════════════════════════
def fig11_metrics_summary():
    fig = plt.figure(figsize=(14, 8), facecolor=BG)
    fig.suptitle("Gambar 10.13 — Ringkasan Metrik Evaluasi Simulasi siLacak\n"
                 "Setiap metrik dapat ditelusuri ke persamaan Bab 4",
                 fontsize=12, fontweight='bold')

    gs = GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.4)

    # ── GPS Error distribution ──
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_facecolor(BG)
    np.random.seed(42)
    errors_los = np.abs(np.random.normal(5.2, 2.1, 600))
    errors_nlos = np.abs(np.random.normal(14.5, 5.8, 400))
    ax1.hist(errors_los, bins=25, color=C_GREEN, alpha=0.65, density=True,
             label="LOS (target ≤10 m)")
    ax1.hist(errors_nlos, bins=25, color=C_ORANGE, alpha=0.55, density=True,
             label="NLOS (target ≤25 m)")
    ax1.axvline(10, color=C_GREEN, lw=1.8, ls='--')
    ax1.axvline(25, color=C_ORANGE, lw=1.8, ls='--')
    ax1.set_xlabel("GPS Error (m)", fontsize=9)
    ax1.set_ylabel("Densitas", fontsize=9)
    ax1.set_title("GPS Error (Pers. 4.5)", fontsize=10, fontweight='bold')
    ax1.legend(fontsize=7.5)
    ax1.grid(True, alpha=0.2)

    # ── RSSI vs Jarak ──
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_facecolor(BG)
    d = np.linspace(50, 2000, 300)
    fspl = 20 * np.log10(d / 1000) + 20 * np.log10(923) + 32.44
    rssi_los = 17 + 2 + 2 - fspl + np.random.normal(0, 2, 300)
    rssi_nlos = rssi_los - np.random.exponential(8, 300)
    ax2.scatter(d, rssi_los, s=4, color=C_BLUE, alpha=0.4, label="LOS")
    ax2.scatter(d, rssi_nlos, s=4, color=C_ORANGE, alpha=0.35, label="NLOS")
    ax2.plot(d, 17 + 2 + 2 - fspl, color=C_RED, lw=2.2, label="FSPL teoritis")
    ax2.axhline(-129, color=C_PURPLE, lw=1.5, ls='--', alpha=0.7,
                label="Sensitivitas SF9")
    ax2.set_xlabel("Jarak (m)", fontsize=9)
    ax2.set_ylabel("RSSI (dBm)", fontsize=9)
    ax2.set_title("RSSI vs Jarak (Pers. 4.8)", fontsize=10, fontweight='bold')
    ax2.legend(fontsize=7, loc='upper right')
    ax2.grid(True, alpha=0.2)

    # ── PDR per SF ──
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.set_facecolor(BG)
    sf_labels = ['SF7', 'SF8', 'SF9', 'SF10', 'SF11', 'SF12']
    pdr_los = [96, 94, 92, 91, 90, 88]
    pdr_nlos = [78, 82, 85, 87, 88, 89]
    x_pos = np.arange(len(sf_labels))
    w = 0.35
    ax3.bar(x_pos - w/2, pdr_los, w, color=C_GREEN, alpha=0.8, label="LOS")
    ax3.bar(x_pos + w/2, pdr_nlos, w, color=C_ORANGE, alpha=0.8, label="NLOS")
    ax3.axhline(90, color=C_RED, lw=1.8, ls='--', label="Target 90%")
    ax3.axhline(80, color=C_ORANGE, lw=1.5, ls=':', label="Target 80%")
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(sf_labels, fontsize=9)
    ax3.set_ylabel("PDR (%)", fontsize=9)
    ax3.set_ylim(60, 102)
    ax3.set_title("PDR per SF (Pers. 4.17)", fontsize=10, fontweight='bold')
    ax3.legend(fontsize=7.5, loc='lower right')
    ax3.grid(True, axis='y', alpha=0.2)

    # ── Drop reason distribution ──
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.set_facecolor(BG)
    reasons = ['no_route', 'per_model', 'duty_cycle', 'collision', 'weather', 'protocol']
    counts = [12, 35, 18, 22, 8, 5]
    colors_r = [C_PURPLE, C_RED, C_ORANGE, C_YELLOW, C_BLUE, C_GRAY]
    bars = ax4.barh(reasons, counts, color=colors_r, alpha=0.82, edgecolor='white', lw=1.2)
    for bar, cnt in zip(bars, counts):
        ax4.text(cnt + 0.5, bar.get_y() + bar.get_height()/2,
                 f'{cnt}', va='center', fontsize=9, color=C_GRAY, fontweight='bold')
    ax4.set_xlabel("Jumlah kejadian (contoh skenario)", fontsize=9)
    ax4.set_title("Distribusi Drop Reason", fontsize=10, fontweight='bold')
    ax4.grid(True, axis='x', alpha=0.2)

    # ── Latensi distribusi ──
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.set_facecolor(BG)
    lat_1hop = np.abs(np.random.normal(8.5, 3.2, 400))
    lat_2hop = np.abs(np.random.normal(18.2, 5.1, 300))
    ax5.hist(lat_1hop, bins=20, color=C_GREEN, alpha=0.7, density=True,
             label="1 hop (target ≤15 s)")
    ax5.hist(lat_2hop, bins=20, color=C_ORANGE, alpha=0.65, density=True,
             label="2 hop (target ≤30 s)")
    ax5.axvline(15, color=C_GREEN, lw=1.8, ls='--')
    ax5.axvline(30, color=C_ORANGE, lw=1.8, ls='--')
    ax5.set_xlabel("Latensi E2E (detik)", fontsize=9)
    ax5.set_ylabel("Densitas", fontsize=9)
    ax5.set_title("Distribusi Latensi (Pers. 4.19)", fontsize=10, fontweight='bold')
    ax5.legend(fontsize=7.5)
    ax5.grid(True, alpha=0.2)

    # ── Power budget ──
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.set_facecolor(BG)
    soc_t = np.linspace(0, 168, 1000)
    soc_bat = np.maximum(0, 100 - soc_t / 1.68)
    soc_solar = np.clip(
        100 - soc_t / 1.68 + np.maximum(0, np.sin(soc_t * np.pi / 12)) * 22, 0, 100)
    ax6.plot(soc_t, soc_bat, color=C_RED, lw=2.2, label="Hanya baterai (Samsung 30Q)")
    ax6.plot(soc_t, soc_solar, color=C_GREEN, lw=2.2, label="Baterai + panel surya 10Wp")
    ax6.axhline(20, color=C_ORANGE, lw=1.5, ls='--', alpha=0.7,
                label="Batas low-power mode (20%)")
    ax6.fill_between(soc_t, 0, 20, alpha=0.08, color=C_RED)
    ax6.set_xlabel("Waktu operasi (jam)", fontsize=9)
    ax6.set_ylabel("SoC Baterai (%)", fontsize=9)
    ax6.set_title("Power Budget Relay (Pers. 4.21–4.22)", fontsize=10, fontweight='bold')
    ax6.set_xlim(0, 168)
    ax6.set_ylim(0, 105)
    ax6.legend(fontsize=7.5, loc='upper right')
    ax6.grid(True, alpha=0.2)

    plt.savefig(f"{OUT}/fig11_metrics_summary.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ fig11_metrics_summary.png")


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Membuat gambar Bab 10 siLacak...\n")
    fig1_terrain_fresnel()
    fig2_obstacle_map()
    fig3_gps_flowchart()
    fig4_battery_curve()
    fig5_link_budget_waterfall()
    fig6_routing_graph()
    fig7_sf_tradeoff()
    fig8_per_sigmoid()
    fig9_drop_reason_tree()
    fig10_sos_retry()
    fig11_metrics_summary()
    print(f"\nSelesai! Semua gambar tersimpan di:\n{OUT}/")
