"""
Gambar dari data world Gazebo siLacak yang sesungguhnya.
Menggunakan:
  - terrain_height_world() dari scenario.py (fungsi terrain asli)
  - LORA_NODES, BASE_STATION, RADIO_OBSTACLES, TRAILS dari scenario.py
  - mesh DAE mountain_terrain diparse untuk kontur 3D view
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.ticker as mticker
import matplotlib.patheffects as pe
import warnings
warnings.filterwarnings('ignore')

sys.path.insert(0, '/home/ammar/Documents/Simulasi_Hiking/src/hiking_lora_sim')
from hiking_lora_sim.scenario import (
    terrain_height_world, LORA_NODES, BASE_STATION,
    RADIO_OBSTACLES, TRAILS, DEFAULT_METERS_PER_WORLD_UNIT
)

OUT = "/home/ammar/Documents/Simulasi_Hiking/figures_bab10"
DPI = 180
BG  = "#f5f5f5"

# ─── Warna ─────────────────────────────────────────────────────────────────────
C_BS     = "#c62828"
C_RELAY  = "#1565c0"
C_HIKER  = "#e65100"
C_RIDGE  = "#2e7d32"
C_VALLEY = "#0277bd"
C_CRATER = "#6a1b9a"
C_GRID   = "#cfd8dc"

# ─── Build terrain grid ─────────────────────────────────────────────────────────
N = 280
X_range = np.linspace(-120, 100, N)
Y_range = np.linspace(-110, 100, N)
XX, YY  = np.meshgrid(X_range, Y_range)
ZZ      = np.vectorize(terrain_height_world)(XX, YY)

# Semua node
all_nodes = [BASE_STATION] + LORA_NODES

def node_z(x, y, mast=10.0):
    return terrain_height_world(x, y) + mast / (DEFAULT_METERS_PER_WORLD_UNIT)


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR A — Peta Ketinggian Terrain + Semua Node + Rute (dari world asli)
# ══════════════════════════════════════════════════════════════════════════════
def figA_terrain_map():
    fig, ax = plt.subplots(figsize=(13, 11), facecolor=BG)
    ax.set_facecolor("#e8ecef")

    # Terrain heatmap
    terrain_cmap = plt.cm.terrain
    im = ax.contourf(XX, YY, ZZ, levels=40, cmap=terrain_cmap, alpha=0.85, zorder=0)
    cs = ax.contour(XX, YY, ZZ, levels=20, colors='white', linewidths=0.4,
                    alpha=0.4, zorder=1)
    cbar = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
    cbar.set_label("Ketinggian terrain (satuan dunia Gazebo)\n"
                   "terrain_height_world(x,y) — fungsi asli simulasi",
                   fontsize=9)

    # Zona obstacle radio
    obs_styles = {
        'trees':   ("#33691e", "Vegetasi / Hutan"),
        'rocks':   ("#4e342e", "Batuan"),
        'crater':  ("#bf360c", "Kawah"),
        'terrain': ("#546e7a", "Terrain Shadow"),
    }
    for obs in RADIO_OBSTACLES:
        col, _ = obs_styles.get(obs.kind, ("#9e9e9e", "Unknown"))
        circ = Circle((obs.x, obs.y), obs.radius, color=col, alpha=0.22,
                      zorder=2, linewidth=0)
        ax.add_patch(circ)
        circ2 = Circle((obs.x, obs.y), obs.radius, fill=False,
                        edgecolor=col, lw=2.0, linestyle='--', zorder=3)
        ax.add_patch(circ2)
        ax.text(obs.x, obs.y,
                f"{obs.name.replace('_',' ').title()}\nL={obs.loss_db} dB",
                ha='center', va='center', fontsize=7,
                color='white', fontweight='bold', zorder=4,
                bbox=dict(fc=col, alpha=0.7, boxstyle='round,pad=0.15', ec='none'))

    # Rute pendakian
    route_styles = {
        'ridge_route':  (C_RIDGE,  'Ridge Route (Punggungan)',   '-',  2.8),
        'valley_route': (C_VALLEY, 'Valley Route (Lembah)',      '--', 2.5),
        'crater_route': (C_CRATER, 'Crater Route (Kawah)',       ':',  2.5),
    }
    for rname, pts in TRAILS.items():
        col, lbl, ls, lw = route_styles[rname]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=col, lw=lw, linestyle=ls,
                label=lbl, zorder=5, solid_capstyle='round')
        for px, py in pts:
            ax.plot(px, py, 'o', color=col, ms=5, zorder=6,
                    markeredgecolor='white', markeredgewidth=0.6)

    # Base station
    bx, by = BASE_STATION.x, BASE_STATION.y
    bz = terrain_height_world(bx, by)
    ax.plot(bx, by, '*', color=C_BS, ms=22, zorder=8,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.annotate(f"Base Station\n(Basecamp)\n[{bx:.0f}, {by:.0f}]\nz={bz:.1f}",
                (bx, by), xytext=(bx + 8, by + 10),
                fontsize=8, color=C_BS, fontweight='bold', zorder=9,
                bbox=dict(fc='white', ec=C_BS, alpha=0.9,
                          boxstyle='round,pad=0.25'),
                arrowprops=dict(arrowstyle='->', color=C_BS, lw=1.3))

    # Relay nodes
    relay_labels = {
        'node_basecamp_gate': 'Basecamp Gate',
        'node_valley_watch':  'Valley Watch',
        'node_forest_pass':   'Forest Pass',
        'node_ridge_mid':     'Ridge Mid',
        'node_crater_edge':   'Crater Edge',
        'node_upper_traverse':'Upper Traverse',
        'node_north_saddle':  'North Saddle',
        'node_summit_view':   'Summit View',
    }
    offsets = {
        'node_basecamp_gate': (-20, 5),
        'node_valley_watch':  (-22, -8),
        'node_forest_pass':   (8, -10),
        'node_ridge_mid':     (10, 5),
        'node_crater_edge':   (10, -8),
        'node_upper_traverse':(-20, 8),
        'node_north_saddle':  (8, 5),
        'node_summit_view':   (8, -8),
    }
    for node in LORA_NODES:
        nz = terrain_height_world(node.x, node.y)
        ax.plot(node.x, node.y, '^', color=C_RELAY, ms=13, zorder=8,
                markeredgecolor='white', markeredgewidth=1.2)
        dx, dy = offsets.get(node.name, (6, 4))
        short = relay_labels.get(node.name, node.name)
        ax.annotate(f"{short}\n[{node.x:.0f},{node.y:.0f}] z={nz:.1f}",
                    (node.x, node.y), xytext=(node.x + dx, node.y + dy),
                    fontsize=7.5, color=C_RELAY, fontweight='bold', zorder=9,
                    bbox=dict(fc='white', ec=C_RELAY, alpha=0.88,
                              boxstyle='round,pad=0.2'),
                    arrowprops=dict(arrowstyle='->', color=C_RELAY,
                                   lw=1.0, alpha=0.8))

    # Contoh pendaki di jalur ridge
    ridge_pts = TRAILS['ridge_route']
    hike_idx = 5  # tengah jalur
    hx, hy = ridge_pts[hike_idx]
    ax.plot(hx, hy, 'D', color=C_HIKER, ms=13, zorder=8,
            markeredgecolor='white', markeredgewidth=1.2)
    ax.annotate("Pendaki\n(Portable Node)\nridge_route",
                (hx, hy), xytext=(hx - 30, hy + 12),
                fontsize=8, color=C_HIKER, fontweight='bold', zorder=9,
                bbox=dict(fc='white', ec=C_HIKER, alpha=0.9,
                          boxstyle='round,pad=0.25'),
                arrowprops=dict(arrowstyle='->', color=C_HIKER, lw=1.3))

    # Anotasi puncak
    ax.text(74, 70, "SUMMIT\n(z≈65)", ha='center', va='center',
            fontsize=9, color='white', fontweight='bold', zorder=7,
            bbox=dict(fc="#37474f", ec='white', alpha=0.85,
                      boxstyle='round,pad=0.3'))

    # Legend
    legend_els = [
        Line2D([0],[0], color=C_RIDGE,  lw=2.8, ls='-',  label='Ridge Route'),
        Line2D([0],[0], color=C_VALLEY, lw=2.5, ls='--', label='Valley Route'),
        Line2D([0],[0], color=C_CRATER, lw=2.5, ls=':',  label='Crater Route'),
        Line2D([0],[0], marker='*', color='w', markerfacecolor=C_BS,
               ms=14, label='Base Station'),
        Line2D([0],[0], marker='^', color='w', markerfacecolor=C_RELAY,
               ms=10, label='Relay Node (8 node)'),
        Line2D([0],[0], marker='D', color='w', markerfacecolor=C_HIKER,
               ms=10, label='Pendaki (Portable Node)'),
        mpatches.Patch(fc='#33691e', alpha=0.55, label='Zona Vegetasi'),
        mpatches.Patch(fc='#4e342e', alpha=0.55, label='Zona Batuan'),
        mpatches.Patch(fc='#bf360c', alpha=0.55, label='Zona Kawah'),
    ]
    ax.legend(handles=legend_els, loc='lower right', fontsize=8.5,
              framealpha=0.92, ncol=2, title="Legenda", title_fontsize=9)

    ax.set_xlim(-120, 105)
    ax.set_ylim(-115, 105)
    ax.set_xlabel("Koordinat X — World Gazebo (1 wu = 35 m nyata)", fontsize=10)
    ax.set_ylabel("Koordinat Y — World Gazebo", fontsize=10)
    ax.set_title("Gambar 10.3 — Peta Terrain siLacak dari World Gazebo\n"
                 "Ketinggian, Rute Pendakian, Relay Node, dan Zona Obstacle Radio (Data Aktual Simulasi)",
                 fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.2, color=C_GRID, lw=0.7)

    plt.tight_layout()
    plt.savefig(f"{OUT}/worldA_terrain_map.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ worldA_terrain_map.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR B — View 3D Terrain + Node + Rute (perspektif Gazebo-like)
# ══════════════════════════════════════════════════════════════════════════════
def figB_terrain_3d():
    fig = plt.figure(figsize=(15, 10), facecolor='#1a1a2e')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#1a1a2e')

    # Terrain 3D — resolusi lebih kasar untuk kecepatan
    N3 = 120
    X3 = np.linspace(-120, 100, N3)
    Y3 = np.linspace(-110, 100, N3)
    XX3, YY3 = np.meshgrid(X3, Y3)
    ZZ3 = np.vectorize(terrain_height_world)(XX3, YY3)

    # Colormap terrain
    norm = mcolors.Normalize(vmin=ZZ3.min(), vmax=ZZ3.max())
    colors_3d = plt.cm.terrain(norm(ZZ3))

    surf = ax.plot_surface(XX3, YY3, ZZ3, facecolors=colors_3d,
                           rstride=2, cstride=2, alpha=0.85,
                           linewidth=0, antialiased=True, zorder=1)

    # Rute
    route_styles = {
        'ridge_route':  (C_RIDGE,  3.0, '-'),
        'valley_route': (C_VALLEY, 2.5, '--'),
        'crater_route': (C_CRATER, 2.5, ':'),
    }
    for rname, pts in TRAILS.items():
        col, lw, ls = route_styles[rname]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [terrain_height_world(p[0], p[1]) + 1.0 for p in pts]
        ax.plot(xs, ys, zs, color=col, lw=lw, linestyle=ls, zorder=5,
                label=rname.replace('_', ' ').title())

    # Base station
    bx, by = BASE_STATION.x, BASE_STATION.y
    bz = terrain_height_world(bx, by) + 1.5
    ax.scatter([bx], [by], [bz], s=200, c=C_BS, marker='*', zorder=8,
               edgecolors='white', linewidth=1.5, label='Base Station')
    ax.text(bx, by, bz + 3, 'Base\nStation', color=C_BS, fontsize=8,
            fontweight='bold', ha='center')

    # Relay nodes — kolom vertikal untuk visibilitas
    for node in LORA_NODES:
        nz_base = terrain_height_world(node.x, node.y)
        nz_top  = nz_base + 2.5
        ax.plot([node.x, node.x], [node.y, node.y], [nz_base, nz_top],
                color=C_RELAY, lw=2.5, zorder=7, alpha=0.9)
        ax.scatter([node.x], [node.y], [nz_top], s=120, c=C_RELAY,
                   marker='^', zorder=8, edgecolors='white', linewidth=1.0)
        short = node.name.replace('node_', '').replace('_', '\n')
        ax.text(node.x, node.y, nz_top + 2.5, short,
                color='#90caf9', fontsize=6.5, ha='center', fontweight='bold')

    # Pendaki
    hx, hy = TRAILS['ridge_route'][5]
    hz = terrain_height_world(hx, hy) + 1.5
    ax.scatter([hx], [hy], [hz], s=150, c=C_HIKER, marker='D',
               zorder=8, edgecolors='white', linewidth=1.2)
    ax.text(hx, hy, hz + 3, 'Pendaki\n(ridge)', color=C_HIKER,
            fontsize=7.5, ha='center', fontweight='bold')

    # Simulasi link radio TX-RX (pendaki → ridge_mid)
    rm = next(n for n in LORA_NODES if n.name == 'node_ridge_mid')
    rmz = terrain_height_world(rm.x, rm.y) + 2.5
    ax.plot([hx, rm.x], [hy, rm.y], [hz, rmz],
            color='#ffeb3b', lw=2.0, linestyle='--', alpha=0.75,
            label='Link Radio aktif', zorder=6)
    ax.text((hx + rm.x) / 2, (hy + rm.y) / 2, (hz + rmz) / 2 + 2,
            'LoRa Link\n+22 dB', color='#ffeb3b', fontsize=7.5, ha='center')

    # Styling
    ax.set_xlabel("X (World Units)", fontsize=9, color='white', labelpad=8)
    ax.set_ylabel("Y (World Units)", fontsize=9, color='white', labelpad=8)
    ax.set_zlabel("Ketinggian", fontsize=9, color='white', labelpad=8)
    ax.tick_params(colors='#90a4ae', labelsize=7)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#37474f')
    ax.yaxis.pane.set_edgecolor('#37474f')
    ax.zaxis.pane.set_edgecolor('#37474f')
    ax.grid(True, color='#37474f', lw=0.5, alpha=0.5)

    ax.view_init(elev=32, azim=-55)
    ax.set_title("Gambar 10.4 — Visualisasi 3D Terrain World Gazebo siLacak\n"
                 "Terrain prosedural, rute pendakian, relay node, dan simulasi link radio",
                 fontsize=12, fontweight='bold', color='white', pad=12)

    legend_els = [
        Line2D([0],[0], color=C_RIDGE,  lw=2.5, label='Ridge Route'),
        Line2D([0],[0], color=C_VALLEY, lw=2.5, ls='--', label='Valley Route'),
        Line2D([0],[0], color=C_CRATER, lw=2.5, ls=':', label='Crater Route'),
        Line2D([0],[0], marker='*', color='w', markerfacecolor=C_BS,
               ms=12, label='Base Station'),
        Line2D([0],[0], marker='^', color='w', markerfacecolor=C_RELAY,
               ms=9, label='Relay Node'),
        Line2D([0],[0], marker='D', color='w', markerfacecolor=C_HIKER,
               ms=9, label='Pendaki'),
        Line2D([0],[0], color='#ffeb3b', lw=2, ls='--', label='LoRa Link'),
    ]
    legend = ax.legend(handles=legend_els, loc='upper left', fontsize=8,
                       framealpha=0.3, facecolor='#1a1a2e',
                       labelcolor='white', edgecolor='#546e7a')

    plt.tight_layout()
    plt.savefig(f"{OUT}/worldB_terrain_3d.png", dpi=DPI, bbox_inches='tight',
                facecolor='#1a1a2e')
    plt.close()
    print("✓ worldB_terrain_3d.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR C — Profil Ketinggian Ketiga Rute (Elevation Profile)
# ══════════════════════════════════════════════════════════════════════════════
def figC_elevation_profiles():
    fig, axes = plt.subplots(3, 1, figsize=(13, 12), facecolor=BG, sharex=False)
    fig.suptitle("Gambar 10.5 — Profil Ketinggian Ketiga Rute Pendakian\n"
                 "Berdasarkan fungsi terrain_height_world() dari world Gazebo siLacak",
                 fontsize=12, fontweight='bold', y=0.99)

    route_cfg = [
        ('ridge_route',  C_RIDGE,  'Ridge Route — Jalur Punggungan',
         "Jalur LOS dominan, terrain shadow minimal, PDR tinggi"),
        ('valley_route', C_VALLEY, 'Valley Route — Jalur Lembah',
         "Lintasan melalui vegetasi lebat, NLOS tinggi, GPS error besar"),
        ('crater_route', C_CRATER, 'Crater Route — Jalur Kawah',
         "Melewati kawah, diffraction loss signifikan, relay area atas kritis"),
    ]

    M_PER_WU = DEFAULT_METERS_PER_WORLD_UNIT

    for ax, (rname, col, title, desc) in zip(axes, route_cfg):
        ax.set_facecolor(BG)
        pts = TRAILS[rname]

        # Buat jalur interpolasi halus
        n_interp = 300
        t_pts = np.linspace(0, 1, len(pts))
        t_fine = np.linspace(0, 1, n_interp)
        xs_raw = [p[0] for p in pts]
        ys_raw = [p[1] for p in pts]
        xs = np.interp(t_fine, t_pts, xs_raw)
        ys = np.interp(t_fine, t_pts, ys_raw)
        zs_wu = np.array([terrain_height_world(x, y) for x, y in zip(xs, ys)])
        zs_m  = zs_wu * M_PER_WU  # konversi ke meter

        # Jarak kumulatif (meter)
        dx = np.diff(xs) * M_PER_WU
        dy = np.diff(ys) * M_PER_WU
        dist = np.concatenate([[0], np.cumsum(np.sqrt(dx**2 + dy**2))])

        # Plot fill
        ax.fill_between(dist/1000, 0, zs_m, color=col, alpha=0.18)
        ax.plot(dist/1000, zs_m, color=col, lw=2.5, label=title)

        # Relay nodes pada rute ini
        for node in LORA_NODES:
            # Cari titik terdekat di rute
            dists_to_node = [np.sqrt((node.x-x)**2 + (node.y-y)**2)
                             for x, y in zip(xs, ys)]
            idx_closest = np.argmin(dists_to_node)
            if dists_to_node[idx_closest] < 15:
                nz_m = terrain_height_world(node.x, node.y) * M_PER_WU
                nd = dist[idx_closest] / 1000
                ax.plot(nd, nz_m + 50, '^', color=C_RELAY, ms=11,
                        markeredgecolor='white', markeredgewidth=1.0, zorder=5)
                short = node.name.replace('node_', '').replace('_', ' ')
                ax.annotate(short, (nd, nz_m + 60), ha='center',
                            fontsize=7.5, color=C_RELAY, fontweight='bold',
                            bbox=dict(fc='white', ec=C_RELAY, alpha=0.85,
                                      boxstyle='round,pad=0.15'))

        # Base station dan summit
        ax.plot(0, zs_m[0], '*', color=C_BS, ms=16,
                markeredgecolor='white', markeredgewidth=1.0, zorder=6)
        ax.annotate("Base\nStation", (0, zs_m[0]), xytext=(0.1, zs_m[0] + 80),
                    fontsize=8, color=C_BS, fontweight='bold',
                    bbox=dict(fc='white', ec=C_BS, alpha=0.85,
                              boxstyle='round,pad=0.15'),
                    arrowprops=dict(arrowstyle='->', color=C_BS, lw=1.2))
        ax.plot(dist[-1]/1000, zs_m[-1], 'D', color='#37474f', ms=12,
                markeredgecolor='white', markeredgewidth=1.0, zorder=6)
        ax.annotate(f"SUMMIT\n{zs_m[-1]:.0f} m", (dist[-1]/1000, zs_m[-1]),
                    xytext=(dist[-1]/1000 - 0.5, zs_m[-1] + 80),
                    fontsize=8, color='#37474f', fontweight='bold',
                    bbox=dict(fc='white', ec='#37474f', alpha=0.85,
                              boxstyle='round,pad=0.15'),
                    arrowprops=dict(arrowstyle='->', color='#37474f', lw=1.2))

        # Zona obstacle
        for obs in RADIO_OBSTACLES:
            obs_dists = [np.sqrt((obs.x-x)**2 + (obs.y-y)**2)
                         for x, y in zip(xs, ys)]
            min_d = min(obs_dists)
            if min_d < obs.radius * 1.5:
                idx_o = np.argmin(obs_dists)
                od = dist[idx_o] / 1000
                col_o = {"trees":"#33691e","rocks":"#4e342e",
                         "crater":"#bf360c","terrain":"#546e7a"}.get(obs.kind,"#9e9e9e")
                span = obs.radius * M_PER_WU / 1000
                ax.axvspan(max(0, od - span/2), od + span/2,
                           alpha=0.12, color=col_o,
                           label=f"Obstacle: {obs.name} ({obs.loss_db} dB)")
                ax.text(od, ax.get_ylim()[0] if ax.get_ylim()[0] > 0 else 50,
                        f"L={obs.loss_db}dB", ha='center', fontsize=7,
                        color=col_o, fontweight='bold', alpha=0.9)

        total_km = dist[-1] / 1000
        gain_m   = zs_m[-1] - zs_m[0]
        ax.set_title(f"({['a','b','c'][axes.tolist().index(ax)]}) {title}\n"
                     f"    {desc}",
                     fontsize=10, fontweight='bold', loc='left', pad=4)
        ax.set_xlabel("Jarak horizontal dari basecamp (km)", fontsize=9)
        ax.set_ylabel("Ketinggian (m nyata)", fontsize=9)
        ax.text(0.98, 0.97,
                f"Total jarak: {total_km:.1f} km\nTotal elevasi: +{gain_m:.0f} m",
                transform=ax.transAxes, ha='right', va='top',
                fontsize=9, color=col,
                bbox=dict(fc='white', ec=col, alpha=0.85, boxstyle='round,pad=0.3'))
        ax.grid(True, alpha=0.25, lw=0.7)
        ax.set_ylim(bottom=0)

    plt.tight_layout(rect=[0, 0, 1, 0.975])
    plt.savefig(f"{OUT}/worldC_elevation_profiles.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ worldC_elevation_profiles.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR D — Link Budget aktual dari data posisi relay (RSSI vs jarak real)
# ══════════════════════════════════════════════════════════════════════════════
def figD_real_link_budget():
    """
    Hitung FSPL dan link margin antar SEMUA pasangan node (sesuai lora_network.py)
    menggunakan koordinat nyata dari scenario.py
    """
    import itertools

    FREQ_MHZ = 923.0
    TX_DBM   = 17.0
    G_ANT    = 2.0   # gain antena tx + rx

    def fspl(d_m):
        if d_m < 1: d_m = 1
        return 20*np.log10(d_m/1000) + 20*np.log10(FREQ_MHZ) + 32.44

    def dist3d(n1, n2):
        z1 = node_z(n1.x, n1.y)
        z2 = node_z(n2.x, n2.y)
        return np.sqrt((n1.x-n2.x)**2 + (n1.y-n2.y)**2 + (z1-z2)**2) \
               * DEFAULT_METERS_PER_WORLD_UNIT

    sensitivity = {7:-123, 8:-126, 9:-129, 10:-132, 11:-134.5, 12:-136}
    sf_colors   = {7:'#e53935', 8:'#fb8c00', 9:'#fdd835',
                   10:'#43a047', 11:'#1e88e5', 12:'#8e24aa'}

    all_nd = [BASE_STATION] + LORA_NODES
    pairs = list(itertools.combinations(all_nd, 2))

    fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor=BG)
    fig.suptitle("Gambar 10.6 — Link Budget Aktual Antar Node siLacak\n"
                 "Dihitung dari koordinat nyata scenario.py dengan FSPL 923 MHz (Persamaan 4.6–4.9)",
                 fontsize=12, fontweight='bold')

    # ── ax[0]: RSSI vs jarak semua pasangan per SF ──
    ax = axes[0]
    ax.set_facecolor(BG)

    d_range = np.linspace(10, 8000, 500)
    for sf, col in sf_colors.items():
        rssi_curve = TX_DBM + 2*G_ANT - (20*np.log10(d_range/1000)
                     + 20*np.log10(FREQ_MHZ) + 32.44)
        ax.plot(d_range, rssi_curve, color=col, lw=1.5, alpha=0.5,
                label=f"SF{sf} FSPL (S={sensitivity[sf]} dBm)")
        ax.axhline(sensitivity[sf], color=col, lw=0.8, ls='--', alpha=0.35)

    # Plot titik nyata setiap pasangan node
    for n1, n2 in pairs:
        d = dist3d(n1, n2)
        rssi_ideal = TX_DBM + 2*G_ANT - fspl(d)
        is_viable = rssi_ideal > sensitivity[9]  # cek dengan SF9
        col_dot = C_GREEN if is_viable else C_RED
        ax.scatter(d, rssi_ideal, s=55, color=col_dot, zorder=5,
                   alpha=0.85, edgecolors='white', linewidth=0.6)
        # Label jarak untuk link panjang
        if d > 4000:
            short1 = n1.name.replace('node_','').replace('_',' ')
            short2 = n2.name.replace('node_','').replace('_',' ')
            ax.annotate(f"{short1}↔{short2}\n{d:.0f}m",
                        (d, rssi_ideal), fontsize=6.5,
                        xytext=(0, -18), textcoords='offset points',
                        ha='center', color=C_RED,
                        bbox=dict(fc='white', ec=C_RED, alpha=0.7,
                                  boxstyle='round,pad=0.1'))

    ax.axhline(-129, color='black', lw=2.0, ls='-',
               label="Sensitivitas SF9 = −129 dBm", alpha=0.8)
    ax.fill_between(d_range, -129, ax.get_ylim()[0] if ax.get_ylim()[0] else -170,
                    alpha=0.06, color=C_RED)
    ax.text(6000, -132, "Zone tidak\nterjangkau SF9", fontsize=8,
            color=C_RED, ha='center')

    ax.set_xlabel("Jarak antar node (m)", fontsize=10)
    ax.set_ylabel("RSSI Ideal (dBm)", fontsize=10)
    ax.set_title("(a) RSSI vs Jarak semua pasangan node nyata\n"
                 "● = layak SF9,  ● = tidak layak", fontsize=9.5, pad=5)
    ax.legend(fontsize=7.5, loc='upper right', framealpha=0.9, ncol=2)
    ax.grid(True, alpha=0.2)
    ax.set_xlim(0, 9000)

    # ── ax[1]: Heatmap link margin antar semua pasangan ──
    ax2 = axes[1]
    n_nodes = len(all_nd)
    margin_matrix = np.full((n_nodes, n_nodes), np.nan)
    node_names = [n.name.replace('node_','').replace('_','\n') for n in all_nd]
    node_names[0] = 'Base\nStation'

    for i, n1 in enumerate(all_nd):
        for j, n2 in enumerate(all_nd):
            if i != j:
                d = dist3d(n1, n2)
                rssi = TX_DBM + 2*G_ANT - fspl(d)
                margin_matrix[i, j] = rssi - sensitivity[9]

    im2 = ax2.imshow(margin_matrix, cmap='RdYlGn', vmin=-20, vmax=50,
                     aspect='auto')
    ax2.set_xticks(range(n_nodes))
    ax2.set_yticks(range(n_nodes))
    ax2.set_xticklabels(node_names, fontsize=7.5, rotation=45, ha='right')
    ax2.set_yticklabels(node_names, fontsize=7.5)
    ax2.set_title("(b) Heatmap Link Margin (dB) SF9\nantar semua pasangan node",
                  fontsize=9.5, pad=5)
    cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
    cbar2.set_label("Link Margin (dB)\n≥0 = layak,  <0 = tidak layak", fontsize=8)

    # Nilai dalam sel
    for i in range(n_nodes):
        for j in range(n_nodes):
            if not np.isnan(margin_matrix[i, j]):
                v = margin_matrix[i, j]
                color = 'black' if abs(v) < 30 else 'white'
                ax2.text(j, i, f"{v:.0f}", ha='center', va='center',
                         fontsize=7, color=color, fontweight='bold')

    plt.tight_layout()
    plt.savefig(f"{OUT}/worldD_real_link_budget.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ worldD_real_link_budget.png")


# ══════════════════════════════════════════════════════════════════════════════
# GAMBAR E — Kontur Terrain tiap Rute dengan LOS/NLOS Check real
# ══════════════════════════════════════════════════════════════════════════════
def figE_los_nlos_analysis():
    """
    Untuk setiap titik di setiap rute: apakah link ke relay terdekat LOS atau NLOS?
    Gunakan sampling profil terrain dari scenario.terrain_height_world
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), facecolor=BG)
    fig.suptitle("Gambar 10.7 — Analisis LOS/NLOS Pendaki ke Relay Node\n"
                 "Berdasarkan sampling terrain_height_world() — data nyata simulasi",
                 fontsize=12, fontweight='bold', y=0.99)

    route_cfg = [
        ('ridge_route',  C_RIDGE,  '(a) Ridge Route'),
        ('valley_route', C_VALLEY, '(b) Valley Route'),
        ('crater_route', C_CRATER, '(c) Crater Route'),
    ]

    def check_los(x1, y1, z1, x2, y2, z2, n_samples=30):
        """Return True jika LOS (tidak ada terrain yang memotong garis lurus)"""
        for t in np.linspace(0.05, 0.95, n_samples):
            xi = x1 + t*(x2-x1)
            yi = y1 + t*(y2-y1)
            zi_line = z1 + t*(z2-z1)
            zi_terrain = terrain_height_world(xi, yi)
            if zi_terrain > zi_line + 0.3:
                return False
        return True

    MAST_WU = 10.0 / DEFAULT_METERS_PER_WORLD_UNIT  # tinggi mast dalam world units

    for ax, (rname, col, title) in zip(axes, route_cfg):
        ax.set_facecolor(BG)
        pts = TRAILS[rname]

        # Interpolasi rute halus
        n_interp = 80
        t_pts = np.linspace(0, 1, len(pts))
        t_fine = np.linspace(0, 1, n_interp)
        xs = np.interp(t_fine, t_pts, [p[0] for p in pts])
        ys = np.interp(t_fine, t_pts, [p[1] for p in pts])
        zs = np.array([terrain_height_world(x, y) for x, y in zip(xs, ys)])

        # Jarak kumulatif km
        dx = np.diff(xs) * DEFAULT_METERS_PER_WORLD_UNIT
        dy = np.diff(ys) * DEFAULT_METERS_PER_WORLD_UNIT
        dist = np.concatenate([[0], np.cumsum(np.sqrt(dx**2+dy**2))]) / 1000

        # Plot terrain
        ax.fill_between(dist, 0, zs * DEFAULT_METERS_PER_WORLD_UNIT,
                        color='#795548', alpha=0.4)
        ax.plot(dist, zs * DEFAULT_METERS_PER_WORLD_UNIT, color='#4e342e',
                lw=1.5, alpha=0.7)

        # Untuk setiap titik di rute, cari relay terdekat dan cek LOS
        los_status = []
        best_relays = []
        for i, (hx, hy, hz) in enumerate(zip(xs, ys, zs)):
            hz_ant = hz + MAST_WU * 0.3  # pendaki + sedikit ketinggian

            best_node = None
            best_margin = -999
            best_los = False

            for node in LORA_NODES:
                nz = terrain_height_world(node.x, node.y) + MAST_WU
                d_m = np.sqrt((hx-node.x)**2 + (hy-node.y)**2 + (hz_ant-nz)**2) \
                      * DEFAULT_METERS_PER_WORLD_UNIT
                if d_m < 50: continue
                fspl_val = 20*np.log10(max(d_m,1)/1000) + 20*np.log10(923) + 32.44
                rssi = 17 + 4 - fspl_val
                margin = rssi - (-129)  # SF9
                if margin > best_margin:
                    best_margin = margin
                    best_node = node
                    best_los = check_los(hx, hy, hz_ant,
                                         node.x, node.y, nz, n_samples=20)

            los_status.append(best_los)
            best_relays.append((best_node, best_margin))

        # Plot LOS/NLOS di atas elevasi
        hiker_elev = zs * DEFAULT_METERS_PER_WORLD_UNIT + 80
        for i in range(len(dist)):
            is_los = los_status[i]
            c_dot = C_GREEN if is_los else C_RED
            ax.plot(dist[i], hiker_elev[i], 'o', color=c_dot,
                    ms=5, markeredgewidth=0, alpha=0.85, zorder=4)

        # Colorbar strip
        los_arr = np.array(los_status, dtype=float).reshape(1, -1)
        ax.imshow(los_arr, aspect='auto', cmap='RdYlGn',
                  extent=[dist[0], dist[-1],
                          max(zs)*DEFAULT_METERS_PER_WORLD_UNIT*1.12,
                          max(zs)*DEFAULT_METERS_PER_WORLD_UNIT*1.22],
                  vmin=0, vmax=1, zorder=3, alpha=0.7)

        n_los  = sum(los_status)
        n_nlos = len(los_status) - n_los
        pct_los = n_los / len(los_status) * 100

        # Relay nodes
        for node in LORA_NODES:
            nz_m = terrain_height_world(node.x, node.y) * DEFAULT_METERS_PER_WORLD_UNIT
            # Cari posisi node di rute terdekat
            dists_to_node = [np.sqrt((node.x-x)**2+(node.y-y)**2)
                             for x, y in zip(xs, ys)]
            idx_c = np.argmin(dists_to_node)
            if dists_to_node[idx_c] < 20:
                nd_km = dist[idx_c]
                ax.plot(nd_km, nz_m + 200, '^', color=C_RELAY, ms=11,
                        markeredgecolor='white', zorder=5)
                short = node.name.replace('node_','').replace('_',' ')
                ax.annotate(short, (nd_km, nz_m + 280), ha='center',
                            fontsize=7.5, color=C_RELAY, fontweight='bold',
                            bbox=dict(fc='white', ec=C_RELAY, alpha=0.85,
                                      boxstyle='round,pad=0.12'))

        ax.set_title(f"{title}\n"
                     f"LOS: {n_los}/{len(los_status)} titik ({pct_los:.0f}%)  |  "
                     f"NLOS: {n_nlos}/{len(los_status)} titik ({100-pct_los:.0f}%)",
                     fontsize=10, fontweight='bold', loc='left', pad=4)
        ax.set_xlabel("Jarak dari basecamp (km)", fontsize=9)
        ax.set_ylabel("Ketinggian (m nyata)", fontsize=9)

        legend_els = [
            Line2D([0],[0], marker='o', color='w', markerfacecolor=C_GREEN,
                   ms=8, label='LOS (Sinyal langsung)'),
            Line2D([0],[0], marker='o', color='w', markerfacecolor=C_RED,
                   ms=8, label='NLOS (Terrain menghalangi)'),
            Line2D([0],[0], marker='^', color='w', markerfacecolor=C_RELAY,
                   ms=9, label='Relay Node'),
        ]
        ax.legend(handles=legend_els, loc='upper left', fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.2)
        ax.set_ylim(bottom=0)

    plt.tight_layout(rect=[0, 0, 1, 0.975])
    plt.savefig(f"{OUT}/worldE_los_nlos_analysis.png", dpi=DPI, bbox_inches='tight')
    plt.close()
    print("✓ worldE_los_nlos_analysis.png")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Membuat gambar dari world data Gazebo siLacak...\n")
    figA_terrain_map()
    figB_terrain_3d()
    figC_elevation_profiles()
    figD_real_link_budget()
    figE_los_nlos_analysis()
    print(f"\nSelesai! Gambar tersimpan di:\n{OUT}/")
