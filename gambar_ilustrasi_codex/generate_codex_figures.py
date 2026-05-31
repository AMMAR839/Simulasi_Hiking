#!/usr/bin/env python3
"""Generate clean report figures for Bab 10 siLacak PoC.

The figures are intentionally illustrative and report-oriented. Terrain,
routes, relay nodes, base station, and obstacle coordinates are taken from the
current simulation scenario so the visuals stay consistent with the project.
"""

from __future__ import annotations

import math
import sys
import warnings
from pathlib import Path
from textwrap import fill

warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.colors import LinearSegmentedColormap


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src" / "hiking_lora_sim" / "hiking_lora_sim"))

import scenario  # noqa: E402


plt.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 220,
    "font.family": "DejaVu Sans",
    "font.size": 9,
    "axes.titlesize": 13,
    "axes.labelsize": 9,
    "axes.edgecolor": "#263238",
    "axes.linewidth": 0.8,
    "xtick.color": "#37474f",
    "ytick.color": "#37474f",
    "text.color": "#1f2933",
})


COL = {
    "navy": "#18324a",
    "blue": "#2b6cb0",
    "sky": "#90cdf4",
    "green": "#2f855a",
    "mint": "#9ae6b4",
    "orange": "#dd6b20",
    "red": "#c53030",
    "gray": "#718096",
    "light": "#f7fafc",
    "line": "#2d3748",
    "purple": "#6b46c1",
    "yellow": "#d69e2e",
}


def save(fig: plt.Figure, name: str) -> None:
    path = OUT / name
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def title(ax, text: str, subtitle: str | None = None) -> None:
    ax.set_title("")
    ax.text(
        0,
        1.09,
        text,
        transform=ax.transAxes,
        fontsize=13,
        fontweight="bold",
        color=COL["navy"],
        ha="left",
        va="bottom",
        clip_on=False,
    )
    if subtitle:
        ax.text(
            0,
            1.035,
            subtitle,
            transform=ax.transAxes,
            fontsize=8.5,
            color="#536471",
            va="bottom",
        )


def box(ax, xy, w, h, text, fc="#ffffff", ec=COL["line"], lw=1.1, fs=8.5):
    rect = patches.FancyBboxPatch(
        xy,
        w,
        h,
        boxstyle="round,pad=0.025,rounding_size=0.04",
        fc=fc,
        ec=ec,
        lw=lw,
    )
    ax.add_patch(rect)
    ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha="center", va="center", fontsize=fs)
    return rect


def arrow(ax, start, end, color=COL["line"], lw=1.2, style="-|>"):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(arrowstyle=style, color=color, lw=lw, shrinkA=3, shrinkB=3),
    )


def terrain_grid(n: int = 180):
    x = np.linspace(-115, 105, n)
    y = np.linspace(-105, 95, n)
    xx, yy = np.meshgrid(x, y)
    zz = np.vectorize(scenario.terrain_height_world)(xx, yy)
    return x, y, xx, yy, zz


def plot_routes_nodes(ax, labels: bool = True):
    route_colors = {
        "ridge_route": COL["orange"],
        "valley_route": COL["green"],
        "crater_route": COL["purple"],
    }
    for name, pts in scenario.TRAILS.items():
        arr = np.array(pts)
        ax.plot(arr[:, 0], arr[:, 1], lw=2.2, color=route_colors[name], label=name.replace("_", " "))
        ax.scatter(arr[:, 0], arr[:, 1], s=7, color=route_colors[name], zorder=4)

    ax.scatter([scenario.BASE_STATION.x], [scenario.BASE_STATION.y], s=90, marker="s",
               color=COL["blue"], edgecolor="white", linewidth=1.2, zorder=7, label="base station")
    ax.scatter([n.x for n in scenario.LORA_NODES], [n.y for n in scenario.LORA_NODES], s=55,
               marker="^", color=COL["yellow"], edgecolor="#5f370e", linewidth=0.8, zorder=7, label="relay node")

    if labels:
        ax.text(scenario.BASE_STATION.x - 7, scenario.BASE_STATION.y - 7, "base", fontsize=7, color=COL["blue"])
        for n in scenario.LORA_NODES:
            short = n.name.replace("node_", "")
            ax.text(n.x + 2, n.y + 2, short, fontsize=6.5, color="#3b2f13")


def fig01_architecture():
    fig, ax = plt.subplots(figsize=(11.5, 6.2))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6.4)
    ax.axis("off")
    title(ax, "Arsitektur Alur Data Simulasi siLacak", "ROS 2 menjalankan logika; Gazebo hanya sebagai visualisasi lingkungan.")

    box(ax, (0.4, 4.6), 2.2, 0.9, "Gazebo Harmonic\nterrain + visual node", "#eef7ff", COL["blue"])
    box(ax, (0.4, 2.9), 2.2, 0.9, "Node Pendaki\npose, GPS, baterai", "#f0fff4", COL["green"])
    box(ax, (3.5, 3.7), 2.2, 0.9, "Jaringan LoRa\nlink budget + routing", "#fffaf0", COL["orange"])
    box(ax, (6.6, 4.8), 2.0, 0.8, "Base Station\npaket diterima", "#ebf8ff", COL["blue"])
    box(ax, (6.6, 3.4), 2.0, 0.8, "Dashboard\nmonitor real-time", "#faf5ff", COL["purple"])
    box(ax, (6.6, 2.0), 2.0, 0.8, "Data Logger\nCSV simulasi", "#f7fafc", COL["gray"])
    box(ax, (9.6, 2.7), 1.9, 0.9, "Analisis Offline\ngrafik evaluasi", "#fff5f5", COL["red"])

    arrow(ax, (1.5, 4.6), (1.5, 3.8), COL["blue"])
    arrow(ax, (2.6, 3.35), (3.5, 4.05), COL["green"])
    arrow(ax, (5.7, 4.15), (6.6, 5.2), COL["orange"])
    arrow(ax, (5.7, 4.05), (6.6, 3.8), COL["orange"])
    arrow(ax, (5.7, 3.95), (6.6, 2.4), COL["orange"])
    arrow(ax, (8.6, 2.4), (9.6, 3.1), COL["gray"])

    ax.text(2.85, 4.45, "/pose\n/gps", fontsize=7, color=COL["green"], ha="center")
    ax.text(5.95, 4.7, "/network_event\n/location", fontsize=7, color=COL["orange"], ha="center")
    ax.text(8.95, 2.82, "positions.csv\nnetwork_events.csv", fontsize=7, color=COL["gray"], ha="center")
    save(fig, "codex_fig10_01_architecture.png")


def fig02_topology():
    fig, ax = plt.subplots(figsize=(7.2, 9.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 13)
    ax.axis("off")
    title(ax, "Topologi Konseptual LoRa Multi-Hop", "Relay bertingkat dari summit menuju basecamp.")

    nodes = {
        "Area Summit": (5, 12),
        "node_summit_view": (5, 10.7),
        "node_north_saddle": (5, 9.4),
        "node_upper_traverse": (2.4, 7.8),
        "node_crater_edge": (7.6, 7.8),
        "node_ridge_mid": (5, 6.2),
        "node_forest_pass": (2.4, 4.6),
        "node_valley_watch": (7.6, 4.6),
        "node_basecamp_gate": (5, 3.0),
        "Base Station\nBasecamp": (5, 1.4),
    }
    edges = [
        ("Area Summit", "node_summit_view"),
        ("node_summit_view", "node_north_saddle"),
        ("node_north_saddle", "node_upper_traverse"),
        ("node_north_saddle", "node_crater_edge"),
        ("node_upper_traverse", "node_ridge_mid"),
        ("node_crater_edge", "node_ridge_mid"),
        ("node_ridge_mid", "node_forest_pass"),
        ("node_ridge_mid", "node_valley_watch"),
        ("node_forest_pass", "node_basecamp_gate"),
        ("node_valley_watch", "node_basecamp_gate"),
        ("node_basecamp_gate", "Base Station\nBasecamp"),
    ]
    for a, b in edges:
        arrow(ax, nodes[a], nodes[b], "#566573", lw=1.3)
    for name, (x, y) in nodes.items():
        fc = "#e8f4fd" if "Base" in name else "#fff8e6" if "node" in name else "#edf2f7"
        ec = COL["blue"] if "Base" in name else COL["orange"] if "node" in name else COL["gray"]
        box(ax, (x - 1.45, y - 0.35), 2.9, 0.7, name, fc, ec, fs=8.2)
    save(fig, "codex_fig10_02_topology.png")


def fig03_environment_map():
    x, y, xx, yy, zz = terrain_grid()
    fig, ax = plt.subplots(figsize=(10.5, 8.0))
    cmap = LinearSegmentedColormap.from_list("terrain_report", ["#dff0d8", "#f4e4a6", "#b58b5b", "#f7fafc"])
    cf = ax.contourf(xx, yy, zz, levels=24, cmap=cmap, alpha=0.94)
    ax.contour(xx, yy, zz, levels=12, colors="#4a5568", linewidths=0.25, alpha=0.35)
    for obs in scenario.RADIO_OBSTACLES:
        color = {"trees": "#2f855a", "rocks": "#718096", "crater": "#c05621", "terrain": "#805ad5"}.get(obs.kind, "#718096")
        circ = patches.Circle((obs.x, obs.y), obs.radius, fc=color, ec=color, alpha=0.18, lw=1.6)
        ax.add_patch(circ)
        ax.text(obs.x, obs.y, obs.name.replace("_", "\n"), fontsize=6.4, ha="center", va="center", color="#1a202c")
    plot_routes_nodes(ax)
    cb = fig.colorbar(cf, ax=ax, fraction=0.036, pad=0.02)
    cb.set_label("ketinggian terrain relatif (world unit)")
    ax.set_aspect("equal")
    ax.set_xlabel("Koordinat X world")
    ax.set_ylabel("Koordinat Y world")
    ax.legend(loc="lower right", fontsize=7.2, frameon=True)
    title(ax, "Peta Lingkungan Simulasi", "Terrain, rute, obstacle radio, relay node, dan base station pada satu sistem koordinat.")
    save(fig, "codex_fig10_03_environment_map.png")


def fig04_los_diffraction():
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.9), sharey=True)
    xs = np.linspace(0, 1, 180)
    base = 0.18 + 0.18 * np.sin(xs * np.pi * 1.4) + 0.08 * np.sin(xs * np.pi * 5.0)
    profiles = [
        base * 0.55,
        base * 0.9 + 0.38 * np.exp(-((xs - 0.55) ** 2) / 0.015),
    ]
    titles = ["LOS bersih", "NLOS: terrain shadow + diffraction"]
    for ax, prof, ttl in zip(axes, profiles, titles):
        tx = (0.05, 0.45)
        rx = (0.95, 0.68)
        los = np.interp(xs, [tx[0], rx[0]], [tx[1], rx[1]])
        fresnel = 0.08 * np.sin(np.pi * xs)
        ax.fill_between(xs, los - fresnel, los + fresnel, color=COL["sky"], alpha=0.25, label="zona clearance")
        ax.plot(xs, los, color=COL["blue"], lw=2.0, label="garis LOS")
        ax.fill_between(xs, 0, prof, color="#8d6e63", alpha=0.65, label="terrain")
        ax.scatter([tx[0], rx[0]], [tx[1], rx[1]], s=70, color=[COL["green"], COL["orange"]], zorder=5)
        ax.text(tx[0], tx[1] + 0.06, "TX", ha="center", fontsize=8, fontweight="bold")
        ax.text(rx[0], rx[1] + 0.06, "RX", ha="center", fontsize=8, fontweight="bold")
        if "NLOS" in ttl:
            ax.annotate("diffraction loss", xy=(0.55, prof.max()), xytext=(0.40, 0.93),
                        arrowprops=dict(arrowstyle="->", color=COL["red"]), color=COL["red"], fontsize=8)
        ax.set_title(ttl, fontweight="bold", color=COL["navy"])
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1.08)
        ax.set_xlabel("posisi sepanjang link")
        ax.grid(True, alpha=0.18)
    axes[0].set_ylabel("elevasi relatif")
    axes[0].legend(loc="upper left", fontsize=7)
    fig.suptitle("Ilustrasi LOS, Zona Clearance, dan Difraksi Terrain", fontweight="bold", color=COL["navy"])
    save(fig, "codex_fig10_04_los_diffraction.png")


def fig05_network_3d():
    x, y, xx, yy, zz = terrain_grid(120)
    fig, ax = plt.subplots(figsize=(11.0, 7.2))
    # Pseudo-isometric projection. This avoids relying on mpl_toolkits.mplot3d
    # while still showing that node placement follows terrain elevation.
    uu = xx - 0.48 * yy
    vv = 0.42 * yy + 1.55 * zz
    mesh = ax.scatter(uu.ravel(), vv.ravel(), c=zz.ravel(), cmap="terrain", s=4, marker="s", alpha=0.78, linewidths=0)
    route_colors = {"ridge_route": COL["orange"], "valley_route": COL["green"], "crater_route": COL["purple"]}
    for name, pts in scenario.TRAILS.items():
        arr = np.array(pts)
        z = np.array([scenario.terrain_height_world(px, py) + 2 for px, py in arr])
        u = arr[:, 0] - 0.48 * arr[:, 1]
        v = 0.42 * arr[:, 1] + 1.55 * z
        ax.plot(u, v, color=route_colors[name], lw=2.8, label=name.replace("_", " "))
    for st in [scenario.BASE_STATION] + scenario.LORA_NODES:
        z = scenario.terrain_height_world(st.x, st.y) + 4
        u = st.x - 0.48 * st.y
        v = 0.42 * st.y + 1.55 * z
        marker = "s" if st.kind == "base" else "^"
        color = COL["blue"] if st.kind == "base" else COL["yellow"]
        ax.scatter([u], [v], s=65, marker=marker, color=color, edgecolor="black", linewidth=0.55, zorder=5)
    cb = fig.colorbar(mesh, ax=ax, fraction=0.034, pad=0.02)
    cb.set_label("ketinggian relatif")
    ax.set_aspect("equal")
    ax.set_xlabel("proyeksi isometrik X")
    ax.set_ylabel("proyeksi isometrik elevasi")
    ax.set_title("Visualisasi Isometrik Terrain, Rute, dan Node LoRa", loc="left", fontweight="bold", color=COL["navy"], pad=8)
    ax.legend(loc="upper left", fontsize=7)
    save(fig, "codex_fig10_05_network_3d.png")


def fig06_gps_pipeline():
    fig, ax = plt.subplots(figsize=(11.2, 6.2))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6.8)
    ax.axis("off")

    ax.text(0.0, 6.55, "Model GPS Sintetis", fontsize=15, fontweight="bold", color=COL["navy"], va="top")
    ax.text(
        0.0,
        6.18,
        "Alur pembentukan koordinat GPS dari posisi terrain sampai metrik error lokasi.",
        fontsize=9,
        color="#536471",
        va="top",
    )

    def rect(x, y, w, h, header, body, fc="#ffffff", ec="#1f2933", header_fc="#e8eef5"):
        ax.add_patch(patches.Rectangle((x, y), w, h, fc=fc, ec=ec, lw=1.05))
        ax.add_patch(patches.Rectangle((x, y + h - 0.38), w, 0.38, fc=header_fc, ec=ec, lw=1.05))
        ax.text(x + 0.12, y + h - 0.19, header, ha="left", va="center", fontsize=7.8, fontweight="bold", color=COL["navy"])
        ax.text(x + w / 2, y + (h - 0.38) / 2, body, ha="center", va="center", fontsize=8.0, color="#263238")

    y = 3.95
    h = 1.35
    rect(0.2, y, 1.65, h, "Input", "Posisi terrain\nx, y, altitude")
    rect(2.15, y, 1.75, h, "Transformasi", "Konversi lokal\nke lat/lon\n(Pers. 4.1)")
    rect(4.2, y, 1.55, h, "Validitas", "TTFF\nNO_FIX -> FIX", header_fc="#edf2f7")
    rect(8.9, y, 1.65, h, "Output", "GPS sintetis\nlat, lon, alt", fc="#f0fff4", header_fc="#d9f5e5")

    arrow(ax, (1.85, y + h / 2), (2.15, y + h / 2), "#455a64")
    arrow(ax, (3.9, y + h / 2), (4.2, y + h / 2), "#455a64")
    arrow(ax, (5.75, y + h / 2), (6.15, y + h / 2), "#455a64")
    arrow(ax, (8.55, y + h / 2), (8.9, y + h / 2), "#455a64")

    ax.add_patch(patches.Rectangle((6.15, 3.35), 2.4, 2.55, fc="#ffffff", ec=COL["orange"], lw=1.15))
    ax.add_patch(patches.Rectangle((6.15, 5.52), 2.4, 0.38, fc="#fff3df", ec=COL["orange"], lw=1.15))
    ax.text(6.27, 5.71, "Model Gangguan", ha="left", va="center", fontsize=7.8, fontweight="bold", color=COL["navy"])

    sub = [
        ("DOP lingkungan", 6.32, 4.86),
        ("Noise Gaussian", 7.42, 4.86),
        ("Multipath", 6.32, 4.05),
        ("Delay & drift", 7.42, 4.05),
    ]
    for txt, sx, sy in sub:
        ax.add_patch(patches.Rectangle((sx, sy), 0.95, 0.54, fc="#f8fafc", ec="#a0aec0", lw=0.8))
        ax.text(sx + 0.475, sy + 0.27, txt, ha="center", va="center", fontsize=6.8, color="#263238")

    ax.text(6.35, 3.55, r"$\sigma_{eff} = \sigma_{GPS}DOP + M_{multipath}$", fontsize=9, color=COL["orange"])

    ax.add_patch(patches.Rectangle((0.2, 1.05), 10.35, 1.55, fc="#f8fafc", ec="#cbd5e0", lw=0.9))
    ax.text(0.45, 2.25, "Persamaan yang digunakan", fontsize=8.3, fontweight="bold", color=COL["navy"])
    ax.text(0.45, 1.82, r"$\Delta lat = north_m/111320$", fontsize=8.5, color="#263238")
    ax.text(3.25, 1.82, r"$\Delta lon = east_m/(111320\cos(lat_0))$", fontsize=8.5, color="#263238")
    ax.text(7.25, 1.82, r"$e_{pos}=\sqrt{(\Delta x)^2+(\Delta y)^2}$", fontsize=8.5, color="#263238")
    ax.text(0.45, 1.35, "Keluaran GPS sintetis dibandingkan dengan posisi terrain untuk menghasilkan metrik GPS error.", fontsize=8.2, color="#536471")

    ax.text(10.72, 4.65, "Evaluasi", fontsize=8, fontweight="bold", color=COL["navy"])
    ax.add_patch(patches.Rectangle((10.72, 3.95), 1.05, 0.48, fc="#ffffff", ec=COL["blue"], lw=0.9))
    ax.text(11.245, 4.19, "GPS error", ha="center", va="center", fontsize=7.3)
    arrow(ax, (10.55, y + h / 2), (10.72, 4.19), COL["blue"])
    save(fig, "codex_fig10_06_gps_pipeline.png")


def fig07_battery_tx():
    soc = np.linspace(0, 100, 101)
    voltage = 3.0 + 1.2 * (soc / 100)
    reduction = np.where(soc < 30, (1 - soc / 30) * 6.0, 0)
    tx = 17.0 - reduction
    fig, ax1 = plt.subplots(figsize=(9.2, 4.8))
    ax1.plot(soc, voltage, color=COL["green"], lw=2.4, label="tegangan baterai")
    ax1.set_xlabel("State of Charge (%)")
    ax1.set_ylabel("Tegangan (V)", color=COL["green"])
    ax1.tick_params(axis="y", labelcolor=COL["green"])
    ax1.grid(True, alpha=0.22)
    ax2 = ax1.twinx()
    ax2.plot(soc, tx, color=COL["orange"], lw=2.4, label="effective TX power")
    ax2.set_ylabel("Effective TX Power (dBm)", color=COL["orange"])
    ax2.tick_params(axis="y", labelcolor=COL["orange"])
    ax1.axvspan(0, 30, color=COL["red"], alpha=0.08)
    ax1.text(15, 4.08, "zona reduksi TX power", ha="center", fontsize=8, color=COL["red"])
    title(ax1, "Model Baterai dan Pengaruhnya ke TX Power", "SoC rendah menurunkan daya pancar efektif dalam link budget.")
    save(fig, "codex_fig10_07_battery_tx.png")


def fig08_link_budget():
    def segment_intersects_circle(ax, ay, bx, by, cx, cy, radius):
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            return math.hypot(ax - cx, ay - cy) <= radius
        t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / length_sq))
        return math.hypot(ax + t * dx - cx, ay + t * dy - cy) <= radius

    def chord_length_m(ax, ay, bx, by, cx, cy, radius, scale):
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            dist = math.hypot(ax - cx, ay - cy)
            return 0.0 if dist > radius else 2.0 * math.sqrt(max(0.0, radius * radius - dist * dist)) * scale
        t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / length_sq))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        dist = math.hypot(closest_x - cx, closest_y - cy)
        if dist >= radius:
            return 0.0
        return 2.0 * math.sqrt(max(0.0, radius * radius - dist * dist)) * scale

    def radio_station(st):
        mast = 14.0 if st.kind == "base" else 10.0
        z = scenario.terrain_altitude_m(st.x, st.y, scenario.DEFAULT_METERS_PER_WORLD_UNIT) + mast
        return scenario.Station(st.name, st.x, st.y, z, st.kind)

    def obstacle_loss(left, right, weather="clear"):
        wet_factor = {"clear": 1.00, "fog": 1.15, "light_rain": 1.40, "heavy_rain": 1.70, "thunderstorm": 2.00}[weather]
        loss = 0.0
        crossed = []
        scale = scenario.DEFAULT_METERS_PER_WORLD_UNIT
        for obs in scenario.RADIO_OBSTACLES:
            if not segment_intersects_circle(left.x, left.y, right.x, right.y, obs.x, obs.y, obs.radius):
                continue
            chord = chord_length_m(left.x, left.y, right.x, right.y, obs.x, obs.y, obs.radius, scale)
            ref = 2.0 * obs.radius * scale
            depth = chord / ref if ref > 0 else 1.0
            if obs.kind == "trees":
                item_loss = obs.loss_db * depth * wet_factor
            else:
                item_loss = obs.loss_db * depth
            loss += max(0.0, item_loss)
            crossed.append(obs.name)
        return loss, crossed

    def terrain_shadow_loss(left, right, frequency_mhz=915.0):
        sample_count = 18
        scale = scenario.DEFAULT_METERS_PER_WORLD_UNIT
        obstructed = 0
        total_horiz_m = math.hypot((right.x - left.x) * scale, (right.y - left.y) * scale)
        worst_clearance_m = 0.0
        worst_d1_m = 1.0
        worst_d2_m = 1.0
        for s in range(1, sample_count):
            ratio = s / sample_count
            x = left.x + (right.x - left.x) * ratio
            y = left.y + (right.y - left.y) * ratio
            line_alt = left.z + (right.z - left.z) * ratio
            terrain_alt = scenario.terrain_altitude_m(x, y, scale)
            d1_h = ratio * total_horiz_m
            d2_h = (1.0 - ratio) * total_horiz_m
            curvature_m = (d1_h * d2_h) / (2.0 * 6_370_000.0)
            terrain_alt_eff = terrain_alt + curvature_m
            fresnel_clearance_m = 8.0 + 0.012 * min(s, sample_count - s) * scale
            if terrain_alt_eff + fresnel_clearance_m > line_alt:
                obstructed += 1
            clearance = terrain_alt_eff - line_alt
            if clearance > worst_clearance_m:
                worst_clearance_m = clearance
                worst_d1_m = max(1.0, d1_h)
                worst_d2_m = max(1.0, d2_h)
        shadow = min(22.0, 3.2 * obstructed) if obstructed > 0 else 0.0
        diffraction = 0.0
        if worst_clearance_m > 0.0:
            wavelength_m = 3e8 / (frequency_mhz * 1e6)
            denom = wavelength_m * worst_d1_m * worst_d2_m
            if denom > 0.0:
                nu = worst_clearance_m * math.sqrt(2.0 * (worst_d1_m + worst_d2_m) / denom)
                if nu > -0.7:
                    raw = 6.02 + 9.11 * nu + 1.27 * nu * nu if nu <= 2.4 else 13.46 + 20.0 * math.log10(nu)
                    diffraction = max(0.0, raw)
        return shadow, max(0.0, diffraction - shadow)

    stations = {st.name: radio_station(st) for st in [scenario.BASE_STATION] + scenario.LORA_NODES}
    left = stations["node_upper_traverse"]
    right = stations["node_north_saddle"]
    scale = scenario.DEFAULT_METERS_PER_WORLD_UNIT
    frequency_mhz = 915.0
    tx_power_dbm = 17.0
    antenna_gain_db = 2.0
    receiver_sensitivity_dbm = -129.0
    terrain_loss_db_per_km = 2.5
    weather = "clear"

    distance_m = math.sqrt(((left.x - right.x) * scale) ** 2 + ((left.y - right.y) * scale) ** 2 + (left.z - right.z) ** 2)
    distance_km = max(distance_m / 1000.0, 0.001)
    fspl = 32.44 + 20.0 * math.log10(distance_km) + 20.0 * math.log10(frequency_mhz)
    obs_loss, crossed = obstacle_loss(left, right, weather)
    shadow_loss, diffraction_loss = terrain_shadow_loss(left, right, frequency_mhz)
    terrain_loss = distance_km * terrain_loss_db_per_km
    weather_loss = 0.0
    humidity_temp_loss = 0.0
    total_input = tx_power_dbm + antenna_gain_db * 2.0
    rx_dbm = total_input - fspl - obs_loss - shadow_loss - diffraction_loss - terrain_loss - weather_loss - humidity_temp_loss
    margin = rx_dbm - receiver_sensitivity_dbm

    losses = [
        ("FSPL", fspl),
        ("Obstacle", obs_loss),
        ("Terrain\nshadow", shadow_loss),
        ("Diffraction", diffraction_loss),
        ("Terrain\nscatter", terrain_loss),
        ("Weather/\nhumidity", weather_loss + humidity_temp_loss),
    ]
    labels = ["Ptx+Gain"] + [x[0] for x in losses] + ["RSSI"]
    fig = plt.figure(figsize=(11.2, 6.2))
    fig.text(0.075, 0.965, "Waterfall Link Budget LoRa", fontsize=13.2, fontweight="bold", color=COL["navy"], ha="left", va="top")
    fig.text(
        0.075,
        0.925,
        "Nilai komponen dihitung dari model simulasi pada satu contoh link relay.",
        fontsize=8.4,
        color="#536471",
        ha="left",
        va="top",
    )
    gs = fig.add_gridspec(1, 2, width_ratios=[4.7, 1.45], wspace=0.16, left=0.075, right=0.965, bottom=0.12, top=0.84)
    ax = fig.add_subplot(gs[0, 0])
    ax_info = fig.add_subplot(gs[0, 1])
    ax_info.axis("off")

    x = np.arange(len(labels))
    pos_color = "#2f6f4e"
    loss_color = "#9b3d3d"
    final_color = "#2f5f8f"
    ax.bar(x[0], total_input, bottom=0, color=pos_color, width=0.58)
    ax.text(x[0], total_input + 2.5, f"{total_input:.1f}", ha="center", fontsize=7.5, fontweight="bold", color=pos_color)
    running = total_input
    for i, (_, loss) in enumerate(losses, start=1):
        if loss > 0:
            ax.bar(x[i], -loss, bottom=running, color=loss_color, width=0.58, alpha=0.88)
            ax.text(x[i], running - loss / 2, f"-{loss:.1f}", ha="center", va="center", fontsize=7.2, color="white")
        else:
            ax.bar(x[i], 0.01, bottom=running, color="#cbd5e0", width=0.58)
            ax.text(x[i], running + 2.0, "0.0", ha="center", fontsize=7.0, color="#4a5568")
        ax.plot([x[i - 1] + 0.29, x[i] - 0.29], [running, running], color="#8b98a8", lw=0.9)
        running -= loss
    ax.bar(x[-1], rx_dbm, color=final_color, width=0.58)
    ax.axhline(receiver_sensitivity_dbm, color="#4a5568", lw=1.3, ls="--", label="Sensitivitas SF9")
    ax.text(
        x[-1],
        -108,
        f"RSSI {rx_dbm:.1f} dBm\nmargin {margin:.1f} dB",
        ha="center",
        va="center",
        color=final_color,
        fontweight="bold",
        fontsize=8,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=final_color, lw=0.8, alpha=0.94),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Daya / loss (dB, dBm)")
    ax.set_ylim(-140, 35)
    ax.grid(axis="y", color="#d9dee6", alpha=0.65, linewidth=0.55)
    ax.set_axisbelow(True)
    ax.legend(loc="lower left", fontsize=7.8, frameon=True)
    ax.set_title("")

    ax_info.add_patch(patches.Rectangle((0.03, 0.04), 0.94, 0.92, fc="#f8fafc", ec="#cbd5e0", lw=0.9))
    ax_info.text(0.10, 0.91, "Ringkasan Link", fontsize=9.2, fontweight="bold", color=COL["navy"], transform=ax_info.transAxes)
    obstacle_text = ",\n".join(name.replace("_", " ") for name in crossed)
    summary = [
        ("Dari", left.name.replace("node_", "")),
        ("Ke", right.name.replace("node_", "")),
        ("Jarak", f"{distance_m:.0f} m"),
        ("Cuaca", weather),
        ("Obstacle", obstacle_text),
        ("RSSI", f"{rx_dbm:.1f} dBm"),
        ("Margin", f"{margin:.1f} dB"),
    ]
    y0 = 0.82
    for label, value in summary:
        ax_info.text(0.10, y0, label, fontsize=7.3, fontweight="bold", color="#4a5568", transform=ax_info.transAxes)
        ax_info.text(0.10, y0 - 0.045, value, fontsize=7.15, color="#1f2933", transform=ax_info.transAxes, wrap=True)
        y0 -= 0.13 if label == "Obstacle" else 0.105
    ax_info.plot([0.10, 0.90], [0.105, 0.105], color="#cbd5e0", lw=0.8, transform=ax_info.transAxes)
    ax_info.text(
        0.10,
        0.060,
        r"$P_{rx}=P_{tx}+G_{tx}+G_{rx}-L_{total}$",
        fontsize=7.0,
        color=COL["navy"],
        transform=ax_info.transAxes,
    )
    save(fig, "codex_fig10_08_link_budget.png")


def fig09_routing_multihop():
    fig, ax = plt.subplots(figsize=(10.2, 7.0))
    x, y, xx, yy, zz = terrain_grid(120)
    ax.contourf(xx, yy, zz, levels=20, cmap="YlGnBu", alpha=0.25)
    plot_routes_nodes(ax, labels=True)
    stations = [scenario.BASE_STATION] + scenario.LORA_NODES
    for i, a in enumerate(stations):
        for b in stations[i + 1:]:
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d < 95:
                ax.plot([a.x, b.x], [a.y, b.y], color="#a0aec0", lw=0.7, alpha=0.45)
    route = [
        scenario.BASE_STATION,
        scenario.LORA_NODES[0],
        scenario.LORA_NODES[3],
        scenario.LORA_NODES[5],
        scenario.LORA_NODES[6],
        scenario.LORA_NODES[7],
    ]
    for a, b in zip(route[:-1], route[1:]):
        ax.plot([a.x, b.x], [a.y, b.y], color=COL["red"], lw=3.2, alpha=0.92)
    ax.text(-72, 73, "edge abu-abu = kandidat link\nmerah = contoh route terpilih", fontsize=8,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cbd5e0"))
    ax.set_aspect("equal")
    ax.set_xlabel("X world")
    ax.set_ylabel("Y world")
    title(ax, "Graf Routing Multi-Hop", "Dijkstra memilih jalur layak menuju base station berdasarkan link budget deterministik.")
    save(fig, "codex_fig10_09_routing_multihop.png")


def lora_toa_ms(sf: int, bw: float = 125000.0, payload: int = 20) -> float:
    tsym = (2 ** sf) / bw
    tpreamble = (8 + 4.25) * tsym
    npayload = max(8, math.ceil((8 * payload - 4 * sf + 28 + 16) / (4 * sf)) * 5)
    return (tpreamble + npayload * tsym) * 1000


def fig10_sf_tradeoff():
    sfs = np.arange(7, 13)
    sensitivity = np.array([-123, -126, -129, -132, -134.5, -136])
    toa = np.array([lora_toa_ms(int(sf)) for sf in sfs])
    fig, ax1 = plt.subplots(figsize=(9.5, 5.0))
    ax1.bar(sfs - 0.18, -sensitivity, width=0.35, color=COL["blue"], alpha=0.82, label="sensitivitas (abs)")
    ax1.set_ylabel("|Sensitivitas| (dB)")
    ax1.set_xlabel("Spreading Factor")
    ax1.grid(axis="y", alpha=0.18)
    ax2 = ax1.twinx()
    ax2.plot(sfs, toa, color=COL["orange"], lw=2.5, marker="o", label="Time on Air")
    ax2.set_ylabel("Time on Air (ms)")
    for sf, t in zip(sfs, toa):
        ax2.text(sf, t + 12, f"{t:.0f}", ha="center", fontsize=7, color=COL["orange"])
    title(ax1, "Trade-off Spreading Factor LoRa", "SF tinggi memperbaiki sensitivitas tetapi memperpanjang time on air.")
    save(fig, "codex_fig10_10_sf_tradeoff.png")


def fig11_per_pdr():
    margin = np.linspace(-15, 20, 300)
    per = 1 / (1 + np.exp(0.8 * (margin - 2)))
    hop_counts = [1, 2, 3, 4]
    per_link = np.linspace(0, 0.8, 200)
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8))
    axes[0].plot(margin, per * 100, color=COL["red"], lw=2.5)
    axes[0].axvline(0, color=COL["gray"], ls="--", lw=1)
    axes[0].set_xlabel("Link margin (dB)")
    axes[0].set_ylabel("PER (%)")
    axes[0].set_title("PER sigmoid terhadap margin", fontweight="bold", color=COL["navy"])
    axes[0].grid(True, alpha=0.22)
    for h in hop_counts:
        pdr = ((1 - per_link) ** h) * 100
        axes[1].plot(per_link * 100, pdr, lw=2, label=f"{h} hop")
    axes[1].set_xlabel("PER per hop (%)")
    axes[1].set_ylabel("PDR end-to-end (%)")
    axes[1].set_title("Akumulasi risiko pada multi-hop", fontweight="bold", color=COL["navy"])
    axes[1].grid(True, alpha=0.22)
    axes[1].legend()
    fig.suptitle("Model PER dan PDR End-to-End", x=0.02, ha="left", fontweight="bold", color=COL["navy"])
    save(fig, "codex_fig10_11_per_pdr.png")


def fig12_drop_reason_tree():
    fig, ax = plt.subplots(figsize=(13.0, 5.8))
    ax.set_xlim(0, 15.1)
    ax.set_ylim(0, 5.8)
    ax.axis("off")
    title(ax, "Pohon Keputusan Drop Reason", "Setiap paket gagal dicatat dengan penyebab teknis yang spesifik.")
    y_main = 3.9
    main = [
        ("Paket\nLoRa", 0.8, "#edf2f7"),
        ("Route\nvalid?", 2.7, "#ebf8ff"),
        ("Duty cycle\naman?", 4.6, "#fefcbf"),
        ("Collision?", 6.5, "#fff5f5"),
        ("PER\nmodel?", 8.4, "#fff5f5"),
        ("Cuaca\nburuk?", 10.3, "#f0fff4"),
        ("Protocol\nOK?", 12.2, "#faf5ff"),
        ("DELIVERED", 13.95, "#e6fffa"),
    ]
    centers = {}
    for text, x, fc in main:
        w = 1.35 if text != "DELIVERED" else 1.25
        box(ax, (x - w / 2, y_main - 0.42), w, 0.84, text, fc, COL["line"], fs=7.8)
        centers[text] = (x, y_main)
    for (_, x1, _), (_, x2, _) in zip(main[:-1], main[1:]):
        arrow(ax, (x1 + 0.68, y_main), (x2 - 0.68, y_main), "#566573", lw=1.05)

    drops = [
        ("no_route", 2.7),
        ("duty_cycle", 4.6),
        ("collision", 6.5),
        ("per_model", 8.4),
        ("weather", 10.3),
        ("protocol", 12.2),
    ]
    for reason, x in drops:
        box(ax, (x - 0.72, 1.35), 1.44, 0.72, reason, "#fed7d7", COL["red"], fs=7.8)
        arrow(ax, (x, y_main - 0.44), (x, 2.07), COL["red"], lw=1.0)
    ax.text(7.0, 2.55, "Jika syarat gagal, paket diberi drop_reason spesifik; jika lolos semua, paket dianggap terkirim.",
            ha="center", fontsize=8.2, color="#536471")
    save(fig, "codex_fig10_12_drop_reason_tree.png")


def fig13_metrics_summary():
    fig, ax = plt.subplots(figsize=(12.0, 6.8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")

    ax.text(0.0, 6.75, "Metrik Evaluasi Simulasi siLacak", fontsize=15, fontweight="bold", color=COL["navy"], va="top")
    ax.text(
        0.0,
        6.38,
        "Ringkasan ukuran yang digunakan untuk menilai akurasi lokasi, kualitas komunikasi, dan operasi perangkat.",
        fontsize=9,
        color="#536471",
        va="top",
    )

    x0, y0 = 0.25, 5.55
    table_w = 11.5
    header_h = 0.55
    row_h = 0.88
    col_w = [2.15, 3.05, 2.35, 3.95]
    headers = ["Aspek Evaluasi", "Metrik Utama", "Sumber Data", "Makna Evaluasi"]
    rows = [
        (
            "Akurasi lokasi",
            "GPS error, DOP, multipath, delay, drift",
            "positions.csv",
            "Menilai seberapa jauh koordinat GPS sintetis menyimpang dari posisi terrain sebenarnya.",
        ),
        (
            "Kualitas link LoRa",
            "RSSI, SNR, link margin",
            "network_events.csv",
            "Menilai apakah daya terima masih berada di atas sensitivitas receiver.",
        ),
        (
            "Keandalan paket",
            "PDR, PER, drop_reason",
            "network_events.csv",
            "Menilai rasio paket yang berhasil sampai dan penyebab teknis paket yang gagal.",
        ),
        (
            "Routing multi-hop",
            "hop count, route aktif, latency",
            "network_events.csv",
            "Menilai kemampuan sistem memilih relay dan meneruskan paket menuju base station.",
        ),
        (
            "Operasi perangkat",
            "SoC baterai, TX power, ToA, duty cycle",
            "positions.csv dan network_events.csv",
            "Menilai keberlanjutan operasi perangkat dan beban kanal LoRa.",
        ),
    ]

    header_color = COL["navy"]
    grid_color = "#cbd5e0"
    text_color = "#24313d"

    cx = x0
    for i, (head, width) in enumerate(zip(headers, col_w)):
        ax.add_patch(patches.Rectangle((cx, y0), width, header_h, fc=header_color, ec=header_color, lw=0.9))
        ax.text(cx + 0.12, y0 + header_h / 2, head, ha="left", va="center", fontsize=8.0, fontweight="bold", color="white")
        cx += width

    y = y0 - row_h
    for r, row in enumerate(rows):
        fill_color = "#ffffff" if r % 2 == 0 else "#f8fafc"
        cx = x0
        for c, (cell, width) in enumerate(zip(row, col_w)):
            ax.add_patch(patches.Rectangle((cx, y), width, row_h, fc=fill_color, ec=grid_color, lw=0.75))
            wrap_width = [18, 30, 22, 44][c]
            fs = 7.6 if c != 3 else 7.35
            weight = "bold" if c == 0 else "normal"
            color = COL["navy"] if c == 0 else text_color
            ax.text(cx + 0.12, y + row_h / 2, fill(str(cell), wrap_width), ha="left", va="center", fontsize=fs, fontweight=weight, color=color)
            cx += width
        y -= row_h

    ax.add_patch(patches.Rectangle((x0, y0 - row_h * len(rows)), table_w, row_h * len(rows) + header_h, fc="none", ec="#94a3b8", lw=1.0))

    ax.add_patch(patches.Rectangle((0.25, 0.35), 11.5, 0.62, fc="#eef6ff", ec="#b7c7d9", lw=0.85))
    ax.text(
        0.45,
        0.66,
        "Interpretasi: PoC dinilai layak apabila alur data berjalan, PDR dapat dihitung, drop_reason tercatat, dan metrik GPS/radio/baterai tersedia untuk analisis.",
        ha="left",
        va="center",
        fontsize=8.0,
        color=COL["navy"],
    )
    save(fig, "codex_fig10_13_metrics_summary.png")


def fig14_scenario_matrix():
    routes = ["Ridge", "Valley", "Crater"]
    weather = ["Clear", "Fog", "Light rain", "Heavy rain", "Thunderstorm"]
    fig, ax = plt.subplots(figsize=(9.6, 5.5))
    data = np.array([
        [1.0, 0.88, 0.80, 0.67, 0.52],
        [0.76, 0.66, 0.58, 0.43, 0.30],
        [0.84, 0.73, 0.64, 0.50, 0.37],
    ])
    im = ax.imshow(data, cmap="RdYlGn", vmin=0, vmax=1)
    ax.set_xticks(np.arange(len(weather)), labels=weather, rotation=25, ha="right")
    ax.set_yticks(np.arange(len(routes)), labels=routes)
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            ax.text(j, i, f"{data[i, j]:.2f}", ha="center", va="center", color="#1a202c", fontsize=8)
    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.03)
    cb.set_label("indikator kelayakan relatif")
    title(ax, "Matriks Skenario Pengujian", "Rute dan cuaca memberi tingkat kesulitan komunikasi yang berbeda.")
    save(fig, "codex_fig10_14_scenario_matrix.png")


def fig15_weather_impact():
    weather = ["clear", "fog", "light rain", "heavy rain", "thunderstorm"]
    attenuation = np.array([0.0, 0.01, 0.03, 0.06, 0.10])
    wet = np.array([1.0, 1.05, 1.25, 1.70, 2.0])
    speed = np.array([1.0, 0.85, 0.80, 0.65, 0.50])
    drop = np.array([0, 1, 2, 5, 15])
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 6.8))
    axes = axes.ravel()
    bars = [
        (attenuation, "Atenuasi tambahan (dB/km)", COL["blue"]),
        (wet, "Wet factor vegetasi", COL["green"]),
        (speed, "Faktor kecepatan pendaki", COL["orange"]),
        (drop, "Extra drop probability (%)", COL["red"]),
    ]
    for ax, (values, ylabel, color) in zip(axes, bars):
        ax.bar(weather, values, color=color, alpha=0.82)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", rotation=25)
        ax.grid(axis="y", alpha=0.18)
    fig.suptitle("Pengaruh Profil Cuaca pada Simulasi", x=0.02, ha="left", fontweight="bold", color=COL["navy"])
    save(fig, "codex_fig10_15_weather_impact.png")


def fig16_poc_pdr():
    labels = ["Ridge\nclear", "Valley\nclear", "Crater\nclear", "Ridge\nrain", "Valley\nrain", "Crater\nrain"]
    pdr = np.array([91, 78, 84, 83, 59, 68])
    fig, ax = plt.subplots(figsize=(9.6, 5.0))
    colors = [COL["green"] if v >= 80 else COL["orange"] if v >= 65 else COL["red"] for v in pdr]
    ax.bar(labels, pdr, color=colors, alpha=0.9)
    ax.axhline(80, color=COL["blue"], ls="--", lw=1.5, label="target evaluasi awal 80%")
    for i, v in enumerate(pdr):
        ax.text(i, v + 2, f"{v}%", ha="center", fontweight="bold")
    ax.set_ylim(0, 105)
    ax.set_ylabel("PDR model (%)")
    ax.legend()
    ax.grid(axis="y", alpha=0.18)
    title(ax, "Ringkasan PDR PoC dari Model Simulasi", "Angka adalah ilustrasi evaluasi model, bukan pengukuran RF lapangan.")
    save(fig, "codex_fig10_16_poc_pdr.png")


def fig17_drop_reason_heatmap():
    scenarios = ["Ridge\nclear", "Valley\nclear", "Crater\nclear", "Valley\nrain", "Multi\nhiker"]
    reasons = ["no_route", "per_model", "collision", "duty_cycle", "weather", "protocol"]
    data = np.array([
        [2, 16, 7, 0, 0, 1],
        [8, 27, 9, 0, 0, 1],
        [6, 24, 8, 0, 0, 1],
        [14, 36, 10, 0, 7, 2],
        [4, 18, 22, 9, 1, 2],
    ])
    fig, ax = plt.subplots(figsize=(10.0, 5.2))
    im = ax.imshow(data, cmap="YlOrRd")
    ax.set_xticks(np.arange(len(reasons)), labels=reasons, rotation=25, ha="right")
    ax.set_yticks(np.arange(len(scenarios)), labels=scenarios)
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            ax.text(j, i, str(data[i, j]), ha="center", va="center", fontsize=8)
    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.03)
    cb.set_label("jumlah paket gagal")
    title(ax, "Heatmap Drop Reason", "Visual ini memisahkan penyebab kegagalan paket per skenario.")
    save(fig, "codex_fig10_17_drop_reason_heatmap.png")


def fig18_metrics_radar():
    labels = ["GPS", "PDR", "Routing", "Energi", "Cuaca", "Logging"]
    vals = np.array([0.82, 0.76, 0.88, 0.70, 0.63, 0.92])
    angles = np.linspace(0, 2 * np.pi, len(labels), endpoint=False)
    vals_closed = np.r_[vals, vals[0]]
    angles_closed = np.r_[angles, angles[0]]
    fig = plt.figure(figsize=(7.2, 6.4))
    ax = fig.add_subplot(111, polar=True)
    ax.plot(angles_closed, vals_closed, color=COL["blue"], lw=2.4)
    ax.fill(angles_closed, vals_closed, color=COL["sky"], alpha=0.35)
    ax.set_xticks(angles, labels)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_ylim(0, 1)
    ax.grid(True, alpha=0.3)
    ax.set_title("Ringkasan Kelayakan PoC Simulasi", loc="left", fontweight="bold", color=COL["navy"], pad=18)
    save(fig, "codex_fig10_18_metrics_radar.png")


def main() -> None:
    generators = [
        fig01_architecture,
        fig02_topology,
        fig03_environment_map,
        fig04_los_diffraction,
        fig05_network_3d,
        fig06_gps_pipeline,
        fig07_battery_tx,
        fig08_link_budget,
        fig09_routing_multihop,
        fig10_sf_tradeoff,
        fig11_per_pdr,
        fig12_drop_reason_tree,
        fig13_metrics_summary,
        fig14_scenario_matrix,
        fig15_weather_impact,
        fig16_poc_pdr,
        fig17_drop_reason_heatmap,
        fig18_metrics_radar,
    ]
    for gen in generators:
        gen()
    print(f"Generated {len(generators)} figures in {OUT}")


if __name__ == "__main__":
    main()
