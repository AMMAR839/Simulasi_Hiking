"""
Data logger node — subscribe ke topik ROS yang sudah ada, export ke 2 file CSV:
  - positions.csv      : posisi hiker per-tick (ground truth + GPS berderau)
  - network_events.csv : event LoRa per-paket (RSSI, SNR, jarak, SF, dll.)

Jalankan bersamaan simulasi:
  ros2 run hiking_lora_sim data_logger

File CSV tersimpan di ~/sim_logs/sim_<timestamp>/
Setelah simulasi selesai, analisis offline dengan: python tools/analysis_error.py
"""
import csv
import json
import os
import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Regex untuk parse format teks /hiker/status
# Format: "hiker1 track=trail1 x=12.34 y=56.78 terrain_z=0.10 alt=100.0m
#          gps=(-6.8914700,107.6106600) bat=95.0% dop=1.20 gps_err=2.3m
#          drift=+0.1ms hw_delay=5ms weather=clear speed_factor=1.00 ..."
_STATUS_RE = re.compile(
    r"(?P<hiker_id>\S+)\s+track=(?P<trail>\S+)\s+"
    r"x=(?P<x>[+-]?[\d.]+)\s+y=(?P<y>[+-]?[\d.]+)\s+"
    r"terrain_z=(?P<terrain_z>[+-]?[\d.]+)\s+alt=(?P<altitude>[\d.]+)m\s+"
    r"gps=\((?P<gps_lat>[+-]?[\d.]+),(?P<gps_lon>[+-]?[\d.]+)\)\s+"
    r"bat=(?P<battery>[\d.]+)%\s+"
    r"dop=(?P<dop>[\d.]+)\s+gps_err=(?P<gps_err>[\d.]+)m\s+"
    r"drift=(?P<drift>[+-]?[\d.]+)ms\s+hw_delay=(?P<hw_delay>[\d.]+)ms\s+"
    r"weather=(?P<weather>\S+)"
)

_POS_FIELDS = [
    "timestamp_s", "hiker_id", "trail",
    "true_x_wu", "true_y_wu", "terrain_z_wu", "altitude_m",
    "gps_lat", "gps_lon",
    "gps_error_m", "dop",
    "drift_ms", "hw_delay_ms",
    "weather",
]

_NET_FIELDS = [
    "timestamp_s", "hiker_id",
    "delivered", "drop_reason",
    # Posisi hiker (ground truth dalam world units)
    "true_x_wu", "true_y_wu", "true_alt_m",
    # GPS payload yang dikirim dalam paket LoRa (sudah berderau)
    "gps_lat", "gps_lon", "gps_alt_m",
    # Link RF terbaik / terburuk
    "distance_m", "rx_dbm", "snr_db", "margin_db",
    "obstacle_loss_db", "terrain_shadow_db", "diffraction_db",
    "terrain_scatter_db",
    "fading_db", "fading_type", "weather_loss_db", "temp_noise_db",
    # LoRa parameter
    "spreading_factor", "data_rate_bps", "time_on_air_ms",
    "duty_cycle_pct",
    # Lingkungan
    "weather", "temperature_c", "humidity_pct",
    # Rute
    "route", "hop_count",
]


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("data_logger")

        # Buat direktori output
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.expanduser(f"~/sim_logs/sim_{ts}")
        os.makedirs(out_dir, exist_ok=True)
        self._out_dir = out_dir

        # Buka file CSV
        pos_path = os.path.join(out_dir, "positions.csv")
        net_path = os.path.join(out_dir, "network_events.csv")
        self._pos_file = open(pos_path, "w", newline="", encoding="utf-8")
        self._net_file = open(net_path, "w", newline="", encoding="utf-8")
        self._pos_writer = csv.DictWriter(self._pos_file, fieldnames=_POS_FIELDS)
        self._net_writer = csv.DictWriter(self._net_file, fieldnames=_NET_FIELDS)
        self._pos_writer.writeheader()
        self._net_writer.writeheader()

        # Subscribe ke status hiker — topik tunggal (/hiker/status)
        self.create_subscription(String, "/hiker/status", self._on_status, 10)

        # Subscribe ke aggregate multi-hiker (/hikers/status) — JSON dengan field "text"
        self.create_subscription(String, "/hikers/status", self._on_aggregate_status, 10)

        # Subscribe ke network event LoRa.
        # Single-hiker: event_pub → /lora/network_event
        # Multi-hiker: global_event_pub JUGA → /lora/network_event (topik yang sama)
        # Jadi cukup satu subscription.
        self.create_subscription(String, "/lora/network_event", self._on_event, 10)

        self._pos_count = 0
        self._net_count = 0
        self.create_timer(15.0, self._log_stats)

        self.get_logger().info(
            f"DataLogger aktif. Output → {out_dir}\n"
            f"  positions.csv      : data posisi hiker per-tick\n"
            f"  network_events.csv : event LoRa per-paket"
        )

    # ─── Status hiker (format teks) ────────────────────────────────────────
    def _on_status(self, msg: String) -> None:
        self._parse_and_write_status(msg.data)

    def _on_aggregate_status(self, msg: String) -> None:
        # Aggregate JSON mengandung field "text" yang isinya string status lengkap
        try:
            data = json.loads(msg.data)
            text = data.get("text", "")
            if text:
                self._parse_and_write_status(text)
        except (json.JSONDecodeError, AttributeError):
            pass

    def _parse_and_write_status(self, text: str) -> None:
        m = _STATUS_RE.search(text)
        if not m:
            return
        stamp = self.get_clock().now().nanoseconds / 1e9
        self._pos_writer.writerow({
            "timestamp_s":  f"{stamp:.3f}",
            "hiker_id":     m["hiker_id"],
            "trail":        m["trail"],
            "true_x_wu":    m["x"],
            "true_y_wu":    m["y"],
            "terrain_z_wu": m["terrain_z"],
            "altitude_m":   m["altitude"],
            "gps_lat":      m["gps_lat"],
            "gps_lon":      m["gps_lon"],
            "gps_error_m":  m["gps_err"],
            "dop":          m["dop"],
            "drift_ms":     m["drift"],
            "hw_delay_ms":  m["hw_delay"],
            "weather":      m["weather"],
        })
        self._pos_file.flush()
        self._pos_count += 1

    # ─── Network event LoRa ────────────────────────────────────────────────
    def _on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        stamp = ev.get("stamp", self.get_clock().now().nanoseconds / 1e9)
        links = ev.get("links", [])
        gps = ev.get("gps", {})
        hl = ev.get("hiker_local", {})
        protocol = ev  # hop_count ada di root event

        # Ambil link terburuk (bottleneck path) untuk satu baris representatif
        worst: dict = {}
        if links:
            worst = min(links, key=lambda lk: lk.get("margin_db", 999.0))

        self._net_writer.writerow({
            "timestamp_s":      f"{stamp:.3f}",
            "hiker_id":         ev.get("hiker_id", ""),
            "delivered":        ev.get("delivered", False),
            "drop_reason":      ev.get("drop_reason") or "",
            # Posisi ground truth (world units)
            "true_x_wu":        hl.get("x", ""),
            "true_y_wu":        hl.get("y", ""),
            "true_alt_m":       hl.get("alt_m", ""),
            # GPS berderau (isi paket LoRa yang diterima base station)
            "gps_lat":          gps.get("lat", ""),
            "gps_lon":          gps.get("lon", ""),
            "gps_alt_m":        gps.get("alt_m", ""),
            # RF metrics
            "distance_m":       worst.get("distance_m", ""),
            "rx_dbm":           worst.get("rx_dbm", ""),
            "snr_db":           ev.get("snr_db", ""),
            "margin_db":        worst.get("margin_db", ""),
            "obstacle_loss_db":  worst.get("obstacle_loss_db", ""),
            "terrain_shadow_db": worst.get("terrain_shadow_loss_db", ""),
            "diffraction_db":    worst.get("diffraction_loss_db", ""),
            "terrain_scatter_db": worst.get("terrain_loss_db", ""),
            "fading_db":         worst.get("fading_loss_db", ""),
            "fading_type":      worst.get("fading_type", ""),
            "weather_loss_db":  worst.get("weather_loss_db", ""),
            "temp_noise_db":    worst.get("temp_noise_db", ""),
            # LoRa parameter
            "spreading_factor": ev.get("spreading_factor", ""),
            "data_rate_bps":    ev.get("data_rate_bps", ""),
            "time_on_air_ms":   ev.get("time_on_air_ms", ""),
            "duty_cycle_pct":   ev.get("duty_cycle_used_pct", ""),
            # Lingkungan
            "weather":          ev.get("weather", ""),
            "temperature_c":    ev.get("temperature_c", ""),
            "humidity_pct":     ev.get("humidity_pct", ""),
            # Rute
            "route":            " -> ".join(ev.get("route", [])),
            "hop_count":        ev.get("hop_count", ""),
        })
        self._net_file.flush()
        self._net_count += 1

    # ─── Statistik periodik ────────────────────────────────────────────────
    def _log_stats(self) -> None:
        self.get_logger().info(
            f"Logger: {self._pos_count} baris posisi, {self._net_count} event jaringan tersimpan."
        )

    def destroy_node(self) -> None:
        self._pos_file.close()
        self._net_file.close()
        self.get_logger().info(
            f"DataLogger selesai.\n"
            f"  {self._pos_count} baris → positions.csv\n"
            f"  {self._net_count} baris → network_events.csv\n"
            f"  Direktori: {self._out_dir}\n"
            f"  Analisis: python tools/analysis_error.py "
            f"--pos {self._out_dir}/positions.csv "
            f"--net {self._out_dir}/network_events.csv"
        )
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()
