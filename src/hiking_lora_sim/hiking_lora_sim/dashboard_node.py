"""Dashboard terminal real-time untuk simulasi Hiking LoRa."""

import json
import os
import sys
import time
from collections import deque
from typing import Deque, Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hiking_lora_sim.scenario import trail_length, TRAILS

_RESET = "\033[0m"
_BOLD = "\033[1m"
_GREEN = "\033[32m"
_RED = "\033[31m"
_YELLOW = "\033[33m"
_CYAN = "\033[36m"
_BLUE = "\033[34m"
_WHITE = "\033[37m"
_CLEAR_SCREEN = "\033[2J\033[H"

_WEATHER_ICON = {
    "clear": "Cerah",
    "fog": "Kabut",
    "light_rain": "Hujan Ringan",
    "heavy_rain": "Hujan Lebat",
    "thunderstorm": "Badai Petir",
}

_ROLLING_WINDOW = 50


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard")

        self.declare_parameter("refresh_rate_hz", 0.5)

        refresh_hz = float(self.get_parameter("refresh_rate_hz").value)

        self._events: Deque[Dict] = deque(maxlen=_ROLLING_WINDOW)
        self._last_event: Optional[Dict] = None
        self._last_battery: Optional[Dict] = None
        self._last_batteries: Dict[str, Dict] = {}
        self._last_statuses: Dict[str, Dict] = {}
        self._start_time = time.monotonic()
        self._total_packets = 0
        self._delivered_packets = 0

        self.create_subscription(String, "/lora/network_event", self._on_event, 10)
        self.create_subscription(String, "/hiker/battery", self._on_battery, 10)
        self.create_subscription(String, "/hikers/battery", self._on_battery, 10)
        self.create_subscription(String, "/hikers/status", self._on_status, 10)

        self._timer = self.create_timer(1.0 / max(0.1, refresh_hz), self._render)

        self.get_logger().info("Dashboard terminal aktif.")

    def _on_event(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._events.append(data)
        self._last_event = data
        self._total_packets += 1
        if data.get("delivered"):
            self._delivered_packets += 1

    def _on_battery(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        hiker_id = str(data.get("hiker_id", "hiker"))
        self._last_battery = data
        self._last_batteries[hiker_id] = data

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        hiker_id = str(data.get("hiker_id", "hiker"))
        self._last_statuses[hiker_id] = data

    def _render(self) -> None:
        if not sys.stdout.isatty():
            return

        elapsed_s = int(time.monotonic() - self._start_time)
        hh = elapsed_s // 3600
        mm = (elapsed_s % 3600) // 60
        ss = elapsed_s % 60

        lines = []
        w = 64

        def hr(char="─"):
            return char * w

        lines.append(_CLEAR_SCREEN)
        lines.append(f"{_BOLD}{_CYAN}{'═' * w}{_RESET}")
        lines.append(f"{_BOLD}{_CYAN}  SIMULASI HIKING LoRa — DASHBOARD{_RESET}")
        lines.append(f"{_CYAN}{'═' * w}{_RESET}")

        # Waktu & info hiker
        ev = self._last_event
        hiker_id = "hiker"
        trail_name = "—"
        progress_pct = 0.0
        lat_str = lon_str = alt_str = "—"
        sf = "—"
        toa = "—"
        dr = "—"
        weather = "—"

        if ev:
            hiker_id = str(ev.get("hiker_id", "hiker"))
            trail_name = ev.get("entry_node", "—") or "—"
            sf_val = ev.get("spreading_factor")
            if sf_val:
                sf = str(sf_val)
            toa_val = ev.get("time_on_air_ms")
            if toa_val:
                toa = f"{toa_val:.0f} ms"
            dr_val = ev.get("data_rate_bps")
            if dr_val:
                dr = f"{dr_val:.0f} bps"
            weather_raw = ev.get("weather", "clear")
            weather = _WEATHER_ICON.get(weather_raw, weather_raw)
            gps = ev.get("gps", {})
            if gps:
                lat_str = f"{gps.get('lat', 0):.6f}"
                lon_str = f"{gps.get('lon', 0):.6f}"
                alt_str = f"{gps.get('alt_m', 0):.0f} m"
            local = ev.get("hiker_local", {})
            if local:
                for tname, tpoints in TRAILS.items():
                    total = trail_length(tpoints)
                    if total > 0:
                        trail_name = tname
                        break

        bat = self._last_batteries.get(hiker_id) or self._last_battery
        bat_str = "—"
        bat_color = _WHITE
        if bat:
            pct = bat.get("percentage", 100.0)
            rem = bat.get("remaining_mah", 0.0)
            bat_str = f"{pct:.1f}% ({rem:.0f} mAh sisa)"
            bat_color = _GREEN if pct > 50 else (_YELLOW if pct > 20 else _RED)

        lines.append(f"  Waktu      : {_BOLD}{hh:02d}:{mm:02d}:{ss:02d}{_RESET}")
        lines.append(f"  Pendaki    : {_BOLD}{hiker_id}{_RESET}")
        lines.append(f"  GPS        : lat={lat_str}  lon={lon_str}  alt={alt_str}")
        lines.append(f"  Baterai    : {bat_color}{bat_str}{_RESET}")
        lines.append(f"  Cuaca      : {weather}")
        lines.append(f"  SF / ToA   : SF{sf}  {toa}  ({dr})")
        lines.append(hr())

        # Statistik jaringan (rolling window)
        lines.append(f"{_BOLD}  STATISTIK JARINGAN (terakhir {_ROLLING_WINDOW} paket){_RESET}")
        window = list(self._events)
        total_w = len(window)
        deliv_w = sum(1 for e in window if e.get("delivered"))
        drop_w = total_w - deliv_w
        rate_w = (deliv_w / total_w * 100.0) if total_w > 0 else 0.0
        rate_color = _GREEN if rate_w >= 80 else (_YELLOW if rate_w >= 50 else _RED)

        lines.append(f"  Total (sesi)     : {self._total_packets}")
        lines.append(f"  Terkirim (window): {_BOLD}{rate_color}{deliv_w}/{total_w}  ({rate_w:.1f}%){_RESET}")
        lines.append(f"  Gagal    (window): {drop_w}")
        if self._last_batteries:
            active = []
            for hid in sorted(self._last_batteries):
                pct = self._last_batteries[hid].get("percentage", 0.0)
                active.append(f"{hid}:{pct:.0f}%")
            lines.append(f"  Pendaki aktif     : {', '.join(active[:5])}")
            if len(active) > 5:
                lines.append(f"                      +{len(active) - 5} pendaki lain")
        lines.append(hr())

        # Route aktif
        lines.append(f"{_BOLD}  ROUTE AKTIF{_RESET}")
        if ev and ev.get("delivered") and ev.get("links"):
            route = ev.get("route", [])
            links = ev.get("links", [])

            route_str = " → ".join(route)
            lines.append(f"  {_GREEN}{route_str}{_RESET}")
            lines.append("")

            for link in links:
                src = link.get("from", "?")
                dst = link.get("to", "?")
                dist = link.get("distance_m", 0.0)
                margin = link.get("margin_db", 0.0)
                fading = link.get("fading_type", "—")
                weather_loss = link.get("weather_loss_db", 0.0)
                margin_color = _GREEN if margin >= 10 else (_YELLOW if margin >= 0 else _RED)

                lines.append(
                    f"  {src} → {dst}"
                )
                lines.append(
                    f"    Jarak: {dist:.0f}m  "
                    f"Margin: {margin_color}{margin:+.1f} dB{_RESET}  "
                    f"Fading: {fading}  "
                    f"Cuaca loss: {weather_loss:.1f} dB"
                )
        elif ev and not ev.get("delivered"):
            lines.append(f"  {_RED}Paket terakhir GAGAL dikirim{_RESET}")
        else:
            lines.append("  Menunggu data...")
        lines.append(hr())

        # Baterai detail
        if bat:
            lines.append(f"{_BOLD}  BATERAI PERANGKAT{_RESET}")
            lines.append(
                f"  Terpakai  : {bat.get('used_mah', 0):.1f} mAh  "
                f"Energi: {bat.get('total_energy_mj', 0):.0f} mJ"
            )
            lines.append(
                f"  Tx Count  : {bat.get('tx_count', 0)}  "
                f"Est. sisa: {bat.get('hours_remaining', 0):.1f} jam"
            )
            lines.append(hr())

        lines.append(f"{_CYAN}{'═' * w}{_RESET}")
        lines.append(f"  Tekan Ctrl+C untuk keluar")

        print("\n".join(lines), flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Bersihkan layar sebelum keluar
        if sys.stdout.isatty():
            print(_RESET, end="", flush=True)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
