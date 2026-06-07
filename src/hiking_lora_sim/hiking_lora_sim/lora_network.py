import heapq
import json
import random
import time
from collections import deque
from dataclasses import asdict
from math import ceil, exp, hypot, log10, pi, sqrt
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from hiking_lora_sim.scenario import (
    BASE_STATION,
    LORA_NODES,
    RADIO_OBSTACLES,
    Station,
    load_scenario_yaml,
    local_to_gps,
    station_radio_altitude_m,
    terrain_altitude_m,
    terrain_height_world,
)

# ---------------------------------------------------------------------------
# Sensitivitas receiver per Spreading Factor — EByte E220-900T22D (LLCC68)
# BW=125 kHz, CR=4/5, f=915 MHz
# Sumber: EByte E220-900T22D Datasheet v1.0, Tabel Parameter RF
#         Semtech LLCC68 Datasheet DS.LLCC68.W.APP, Table 14
# ---------------------------------------------------------------------------
_SF_SENSITIVITY: Dict[int, float] = {
    7:  -131.0,   # LLCC68 @SF7,  BW=125 kHz, CR=4/5
    8:  -134.0,   # LLCC68 @SF8
    9:  -137.0,   # LLCC68 @SF9
    10: -139.0,   # LLCC68 @SF10
    11: -142.0,   # LLCC68 @SF11
    12: -148.0,   # LLCC68 @SF12  (Spesifikasi E220-900T22D)
}

_SF_DATA_RATE: Dict[int, float] = {
    7: 5468.75,
    8: 3125.0,
    9: 1757.8,
    10: 878.9,
    11: 476.6,
    12: 250.0,
}

# ---------------------------------------------------------------------------
# Profil cuaca
# ---------------------------------------------------------------------------
WEATHER_PROFILES: Dict[str, Dict] = {
    "clear":        {"attn_db_km": 0.00, "noise_db": 0.0,  "extra_drop_prob": 0.00},
    "fog":          {"attn_db_km": 0.00, "noise_db": 0.5,  "extra_drop_prob": 0.00},
    "light_rain":   {"attn_db_km": 0.01, "noise_db": 1.0,  "extra_drop_prob": 0.00},
    "heavy_rain":   {"attn_db_km": 0.05, "noise_db": 3.0,  "extra_drop_prob": 0.05},
    "thunderstorm": {"attn_db_km": 0.10, "noise_db": 8.0,  "extra_drop_prob": 0.15},
}

# ---------------------------------------------------------------------------
# Thermal noise floor: kTB + NF
# -174 dBm/Hz (300K) + 10·log10(125000 Hz) + 6 dB NF ≈ −117 dBm
# ---------------------------------------------------------------------------
_THERMAL_NOISE_DBM: float = -174.0 + 10.0 * log10(125_000) + 6.0

# ---------------------------------------------------------------------------
# Wet-foliage multiplier (daun basah lebih menyerap sinyal)
# ---------------------------------------------------------------------------
_WET_FOLIAGE_FACTOR: Dict[str, float] = {
    "clear":        1.00,
    "fog":          1.15,
    "light_rain":   1.40,
    "heavy_rain":   1.70,
    "thunderstorm": 2.00,
}

# ---------------------------------------------------------------------------
# Duty-cycle (ITU regulasi, sub-GHz 1% / jam)
# ---------------------------------------------------------------------------
_DUTY_CYCLE_LIMIT  = 0.01          # 1 %
_DUTY_CYCLE_WINDOW_S = 3600.0      # jendela 1 jam

# ---------------------------------------------------------------------------
# Protokol paket
# ---------------------------------------------------------------------------
_MAX_HOP_COUNT = 6
_TTL_SECONDS   = 30.0   # TTL paket sebelum expired (konseptual)

# ---------------------------------------------------------------------------
# Hardware delay relay (ms)
# ---------------------------------------------------------------------------
_RELAY_HW_DELAY_MS = (80.0, 180.0)
_BASE_HW_DELAY_MS  = (20.0,  60.0)
_QUEUE_DELAY_MEAN_MS = 50.0   # mean queue wait (exponential distribution)
_QUEUE_DELAY_MAX_MS  = 300.0  # cap

# Probabilitas collision channel (0.5% base; meningkat jika banyak node)
_BASE_COLLISION_PROB = 0.005


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def _lora_time_on_air_ms(sf: int, payload_bytes: int = 20) -> float:
    """Estimasi Time on Air LoRa (ms), BW=125kHz, CR=4/5, header eksplisit."""
    bw = 125_000.0
    t_sym = (2 ** sf) / bw
    t_preamble = (8 + 4.25) * t_sym
    n_payload = max(8, ceil((8 * payload_bytes - 4 * sf + 28 + 16) / (4 * sf)) * 5)
    return (t_preamble + n_payload * t_sym) * 1000.0


def _per_from_margin(margin_db: float) -> float:
    """
    Packet Error Rate (PER) dari link margin menggunakan fungsi logistik.
    PER=50% pada margin=0 dB (tepat di ambang sensitivitas),
    PER≈5% pada margin=6 dB, PER≈1% pada margin=10 dB.
    """
    return 1.0 / (1.0 + exp(0.8 * (margin_db - 2.0)))


def _chord_length_m(
    ax: float, ay: float, bx: float, by: float,
    cx: float, cy: float, radius: float,
    meters_per_world_unit: float,
) -> float:
    """
    Panjang tali busur jalur sinyal yang melewati obstacle circle (dalam meter).
    Digunakan untuk menghitung kedalaman penetrasi vegetasi.
    """
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        d = hypot(ax - cx, ay - cy)
        if d > radius:
            return 0.0
        return 2.0 * sqrt(max(0.0, radius * radius - d * d)) * meters_per_world_unit

    t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / length_sq))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    d = hypot(closest_x - cx, closest_y - cy)
    if d >= radius:
        return 0.0
    return 2.0 * sqrt(max(0.0, radius * radius - d * d)) * meters_per_world_unit


# ---------------------------------------------------------------------------
# LoraNetwork node
# ---------------------------------------------------------------------------

class LoraNetwork(Node):
    def __init__(self) -> None:
        super().__init__("lora_network")

        self.declare_parameter("meters_per_world_unit", 35.0)
        self.declare_parameter("frequency_mhz", 923.0)
        # EByte E220-900T22D: TX max 22 dBm (Ref: EByte datasheet v1.0 §3.1)
        self.declare_parameter("tx_power_dbm", 22.0)
        # Antena SMA female 5 dBi (spesifikasi komponen hardware proyek)
        self.declare_parameter("antenna_gain_db", 5.0)
        # Terrain scatter (ground bounce, hanya untuk LOS — NLOS sudah di shadow model)
        # 1.0 dB/km: sesuai ITU-R P.452 rural sub-GHz (sebelumnya 2.5 → bias berlebihan)
        self.declare_parameter("terrain_loss_db_per_km", 1.0)
        self.declare_parameter("reference_lat", -6.89148)
        self.declare_parameter("reference_lon", 107.61066)
        self.declare_parameter("hiker_id", "hiker")
        self.declare_parameter("topic_prefix", "")
        self.declare_parameter("publish_global_events", True)
        self.declare_parameter("active_hiker_count", 1)
        # SF
        self.declare_parameter("spreading_factor", 9)
        # Fading
        self.declare_parameter("fading_model", "rayleigh")
        self.declare_parameter("rician_k_db", 10.0)
        # Dynamic routes
        self.declare_parameter("routes_file", "")
        # Cuaca
        self.declare_parameter("weather", "clear")
        # Model PER
        self.declare_parameter("use_per_model", True)
        # Lingkungan
        self.declare_parameter("temperature_c", 22.0)
        self.declare_parameter("humidity_pct", 60.0)

        self.meters_per_world_unit = float(self.get_parameter("meters_per_world_unit").value)
        self.frequency_mhz = float(self.get_parameter("frequency_mhz").value)
        self.tx_power_dbm = float(self.get_parameter("tx_power_dbm").value)
        self.antenna_gain_db = float(self.get_parameter("antenna_gain_db").value)
        self.terrain_loss_db_per_km = float(self.get_parameter("terrain_loss_db_per_km").value)
        self.reference_lat = float(self.get_parameter("reference_lat").value)
        self.reference_lon = float(self.get_parameter("reference_lon").value)
        self.hiker_id = str(self.get_parameter("hiker_id").value)
        self.topic_prefix = normalize_topic_prefix(
            str(self.get_parameter("topic_prefix").value)
        )
        self.publish_global_events = parameter_as_bool(
            self.get_parameter("publish_global_events").value
        )
        self.active_hiker_count = max(1, int(self.get_parameter("active_hiker_count").value))

        sf_raw = int(self.get_parameter("spreading_factor").value)
        self.spreading_factor = max(7, min(12, sf_raw))
        self.receiver_sensitivity_dbm = _SF_SENSITIVITY[self.spreading_factor]
        self.time_on_air_ms = _lora_time_on_air_ms(self.spreading_factor)
        self.data_rate_bps = _SF_DATA_RATE[self.spreading_factor]

        self.fading_model = str(self.get_parameter("fading_model").value)
        rician_k_db = float(self.get_parameter("rician_k_db").value)
        self.rician_k_linear = 10.0 ** (rician_k_db / 10.0)
        self._fading_rng = random.Random()

        weather_raw = str(self.get_parameter("weather").value)
        if weather_raw not in WEATHER_PROFILES:
            self.get_logger().warn(f"Unknown weather='{weather_raw}', using 'clear'.")
            weather_raw = "clear"
        self.weather = weather_raw
        self._weather_profile = WEATHER_PROFILES[self.weather]
        self._weather_rng = random.Random()

        self._use_per_model = bool(self.get_parameter("use_per_model").value)
        self._per_rng = random.Random()
        self._collision_rng = random.Random()
        self._hw_rng = random.Random()

        # Lingkungan
        self._temperature_c = float(self.get_parameter("temperature_c").value)
        self._humidity_pct = float(self.get_parameter("humidity_pct").value)

        # TX power: nominal dan efektif (bisa dikurangi oleh baterai)
        self._nominal_tx_power_dbm = self.tx_power_dbm
        self._effective_tx_power_dbm = self.tx_power_dbm

        # Protokol paket
        self._seq_num = 0
        self._recent_seqs: deque = deque(maxlen=100)  # window deteksi duplikat

        # Duty cycle: rolling window (timestamp_s, toa_ms)
        self._duty_window: deque = deque()

        # Skenario dari YAML
        routes_file = str(self.get_parameter("routes_file").value)
        scenario = load_scenario_yaml(routes_file) if routes_file else None
        if scenario:
            self._lora_nodes = scenario.get("lora_nodes", LORA_NODES)
            self._base_station = scenario.get("base_station", BASE_STATION)
            self._radio_obstacles = scenario.get("radio_obstacles", RADIO_OBSTACLES)
            self.get_logger().info(f"Skenario dimuat dari: {routes_file}")
        else:
            self._lora_nodes = LORA_NODES
            self._base_station = BASE_STATION
            self._radio_obstacles = RADIO_OBSTACLES

        self.last_pose: Optional[PoseStamped] = None
        self.last_gps: Optional[NavSatFix] = None
        self._sos_active = False
        self._sos_count = 0
        self._last_sos_source = None

        self.event_pub = self.create_publisher(
            String, topic_for(self.topic_prefix, "lora/network_event", "/lora/network_event"), 10
        )
        self.base_pub = self.create_publisher(
            String,
            topic_for(self.topic_prefix, "base_station/hiker_location", "/base_station/hiker_location"),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, topic_for(self.topic_prefix, "lora/markers", "/lora/markers"), 10
        )
        self.global_event_pub = None
        self.global_base_pub = None
        if self.topic_prefix and self.publish_global_events:
            self.global_event_pub = self.create_publisher(String, "/lora/network_event", 10)
            self.global_base_pub = self.create_publisher(String, "/base_station/hiker_location", 10)

        self.create_subscription(
            PoseStamped,
            topic_for(self.topic_prefix, "pose", "/hiker/pose"),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            topic_for(self.topic_prefix, "gps", "/hiker/gps"),
            self.gps_callback,
            10,
        )
        self.create_subscription(
            String,
            topic_for(self.topic_prefix, "battery", "/hiker/battery"),
            self._on_battery,
            10,
        )
        self.create_subscription(String, "/hiker/sos", self._on_sos, 10)

        # Interval TX dihitung dari ToA dan batas duty cycle 1%/jam (ITU).
        # tx_interval = ToA / limit → persis mengisi 1% tanpa pernah melebihi.
        tx_interval_s = self.time_on_air_ms / (1000.0 * _DUTY_CYCLE_LIMIT)
        self.timer = self.create_timer(tx_interval_s, self.tick)
        self.get_logger().info(
            f"LoRa network simulator started for {self.hiker_id}. "
            f"SF={self.spreading_factor} "
            f"sensitivity={self.receiver_sensitivity_dbm}dBm "
            f"ToA={self.time_on_air_ms:.1f}ms rate={self.data_rate_bps:.0f}bps "
            f"fading={self.fading_model} weather={self.weather} "
            f"active_hikers={self.active_hiker_count}"
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def pose_callback(self, msg: PoseStamped) -> None:
        self.last_pose = msg

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps = msg

    def _on_battery(self, msg: String) -> None:
        """Sesuaikan TX power dan data lingkungan berdasarkan status baterai hiker."""
        try:
            data = json.loads(msg.data)
            # TX power reduction (dikirim oleh hiker_agent)
            eff = data.get("effective_tx_power_dbm", self._nominal_tx_power_dbm)
            self._effective_tx_power_dbm = float(eff)
            # Lingkungan
            if "temperature_c" in data:
                self._temperature_c = float(data["temperature_c"])
            if "humidity_pct" in data:
                self._humidity_pct = float(data["humidity_pct"])
        except (json.JSONDecodeError, ValueError):
            pass

    def _on_sos(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        target = str(
            data.get("target_hiker_id", data.get("hiker_id", data.get("target", "")))
        ).strip()
        if not self._target_matches(target):
            return

        self._sos_active = parameter_as_bool(data.get("active", True))
        self._last_sos_source = data.get("source", "unknown")
        if self._sos_active:
            self._sos_count += 1
            self.get_logger().warn(f"SOS flag set for {self.hiker_id}; next LoRa events will carry SOS.")
        else:
            self.get_logger().info(f"SOS flag cleared for {self.hiker_id}.")

    def _target_matches(self, target: str) -> bool:
        if target == self.hiker_id:
            return True
        return self.hiker_id == "hiker" and target in {"hiker", "hiker_1"}

    # -----------------------------------------------------------------------
    # Duty cycle
    # -----------------------------------------------------------------------

    def _duty_cycle_allowed(self) -> Tuple[bool, float]:
        """
        Cek apakah transmisi diizinkan berdasarkan batas duty cycle 1%/jam (ITU).
        MODIFIED: Adjust duty cycle limit untuk multi-hop efficiency.
        Kembalikan (allowed, used_fraction).
        """
        now = time.time()
        cutoff = now - _DUTY_CYCLE_WINDOW_S
        while self._duty_window and self._duty_window[0][0] < cutoff:
            self._duty_window.popleft()
        used_ms = sum(e[1] for e in self._duty_window)
        
        # Multi-hop networks dapat sedikit relax duty cycle limit (1.5% instead of 1%)
        # karena relay nodes berbagi beban transmisi
        multi_hop_limit = _DUTY_CYCLE_LIMIT * 1.5
        budget_ms = _DUTY_CYCLE_WINDOW_S * multi_hop_limit * 1000.0  # 54,000 ms
        allowed = (used_ms + self.time_on_air_ms) <= budget_ms
        return allowed, used_ms / budget_ms if budget_ms > 0 else 0.0

    def _record_duty_cycle(self) -> None:
        self._duty_window.append((time.time(), self.time_on_air_ms))

    # -----------------------------------------------------------------------
    # Hop latency
    # -----------------------------------------------------------------------

    def _compute_hop_latency(self, route: List[str]) -> Dict:
        """
        Simulasi latency jaringan multi-hop:
        queue delay + processing delay + backoff + kemungkinan retransmisi.
        """
        n_hops = max(0, len(route) - 1)
        if n_hops == 0:
            return {
                "per_hop_latency_ms":  [],
                "queue_delay_ms":      0.0,
                "processing_delay_ms": 0.0,
                "retransmission_count": 0,
                "end_to_end_latency_ms": round(self.time_on_air_ms, 1),
            }

        per_hop = []
        total_queue_ms = 0.0
        total_proc_ms  = 0.0
        total_retx     = 0

        for i in range(n_hops):
            node_name = route[i + 1] if i + 1 < len(route) else "base_station"

            # Queue delay (exponential, rata-rata 50 ms)
            q_ms = min(
                self._hw_rng.expovariate(1.0 / _QUEUE_DELAY_MEAN_MS),
                _QUEUE_DELAY_MAX_MS,
            )

            # Processing delay per node
            if "base" in node_name:
                p_ms = self._hw_rng.uniform(*_BASE_HW_DELAY_MS)
            else:
                p_ms = self._hw_rng.uniform(*_RELAY_HW_DELAY_MS)

            # Backoff (15% peluang channel sibuk → tunggu)
            backoff_ms = 0.0
            if self._hw_rng.random() < 0.15:
                backoff_ms = self._hw_rng.uniform(10.0, 100.0)

            # Retransmisi (5% peluang ACK missed)
            retx_count = 0
            retx_delay_ms = 0.0
            if self._hw_rng.random() < 0.05:
                retx_count = 1
                retx_delay_ms = self.time_on_air_ms + self._hw_rng.uniform(100.0, 500.0)

            hop_ms = self.time_on_air_ms + q_ms + p_ms + backoff_ms + retx_delay_ms
            per_hop.append(round(hop_ms, 1))
            total_queue_ms += q_ms
            total_proc_ms  += p_ms
            total_retx     += retx_count

        # E2E = first-hop ToA + semua relay hops
        e2e_ms = self.time_on_air_ms + sum(per_hop)
        return {
            "per_hop_latency_ms":   [round(ms, 1) for ms in per_hop],
            "queue_delay_ms":       round(total_queue_ms, 1),
            "processing_delay_ms":  round(total_proc_ms, 1),
            "retransmission_count": total_retx,
            "end_to_end_latency_ms": round(e2e_ms, 1),
        }

    # -----------------------------------------------------------------------
    # Packet protocol
    # -----------------------------------------------------------------------

    def _packet_protocol(self, delivered: bool, route: List[str], worst_margin: float) -> Dict:
        """
        Simulasi protokol paket: sequence number, CRC, deteksi duplikat,
        hop count limit, dan TTL.
        """
        self._seq_num = (self._seq_num + 1) % 65536
        seq = self._seq_num
        hop_count = max(0, len(route) - 1) if route else 0

        # CRC error: makin buruk margin → makin tinggi error rate
        # Base CRC error rate 0.5%; meningkat jika margin rendah
        if delivered:
            crc_err_prob = max(0.005, _per_from_margin(worst_margin) * 0.1)
            crc_valid = self._hw_rng.random() > crc_err_prob
        else:
            crc_valid = False

        # Deteksi duplikat (paket yang sama via jalur berbeda)
        is_duplicate = seq in self._recent_seqs
        if delivered:
            self._recent_seqs.append(seq)

        # Hop count limit
        hop_count_exceeded = hop_count > _MAX_HOP_COUNT

        # TTL (hanya flag konseptual; dalam simulasi real diperlukan timestamp)
        ttl_expired = False

        # Paket dibuang jika ada pelanggaran protokol
        packet_discarded = hop_count_exceeded or ttl_expired or (delivered and not crc_valid)

        # Header overhead: LoRa MAC header (13 byte) + 2 byte per hop dalam routing path
        header_bytes = 13 + hop_count * 2

        return {
            "sequence_number":    seq,
            "hop_count":          hop_count,
            "crc_valid":          crc_valid,
            "is_duplicate":       is_duplicate,
            "hop_count_exceeded": hop_count_exceeded,
            "ttl_expired":        ttl_expired,
            "packet_discarded":   packet_discarded,
            "header_bytes":       header_bytes,
        }

    # -----------------------------------------------------------------------
    # Main tick
    # -----------------------------------------------------------------------

    def tick(self) -> None:
        if self.last_pose is None:
            self.publish_markers(None, [])
            return

        hiker = self.hiker_station_from_pose(self.last_pose)
        entry_node, entry_link = self.select_entry_node(hiker)
        route, route_links = self.route_to_base(entry_node) if entry_node else ([], [])

        delivered = bool(
            entry_node
            and route
            and route[-1] == self._base_station.name
            and entry_link
        )

        # Kumpulkan semua link
        all_links: List[Dict] = []
        if entry_link:
            all_links.append(entry_link)
        all_links.extend(route_links)

        # --- Duty cycle check ---
        duty_allowed, duty_used = self._duty_cycle_allowed()
        duty_limit_hit = False
        if delivered and not duty_allowed:
            delivered = False
            duty_limit_hit = True
            self.get_logger().warn(
                f"Duty cycle limit hit! Used={duty_used*100:.1f}% of 1% budget."
            )
        if duty_allowed:
            self._record_duty_cycle()

        # --- PER vs SNR (probabilistic drop untuk link borderline) ---
        # MODIFIED: Reduce PER aggressiveness untuk multi-hop (relay packets already penalized by hops)
        per_drop = False
        if delivered and self._use_per_model and all_links:
            worst_margin = min(lk["margin_db"] for lk in all_links)
            # Multi-hop packets: reduce PER by 30% per hop (relays have better equipment)
            hop_count = len(all_links)
            per_reduction = 0.7 ** (hop_count - 1)  # 1.0 untuk 1 hop, 0.7 untuk 2 hop, 0.49 untuk 3+
            per = _per_from_margin(worst_margin) * per_reduction
            if self._per_rng.random() < per:
                delivered = False
                per_drop = True

        # --- Channel collision (pure ALOHA model, Abramson 1970) ---
        # G = offered load = n_tx × ToA / T_window, dimana setiap node TX sekali
        # per tx_interval_s. P_collision = 1 - e^(-2G) (pure ALOHA).
        # Relay networks sedikit mengurangi collision karena TDMA-like scheduling.
        collision_drop = False
        if delivered:
            n_contenders = len(self._lora_nodes) + self.active_hiker_count
            toa_s = self.time_on_air_ms / 1000.0
            tx_interval_s = toa_s / _DUTY_CYCLE_LIMIT  # interval tiap node (worst case)
            g_load = n_contenders * toa_s / tx_interval_s  # offered load = n × duty_cycle
            col_prob = 1.0 - exp(-2.0 * g_load)  # pure ALOHA collision prob
            col_prob *= 0.6  # relay network scheduling mengurangi 40%
            if self._collision_rng.random() < col_prob:
                delivered = False
                collision_drop = True

        # --- Extra drop dari cuaca (thunderstorm/heavy rain) ---
        weather_drop = False
        if delivered and self._weather_profile["extra_drop_prob"] > 0:
            if self._weather_rng.random() < self._weather_profile["extra_drop_prob"]:
                delivered = False
                weather_drop = True

        if not delivered:
            all_links = []
            route = []

        # --- Hop latency ---
        full_route = ([self.hiker_id] + route) if delivered else []
        latency = self._compute_hop_latency(full_route)

        # --- Packet protocol ---
        worst_margin_for_proto = (
            min(lk["margin_db"] for lk in all_links) if all_links else 0.0
        )
        protocol = self._packet_protocol(delivered, full_route, worst_margin_for_proto)

        # Jika packet_discarded oleh protokol, override delivered
        if protocol["packet_discarded"] and delivered:
            delivered = False

        gps_payload = self.gps_payload(hiker)

        # SNR dari worst-case link
        snr_db = None
        if all_links:
            worst_rx = min(lk["rx_dbm"] for lk in all_links)
            snr_db = round(worst_rx - _THERMAL_NOISE_DBM, 1)

        event = {
            "hiker_id":       self.hiker_id,
            "stamp":          self.get_clock().now().nanoseconds / 1e9,
            "delivered":      delivered,
            "entry_node":     entry_node.name if entry_node else None,
            "route":          full_route,
            "hiker_local":    {"x": round(hiker.x, 2), "y": round(hiker.y, 2), "alt_m": round(hiker.z, 1)},
            "gps":            gps_payload,
            "links":          all_links,
            "obstacles":      [asdict(o) for o in self._radio_obstacles],
            # LoRa RF
            "spreading_factor": self.spreading_factor,
            "data_rate_bps":  round(self.data_rate_bps, 1),
            "time_on_air_ms": round(self.time_on_air_ms, 1),
            "snr_db":         snr_db,
            "thermal_noise_dbm": round(_THERMAL_NOISE_DBM, 1),
            # Cuaca
            "weather":        self.weather,
            # Drop reasons
            "drop_reason": (
                "duty_cycle"  if duty_limit_hit  else
                "per_model"   if per_drop        else
                "collision"   if collision_drop  else
                "weather"     if weather_drop    else
                "protocol"    if protocol["packet_discarded"] else
                "no_route"    if not entry_node or not route else
                None
            ),
            "duty_cycle_used_pct": round(duty_used * 100.0, 2),
            # Hop latency
            **latency,
            # Packet protocol
            **protocol,
            # Lingkungan
            "temperature_c": round(self._temperature_c, 1),
            "humidity_pct":  round(self._humidity_pct, 1),
            "active_hiker_count": self.active_hiker_count,
            "sos_active": self._sos_active,
            "sos_count": self._sos_count,
            "sos_source": self._last_sos_source,
        }

        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self.event_pub.publish(msg)
        if self.global_event_pub is not None:
            self.global_event_pub.publish(msg)

        if delivered:
            base_msg = String()
            worst_margin_v = min(lk["margin_db"] for lk in all_links) if all_links else 0.0
            base_msg.data = json.dumps(
                {
                    "hiker_id":           self.hiker_id,
                    "hiker_gps":         gps_payload,
                    "route":             full_route,
                    "hop_count":         protocol["hop_count"],
                    "worst_margin_db":   round(worst_margin_v, 1),
                    "received_by":       self._base_station.name,
                    "spreading_factor":  self.spreading_factor,
                    "weather":           self.weather,
                    "end_to_end_latency_ms": latency["end_to_end_latency_ms"],
                    "sequence_number":   protocol["sequence_number"],
                    "sos_active":        self._sos_active,
                    "sos_count":         self._sos_count,
                },
                separators=(",", ":"),
            )
            self.base_pub.publish(base_msg)
            if self.global_base_pub is not None:
                self.global_base_pub.publish(base_msg)
        else:
            self.get_logger().warn(
                f"LoRa packet dropped for {self.hiker_id}: {event['drop_reason']}. weather={self.weather}"
            )

        self.publish_markers(hiker, all_links)

    # -----------------------------------------------------------------------
    # Station helpers
    # -----------------------------------------------------------------------

    def hiker_station_from_pose(self, pose: PoseStamped) -> Station:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        altitude = terrain_altitude_m(x, y, self.meters_per_world_unit) + 1.8
        return Station(self.hiker_id, x, y, altitude, "hiker")

    def radio_station(self, station: Station) -> Station:
        return Station(
            station.name,
            station.x,
            station.y,
            station_radio_altitude_m(station, self.meters_per_world_unit),
            station.kind,
        )

    def gps_payload(self, hiker: Station) -> Dict[str, float]:
        if self.last_gps is not None:
            return {
                "lat":   round(float(self.last_gps.latitude),  7),
                "lon":   round(float(self.last_gps.longitude), 7),
                "alt_m": round(float(self.last_gps.altitude),  1),
            }
        lat, lon, alt = local_to_gps(
            hiker.x, hiker.y, hiker.z,
            self.reference_lat, self.reference_lon, self.meters_per_world_unit,
        )
        return {"lat": round(lat, 7), "lon": round(lon, 7), "alt_m": round(alt, 1)}

    def select_entry_node(self, hiker: Station) -> Tuple[Optional[Station], Optional[Dict]]:
        """
        Pilih entry node berdasarkan margin DETERMINISTIK (tanpa fading) terbesar.
        Fading tidak dipakai untuk keputusan routing agar stabil dan tidak fluktuatif.
        Entry_link yang dikembalikan dihitung DENGAN fading untuk tampilan/PER model.
        """
        # Screen kandidat dengan margin deterministik (tanpa fading)
        candidates = []
        for node in self._lora_nodes:
            radio_node = self.radio_station(node)
            link_det = self.link_budget(hiker, radio_node, with_fading=False)
            if link_det["margin_db"] >= 0.0:
                candidates.append((-link_det["margin_db"], radio_node))

        # Urutkan: margin terbesar lebih dulu
        candidates.sort(key=lambda item: item[0])

        # Coba satu per satu, mulai margin terbaik; return yang punya route ke base
        for _, candidate_node in candidates:
            route, _ = self.route_to_base(candidate_node, with_fading_links=False)
            if route and route[-1] == self._base_station.name:
                # Hitung entry_link DENGAN fading untuk dashboard dan PER model
                radio_node = self.radio_station(candidate_node)
                display_link = self.link_budget(hiker, radio_node, with_fading=True)
                return candidate_node, display_link

        # Fallback: margin sedikit negatif (−3 dB) — koneksi sangat marginal
        fallback = []
        for node in self._lora_nodes:
            radio_node = self.radio_station(node)
            link_det = self.link_budget(hiker, radio_node, with_fading=False)
            if -3.0 <= link_det["margin_db"] < 0.0:
                fallback.append((-link_det["margin_db"], radio_node))
        fallback.sort(key=lambda item: item[0])
        for _, candidate_node in fallback:
            route, _ = self.route_to_base(candidate_node, with_fading_links=False)
            if route and route[-1] == self._base_station.name:
                radio_node = self.radio_station(candidate_node)
                display_link = self.link_budget(hiker, radio_node, with_fading=True)
                return candidate_node, display_link

        return None, None

    def route_to_base(
        self, start: Optional[Station], with_fading_links: bool = True
    ) -> Tuple[List[str], List[Dict]]:
        """
        Dijkstra routing: adjacency graph dibangun TANPA fading (deterministik).
        Fading di link_budget hanya dipakai saat hitung PER / tampilan dashboard.
        Ini mencegah 'no_route' acak akibat Rayleigh fading yang jelek saat routing.

        with_fading_links=True  → path_links dikembalikan dengan fading (untuk PER/display)
        with_fading_links=False → path_links tanpa fading (untuk cek eksistensi rute cepat)
        """
        if start is None:
            return [], []

        all_stations = [self._base_station] + self._lora_nodes
        stations = {s.name: self.radio_station(s) for s in all_stations}

        # Bangun adjacency dengan margin DETERMINISTIK (tanpa fading)
        adjacency: Dict[str, List[Tuple[float, str]]] = {n: [] for n in stations}
        names = list(stations.keys())
        for li, ln in enumerate(names):
            for rn in names[li + 1:]:
                link_det = self.link_budget(stations[ln], stations[rn], with_fading=False)
                if link_det["margin_db"] >= 0.0:
                    hop_penalty  = 100.0
                    dist_weight  = link_det["distance_m"] * 0.1
                    weight = hop_penalty + dist_weight
                    adjacency[ln].append((weight, rn))
                    adjacency[rn].append((weight, ln))

        # Dijkstra — simpan hanya nama node (bukan link), lebih ringan
        queue: List[Tuple[float, str, List[str]]] = [(0.0, start.name, [start.name])]
        visited: set = set()

        while queue:
            cost, name, path_names = heapq.heappop(queue)
            if name in visited:
                continue
            visited.add(name)

            if name == self._base_station.name:
                # Temukan rute → bangun path_links dengan fading sesuai flag
                path_links: List[Dict] = []
                for i in range(len(path_names) - 1):
                    src = stations[path_names[i]]
                    dst = stations[path_names[i + 1]]
                    lk  = self.link_budget(src, dst, with_fading=with_fading_links)
                    path_links.append(lk)
                return path_names, path_links

            for edge_cost, neighbor in adjacency[name]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + edge_cost, neighbor, path_names + [neighbor]))

        return [], []

    # -----------------------------------------------------------------------
    # Link budget (enhanced)
    # -----------------------------------------------------------------------

    def link_budget(self, left: Station, right: Station, with_fading: bool = True) -> Dict:
        dx_m = (left.x - right.x) * self.meters_per_world_unit
        dy_m = (left.y - right.y) * self.meters_per_world_unit
        dz_m = left.z - right.z
        distance_m  = max(sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m), 1.0)
        distance_km = distance_m / 1000.0

        # FSPL (free-space path loss)
        fspl = 32.44 + 20.0 * log10(distance_km) + 20.0 * log10(self.frequency_mhz)

        # Obstacle loss (depth-aware + wet foliage)
        obstacle_loss, crossed = self.obstacle_loss(left, right)
        obstacle_loss_raw = obstacle_loss  # simpan sebelum koreksi untuk is_los

        # Terrain shadow + knife-edge diffraction
        shadow_loss, diffraction_loss = self.terrain_shadow_loss(left, right)

        # Korelasi terrain-obstacle (ITU-R P.833 §4.3):
        # Ketika terrain mendominasi (shadow/diffraction besar), sinyal melewati
        # puncak terrain melalui difraksi — jalur ini sebagian besar menghindari
        # vegetasi di lembah. Obstacle loss dikurangi proporsional dengan dominasi terrain.
        terrain_excess_db = shadow_loss + diffraction_loss
        if terrain_excess_db > 5.0:
            terrain_dom = min(1.0, terrain_excess_db / 25.0)
            obstacle_loss = obstacle_loss * (1.0 - 0.60 * terrain_dom)

        # Terrain scatter (ground bounce, ITU-R P.452):
        # Hanya relevan untuk jalur LOS (shadow=0, diffraction=0).
        # Untuk NLOS, komponen scatter sudah termasuk dalam model shadow/diffraction.
        if shadow_loss == 0.0 and diffraction_loss == 0.0:
            terrain_loss = distance_km * self.terrain_loss_db_per_km
        else:
            terrain_loss = 0.0

        # Weather attenuation
        weather_loss = (
            self._weather_profile["attn_db_km"] * distance_km
            + self._weather_profile["noise_db"]
        )

        # Temperature-based noise effect
        # <0°C: komponen elektronik kurang efisien (+1.0 dB noise figure)
        # >40°C: thermal noise lebih tinggi (+0.5 dB)
        temp_noise_db = 0.0
        if self._temperature_c < 0.0:
            temp_noise_db = min(1.5, abs(self._temperature_c) * 0.05)
        elif self._temperature_c > 40.0:
            temp_noise_db = min(1.0, (self._temperature_c - 40.0) * 0.03)

        # Humidity atmospheric absorption (~0.003 dB/km at 100% RH, negligible but modeled)
        humidity_loss = distance_km * max(0.0, (self._humidity_pct - 50.0) / 50.0) * 0.003

        # TX power: gunakan efektif untuk hiker, full power untuk relay/base
        # PENTING: relay nodes tetap full TX power untuk maximize reach
        tx_power = (
            self._effective_tx_power_dbm if left.kind == "hiker"
            else self.tx_power_dbm  # Relay & base station always full power
        )

        rx_dbm = (
            tx_power
            + self.antenna_gain_db * 2.0
            - fspl
            - obstacle_loss
            - shadow_loss
            - diffraction_loss
            - terrain_loss
            - weather_loss
            - temp_noise_db
            - humidity_loss
        )

        # Fading (Rayleigh/Rician) — hanya disample jika with_fading=True.
        # Routing (route_to_base, select_entry_node) memanggil dengan with_fading=False
        # agar Dijkstra menggunakan margin deterministik → routing stabil, tidak no_route acak.
        # Fading tetap dipakai di entry_link display dan PER model (via tick()).
        is_los = obstacle_loss_raw == 0.0 and shadow_loss == 0.0
        fading_loss = 0.0
        fading_type = "none"
        if with_fading and self.fading_model != "none":
            if is_los:
                fading_loss = self._rician_fading_db(self.rician_k_linear)
                fading_type = "LOS/Rician"
            else:
                fading_loss = self._rayleigh_fading_db()
                fading_type = "NLOS/Rayleigh"
            rx_dbm -= fading_loss

        # SNR terhadap thermal noise floor
        snr_db = rx_dbm - _THERMAL_NOISE_DBM

        margin = rx_dbm - self.receiver_sensitivity_dbm
        return {
            "from":                   left.name,
            "to":                     right.name,
            "distance_m":             round(distance_m, 1),
            "rx_dbm":                 round(rx_dbm, 1),
            "snr_db":                 round(snr_db, 1),
            "margin_db":              round(margin, 1),
            "obstacle_loss_db":       round(obstacle_loss, 1),
            "terrain_shadow_loss_db": round(shadow_loss, 1),
            "diffraction_loss_db":    round(diffraction_loss, 1),
            "terrain_loss_db":        round(terrain_loss, 1),
            "weather_loss_db":        round(weather_loss, 1),
            "temp_noise_db":          round(temp_noise_db, 2),
            "humidity_loss_db":       round(humidity_loss, 3),
            "fading_loss_db":         round(fading_loss, 1),
            "fading_type":            fading_type,
            "crossed_obstacles":      crossed,
        }

    # -----------------------------------------------------------------------
    # Obstacle loss (depth-aware + wet foliage + wind fluctuation)
    # -----------------------------------------------------------------------

    def obstacle_loss(self, left: Station, right: Station) -> Tuple[float, List[str]]:
        loss = 0.0
        crossed: List[str] = []
        wet_factor = _WET_FOLIAGE_FACTOR.get(self.weather, 1.0)

        for obs in self._radio_obstacles:
            if not segment_intersects_circle(
                left.x, left.y, right.x, right.y,
                obs.x, obs.y, obs.radius,
            ):
                continue

            # Kedalaman penetrasi (chord length)
            chord_m = _chord_length_m(
                left.x, left.y, right.x, right.y,
                obs.x, obs.y, obs.radius,
                self.meters_per_world_unit,
            )
            # Referensi: diameter penuh (worst case)
            ref_chord_m = 2.0 * obs.radius * self.meters_per_world_unit
            depth_factor = (chord_m / ref_chord_m) if ref_chord_m > 0.0 else 1.0

            if obs.kind == "trees":
                # Vegetasi: depth × wet foliage + wind fluctuation ±5%
                base_loss = obs.loss_db * depth_factor * wet_factor
                wind_var = self._fading_rng.uniform(-0.05, 0.05)
                link_loss = base_loss * (1.0 + wind_var)
            else:
                # Batu/crater/terrain: depth factor tanpa wet multiplier
                link_loss = obs.loss_db * depth_factor

            loss += max(0.0, link_loss)
            crossed.append(obs.name)

        return loss, crossed

    # -----------------------------------------------------------------------
    # Terrain shadow loss + knife-edge diffraction (Fresnel-Kirchhoff)
    # -----------------------------------------------------------------------

    def terrain_shadow_loss(self, left: Station, right: Station) -> Tuple[float, float]:
        """
        Return (shadow_loss_db, diffraction_loss_db).
        Shadow loss: heuristik sampling (existing model).
        Diffraction loss: Fresnel-Kirchhoff knife-edge untuk puncak terrain terburuk.
        """
        sample_count = 18
        obstructed_samples = 0

        # Total jarak horizontal untuk diffraction
        total_horiz_m = hypot(
            (right.x - left.x) * self.meters_per_world_unit,
            (right.y - left.y) * self.meters_per_world_unit,
        )

        worst_clearance_m = 0.0  # terrain melebihi LOS (positif = halangan)
        worst_d1_m = 1.0
        worst_d2_m = 1.0

        for s in range(1, sample_count):
            ratio = s / sample_count
            x = left.x  + (right.x  - left.x)  * ratio
            y = left.y  + (right.y  - left.y)  * ratio
            line_alt = left.z + (right.z - left.z) * ratio
            terrain_alt = terrain_altitude_m(x, y, self.meters_per_world_unit)

            # Earth curvature correction (untuk jarak jauh)
            d1_h = ratio * total_horiz_m
            d2_h = (1.0 - ratio) * total_horiz_m
            r_earth = 6_370_000.0
            curvature_m = (d1_h * d2_h) / (2.0 * r_earth)
            terrain_alt_eff = terrain_alt + curvature_m

            fresnel_clearance_m = (
                8.0 + 0.012 * min(s, sample_count - s) * self.meters_per_world_unit
            )
            if terrain_alt_eff + fresnel_clearance_m > line_alt:
                obstructed_samples += 1

            clearance = terrain_alt_eff - line_alt
            if clearance > worst_clearance_m:
                worst_clearance_m = clearance
                worst_d1_m = max(1.0, d1_h)
                worst_d2_m = max(1.0, d2_h)

        shadow_loss = min(22.0, 3.2 * obstructed_samples) if obstructed_samples > 0 else 0.0

        # Knife-edge diffraction (Fresnel-Kirchhoff)
        diffraction_loss = 0.0
        if worst_clearance_m > 0.0:
            wavelength_m = 3e8 / (self.frequency_mhz * 1e6)
            denom = wavelength_m * worst_d1_m * worst_d2_m
            if denom > 0.0:
                nu = worst_clearance_m * sqrt(
                    2.0 * (worst_d1_m + worst_d2_m) / denom
                )
                # Fresnel-Kirchhoff: polinomial hanya valid untuk ν ≤ 2.4
                # Untuk ν > 2.4 gunakan ITU-R P.526-15 asimtotik agar tidak
                # menghasilkan ribuan dB loss yang tidak realistis.
                if nu > -0.7:
                    if nu <= 2.4:
                        raw = 6.02 + 9.11 * nu + 1.27 * nu * nu
                    else:
                        raw = 13.46 + 20.0 * log10(nu)
                    diffraction_loss = max(0.0, raw)

        # Gunakan maksimum (hindari double-counting)
        return shadow_loss, max(0.0, diffraction_loss - shadow_loss)

    # -----------------------------------------------------------------------
    # Fading helpers
    # -----------------------------------------------------------------------

    def _rayleigh_fading_db(self) -> float:
        s2 = 1.0 / sqrt(2.0)
        r = self._fading_rng.gauss(0.0, s2)
        i = self._fading_rng.gauss(0.0, s2)
        amplitude = sqrt(r * r + i * i)
        return -20.0 * log10(max(amplitude, 1e-10))

    def _rician_fading_db(self, k_linear: float) -> float:
        nu    = sqrt(k_linear / (k_linear + 1.0))
        sigma = 1.0 / sqrt(2.0 * (k_linear + 1.0))
        r = self._fading_rng.gauss(nu, sigma)
        i = self._fading_rng.gauss(0.0, sigma)
        amplitude = sqrt(r * r + i * i)
        return -20.0 * log10(max(amplitude, 1e-10))

    # -----------------------------------------------------------------------
    # RViz markers (unchanged)
    # -----------------------------------------------------------------------

    def publish_markers(self, hiker: Optional[Station], links: List[Dict]) -> None:
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        for station in [self._base_station] + self._lora_nodes:
            markers.markers.append(self.station_marker(station, marker_id, now))
            marker_id += 1

        if hiker is not None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = self.hiker_id
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = hiker.x
            marker.pose.position.y = hiker.y
            marker.pose.position.z = terrain_height_world(hiker.x, hiker.y) + 2.0
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.62
            marker.color.b = 0.08
            marker.color.a = 1.0
            markers.markers.append(marker)
            marker_id += 1

        if hiker is not None and links:
            station_lookup = {s.name: s for s in [self._base_station] + self._lora_nodes}
            station_lookup[self.hiker_id] = Station(
                self.hiker_id, hiker.x, hiker.y,
                terrain_height_world(hiker.x, hiker.y) + 2.4, "hiker"
            )
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = now
            line.ns = f"{self.hiker_id}_lora_route"
            line.id = marker_id
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.18
            line.color.r = 0.1
            line.color.g = 0.85
            line.color.b = 1.0
            line.color.a = 0.95
            for link in links:
                start = station_lookup.get(link["from"])
                end   = station_lookup.get(link["to"])
                if start is None or end is None:
                    continue
                sz = start.z if start.name == self.hiker_id else terrain_height_world(start.x, start.y) + 7.2
                ez = end.z   if end.name   == self.hiker_id else terrain_height_world(end.x,   end.y)   + 7.2
                line.points.append(Point(x=start.x, y=start.y, z=sz))
                line.points.append(Point(x=end.x,   y=end.y,   z=ez))
            markers.markers.append(line)

        self.marker_pub.publish(markers)

    def station_marker(self, station: Station, marker_id: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "stations"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = station.x
        marker.pose.position.y = station.y
        marker.pose.position.z = terrain_height_world(station.x, station.y) + 3.0
        marker.scale.x = 0.9
        marker.scale.y = 0.9
        marker.scale.z = 4.0
        if station.kind == "base":
            marker.color.r = 0.0
            marker.color.g = 0.25
            marker.color.b = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.95
            marker.color.b = 0.45
        marker.color.a = 1.0
        return marker


# ---------------------------------------------------------------------------
# Geometry helper
# ---------------------------------------------------------------------------

def segment_intersects_circle(
    ax: float, ay: float, bx: float, by: float,
    cx: float, cy: float, radius: float,
) -> bool:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return hypot(ax - cx, ay - cy) <= radius
    t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / length_sq))
    return hypot(ax + t * dx - cx, ay + t * dy - cy) <= radius


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def normalize_topic_prefix(prefix: str) -> str:
    prefix = prefix.strip()
    if not prefix:
        return ""
    return "/" + prefix.strip("/")


def topic_for(prefix: str, suffix: str, legacy_topic: str) -> str:
    if not prefix:
        return legacy_topic
    return f"{prefix}/{suffix.lstrip('/')}"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoraNetwork()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
