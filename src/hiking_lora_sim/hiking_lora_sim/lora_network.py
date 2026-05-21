import heapq
import json
import random
from dataclasses import asdict
from math import ceil, hypot, log10, sqrt
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

# Sensitivity LoRa per SF (dBm), BW=125 kHz, sesuai spesifikasi Semtech SX1276
_SF_SENSITIVITY: Dict[int, float] = {
    7: -123.0,
    8: -126.0,
    9: -129.0,
    10: -132.0,
    11: -134.5,
    12: -136.0,
}

# Data rate (bps) per SF, BW=125 kHz, CR=4/5
_SF_DATA_RATE: Dict[int, float] = {
    7: 5468.75,
    8: 3125.0,
    9: 1757.8,
    10: 878.9,
    11: 476.6,
    12: 250.0,
}

# Profil efek cuaca pada propagasi 915 MHz
WEATHER_PROFILES: Dict[str, Dict] = {
    "clear":        {"attn_db_km": 0.00, "noise_db": 0.0,  "extra_drop_prob": 0.00},
    "fog":          {"attn_db_km": 0.00, "noise_db": 0.5,  "extra_drop_prob": 0.00},
    "light_rain":   {"attn_db_km": 0.01, "noise_db": 1.0,  "extra_drop_prob": 0.00},
    "heavy_rain":   {"attn_db_km": 0.05, "noise_db": 3.0,  "extra_drop_prob": 0.05},
    "thunderstorm": {"attn_db_km": 0.10, "noise_db": 8.0,  "extra_drop_prob": 0.15},
}


def _lora_time_on_air_ms(sf: int, payload_bytes: int = 20) -> float:
    """Estimasi Time on Air LoRa (ms), BW=125kHz, CR=4/5, header eksplisit."""
    bw = 125_000.0
    t_sym = (2 ** sf) / bw
    t_preamble = (8 + 4.25) * t_sym
    n_payload = max(8, ceil((8 * payload_bytes - 4 * sf + 28 + 16) / (4 * sf)) * 5)
    return (t_preamble + n_payload * t_sym) * 1000.0


class LoraNetwork(Node):
    def __init__(self) -> None:
        super().__init__("lora_network")

        self.declare_parameter("meters_per_world_unit", 35.0)
        self.declare_parameter("frequency_mhz", 915.0)
        self.declare_parameter("tx_power_dbm", 17.0)
        self.declare_parameter("antenna_gain_db", 2.0)
        self.declare_parameter("terrain_loss_db_per_km", 2.5)
        self.declare_parameter("reference_lat", -6.89148)
        self.declare_parameter("reference_lon", 107.61066)
        # Fitur 5: Spreading Factor
        self.declare_parameter("spreading_factor", 9)
        # Fitur 7: Fading model
        self.declare_parameter("fading_model", "rayleigh")
        self.declare_parameter("rician_k_db", 10.0)
        # Fitur 9: Dynamic routes
        self.declare_parameter("routes_file", "")
        # Fitur 10: Cuaca
        self.declare_parameter("weather", "clear")

        self.meters_per_world_unit = float(self.get_parameter("meters_per_world_unit").value)
        self.frequency_mhz = float(self.get_parameter("frequency_mhz").value)
        self.tx_power_dbm = float(self.get_parameter("tx_power_dbm").value)
        self.antenna_gain_db = float(self.get_parameter("antenna_gain_db").value)
        self.terrain_loss_db_per_km = float(self.get_parameter("terrain_loss_db_per_km").value)
        self.reference_lat = float(self.get_parameter("reference_lat").value)
        self.reference_lon = float(self.get_parameter("reference_lon").value)

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

        # Fitur 9: muat skenario dari YAML jika tersedia
        routes_file = str(self.get_parameter("routes_file").value)
        scenario = load_scenario_yaml(routes_file) if routes_file else None
        if scenario:
            self._lora_nodes = scenario["lora_nodes"]
            self._base_station = scenario["base_station"]
            self._radio_obstacles = scenario["radio_obstacles"]
            self.get_logger().info(f"Skenario dimuat dari: {routes_file}")
        else:
            self._lora_nodes = LORA_NODES
            self._base_station = BASE_STATION
            self._radio_obstacles = RADIO_OBSTACLES

        self.last_pose: Optional[PoseStamped] = None
        self.last_gps: Optional[NavSatFix] = None

        self.event_pub = self.create_publisher(String, "/lora/network_event", 10)
        self.base_pub = self.create_publisher(String, "/base_station/hiker_location", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/lora/markers", 10)
        self.create_subscription(PoseStamped, "/hiker/pose", self.pose_callback, 10)
        self.create_subscription(NavSatFix, "/hiker/gps", self.gps_callback, 10)

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f"LoRa network simulator started. SF={self.spreading_factor} "
            f"sensitivity={self.receiver_sensitivity_dbm}dBm "
            f"ToA={self.time_on_air_ms:.1f}ms rate={self.data_rate_bps:.0f}bps "
            f"fading={self.fading_model} weather={self.weather}"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.last_pose = msg

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps = msg

    def tick(self) -> None:
        if self.last_pose is None:
            self.publish_markers(None, [])
            return

        hiker = self.hiker_station_from_pose(self.last_pose)
        entry_node, entry_link = self.select_entry_node(hiker)
        route, route_links = self.route_to_base(entry_node) if entry_node else ([], [])

        delivered = bool(
            entry_node and route and route[-1] == self._base_station.name and entry_link
        )

        # Fitur 10: drop acak akibat cuaca (thunderstorm, heavy_rain)
        if delivered and self._weather_profile["extra_drop_prob"] > 0:
            if self._weather_rng.random() < self._weather_profile["extra_drop_prob"]:
                delivered = False
                route = []
                route_links = []

        links = []
        if entry_link:
            links.append(entry_link)
        links.extend(route_links)

        gps_payload = self.gps_payload(hiker)
        event = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "delivered": delivered,
            "entry_node": entry_node.name if entry_node else None,
            "route": ["hiker"] + route if delivered else [],
            "hiker_local": {"x": round(hiker.x, 2), "y": round(hiker.y, 2), "alt_m": round(hiker.z, 1)},
            "gps": gps_payload,
            "links": links,
            "obstacles": [asdict(obstacle) for obstacle in self._radio_obstacles],
            "spreading_factor": self.spreading_factor,
            "data_rate_bps": round(self.data_rate_bps, 1),
            "time_on_air_ms": round(self.time_on_air_ms, 1),
            "weather": self.weather,
        }

        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self.event_pub.publish(msg)

        if delivered:
            base_msg = String()
            worst_margin = min(link["margin_db"] for link in links) if links else 0.0
            base_msg.data = json.dumps(
                {
                    "hiker_gps": gps_payload,
                    "route": event["route"],
                    "hop_count": len(event["route"]) - 1,
                    "worst_margin_db": round(worst_margin, 1),
                    "received_by": self._base_station.name,
                    "spreading_factor": self.spreading_factor,
                    "weather": self.weather,
                },
                separators=(",", ":"),
            )
            self.base_pub.publish(base_msg)
        else:
            self.get_logger().warn(
                f"LoRa packet dropped: no reachable route. weather={self.weather}"
            )

        self.publish_markers(hiker, links)

    def hiker_station_from_pose(self, pose: PoseStamped) -> Station:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        altitude = terrain_altitude_m(x, y, self.meters_per_world_unit) + 1.8
        return Station("hiker", x, y, altitude, "hiker")

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
                "lat": round(float(self.last_gps.latitude), 7),
                "lon": round(float(self.last_gps.longitude), 7),
                "alt_m": round(float(self.last_gps.altitude), 1),
            }
        lat, lon, alt = local_to_gps(
            hiker.x,
            hiker.y,
            hiker.z,
            self.reference_lat,
            self.reference_lon,
            self.meters_per_world_unit,
        )
        return {"lat": round(lat, 7), "lon": round(lon, 7), "alt_m": round(alt, 1)}

    def select_entry_node(self, hiker: Station) -> Tuple[Optional[Station], Optional[Dict]]:
        reachable = []
        for node in self._lora_nodes:
            radio_node = self.radio_station(node)
            link = self.link_budget(hiker, radio_node)
            if link["margin_db"] >= 0.0:
                reachable.append((link["distance_m"], -link["margin_db"], radio_node, link))
        if not reachable:
            return None, None
        reachable.sort(key=lambda item: (item[0], item[1]))
        _, _, node, link = reachable[0]
        return node, link

    def route_to_base(self, start: Optional[Station]) -> Tuple[List[str], List[Dict]]:
        if start is None:
            return [], []

        all_stations = [self._base_station] + self._lora_nodes
        stations = {s.name: self.radio_station(s) for s in all_stations}
        adjacency: Dict[str, List[Tuple[float, str, Dict]]] = {name: [] for name in stations}

        names = list(stations.keys())
        for left_index, left_name in enumerate(names):
            for right_name in names[left_index + 1:]:
                left = stations[left_name]
                right = stations[right_name]
                link = self.link_budget(left, right)
                if link["margin_db"] >= 0.0:
                    weight = link["distance_m"] + max(0.0, 25.0 - link["margin_db"]) * 8.0
                    adjacency[left_name].append((weight, right_name, link))
                    reverse_link = dict(link)
                    reverse_link["from"] = right_name
                    reverse_link["to"] = left_name
                    adjacency[right_name].append((weight, left_name, reverse_link))

        queue = [(0.0, start.name, [])]
        visited: set = set()
        while queue:
            cost, name, path_links = heapq.heappop(queue)
            if name in visited:
                continue
            visited.add(name)
            if name == self._base_station.name:
                route = [start.name]
                for link in path_links:
                    route.append(link["to"])
                return route, path_links
            for edge_cost, neighbor, link in adjacency[name]:
                if neighbor in visited:
                    continue
                heapq.heappush(queue, (cost + edge_cost, neighbor, path_links + [link]))

        return [], []

    def link_budget(self, left: Station, right: Station) -> Dict:
        dx_m = (left.x - right.x) * self.meters_per_world_unit
        dy_m = (left.y - right.y) * self.meters_per_world_unit
        dz_m = left.z - right.z
        distance_m = max(sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m), 1.0)
        distance_km = distance_m / 1000.0

        fspl = 32.44 + 20.0 * log10(distance_km) + 20.0 * log10(self.frequency_mhz)
        obstacle_loss, crossed = self.obstacle_loss(left, right)
        shadow_loss = self.terrain_shadow_loss(left, right)
        terrain_loss = distance_km * self.terrain_loss_db_per_km

        # Fitur 10: Efek cuaca
        weather_loss = (
            self._weather_profile["attn_db_km"] * distance_km
            + self._weather_profile["noise_db"]
        )

        rx_dbm = (
            self.tx_power_dbm
            + self.antenna_gain_db * 2.0
            - fspl
            - obstacle_loss
            - shadow_loss
            - terrain_loss
            - weather_loss
        )

        # Fitur 7: Fading model
        is_los = obstacle_loss == 0.0 and shadow_loss == 0.0
        fading_loss = 0.0
        fading_type = "none"
        if self.fading_model != "none":
            if is_los:
                fading_loss = self._rician_fading_db(self.rician_k_linear)
                fading_type = "LOS/Rician"
            else:
                fading_loss = self._rayleigh_fading_db()
                fading_type = "NLOS/Rayleigh"
            rx_dbm -= fading_loss

        margin = rx_dbm - self.receiver_sensitivity_dbm
        return {
            "from": left.name,
            "to": right.name,
            "distance_m": round(distance_m, 1),
            "rx_dbm": round(rx_dbm, 1),
            "margin_db": round(margin, 1),
            "obstacle_loss_db": round(obstacle_loss, 1),
            "terrain_shadow_loss_db": round(shadow_loss, 1),
            "terrain_loss_db": round(terrain_loss, 1),
            "weather_loss_db": round(weather_loss, 1),
            "fading_loss_db": round(fading_loss, 1),
            "fading_type": fading_type,
            "crossed_obstacles": crossed,
        }

    def obstacle_loss(self, left: Station, right: Station) -> Tuple[float, List[str]]:
        loss = 0.0
        crossed: List[str] = []
        for obstacle in self._radio_obstacles:
            if segment_intersects_circle(
                left.x, left.y, right.x, right.y, obstacle.x, obstacle.y, obstacle.radius
            ):
                loss += obstacle.loss_db
                crossed.append(obstacle.name)
        return loss, crossed

    def terrain_shadow_loss(self, left: Station, right: Station) -> float:
        obstructed_samples = 0
        sample_count = 18
        for sample in range(1, sample_count):
            ratio = sample / sample_count
            x = left.x + (right.x - left.x) * ratio
            y = left.y + (right.y - left.y) * ratio
            line_alt = left.z + (right.z - left.z) * ratio
            terrain_alt = terrain_altitude_m(x, y, self.meters_per_world_unit)
            fresnel_clearance_m = (
                8.0 + 0.012 * min(sample, sample_count - sample) * self.meters_per_world_unit
            )
            if terrain_alt + fresnel_clearance_m > line_alt:
                obstructed_samples += 1
        if obstructed_samples == 0:
            return 0.0
        return min(22.0, 3.2 * obstructed_samples)

    # --- Fading helpers (pure stdlib) ---

    def _rayleigh_fading_db(self) -> float:
        """Rayleigh fading: NLOS, dua komponen Gaussian i.i.d."""
        s2 = 1.0 / sqrt(2.0)
        r = self._fading_rng.gauss(0.0, s2)
        i = self._fading_rng.gauss(0.0, s2)
        amplitude = sqrt(r * r + i * i)
        return -20.0 * log10(max(amplitude, 1e-10))

    def _rician_fading_db(self, k_linear: float) -> float:
        """Rician fading: LOS + scattering, K-factor = k_linear (linear)."""
        nu = sqrt(k_linear / (k_linear + 1.0))
        sigma = 1.0 / sqrt(2.0 * (k_linear + 1.0))
        r = self._fading_rng.gauss(nu, sigma)
        i = self._fading_rng.gauss(0.0, sigma)
        amplitude = sqrt(r * r + i * i)
        return -20.0 * log10(max(amplitude, 1e-10))

    # --- RViz markers ---

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
            marker.ns = "hiker"
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
            station_lookup["hiker"] = Station(
                "hiker", hiker.x, hiker.y, terrain_height_world(hiker.x, hiker.y) + 2.4, "hiker"
            )
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = now
            line.ns = "lora_route"
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
                end = station_lookup.get(link["to"])
                if start is None or end is None:
                    continue
                start_z = (
                    start.z if start.name == "hiker"
                    else terrain_height_world(start.x, start.y) + 7.2
                )
                end_z = (
                    end.z if end.name == "hiker"
                    else terrain_height_world(end.x, end.y) + 7.2
                )
                line.points.append(Point(x=start.x, y=start.y, z=start_z))
                line.points.append(Point(x=end.x, y=end.y, z=end_z))
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
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return hypot(closest_x - cx, closest_y - cy) <= radius


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
