import json
import random
from math import cos, hypot, sin

try:
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.transport13 import Node as GzNode
except ImportError:
    GzBoolean = None
    GzPose = None
    GzNode = None

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from hiking_lora_sim.scenario import (
    ACTIVE_TRAIL_NAME,
    RADIO_OBSTACLES,
    TRAILS,
    load_scenario_yaml,
    local_to_gps,
    point_on_trail,
    terrain_altitude_m,
    terrain_height_world,
)

# Time on Air default SF9 (detik) untuk kalkulasi baterai
_DEFAULT_TOA_S = 0.124

# GPS cold start delay
_DEFAULT_TTFF_S = 8.0

# Offset visual supaya mesh pendaki kecil tetap menempel tanpa clipping di terrain miring.
_DEFAULT_HIKER_VISUAL_Z_OFFSET = 0.08

# Faktor kecepatan gerak pendaki berdasarkan cuaca
_WEATHER_SPEED_FACTOR = {
    "clear":        1.00,
    "fog":          0.85,
    "light_rain":   0.80,
    "heavy_rain":   0.65,
    "thunderstorm": 0.50,
}

# Model tegangan baterai Li-Ion
_BATTERY_VOLTAGE_FULL  = 4.20  # V pada SoC 100%
_BATTERY_VOLTAGE_EMPTY = 3.00  # V batas cutoff


# ---------------------------------------------------------------------------
# GPS error helper functions
# ---------------------------------------------------------------------------

def _gps_dop_factor(x: float, y: float, obstacles) -> float:
    """
    HDOP faktor berdasarkan posisi: >1.0 di hutan/lembah karena pandangan satelit
    terhalang kanopi atau terrain. Semakin dalam di dalam obstacle → DOP makin buruk.
    """
    dop = 1.0
    for obs in obstacles:
        d = hypot(x - obs.x, y - obs.y)
        if d < obs.radius:
            depth_frac = 1.0 - d / obs.radius  # 0 di tepi, 1 di pusat
            if obs.kind == "trees":
                dop += depth_frac * 1.8   # hutan rapat: satelit banyak terhalang
            elif obs.kind == "terrain":
                dop += depth_frac * 1.2   # bayangan terrain
            elif obs.kind == "crater":
                dop += depth_frac * 0.8   # dinding kawah menutupi sebagian langit
            elif obs.kind == "rocks":
                dop += depth_frac * 0.5   # batu: sebagian terhalang
    return min(dop, 4.5)  # cap HDOP=4.5 (sangat buruk)


def _multipath_noise_m(x: float, y: float, obstacles, base_noise_m: float) -> float:
    """
    Noise GPS tambahan akibat multipath: sinyal memantul dari tebing/batu di sekitar
    posisi, menyebabkan error posisi tambahan.
    """
    extra = 0.0
    for obs in obstacles:
        d = hypot(x - obs.x, y - obs.y)
        # Multipath paling kuat tepat di tepi dan sedikit di luar batu/terrain
        if obs.kind in ("rocks", "terrain") and d < obs.radius * 1.5:
            proximity = max(0.0, 1.0 - d / (obs.radius * 1.5))
            extra += proximity * base_noise_m * 2.5
    return extra


class HikerAgent(Node):
    def __init__(self) -> None:
        super().__init__("hiker_agent")

        self.declare_parameter("speed_world_units_s", 0.85)
        self.declare_parameter("gps_noise_std_m", 2.0)
        self.declare_parameter("meters_per_world_unit", 35.0)
        self.declare_parameter("reference_lat", -6.89148)
        self.declare_parameter("reference_lon", 107.61066)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("trail_name", ACTIVE_TRAIL_NAME)
        self.declare_parameter("gazebo_pose_control", True)
        self.declare_parameter("gazebo_world_name", "hiking_lora_world")
        self.declare_parameter("gazebo_hiker_model", "hiker")
        self.declare_parameter("gazebo_pose_timeout_ms", 50)
        self.declare_parameter("gazebo_visual_rate_hz", 10.0)
        self.declare_parameter("gazebo_hiker_z_offset", _DEFAULT_HIKER_VISUAL_Z_OFFSET)
        # Baterai
        self.declare_parameter("battery_capacity_mah", 3000.0)
        self.declare_parameter("tx_current_ma", 120.0)
        self.declare_parameter("idle_current_ma", 3.0)
        self.declare_parameter("supply_voltage_v", 3.7)
        # Rute dinamis
        self.declare_parameter("routes_file", "")
        # GPS & hardware
        self.declare_parameter("ttff_delay_s", _DEFAULT_TTFF_S)
        # Lingkungan
        self.declare_parameter("weather", "clear")
        self.declare_parameter("temperature_c", 22.0)
        self.declare_parameter("humidity_pct", 60.0)

        self.speed_world_units_s = float(self.get_parameter("speed_world_units_s").value)
        self.gps_noise_std_m = float(self.get_parameter("gps_noise_std_m").value)
        self.meters_per_world_unit = float(self.get_parameter("meters_per_world_unit").value)
        self.reference_lat = float(self.get_parameter("reference_lat").value)
        self.reference_lon = float(self.get_parameter("reference_lon").value)
        self.trail_name = str(self.get_parameter("trail_name").value)
        self.gazebo_pose_control = parameter_as_bool(
            self.get_parameter("gazebo_pose_control").value
        )
        self.gazebo_world_name = str(self.get_parameter("gazebo_world_name").value)
        self.gazebo_hiker_model = str(self.get_parameter("gazebo_hiker_model").value)
        self.gazebo_pose_timeout_ms = int(self.get_parameter("gazebo_pose_timeout_ms").value)
        self.gazebo_visual_rate_hz = float(self.get_parameter("gazebo_visual_rate_hz").value)
        self.gazebo_hiker_z_offset = float(self.get_parameter("gazebo_hiker_z_offset").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # Baterai
        self._battery_capacity_mah = float(self.get_parameter("battery_capacity_mah").value)
        self._tx_current_ma = float(self.get_parameter("tx_current_ma").value)
        self._idle_current_ma = float(self.get_parameter("idle_current_ma").value)
        self._supply_voltage_v = float(self.get_parameter("supply_voltage_v").value)
        self._dt = 1.0 / max(0.1, publish_rate_hz)
        self._charge_used_mah = 0.0
        self._tx_count = 0

        # GPS error state
        self._ttff_delay_s = float(self.get_parameter("ttff_delay_s").value)
        self._ttff_done = False

        # Hardware error: clock drift (±0.08 ms/s ≈ ±5s/hari, typical crystal drift)
        self._clock_drift_s = 0.0
        hw_seed = random.Random()
        self._clock_drift_rate = hw_seed.gauss(0.0, 8e-5)
        self._hw_rng = random.Random()

        # Node state
        self._node_active = True
        self._low_power_mode = False
        self._low_power_logged = False

        # Lingkungan
        self._weather = str(self.get_parameter("weather").value)
        self._temperature_c = float(self.get_parameter("temperature_c").value)
        self._humidity_pct = float(self.get_parameter("humidity_pct").value)

        # Muat skenario dari YAML jika ada
        routes_file = str(self.get_parameter("routes_file").value)
        active_trails = TRAILS
        gps_obstacles = list(RADIO_OBSTACLES)
        if routes_file:
            scenario = load_scenario_yaml(routes_file)
            if scenario:
                if scenario.get("trails"):
                    active_trails = scenario["trails"]
                    self.get_logger().info(f"Rute dimuat dari: {routes_file}")
                if scenario.get("radio_obstacles"):
                    gps_obstacles = scenario["radio_obstacles"]
        self._gps_obstacles = gps_obstacles

        if self.trail_name not in active_trails:
            self.get_logger().warn(
                f"Unknown trail_name={self.trail_name}; using {ACTIVE_TRAIL_NAME}."
            )
            self.trail_name = ACTIVE_TRAIL_NAME
            active_trails = TRAILS
        self.trail_points = active_trails[self.trail_name]

        self.gz_pose_service = f"/world/{self.gazebo_world_name}/set_pose"
        self.gz_node = None
        self.gz_pose_wait_logged = False
        self.gz_pose_active_logged = False
        if self.gazebo_pose_control:
            if GzNode is None:
                self.get_logger().warn(
                    "Gazebo Python transport is unavailable; Gazebo hiker visual will not be pose-controlled."
                )
            else:
                self.gz_node = GzNode()

        self.pose_pub = self.create_publisher(PoseStamped, "/hiker/pose", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/hiker/gps", 10)
        self.status_pub = self.create_publisher(String, "/hiker/status", 10)
        self.battery_pub = self.create_publisher(String, "/hiker/battery", 10)

        self.random = random.Random(42)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self._dt, self.publish_state)
        self.visual_timer = None
        if self.gz_node is not None:
            visual_dt = 1.0 / max(1.0, self.gazebo_visual_rate_hz)
            self.visual_timer = self.create_timer(visual_dt, self.publish_gazebo_visual)

        self.get_logger().info("Hiker GPS simulator started. Publishing /hiker/gps and /hiker/pose.")

    def route_pose_at(self, elapsed: float):
        speed_factor = _WEATHER_SPEED_FACTOR.get(self._weather, 1.0)
        effective_speed = self.speed_world_units_s * speed_factor
        if self._low_power_mode:
            effective_speed *= 0.85

        distance = elapsed * effective_speed
        x, y, yaw = point_on_trail(distance, self.trail_points)
        terrain_z = terrain_height_world(x, y)
        return x, y, yaw, terrain_z, speed_factor, effective_speed

    def publish_state(self) -> None:
        # --- Node shutdown saat baterai habis ---
        if not self._node_active:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # --- Clock drift (akumulasi per tick) ---
        self._clock_drift_s += self._clock_drift_rate * self._dt
        clock_drift_ms = self._clock_drift_s * 1000.0

        # --- Hardware processing latency (ESP32 sensor read + packet prep) ---
        sensor_read_delay_ms  = self._hw_rng.uniform(10.0, 50.0)
        processing_delay_ms   = self._hw_rng.uniform(30.0, 150.0)
        packet_prep_delay_ms  = self._hw_rng.uniform(5.0, 30.0)
        hw_delay_ms = sensor_read_delay_ms + processing_delay_ms + packet_prep_delay_ms

        # --- Watchdog restart (probabilitas sangat kecil: ~0.01% per tick) ---
        node_restarted = False
        if self._hw_rng.random() < 0.0001:
            node_restarted = True
            self.get_logger().warn("Watchdog restart simulated! Skipping this tick.")
            return

        # --- TTFF: GPS tidak tersedia saat cold start ---
        if elapsed < self._ttff_delay_s:
            stamp = self.get_clock().now().to_msg()
            # Tetap publish pose ROS meskipun GPS belum fix.
            x_raw, y_raw, yaw_raw, terrain_z_raw, _, _ = self.route_pose_at(elapsed)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = x_raw
            pose.pose.position.y = y_raw
            pose.pose.position.z = terrain_z_raw + 1.0
            pose.pose.orientation.z = sin(yaw_raw * 0.5)
            pose.pose.orientation.w = cos(yaw_raw * 0.5)
            self.pose_pub.publish(pose)

            gps = NavSatFix()
            gps.header.stamp = stamp
            gps.header.frame_id = "hiker_gps"
            gps.status.status = NavSatStatus.STATUS_NO_FIX  # belum ada fix
            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.latitude = 0.0
            gps.longitude = 0.0
            gps.altitude = 0.0
            self.gps_pub.publish(gps)
            return

        if not self._ttff_done:
            self._ttff_done = True
            self.get_logger().info(f"GPS fix acquired after {elapsed:.1f}s (TTFF cold start done).")

        # --- Kecepatan gerak berdasarkan cuaca ---
        x, y, yaw, terrain_z, speed_factor, _ = self.route_pose_at(elapsed)
        altitude = terrain_altitude_m(x, y, self.meters_per_world_unit)

        # --- GPS DOP berdasarkan posisi (hutan/lembah → DOP buruk) ---
        dop = _gps_dop_factor(x, y, self._gps_obstacles)

        # --- GPS multipath noise dari tebing/batu ---
        multipath_m = _multipath_noise_m(x, y, self._gps_obstacles, self.gps_noise_std_m)

        # --- Total GPS noise = gaussian × DOP + multipath ---
        effective_noise_m = self.gps_noise_std_m * dop + multipath_m

        noise_east  = self.random.gauss(0.0, effective_noise_m)
        noise_north = self.random.gauss(0.0, effective_noise_m)
        noisy_x = x + noise_east  / self.meters_per_world_unit
        noisy_y = y + noise_north / self.meters_per_world_unit
        noisy_altitude = altitude + self.random.gauss(0.0, effective_noise_m * 0.5)
        lat, lon, alt = local_to_gps(
            noisy_x,
            noisy_y,
            noisy_altitude,
            self.reference_lat,
            self.reference_lon,
            self.meters_per_world_unit,
        )

        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = terrain_z + 1.0
        pose.pose.orientation.z = sin(yaw * 0.5)
        pose.pose.orientation.w = cos(yaw * 0.5)
        self.pose_pub.publish(pose)

        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = "hiker_gps"
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = alt
        variance = effective_noise_m * effective_noise_m
        gps.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(gps)

        # --- Model baterai ---
        self._tx_count += 1
        toa_s = min(_DEFAULT_TOA_S, self._dt)
        idle_s = max(0.0, self._dt - toa_s)

        # Low-power mode: TX current dikurangi 50%
        effective_tx_current = self._tx_current_ma * (0.5 if self._low_power_mode else 1.0)
        charge_tick = (effective_tx_current * toa_s + self._idle_current_ma * idle_s) / 3600.0
        self._charge_used_mah += charge_tick

        remaining_mah = max(0.0, self._battery_capacity_mah - self._charge_used_mah)
        percentage = remaining_mah / self._battery_capacity_mah * 100.0

        # --- Voltage drop (linear dari 4.2V → 3.0V) ---
        soc = remaining_mah / self._battery_capacity_mah  # 0.0–1.0
        voltage_v = _BATTERY_VOLTAGE_FULL - (1.0 - soc) * (
            _BATTERY_VOLTAGE_FULL - _BATTERY_VOLTAGE_EMPTY
        )

        # --- TX power reduction saat baterai < 30% SoC (max −6 dBm) ---
        if soc < 0.30:
            tx_reduction_db = (1.0 - soc / 0.30) * 6.0
            effective_tx_power_dbm = 17.0 - tx_reduction_db
        else:
            effective_tx_power_dbm = 17.0

        total_energy_mj = self._charge_used_mah / 1000.0 * self._supply_voltage_v * 3600.0
        avg_current_ma = (
            effective_tx_current * (toa_s / self._dt)
            + self._idle_current_ma * (idle_s / self._dt)
        )
        hours_remaining = (remaining_mah / avg_current_ma) if avg_current_ma > 0 else 0.0

        # --- Node shutdown saat baterai habis ---
        if remaining_mah <= 0.0:
            self._node_active = False
            self.get_logger().warn("Battery exhausted! Node shutting down.")

        # --- Low-power mode transition ---
        prev_lp = self._low_power_mode
        self._low_power_mode = percentage < 20.0
        if self._low_power_mode and not prev_lp:
            self.get_logger().warn(
                f"Low-power mode activated at {percentage:.1f}% battery."
            )

        bat_msg = String()
        bat_msg.data = json.dumps(
            {
                "percentage":            round(percentage, 1),
                "remaining_mah":         round(remaining_mah, 1),
                "used_mah":              round(self._charge_used_mah, 1),
                "tx_count":              self._tx_count,
                "total_energy_mj":       round(total_energy_mj, 1),
                "hours_remaining":       round(hours_remaining, 2),
                # Tambahan baru
                "voltage_v":             round(voltage_v, 3),
                "effective_tx_power_dbm": round(effective_tx_power_dbm, 1),
                "low_power_mode":        self._low_power_mode,
                "node_active":           self._node_active,
                "temperature_c":         round(self._temperature_c, 1),
                "humidity_pct":          round(self._humidity_pct, 1),
            },
            separators=(",", ":"),
        )
        self.battery_pub.publish(bat_msg)

        gps_error_m = hypot(noise_east, noise_north)
        status = String()
        status.data = (
            f"hiker track={self.trail_name} x={x:.2f} y={y:.2f} "
            f"terrain_z={terrain_z:.2f} alt={altitude:.1f}m "
            f"gps=({lat:.7f},{lon:.7f}) bat={percentage:.1f}% "
            f"dop={dop:.2f} gps_err={gps_error_m:.1f}m "
            f"drift={clock_drift_ms:+.1f}ms hw_delay={hw_delay_ms:.0f}ms "
            f"weather={self._weather} speed_factor={speed_factor:.2f} "
            f"node_restarted={node_restarted}"
        )
        self.status_pub.publish(status)

    def publish_gazebo_visual(self) -> None:
        if not self._node_active:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x, y, yaw, terrain_z, _, _ = self.route_pose_at(elapsed)
        self.publish_gazebo_pose(x, y, yaw, terrain_z)

    def publish_gazebo_pose(self, x: float, y: float, yaw: float, terrain_z: float) -> None:
        if self.gz_node is None or GzPose is None or GzBoolean is None:
            return

        pose = GzPose()
        pose.name = self.gazebo_hiker_model
        pose.position.x = x
        pose.position.y = y
        # The generated hiker model origin sits at foot level, so place it just
        # above the terrain instead of at GPS/body height. The extra offset
        # prevents terrain clipping on steep slopes.
        pose.position.z = terrain_z + self.gazebo_hiker_z_offset
        pose.orientation.z = sin(yaw * 0.5)
        pose.orientation.w = cos(yaw * 0.5)

        ok, response = self.gz_node.request(
            self.gz_pose_service,
            pose,
            GzPose,
            GzBoolean,
            max(1, self.gazebo_pose_timeout_ms),
        )
        if ok and response.data:
            if not self.gz_pose_active_logged:
                self.get_logger().info(
                    f"Gazebo hiker visual is following {self.trail_name} through {self.gz_pose_service}."
                )
                self.gz_pose_active_logged = True
            return

        if not self.gz_pose_wait_logged:
            self.get_logger().warn(
                f"Waiting for Gazebo pose service {self.gz_pose_service}; ROS GPS simulation is still running."
            )
            self.gz_pose_wait_logged = True


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HikerAgent()
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
