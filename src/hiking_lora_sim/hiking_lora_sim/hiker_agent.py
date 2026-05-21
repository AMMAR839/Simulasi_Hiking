import json
import random
from math import cos, sin

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
    TRAILS,
    load_scenario_yaml,
    local_to_gps,
    point_on_trail,
    terrain_altitude_m,
    terrain_height_world,
)

# Time on Air default SF9 (detik) untuk kalkulasi baterai
_DEFAULT_TOA_S = 0.124


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
        self.declare_parameter("gazebo_pose_timeout_ms", 20)
        # Fitur 8: Baterai
        self.declare_parameter("battery_capacity_mah", 3000.0)
        self.declare_parameter("tx_current_ma", 120.0)
        self.declare_parameter("idle_current_ma", 3.0)
        self.declare_parameter("supply_voltage_v", 3.7)
        # Fitur 9: Dynamic routes
        self.declare_parameter("routes_file", "")

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
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # Fitur 8: parameter baterai
        self._battery_capacity_mah = float(self.get_parameter("battery_capacity_mah").value)
        self._tx_current_ma = float(self.get_parameter("tx_current_ma").value)
        self._idle_current_ma = float(self.get_parameter("idle_current_ma").value)
        self._supply_voltage_v = float(self.get_parameter("supply_voltage_v").value)
        self._dt = 1.0 / max(0.1, publish_rate_hz)
        self._charge_used_mah = 0.0
        self._tx_count = 0

        # Fitur 9: muat rute dari YAML jika tersedia
        routes_file = str(self.get_parameter("routes_file").value)
        active_trails = TRAILS
        if routes_file:
            scenario = load_scenario_yaml(routes_file)
            if scenario and scenario.get("trails"):
                active_trails = scenario["trails"]
                self.get_logger().info(f"Rute dimuat dari: {routes_file}")

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

        self.get_logger().info("Hiker GPS simulator started. Publishing /hiker/gps and /hiker/pose.")

    def publish_state(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        distance = elapsed * self.speed_world_units_s
        x, y, yaw = point_on_trail(distance, self.trail_points)
        terrain_z = terrain_height_world(x, y)
        altitude = terrain_altitude_m(x, y, self.meters_per_world_unit)

        noise_east = self.random.gauss(0.0, self.gps_noise_std_m)
        noise_north = self.random.gauss(0.0, self.gps_noise_std_m)
        noisy_x = x + noise_east / self.meters_per_world_unit
        noisy_y = y + noise_north / self.meters_per_world_unit
        noisy_altitude = altitude + self.random.gauss(0.0, self.gps_noise_std_m * 0.5)
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
        variance = self.gps_noise_std_m * self.gps_noise_std_m
        gps.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(gps)

        # Fitur 8: hitung dan publish status baterai
        self._tx_count += 1
        toa_s = min(_DEFAULT_TOA_S, self._dt)
        idle_s = max(0.0, self._dt - toa_s)
        charge_tick = (self._tx_current_ma * toa_s + self._idle_current_ma * idle_s) / 3600.0
        self._charge_used_mah += charge_tick
        remaining_mah = max(0.0, self._battery_capacity_mah - self._charge_used_mah)
        percentage = remaining_mah / self._battery_capacity_mah * 100.0
        total_energy_mj = self._charge_used_mah / 1000.0 * self._supply_voltage_v * 3600.0
        hours_remaining = (
            remaining_mah / (self._tx_current_ma * toa_s / self._dt + self._idle_current_ma * idle_s / self._dt)
            / 1000.0
            if self._tx_current_ma > 0
            else 0.0
        )

        bat_msg = String()
        bat_msg.data = json.dumps(
            {
                "percentage": round(percentage, 1),
                "remaining_mah": round(remaining_mah, 1),
                "used_mah": round(self._charge_used_mah, 1),
                "tx_count": self._tx_count,
                "total_energy_mj": round(total_energy_mj, 1),
                "hours_remaining": round(hours_remaining, 2),
            },
            separators=(",", ":"),
        )
        self.battery_pub.publish(bat_msg)

        status = String()
        status.data = (
            f"hiker track={self.trail_name} x={x:.2f} y={y:.2f} "
            f"terrain_z={terrain_z:.2f} alt={altitude:.1f}m "
            f"gps=({lat:.7f},{lon:.7f}) bat={percentage:.1f}%"
        )
        self.status_pub.publish(status)
        self.publish_gazebo_pose(x, y, yaw, terrain_z)

    def publish_gazebo_pose(self, x: float, y: float, yaw: float, terrain_z: float) -> None:
        if self.gz_node is None or GzPose is None or GzBoolean is None:
            return

        pose = GzPose()
        pose.name = self.gazebo_hiker_model
        pose.position.x = x
        pose.position.y = y
        pose.position.z = terrain_z + 1.05
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
