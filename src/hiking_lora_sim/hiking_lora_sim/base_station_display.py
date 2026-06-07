import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class BaseStationDisplay(Node):
    def __init__(self) -> None:
        super().__init__("base_station_display")
        self.create_subscription(String, "/base_station/hiker_location", self.location_callback, 10)
        self.create_subscription(String, "/lora/network_event", self.network_callback, 10)
        self.last_delivery_stamp = None
        self.get_logger().info("Base station display ready. Waiting for LoRa packets.")

    def location_callback(self, msg: String) -> None:
        payload = json.loads(msg.data)
        hiker_id = payload.get("hiker_id", "hiker")
        gps = payload["hiker_gps"]
        route = " -> ".join(payload["route"])
        sos_prefix = "[SOS] " if payload.get("sos_active") else ""
        self.last_delivery_stamp = self.get_clock().now()
        self.get_logger().info(
            "%sReceived %s location lat=%.7f lon=%.7f alt=%.1fm hops=%d margin=%.1fdB route=%s"
            % (
                sos_prefix,
                hiker_id,
                gps["lat"],
                gps["lon"],
                gps["alt_m"],
                payload["hop_count"],
                payload["worst_margin_db"],
                route,
            )
        )

    def network_callback(self, msg: String) -> None:
        payload = json.loads(msg.data)
        if payload.get("delivered"):
            return
        hiker_id = payload.get("hiker_id", "hiker")
        self.get_logger().warn(f"No base-station delivery for latest packet from {hiker_id}.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseStationDisplay()
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
