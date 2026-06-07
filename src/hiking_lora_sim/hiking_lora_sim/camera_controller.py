import json
from math import atan2, cos, pi, sin

try:
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    from gz.msgs10.gui_camera_pb2 import GUICamera as GzGuiCamera
    from gz.transport13 import Node as GzNode
except ImportError:
    GzBoolean = None
    GzGuiCamera = None
    GzNode = None

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hiking_lora_sim.scenario import WORLD_EXTENT


class CameraController(Node):
    """Runtime Gazebo GUI camera control for follow and overview views."""

    def __init__(self) -> None:
        super().__init__("camera_controller")

        self.declare_parameter("target_hiker_id", "hiker_1")
        self.declare_parameter("active_hiker_count", 1)
        self.declare_parameter("camera_update_rate_hz", 8.0)
        self.declare_parameter("gui_camera_timeout_ms", 80)
        self.declare_parameter("follow_camera_height_world_units", 24.0)
        self.declare_parameter("follow_camera_offset_x", 0.0)
        self.declare_parameter("follow_camera_offset_y", 0.0)
        self.declare_parameter("follow_camera_pitch_rad", pi / 2.0)
        self.declare_parameter("follow_camera_roll_rad", 0.0)
        self.declare_parameter("follow_camera_yaw_mode", "fixed")
        self.declare_parameter("follow_camera_fixed_yaw_rad", 0.0)
        self.declare_parameter("overview_camera_x", 0.0)
        self.declare_parameter("overview_camera_y", 0.0)
        self.declare_parameter("overview_camera_z", 220.0)
        self.declare_parameter("overview_camera_roll_rad", 0.0)
        self.declare_parameter("overview_camera_pitch_rad", pi / 2.0)
        self.declare_parameter("overview_camera_yaw_rad", 0.0)
        self.declare_parameter("overview_camera_min_z", 20.0)
        self.declare_parameter("overview_camera_max_z", 420.0)

        self._target_hiker_id = str(self.get_parameter("target_hiker_id").value).strip() or "hiker_1"
        self._active_hiker_count = int(self.get_parameter("active_hiker_count").value)
        update_rate_hz = float(self.get_parameter("camera_update_rate_hz").value)
        self._timeout_ms = int(self.get_parameter("gui_camera_timeout_ms").value)
        self._follow_height = float(
            self.get_parameter("follow_camera_height_world_units").value
        )
        self._follow_offset_x = float(self.get_parameter("follow_camera_offset_x").value)
        self._follow_offset_y = float(self.get_parameter("follow_camera_offset_y").value)
        self._follow_pitch = float(self.get_parameter("follow_camera_pitch_rad").value)
        self._follow_roll = float(self.get_parameter("follow_camera_roll_rad").value)
        self._follow_yaw_mode = str(self.get_parameter("follow_camera_yaw_mode").value).strip().lower()
        self._follow_fixed_yaw = float(self.get_parameter("follow_camera_fixed_yaw_rad").value)

        self._overview_x = float(self.get_parameter("overview_camera_x").value)
        self._overview_y = float(self.get_parameter("overview_camera_y").value)
        self._overview_z = float(self.get_parameter("overview_camera_z").value)
        self._overview_roll = float(self.get_parameter("overview_camera_roll_rad").value)
        self._overview_pitch = float(self.get_parameter("overview_camera_pitch_rad").value)
        self._overview_yaw = float(self.get_parameter("overview_camera_yaw_rad").value)
        self._overview_min_z = float(self.get_parameter("overview_camera_min_z").value)
        self._overview_max_z = float(self.get_parameter("overview_camera_max_z").value)

        self._mode = "overview"
        self._pose_by_hiker = {}
        self._warned_no_gz = False
        self._warned_no_pose = False
        self._warned_no_gui_service = False

        self._gz_node = None
        if GzNode is None:
            self.get_logger().warn(
                "Gazebo Python transport is unavailable; camera Q/E commands cannot move the GUI camera."
            )
        else:
            self._gz_node = GzNode()

        self._status_pub = self.create_publisher(String, "/hiker/camera_status", 10)
        self.create_subscription(String, "/hiker/camera_control", self._on_camera_command, 10)
        self._subscribe_pose_topics()

        timer_period = 1.0 / max(1.0, update_rate_hz)
        self.create_timer(timer_period, self._tick)

        self.get_logger().info(
            "Camera controller ready. Q follows the selected hiker from above; "
            "E returns to mountain overview."
        )

    def _subscribe_pose_topics(self) -> None:
        self.create_subscription(
            PoseStamped,
            "/hiker/pose",
            lambda msg: self._on_pose("hiker", msg),
            10,
        )
        for index in range(1, 11):
            hiker_id = f"hiker_{index}"
            self.create_subscription(
                PoseStamped,
                f"/{hiker_id}/pose",
                lambda msg, current_hiker=hiker_id: self._on_pose(current_hiker, msg),
                10,
            )

    def _on_pose(self, hiker_id: str, msg: PoseStamped) -> None:
        self._pose_by_hiker[hiker_id] = msg
        if hiker_id == "hiker":
            self._pose_by_hiker["hiker_1"] = msg
        elif hiker_id == "hiker_1" and self._active_hiker_count <= 1:
            self._pose_by_hiker["hiker"] = msg

    def _on_camera_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        target = str(command.get("target_hiker_id", command.get("target", ""))).strip()
        if target:
            self._target_hiker_id = self._normalize_target(target)

        mode = str(command.get("mode", "")).strip().lower()
        if mode == "select_target":
            self.get_logger().info(f"Camera target selected: {self._target_hiker_id}")
            return

        if mode == "follow":
            self._mode = "follow"
            self._warned_no_pose = False
            self.get_logger().info(f"Camera follow mode: {self._target_hiker_id}")
            self._publish_status()
            return

        if mode == "overview":
            self._mode = "overview"
            self.get_logger().info("Camera overview mode.")
            self._move_overview_camera()
            self._publish_status()
            return

        if mode == "overview_adjust":
            self._mode = "overview"
            self._apply_overview_adjust(command)
            self._move_overview_camera()
            self._publish_status()

    def _normalize_target(self, target: str) -> str:
        if self._active_hiker_count <= 1 and target in {"hiker", "hiker_1"}:
            return "hiker"
        return target

    def _tick(self) -> None:
        if self._mode == "follow":
            self._move_follow_camera()

    def _move_follow_camera(self) -> None:
        target = self._target_hiker_id
        pose_msg = self._pose_by_hiker.get(target)
        if pose_msg is None and target == "hiker":
            pose_msg = self._pose_by_hiker.get("hiker_1")
        elif pose_msg is None and target == "hiker_1":
            pose_msg = self._pose_by_hiker.get("hiker")

        if pose_msg is None:
            if not self._warned_no_pose:
                self.get_logger().warn(
                    f"Camera follow waits for pose topic of {self._target_hiker_id}."
                )
                self._warned_no_pose = True
            return

        yaw = self._yaw_from_pose(pose_msg)
        camera_yaw = yaw if self._follow_yaw_mode == "hiker" else self._follow_fixed_yaw
        x = pose_msg.pose.position.x + self._follow_offset_x
        y = pose_msg.pose.position.y + self._follow_offset_y
        z = pose_msg.pose.position.z + self._follow_height
        self._move_gui_camera(
            x=x,
            y=y,
            z=z,
            roll=self._follow_roll,
            pitch=self._follow_pitch,
            yaw=camera_yaw,
        )
        self._publish_status()

    def _move_overview_camera(self) -> None:
        self._move_gui_camera(
            x=self._overview_x,
            y=self._overview_y,
            z=self._overview_z,
            roll=self._overview_roll,
            pitch=self._overview_pitch,
            yaw=self._overview_yaw,
        )

    def _move_gui_camera(
        self,
        *,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> bool:
        if self._gz_node is None or GzGuiCamera is None or GzBoolean is None:
            if not self._warned_no_gz:
                self.get_logger().warn("Gazebo GUI camera transport is not available.")
                self._warned_no_gz = True
            return False

        request = GzGuiCamera()
        request.name = "user_camera"
        request.view_controller = "orbit"
        request.projection_type = "perspective"
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        qx, qy, qz, qw = self._quaternion_from_euler(roll, pitch, yaw)
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        ok, response = self._gz_node.request(
            "/gui/move_to/pose",
            request,
            GzGuiCamera,
            GzBoolean,
            max(1, self._timeout_ms),
        )
        if ok and response.data:
            return True

        if not self._warned_no_gui_service:
            self.get_logger().warn(
                "Waiting for Gazebo GUI camera service /gui/move_to/pose. "
                "Open Gazebo with GUI enabled to use Q/E camera switching."
            )
            self._warned_no_gui_service = True
        return False

    def _apply_overview_adjust(self, command) -> None:
        self._overview_x += self._float_value(command.get("dx", 0.0))
        self._overview_y += self._float_value(command.get("dy", 0.0))
        self._overview_z += self._float_value(command.get("dz", 0.0))
        self._overview_yaw += self._float_value(command.get("dyaw", 0.0))
        self._overview_pitch += self._float_value(command.get("dpitch", 0.0))

        self._overview_x = max(-WORLD_EXTENT * 2.0, min(WORLD_EXTENT * 2.0, self._overview_x))
        self._overview_y = max(-WORLD_EXTENT * 2.0, min(WORLD_EXTENT * 2.0, self._overview_y))
        self._overview_z = max(self._overview_min_z, min(self._overview_max_z, self._overview_z))
        self._overview_pitch = max(0.25, min(pi / 2.0, self._overview_pitch))
        self.get_logger().info(
            "Overview camera adjusted: "
            f"x={self._overview_x:.1f} y={self._overview_y:.1f} z={self._overview_z:.1f} "
            f"pitch={self._overview_pitch:.2f} yaw={self._overview_yaw:.2f}"
        )

    @staticmethod
    def _float_value(value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _yaw_from_pose(msg: PoseStamped) -> float:
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "mode": self._mode,
                "target_hiker_id": self._target_hiker_id,
                "overview": {
                    "x": round(self._overview_x, 2),
                    "y": round(self._overview_y, 2),
                    "z": round(self._overview_z, 2),
                    "pitch": round(self._overview_pitch, 3),
                    "yaw": round(self._overview_yaw, 3),
                },
            },
            separators=(",", ":"),
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraController()
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
