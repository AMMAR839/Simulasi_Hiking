import json
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        self.declare_parameter("manual_command_duration_s", 0.15)
        self.declare_parameter("overview_camera_xy_step", 8.0)
        self.declare_parameter("overview_camera_z_step", 10.0)
        self.declare_parameter("overview_camera_angle_step_rad", 0.10)
        self._duration_s = float(self.get_parameter("manual_command_duration_s").value)
        self._overview_xy_step = float(self.get_parameter("overview_camera_xy_step").value)
        self._overview_z_step = float(self.get_parameter("overview_camera_z_step").value)
        self._overview_angle_step_rad = float(
            self.get_parameter("overview_camera_angle_step_rad").value
        )
        self._active_hiker_index = 1
        self._running = True
        self._stream = None
        self._owns_stream = False
        self._original_termios = None

        self.command_pub = self.create_publisher(String, "/hiker/manual_control", 10)
        self.camera_pub = self.create_publisher(String, "/hiker/camera_control", 10)
        self.sos_pub = self.create_publisher(String, "/hiker/sos", 10)
        self._open_keyboard()
        self.create_timer(0.02, self._poll_keyboard)

        self.get_logger().info(
            "Keyboard teleop ready. 1-9/0 select and follow hiker, W/S move, A/D turn, "
            "B SOS, R auto path, Q follow camera, E overview camera, X quit."
        )

    @property
    def running(self) -> bool:
        return self._running

    def _open_keyboard(self) -> None:
        if sys.stdin.isatty():
            self._stream = sys.stdin
        else:
            try:
                self._stream = open("/dev/tty", "r", encoding="utf-8", buffering=1)
                self._owns_stream = True
            except OSError:
                self.get_logger().error(
                    "No interactive terminal available; keyboard teleop cannot read WASD input."
                )
                return

        fd = self._stream.fileno()
        self._original_termios = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    def _poll_keyboard(self) -> None:
        if self._stream is None:
            return

        key = self._read_key()
        while key:
            self._handle_key(key)
            key = self._read_key()

    def _read_key(self) -> str:
        readable, _, _ = select.select([self._stream], [], [], 0.0)
        if not readable:
            return ""
        return self._stream.read(1)

    def _handle_key(self, key: str) -> None:
        key = key.lower()

        if key in "1234567890":
            self._active_hiker_index = 10 if key == "0" else int(key)
            self.get_logger().info(
                f"Manual target selected and camera follow requested: {self._target_hiker_id()}"
            )
            self._publish_camera_command(mode="follow")
            return

        if key == "\x1b" or key == "x":
            self._running = False
            self.get_logger().info("Keyboard teleop stopped.")
            return

        if key == "q":
            self._publish_camera_command(mode="follow")
            return

        if key == "e":
            self._publish_camera_command(mode="overview")
            return

        if key == "r":
            self._publish_command(mode="auto")
            return

        if key == "b":
            self._publish_sos()
            return

        if key == "w":
            self._publish_command(mode="manual", forward=1.0)
        elif key == "s":
            self._publish_command(mode="manual", forward=-1.0)
        elif key == "a":
            self._publish_command(mode="manual", turn=1.0)
        elif key == "d":
            self._publish_command(mode="manual", turn=-1.0)
        elif key == "i":
            self._publish_camera_command(mode="overview_adjust", dy=self._overview_xy_step)
        elif key == "k":
            self._publish_camera_command(mode="overview_adjust", dy=-self._overview_xy_step)
        elif key == "j":
            self._publish_camera_command(mode="overview_adjust", dx=-self._overview_xy_step)
        elif key == "l":
            self._publish_camera_command(mode="overview_adjust", dx=self._overview_xy_step)
        elif key == "u":
            self._publish_camera_command(mode="overview_adjust", dz=-self._overview_z_step)
        elif key == "o":
            self._publish_camera_command(mode="overview_adjust", dz=self._overview_z_step)
        elif key == "[":
            self._publish_camera_command(mode="overview_adjust", dyaw=-self._overview_angle_step_rad)
        elif key == "]":
            self._publish_camera_command(mode="overview_adjust", dyaw=self._overview_angle_step_rad)
        elif key == "-":
            self._publish_camera_command(mode="overview_adjust", dpitch=-self._overview_angle_step_rad)
        elif key == "=":
            self._publish_camera_command(mode="overview_adjust", dpitch=self._overview_angle_step_rad)

    def _target_hiker_id(self) -> str:
        return f"hiker_{self._active_hiker_index}"

    def _publish_command(self, mode: str, forward: float = 0.0, turn: float = 0.0) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "target_hiker_id": self._target_hiker_id(),
                "mode": mode,
                "forward": forward,
                "turn": turn,
                "duration_s": self._duration_s,
            },
            separators=(",", ":"),
        )
        self.command_pub.publish(msg)

    def _publish_camera_command(self, mode: str, **deltas: float) -> None:
        payload = {
            "target_hiker_id": self._target_hiker_id(),
            "mode": mode,
        }
        payload.update(deltas)
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.camera_pub.publish(msg)

    def _publish_sos(self) -> None:
        target = self._target_hiker_id()
        msg = String()
        msg.data = json.dumps(
            {
                "target_hiker_id": target,
                "hiker_id": target,
                "active": True,
                "source": "keyboard_teleop",
                "event": "sos",
            },
            separators=(",", ":"),
        )
        self.sos_pub.publish(msg)
        self.get_logger().warn(f"SOS requested by {target}.")

    def destroy_node(self) -> None:
        if self._stream is not None and self._original_termios is not None:
            try:
                termios.tcsetattr(self._stream.fileno(), termios.TCSADRAIN, self._original_termios)
            except termios.error:
                pass
        if self._owns_stream and self._stream is not None:
            self._stream.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
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
