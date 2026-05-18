"""Mock robot with mixed interfaces, for exercising the example plugin.

Stands in for a robot whose control surface spans three transports:

* **UDP** — streams telemetry packets and receives velocity/heartbeat packets.
* **ROS topic** — publishes its battery level on ``myrobot/battery``.
* **ROS service** — answers a docking request on ``myrobot/dock``.

It is an ``rclpy`` node (for the ROS interfaces) that also runs UDP threads.
Run it alongside a Sugarcoat recipe that uses ``MyRobotPlugin`` with matching
host/ports, or on its own to sanity-check the wire protocols.

    python3 server_node.py --telemetry-port 9870 --command-port 9871
"""

import argparse
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from myrobot_plugin.codecs import (
    decode_command,
    encode_telemetry,
    HEARTBEAT_OP,
)


class MockRobot(Node):
    """A localhost robot with UDP + ROS topic + ROS service interfaces."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        telemetry_port: int = 9870,
        command_port: int = 9871,
        battery_topic: str = "myrobot/battery",
        dock_service: str = "myrobot/dock",
        telemetry_rate_hz: float = 50.0,
    ):
        super().__init__("mock_robot")
        self.telemetry_addr = (host, telemetry_port)
        self.command_port = command_port
        self.telemetry_period = 1.0 / telemetry_rate_hz
        self._stop = threading.Event()

        # --- UDP socket: streams telemetry from / receives commands on ---
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, command_port))
        self._sock.settimeout(0.5)

        # --- ROS interfaces ---
        self._battery = 100.0
        self.docked = False
        self._battery_pub = self.create_publisher(Float32, battery_topic, 10)
        self._battery_timer = self.create_timer(0.5, self._publish_battery)
        self._dock_srv = self.create_service(
            Trigger, dock_service, self._on_dock_request
        )

        # --- integrated pose driven by the latest UDP velocity command ---
        self._x = self._y = self._yaw = 0.0
        self._vx = self._vy = self._vyaw = 0.0

        # --- UDP worker threads ---
        self._tx_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()
        self.get_logger().info(
            f"mock robot up: UDP telemetry -> {self.telemetry_addr}, "
            f"UDP commands <- :{command_port}, ROS battery + dock service"
        )

    # -- ROS interfaces ------------------------------------------------------
    def _publish_battery(self) -> None:
        self._battery = max(0.0, self._battery - 0.1)
        self._battery_pub.publish(Float32(data=self._battery))

    def _on_dock_request(self, _request, response):
        self.docked = True
        self._vx = self._vy = self._vyaw = 0.0
        self.get_logger().info("mock robot: docking")
        response.success = True
        response.message = "docked"
        return response

    # -- UDP interfaces ------------------------------------------------------
    def _command_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if data and data[0] == HEARTBEAT_OP:
                continue
            command = decode_command(data)
            if command is not None:
                self._vx, self._vy, self._vyaw = command

    def _telemetry_loop(self) -> None:
        last = time.time()
        while not self._stop.is_set():
            now = time.time()
            dt = now - last
            last = now
            self._x += (
                self._vx * math.cos(self._yaw) - self._vy * math.sin(self._yaw)
            ) * dt
            self._y += (
                self._vx * math.sin(self._yaw) + self._vy * math.cos(self._yaw)
            ) * dt
            self._yaw += self._vyaw * dt
            try:
                self._sock.sendto(
                    encode_telemetry(self._x, self._y, self._yaw),
                    self.telemetry_addr,
                )
            except OSError:
                break
            time.sleep(self.telemetry_period)

    # -- shutdown ------------------------------------------------------------
    def stop(self) -> None:
        """Stop the UDP threads and close the socket."""
        self._stop.set()
        self._tx_thread.join(timeout=2.0)
        self._rx_thread.join(timeout=2.0)
        try:
            self._sock.close()
        except OSError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mock mixed-interface robot for the example plugin"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--telemetry-port", type=int, default=9870)
    parser.add_argument("--command-port", type=int, default=9871)
    parser.add_argument("--telemetry-rate", type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    robot = MockRobot(
        host=args.host,
        telemetry_port=args.telemetry_port,
        command_port=args.command_port,
        telemetry_rate_hz=args.telemetry_rate,
    )
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
